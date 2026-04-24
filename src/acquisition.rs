use defmt::{error, info, warn};
use cortex_m::peripheral::SCB;
use embassy_stm32::usart::BufferedUartTx;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};

use crate::mdc04_driver::{self, SensorBinding, SensorSample};
use crate::uart;
use crate::RecoverableI2cBus;

pub const SENSOR_INIT_TIMEOUT_MS: u64 = 120;
pub const SENSOR_READ_TIMEOUT_MS: u64 = 60;
pub const I2C_SCAN_TIMEOUT_MS: u64 = 5;
pub const SOFT_RESET_DELAY_MS: u64 = 2;
pub const SENSOR_BINDINGS: [SensorBinding; 3] = [
    SensorBinding::new("i2c1", 0x44, 0x08), // sensor 1: cos_hex trim
    SensorBinding::new("i2c2", 0x44, 0x08), // sensor 2: cos_hex trim
    SensorBinding::new("i2c2", 0x45, 0x08), // sensor 3: cos_hex trim
];

#[derive(Clone, Copy)]
pub struct ActiveSensors(u8);

impl ActiveSensors {
    pub const fn none() -> Self { Self(0) }
    fn set(&mut self, index: usize, present: bool) {
        let bit = 1 << index;
        if present { self.0 |= bit; } else { self.0 &= !bit; }
    }
    fn has(&self, index: usize) -> bool { self.0 & (1 << index) != 0 }
    fn count(&self) -> u32 { self.0.count_ones() }
}

pub async fn init_sensors(
    i2c1: &mut RecoverableI2cBus,
    i2c2: &mut RecoverableI2cBus,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) -> ActiveSensors {
    uart::write_text_line(uart_tx, "stage: sensor-init-begin");
    uart::write_text_line(uart_tx, "stage: i2c-scan-begin");

    let mut active_sensors = scan_expected_sensors(i2c1, i2c2).await;
    info!("I2C scan complete, {} configured sensor(s) present", active_sensors.count());
    uart::write_text_line(uart_tx, "stage: i2c-scan-done");

    for (index, sensor) in SENSOR_BINDINGS.iter().copied().enumerate() {
        if !active_sensors.has(index) {
            warn!(
                "Sensor missing bus={} addr=0x{:02X}, leaving CSV slot invalid",
                sensor.bus, sensor.address
            );
            uart::write_text_line(uart_tx, "stage: sensor-missing");
            continue;
        }

        info!("Init sensor bus={} addr=0x{:02X}", sensor.bus, sensor.address);
        let ok = match sensor.bus {
            "i2c1" => init_detected_sensor(i2c1, sensor, uart_tx, delay).await,
            _ => init_detected_sensor(i2c2, sensor, uart_tx, delay).await,
        };
        if !ok {
            active_sensors.set(index, false);
        }
    }
    uart::write_text_line(uart_tx, "stage: sensor-init-done");
    active_sensors
}

pub async fn run_loop(
    i2c1: &mut RecoverableI2cBus,
    i2c2: &mut RecoverableI2cBus,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
    mut active_sensors: ActiveSensors,
) -> ! {
    info!("Entering acquisition loop");

    loop {
        let timestamp_ms = u64_to_i64_saturating(Instant::now().as_millis());

        if active_sensors.has(0) {
            trigger_sensor_conversion(i2c1, SENSOR_BINDINGS[0], &mut active_sensors, uart_tx, delay)
                .await;
            Timer::after(Duration::from_micros(
                mdc04_driver::COMMAND_GAP_US as u64,
            ))
            .await;
        }
        if active_sensors.has(1) {
            trigger_sensor_conversion(i2c2, SENSOR_BINDINGS[1], &mut active_sensors, uart_tx, delay)
                .await;
            Timer::after(Duration::from_micros(
                mdc04_driver::COMMAND_GAP_US as u64,
            ))
            .await;
        }
        if active_sensors.has(2) {
            trigger_sensor_conversion(i2c2, SENSOR_BINDINGS[2], &mut active_sensors, uart_tx, delay)
                .await;
        }

        Timer::after(Duration::from_micros(
            mdc04_driver::SINGLE_CHANNEL_CONVERSION_US_HIGH_REPEATABILITY as u64,
        ))
        .await;

        let s1 = if active_sensors.has(0) {
            read_sensor_result(i2c1, SENSOR_BINDINGS[0], &mut active_sensors, uart_tx, delay).await
        } else {
            SensorSample::invalid()
        };
        let s2 = if active_sensors.has(1) {
            read_sensor_result(i2c2, SENSOR_BINDINGS[1], &mut active_sensors, uart_tx, delay).await
        } else {
            SensorSample::invalid()
        };
        let s3 = if active_sensors.has(2) {
            read_sensor_result(i2c2, SENSOR_BINDINGS[2], &mut active_sensors, uart_tx, delay).await
        } else {
            SensorSample::invalid()
        };

        let line = uart::build_csv_line(timestamp_ms, s1, s2, s3);
        uart::write_csv_line(uart_tx, line.as_str());
    }
}

async fn scan_expected_sensors(
    i2c1: &mut RecoverableI2cBus,
    i2c2: &mut RecoverableI2cBus,
) -> ActiveSensors {
    let mut active_sensors = ActiveSensors::none();

    scan_bus("i2c1", i2c1, &mut active_sensors).await;
    scan_bus("i2c2", i2c2, &mut active_sensors).await;

    active_sensors
}

async fn scan_bus(
    bus_name: &'static str,
    i2c: &mut RecoverableI2cBus,
    active_sensors: &mut ActiveSensors,
) {
    info!("Scanning {}", bus_name);

    for address in 0x08..0x78 {
        if probe_address(i2c, address).await {
            info!("I2C device detected bus={} addr=0x{:02X}", bus_name, address);

            for (index, binding) in SENSOR_BINDINGS.iter().enumerate() {
                if binding.bus == bus_name && binding.address == address {
                    active_sensors.set(index, true);
                    info!("Matched logical sensor {} on {}", index + 1, bus_name);
                }
            }
        }
    }
}

async fn probe_address(i2c: &mut RecoverableI2cBus, address: u8) -> bool {
    matches!(
        with_timeout(
            Duration::from_millis(I2C_SCAN_TIMEOUT_MS),
            i2c.probe(address)
        )
        .await,
        Ok(Ok(()))
    )
}

async fn init_detected_sensor(
    i2c: &mut RecoverableI2cBus,
    sensor: SensorBinding,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) -> bool {
    match with_timeout(
        Duration::from_millis(SENSOR_INIT_TIMEOUT_MS),
        i2c.init_sensor(sensor.address, sensor.cos_hex, delay),
    )
    .await
    {
        Ok(Ok(())) => {
            info!("Sensor init ok bus={} addr=0x{:02X}", sensor.bus, sensor.address);
            uart::write_text_line(uart_tx, "stage: sensor-init-ok");
            true
        }
        Ok(Err(_)) => {
            warn!("init failed bus={} addr=0x{:02X}", sensor.bus, sensor.address);
            uart::write_text_line(uart_tx, "stage: sensor-init-failed");
            false
        }
        Err(_) => {
            warn!("init timeout bus={} addr=0x{:02X}", sensor.bus, sensor.address);
            uart::write_text_line(uart_tx, "stage: sensor-init-timeout");
            false
        }
    }
}

async fn trigger_sensor_conversion(
    i2c: &mut RecoverableI2cBus,
    sensor: SensorBinding,
    active_sensors: &mut ActiveSensors,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) {
    if start_conversion_once(i2c, sensor.address).await {
        return;
    }

    interface_reset_bus(i2c, active_sensors, uart_tx, delay).await;
    if start_conversion_once(i2c, sensor.address).await {
        return;
    }

    soft_reset_bus(i2c, active_sensors, uart_tx, delay).await;
    if start_conversion_once(i2c, sensor.address).await {
        return;
    }

    reboot_mcu(uart_tx);
}

async fn read_sensor_result(
    i2c: &mut RecoverableI2cBus,
    sensor: SensorBinding,
    active_sensors: &mut ActiveSensors,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) -> SensorSample {
    match with_timeout(
        Duration::from_millis(SENSOR_READ_TIMEOUT_MS),
        i2c.read_conversion_result(sensor.address, delay),
    )
    .await
    {
        Ok(Ok(sample)) => return sample,
        Ok(Err(_)) => {
            error!("read failed addr=0x{:02X}", sensor.address);
        }
        Err(_) => {
            warn!("read timeout addr=0x{:02X}", sensor.address);
        }
    }

    interface_reset_bus(i2c, active_sensors, uart_tx, delay).await;
    match with_timeout(
        Duration::from_millis(SENSOR_READ_TIMEOUT_MS),
        i2c.read_conversion_result(sensor.address, delay),
    )
    .await
    {
        Ok(Ok(sample)) => return sample,
        Ok(Err(_)) => error!("read failed after bus reset addr=0x{:02X}", sensor.address),
        Err(_) => warn!("read timeout after bus reset addr=0x{:02X}", sensor.address),
    }

    soft_reset_bus(i2c, active_sensors, uart_tx, delay).await;
    match with_timeout(
        Duration::from_millis(SENSOR_READ_TIMEOUT_MS),
        i2c.read_conversion_result(sensor.address, delay),
    )
    .await
    {
        Ok(Ok(sample)) => sample,
        Ok(Err(_)) => {
            error!("read failed after soft reset addr=0x{:02X}", sensor.address);
            reboot_mcu(uart_tx);
        }
        Err(_) => {
            warn!("read timeout after soft reset addr=0x{:02X}", sensor.address);
            reboot_mcu(uart_tx);
        }
    }
}

async fn start_conversion_once(i2c: &mut RecoverableI2cBus, address: u8) -> bool {
    match with_timeout(
        Duration::from_millis(SENSOR_READ_TIMEOUT_MS),
        i2c.start_conversion(address),
    )
    .await
    {
        Ok(Ok(())) => true,
        Ok(Err(_)) => {
            error!("start conv failed addr=0x{:02X}", address);
            false
        }
        Err(_) => {
            warn!("start conv timeout addr=0x{:02X}", address);
            false
        }
    }
}

async fn interface_reset_bus(
    i2c: &mut RecoverableI2cBus,
    active_sensors: &mut ActiveSensors,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) {
    info!("bus-recover {}", i2c.name());
    uart::write_text_line(uart_tx, "stage: bus-recover");
    i2c.recover_interface(delay).await;
    rescan_bus(i2c, active_sensors).await;
    init_bus_sensors(i2c, active_sensors, uart_tx, delay).await;
}

async fn soft_reset_bus(
    i2c: &mut RecoverableI2cBus,
    active_sensors: &mut ActiveSensors,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) {
    info!("sensor-soft-reset {}", i2c.name());
    uart::write_text_line(uart_tx, "stage: sensor-soft-reset");
    for (index, sensor) in SENSOR_BINDINGS.iter().copied().enumerate() {
        if sensor.bus != i2c.name() || !active_sensors.has(index) {
            continue;
        }
        let _ = with_timeout(
            Duration::from_millis(SENSOR_INIT_TIMEOUT_MS),
            i2c.soft_reset(sensor.address),
        )
        .await;
    }
    Timer::after(Duration::from_millis(SOFT_RESET_DELAY_MS)).await;
    init_bus_sensors(i2c, active_sensors, uart_tx, delay).await;
}

async fn rescan_bus(i2c: &mut RecoverableI2cBus, active_sensors: &mut ActiveSensors) {
    for (index, sensor) in SENSOR_BINDINGS.iter().enumerate() {
        if sensor.bus == i2c.name() {
            active_sensors.set(index, false);
        }
    }
    scan_bus(i2c.name(), i2c, active_sensors).await;
}

async fn init_bus_sensors(
    i2c: &mut RecoverableI2cBus,
    active_sensors: &mut ActiveSensors,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) {
    for (index, sensor) in SENSOR_BINDINGS.iter().copied().enumerate() {
        if sensor.bus != i2c.name() || !active_sensors.has(index) {
            continue;
        }
        if !init_detected_sensor(i2c, sensor, uart_tx, delay).await {
            active_sensors.set(index, false);
        }
    }
}

fn reboot_mcu(uart_tx: &mut BufferedUartTx<'static>) -> ! {
    error!("mcu-reboot");
    uart::write_text_line(uart_tx, "stage: mcu-reboot");
    SCB::sys_reset()
}

fn u64_to_i64_saturating(value: u64) -> i64 {
    if value > i64::MAX as u64 {
        i64::MAX
    } else {
        value as i64
    }
}
