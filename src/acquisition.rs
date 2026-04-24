use defmt::{error, info, warn};
use embassy_stm32::usart::BufferedUartTx;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};

use crate::mdc04_driver::{self, SensorBinding, SensorSample};
use crate::uart;

pub const SENSOR_INIT_TIMEOUT_MS: u64 = 120;
pub const SENSOR_READ_TIMEOUT_MS: u64 = 60;
pub const I2C_SCAN_TIMEOUT_MS: u64 = 5;
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

pub async fn init_sensors<I2C1, I2C2, E1, E2>(
    i2c1: &mut I2C1,
    i2c2: &mut I2C2,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) -> ActiveSensors
where
    I2C1: embedded_hal_async::i2c::I2c<Error = E1>,
    I2C2: embedded_hal_async::i2c::I2c<Error = E2>,
{
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

pub async fn run_loop<I2C1, I2C2, E1, E2>(
    i2c1: &mut I2C1,
    i2c2: &mut I2C2,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
    active_sensors: ActiveSensors,
) -> !
where
    I2C1: embedded_hal_async::i2c::I2c<Error = E1>,
    I2C2: embedded_hal_async::i2c::I2c<Error = E2>,
{
    info!("Entering acquisition loop");

    loop {
        let timestamp_ms = u64_to_i64_saturating(Instant::now().as_millis());

        if active_sensors.has(0) {
            trigger_sensor_conversion(i2c1, SENSOR_BINDINGS[0].address).await;
            Timer::after(Duration::from_micros(
                mdc04_driver::COMMAND_GAP_US as u64,
            ))
            .await;
        }
        if active_sensors.has(1) {
            trigger_sensor_conversion(i2c2, SENSOR_BINDINGS[1].address).await;
            Timer::after(Duration::from_micros(
                mdc04_driver::COMMAND_GAP_US as u64,
            ))
            .await;
        }
        if active_sensors.has(2) {
            trigger_sensor_conversion(i2c2, SENSOR_BINDINGS[2].address).await;
        }

        Timer::after(Duration::from_micros(
            mdc04_driver::SINGLE_CHANNEL_CONVERSION_US_HIGH_REPEATABILITY as u64,
        ))
        .await;

        let s1 = if active_sensors.has(0) {
            read_sensor_result(i2c1, SENSOR_BINDINGS[0].address, delay).await
        } else {
            SensorSample::invalid()
        };
        let s2 = if active_sensors.has(1) {
            read_sensor_result(i2c2, SENSOR_BINDINGS[1].address, delay).await
        } else {
            SensorSample::invalid()
        };
        let s3 = if active_sensors.has(2) {
            read_sensor_result(i2c2, SENSOR_BINDINGS[2].address, delay).await
        } else {
            SensorSample::invalid()
        };

        let line = uart::build_csv_line(timestamp_ms, s1, s2, s3);
        uart::write_csv_line(uart_tx, line.as_str());
    }
}

async fn scan_expected_sensors<I2C1, I2C2, E1, E2>(
    i2c1: &mut I2C1,
    i2c2: &mut I2C2,
) -> ActiveSensors
where
    I2C1: embedded_hal_async::i2c::I2c<Error = E1>,
    I2C2: embedded_hal_async::i2c::I2c<Error = E2>,
{
    let mut active_sensors = ActiveSensors::none();

    scan_bus("i2c1", i2c1, &mut active_sensors).await;
    scan_bus("i2c2", i2c2, &mut active_sensors).await;

    active_sensors
}

async fn scan_bus<I2C, E>(bus_name: &'static str, i2c: &mut I2C, active_sensors: &mut ActiveSensors)
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
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

async fn probe_address<I2C, E>(i2c: &mut I2C, address: u8) -> bool
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    matches!(
        with_timeout(
            Duration::from_millis(I2C_SCAN_TIMEOUT_MS),
            i2c.write(address, &[])
        )
        .await,
        Ok(Ok(()))
    )
}

async fn init_detected_sensor<I2C, E>(
    i2c: &mut I2C,
    sensor: SensorBinding,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) -> bool
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    match with_timeout(
        Duration::from_millis(SENSOR_INIT_TIMEOUT_MS),
        mdc04_driver::init_sensor(i2c, sensor.address, sensor.cos_hex, delay),
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

async fn trigger_sensor_conversion<I2C, E>(i2c: &mut I2C, address: u8)
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    match with_timeout(
        Duration::from_millis(SENSOR_READ_TIMEOUT_MS),
        mdc04_driver::start_conversion(i2c, address),
    )
    .await
    {
        Ok(Ok(())) => {}
        Ok(Err(_)) => error!("start conv failed addr=0x{:02X}", address),
        Err(_) => warn!("start conv timeout addr=0x{:02X}", address),
    }
}

async fn read_sensor_result<I2C, E>(
    i2c: &mut I2C,
    address: u8,
    delay: &mut Delay,
) -> SensorSample
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    match with_timeout(
        Duration::from_millis(SENSOR_READ_TIMEOUT_MS),
        mdc04_driver::read_conversion_result(i2c, address, delay),
    )
    .await
    {
        Ok(Ok(sample)) => sample,
        Ok(Err(_)) => {
            error!("read failed addr=0x{:02X}", address);
            SensorSample::invalid()
        }
        Err(_) => {
            warn!("read timeout addr=0x{:02X}", address);
            SensorSample::invalid()
        }
    }
}

fn u64_to_i64_saturating(value: u64) -> i64 {
    if value > i64::MAX as u64 {
        i64::MAX
    } else {
        value as i64
    }
}
