use defmt::{error, info, warn};
use embassy_stm32::usart::BufferedUartTx;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};

use crate::mdc04_driver::{self, SensorBinding, SensorSample};
use crate::uart;

pub const SENSOR_INIT_TIMEOUT_MS: u64 = 120;
pub const SENSOR_READ_TIMEOUT_MS: u64 = 60;
pub const SENSOR_BINDINGS: [SensorBinding; 3] = [
    SensorBinding::new("i2c1", 0x44, 0x08), // sensor 1: cos_hex trim
    SensorBinding::new("i2c2", 0x44, 0x08), // sensor 2: cos_hex trim
    SensorBinding::new("i2c2", 0x45, 0x08), // sensor 3: cos_hex trim
];

pub async fn init_sensors<I2C1, I2C2, E1, E2>(
    i2c1: &mut I2C1,
    i2c2: &mut I2C2,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) where
    I2C1: embedded_hal_async::i2c::I2c<Error = E1>,
    I2C2: embedded_hal_async::i2c::I2c<Error = E2>,
{
    uart::write_text_line(uart_tx, "stage: sensor-init-begin");
    for sensor in SENSOR_BINDINGS {
        info!("Init sensor bus={} addr=0x{:02X}", sensor.bus, sensor.address);
        match sensor.bus {
            "i2c1" => {
                let init_result = with_timeout(
                    Duration::from_millis(SENSOR_INIT_TIMEOUT_MS),
                    mdc04_driver::init_sensor(i2c1, sensor.address, sensor.cos_hex, delay),
                )
                .await;
                match init_result {
                    Ok(Ok(())) => {
                        info!("Sensor init ok bus={} addr=0x{:02X}", sensor.bus, sensor.address);
                        uart::write_text_line(uart_tx, "stage: sensor-init-ok");
                    }
                    Ok(Err(_)) => {
                        warn!("init failed bus={} addr=0x{:02X}", sensor.bus, sensor.address);
                        uart::write_text_line(uart_tx, "stage: sensor-init-failed");
                    }
                    Err(_) => {
                        warn!(
                            "init timeout bus={} addr=0x{:02X}",
                            sensor.bus, sensor.address
                        );
                        uart::write_text_line(uart_tx, "stage: sensor-init-timeout");
                    }
                }
            }
            _ => {
                let init_result = with_timeout(
                    Duration::from_millis(SENSOR_INIT_TIMEOUT_MS),
                    mdc04_driver::init_sensor(i2c2, sensor.address, sensor.cos_hex, delay),
                )
                .await;
                match init_result {
                    Ok(Ok(())) => {
                        info!("Sensor init ok bus={} addr=0x{:02X}", sensor.bus, sensor.address);
                        uart::write_text_line(uart_tx, "stage: sensor-init-ok");
                    }
                    Ok(Err(_)) => {
                        warn!("init failed bus={} addr=0x{:02X}", sensor.bus, sensor.address);
                        uart::write_text_line(uart_tx, "stage: sensor-init-failed");
                    }
                    Err(_) => {
                        warn!(
                            "init timeout bus={} addr=0x{:02X}",
                            sensor.bus, sensor.address
                        );
                        uart::write_text_line(uart_tx, "stage: sensor-init-timeout");
                    }
                }
            }
        }
    }
    uart::write_text_line(uart_tx, "stage: sensor-init-done");
}

pub async fn run_loop<I2C1, I2C2, E1, E2>(
    i2c1: &mut I2C1,
    i2c2: &mut I2C2,
    uart_tx: &mut BufferedUartTx<'static>,
    delay: &mut Delay,
) -> !
where
    I2C1: embedded_hal_async::i2c::I2c<Error = E1>,
    I2C2: embedded_hal_async::i2c::I2c<Error = E2>,
{
    info!("Entering acquisition loop");

    loop {
        let timestamp_ms = u64_to_i64_saturating(Instant::now().as_millis());

        trigger_sensor_conversion(i2c1, SENSOR_BINDINGS[0].address).await;
        Timer::after(Duration::from_micros(
            mdc04_driver::COMMAND_GAP_US as u64,
        ))
        .await;
        trigger_sensor_conversion(i2c2, SENSOR_BINDINGS[1].address).await;
        Timer::after(Duration::from_micros(
            mdc04_driver::COMMAND_GAP_US as u64,
        ))
        .await;
        trigger_sensor_conversion(i2c2, SENSOR_BINDINGS[2].address).await;

        Timer::after(Duration::from_micros(
            mdc04_driver::SINGLE_CHANNEL_CONVERSION_US_HIGH_REPEATABILITY as u64,
        ))
        .await;

        let s1 = read_sensor_result(i2c1, SENSOR_BINDINGS[0].address, delay).await;
        let s2 = read_sensor_result(i2c2, SENSOR_BINDINGS[1].address, delay).await;
        let s3 = read_sensor_result(i2c2, SENSOR_BINDINGS[2].address, delay).await;

        let line = uart::build_csv_line(timestamp_ms, s1, s2, s3);
        uart::write_csv_line(uart_tx, line.as_str());
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
