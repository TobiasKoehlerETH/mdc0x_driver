mod crc;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

use self::crc::crc8_i2c;

const REG_CAP3_LSB: u8 = 0x10;
const REG_CAP3_MSB: u8 = 0x11;
const REG_CONFIG: u8 = 0x06;
const REG_CH_SEL: u8 = 0x1C;
const REG_COS: u8 = 0x1D;
const REG_CFB: u8 = 0x22;

const CMD_CONVERT_T: u16 = 0xCC44;
const CMD_CONVERT_C: u16 = 0xCC66;
const CMD_CLEAR_STATUS: u16 = 0x3041;
const CMD_SOFT_RESET: u16 = 0x30A2;

const CMD_READ_ONE_BYTE_H: u8 = 0xD2;
const CMD_WRITE_ONE_BYTE_H: u8 = 0x52;

const CONFIG_HIGH_REPEATABILITY_SINGLE_SHOT_CLOCK_STRETCH: u8 = 0x22;
const CH_SEL_CH3: u8 = 0x03;
pub const SINGLE_CHANNEL_CONVERSION_US_HIGH_REPEATABILITY: u32 = 10_500;
pub const COMMAND_GAP_US: u32 = 1000;
const STARTUP_WAKE_DELAY_US: u32 = 2_000;
const INIT_RETRIES: u8 = 3;

#[derive(Clone, Copy)]
pub struct SensorBinding {
    pub bus: &'static str,
    pub address: u8,
    /// Value written to REG_COS (0x1D) during sensor init.
    /// Adjust per-sensor to trim the capacitance offset.
    pub cos_hex: u8,
}

impl SensorBinding {
    pub const fn new(bus: &'static str, address: u8, cos_hex: u8) -> Self {
        Self { bus, address, cos_hex }
    }
}

#[derive(Clone, Copy)]
pub struct SensorSample {
    pub cap_counts: i32,
    pub temp_mdegc: i32,
}

impl SensorSample {
    pub const fn invalid() -> Self {
        Self {
            cap_counts: -1,
            temp_mdegc: -1,
        }
    }
}

#[derive(Debug)]
pub enum DriverError<E> {
    I2c(E),
    Crc,
    InvalidFrame,
}

pub async fn init_sensor<I2C, D, E>(
    i2c: &mut I2C,
    address: u8,
    cos_hex: u8,
    delay: &mut D,
) -> Result<(), DriverError<E>>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    let mut attempts = 0u8;
    while attempts < INIT_RETRIES {
        attempts = attempts.saturating_add(1);
        delay.delay_us(STARTUP_WAKE_DELAY_US).await;

        if init_sensor_once(i2c, address, cos_hex, delay).await.is_ok() {
            return Ok(());
        }
    }

    init_sensor_once(i2c, address, cos_hex, delay).await
}

pub async fn start_conversion<I2C, E>(
    i2c: &mut I2C,
    address: u8,
) -> Result<(), DriverError<E>>
where
    I2C: I2c<Error = E>,
{
    write_cmd16(i2c, address, CMD_CONVERT_C).await
}

pub async fn soft_reset<I2C, E>(i2c: &mut I2C, address: u8) -> Result<(), DriverError<E>>
where
    I2C: I2c<Error = E>,
{
    write_cmd16(i2c, address, CMD_SOFT_RESET).await
}

pub async fn read_conversion_result<I2C, D, E>(
    i2c: &mut I2C,
    address: u8,
    delay: &mut D,
) -> Result<SensorSample, DriverError<E>>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    let cap_lsb = read_one_byte(i2c, address, REG_CAP3_LSB).await?;
    delay.delay_us(COMMAND_GAP_US).await;
    let cap_msb = read_one_byte(i2c, address, REG_CAP3_MSB).await?;
    delay.delay_us(COMMAND_GAP_US).await;
    let cap_counts = i32::from(u16::from_le_bytes([cap_lsb, cap_msb]));

    let temp_raw = read_temperature_stretch_raw(i2c, address).await?;

    Ok(SensorSample {
        cap_counts,
        temp_mdegc: celsius_raw_to_mdegc(temp_raw),
    })
}

async fn read_temperature_stretch_raw<I2C, E>(
    i2c: &mut I2C,
    address: u8,
) -> Result<i16, DriverError<E>>
where
    I2C: I2c<Error = E>,
{
    let mut frame = [0u8; 3];
    i2c.write_read(address, &CMD_CONVERT_T.to_be_bytes(), &mut frame)
        .await
        .map_err(DriverError::I2c)?;

    if crc8_i2c(&frame[0..2]) != frame[2] {
        return Err(DriverError::Crc);
    }

    Ok(i16::from_be_bytes([frame[0], frame[1]]))
}

async fn write_cmd16<I2C, E>(i2c: &mut I2C, address: u8, cmd: u16) -> Result<(), DriverError<E>>
where
    I2C: I2c<Error = E>,
{
    i2c.write(address, &cmd.to_be_bytes())
        .await
        .map_err(DriverError::I2c)
}

async fn read_one_byte<I2C, E>(
    i2c: &mut I2C,
    address: u8,
    reg: u8,
) -> Result<u8, DriverError<E>>
where
    I2C: I2c<Error = E>,
{
    let mut frame = [0u8; 3];
    i2c.write_read(address, &[CMD_READ_ONE_BYTE_H, reg], &mut frame)
        .await
        .map_err(DriverError::I2c)?;

    if frame[1] != 0xFF {
        return Err(DriverError::InvalidFrame);
    }

    if crc8_i2c(&frame[0..2]) != frame[2] {
        return Err(DriverError::Crc);
    }

    Ok(frame[0])
}

async fn write_one_byte<I2C, E>(
    i2c: &mut I2C,
    address: u8,
    reg: u8,
    value: u8,
) -> Result<(), DriverError<E>>
where
    I2C: I2c<Error = E>,
{
    let mut frame = [0u8; 5];
    frame[0] = CMD_WRITE_ONE_BYTE_H;
    frame[1] = reg;
    frame[2] = value;
    frame[3] = 0xFF;
    frame[4] = crc8_i2c(&frame[2..4]);

    i2c.write(address, &frame).await.map_err(DriverError::I2c)
}

fn celsius_raw_to_mdegc(raw: i16) -> i32 {
    (i32::from(raw) * 1000) / 256 + 40_000
}

async fn init_sensor_once<I2C, D, E>(
    i2c: &mut I2C,
    address: u8,
    cos_hex: u8,
    delay: &mut D,
) -> Result<(), DriverError<E>>
where
    I2C: I2c<Error = E>,
    D: DelayNs,
{
    write_one_byte(i2c, address, REG_CONFIG, CONFIG_HIGH_REPEATABILITY_SINGLE_SHOT_CLOCK_STRETCH)
        .await?;
    delay.delay_us(COMMAND_GAP_US).await;

    write_cmd16(i2c, address, CMD_CLEAR_STATUS).await?;
    delay.delay_us(COMMAND_GAP_US).await;

    write_one_byte(i2c, address, REG_CH_SEL, CH_SEL_CH3).await?;
    delay.delay_us(COMMAND_GAP_US).await;

    write_one_byte(i2c, address, REG_COS, cos_hex).await?;
    delay.delay_us(COMMAND_GAP_US).await;

    write_one_byte(i2c, address, REG_CFB, 0x00).await?;
    delay.delay_us(COMMAND_GAP_US).await;

    Ok(())
}
