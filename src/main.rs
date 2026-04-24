#![no_std]
#![no_main]

mod acquisition;
mod mdc04_driver;
mod uart;

use core::mem;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::gpio::{Level, OutputOpenDrain, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{pac, usart};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_stm32::Peri;
use embedded_hal_async::delay::DelayNs;
use embassy_time::Delay;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C2 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
});

const I2C_FREQ_KHZ: u32 = 200;

#[derive(Clone, Copy)]
enum BusKind {
    I2c1,
    I2c2,
}

pub struct RecoverableI2cBus {
    name: &'static str,
    kind: BusKind,
    config: embassy_stm32::i2c::Config,
    i2c: Option<I2c<'static, Async, embassy_stm32::i2c::Master>>,
}

impl RecoverableI2cBus {
    pub fn new_i2c1(config: embassy_stm32::i2c::Config) -> Self {
        Self {
            name: "i2c1",
            kind: BusKind::I2c1,
            config,
            i2c: Some(Self::build_i2c(BusKind::I2c1, config)),
        }
    }

    pub fn new_i2c2(config: embassy_stm32::i2c::Config) -> Self {
        Self {
            name: "i2c2",
            kind: BusKind::I2c2,
            config,
            i2c: Some(Self::build_i2c(BusKind::I2c2, config)),
        }
    }

    pub fn name(&self) -> &'static str {
        self.name
    }

    pub async fn probe(&mut self, address: u8) -> Result<(), embassy_stm32::i2c::Error> {
        self.i2c().write(address, &[]).await
    }

    pub async fn init_sensor(
        &mut self,
        address: u8,
        cos_hex: u8,
        delay: &mut Delay,
    ) -> Result<(), mdc04_driver::DriverError<embassy_stm32::i2c::Error>> {
        mdc04_driver::init_sensor(self.i2c(), address, cos_hex, delay).await
    }

    pub async fn start_conversion(
        &mut self,
        address: u8,
    ) -> Result<(), mdc04_driver::DriverError<embassy_stm32::i2c::Error>> {
        mdc04_driver::start_conversion(self.i2c(), address).await
    }

    pub async fn read_conversion_result(
        &mut self,
        address: u8,
        delay: &mut Delay,
    ) -> Result<mdc04_driver::SensorSample, mdc04_driver::DriverError<embassy_stm32::i2c::Error>>
    {
        mdc04_driver::read_conversion_result(self.i2c(), address, delay).await
    }

    pub async fn soft_reset(
        &mut self,
        address: u8,
    ) -> Result<(), mdc04_driver::DriverError<embassy_stm32::i2c::Error>> {
        mdc04_driver::soft_reset(self.i2c(), address).await
    }

    pub async fn recover_interface(&mut self, delay: &mut Delay) {
        drop(self.i2c.take());
        match self.kind {
            BusKind::I2c1 => {
                pulse_i2c_lines(
                    unsafe { peripherals::PB8::steal() },
                    unsafe { peripherals::PB9::steal() },
                    delay,
                )
                .await;
            }
            BusKind::I2c2 => {
                pulse_i2c_lines(
                    unsafe { peripherals::PB10::steal() },
                    unsafe { peripherals::PB11::steal() },
                    delay,
                )
                .await;
            }
        }
        self.i2c = Some(Self::build_i2c(self.kind, self.config));
    }

    fn i2c(&mut self) -> &mut I2c<'static, Async, embassy_stm32::i2c::Master> {
        self.i2c.as_mut().unwrap()
    }

    fn build_i2c(
        kind: BusKind,
        config: embassy_stm32::i2c::Config,
    ) -> I2c<'static, Async, embassy_stm32::i2c::Master> {
        match kind {
            BusKind::I2c1 => I2c::new(
                unsafe { peripherals::I2C1::steal() },
                unsafe { peripherals::PB8::steal() },
                unsafe { peripherals::PB9::steal() },
                Irqs,
                unsafe { peripherals::DMA1_CH1::steal() },
                unsafe { peripherals::DMA1_CH2::steal() },
                config,
            ),
            BusKind::I2c2 => I2c::new(
                unsafe { peripherals::I2C2::steal() },
                unsafe { peripherals::PB10::steal() },
                unsafe { peripherals::PB11::steal() },
                Irqs,
                unsafe { peripherals::DMA1_CH3::steal() },
                unsafe { peripherals::DMA1_CH4::steal() },
                config,
            ),
        }
    }
}

async fn pulse_i2c_lines(
    scl: Peri<'static, impl embassy_stm32::gpio::Pin>,
    sda: Peri<'static, impl embassy_stm32::gpio::Pin>,
    delay: &mut Delay,
) {
    let mut scl = OutputOpenDrain::new(scl, Level::High, Speed::Medium);
    let mut sda = OutputOpenDrain::new(sda, Level::High, Speed::Medium);
    sda.set_high();
    delay.delay_us(5).await;
    for _ in 0..9 {
        scl.set_low();
        delay.delay_us(5).await;
        scl.set_high();
        delay.delay_us(5).await;
    }
    sda.set_high();
    delay.delay_us(5).await;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::{
            AHBPrescaler, APBPrescaler, Hsi, HsiSysDiv, Pll, PllMul, PllPreDiv, PllRDiv, PllSource,
            Sysclk,
        };
        config.rcc.hsi = Some(Hsi { sys_div: HsiSysDiv::DIV1 });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL12,
            divr: Some(PllRDiv::DIV4),
            divp: None,
            divq: None,
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
    }

    let p = embassy_stm32::init(config);
    info!("MCU init done");

    // Remap USART1 TX/RX from PA9/PA10 pins to PA11/PA12 pins (PCB routing)
    pac::SYSCFG.cfgr1().modify(|w| {
        w.set_pa11_rmp(true);
        w.set_pa12_rmp(true);
    });

    let (_uart_rx, mut uart_tx) = uart::init_uart(p.USART1, p.PA10, p.PA9);
    uart::write_text_line(&mut uart_tx, "FW boot");
    uart::write_csv_line(&mut uart_tx, "0,-1,-1,-1,-1,-1,-1");
    uart::write_text_line(&mut uart_tx, "stage: uart-ready");
    info!("UART ready");

    let mut i2c_cfg = embassy_stm32::i2c::Config::default();
    i2c_cfg.frequency = Hertz::khz(I2C_FREQ_KHZ);
    i2c_cfg.sda_pullup = false;
    i2c_cfg.scl_pullup = false;
    i2c_cfg.gpio_speed = Speed::High;

    mem::forget(p.I2C1);
    mem::forget(p.PB8);
    mem::forget(p.PB9);
    mem::forget(p.DMA1_CH1);
    mem::forget(p.DMA1_CH2);
    info!("Init I2C1");
    let mut i2c1 = RecoverableI2cBus::new_i2c1(i2c_cfg);
    info!("I2C1 ready");

    mem::forget(p.I2C2);
    mem::forget(p.PB10);
    mem::forget(p.PB11);
    mem::forget(p.DMA1_CH3);
    mem::forget(p.DMA1_CH4);
    info!("Init I2C2");
    let mut i2c2 = RecoverableI2cBus::new_i2c2(i2c_cfg);
    info!("I2C2 ready");

    let mut delay = Delay;

    let active_sensors =
        acquisition::init_sensors(&mut i2c1, &mut i2c2, &mut uart_tx, &mut delay).await;
    acquisition::run_loop(&mut i2c1, &mut i2c2, &mut uart_tx, &mut delay, active_sensors).await;
}
