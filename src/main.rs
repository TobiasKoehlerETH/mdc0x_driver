#![no_std]
#![no_main]

mod acquisition;
mod mdc04_driver;
mod uart;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::gpio::Speed;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::{pac, usart};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Delay;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C2 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
});

const I2C_FREQ_KHZ: u32 = 400;

static I2C1_BUS: StaticCell<I2c<'static, Async, embassy_stm32::i2c::Master>> = StaticCell::new();
static I2C2_BUS: StaticCell<I2c<'static, Async, embassy_stm32::i2c::Master>> = StaticCell::new();

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

    info!("Init I2C1");
    let i2c1 = I2C1_BUS.init(I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        i2c_cfg,
    ));
    info!("I2C1 ready");

    info!("Init I2C2");
    let i2c2 = I2C2_BUS.init(I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH4,
        i2c_cfg,
    ));
    info!("I2C2 ready");

    let mut delay = Delay;

    acquisition::init_sensors(i2c1, i2c2, &mut uart_tx, &mut delay).await;
    acquisition::run_loop(i2c1, i2c2, &mut uart_tx, &mut delay).await;
}
