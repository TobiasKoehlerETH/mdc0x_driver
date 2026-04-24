use core::fmt::Write;

use embassy_stm32::Peri;
use embassy_stm32::peripherals;
use embassy_stm32::usart::{self, BufferedUart, BufferedUartRx, BufferedUartTx};
use embedded_io::Write as _;
use heapless::String;
use static_cell::ConstStaticCell;

use crate::Irqs;
use crate::mdc04_driver::SensorSample;

pub const UART_BAUDRATE: u32 = 230_400;

static UART_TX_BUF: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0u8; 64]);
static UART_RX_BUF: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0u8; 64]);

pub fn init_uart(
    usart: Peri<'static, peripherals::USART1>,
    pa10: Peri<'static, peripherals::PA10>,
    pa9: Peri<'static, peripherals::PA9>,
) -> (BufferedUartRx<'static>, BufferedUartTx<'static>) {
    let tx_buf = UART_TX_BUF.take();
    let rx_buf = UART_RX_BUF.take();

    let mut cfg = usart::Config::default();
    cfg.baudrate = UART_BAUDRATE;

    let buffered = BufferedUart::new(usart, pa10, pa9, tx_buf, rx_buf, Irqs, cfg).unwrap();
    let (tx, rx) = buffered.split();
    (rx, tx)
}

pub fn write_csv_line(uart_tx: &mut BufferedUartTx<'static>, line: &str) {
    let _ = uart_tx.write_all(line.as_bytes());
    let _ = uart_tx.write_all(b"\n");
    let _ = uart_tx.flush();
}

pub fn write_text_line(uart_tx: &mut BufferedUartTx<'static>, line: &str) {
    let _ = uart_tx.write_all(b"# ");
    let _ = uart_tx.write_all(line.as_bytes());
    let _ = uart_tx.write_all(b"\n");
    let _ = uart_tx.flush();
}

pub fn build_csv_line(
    timestamp_ms: i64,
    s1: SensorSample,
    s2: SensorSample,
    s3: SensorSample,
) -> String<96> {
    let mut line = String::<96>::new();
    let _ = write!(
        line,
        "{},{},{},{},{},{},{}",
        timestamp_ms,
        s1.cap_counts,
        s1.temp_mdegc,
        s2.cap_counts,
        s2.temp_mdegc,
        s3.cap_counts,
        s3.temp_mdegc
    );
    line
}
