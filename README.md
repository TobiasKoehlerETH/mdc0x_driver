# MDC0x Rust Embassy Driver

This project is a Rust Embassy implementation of a driver for the MDC0x capacitive-to-digital converter (CDC), packaged here as standalone firmware for STM32G030C8 that configures up to three MDC04 sensors and streams CSV over UART.

The project structure is organized as follows:
- `src/main.rs`: Application entry point and hardware initialization.
- `src/acquisition.rs`: Sensor acquisition loop and timing.
- `src/uart.rs`: UART formatting and transmission.
- `src/mdc04_driver/`: Local minimal MDC04 driver (`mod.rs`, `crc.rs`).

## Scope
- I2C mode on MDC04.
- Scan both I2C buses at startup and enable whichever configured sensors are actually present.
- Configure each sensor with a custom `COS` trim value and a fixed `CFB=0x00` span value.
- High-repeatability single-shot reads of channel 3 and high-repeatability temperature.
- Auto-stream CSV as fast as possible over UART.
- Interleaved sensor conversions to maximize sampling rate.
- No command protocol, no processing pipeline, no calibration controls.

## Hardware Pinout
| Function | MCU Pin | Notes |
| --- | --- | --- |
| UART TX | PA9 | USART1 TX, 230400 baud |
| I2C1 SCL | PB8 | Sensor bus 1 |
| I2C1 SDA | PB9 | Sensor bus 1 |
| I2C2 SCL | PB10 | Sensor bus 2 |
| I2C2 SDA | PB11 | Sensor bus 2 |
| SWDIO | PA13 | defmt RTT debug |
| SWCLK | PA14 | defmt RTT debug |

## Sensor Topology
| Logical sensor | Bus | Address |
| --- | --- | --- |
| Sensor 1 | I2C1 | `0x44` |
| Sensor 2 | I2C2 | `0x44` |
| Sensor 3 | I2C2 | `0x45` |

## MDC04 Registers Used
| Register | Address | Purpose |
| --- | --- | --- |
| `CAP3_LSB` | `0x10` | Channel 3 capacitance low byte |
| `CAP3_MSB` | `0x11` | Channel 3 capacitance high byte |
| `CONFIG` | `0x06` | Repeatability/sample mode/clock stretch |
| `CH_SEL` | `0x1C` | Channel selection |
| `COS` | `0x1D` | Offset register |
| `CFB` | `0x22` | Span and COS range register |

## MDC04 Commands Used
| Command | Value | Purpose |
| --- | --- | --- |
| `CONVERT_T` | `0xCC44` | Trigger/read temperature |
| `CONVERT_C` | `0xCC66` | Trigger capacitance conversion |
| `READ_ONE_BYTE` | `0xD2XX` | Read single register byte |
| `WRITE_ONE_BYTE` | `0x52XX` | Write single register byte |
| `CLEAR_STATUS` | `0x3041` | Clear status flags |

## CSV Format
Outputs one line as fast as the sensors can be read (expected output data rate is ~35 Hz):

`timestamp_ms,c1_counts,t1_mdegc,c2_counts,t2_mdegc,c3_counts,t3_mdegc`

Error sentinel per sensor field is `-1`.

## Read Sequence
At startup, the firmware scans both I2C buses and only initializes the configured sensor slots that acknowledge on their expected bus/address. During acquisition it interleaves conversions only for the detected sensors:
1. Scan `i2c1` and `i2c2`.
2. Match discovered devices against the configured logical sensor slots.
3. Initialize only the sensors that were found.
4. Send `CONVERT_C` to each detected sensor, with `1 ms` (`COMMAND_GAP_US`) between commands.
5. Wait `10.5 ms` (`SINGLE_CHANNEL_CONVERSION_US_HIGH_REPEATABILITY`) for conversion completion.
6. Read each detected sensor result.
7. Emit one CSV line over UART at `230400` baud.
8. Keep missing sensor slots as `-1` in the CSV output.

This parallel conversion strategy reduces total read time from ~40.5ms (sequential waiting) per cycle down to ~15ms per cycle.

## Build
From the repository root:

```bash
cargo check
cargo build --release
```

## Flash/Run
Probe-rs runner is configured in `.cargo/config.toml`:

```bash
cargo run --release
```

## Notes
- Temperature conversion used: `mdegC = raw * 1000 / 256 + 40000`.
- Clock stretching is enabled.
- I2C frequency is 200 kHz.
