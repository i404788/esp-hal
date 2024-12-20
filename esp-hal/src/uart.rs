//! # Universal Asynchronous Receiver/Transmitter (UART)
//!
//! ## Overview
//!
//! The UART is a hardware peripheral which handles communication using serial
//! communication interfaces, such as RS232 and RS485. This peripheral provides
//! a cheap and ubiquitous method for full- and half-duplex communication
//! between devices.
//!
//! Depending on your device, two or more UART controllers are available for
//! use, all of which can be configured and used in the same way. All UART
//! controllers are compatible with UART-enabled devices from various
//! manufacturers, and can also support Infrared Data Association (IrDA)
//! protocols.
//!
//! ## Configuration
//!
//! Each UART controller is individually configurable, and the usual setting
//! such as baud rate, data bits, parity, and stop bits can easily be
//! configured. Additionally, the receive (RX) and transmit (TX) pins need to
//! be specified.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::Uart;
//! use esp_hal::gpio::Io;
//!
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! let mut uart1 = Uart::new(
//!     peripherals.UART1,
//!     io.pins.gpio1,
//!     io.pins.gpio2,
//! ).unwrap();
//! # }
//! ```
//! 
//! The UART controller can be configured to invert the polarity of the pins.
//! This is achieved by inverting the desired pins, and then constructing the
//! UART instance using the inverted pins.
//!
//! ## Usage
//!
//! The UART driver implements a number of third-party traits, with the
//! intention of making the HAL inter-compatible with various device drivers
//! from the community. This includes, but is not limited to, the [embedded-hal]
//! and [embedded-io] blocking traits, and the [embedded-hal-async] and
//! [embedded-io-async] asynchronous traits.
//!
//! In addition to the interfaces provided by these traits, native APIs are also
//! available. See the examples below for more information on how to interact
//! with this driver.
//!
//! ## Examples
//! ### Sending and Receiving Data
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{config::Config, Uart};
//! # use esp_hal::gpio::Io;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let mut uart1 = Uart::new_with_config(
//! #     peripherals.UART1,
//! #     Config::default(),
//! #     io.pins.gpio1,
//! #     io.pins.gpio2,
//! # ).unwrap();
//! // Write bytes out over the UART:
//! uart1.write_bytes(b"Hello, world!").expect("write error!");
//! # }
//! ```
//! 
//! ### Splitting the UART into RX and TX Components
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{config::Config, Uart};
//! # use esp_hal::gpio::Io;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let mut uart1 = Uart::new_with_config(
//! #     peripherals.UART1,
//! #     Config::default(),
//! #     io.pins.gpio1,
//! #     io.pins.gpio2,
//! # ).unwrap();
//! // The UART can be split into separate Transmit and Receive components:
//! let (mut rx, mut tx) = uart1.split();
//!
//! // Each component can be used individually to interact with the UART:
//! tx.write_bytes(&[42u8]).expect("write error!");
//! let byte = rx.read_byte().expect("read error!");
//! # }
//! ```
//! 
//! ### Inverting RX and TX Pins
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::Uart;
//! use esp_hal::gpio::Io;
//!
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! let rx = io.pins.gpio2.peripheral_input().inverted();
//! let tx = io.pins.gpio1.into_peripheral_output().inverted();
//! let mut uart1 = Uart::new(
//!     peripherals.UART1,
//!     rx,
//!     tx,
//! ).unwrap();
//! # }
//! ```
//! 
//! ### Constructing RX and TX Components
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{UartTx, UartRx};
//! use esp_hal::gpio::Io;
//!
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! let tx = UartTx::new(peripherals.UART0, io.pins.gpio1).unwrap();
//! let rx = UartRx::new(peripherals.UART1, io.pins.gpio2).unwrap();
//! # }
//! ```
//! 
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [embedded-io-async]: https://docs.rs/embedded-io-async/latest/embedded_io_async/

use core::marker::PhantomData;

use enumset::{EnumSet, EnumSetType};

use self::config::Config;
use crate::{
    clock::Clocks,
    dma::PeripheralMarker,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        InputSignal,
        OutputSignal,
        Pull,
    },
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{uart0::RegisterBlock, Interrupt},
    private::Internal,
    system::PeripheralClockControl,
    Blocking,
    InterruptConfigurable,
    Mode,
};

const CONSOLE_UART_NUM: usize = 0;
const UART_FIFO_SIZE: u16 = 128;

#[cfg(not(any(esp32, esp32s2)))]
use crate::soc::constants::RC_FAST_CLK;
#[cfg(any(esp32, esp32s2))]
use crate::soc::constants::REF_TICK;

/// UART Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// An invalid configuration argument was provided.
    ///
    /// This error occurs when an incorrect or invalid argument is passed during
    /// the configuration of the UART peripheral.
    InvalidArgument,

    /// The RX FIFO overflowed.
    RxFifoOvf,

    /// A glitch was detected on the RX line.
    ///
    /// This error occurs when an unexpected or erroneous signal (glitch) is
    /// detected on the UART RX line, which could lead to incorrect data
    /// reception.
    RxGlitchDetected,

    /// A framing error was detected on the RX line.
    ///
    /// This error occurs when the received data does not conform to the
    /// expected UART frame format.
    RxFrameError,

    /// A parity error was detected on the RX line.
    ///
    /// This error occurs when the parity bit in the received data does not
    /// match the expected parity configuration.
    /// with the `async` feature.
    RxParityError,
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        embedded_hal_nb::serial::ErrorKind::Other
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

// (outside of `config` module in order not to "use" it an extra time)
/// UART clock source
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// APB_CLK clock source (default for UART on all the chips except of
    /// esp32c6 and esp32h2)
    Apb,
    #[cfg(not(any(esp32, esp32s2)))]
    /// RC_FAST_CLK clock source (17.5 MHz)
    RcFast,
    #[cfg(not(any(esp32, esp32s2)))]
    /// XTAL_CLK clock source (default for UART on esp32c6 and esp32h2 and
    /// LP_UART)
    Xtal,
    #[cfg(any(esp32, esp32s2))]
    /// REF_TICK clock source (derived from XTAL or RC_FAST, 1MHz)
    RefTick,
}

/// UART Configuration
pub mod config {

    // see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L61>
    const UART_FULL_THRESH_DEFAULT: u16 = 120;
    // see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L63>
    const UART_TOUT_THRESH_DEFAULT: u8 = 10;

    /// Number of data bits
    ///
    /// This enum represents the various configurations for the number of data
    /// bits used in UART communication. The number of data bits defines the
    /// length of each transmitted or received data frame.
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DataBits {
        /// 5 data bits per frame.
        DataBits5 = 0,
        /// 6 data bits per frame.
        DataBits6 = 1,
        /// 7 data bits per frame.
        DataBits7 = 2,
        /// 8 data bits per frame (most common).
        DataBits8 = 3,
    }

    /// Parity check
    ///
    /// Parity is a form of error detection in UART communication, used to
    /// ensure that the data has not been corrupted during transmission. The
    /// parity bit is added to the data bits to make the number of 1-bits
    /// either even or odd.
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Parity {
        /// No parity bit is used (most common).
        ParityNone,
        /// Even parity: the parity bit is set to make the total number of
        /// 1-bits even.
        ParityEven,
        /// Odd parity: the parity bit is set to make the total number of 1-bits
        /// odd.
        ParityOdd,
    }

    /// Number of stop bits
    ///
    /// The stop bit(s) signal the end of a data packet in UART communication.
    /// This enum defines the possible configurations for the number of stop
    /// bits.
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum StopBits {
        /// 1 stop bit.
        STOP1   = 1,
        /// 1.5 stop bits.
        STOP1P5 = 2,
        /// 2 stop bits.
        STOP2   = 3,
    }

    /// UART Configuration
    #[derive(Debug, Copy, Clone)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Config {
        /// The baud rate (speed) of the UART communication in bits per second
        /// (bps).
        pub baudrate: u32,
        /// Number of data bits in each frame (5, 6, 7, or 8 bits).
        pub data_bits: DataBits,
        /// Parity setting (None, Even, or Odd).
        pub parity: Parity,
        /// Number of stop bits in each frame (1, 1.5, or 2 bits).
        pub stop_bits: StopBits,
        /// Clock source used by the UART peripheral.
        pub clock_source: super::ClockSource,
        /// Threshold level at which the RX FIFO is considered full.
        pub rx_fifo_full_threshold: u16,
        /// Optional timeout value for RX operations.
        pub rx_timeout: Option<u8>,
    }

    impl Config {
        /// Sets the baud rate for the UART configuration.
        pub fn baudrate(mut self, baudrate: u32) -> Self {
            self.baudrate = baudrate;
            self
        }

        /// Configures the UART to use no parity check.
        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        /// Configures the UART to use even parity check.
        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        /// Configures the UART to use odd parity check.
        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        /// Sets the number of data bits for the UART configuration.
        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        /// Sets the number of stop bits for the UART configuration.
        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }

        /// Sets the clock source for the UART configuration.
        pub fn clock_source(mut self, source: super::ClockSource) -> Self {
            self.clock_source = source;
            self
        }

        /// Calculates the total symbol length in bits based on the configured
        /// data bits, parity, and stop bits.
        pub fn symbol_length(&self) -> u8 {
            let mut length: u8 = 1; // start bit
            length += match self.data_bits {
                DataBits::DataBits5 => 5,
                DataBits::DataBits6 => 6,
                DataBits::DataBits7 => 7,
                DataBits::DataBits8 => 8,
            };
            length += match self.parity {
                Parity::ParityNone => 0,
                _ => 1,
            };
            length += match self.stop_bits {
                StopBits::STOP1 => 1,
                _ => 2, // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
            };
            length
        }

        /// Sets the RX FIFO full threshold for the UART configuration.
        pub fn rx_fifo_full_threshold(mut self, threshold: u16) -> Self {
            self.rx_fifo_full_threshold = threshold;
            self
        }

        /// Sets the RX timeout for the UART configuration.
        pub fn rx_timeout(mut self, timeout: Option<u8>) -> Self {
            self.rx_timeout = timeout;
            self
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115_200,
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
                #[cfg(any(esp32c6, esp32h2, lp_uart))]
                clock_source: super::ClockSource::Xtal,
                #[cfg(not(any(esp32c6, esp32h2, lp_uart)))]
                clock_source: super::ClockSource::Apb,
                rx_fifo_full_threshold: UART_FULL_THRESH_DEFAULT,
                rx_timeout: Some(UART_TOUT_THRESH_DEFAULT),
            }
        }
    }

    /// Configuration for the AT-CMD detection functionality
    pub struct AtCmdConfig {
        /// Optional idle time before the AT command detection begins, in clock
        /// cycles.
        pub pre_idle_count: Option<u16>,
        /// Optional idle time after the AT command detection ends, in clock
        /// cycles.
        pub post_idle_count: Option<u16>,
        /// Optional timeout between characters in the AT command, in clock
        /// cycles.
        pub gap_timeout: Option<u16>,
        /// The character that triggers the AT command detection.
        pub cmd_char: u8,
        /// Optional number of characters to detect as part of the AT command.
        pub char_num: Option<u8>,
    }

    impl AtCmdConfig {
        /// Creates a new `AtCmdConfig` with the specified configuration.
        ///
        /// This function sets up the AT command detection parameters, including
        /// pre- and post-idle times, a gap timeout, the triggering command
        /// character, and the number of characters to detect.
        pub fn new(
            pre_idle_count: Option<u16>,
            post_idle_count: Option<u16>,
            gap_timeout: Option<u16>,
            cmd_char: u8,
            char_num: Option<u8>,
        ) -> AtCmdConfig {
            Self {
                pre_idle_count,
                post_idle_count,
                gap_timeout,
                cmd_char,
                char_num,
            }
        }
    }
}

struct UartBuilder<'d, M, T = AnyUart> {
    uart: PeripheralRef<'d, T>,
    phantom: PhantomData<M>,
}

impl<'d, M, T> UartBuilder<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    fn new(uart: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(uart);
        Self {
            uart,
            phantom: PhantomData,
        }
    }

    fn with_rx<RX: PeripheralInput>(self, rx: impl Peripheral<P = RX> + 'd) -> Self {
        crate::into_mapped_ref!(rx);
        rx.init_input(Pull::Up, Internal);
        rx.connect_input_to_peripheral(self.uart.rx_signal(), Internal);

        self
    }

    fn with_tx<TX: PeripheralOutput>(self, tx: impl Peripheral<P = TX> + 'd) -> Self {
        crate::into_mapped_ref!(tx);
        // Make sure we don't cause an unexpected low pulse on the pin.
        tx.set_output_high(true, Internal);
        tx.set_to_push_pull_output(Internal);
        tx.connect_peripheral_to_output(self.uart.tx_signal(), Internal);

        self
    }

    fn init(self, config: Config) -> Result<Uart<'d, M, T>, Error> {
        let mut serial = Uart {
            rx: UartRx {
                uart: unsafe { self.uart.clone_unchecked() },
                phantom: PhantomData,
                at_cmd_config: None,
                rx_timeout_config: None,
                #[cfg(not(esp32))]
                symbol_len: config.symbol_length(),
            },
            tx: UartTx {
                uart: self.uart,
                phantom: PhantomData,
            },
        };
        serial.init(config)?;

        Ok(serial)
    }
}

/// UART (Full-duplex)
pub struct Uart<'d, M, T = AnyUart> {
    rx: UartRx<'d, M, T>,
    tx: UartTx<'d, M, T>,
}

/// UART (Transmit)
pub struct UartTx<'d, M, T = AnyUart> {
    uart: PeripheralRef<'d, T>,
    phantom: PhantomData<M>,
}

/// UART (Receive)
pub struct UartRx<'d, M, T = AnyUart> {
    uart: PeripheralRef<'d, T>,
    phantom: PhantomData<M>,
    at_cmd_config: Option<config::AtCmdConfig>,
    rx_timeout_config: Option<u8>,
    #[cfg(not(esp32))]
    symbol_len: u8,
}

impl<'d, M, T> UartTx<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    /// Configure RTS pin
    pub fn with_rts<RTS: PeripheralOutput>(self, rts: impl Peripheral<P = RTS> + 'd) -> Self {
        crate::into_mapped_ref!(rts);
        rts.set_to_push_pull_output(Internal);
        rts.connect_peripheral_to_output(self.uart.rts_signal(), Internal);

        self
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        let count = data.len();

        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))?;

        Ok(count)
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        let register_block = self.uart.register_block();
        if get_tx_fifo_count(register_block) < UART_FIFO_SIZE {
            register_block
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Flush the transmit buffer of the UART
    pub fn flush_tx(&mut self) -> nb::Result<(), Error> {
        if self.uart.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'d> UartTx<'d, Blocking> {
    /// Create a new UART TX instance in [`Blocking`] mode.
    pub fn new<TX: PeripheralOutput>(
        uart: impl Peripheral<P = impl Instance> + 'd,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_typed(uart.map_into(), tx)
    }

    /// Create a new UART TX instance with configuration options in
    /// [`Blocking`] mode.
    pub fn new_with_config<TX: PeripheralOutput>(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_with_config_typed(uart.map_into(), config, tx)
    }
}

impl<'d, T> UartTx<'d, Blocking, T>
where
    T: Instance,
{
    /// Create a new UART TX instance in [`Blocking`] mode.
    pub fn new_typed<TX: PeripheralOutput>(
        uart: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_with_config_typed(uart, Config::default(), tx)
    }

    /// Create a new UART TX instance with configuration options in
    /// [`Blocking`] mode.
    pub fn new_with_config_typed<TX: PeripheralOutput>(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        let (_, uart_tx) = UartBuilder::<'d, Blocking, T>::new(uart)
            .with_tx(tx)
            .init(config)?
            .split();

        Ok(uart_tx)
    }
}

#[allow(clippy::useless_conversion)]
/// Returns the number of bytes currently in the TX FIFO for this UART
/// instance.
fn get_tx_fifo_count(register_block: &RegisterBlock) -> u16 {
    register_block.status().read().txfifo_cnt().bits().into()
}

#[allow(clippy::useless_conversion)]
fn get_rx_fifo_count(register_block: &RegisterBlock) -> u16 {
    let fifo_cnt: u16 = register_block.status().read().rxfifo_cnt().bits().into();
    // Calculate the real count based on the FIFO read and write offset address:
    // https://www.espressif.com/sites/default/files/documentation/esp32_errata_en.pdf
    // section 3.17
    #[cfg(esp32)]
    {
        let status = register_block.mem_rx_status().read();
        let rd_addr = status.mem_rx_rd_addr().bits();
        let wr_addr = status.mem_rx_wr_addr().bits();

        if wr_addr > rd_addr {
            wr_addr - rd_addr
        } else if wr_addr < rd_addr {
            (wr_addr + UART_FIFO_SIZE) - rd_addr
        } else if fifo_cnt > 0 {
            UART_FIFO_SIZE
        } else {
            0
        }
    }

    #[cfg(not(esp32))]
    fifo_cnt
}

fn read_bytes(register_block: &RegisterBlock, buf: &mut [u8]) -> Result<(), Error> {
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
            let fifo = unsafe {
                &*((register_block.fifo().as_ptr() as *mut u8).add(0x20C00000)
                    as *mut crate::peripherals::uart0::FIFO)
            };
        } else {
            let fifo = register_block.fifo();
        }
    }

    for byte in buf.iter_mut() {
        while get_rx_fifo_count(register_block) == 0 {
            // Block until we received at least one byte
        }
        *byte = fifo.read().rxfifo_rd_byte().bits();
    }

    Ok(())
}

fn read_byte(register_block: &RegisterBlock) -> nb::Result<u8, Error> {
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
            let fifo = unsafe {
                &*((register_block.fifo().as_ptr() as *mut u8).add(0x20C00000)
                    as *mut crate::peripherals::uart0::FIFO)
            };
        } else {
            let fifo = register_block.fifo();
        }
    }

    if get_rx_fifo_count(register_block) > 0 {
        Ok(fifo.read().rxfifo_rd_byte().bits())
    } else {
        Err(nb::Error::WouldBlock)
    }
}

fn drain_fifo(register_block: &RegisterBlock, buf: &mut [u8]) -> usize {
    let mut count = 0;
    while count < buf.len() {
        if let Ok(byte) = read_byte(register_block) {
            buf[count] = byte;
            count += 1;
        } else {
            break;
        }
    }
    count
}

impl<'d, M, T> UartRx<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    /// Configure CTS pin
    pub fn with_cts<CTS: PeripheralInput>(self, cts: impl Peripheral<P = CTS> + 'd) -> Self {
        crate::into_mapped_ref!(cts);
        cts.init_input(Pull::None, Internal);
        cts.connect_input_to_peripheral(self.uart.cts_signal(), Internal);

        self
    }

    /// Fill a buffer with received bytes
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        read_bytes(self.uart.register_block(), buf)
    }

    /// Read a byte from the UART
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        read_byte(self.uart.register_block())
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes. Never blocks
    pub fn drain_fifo(&mut self, buf: &mut [u8]) -> usize {
        drain_fifo(self.uart.register_block(), buf)
    }

    /// Configures the RX-FIFO threshold
    ///
    /// # Errors
    /// `Err(Error::InvalidArgument)` if provided value exceeds maximum value
    /// for SOC :
    /// - `esp32` **0x7F**
    /// - `esp32c6`, `esp32h2` **0xFF**
    /// - `esp32c3`, `esp32c2`, `esp32s2` **0x1FF**
    /// - `esp32s3` **0x3FF**
    fn set_rx_fifo_full_threshold(&mut self, threshold: u16) -> Result<(), Error> {
        #[cfg(esp32)]
        const MAX_THRHD: u16 = 0x7F;
        #[cfg(any(esp32c6, esp32h2))]
        const MAX_THRHD: u16 = 0xFF;
        #[cfg(any(esp32c3, esp32c2, esp32s2))]
        const MAX_THRHD: u16 = 0x1FF;
        #[cfg(esp32s3)]
        const MAX_THRHD: u16 = 0x3FF;

        if threshold > MAX_THRHD {
            return Err(Error::InvalidArgument);
        }

        self.uart
            .register_block()
            .conf1()
            .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(threshold as _) });

        Ok(())
    }

    /// Configures the Receive Timeout detection setting
    ///
    /// # Arguments
    /// `timeout` - the number of symbols ("bytes") to wait for before
    /// triggering a timeout. Pass None to disable the timeout.
    ///
    ///  # Errors
    /// `Err(Error::InvalidArgument)` if the provided value exceeds the maximum
    /// value for SOC :
    /// - `esp32`: Symbol size is fixed to 8, do not pass a value > **0x7F**.
    /// - `esp32c2`, `esp32c3`, `esp32c6`, `esp32h2`, esp32s2`, esp32s3`: The
    ///   value you pass times the symbol size must be <= **0x3FF**
    fn set_rx_timeout(&mut self, timeout: Option<u8>) -> Result<(), Error> {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                const MAX_THRHD: u8 = 0x7F; // 7 bits
            } else {
                const MAX_THRHD: u16 = 0x3FF; // 10 bits
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let reg_thrhd = self.uart.register_block().conf1();
            } else if #[cfg(any(esp32c6, esp32h2))] {
                let reg_thrhd = self.uart.register_block().tout_conf();
            } else {
                let reg_thrhd = self.uart.register_block().mem_conf();
            }
        }

        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let reg_en = self.uart.register_block().tout_conf();
            } else {
                let reg_en = self.uart.register_block().conf1();
            }
        }

        match timeout {
            None => {
                reg_en.modify(|_, w| w.rx_tout_en().clear_bit());
            }
            Some(timeout) => {
                // the esp32 counts directly in number of symbols (symbol len fixed to 8)
                #[cfg(esp32)]
                let timeout_reg = timeout;
                // all other count in bits, so we need to multiply by the symbol len.
                #[cfg(not(esp32))]
                let timeout_reg = timeout as u16 * self.symbol_len as u16;

                if timeout_reg > MAX_THRHD {
                    return Err(Error::InvalidArgument);
                }

                reg_thrhd.modify(|_, w| unsafe { w.rx_tout_thrhd().bits(timeout_reg) });
                reg_en.modify(|_, w| w.rx_tout_en().set_bit());
            }
        }

        self.rx_timeout_config = timeout;

        self.uart.sync_regs();
        Ok(())
    }
}

impl<'d> UartRx<'d, Blocking> {
    /// Create a new UART RX instance in [`Blocking`] mode.
    pub fn new<RX: PeripheralInput>(
        uart: impl Peripheral<P = impl Instance> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
    ) -> Result<Self, Error> {
        UartRx::new_typed(uart.map_into(), rx)
    }

    /// Create a new UART RX instance with configuration options in
    /// [`Blocking`] mode.
    pub fn new_with_config<RX: PeripheralInput>(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
        rx: impl Peripheral<P = RX> + 'd,
    ) -> Result<Self, Error> {
        UartRx::new_with_config_typed(uart.map_into(), config, rx)
    }
}

impl<'d, T> UartRx<'d, Blocking, T>
where
    T: Instance,
{
    /// Create a new UART RX instance in [`Blocking`] mode.
    pub fn new_typed<RX: PeripheralInput>(
        uart: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_with_config_typed(uart, Config::default(), rx)
    }

    /// Create a new UART RX instance with configuration options in
    /// [`Blocking`] mode.
    pub fn new_with_config_typed<RX: PeripheralInput>(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        rx: impl Peripheral<P = RX> + 'd,
    ) -> Result<Self, Error> {
        let (uart_rx, _) = UartBuilder::new(uart).with_rx(rx).init(config)?.split();

        Ok(uart_rx)
    }
}

impl<'d> Uart<'d, Blocking> {
    /// Create a new UART instance in [`Blocking`] mode.
    pub fn new<RX: PeripheralInput, TX: PeripheralOutput>(
        uart: impl Peripheral<P = impl Instance> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_typed(uart.map_into(), rx, tx)
    }

    /// Create a new UART instance with configuration options in
    /// [`Blocking`] mode.
    pub fn new_with_config<RX: PeripheralInput, TX: PeripheralOutput>(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
        rx: impl Peripheral<P = RX> + 'd,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_with_config_typed(uart.map_into(), config, rx, tx)
    }
}

impl<'d, T> Uart<'d, Blocking, T>
where
    T: Instance,
{
    /// Create a new UART instance in [`Blocking`] mode.
    pub fn new_typed<RX: PeripheralInput, TX: PeripheralOutput>(
        uart: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        Self::new_with_config_typed(uart, Config::default(), rx, tx)
    }

    /// Create a new UART instance with configuration options in
    /// [`Blocking`] mode.
    pub fn new_with_config_typed<RX: PeripheralInput, TX: PeripheralOutput>(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        rx: impl Peripheral<P = RX> + 'd,
        tx: impl Peripheral<P = TX> + 'd,
    ) -> Result<Self, Error> {
        UartBuilder::new(uart).with_tx(tx).with_rx(rx).init(config)
    }
}

/// List of exposed UART events.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartInterrupt {
    /// Indicates that the received has detected the configured
    /// [`Uart::set_at_cmd`] character.
    AtCmd,

    /// The transmitter has finished sending out all data from the FIFO.
    TxDone,

    /// The receiver has received more data than what
    /// [`Config::rx_fifo_full_threshold`] specifies.
    RxFifoFull,
}

impl<'d, M, T> Uart<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    fn inner_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        // `self.tx.uart` and `self.rx.uart` are the same
        unsafe {
            crate::interrupt::bind_interrupt(self.tx.uart.interrupt(), handler.handler());
            crate::interrupt::enable(self.tx.uart.interrupt(), handler.priority()).unwrap();
        }
    }

    /// Configure CTS pin
    pub fn with_cts<CTS: PeripheralInput>(mut self, cts: impl Peripheral<P = CTS> + 'd) -> Self {
        self.rx = self.rx.with_cts(cts);
        self
    }

    /// Configure RTS pin
    pub fn with_rts<RTS: PeripheralOutput>(mut self, rts: impl Peripheral<P = RTS> + 'd) -> Self {
        self.tx = self.tx.with_rts(rts);
        self
    }

    fn register_block(&self) -> &RegisterBlock {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.register_block()
    }

    fn sync_regs(&self) {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.sync_regs();
    }

    /// Split the UART into a transmitter and receiver
    ///
    /// This is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartRx<'d, M, T>, UartTx<'d, M, T>) {
        (self.rx, self.tx)
    }

    /// Write bytes out over the UART
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write_bytes(data)
    }

    /// Fill a buffer with received bytes
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.rx.read_bytes(buf)
    }

    /// Configures the AT-CMD detection settings
    pub fn set_at_cmd(&mut self, config: config::AtCmdConfig) {
        let register_block = self.register_block();

        #[cfg(not(any(esp32, esp32s2)))]
        register_block
            .clk_conf()
            .modify(|_, w| w.sclk_en().clear_bit());

        register_block.at_cmd_char().write(|w| unsafe {
            w.at_cmd_char().bits(config.cmd_char);
            w.char_num().bits(config.char_num.unwrap_or(1))
        });

        if let Some(pre_idle_count) = config.pre_idle_count {
            register_block
                .at_cmd_precnt()
                .write(|w| unsafe { w.pre_idle_num().bits(pre_idle_count as _) });
        }

        if let Some(post_idle_count) = config.post_idle_count {
            register_block
                .at_cmd_postcnt()
                .write(|w| unsafe { w.post_idle_num().bits(post_idle_count as _) });
        }

        if let Some(gap_timeout) = config.gap_timeout {
            register_block
                .at_cmd_gaptout()
                .write(|w| unsafe { w.rx_gap_tout().bits(gap_timeout as _) });
        }

        #[cfg(not(any(esp32, esp32s2)))]
        register_block
            .clk_conf()
            .modify(|_, w| w.sclk_en().set_bit());

        self.sync_regs();

        self.rx.at_cmd_config = Some(config);
    }

    /// Listen for the given interrupts
    fn enable_listen(&mut self, interrupts: EnumSet<UartInterrupt>, enable: bool) {
        let reg_block = self.register_block();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    UartInterrupt::AtCmd => w.at_cmd_char_det().bit(enable),
                    UartInterrupt::TxDone => w.tx_done().bit(enable),
                    UartInterrupt::RxFifoFull => w.rxfifo_full().bit(enable),
                };
            }
            w
        });
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<UartInterrupt>>) {
        self.enable_listen(interrupts.into(), true);
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<UartInterrupt>>) {
        self.enable_listen(interrupts.into(), false);
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<UartInterrupt> {
        let mut res = EnumSet::new();
        let reg_block = self.register_block();

        let ints = reg_block.int_raw().read();

        if ints.at_cmd_char_det().bit_is_set() {
            res.insert(UartInterrupt::AtCmd);
        }
        if ints.tx_done().bit_is_set() {
            res.insert(UartInterrupt::TxDone);
        }
        if ints.rxfifo_full().bit_is_set() {
            res.insert(UartInterrupt::RxFifoFull);
        }

        res
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<UartInterrupt>) {
        let reg_block = self.register_block();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    UartInterrupt::AtCmd => w.at_cmd_char_det().clear_bit_by_one(),
                    UartInterrupt::TxDone => w.tx_done().clear_bit_by_one(),
                    UartInterrupt::RxFifoFull => w.rxfifo_full().clear_bit_by_one(),
                };
            }
            w
        });
    }

    /// Write a byte out over the UART
    pub fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        self.tx.write_byte(word)
    }

    /// Flush the transmit buffer of the UART
    pub fn flush_tx(&mut self) -> nb::Result<(), Error> {
        self.tx.flush_tx()
    }

    /// Read a byte from the UART
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        self.rx.read_byte()
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> &mut Self {
        // workaround for hardware issue, when UART stop bit set as 2-bit mode.
        #[cfg(esp32)]
        if stop_bits == config::StopBits::STOP2 {
            self.register_block()
                .rs485_conf()
                .modify(|_, w| w.dl1_en().bit(true));

            self.register_block()
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(1) });
        } else {
            self.register_block()
                .rs485_conf()
                .modify(|_, w| w.dl1_en().bit(false));

            self.register_block()
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
        }

        #[cfg(not(esp32))]
        self.register_block()
            .conf0()
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

        self
    }

    fn change_data_bits(&mut self, data_bits: config::DataBits) -> &mut Self {
        self.register_block()
            .conf0()
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

        self
    }

    fn change_parity(&mut self, parity: config::Parity) -> &mut Self {
        self.register_block().conf0().modify(|_, w| match parity {
            config::Parity::ParityNone => w.parity_en().clear_bit(),
            config::Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
            config::Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
        });

        self
    }

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn change_baud_internal(&self, baudrate: u32, clock_source: ClockSource) {
        let clocks = Clocks::get();
        let clk = match clock_source {
            ClockSource::Apb => clocks.apb_clock.to_Hz(),
            ClockSource::Xtal => clocks.xtal_clock.to_Hz(),
            ClockSource::RcFast => RC_FAST_CLK.to_Hz(),
        };

        if clock_source == ClockSource::RcFast {
            let rtc_cntl = unsafe { &*crate::peripherals::RTC_CNTL::ptr() };
            rtc_cntl
                .clk_conf()
                .modify(|_, w| w.dig_clk8m_en().variant(true));
            // esp_rom_delay_us(SOC_DELAY_RC_FAST_DIGI_SWITCH);
            crate::rom::ets_delay_us(5);
        }

        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);
        self.register_block().clk_conf().write(|w| unsafe {
            w.sclk_sel().bits(match clock_source {
                ClockSource::Apb => 1,
                ClockSource::RcFast => 2,
                ClockSource::Xtal => 3,
            });
            w.sclk_div_a().bits(0);
            w.sclk_div_b().bits(0);
            w.sclk_div_num().bits(clk_div as u8 - 1);
            w.rx_sclk_en().bit(true);
            w.tx_sclk_en().bit(true)
        });

        let divider = (clk << 4) / (baudrate * clk_div);
        let divider_integer = (divider >> 4) as u16;
        let divider_frag = (divider & 0xf) as u8;
        self.register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider_integer).frag().bits(divider_frag) });
    }

    #[cfg(any(esp32c6, esp32h2))]
    fn change_baud_internal(&self, baudrate: u32, clock_source: ClockSource) {
        let clocks = Clocks::get();
        let clk = match clock_source {
            ClockSource::Apb => clocks.apb_clock.to_Hz(),
            ClockSource::Xtal => clocks.xtal_clock.to_Hz(),
            ClockSource::RcFast => RC_FAST_CLK.to_Hz(),
        };

        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = clk.div_ceil(max_div * baudrate);

        // UART clocks are configured via PCR
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };

        match self.tx.uart.uart_number() {
            0 => {
                pcr.uart0_conf()
                    .modify(|_, w| w.uart0_rst_en().clear_bit().uart0_clk_en().set_bit());

                pcr.uart0_sclk_conf().modify(|_, w| unsafe {
                    w.uart0_sclk_div_a().bits(0);
                    w.uart0_sclk_div_b().bits(0);
                    w.uart0_sclk_div_num().bits(clk_div as u8 - 1);
                    w.uart0_sclk_sel().bits(match clock_source {
                        ClockSource::Apb => 1,
                        ClockSource::RcFast => 2,
                        ClockSource::Xtal => 3,
                    });
                    w.uart0_sclk_en().set_bit()
                });
            }
            1 => {
                pcr.uart1_conf()
                    .modify(|_, w| w.uart1_rst_en().clear_bit().uart1_clk_en().set_bit());

                pcr.uart1_sclk_conf().modify(|_, w| unsafe {
                    w.uart1_sclk_div_a().bits(0);
                    w.uart1_sclk_div_b().bits(0);
                    w.uart1_sclk_div_num().bits(clk_div as u8 - 1);
                    w.uart1_sclk_sel().bits(match clock_source {
                        ClockSource::Apb => 1,
                        ClockSource::RcFast => 2,
                        ClockSource::Xtal => 3,
                    });
                    w.uart1_sclk_en().set_bit()
                });
            }
            _ => unreachable!(), // ESP32-C6 only has 2 UART instances
        }

        let clk = clk / clk_div;
        let divider = clk / baudrate;
        let divider = divider as u16;

        self.register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

        self.sync_regs();
    }

    #[cfg(any(esp32, esp32s2))]
    fn change_baud_internal(&self, baudrate: u32, clock_source: ClockSource) {
        let clocks = Clocks::get();
        let clk = match clock_source {
            ClockSource::Apb => clocks.apb_clock.to_Hz(),
            ClockSource::RefTick => REF_TICK.to_Hz(), /* ESP32(/-S2) TRM, section 3.2.4.2
                                                       * (6.2.4.2 for S2) */
        };

        self.register_block().conf0().modify(|_, w| {
            w.tick_ref_always_on().bit(match clock_source {
                ClockSource::Apb => true,
                ClockSource::RefTick => false,
            })
        });

        let divider = clk / baudrate;

        self.register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    /// Modify UART baud rate and reset TX/RX fifo.
    pub fn change_baud(&mut self, baudrate: u32, clock_source: ClockSource) {
        self.change_baud_internal(baudrate, clock_source);
        self.txfifo_reset();
        self.rxfifo_reset();
    }

    #[inline(always)]
    fn init(&mut self, config: Config) -> Result<(), Error> {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                // Nothing to do
            } else if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                let system = unsafe { crate::peripherals::SYSTEM::steal() };
                system
                    .perip_clk_en0()
                    .modify(|_, w| w.uart_mem_clk_en().set_bit());
            } else {
                self.register_block()
                    .conf0()
                    .modify(|_, w| w.mem_clk_en().set_bit());
            }
        };

        PeripheralClockControl::enable(self.tx.uart.peripheral());
        self.uart_peripheral_reset();
        self.tx.uart.disable_rx_interrupts();
        self.tx.uart.disable_tx_interrupts();

        self.rx
            .set_rx_fifo_full_threshold(config.rx_fifo_full_threshold)?;
        self.rx.set_rx_timeout(config.rx_timeout)?;
        self.change_baud_internal(config.baudrate, config.clock_source);
        self.change_data_bits(config.data_bits);
        self.change_parity(config.parity);
        self.change_stop_bits(config.stop_bits);

        // Setting err_wr_mask stops uart from storing data when data is wrong according
        // to reference manual
        self.register_block()
            .conf0()
            .modify(|_, w| w.err_wr_mask().set_bit());

        // Reset Tx/Rx FIFOs
        self.rxfifo_reset();
        self.txfifo_reset();
        crate::rom::ets_delay_us(15);

        // Make sure we are starting in a "clean state" - previous operations might have
        // run into error conditions
        self.register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(u32::MAX) });

        Ok(())
    }

    #[inline(always)]
    fn uart_peripheral_reset(&self) {
        // don't reset the console UART - this will cause trouble (i.e. the UART will
        // start to transmit garbage)
        //
        // We should only reset the console UART if it was absolutely unused before.
        // Apparently the bootloader (and maybe the ROM code) writing to the UART is
        // already enough to make this a no-go. (i.e. one needs to mute the ROM
        // code via efuse / strapping pin AND use a silent bootloader)
        //
        // Ideally this should be configurable once we have a solution for https://github.com/esp-rs/esp-hal/issues/1111
        // see https://github.com/espressif/esp-idf/blob/5f4249357372f209fdd57288265741aaba21a2b1/components/esp_driver_uart/src/uart.c#L179
        if self.tx.uart.uart_number() == CONSOLE_UART_NUM {
            return;
        }

        #[cfg(not(any(esp32, esp32s2)))]
        self.register_block()
            .clk_conf()
            .modify(|_, w| w.rst_core().set_bit());

        PeripheralClockControl::reset(self.tx.uart.peripheral());

        #[cfg(not(any(esp32, esp32s2)))]
        self.register_block()
            .clk_conf()
            .modify(|_, w| w.rst_core().clear_bit());
    }

    fn rxfifo_reset(&mut self) {
        self.register_block()
            .conf0()
            .modify(|_, w| w.rxfifo_rst().set_bit());
        self.sync_regs();

        self.register_block()
            .conf0()
            .modify(|_, w| w.rxfifo_rst().clear_bit());
        self.sync_regs();
    }

    fn txfifo_reset(&mut self) {
        self.register_block()
            .conf0()
            .modify(|_, w| w.txfifo_rst().set_bit());
        self.sync_regs();

        self.register_block()
            .conf0()
            .modify(|_, w| w.txfifo_rst().clear_bit());
        self.sync_regs();
    }
}

impl<T> crate::private::Sealed for Uart<'_, Blocking, T> where T: Instance {}

impl<T> InterruptConfigurable for Uart<'_, Blocking, T>
where
    T: Instance,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.inner_set_interrupt_handler(handler);
    }
}

/// UART Peripheral Instance
pub trait Instance: Peripheral<P = Self> + PeripheralMarker + Into<AnyUart> + 'static {
    /// Returns a reference to the UART register block for the specific
    /// instance.
    ///
    /// # Safety
    /// This function returns a reference to the raw hardware registers, so
    /// direct interaction with the registers may require careful handling
    /// to avoid unintended side effects.
    fn register_block(&self) -> &RegisterBlock;

    /// Returns the UART number associated with this instance (e.g., UART0,
    /// UART1, etc.).
    fn uart_number(&self) -> usize;

    /// Returns the interrupt handler for this UART instance.
    fn async_handler(&self) -> InterruptHandler;

    /// Returns the interrupt associated with this UART instance.
    fn interrupt(&self) -> Interrupt;

    #[doc(hidden)]
    #[inline(always)]
    fn sync_regs(&self) {
        #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
        {
            cfg_if::cfg_if! {
                if #[cfg(any(esp32c6, esp32h2))] {
                    let update_reg = self.register_block().reg_update();
                } else {
                    let update_reg = self.register_block().id();
                }
            }

            update_reg.modify(|_, w| w.reg_update().set_bit());

            while update_reg.read().reg_update().bit_is_set() {
                // wait
            }
        }
    }

    /// Disables all TX-related interrupts for this UART instance.
    ///
    /// This function clears and disables the `transmit FIFO empty` interrupt,
    /// `transmit break done`, `transmit break idle done`, and `transmit done`
    /// interrupts.
    fn disable_tx_interrupts(&self) {
        self.register_block().int_clr().write(|w| {
            w.txfifo_empty().clear_bit_by_one();
            w.tx_brk_done().clear_bit_by_one();
            w.tx_brk_idle_done().clear_bit_by_one();
            w.tx_done().clear_bit_by_one()
        });

        self.register_block().int_ena().write(|w| {
            w.txfifo_empty().clear_bit();
            w.tx_brk_done().clear_bit();
            w.tx_brk_idle_done().clear_bit();
            w.tx_done().clear_bit()
        });
    }

    /// Disables all RX-related interrupts for this UART instance.
    ///
    /// This function clears and disables the `receive FIFO full` interrupt,
    /// `receive FIFO overflow`, `receive FIFO timeout`, and `AT command
    /// character detection` interrupts.
    fn disable_rx_interrupts(&self) {
        self.register_block().int_clr().write(|w| {
            w.rxfifo_full().clear_bit_by_one();
            w.rxfifo_ovf().clear_bit_by_one();
            w.rxfifo_tout().clear_bit_by_one();
            w.at_cmd_char_det().clear_bit_by_one()
        });

        self.register_block().int_ena().write(|w| {
            w.rxfifo_full().clear_bit();
            w.rxfifo_ovf().clear_bit();
            w.rxfifo_tout().clear_bit();
            w.at_cmd_char_det().clear_bit()
        });
    }

    /// Checks if the TX line is idle for this UART instance.
    ///
    /// Returns `true` if the transmit line is idle, meaning no data is
    /// currently being transmitted.
    fn is_tx_idle(&self) -> bool {
        #[cfg(esp32)]
        let status = self.register_block().status();
        #[cfg(not(esp32))]
        let status = self.register_block().fsm_status();

        status.read().st_utx_out().bits() == 0x0
    }

    /// Checks if the RX line is idle for this UART instance.
    ///
    /// Returns `true` if the receive line is idle, meaning no data is currently
    /// being received.
    fn is_rx_idle(&self) -> bool {
        #[cfg(esp32)]
        let status = self.register_block().status();
        #[cfg(not(esp32))]
        let status = self.register_block().fsm_status();

        status.read().st_urx_out().bits() == 0x0
    }

    /// Returns the output signal identifier for the TX pin of this UART
    /// instance.
    fn tx_signal(&self) -> OutputSignal;

    /// Returns the input signal identifier for the RX pin of this UART
    /// instance.
    fn rx_signal(&self) -> InputSignal;

    /// Returns the input signal identifier for the CTS (Clear to Send) pin of
    /// this UART instance.
    fn cts_signal(&self) -> InputSignal;

    /// Returns the output signal identifier for the RTS (Request to Send) pin
    /// of this UART instance.
    fn rts_signal(&self) -> OutputSignal;
}

macro_rules! impl_instance {
    ($inst:ident, $num:expr, $txd:ident, $rxd:ident, $cts:ident, $rts:ident, $async_handler:path) => {
        impl Instance for crate::peripherals::$inst {
            #[inline(always)]
            fn register_block(&self) -> &RegisterBlock {
                unsafe { &*crate::peripherals::$inst::PTR }
            }

            #[inline(always)]
            fn uart_number(&self) -> usize {
                $num
            }

            #[inline(always)]
            fn async_handler(&self) -> InterruptHandler {
                $async_handler
            }

            #[inline(always)]
            fn interrupt(&self) -> Interrupt {
                Interrupt::$inst
            }

            #[inline(always)]
            fn tx_signal(&self) -> OutputSignal {
                OutputSignal::$txd
            }

            #[inline(always)]
            fn rx_signal(&self) -> InputSignal {
                InputSignal::$rxd
            }

            #[inline(always)]
            fn cts_signal(&self) -> InputSignal {
                InputSignal::$cts
            }

            #[inline(always)]
            fn rts_signal(&self) -> OutputSignal {
                OutputSignal::$rts
            }
        }
    };
}

impl_instance!(UART0, 0, U0TXD, U0RXD, U0CTS, U0RTS, asynch::uart0);
impl_instance!(UART1, 1, U1TXD, U1RXD, U1CTS, U1RTS, asynch::uart1);
#[cfg(uart2)]
impl_instance!(UART2, 2, U2TXD, U2RXD, U2CTS, U2RTS, asynch::uart2);

impl<T, M> ufmt_write::uWrite for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.tx.write_str(s)
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        self.tx.write_char(ch)
    }
}

impl<T, M> ufmt_write::uWrite for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_bytes(s.as_bytes())?;
        Ok(())
    }
}

impl<T, M> core::fmt::Write for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

impl<T, M> core::fmt::Write for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

impl<T, M> embedded_hal_02::serial::Write<u8> for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<T, M> embedded_hal_02::serial::Write<u8> for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

impl<T, M> embedded_hal_02::serial::Read<u8> for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl<T, M> embedded_hal_02::serial::Read<u8> for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T, M> embedded_hal_nb::serial::ErrorType for Uart<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_hal_nb::serial::ErrorType for UartTx<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_hal_nb::serial::ErrorType for UartRx<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_hal_nb::serial::Read for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T, M> embedded_hal_nb::serial::Read for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T, M> embedded_hal_nb::serial::Write for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

impl<T, M> embedded_hal_nb::serial::Write for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

impl<T, M> embedded_io::ErrorType for Uart<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_io::ErrorType for UartTx<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_io::ErrorType for UartRx<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_io::Read for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}

impl<T, M> embedded_io::Read for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        while get_rx_fifo_count(self.uart.register_block()) == 0 {
            // Block until we received at least one byte
        }

        Ok(self.drain_fifo(buf))
    }
}

impl<T, M> embedded_io::ReadReady for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.rx.read_ready()
    }
}

impl<T, M> embedded_io::ReadReady for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(get_rx_fifo_count(self.uart.register_block()) > 0)
    }
}

impl<T, M> embedded_io::Write for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<T, M> embedded_io::Write for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            match self.flush_tx() {
                Ok(_) => break,
                Err(nb::Error::WouldBlock) => { /* Wait */ }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }

        Ok(())
    }
}

mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use enumset::{EnumSet, EnumSetType};
    use procmacros::handler;

    use super::*;
    use crate::Async;

    const NUM_UART: usize = 1 + cfg!(uart1) as usize + cfg!(uart2) as usize;

    static TX_WAKERS: [AtomicWaker; NUM_UART] = [const { AtomicWaker::new() }; NUM_UART];
    static RX_WAKERS: [AtomicWaker; NUM_UART] = [const { AtomicWaker::new() }; NUM_UART];

    #[derive(EnumSetType, Debug)]
    pub(crate) enum TxEvent {
        TxDone,
        TxFiFoEmpty,
    }
    #[derive(EnumSetType, Debug)]
    pub(crate) enum RxEvent {
        FifoFull,
        CmdCharDetected,
        FifoOvf,
        FifoTout,
        GlitchDetected,
        FrameError,
        ParityError,
    }

    /// A future that resolves when the passed interrupt is triggered,
    /// or has been triggered in the meantime (flag set in INT_RAW).
    /// Upon construction the future enables the passed interrupt and when it
    /// is dropped it disables the interrupt again. The future returns the event
    /// that was initially passed, when it resolves.
    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub(crate) struct UartRxFuture<'d, T>
    where
        T: Instance,
    {
        events: EnumSet<RxEvent>,
        uart: &'d mut T,
        registered: bool,
    }
    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub(crate) struct UartTxFuture<'d, T>
    where
        T: Instance,
    {
        events: EnumSet<TxEvent>,
        uart: &'d mut T,
        registered: bool,
    }

    impl<'d, T: Instance> UartRxFuture<'d, T> {
        pub fn new(uart: &'d mut T, events: impl Into<EnumSet<RxEvent>>) -> Self {
            Self {
                events: events.into(),
                uart,
                registered: false,
            }
        }

        fn get_triggered_events(&self) -> EnumSet<RxEvent> {
            let interrupts_enabled = self.uart.register_block().int_ena().read();
            let mut events_triggered = EnumSet::new();
            for event in self.events {
                let event_triggered = match event {
                    RxEvent::FifoFull => interrupts_enabled.rxfifo_full().bit_is_clear(),
                    RxEvent::CmdCharDetected => interrupts_enabled.at_cmd_char_det().bit_is_clear(),

                    RxEvent::FifoOvf => interrupts_enabled.rxfifo_ovf().bit_is_clear(),
                    RxEvent::FifoTout => interrupts_enabled.rxfifo_tout().bit_is_clear(),
                    RxEvent::GlitchDetected => interrupts_enabled.glitch_det().bit_is_clear(),
                    RxEvent::FrameError => interrupts_enabled.frm_err().bit_is_clear(),
                    RxEvent::ParityError => interrupts_enabled.parity_err().bit_is_clear(),
                };
                if event_triggered {
                    events_triggered |= event;
                }
            }
            events_triggered
        }

        fn enable_listen(&self, enable: bool) {
            self.uart.register_block().int_ena().modify(|_, w| {
                for event in self.events {
                    match event {
                        RxEvent::FifoFull => w.rxfifo_full().bit(enable),
                        RxEvent::CmdCharDetected => w.at_cmd_char_det().bit(enable),
                        RxEvent::FifoOvf => w.rxfifo_ovf().bit(enable),
                        RxEvent::FifoTout => w.rxfifo_tout().bit(enable),
                        RxEvent::GlitchDetected => w.glitch_det().bit(enable),
                        RxEvent::FrameError => w.frm_err().bit(enable),
                        RxEvent::ParityError => w.parity_err().bit(enable),
                    };
                }
                w
            });
        }
    }

    impl<'d, T: Instance> core::future::Future for UartRxFuture<'d, T> {
        type Output = EnumSet<RxEvent>;

        fn poll(
            mut self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            if !self.registered {
                RX_WAKERS[self.uart.uart_number()].register(cx.waker());
                self.enable_listen(true);
                self.registered = true;
            }
            let events = self.get_triggered_events();
            if !events.is_empty() {
                Poll::Ready(events)
            } else {
                Poll::Pending
            }
        }
    }

    impl<'d, T: Instance> Drop for UartRxFuture<'d, T> {
        fn drop(&mut self) {
            // Although the isr disables the interrupt that occurred directly, we need to
            // disable the other interrupts (= the ones that did not occur), as
            // soon as this future goes out of scope.
            self.enable_listen(false);
        }
    }

    impl<'d, T: Instance> UartTxFuture<'d, T> {
        pub fn new(uart: &'d mut T, events: impl Into<EnumSet<TxEvent>>) -> Self {
            Self {
                events: events.into(),
                uart,
                registered: false,
            }
        }

        fn get_triggered_events(&self) -> bool {
            let interrupts_enabled = self.uart.register_block().int_ena().read();
            let mut event_triggered = false;
            for event in self.events {
                event_triggered |= match event {
                    TxEvent::TxDone => interrupts_enabled.tx_done().bit_is_clear(),
                    TxEvent::TxFiFoEmpty => interrupts_enabled.txfifo_empty().bit_is_clear(),
                }
            }
            event_triggered
        }

        fn enable_listen(&self, enable: bool) {
            self.uart.register_block().int_ena().modify(|_, w| {
                for event in self.events {
                    match event {
                        TxEvent::TxDone => w.tx_done().bit(enable),
                        TxEvent::TxFiFoEmpty => w.txfifo_empty().bit(enable),
                    };
                }
                w
            });
        }
    }

    impl<'d, T: Instance> core::future::Future for UartTxFuture<'d, T> {
        type Output = ();

        fn poll(
            mut self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            if !self.registered {
                TX_WAKERS[self.uart.uart_number()].register(cx.waker());
                self.enable_listen(true);
                self.registered = true;
            }

            if self.get_triggered_events() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'d, T: Instance> Drop for UartTxFuture<'d, T> {
        fn drop(&mut self) {
            // Although the isr disables the interrupt that occurred directly, we need to
            // disable the other interrupts (= the ones that did not occur), as
            // soon as this future goes out of scope.
            self.enable_listen(false);
        }
    }

    impl<'d> Uart<'d, Async> {
        /// Create a new UART instance with defaults in [`Async`] mode.
        pub fn new_async<RX: PeripheralInput, TX: PeripheralOutput>(
            uart: impl Peripheral<P = impl Instance> + 'd,
            rx: impl Peripheral<P = RX> + 'd,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            Uart::new_async_typed(uart.map_into(), rx, tx)
        }

        /// Create a new UART instance with configuration options in [`Async`]
        /// mode.
        pub fn new_async_with_config<RX: PeripheralInput, TX: PeripheralOutput>(
            uart: impl Peripheral<P = impl Instance> + 'd,
            config: Config,
            rx: impl Peripheral<P = RX> + 'd,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            Uart::new_async_with_config_typed(uart.map_into(), config, rx, tx)
        }
    }

    impl<'d, T> Uart<'d, Async, T>
    where
        T: Instance,
    {
        /// Create a new UART instance with defaults in [`Async`] mode.
        pub fn new_async_typed<RX: PeripheralInput, TX: PeripheralOutput>(
            uart: impl Peripheral<P = T> + 'd,
            rx: impl Peripheral<P = RX> + 'd,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            Self::new_async_with_config_typed(uart, Config::default(), rx, tx)
        }

        /// Create a new UART instance with configuration options in [`Async`]
        /// mode.
        pub fn new_async_with_config_typed<RX: PeripheralInput, TX: PeripheralOutput>(
            uart: impl Peripheral<P = T> + 'd,
            config: Config,
            rx: impl Peripheral<P = RX> + 'd,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            // FIXME: at the time of writing, the order of the pin assignments matters:
            // first binding RX, then TX makes tests fail. This is bad and needs to be
            // figured out.
            let mut this = UartBuilder::new(uart)
                .with_tx(tx)
                .with_rx(rx)
                .init(config)?;

            this.inner_set_interrupt_handler(this.tx.uart.async_handler());

            Ok(this)
        }
    }

    impl<T> Uart<'_, Async, T>
    where
        T: Instance,
    {
        /// Asynchronously reads data from the UART receive buffer into the
        /// provided buffer.
        pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            self.rx.read_async(buf).await
        }

        /// Asynchronously writes data to the UART transmit buffer.
        pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, Error> {
            self.tx.write_async(words).await
        }

        /// Asynchronously flushes the UART transmit buffer.
        pub async fn flush_async(&mut self) -> Result<(), Error> {
            self.tx.flush_async().await
        }
    }

    impl<'d> UartTx<'d, Async> {
        /// Create a new UART TX instance in [`Async`] mode.
        pub fn new_async<TX: PeripheralOutput>(
            uart: impl Peripheral<P = impl Instance> + 'd,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            UartTx::new_async_typed(uart.map_into(), tx)
        }

        /// Create a new UART TX instance with configuration options in
        /// [`Async`] mode.
        pub fn new_async_with_config<TX: PeripheralOutput>(
            uart: impl Peripheral<P = impl Instance> + 'd,
            config: Config,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            UartTx::new_async_with_config_typed(uart.map_into(), config, tx)
        }
    }

    impl<'d, T> UartTx<'d, Async, T>
    where
        T: Instance,
    {
        /// Create a new UART TX instance in [`Async`] mode.
        pub fn new_async_typed<TX: PeripheralOutput>(
            uart: impl Peripheral<P = T> + 'd,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            UartTx::new_async_with_config_typed(uart, Config::default(), tx)
        }

        /// Create a new UART TX instance with configuration options in
        /// [`Async`] mode.
        pub fn new_async_with_config_typed<TX: PeripheralOutput>(
            uart: impl Peripheral<P = T> + 'd,
            config: Config,
            tx: impl Peripheral<P = TX> + 'd,
        ) -> Result<Self, Error> {
            let mut this = UartBuilder::new(uart).with_tx(tx).init(config)?;

            this.inner_set_interrupt_handler(this.tx.uart.async_handler());

            let (_, uart_tx) = this.split();
            Ok(uart_tx)
        }

        /// Asynchronously writes data to the UART transmit buffer in chunks.
        ///
        /// This function sends the contents of the provided buffer `words` over
        /// the UART. Data is written in chunks to avoid overflowing the
        /// transmit FIFO, and the function waits asynchronously when
        /// necessary for space in the buffer to become available.
        pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, Error> {
            let mut count = 0;
            let mut offset: usize = 0;
            loop {
                let register_block = self.uart.register_block();
                let mut next_offset =
                    offset + (UART_FIFO_SIZE - get_tx_fifo_count(register_block)) as usize;
                if next_offset > words.len() {
                    next_offset = words.len();
                }

                for byte in &words[offset..next_offset] {
                    self.write_byte(*byte).unwrap(); // should never fail
                    count += 1;
                }

                if next_offset >= words.len() {
                    break;
                }

                offset = next_offset;
                UartTxFuture::<T>::new(&mut *self.uart, TxEvent::TxFiFoEmpty).await;
            }

            Ok(count)
        }

        /// Asynchronously flushes the UART transmit buffer.
        ///
        /// This function ensures that all pending data in the transmit FIFO has
        /// been sent over the UART. If the FIFO contains data, it waits
        /// for the transmission to complete before returning.
        pub async fn flush_async(&mut self) -> Result<(), Error> {
            let count = get_tx_fifo_count(self.uart.register_block());
            if count > 0 {
                UartTxFuture::<T>::new(&mut *self.uart, TxEvent::TxDone).await;
            }

            Ok(())
        }
    }

    impl<'d> UartRx<'d, Async> {
        /// Create a new UART RX instance in [`Async`] mode.
        pub fn new_async<RX: PeripheralInput>(
            uart: impl Peripheral<P = impl Instance> + 'd,
            rx: impl Peripheral<P = RX> + 'd,
        ) -> Result<Self, Error> {
            Self::new_async_typed(uart.map_into(), rx)
        }

        /// Create a new UART RX instance with configuration options in
        /// [`Async`] mode.
        pub fn new_async_with_config<RX: PeripheralInput>(
            uart: impl Peripheral<P = impl Instance> + 'd,
            config: Config,
            rx: impl Peripheral<P = RX> + 'd,
        ) -> Result<Self, Error> {
            Self::new_async_with_config_typed(uart.map_into(), config, rx)
        }
    }

    impl<'d, T> UartRx<'d, Async, T>
    where
        T: Instance,
    {
        /// Create a new UART RX instance in [`Async`] mode.
        pub fn new_async_typed<RX: PeripheralInput>(
            uart: impl Peripheral<P = T> + 'd,
            rx: impl Peripheral<P = RX> + 'd,
        ) -> Result<Self, Error> {
            Self::new_async_with_config_typed(uart, Config::default(), rx)
        }

        /// Create a new UART RX instance with configuration options in
        /// [`Async`] mode.
        pub fn new_async_with_config_typed<RX: PeripheralInput>(
            uart: impl Peripheral<P = T> + 'd,
            config: Config,
            rx: impl Peripheral<P = RX> + 'd,
        ) -> Result<Self, Error> {
            let mut this = UartBuilder::new(uart).with_rx(rx).init(config)?;

            this.inner_set_interrupt_handler(this.tx.uart.async_handler());

            let (uart_rx, _) = this.split();
            Ok(uart_rx)
        }

        /// Read async to buffer slice `buf`.
        /// Waits until at least one byte is in the Rx FiFo
        /// and one of the following interrupts occurs:
        /// - `RXFIFO_FULL`
        /// - `RXFIFO_OVF`
        /// - `AT_CMD_CHAR_DET` (only if `set_at_cmd` was called)
        /// - `RXFIFO_TOUT` (only if `set_rx_timeout was called)
        ///
        /// The interrupts in question are enabled during the body of this
        /// function. The method immediately returns when the interrupt
        /// has already occurred before calling this method (e.g. status
        /// bit set, but interrupt not enabled)
        ///
        /// # Params
        /// - `buf` buffer slice to write the bytes into
        ///
        ///
        /// # Ok
        /// When successful, returns the number of bytes written to buf.
        /// This method will never return Ok(0)
        pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            if buf.is_empty() {
                return Err(Error::InvalidArgument);
            }

            loop {
                let mut events = RxEvent::FifoFull
                    | RxEvent::FifoOvf
                    | RxEvent::FrameError
                    | RxEvent::GlitchDetected
                    | RxEvent::ParityError;

                if self.at_cmd_config.is_some() {
                    events |= RxEvent::CmdCharDetected;
                }

                if self.rx_timeout_config.is_some() {
                    events |= RxEvent::FifoTout;
                }
                let events_happened = UartRxFuture::<T>::new(&mut self.uart, events).await;
                // always drain the fifo, if an error has occurred the data is lost
                let read_bytes = self.drain_fifo(buf);
                // check error events
                for event_happened in events_happened {
                    match event_happened {
                        RxEvent::FifoOvf => return Err(Error::RxFifoOvf),
                        RxEvent::GlitchDetected => return Err(Error::RxGlitchDetected),
                        RxEvent::FrameError => return Err(Error::RxFrameError),
                        RxEvent::ParityError => return Err(Error::RxParityError),
                        RxEvent::FifoFull | RxEvent::CmdCharDetected | RxEvent::FifoTout => {
                            continue
                        }
                    }
                }
                // Unfortunately, the uart's rx-timeout counter counts up whenever there is
                // data in the fifo, even if the interrupt is disabled and the status bit
                // cleared. Since we do not drain the fifo in the interrupt handler, we need to
                // reset the counter here, after draining the fifo.
                self.uart
                    .register_block()
                    .int_clr()
                    .write(|w| w.rxfifo_tout().clear_bit_by_one());

                if read_bytes > 0 {
                    return Ok(read_bytes);
                }
            }
        }
    }

    impl<T> embedded_io_async::Read for Uart<'_, Async, T>
    where
        T: Instance,
    {
        /// In contrast to the documentation of embedded_io_async::Read, this
        /// method blocks until an uart interrupt occurs.
        /// See UartRx::read_async for more details.
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_async(buf).await
        }
    }

    impl<T> embedded_io_async::Read for UartRx<'_, Async, T>
    where
        T: Instance,
    {
        /// In contrast to the documentation of embedded_io_async::Read, this
        /// method blocks until an uart interrupt occurs.
        /// See UartRx::read_async for more details.
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_async(buf).await
        }
    }

    impl<T> embedded_io_async::Write for Uart<'_, Async, T>
    where
        T: Instance,
    {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_async(buf).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush_async().await
        }
    }

    impl<T> embedded_io_async::Write for UartTx<'_, Async, T>
    where
        T: Instance,
    {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_async(buf).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush_async().await
        }
    }

    /// Interrupt handler for all UART instances
    /// Clears and disables interrupts that have occurred and have their enable
    /// bit set. The fact that an interrupt has been disabled is used by the
    /// futures to detect that they should indeed resolve after being woken up
    fn intr_handler(uart: &RegisterBlock) -> (bool, bool) {
        let interrupts = uart.int_st().read();
        let interrupt_bits = interrupts.bits(); // = int_raw & int_ena
        if interrupt_bits == 0 {
            return (false, false);
        }
        let rx_wake = interrupts.rxfifo_full().bit_is_set()
            || interrupts.rxfifo_ovf().bit_is_set()
            || interrupts.rxfifo_tout().bit_is_set()
            || interrupts.at_cmd_char_det().bit_is_set()
            || interrupts.glitch_det().bit_is_set()
            || interrupts.frm_err().bit_is_set()
            || interrupts.parity_err().bit_is_set();
        let tx_wake = interrupts.tx_done().bit_is_set() || interrupts.txfifo_empty().bit_is_set();
        uart.int_clr().write(|w| unsafe { w.bits(interrupt_bits) });
        uart.int_ena()
            .modify(|r, w| unsafe { w.bits(r.bits() & !interrupt_bits) });

        (rx_wake, tx_wake)
    }

    #[cfg(uart0)]
    #[handler]
    pub(super) fn uart0() {
        let uart = unsafe { &*crate::peripherals::UART0::ptr() };
        let (rx, tx) = intr_handler(uart);
        if rx {
            RX_WAKERS[0].wake();
        }
        if tx {
            TX_WAKERS[0].wake();
        }
    }

    #[cfg(uart1)]
    #[handler]
    pub(super) fn uart1() {
        let uart = unsafe { &*crate::peripherals::UART1::ptr() };
        let (rx, tx) = intr_handler(uart);
        if rx {
            RX_WAKERS[1].wake();
        }
        if tx {
            TX_WAKERS[1].wake();
        }
    }

    #[cfg(uart2)]
    #[handler]
    pub(super) fn uart2() {
        let uart = unsafe { &*crate::peripherals::UART2::ptr() };
        let (rx, tx) = intr_handler(uart);
        if rx {
            RX_WAKERS[2].wake();
        }
        if tx {
            TX_WAKERS[2].wake();
        }
    }
}

/// Low-power UART
#[cfg(lp_uart)]
pub mod lp_uart {
    use crate::{
        gpio::lp_io::{LowPowerInput, LowPowerOutput},
        peripherals::{LP_CLKRST, LP_UART},
        uart::config::{self, Config},
    };
    /// LP-UART driver
    ///
    /// The driver uses XTAL as clock source.
    pub struct LpUart {
        uart: LP_UART,
    }

    impl LpUart {
        /// Initialize the UART driver using the default configuration
        // TODO: CTS and RTS pins
        pub fn new(uart: LP_UART, _tx: LowPowerOutput<'_, 5>, _rx: LowPowerInput<'_, 4>) -> Self {
            let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
            let lp_aon = unsafe { &*crate::peripherals::LP_AON::PTR };

            // FIXME: use GPIO APIs to configure pins
            lp_aon
                .gpio_mux()
                .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | 1 << 4) });
            lp_aon
                .gpio_mux()
                .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | 1 << 5) });

            lp_io.gpio(4).modify(|_, w| unsafe { w.mcu_sel().bits(1) });
            lp_io.gpio(5).modify(|_, w| unsafe { w.mcu_sel().bits(1) });

            Self::new_with_config(uart, Config::default())
        }

        /// Initialize the UART driver using the provided configuration
        pub fn new_with_config(uart: LP_UART, config: Config) -> Self {
            let mut me = Self { uart };

            // Set UART mode - do nothing for LP

            // Disable UART parity
            // 8-bit world
            // 1-bit stop bit
            me.uart.conf0().modify(|_, w| unsafe {
                w.parity().clear_bit();
                w.parity_en().clear_bit();
                w.bit_num().bits(0x3);
                w.stop_bit_num().bits(0x1)
            });
            // Set tx idle
            me.uart
                .idle_conf()
                .modify(|_, w| unsafe { w.tx_idle_num().bits(0) });
            // Disable hw-flow control
            me.uart
                .hwfc_conf()
                .modify(|_, w| w.rx_flow_en().clear_bit());

            // Get source clock frequency
            // default == SOC_MOD_CLK_RTC_FAST == 2

            // LP_CLKRST.lpperi.lp_uart_clk_sel = 0;
            unsafe { &*LP_CLKRST::PTR }
                .lpperi()
                .modify(|_, w| w.lp_uart_clk_sel().clear_bit());

            // Override protocol parameters from the configuration
            // uart_hal_set_baudrate(&hal, cfg->uart_proto_cfg.baud_rate, sclk_freq);
            me.change_baud_internal(config.baudrate, config.clock_source);
            // uart_hal_set_parity(&hal, cfg->uart_proto_cfg.parity);
            me.change_parity(config.parity);
            // uart_hal_set_data_bit_num(&hal, cfg->uart_proto_cfg.data_bits);
            me.change_data_bits(config.data_bits);
            // uart_hal_set_stop_bits(&hal, cfg->uart_proto_cfg.stop_bits);
            me.change_stop_bits(config.stop_bits);
            // uart_hal_set_tx_idle_num(&hal, LP_UART_TX_IDLE_NUM_DEFAULT);
            me.change_tx_idle(0); // LP_UART_TX_IDLE_NUM_DEFAULT == 0

            // Reset Tx/Rx FIFOs
            me.rxfifo_reset();
            me.txfifo_reset();

            me
        }

        fn rxfifo_reset(&mut self) {
            self.uart.conf0().modify(|_, w| w.rxfifo_rst().set_bit());
            self.update();

            self.uart.conf0().modify(|_, w| w.rxfifo_rst().clear_bit());
            self.update();
        }

        fn txfifo_reset(&mut self) {
            self.uart.conf0().modify(|_, w| w.txfifo_rst().set_bit());
            self.update();

            self.uart.conf0().modify(|_, w| w.txfifo_rst().clear_bit());
            self.update();
        }

        fn update(&mut self) {
            self.uart
                .reg_update()
                .modify(|_, w| w.reg_update().set_bit());
            while self.uart.reg_update().read().reg_update().bit_is_set() {
                // wait
            }
        }

        fn change_baud_internal(&mut self, baudrate: u32, clock_source: super::ClockSource) {
            // TODO: Currently it's not possible to use XtalD2Clk
            let clk = 16_000_000_u32;
            let max_div = 0b1111_1111_1111 - 1;
            let clk_div = clk.div_ceil(max_div * baudrate);

            self.uart.clk_conf().modify(|_, w| unsafe {
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.sclk_div_num().bits(clk_div as u8 - 1);
                w.sclk_sel().bits(match clock_source {
                    super::ClockSource::Xtal => 3,
                    super::ClockSource::RcFast => 2,
                    super::ClockSource::Apb => panic!("Wrong clock source for LP_UART"),
                });
                w.sclk_en().set_bit()
            });

            let clk = clk / clk_div;
            let divider = clk / baudrate;
            let divider = divider as u16;

            self.uart
                .clkdiv()
                .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

            self.update();
        }

        /// Modify UART baud rate and reset TX/RX fifo.
        pub fn change_baud(&mut self, baudrate: u32, clock_source: super::ClockSource) {
            self.change_baud_internal(baudrate, clock_source);
            self.txfifo_reset();
            self.rxfifo_reset();
        }

        fn change_parity(&mut self, parity: config::Parity) -> &mut Self {
            if parity != config::Parity::ParityNone {
                self.uart
                    .conf0()
                    .modify(|_, w| w.parity().bit((parity as u8 & 0x1) != 0));
            }

            self.uart.conf0().modify(|_, w| match parity {
                config::Parity::ParityNone => w.parity_en().clear_bit(),
                config::Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
                config::Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
            });

            self
        }

        fn change_data_bits(&mut self, data_bits: config::DataBits) -> &mut Self {
            self.uart
                .conf0()
                .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

            self.update();

            self
        }

        fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> &mut Self {
            self.uart
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

            self.update();
            self
        }

        fn change_tx_idle(&mut self, idle_num: u16) -> &mut Self {
            self.uart
                .idle_conf()
                .modify(|_, w| unsafe { w.tx_idle_num().bits(idle_num) });

            self.update();
            self
        }
    }
}

crate::any_peripheral! {
    /// Any UART peripheral.
    pub peripheral AnyUart {
        #[cfg(uart0)]
        Uart0(crate::peripherals::UART0),
        #[cfg(uart1)]
        Uart1(crate::peripherals::UART1),
        #[cfg(uart2)]
        Uart2(crate::peripherals::UART2),
    }
}

impl Instance for AnyUart {
    delegate::delegate! {
        to match &self.0 {
            #[cfg(uart0)]
            AnyUartInner::Uart0(uart) => uart,
            #[cfg(uart1)]
            AnyUartInner::Uart1(uart) => uart,
            #[cfg(uart2)]
            AnyUartInner::Uart2(uart) => uart,
        } {
            fn register_block(&self) -> &RegisterBlock;
            fn uart_number(&self) -> usize;
            fn async_handler(&self) -> InterruptHandler;
            fn interrupt(&self) -> Interrupt;
            fn tx_signal(&self) -> OutputSignal;
            fn rx_signal(&self) -> InputSignal;
            fn cts_signal(&self) -> InputSignal;
            fn rts_signal(&self) -> OutputSignal;
        }
    }
}
