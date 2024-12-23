//! SPI loopback test using DMA - send from PSRAM receive to internal RAM
//!
//! The following wiring is assumed:
//! - SCLK => GPIO42
//! - MISO => (looped back to MOSI via the GPIO MUX)
//! - MOSI => GPIO48
//! - CS   => GPIO38
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//!
//! If your module is quad PSRAM then you need to change the `psram` feature in the
//! in the features line below to `quad-psram`.

//% FEATURES: esp-hal/log esp-hal/quad-psram esp-hal/unstable
//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::peripherals::SPI2;
use esp_hal::{
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf, ExternalBurstConfig},
    entry,
    peripheral::Peripheral,
    spi::{
        master::{Config, Spi},
        Mode,
    },
    time::RateExtU32,
};
extern crate alloc;
use log::*;

macro_rules! dma_alloc_buffer {
    ($size:expr, $align:expr) => {{
        let layout = core::alloc::Layout::from_size_align($size, $align).unwrap();
        unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("dma_alloc_buffer: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr, $size)
        }
    }};
}

const DMA_BUFFER_SIZE: usize = 8192;
const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size64;
const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Debug);
    info!("Starting SPI loopback test");
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    let delay = Delay::new();

    let sclk = peripherals.GPIO42;
    let mosi = peripherals.GPIO48;
    let miso = unsafe { mosi.clone_unchecked() };
    let cs = peripherals.GPIO38;
    let sio2 = peripherals.GPIO7;
    let sio3 = peripherals.GPIO6;

    let (_, tx_descriptors) =
        esp_hal::dma_descriptors_chunk_size!(0, DMA_BUFFER_SIZE, DMA_CHUNK_SIZE);
    let tx_buffer = dma_alloc_buffer!(DMA_BUFFER_SIZE, DMA_ALIGNMENT as usize);
    info!(
        "TX: {:p} len {} ({} descripters)",
        tx_buffer.as_ptr(),
        tx_buffer.len(),
        tx_descriptors.len()
    );
    let mut dma_tx_buf =
        Some(DmaTxBuf::new_with_config(tx_descriptors, tx_buffer, DMA_ALIGNMENT).unwrap());

    log::info!("First dmabuffer: {:?}", dma_tx_buf.as_mut());

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        esp_hal::dma_buffers!(DMA_BUFFER_SIZE);

    let dma_rx_buf2 = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx_buf2 = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(25.MHz())
            .with_mode(Mode::Mode0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_miso(miso)
    .with_mosi(mosi)
    .with_sio2(sio2)
    .with_sio3(sio3)
    .with_cs(cs)
    .with_dma(peripherals.DMA_CH0);

    delay.delay_millis(100); // delay to let the above messages display

    for (i, v) in dma_tx_buf
        .as_mut()
        .unwrap()
        .as_mut_slice()
        .iter_mut()
        .enumerate()
    {
        *v = (i % 256) as u8;
    }

    let mut i = 0;

    loop {
        use esp_hal::spi::master::{Address, Command};
        let mut dma_buf = dma_tx_buf.take().unwrap();
        let transfer = spi
            .half_duplex_write(
                esp_hal::spi::DataMode::Quad,
                Command::None,
                Address::Address24(0, esp_hal::spi::DataMode::Quad),
                0u8,
                dma_buf.len(),
                dma_buf,
            )
            .unwrap();
        // let transfer = spi.write(dma_buf.len(), dma_buf).unwrap();
        delay.delay_micros(593);
        // delay.delay_millis(1);

        log::info!("SPI2: {}", unsafe {
            *(SPI2::ptr().add(0x0040) as *const u32).as_ref().unwrap()
        });
        (spi, dma_buf) = transfer.wait();
        dma_tx_buf.replace(dma_buf);

        log::info!("Done");

        delay.delay_millis(1000);
    }
}
