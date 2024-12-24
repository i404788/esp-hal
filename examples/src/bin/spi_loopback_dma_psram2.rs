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
#![feature(vec_push_within_capacity)]

use esp_backtrace as _;
use esp_hal::macros::ram;
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

const DMA_ALIGNMENT: ExternalBurstConfig = ExternalBurstConfig::Size64;
const DMA_CHUNK_SIZE: usize = 4096 - DMA_ALIGNMENT as usize;
const DMA_BUFFER_SIZE: usize = 8192;

// #[ram]
fn effectively_empty(fail: bool) // -> bool
{
    const size: usize = 2;
    let mut x: alloc::vec::Vec<usize> = alloc::vec::Vec::with_capacity(size);
    // unsafe { x.set_len(size) };

    // let layout = core::alloc::Layout::new::<[usize; size]>();
    // let ptr = unsafe { alloc::alloc::alloc(layout) as *mut usize };
    // let mut lsize = 0;

    if fail {
        // for i in 0..size {
        //     // unsafe { *(ptr.add(i)) = i }
        //     unsafe { core::ptr::write(ptr.add(i), i) };
        //     let j = unsafe { *(ptr.add(i)) };
        //     lsize += j;
        // }
        for i in 0..size {
            // ram op
            // x.push(i);
            x.push_within_capacity(i);
            // x.get(i).unwrap_or(&0);
        }
    } else {
        for i in 0..size {
            // nop
            unsafe { core::ptr::read_volatile(SPI2::ptr().add(0x003C) as *const u32) };
        }
    }

    // x[size - 2] % 2 == 0
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Trace);
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

    log::info!(
        "Alignment: {}, Chunk size: {}",
        ExternalBurstConfig::Size64 as usize,
        DMA_CHUNK_SIZE
    );

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

    log::info!("First dmabuffer: {:?}", dma_tx_buf.as_mut());

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
        // delay.delay_micros(1000);
        // delay.delay_micros(1000);
        // delay.delay_micros(1000);
        // let transfer = spi.write(dma_buf.len(), dma_buf).unwrap();
        // log::info!("pre-SPI2: {}", unsafe {
        //     core::ptr::read_volatile(SPI2::ptr().add(0x003C) as *const u32)
        // });

        core::hint::black_box(effectively_empty(false));

        delay.delay_micros(1000);
        // delay.delay_millis(1000);

        log::info!("post-SPI2: {}", unsafe {
            core::ptr::read_volatile(SPI2::ptr().add(0x003C) as *const u32)
        });
        (spi, dma_buf) = transfer.wait();
        dma_tx_buf.replace(dma_buf);

        log::info!("Done");

        delay.delay_millis(1000);
    }
}
