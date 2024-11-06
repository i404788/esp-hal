//! Usb Host example using the embassy-usb host driver
//!
//! This example should be built in release mode.
//!
//! The following wiring is assumed:
//! - DP => GPIO20
//! - DM => GPIO19

//% CHIPS: esp32s2 esp32s3
//% FEATURES: embassy embassy-generic-timers usb-host defmt

#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    cpu_control::CpuControl,
    dma::*,
    dma_buffers,
    gpio::Io,
    otg_fs::asynch::host::{UsbHost, poll_usbhost},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::timg::TimerGroup,
};
use defmt::debug;
use embassy_executor::Spawner;
use embassy_usb::handlers::UsbHostHandler;
use embassy_usb::handlers::kbd::KbdHandler;
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::host::DeviceEvent::Connected;
use embassy_usb_driver::host::UsbHostDriver;
use esp_println::println;
use embassy_time::{Duration, Timer};

pub fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: core::mem::MaybeUninit<[u8; HEAP_SIZE]> = core::mem::MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}


// #[embassy_executor::task]
// async fn usb_softpoller() {
//     loop {
//         poll_usbhost();
//         Timer::after_millis(1).await;
//     }
// }


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    init_heap();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // #[cfg(feature = "log")]
    // esp_println::logger::init_logger(log::LevelFilter::Debug);

    println!("Pre-hostbus init");
    let mut usbhost = UsbHost::new(peripherals.USB0, io.pins.gpio20, io.pins.gpio19);

    // Temporary solution until interrupts are fixed
    // let _ = spawner.spawn(usb_softpoller());

    println!("Detecting device");
    // Wait for root-port to detect device
    let speed = loop {
        match usbhost.bus.wait_for_device_event().await {
            Connected(speed) => break speed,
            _ => {}
        }
    };

    println!("Found device with speed = {:?}", speed);

    let enum_info = usbhost.bus.enumerate_root(speed, 1).await.unwrap();
    Timer::after_micros(20).await;
    let mut kbd = KbdHandler::try_register(&usbhost.bus, enum_info).await.expect("Couldn't register keyboard");
    loop {
        let result = kbd.wait_for_event().await;
        debug!("{}", result);
    }
}
