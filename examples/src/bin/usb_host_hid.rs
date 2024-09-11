//! CDC-ACM serial port example using polling in a busy loop.
//!
//! This example should be built in release mode.
//!
//! The following wiring is assumed:
//! - DP => GPIO20
//! - DM => GPIO19

//% CHIPS: esp32s2 esp32s3

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    gpio::Io,
    otg_fs::{usbhost::UsbHostBus, Usb},
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;
use usbh::{
    driver::{
        kbd::{KbdDriver, KbdEvent, KbdLed},
        log::{EventMask, LogDriver},
    },
    PollResult,
    UsbHost,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

// static mut USB_DMA_MEMORY: [u32; 1024] = [0; 1024];
pub fn init_heap(allocator: &esp_alloc::EspHeap) {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: core::mem::MaybeUninit<[u8; HEAP_SIZE]> = core::mem::MaybeUninit::uninit();

    unsafe {
        allocator.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    init_heap(&ALLOCATOR);

    let peripherals = Peripherals::take();
    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    println!("Pre-hostbus init");
    let usbhost = UsbHostBus::new(peripherals.USB0, io.pins.gpio20, io.pins.gpio19);
    println!("Post-hostbus init, is_connected: {}", usbhost.is_connected());
    let mut usbhost = UsbHost::new(usbhost);
    println!("Post-ubhost init");
    let mut log_driver = LogDriver::new(EventMask::all());
    let mut kbd_driver = KbdDriver::<2>::new();

    let mut num_state: bool = false;

    loop {
        match usbhost.poll(&mut [&mut log_driver, &mut kbd_driver]) {
            PollResult::NoDevice => {
                continue;
            }
            v => {
                println!("usbh poll: {:?}", v);
            }
        }

        println!("Usb state: {:?}", usbhost.get_state());


        match kbd_driver.take_event() {
            None => {}
            Some(KbdEvent::ControlComplete(dev_addr)) => {
                println!("control complete: {:?}", dev_addr);
            }
            Some(KbdEvent::DeviceAdded(dev_addr)) => {
                println!("new dev: {:?}", dev_addr);
                // Keyboard with address dev_addr added
                kbd_driver.set_idle(dev_addr, 0, &mut usbhost).ok().unwrap();
            }
            Some(KbdEvent::DeviceRemoved(dev_addr)) => {
                // Keyboard with address dev_addr removed
                println!("removed dev: {:?}", dev_addr);
            }
            Some(KbdEvent::InputChanged(dev_addr, report)) => {
                // Input on keyboard

                println!("input: {:?}", report);

                // toggle Num lock LED when NumLock key is pressed
                if report.pressed_keys().any(|key| key == 83) {
                    num_state = !num_state;
                    if num_state {
                        println!("Numlock high");
                    } else {
                        println!("Numlock Low");
                    }
                    kbd_driver
                        .set_led(dev_addr, KbdLed::NumLock, num_state, &mut usbhost)
                        .ok()
                        .unwrap();
                }
            }
        }
    }
}
