//! # USB On-The-Go (USB OTG)
//!
//! ## Overview
//! The USB OTG Full-speed (FS) peripheral allows communication
//! with USB devices using either blocking (usb-device) or asynchronous
//! (embassy-usb) APIs.
//!
//! It can operate as either a USB Host or Device, and supports full-speed (FS)
//! and low-speed (LS) data rates of the USB 2.0 specification.
//!
//! The blocking driver uses the `esp_synopsys_usb_otg` crate, which provides
//! the `USB bus` implementation and `USB peripheral traits`.
//!
//! The asynchronous driver uses the `embassy_usb_synopsys_otg` crate, which
//! provides the `USB bus` and `USB device` implementations.
//!
//! The module also relies on other peripheral modules, such as `GPIO`,
//! `system`, and `clock control`, to configure and enable the `USB` peripheral.
//!
//! ## Configuration
//! To use the USB OTG Full-speed peripheral driver, you need to initialize the
//! peripheral and configure its settings. The [`Usb`] struct represents the USB
//! peripheral and requires the GPIO pins that implement [`UsbDp`], and
//! [`UsbDm`], which define the specific types used for USB pin selection.
//!
//! The returned `Usb` instance can be used with the `usb-device` crate, or it
//! can be further configured with [`asynch::Driver`] to be used with the
//! `embassy-usb` crate.
//!
//! ## Examples
//! Visit the [USB Serial] example for an example of using the USB OTG
//! peripheral.
//!
//! [USB Serial]: https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/usb_serial.rs
//!
//! ## Implementation State
//! - Low-speed (LS) is not supported.

pub use esp_synopsys_usb_otg::UsbBus;
use esp_synopsys_usb_otg::UsbPeripheral;
use core::marker::PhantomData;

use crate::{
    gpio::InputSignal, peripheral::{Peripheral, PeripheralRef}, peripherals, rom::ets_delay_us, system::{Peripheral as PeripheralEnable, PeripheralClockControl}
};


#[doc(hidden)]
pub trait UsbDp: crate::private::Sealed {}

#[doc(hidden)]
pub trait UsbDm: crate::private::Sealed {}



pub struct Usb<'d> {
    _usb0: PeripheralRef<'d, peripherals::USB0>,
}


impl<'d> Usb<'d> {
    pub fn new<P, M>(
        usb0: impl Peripheral<P = peripherals::USB0> + 'd,
        _usb_dp: impl Peripheral<P = P> + 'd,
        _usb_dm: impl Peripheral<P = M> + 'd,
    ) -> Self
    where
        P: UsbDp + Send + Sync,
        M: UsbDm + Send + Sync,
    {
        PeripheralClockControl::enable(PeripheralEnable::Usb);

        Self {
            _usb0: usb0.into_ref(),
        }
    }

    fn _enable() {
        unsafe {
            let usb_wrap = &*peripherals::USB_WRAP::PTR;
            usb_wrap.otg_conf().modify(|_, w| {
                   w.usb_pad_enable()
                    .set_bit()
                    .phy_sel()
                    .clear_bit()
                    .clk_en()
                    .set_bit()
                    .ahb_clk_force_on()
                    .set_bit()
                    .phy_clk_force_on()
                    .set_bit()
            });

            #[cfg(esp32s3)]
            {
                let rtc = &*peripherals::LPWR::PTR;
                rtc.usb_conf()
                    .modify(|_, w| 
                                w.sw_hw_usb_phy_sel().set_bit()
                                 .sw_usb_phy_sel().set_bit());
            }

            crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_IDDIG); // connected connector is mini-B side
            crate::gpio::connect_high_to_peripheral(InputSignal::USB_SRP_BVALID); // HIGH to force USB device mode
            crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_VBUSVALID); // receiving a valid Vbus from device
            crate::gpio::connect_low_to_peripheral(InputSignal::USB_OTG_AVALID);
        }
    }

    fn _disable() {
        // TODO
    }
}

unsafe impl<'d> Sync for Usb<'d> {}

unsafe impl<'d> UsbPeripheral for Usb<'d> {
    const REGISTERS: *const () = peripherals::USB0::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 256;
    const ENDPOINT_COUNT: usize = 5;

    fn enable() {
        Self::_enable();
    }

    fn ahb_frequency_hz(&self) -> u32 {
        // unused
        80_000_000
    }
}

#[cfg(feature = "usb-host")]
pub mod usbhost {
    extern crate alloc;

    use embassy_usb_synopsys_otg::
        otg_v1::{vals::{Dpid, Dspd, Eptyp, FrameListLen, Pfivl}, Otg}
    ;
    use procmacros::handler;
    use usbh::{bus::{Error, Event, HostBus, InterruptPipe}, types::ConnectionSpeed};

    use super::*;
    use crate::Cpu;

    // Up to a total of 4KB according to ESP32S3 TRM; however the fifo depth specified is 256*u32=1024=1KB
    // Here also specified in words (u32s)
    #[cfg(not(feature = "esp32p4"))]
    const OTG_FIFO_DEPTH: usize = 256;
    #[cfg(feature = "esp32p4")]
    const OTG_FIFO_DEPTH: usize = 1024;


    // Default FIFO Biasing (see https://github.com/espressif/esp-idf/blob/d7ca8b94c852052e3bc33292287ef4dd62c9eeb1/components/hal/usb_dwc_hal.c#L200)
    const TX_FIFO_WORDS: usize = OTG_FIFO_DEPTH/4;
    const PTX_FIFO_WORDS: usize = OTG_FIFO_DEPTH/8;
    const RX_FIFO_WORDS: usize = OTG_FIFO_DEPTH - PTX_FIFO_WORDS - TX_FIFO_WORDS;

    const RX_FIFO_SIZE: usize = RX_FIFO_WORDS*4;
    const TX_FIFO_SIZE: usize = TX_FIFO_WORDS*4; // NOTE: periodic not used yet but equivalent to this
    const PIPE_COUNT: usize = 8;
    const FRAME_LIST_LEN: FrameListLen = FrameListLen::LEN32;

    // struct PipeDescriptor {
    //     // TODO: generic or const size
    //     buffer: core::cell::UnsafeCell<core::mem::MaybeUninit<[u32; 64]>>,
    // }

    // impl PipeDescriptor {
    //     fn get_buffer_addr(&mut self) -> *mut u8 {
    //         self.buffer.get_mut().as_mut_ptr() as *mut u8
    //     }

    //     fn get_mut(&mut self) -> &mut [u8] { 
    //         unsafe { 
    //             let len = self.buffer.get_mut().assume_init_ref().len();
    //             core::slice::from_raw_parts_mut(self.get_buffer_addr(), len*4) 
    //         }
    //     }

    //     fn get(&self) -> &[u8] {
    //         unsafe { 
    //             let buf = (*self.buffer.get()).assume_init();
    //             core::slice::from_raw_parts(buf.as_ptr() as *const u8, buf.len()*4) 
    //         }
    //     }
    // }

    fn dma_alloc_buffer<T>(length: usize, align: usize) -> &'static mut [T] {
        let size = core::mem::size_of::<T>();
        let layout = core::alloc::Layout::from_size_align(size*length, align).unwrap();
        unsafe {
            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                error!("make_buffers: alloc failed");
                alloc::alloc::handle_alloc_error(layout);
            }
            core::slice::from_raw_parts_mut(ptr as *mut T, length)
        }
    }

    unsafe fn dma_dealloc_buffer<T>(buf: &mut [T], align: usize) {
        let size = core::mem::size_of::<T>();
        alloc::alloc::dealloc(buf.as_mut_ptr() as *mut u8, core::alloc::Layout::from_size_align(buf.len()*size, align).unwrap()); 
    }

    /// Expected to be 8 bytes
    #[repr(C)]
    struct QtdEntry {
        buffer_status: u32,
        buffer: *mut u8,
    }

    /// Shared DMA buffers used by the USB Host driver
    struct UsbHostBuffers {
        // FIXME: frame_list doesn't seem to be yet assigned to the right address
        frame_list: &'static mut [u32]
    }

    impl UsbHostBuffers {
        pub fn new_alloc() -> Self {
            UsbHostBuffers { frame_list: dma_alloc_buffer(FRAME_LIST_LEN.as_value() as usize, 512) }
        }
    }

    impl Drop for UsbHostBuffers {
        fn drop(&mut self) {
            unsafe {
                dma_dealloc_buffer(self.frame_list, 512);
            }
        }
    }

    /// Channel-specific buffers used by USB Host driver in scatter/gather mode
    #[cfg(feature = "usbh-sg")]
    pub struct UsbHostChannelBuffers {
        qtd_list: &'static mut [QtdEntry],
    }

    #[cfg(feature = "usbh-sg")]
    impl UsbHostChannelBuffers {
        pub fn new_alloc(ch_type: Eptyp) -> Self {
            let qtd_len = match ch_type {
                Eptyp::BULK => 2,
                Eptyp::CONTROL => 3,
                Eptyp::INTERRUPT => FRAME_LIST_LEN.as_value(),
                Eptyp::ISOCHRONOUS => 64
            } as usize;

            UsbHostChannelBuffers { qtd_list: dma_alloc_buffer(qtd_len, 512) }
        }
    }

    #[cfg(feature = "usbh-sg")]
    impl Drop for UsbHostChannelBuffers {
        fn drop(&mut self) {
            unsafe {
                dma_dealloc_buffer(self.qtd_list, 512);
            }
        }
    }

    // Channel-specific buffers used by USB Host driver in Buffer-DMA mode
    pub struct UsbHostChannelBuffers {
        buffer: &'static mut [u8],
    }

    impl UsbHostChannelBuffers {
        pub fn new_alloc(buffer_size: usize) -> Self {
            // TODO: unsure if 512 align is required here
            UsbHostChannelBuffers { buffer: dma_alloc_buffer(buffer_size, 512) }
        }
    }

    impl Default for UsbHostChannelBuffers {
        fn default() -> Self {
            UsbHostChannelBuffers::new_alloc(64)
        }
    }

    impl Drop for UsbHostChannelBuffers {
        fn drop(&mut self) {
            unsafe {
                dma_dealloc_buffer(self.buffer, 512);
            }
        }
    }
    
    pub struct UsbHostBus<'d> {
        regs: Otg,
        allocated_pipes: u16,
        txfifo_buf: [u8; TX_FIFO_SIZE],
        txfifo_usage: usize,

        dev_conn: bool,

        channel_buffers: [UsbHostChannelBuffers; PIPE_COUNT],
        buffers: UsbHostBuffers,
        _usb0: PeripheralRef<'d, peripherals::USB0>,
    }

    enum HcdCommand {
        PowerOn,
        Reset
    }

    impl<'d> UsbHostBus<'d> {
        pub fn new<P, M>(
            usb0: impl Peripheral<P = peripherals::USB0> + 'd,
            _usb_dp: impl Peripheral<P = P> + 'd,
            _usb_dm: impl Peripheral<P = M> + 'd,
        ) -> Self
        where
            P: UsbDp + Send + Sync,
            M: UsbDm + Send + Sync,
        {
            PeripheralClockControl::enable(PeripheralEnable::Usb);

            unsafe {
                let usb_wrap = &*peripherals::USB_WRAP::PTR;
                usb_wrap.otg_conf().modify(|_, w| {
                    w.usb_pad_enable().set_bit()
                        .phy_sel().clear_bit()
                        .clk_en().set_bit()
                        .ahb_clk_force_on().set_bit()
                        .phy_clk_force_on().set_bit()
                        .pad_pull_override().set_bit()
                        .dp_pulldown().set_bit()
                        .dm_pulldown().set_bit()
                        .dp_pullup().clear_bit()
                        .dm_pullup().clear_bit()
                });
    
                #[cfg(esp32s3)]
                {
                    let rtc = &*peripherals::LPWR::PTR;
                    rtc.usb_conf()
                        .modify(|_, w| 
                            w.sw_hw_usb_phy_sel().set_bit()
                             .sw_usb_phy_sel().set_bit());
                }

                crate::gpio::connect_low_to_peripheral(InputSignal::USB_SRP_BVALID); // HIGH to force USB device mode
                crate::gpio::connect_low_to_peripheral(InputSignal::USB_OTG_IDDIG); // Indicate A-Device by floating the ID pin
                crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_VBUSVALID);
                crate::gpio::connect_high_to_peripheral(InputSignal::USB_OTG_AVALID); // Assume valid A device
            }

            // We don't actually need to handle interrupts for usbh since it poll and checks the intr regs
            #[cfg(feature = "usbh-async")]
            unsafe {
                crate::interrupt::bind_interrupt(
                    crate::peripherals::Interrupt::USB,
                    interrupt_handler.handler(),
                );
                crate::interrupt::enable(
                    crate::peripherals::Interrupt::USB,
                    interrupt_handler.priority(),
                )
                .unwrap();
            }

            let regs: Otg = unsafe { Otg::from_ptr(Usb::REGISTERS.cast_mut()) };

            debug_assert!(regs.snpsid().read() == 0x4F54400A, "Bad core id, peripheral dead?");
            debug!("Core ID {}", regs.cid().read().product_id());

            // unsafe {
            //     crate::interrupt::bind_interrupt(
            //         crate::peripherals::Interrupt::USB,
            //         interrupt_handler.handler(),
            //     );
            //     crate::interrupt::enable(
            //         crate::peripherals::Interrupt::USB,
            //         interrupt_handler.priority(),
            //     )
            //     .unwrap();
            // }

            Self {
                _usb0: usb0.into_ref(),
                regs,
                
                // rxfifo_buf: [0u8; RX_FIFO_SIZE],
                // rxfifo_usage: 0,
                txfifo_buf: [0u8; TX_FIFO_SIZE],
                txfifo_usage: 0,

                dev_conn: false,
            
                channel_buffers: Default::default(),
                buffers: UsbHostBuffers::new_alloc(),
                allocated_pipes: 1, // Control pipe reserved
            }
        }


        fn hcd_cmd(&mut self, cmd: HcdCommand) {
            match cmd {
                HcdCommand::PowerOn => {
                    // TODO: check for not powered state

                    // Disable all channel interrupts
                    self.regs.haintmsk().write(|w| w.0 = !0);
                    self.regs.gintmsk().modify(|w| {
                        w.set_prtim(true);
                        w.set_hcim(true);
                    });
                    
                    // Power port
                    self.regs.hprt().modify(|w| {
                        w.0 &= !HPRT_W1C_MASK;
                        w.set_ppwr(true);
                    });

                },
                HcdCommand::Reset => {
                    self.reset_bus();
                }
            }
        }

        fn set_frameschedule(&mut self, channel: u8, interval: u8, offset: u8) {
            // Convert interval (ms) to interval_frame (fp/ms)
            const FRAMES_PER_MS: u32 = 12_000_000 / 1000 / 8 / 64;
            let mut interval_frame: u32 = (interval as u32 * FRAMES_PER_MS).max(1);
            let mut offset_frame: u32 = offset as u32 * FRAMES_PER_MS;

            // let interval_frame_list : u32 = ep_char->periodic.interval;
            // let offset_frame_list : u32= ep_char->periodic.offset;
            
            const IS_HS: bool = false; // defined per channel
            // Periodic Frame List works with USB frames. For HS endpoints we must divide interval[microframes] by 8 to get interval[frames]
            if IS_HS {
                interval_frame /= 8;
                offset_frame /= 8;
            }

            // Interval in Periodic Frame List must be power of 2.
            // This is not a HW restriction. It is just a lot easier to schedule channels like this.
            let frame_list_len = FRAME_LIST_LEN.as_value() as u32;
            if interval_frame > frame_list_len {
                interval_frame = frame_list_len;
            }
            // Quantize to lower power of 2 (i.e. 68 => 64, 42 => 32, etc)
            interval_frame = 1 << (u32::BITS - interval_frame.leading_zeros() - 1);

            // Schedule the channel in the frame list
            for i in (0..frame_list_len).step_by(interval_frame as usize) {
                let index = (offset_frame + i) % frame_list_len;
                self.buffers.frame_list[index as usize] |= 1 << channel;
            }

            // For HS endpoints we must write to sched_info field of HCTSIZ register to schedule microframes
            if IS_HS {
                // USB-OTG databook: Table 5-47 (only for HS)
                let sched_info = match interval_frame {
                    // 1 token per 8 microframes
                    8..=u32::MAX => 0b00000001 << (offset_frame % 8),
                    4..8 => 0b00010001 << (offset_frame % 4),
                    2..4 => 0b01010101 << (offset_frame % 2),
                    _ => 0xffu32,
                };
                self.regs.hctsiz(channel as usize).modify(|w| {
                    w.set_schedinfo(sched_info as u8);
                });
            }
        }

        fn is_hs_supported(&self) -> bool {
            self.regs.gccfg_v2().read().phyhsen()
        }

        fn init_fifo(&mut self) {
            debug!("init_fifo");
            debug!("configuring rx fifo size={}", RX_FIFO_WORDS);
            self.regs.grxfsiz().modify(|w| w.set_rxfd(RX_FIFO_WORDS as u16));
            // Configure TX (USB in direction) fifo size for each endpoint
            let mut fifo_top = RX_FIFO_WORDS;

            debug!(
                "configuring tx fifo, offset={}, size={}",
                fifo_top,
                TX_FIFO_WORDS
            );
            // Non-periodic tx fifo
            self.regs.hnptxfsiz().write(|w| {
                w.set_fd(TX_FIFO_WORDS as u16);
                w.set_sa(fifo_top as u16);
            });
            fifo_top += TX_FIFO_WORDS;

            // Periodic tx fifo
            self.regs.hptxfsiz().write(|w| {
                w.set_fd(PTX_FIFO_WORDS as u16);
                w.set_sa(fifo_top as u16);
            });
            fifo_top += PTX_FIFO_WORDS;

            debug_assert!(fifo_top <= OTG_FIFO_DEPTH, "Exceeds maximum fifo allocation");

            // Flush fifos (TX & PTX need to be done separately since txfnum is an indicator of which)
            self.regs.grstctl().write(|w| {
                w.set_rxfflsh(true);
                w.set_txfflsh(true);
                w.set_txfnum(0x1);
            });
            loop {
                let x = self.regs.grstctl().read();
                if !x.rxfflsh() && !x.txfflsh() {
                    break;
                }
            }
            self.regs.grstctl().write(|w| {
                w.set_txfflsh(true);
                w.set_txfnum(0x0);
            });
            while self.regs.grstctl().read().txfflsh() {}
            
        }

        fn read_tx(&mut self, data_len: usize, pid: Dpid, epnum: u8, pipe_ref: usize) {
            debug_assert!(pipe_ref <= PIPE_COUNT);
            debug!("read_tx: {}", data_len);

            let max_packet_size: usize = RX_FIFO_SIZE;
            self.regs.hcchar(pipe_ref).write(|w| {
                w.set_lsdev(false); // LS unsupported
                w.set_epnum(epnum); // Endpoint nr
                w.set_mpsiz(max_packet_size as u16); // Packet size
                w.set_epdir(true); // ENDPOINT_TYPE; IN
                w.set_mcnt(1); // Packets per frame
            });

            let transfer_size: u32 = data_len as u32; // sizeof(SetupPacket)
            self.regs.hctsiz(pipe_ref).write(|w| {
                w.set_xfrsiz(transfer_size); // Transfer size
                w.set_dpid(pid.into());
                w.set_pktcnt(transfer_size.div_ceil(max_packet_size as u32).max(1) as u16);
            });

            self.regs.hcsplt(pipe_ref).write_value(0); 


            assert!(data_len <= self.channel_buffers[pipe_ref].buffer.len(), "Expected transfer_size to be smaller than the transfer");

            // NOTE: Qtd is scatter/gather dma, while this is buffer-dma
            // FIXME: unsure which of hcdma or hcdmab is the actual address; just try both
            #[cfg(not(feature = "usbh-fifo"))]
            {
                self.regs.hcdma(pipe_ref).write(|w| w.0 = self.channel_buffers[pipe_ref].buffer.as_ptr() as u32);
                self.regs.hcdmab(pipe_ref).write_value(self.channel_buffers[pipe_ref].buffer.as_ptr() as u32);
            }
            
            // if IN direction, adjust RX fifo to be eq or larger than requested data
            #[cfg(not(feature = "usbh-dfifo"))]
            debug_assert!(data_len < RX_FIFO_SIZE);

            #[cfg(feature = "usbh-dfifo")]
            if !dir {
                // Calculate the RX FIFO size according to minimum recommendations from reference manual
                // RxFIFO = (5 * number of control endpoints + 8) +
                //          ((largest USB packet used / 4) + 1 for status information) +
                //          (2 * number of OUT endpoints) + 1 for Global NAK
                // with number of control endpoints = 1 we have
                // RxFIFO = 15 + (largest USB packet used / 4) + 2 * number of OUT endpoints
                // we double the largest USB packet size to be able to hold up to 2 packets
                #[inline]
                fn calc_grxfsiz(max_ep_size: u16, ep_count: u16) -> u16 {
                    return 15 + 2 * (max_ep_size / 4) + 2 * ep_count;
                }

                // Calculate required size of RX FIFO
                let sz = calc_grxfsiz(transfer_size+1, ep_count);

                // TODO: assert we don't run out of fifo space
                // if (dwc2->grxfsiz < sz) {
                // TU_ASSERT(sz + _allocated_fifo_words_tx <= _dwc2_controller[rhport].ep_fifo_size / 4);

                // Enlarge RX FIFO if needed
                self.regs.grxfsiz().modify(|w| w.set_rxfd(sz.max(w.rxfd())));
            }

            // Start: dwc_channel_start_transaction(chan, req);
            // Clear interrupts
            self.regs.hcint(pipe_ref).write(|w| w.0 = 0xffffffff);
            self.regs.hcintmsk(pipe_ref).write(|w| w.0 = 0);

            let frame_nr = self.regs.hfnum().read().frnum();
            self.regs.hcchar(pipe_ref).write(|w| {
                // Set odd-frame and enable channel
                w.set_oddfrm(frame_nr & 1 == 0);
                w.set_chena(true);
            });

            // Re-enable channel halt
            self.regs.hcintmsk(pipe_ref).write(|w| {
                w.set_chhm(true);
            });

            // Enable channel interrupts
            self.regs.haintmsk().modify(|w| {
                w.set_haintm(w.haintm() | (1 << pipe_ref));
            });
        }

        // SAFETY: in dma mode (not(usbh-fifo)): data slice needs to exists at least as long as UsbHostBus itself; no local slices
        unsafe fn write_tx(&self, data: &[u8], pid: Dpid, epnum: u8, pipe_ref: usize) {
            debug_assert!(pipe_ref <= PIPE_COUNT);
            debug_assert!(data.len() < TX_FIFO_SIZE);
            debug!("write_tx: {:?} pid: {}, epnum: {}, pipe: {}", data, pid as u8, epnum, pipe_ref);

            let max_packet_size: usize = TX_FIFO_SIZE;

            // NOTE: ep0 supports both directions but only one packet at a time, so multiple writes might be needed to finish a transfer
            // NOTE: usbh only supports single-channel operation for non-pipe data in/out it seems
            // No endpoint (control xfer)
            self.regs.hcchar(pipe_ref).write(|w| {
                w.set_lsdev(false); // LS unsupported
                w.set_epnum(epnum); // Endpoint nr
                w.set_mpsiz(max_packet_size as u16); // Packet size
                w.set_epdir(false); // ENDPOINT_TYPE; OUT
                w.set_mcnt(1); // Packets per frame
            });

            let transfer_size: u32 = data.len() as u32; // sizeof(SetupPacket)
            self.regs.hctsiz(pipe_ref).modify(|w| {
                w.set_xfrsiz(transfer_size); // Transfer size
                w.set_dpid(pid.into());
                w.set_pktcnt(transfer_size.div_ceil(max_packet_size as u32).max(1) as u16);
            });

            self.regs.hcsplt(pipe_ref).write_value(0); 
            // Unsure how to use split_control, seems like it's always enabled for LS/FS in xinu but requires emulating a hub :(
            //  https://github.com/xinu-os/xinu/blob/30777a80f2ebdc47b854f9434bfed6e3569edd2e/system/platforms/arm-rpi/usb_dwc_hcd.c#L959
            // self.regs.hcsplt(pipe_ref).write(|w| {
            //     // Layout: port_addr u7, hub_addr: u7, tx_pos: u2, complete_split: b, reserved: 14, split_en: b
            //     w.
            // });

            // Write all data to fifo or set dma address
            #[cfg(not(feature = "usbh-fifo"))]
            {
                self.regs.hcdma(pipe_ref).write(|w| w.0 = data.as_ptr() as u32);
                self.regs.hcdmab(pipe_ref).write_value(data.as_ptr() as u32);
            }

            #[cfg(feature = "usbh-fifo")]
            {
                let data_iter = self.txfifo_buf.chunks_exact(4);
                for word in data_iter {
                    self.regs.fifo(pipe_ref).write_value(regs::Fifo(u32::from_ne_bytes(word.try_into().unwrap())));
                }

                let remainder = data_iter.remainder();
                if remainder.len() > 0 {
                    let mut remainder_buf: [u8; 4] = [0u8; 4];
                    remainder_buf[..remainder.len()].copy_from_slice(remainder);
                    self.regs.fifo(pipe_ref).write_value(regs::Fifo(u32::from_ne_bytes(remainder_buf)));
                }
            }

            // Start: dwc_channel_start_transaction(chan, req);
            // Clear interrupts
            self.regs.hcint(pipe_ref).write(|w| w.0 = 0xffffffff);
            self.regs.hcintmsk(pipe_ref).write(|w| w.0 = 0);

            let frame_nr = self.regs.hfnum().read().frnum();
            self.regs.hcchar(pipe_ref).write(|w| {
                // Set odd-frame and enable channel
                w.set_oddfrm(frame_nr & 1 == 0);
                w.set_chena(true);
            });

            // Re-enable channel halt
            self.regs.hcintmsk(pipe_ref).write(|w| {
                w.set_chhm(true);
            });

            // Enable channel interrupts
            self.regs.haintmsk().write(|w| w.set_haintm(0xffffu16));
            // self.regs.haintmsk().modify(|w| {
            //     w.set_haintm(w.haintm() | (1 << pipe_ref));
            // });

            self.regs.gintmsk().modify(|w| {
                w.set_hcim(true);
            });
        }

        fn alloc_channel(&mut self, size: u16) -> Option<InterruptPipe> {
            if size > 64 {
                return None;
            }
            let ap = self.allocated_pipes;
            let free_index = (0..PIPE_COUNT).find(|i| ap & (1 << i) == 0)? as u8;

            // dwc2 has no intrinsic backing buffer, we will use the DMA feature to utilize our own buffers
            let ptr = self.channel_buffers[free_index as usize].buffer.as_mut_ptr();

            // self.pipe_info[free_index as usize].0 = size;
            self.allocated_pipes |= 1 << free_index;

            Some(InterruptPipe {
                bus_ref: free_index,
                ptr,
            })
        }

        fn set_port_defaults(&mut self) {
            // Not using descriptor DMA mode
            self.regs.hcfg().modify(|w| {
                w.set_descdma(false);
                w.set_perschedena(false);
            });
            let hprt = self.regs.hprt().read();
            self.regs.hfir().modify(|w| {
                w.set_rldctrl(false);
                w.set_frivl(match hprt.pspd() {
                    1 => 48000,
                    2 => 6000,
                    _ => unreachable!()
                })
            });
        }

        fn release_channel(&mut self, pipe: InterruptPipe) {
            self.allocated_pipes &= !(1 << pipe.bus_ref);
        }

        pub fn is_connected(&self) -> bool {
            self.regs.hprt().read().pcdet()
        }

        fn set_global_interrupt(&mut self, enable: bool) {
            self.regs.gahbcfg().modify(|w| {
                w.set_gint(enable); // unmask global interrupt
            });
        }
    }

    #[cfg(feature = "usbh-async")]
    fn handle_channel_interrupt(ch: usize) {
        // https://github.com/xinu-os/xinu/blob/30777a80f2ebdc47b854f9434bfed6e3569edd2e/system/platforms/arm-rpi/usb_dwc_hcd.c#L1417
    }

    // #[cfg(feature = "usbh-async")]
    #[handler(priority = crate::interrupt::Priority::min())]
    fn interrupt_handler() {
        // SAFETY: not allowed to be re-entrant
        // TODO: atomic latch or mutex-try?

        // let regs: Otg = unsafe { Otg::from_ptr(Usb::REGISTERS.cast_mut()) };
        // let intr = regs.gintsts().read();
        // // debug!("{}", intr.0);
        // regs.gintsts().modify(|w| {});

        // if intr.hcint() {
        //     // Channel has pending interrupt, handle them each individually
            
        //     // let mut chintr = regs.haint().read().haint();
        //     // while chintr != 0 {
        //     //     let idx = chintr.leading_zeros();
        //     //     // todo!("handle_channel_interrupt(idx)");
        //     //     chintr ^= idx;
        //     // }
        // }

        // if intr.hprtint() {
        //     // https://github.com/xinu-os/xinu/blob/30777a80f2ebdc47b854f9434bfed6e3569edd2e/system/platforms/arm-rpi/usb_dwc_hcd.c#L1629
        //     todo!("Update internal port state from the host_port_ctrlstatus reg (if needed)");
        // }
        // let setup_late_cnak = quirk_setup_late_cnak();

        // unsafe {
        //     on_interrupt(
        //         Driver::REGISTERS,
        //         &STATE,
        //         Usb::ENDPOINT_COUNT,
        //         setup_late_cnak,
        //     )
        // }
    }

    impl<'d> Drop for UsbHostBus<'d> {
        fn drop(&mut self) {
            crate::interrupt::disable(Cpu::ProCpu, crate::peripherals::Interrupt::USB);

            #[cfg(multi_core)]
            crate::interrupt::disable(Cpu::AppCpu, crate::peripherals::Interrupt::USB);

            self.regs.gintmsk().write_value(embassy_usb_synopsys_otg::otg_v1::regs::Gintmsk(0xffff_ffff));
            self.set_global_interrupt(false);
        }
    }

    const HPRT_W1C_MASK: u32 = 0x2E; // Mask of interrupts inside HPRT; used to avoid eating interrupts (e.g. SOF)

    /// The implementation of UsbHostBus is a bit complicated since usbh is originally meant for rp2040 which uses a different USB core
    /// There are 2 types of transfers each of which are mapped differently onto the synopsys dwc2 core:
    /// * InterruptPipe: these are mapped onto channels 1..CH_MAX
    /// * fn *_data_*: these are mapped onto channel 0
    /// 
    /// This mapping is not disimilar to RP2040 impl however buffers are now 'double-backed' so data is first stored on the 
    ///     Heap/Stack and then transferred into the TX FIFO; or on poll() transferred from FIFO into Heap/Stack (at which point Event::InterruptPipe can be given)
    impl<'d> HostBus for UsbHostBus<'d> {
        fn reset_controller(&mut self) {

            self.regs.dcfg().write(|w| {
                w.set_pfivl(Pfivl::FRAME_INTERVAL_80);
                w.set_dad(Dspd::FULL_SPEED_INTERNAL as u8);
            });
            
            // Set host config
            self.regs.gusbcfg().write(|w| {
                w.set_fhmod(true); // Force host mode
                w.set_fdmod(false); // Deassert device mode
                w.set_srpcap(false);
                w.set_hnpcap(false);
                // Enable internal full-speed PHY
                w.set_physel(true);
                w.set_trdt(5); // Maximum 
                w.set_tocal(7); // Maximum timeout calibration 
            });

            self.regs.gccfg_v1().modify(|w| {
                w.set_pwrdwn(true); // PwrDown inverted
            });

            self.regs.gccfg_v1().modify(|w| {
                const VBUSDETECT: bool = false;
                w.set_novbussens(!VBUSDETECT);
                w.set_vbusasen(false);
                w.set_vbusbsen(VBUSDETECT);
                w.set_sofouten(false);
            });

            // FIXME: should this really be before host config?
            // Perform core soft-reset
            self.regs.grstctl().modify(|w| w.set_csrst(true));

            debug!("Pre-reset");
            while self.regs.grstctl().read().csrst() {}
            while !self.regs.grstctl().read().ahbidl() {}
            debug!("Post-reset");

            self.allocated_pipes = 1;
            self.txfifo_usage = 0;

            self.regs.pcgcctl().modify(|w| {
                // Disable power down
                w.set_stppclk(false);
            });

            self.init_fifo();

            // self.regs.gintmsk().write_value(embassy_usb_synopsys_otg::otg_v1::regs::Gintmsk(0x0));
            // TODO: not getting any interrupts check esp-idf for interrupt enable switches
            
            self.regs.hprt().write(|w| {
                w.set_pspd(Dspd::FULL_SPEED_INTERNAL as u8);
                w.set_pena(false);
            });
            self.regs.gahbcfg().write(|w| {
                w.set_dmaen(true);
                w.set_hbstlen(0);
            });

            // Enable main host interrupts
            self.regs.gintmsk().write(|w| {
                w.set_discint(true);
                w.set_usbrst(true);
                w.set_hcim(true);
                w.set_prtim(true);
                w.set_enumdnem(true);
            });
            // Clear interrupts
            self.regs.gintsts().write(|w| w.0 = 0);
            self.set_global_interrupt(true);

            self.hcd_cmd(HcdCommand::PowerOn);

            // Should generate port reset interrupt
            let hprt = self.regs.hprt().read();
            debug!("setup [hprtint]: detected: {} (status: {})", hprt.pcdet(), hprt.pcsts());
            debug!("setup [hprtint]: port enable change: {}, port enable: {}, line: {}", hprt.penchng(), hprt.pena(), hprt.plsts());
            // self.reset_bus();
            debug!("Global interrupts: {}", self.regs.gintsts().read().0);
        }

        fn reset_bus(&mut self) {
            debug!("Resetting HostBus");
            self.regs.hprt().modify(|w| {
                // Port reset
                w.0 &= !HPRT_W1C_MASK;
                w.set_prst(true);
            });

            crate::rom::ets_delay_us(30_000);
            self.regs.hprt().modify(|w| {
                w.0 &= !HPRT_W1C_MASK;
                w.set_prst(false);
            });
            crate::rom::ets_delay_us(30_000);

            let hprt = self.regs.hprt().read();
            if !hprt.pena() && !hprt.pcdet() {
                debug!("Warning: Reset doesn't seem sucessful pena: {}, pcdet: {}", hprt.pena(), hprt.pcdet());
            }

            self.dev_conn = false;

            // FIXME: need to re-init fifo, framelist & periodic sched

            //  According to xinu it needs to be cleared after 60s (https://github.com/xinu-os/xinu/blob/30777a80f2ebdc47b854f9434bfed6e3569edd2e/system/platforms/arm-rpi/usb_dwc_hcd.c#L332)
            // According to ERZ32, only 10 ms is needed; and USB_HPRT.PRTENCHNG interrupt detects finish (https://www.silabs.com/documents/public/reference-manuals/EZR32WG-RM.pdf)
        }

        fn enable_sof(&mut self) {
            self.regs.gccfg_v1().modify(|w| { w.set_sofouten(false) });
        }

        fn sof_enabled(&self) -> bool {
            self.regs.gccfg_v1().read().sofouten()
        }

        fn set_recipient(
            &mut self,
            dev_addr: Option<usbh::types::DeviceAddress>,
            endpoint: u8,
            transfer_type: usbh::types::TransferType,
        ) {

            let channel = match transfer_type {
                usbh::types::TransferType::Control => 0,
                _ => 1 // TODO: pop from available
            };

            // NOTE: TRM says 8 channels available; first pipe is control packets, remaining 7 are bulk/interrupt/isochronuous
            self.regs.hcchar(channel).write(|w| { 
                w.set_dad(dev_addr.map(u8::from).unwrap_or(0));
                w.set_epnum(endpoint);
                w.set_eptyp((transfer_type as u8).into());
                
                // w.set_chena(true); // Enable channel
            });
        }

        fn write_setup(&mut self, setup: usbh::types::SetupPacket) {
            debug!("write_setup: rt={}, r={}, v={} i={} len={}", setup.request_type, setup.request, setup.value, setup.index, setup.length);
            
            let mut data = [0u8; 8];
            data[0] = setup.request_type;
            data[1] = setup.request;
            // TODO: is NE correct?
            data[2..4].copy_from_slice(&setup.value.to_ne_bytes());
            data[4..6].copy_from_slice(&setup.index.to_ne_bytes());
            data[6..8].copy_from_slice(&setup.length.to_ne_bytes());

            #[cfg(not(feature = "usbh-fifo"))]
            {
                self.channel_buffers[0].buffer[..8].copy_from_slice(&data);

                // SAFETY: dma_buf lives as long as Self
                unsafe { self.write_tx(&self.channel_buffers[0].buffer[..8], Dpid::MDATA, 0,0) }
            }
            
            #[cfg(feature = "usbh-fifo")]
            // SAFETY: with usbh-fifo data gets copied
            unsafe {self.write_tx(&data, Dpid::MDATA, 0,0) };
        }

        fn write_data_in(&mut self, length: u16, pid: bool) {
            debug!("write_data_in: {} pid={}", length, pid);
            let pid = if pid { Dpid::DATA1 } else { Dpid::DATA0 };
            self.read_tx(length as usize,  pid, 0, 0);
        }

        fn prepare_data_out(&mut self, data: &[u8]) {
            debug!("prepare_data_out: {}", data.len());
            // WARN: we cannot actually write to FIFO because in DWC2 setup is another transfer (see https://github.com/xinu-os/xinu/blob/30777a80f2ebdc47b854f9434bfed6e3569edd2e/system/platforms/arm-rpi/usb_dwc_hcd.c#L870)
            //  so we'll need to fake it (see usbh/lib.rs#control_out)
            // UPD: In theory we might be able to do a prepare_data_out actual by holding back the channel enable; however I'm unsure on the exact interaction with the fifo
            //  I'm assuming here that when a channel is enabled it will `pop` the fifo until it has enough data to complete the transaction (as specified in hctsiz)
            //  in which case as long as the fifo insertion ordering and the channel_en ordering are the same we can schedule OUT transactions (that fit in fifo)
            debug_assert!(self.txfifo_usage == 0, "Expected prepare_data_out to be called only once before write_data_out_prepared; driver trying to stream data?");
            self.txfifo_buf[..data.len()].copy_from_slice(data);
            self.txfifo_usage += data.len();
        }

        fn write_data_out_prepared(&mut self) {
            debug!("Write_data_out_prepared: {}", self.txfifo_usage);
            const CNTRL_CH: u8 = 0;

            // SAFETY: txfifo_buf lives as long as Self
            unsafe { self.write_tx(&self.txfifo_buf[..self.txfifo_usage], Dpid::DATA1, CNTRL_CH, 0); }
            self.txfifo_usage = 0;
        }

        fn poll(&mut self) -> Option<usbh::bus::Event> {
            // WARN: to clear an interrupt write a 1
            let mut intr = self.regs.gintsts().read();
            let hprt: embassy_usb_synopsys_otg::otg_v1::regs::Hprt = self.regs.hprt().read();

            if intr.hcint() || intr.discint() || hprt.pcdet() || hprt.penchng() || hprt.pocchng() {
                self.regs.gintsts().write(|_|{});

                self.regs.hprt().modify(|w| {
                    w.0 &= !HPRT_W1C_MASK;
                    w.set_pcdet(false);
                    w.set_pcsts(false);
                    w.set_penchng(false);
                    w.set_pocchng(false);
                }); // Clear interrupts of hrpt

                debug!("[hprtint]: detected: {} (status: {}), discint: {}", hprt.pcdet(), hprt.pcsts(), intr.discint());
                debug!("[hprtint]: port enable change: {}, port enable: {}, line: {}", hprt.penchng(), hprt.pena(), hprt.plsts());
                

                // The followning checks are mutually exclusive and have a priority ordering as shown
                if intr.discint() {
                    self.regs.gintsts().write(|w|w.set_discint(true));
                    return Some(Event::Detached);
                }

                if  hprt.pocchng() {
                    // May be overcurrenting
                    todo!("Disable port if hprt.overcur");
                }

                // TODO: validate dspd is eqv. to pspd
                #[cfg(fixme)] // Not reliable
                if hprt.penchng() {
                    if hprt.pena(){
                        // Port has become enabled, if a device is attached it should be reset
                        self.set_port_defaults();
                        // self.hcd_cmd(HcdCommand::Reset);
                        return None;
                    } 
                    
                    if !hprt.pena() {
                        // Port became disabled
                        // TODO: check if resetting (if so ignore)
                        return Some(Event::Detached);
                    }
                }

                // NOTE: this should be hprt.pcdet() but it doesn't detect reliably (may have to do with `pena`)
                // NOTE: usbh expects Attached(_) after each reset
                if hprt.pcsts() && !self.dev_conn{
                    crate::rom::ets_delay_us(30_000);
                    let hprt = self.regs.hprt().read();
                    if hprt.pcsts() {
                        let speed = match hprt.pspd() {
                            // 0 => Dspd::HIGH_SPEED,
                            1 => ConnectionSpeed::Full,
                            2 => ConnectionSpeed::Low,
                            _ => unreachable!()
                        };
                        // self.regs.hprt().modify(|w| w.set_pena(true));
                        self.set_port_defaults();
                        debug!("Got device attached speed={:?}", hprt.pspd());
                        self.dev_conn = true;
                        return Some(Event::Attached(speed));
                    } else {
                        return Some(Event::Detached);
                    }
                }
            }

            // Unsure if this is the correct one (also have non-interrupt port resume)
            if intr.wkupint() {
                debug!("wakeup, why don't you put on a little makup");
                intr.set_wkupint(true);
                self.regs.gintsts().write_value(intr);
                return Some(Event::Resume);
            }

            let gccfv1 = self.regs.gccfg_v1().read();
            let hfnum = self.regs.hfnum().read();
            debug!("sofen: {}, hfnum: {}", gccfv1.sofouten(), hfnum.frnum());
            self.regs.gccfg_v1().modify(|w| w.set_sofouten(true));
            // Notify SOF was sent
            if intr.sof() {
                debug!("[sof]");
                intr.set_sof(true);
                self.regs.gintsts().write_value(intr);
                return Some(Event::Sof);
            }

            if intr.hcint() {
                debug!("[hcint]");
                // Channel has pending interrupt, handle them each individually
                let mut chintr = self.regs.haint().read().haint();
                while chintr != 0 {
                    let idx = chintr.leading_zeros() as usize;

                    let mut hcintr = self.regs.hcint(idx).read();
                    if hcintr.stall() {
                        hcintr.set_stall(true);
                        self.regs.hcint(idx).write_value(hcintr);
                        return Some(Event::Stall);
                    }
    
                    if hcintr.txerr() || hcintr.bberr() {
                        // hcintr.dterr() should only for OUT
                        hcintr.set_txerr(true);
                        hcintr.set_bberr(true);
                        self.regs.hcint(idx).write_value(hcintr);
                        return Some(Event::Error(Error::Crc));
                    }

                    // Xinu auto-assumes halt if no error should we not check                    
                    assert!(hcintr.chh(), "Expected channel to be halted; maybe some unknown error occurred?");
                    
                    // DMA-case: if ch.dir == IN; copy DMA

                    // OUT-case: check current out q vs transfer.packet_count (i.e. remaining)
                    //  Assume chunk behaviour (i.e. non-tail packets are max-size)

                    // TODO: for split_control or if we use transfer_size lower than desired we'll need to catch this in the request https://github.com/xinu-os/xinu/blob/30777a80f2ebdc47b854f9434bfed6e3569edd2e/system/platforms/arm-rpi/usb_dwc_hcd.c#L1314C16-L1314C38
                    
                    // TODO: validate edge(chanptr->transfer.packet_count == 0): Event::TransferComplete
                    if hcintr.xfrc() {
                        if idx == 0 {
                            // Channel 0: control

                            // From xinu it seems like hctsize is updated from both sides but I'm unsure (this would make sense for dpid behaviour as well)
                            // assert!(self.regs.hctsiz(idx).read().pktcnt() == 0, "Expected all packets to be transmitted");
                            hcintr.set_xfrc(true);
                            self.regs.hcint(idx).write_value(hcintr);
                            return Some(Event::TransComplete);
                        } else {
                            // Channel 1..16: interrupt pipes (assumed?)
                            hcintr.set_xfrc(true);
                            self.regs.hcint(idx).write_value(hcintr);
                            // TODO: pull data here or in recevied_data?
                            return Some(Event::InterruptPipe(idx as u8));
                        }
                    }

                    chintr ^= 1 << idx as u16;
                }
            }

            // NOTE: there is no CRC error in dwc2, it probably fall under babble/transaction/excess_transaction/data_toggle/etc error :shrug:

            if intr.discint() {
                debug!("[discint]: disconnect");
            }

            if intr.mmis() {
                debug!("[mmisc]: Mode mismatch");
            }

            if intr.srqint() {
                debug!("[srqint]: new session");
            }

            if intr.usbrst() {
                debug!("[usbrst]: Usb reset");
            }

            if intr.enumdne() {
                debug!("[enumdne]: Enumeration done");
            }

            if intr.eopf() {
                debug!("[eopf]: End of periodic frame interrupt");
            }

            if intr.mmis() {
                debug!("[mmis]: Mode mismatch");
            }

            if intr.usbsusp() {
                debug!("[usbsusp]: usb suspend");
            }

            if intr.cidschg() {
                debug!("[cidschg]: Connector ID status change");
            }

            if intr.datafsusp() {
                debug!("[datafsusp]: Data fetch suspended");
            }

            if intr.eopf() {
                debug!("[eopf]: End of period interval frame");
            }

            debug!("[intr]: {}", intr.0);
            // if intr.0 != 0 {
            //     debug!("[intr]: {}", intr.0);
            //     // Write of eq value should clear
            //     // self.regs.gintsts().modify(|w| {});
            // }
            self.regs.gintsts().write(|_|{});

            None
        }

        fn pipe_continue(&mut self, pipe_index: u8) {
            // FIXME: unsure exactly how this is expected to be handled, 
            //  but given that we are talking about interrupt pipes maybe disabling the channel or unscheduling of the channel would be enough?
            // self.pipe_info[pipe_index as usize].0 = false;
        }

        fn create_interrupt_pipe(
            &mut self,
            device_address: usbh::types::DeviceAddress,
            endpoint_number: u8,
            direction: usbh::UsbDirection,
            size: u16,
            interval: u8,
        ) -> Option<InterruptPipe> {
            debug!("Creating new interrupt pipe {}", size);
            let pipe = self.alloc_channel(size)?;
            self.regs.hcchar(pipe.bus_ref.into()).write(|w| {
                w.set_eptyp(Eptyp::INTERRUPT);
                w.set_epnum(endpoint_number);
                w.set_epdir(direction as u8 == 1);
                w.set_dad(device_address.into());
            });

            self.regs.hctsiz(pipe.bus_ref.into()).modify(|w| {
                w.set_dpid(0); // Initial PID is 0
                w.set_doping(false); // Don't ping
                w.set_schedinfo(0xFF);
                w.set_ntdl(0);
            });

            // [XINU]:
            // >  However, due to design flaws in
            //  * this hardware and in USB 2.0 itself, "interrupt" and "isochronous" transfers
            //  * still need to make use of software polling when checking for new data, even
            //  * though each individual transfer is itself interrupt-driven.  This means that,
            //  * for example, if your USB mouse specifies a polling rate of 100 times per
            //  * second, then it will, unfortunately, be polled 100 times per second in
            //  * software. 

            // ESP-IDF disagress and does use a hardware scheduler (as found in hcfg: https://github.com/espressif/esp-idf/blob/d7ca8b94c852052e3bc33292287ef4dd62c9eeb1/components/soc/esp32s2/include/soc/usb_dwc_struct.h#L377-L378)
            // However the calc seems complex: https://github.com/espressif/esp-idf/blob/d7ca8b94c852052e3bc33292287ef4dd62c9eeb1/components/hal/usb_dwc_hal.c#L338
            
            self.regs.hcfg().write(|w| {
                w.set_frlistlen(FRAME_LIST_LEN);
                w.set_perschedena(true);
            });
            // NOTE: interval incoming is in ms-1, while interval for dwc2 is specified in frames
            // This will be handled in poll()
            self.set_frameschedule(pipe.bus_ref, interval, 0);
            Some(pipe)
        }

        fn release_interrupt_pipe(&mut self, pipe: InterruptPipe) {
            let pipe_ref = pipe.bus_ref as usize;
            assert!(pipe_ref <= PIPE_COUNT);
            // Set channel to inactive
            self.regs.hcchar(pipe_ref).write(|w| { w.set_chdis(true); w.set_chena(false); });

            // Disable channel interrupts
            self.regs.hcint(pipe_ref).write(|w| w.0 = 0xffffffff);
            self.regs.hcintmsk(pipe_ref).write(|w| w.0 = 0);
            self.regs.haintmsk().modify(|w| {
                w.set_haintm(w.haintm() & !(1 << pipe_ref));
            });
            self.release_channel(pipe);
        }

        fn interrupt_on_sof(&mut self, enable: bool) {
            // Clear any pending sof interrupt
            self.regs.gintsts().write(|w| w.set_sof(true));
            // Enables/Disables SOF event triggered by interrupt
            self.regs.gintmsk().modify(|w| w.set_sofm(!enable));
        }
        
        fn ls_preamble(&mut self, enabled: bool) {
            // Enable/disable preamble
            unimplemented!("Is not supported (I think) on ESP32S3; also not used by drivers");
        }
        
        fn stop_transaction(&mut self) {
            // Used as reset condition on rxtimeout; not supported by synposys
        }
        
        fn received_data(&self, length: usize) -> &[u8] {
            // Received data from a Control DATA IN req (no need to impl pipe_continue)

            #[cfg(not(feature = "usbh-fifo"))]
            {
                &self.channel_buffers[0].buffer[..length]
            }


            #[cfg(feature = "usbh-fifo")]
            {
                // FIXME: this should be moved to poll() since we don't have a mutable reference here
                assert!(length < self.rxfifo_buf.len());

                // Ceil length/4 to include partial bytes in last fifo read
                let fifo_len = length.div_ceil(4) * 4;
                
                for i in (0..fifo_len).step_by(4) {
                    // According to TinyUSB rx should be fifo0, need to if check tx correct
                    self.rxfifo_buf[i..i+4].copy_from_slice(&self.regs.fifo(0).read().0.to_ne_bytes())
                }

                &self.rxfifo_buf[..length]
            }
        }
    }
}

#[cfg(feature = "async")]
pub mod asynch {
    use embassy_usb_driver::{
        EndpointAddress,
        EndpointAllocError,
        EndpointType,
        Event,
        Unsupported,
    };
    pub use embassy_usb_synopsys_otg::Config;
    use embassy_usb_synopsys_otg::{
        on_interrupt,
        otg_v1::Otg,
        Bus as OtgBus,
        ControlPipe,
        Driver as OtgDriver,
        Endpoint,
        In,
        OtgInstance,
        Out,
        PhyType,
        State,
    };
    use procmacros::handler;

    use super::*;
    use crate::Cpu;

    // From ESP32-S3 TRM:
    // Six additional endpoints (endpoint numbers 1 to 6), configurable as IN or OUT
    const MAX_EP_COUNT: usize = 7;

    static STATE: State<MAX_EP_COUNT> = State::new();

    /// Asynchronous USB driver.
    pub struct Driver<'d> {
        inner: OtgDriver<'d, MAX_EP_COUNT>,
    }

    impl<'d> Driver<'d> {
        const REGISTERS: Otg = unsafe { Otg::from_ptr(Usb::REGISTERS.cast_mut()) };

        /// Initializes USB OTG peripheral with internal Full-Speed PHY, for
        /// asynchronous operation.
        ///
        /// # Arguments
        ///
        /// * `ep_out_buffer` - An internal buffer used to temporarily store
        ///   received packets.
        ///
        /// Must be large enough to fit all OUT endpoint max packet sizes.
        /// Endpoint allocation will fail if it is too small.
        pub fn new(_peri: Usb<'d>, ep_out_buffer: &'d mut [u8], config: Config) -> Self {
            // From `synopsys-usb-otg` crate:
            // This calculation doesn't correspond to one in a Reference Manual.
            // In fact, the required number of words is higher than indicated in RM.
            // The following numbers are pessimistic and were figured out empirically.
            const RX_FIFO_EXTRA_SIZE_WORDS: u16 = 30;

            let instance = OtgInstance {
                regs: Self::REGISTERS,
                state: &STATE,
                fifo_depth_words: Usb::FIFO_DEPTH_WORDS as u16,
                extra_rx_fifo_words: RX_FIFO_EXTRA_SIZE_WORDS,
                endpoint_count: Usb::ENDPOINT_COUNT,
                phy_type: PhyType::InternalFullSpeed,
                quirk_setup_late_cnak: quirk_setup_late_cnak(),
                calculate_trdt_fn: |_| 5,
            };
            Self {
                inner: OtgDriver::new(ep_out_buffer, instance, config),
            }
        }
    }

    impl<'d> embassy_usb_driver::Driver<'d> for Driver<'d> {
        type EndpointOut = Endpoint<'d, Out>;
        type EndpointIn = Endpoint<'d, In>;
        type ControlPipe = ControlPipe<'d>;
        type Bus = Bus<'d>;

        fn alloc_endpoint_in(
            &mut self,
            ep_type: EndpointType,
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<Self::EndpointIn, EndpointAllocError> {
            self.inner
                .alloc_endpoint_in(ep_type, max_packet_size, interval_ms)
        }

        fn alloc_endpoint_out(
            &mut self,
            ep_type: EndpointType,
            max_packet_size: u16,
            interval_ms: u8,
        ) -> Result<Self::EndpointOut, EndpointAllocError> {
            self.inner
                .alloc_endpoint_out(ep_type, max_packet_size, interval_ms)
        }

        fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
            let (bus, cp) = self.inner.start(control_max_packet_size);

            (
                Bus {
                    inner: bus,
                    inited: false,
                },
                cp,
            )
        }
    }

    /// Asynchronous USB bus mainly used internally by `embassy-usb`.
    // We need a custom wrapper implementation to handle custom initialization.
    pub struct Bus<'d> {
        inner: OtgBus<'d, MAX_EP_COUNT>,
        inited: bool,
    }

    impl<'d> Bus<'d> {
        fn init(&mut self) {
            Usb::_enable();

            let r = Driver::REGISTERS;

            // Wait for AHB ready.
            while !r.grstctl().read().ahbidl() {}

            // Configure as device.
            r.gusbcfg().write(|w| {
                // Force device mode
                w.set_fdmod(true);
                w.set_srpcap(false);
                // Enable internal full-speed PHY
                w.set_physel(true);
            });
            self.inner.config_v1();

            // Perform core soft-reset
            r.grstctl().modify(|w| w.set_csrst(true));
            while r.grstctl().read().csrst() {}

            r.pcgcctl().modify(|w| {
                // Disable power down
                w.set_stppclk(false);
            });

            unsafe {
                crate::interrupt::bind_interrupt(
                    crate::peripherals::Interrupt::USB,
                    interrupt_handler.handler(),
                );
                crate::interrupt::enable(
                    crate::peripherals::Interrupt::USB,
                    interrupt_handler.priority(),
                )
                .unwrap();
            }
        }

        fn disable(&mut self) {
            crate::interrupt::disable(Cpu::ProCpu, crate::peripherals::Interrupt::USB);

            #[cfg(multi_core)]
            crate::interrupt::disable(Cpu::AppCpu, crate::peripherals::Interrupt::USB);

            Usb::_disable();
        }
    }

    impl<'d> embassy_usb_driver::Bus for Bus<'d> {
        async fn poll(&mut self) -> Event {
            if !self.inited {
                self.init();
                self.inited = true;
            }

            self.inner.poll().await
        }

        fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
            self.inner.endpoint_set_stalled(ep_addr, stalled)
        }

        fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
            self.inner.endpoint_is_stalled(ep_addr)
        }

        fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
            self.inner.endpoint_set_enabled(ep_addr, enabled)
        }

        async fn enable(&mut self) {
            self.inner.enable().await
        }

        async fn disable(&mut self) {
            self.inner.disable().await
        }

        async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
            self.inner.remote_wakeup().await
        }
    }

    impl<'d> Drop for Bus<'d> {
        fn drop(&mut self) {
            Bus::disable(self);
        }
    }

    fn quirk_setup_late_cnak() -> bool {
        // Our CID register is 4 bytes offset from what's in embassy-usb-synopsys-otg
        let cid = unsafe {
            Driver::REGISTERS
                .as_ptr()
                .cast::<u32>()
                .add(0x40)
                .read_volatile()
        };
        // ESP32-Sx has a different CID register value, too
        cid == 0x4f54_400a || cid & 0xf000 == 0x1000
    }

    #[handler(priority = crate::interrupt::Priority::max())]
    fn interrupt_handler() {
        let setup_late_cnak = quirk_setup_late_cnak();

        unsafe {
            on_interrupt(
                Driver::REGISTERS,
                &STATE,
                Usb::ENDPOINT_COUNT,
                setup_late_cnak,
            )
        }
    }
}
