use core::convert::Infallible;

use nb::block;
use crate::pac::SHA;

// All the hash algorithms introduced in FIPS PUB 180-4 Spec.
// – SHA-1
// – SHA-224
// – SHA-256
// – SHA-384
// – SHA-512
// – SHA-512/224
// – SHA-512/256
// – SHA-512/t
// Two working modes
// – Typical SHA
// – DMA-SHA (not implemented yet)

const ALIGN_SIZE: usize = 4; // size_of::<u32>

// The alignment helper helps you write to registers that only accepts u32 using regular u8s (bytes)
// It keeps a write buffer of 4 u8 (could in theory be 3 but less convient)
// And if the incoming data is not convertable to u32 (i.e. not a multiple of 4 in length) it will
// store the remainder in the buffer until the next call
//
// It assumes incoming `dst` are aligned to desired layout (in future ptr.is_aligned can be used)
// It also assumes that writes are done in FIFO order
#[derive(Debug)]
struct AlignmentHelper {
    buf: [u8; ALIGN_SIZE],
    buf_fill: usize
}

impl AlignmentHelper {
    pub fn default() -> AlignmentHelper {
        AlignmentHelper { buf: [0u8; ALIGN_SIZE], buf_fill: 0 }
    }

    // This function will write any remaining buffer to dst and return the amount of *bytes*
    // written (0 means no write)
    pub unsafe fn flush_to(&mut self, dst: *mut u32) -> usize {
        if self.buf_fill != 0 {
            for i in self.buf_fill..ALIGN_SIZE {
                self.buf[i] = 0;
            }

            //println!("Flushing {:02x?} to {:?}", self.buf, dst);
            dst.write_volatile(u32::from_ne_bytes(self.buf));
        }
        let flushed = self.buf_fill;
        self.buf_fill = 0;
        return flushed;
    }

    // This function is similar to `volatile_set_memory` but will prepend data that was previously
    // ingested and ensure aligned (u32) writes
    pub unsafe fn volatile_write_bytes(&mut self, dst: *mut u32, val: u8, count: usize) {
        let mut cursor = 0;
         if self.buf_fill != 0 {
            for i in self.buf_fill..ALIGN_SIZE {
                self.buf[i] = val;
            }
            //println!("Flushing due to write bytes {:02x?} to {:?}", self.buf, dst);
            dst.write_volatile(u32::from_ne_bytes(self.buf));
            cursor = 1;
            self.buf_fill = 0;
        }
        core::ptr::write_bytes(dst.add(cursor), val, count);
        //core::intrinsics::volatile_set_memory(dst.add(cursor), val, count);
    }

    // This function is similar to `volatile_copy_nonoverlapping_memory`, however it buffers up to
    // a u32 in order to always write to registers in an aligned way. Additionally it will keep
    // stop writing when the end of the register (defined by `dst_bound` relative to `dst`) and
    // returns the remaining data (if not possible to write everything), and if it wrote till
    // dst_bound or exited early (due to lack of data).
    pub unsafe fn aligned_volatile_copy<'a>(&mut self, dst: *mut u32, src: &'a [u8], dst_bound: usize) -> (&'a [u8], bool) {
        assert!(dst_bound > 0);

        let mut nsrc = src;
        let mut cursor = 0;
        if self.buf_fill != 0 {
            // First prepend existing data
            let max_fill = ALIGN_SIZE - self.buf_fill;
            let (nbuf, src) = src.split_at(core::cmp::min(src.len(), max_fill));
            nsrc = src;
            for i in 0..max_fill {
                match nbuf.get(i) {
                    Some(v) => { 
                        self.buf[self.buf_fill+i] = *v ;
                        self.buf_fill += 1;
                    },
                    None => { return (&[], false) } // Used up entire buffer before filling buff_fil
                }
            }

            dst.write_volatile(u32::from_ne_bytes(self.buf));
            //println!("Writing partial to {:02x?} to {:?}", self.buf, dst);
            cursor += 1; 
            self.buf_fill = 0;
        }

        if dst_bound <= cursor * ALIGN_SIZE {
            return (nsrc, true);
        }

        let (to_write, remaining) = nsrc.split_at(core::cmp::min(
                dst_bound-cursor*ALIGN_SIZE, 
                (nsrc.len()/ALIGN_SIZE)*ALIGN_SIZE // TODO: unstable div_floor for clarity?
                ));

        if to_write.len() > 0 {
            //println!("Writing full to {:02x?} ({}) to {:?} {}", to_write, to_write.len(), dst.add(cursor), to_write.len()/ALIGN_SIZE);
       
            // Raw v_c_n_m also works but only when src.len() >= 4 * ALIGN_SIZE, otherwise it be broken
            // core::intrinsics::volatile_copy_nonoverlapping_memory::<u32>(dst.add(cursor), to_write.as_ptr() as *const u32, to_write.len()/alignment);
            for (i, v) in to_write.chunks_exact(ALIGN_SIZE).enumerate() {
                dst.add(i).write_volatile(u32::from_ne_bytes(v.try_into().unwrap()).to_be());
            }
        }

        // If it's data we can't store we don't need to try and align it, just wait for next write
        // Generally this applies when (src/4*4) != src
        let was_bounded = dst_bound - to_write.len() == 0;
        if remaining.len() > 0 && remaining.len() < 4 {
            //println!("Storing partial write {:02x?}", remaining);
            for i in 0..remaining.len() {
                self.buf[i] = remaining[i];
            }
            self.buf_fill = remaining.len();
            return (&[], was_bounded);
        }

        return (remaining, was_bounded);
    }
}


#[derive(Debug)]
pub struct Sha {
    sha: SHA,
    mode: ShaMode,
    alignment_helper: AlignmentHelper,
    cursor: usize,
    first_run: bool,
    finished: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum ShaMode {
    SHA1,
    #[cfg(not(esp32))]
    SHA224,
    SHA256,
    SHA384,
    #[cfg(any(esp32s2, esp32s3, esp32))]
    SHA512,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_224,
    #[cfg(any(esp32s2, esp32s3))]
    SHA512_256,
    // SHA512_(u16) // Max 511
}

// TODO: Maybe make Sha Generic (Sha<Mode>) in order to allow for better
// compiler optimizations? (Requires complex const generics which isn't stable
// yet)

#[cfg(esp32)]
impl Sha {
    pub fn new(sha: SHA, mode: ShaMode) -> Self {
        // Setup SHA Mode
        Self {
            sha,
            mode,
            cursor: 0,
            first_run: true,
            finished: false,
            alignment_helper: AlignmentHelper::default() 
        }
    }

    fn process_buffer(&mut self) {
        if self.first_run {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_start().write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA256 => self.sha.sha256_start().write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA384 => self.sha.sha384_start.write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA512 => self.sha.sha512_start.write(|w| unsafe { w.bits(1u32) }),
            }
            self.first_run = false;
        } else {
            match self.mode {
                ShaMode::SHA1 => self.sha.sha1_continue().write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA256 => self.sha.sha256_continue.write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA384 => self.sha.sha384_continue.write(|w| unsafe { w.bits(1u32) }),
                ShaMode::SHA512 => self.sha.sha512_continue.write(|w| unsafe { w.bits(1u32) }),
            }
        }
    }

    fn is_busy(&mut self) -> bool {
        match self.mode {
            // FIXME: These are marked WO, while being RO
            ShaMode::SHA1 => unsafe { self.sha.sha1_busy.as_ptr().read() == 0 },
            ShaMode::SHA256 => unsafe { self.sha.sha256_busy.as_ptr().read() == 0 },
            ShaMode::SHA384 => unsafe { self.sha.sha384_busy.as_ptr().read() == 0 },
            ShaMode::SHA512 => unsafe { self.sha.sha512_busy.as_ptr().read() == 0 },
        }
    }

    fn chunk_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 | ShaMode::SHA256 => 64,
            ShaMode::SHA384 | ShaMode::SHA512 => 128,
        };
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        let mod_cursor = self.cursor % self.chunk_length();
        unsafe {
            let ptr = (self.sha.text_[0].as_ptr() as *mut u32).add(mod_cursor / ALIGN_SIZE);
            let (remaining, bound_reached) = self.alignment_helper.aligned_volatile_copy(ptr, 
                                                                         buffer, 
                                                                         self.chunk_length()-mod_cursor);
            self.cursor = self.cursor.wrapping_add(buffer.len() - remaining.len());
            if bound_reached {
                self.process_buffer();
            }
            Ok(remaining)
        }
    }

    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            let dst_ptr = (self.sha.text_.as_ptr() as *mut u32).add((self.cursor % self.chunk_length()) / ALIGN_SIZE);
            let flushed =  self.alignment_helper.flush_to(dst_ptr);
            if flushed != 0 {
                self.cursor = self.cursor.wrapping_add(ALIGN_SIZE - flushed);
                if self.cursor % self.chunk_length() == 0 {
                    self.process_buffer();
                }
            }
        }
        
        Ok(())
    }

    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        if !self.finished {
            block!(self.flush_data())?; // Flush partial data, ensures aligned cursor 

            match self.mode {
                // FIXME: These are marked WO, while being RO, also inconsistent func/reg
                ShaMode::SHA1 => unsafe { self.sha.sha1_load.write(|w| w.bits(1u32)) },
                ShaMode::SHA256 => unsafe { self.sha.sha256_load().write(|w| w.bits(1u32)) },
                ShaMode::SHA384 => unsafe { self.sha.sha384_load.write(|w| w.bits(1u32)) },
                ShaMode::SHA512 => unsafe { self.sha.sha512_load.write(|w| w.bits(1u32)) },
            }

            // Spin wait for result, 8-20 clock cycles according to manual
            while self.is_busy() {}
            self.finished = true;
        }

        unsafe {
            // Read SHA1=Text[0:4] | SHA256=Text[0:8] | SHA384=Text[0:11] |
            // SHA512=Text[0:15] TODO: limit read len to digest size
            core::ptr::copy_nonoverlapping::<u32>(
                self.sha.text_.as_ptr() as *const u32,
                output.as_mut_ptr() as *mut u32,
                output.len() / ALIGN_SIZE,
            );
        }
        Ok(())
    }
}


#[cfg(not(esp32))]
fn mode_as_bits(mode: ShaMode) -> u8 {
    match mode {
        ShaMode::SHA1 => 0,
        ShaMode::SHA224 => 1,
        ShaMode::SHA256 => 2,
        ShaMode::SHA384 => 3,
        ShaMode::SHA512 => 4,
        #[cfg(any(esp32s2, esp32s3))]
        ShaMode::SHA512_224 => 5,
        #[cfg(any(esp32s2, esp32s3))]
        ShaMode::SHA512_256 => 6,
        // _ => 0 // TODO: SHA512/t
    }
}

// TODO: Allow/Implemenet SHA512_(u16)

// A few notes on this implementation with regards to 'memcpy',
// - It seems that ptr::write_bytes already acts as volatile, while ptr::copy_* does not (in this case)
// - The registers are *not* cleared after processing, so padding needs to be written out
// - This component uses core::intrinsics::volatile_* which is unstable, but is the only way to
// efficiently copy memory with volatile
// - For this particular registers (and probably others), a full u32 needs to be written partial
// register writes (i.e. in u8 mode) does not work
//   - This means that we need to buffer bytes coming in up to 4 u8's in order to create a full u32
//     - The implementation for this is messy atm (the entire write_data function is for this purpose), it would be good to create a uniform interface
//     for writing into aligned registers 

// This only supports inputs up to u32::MAX in byte size, to increase please see ::finish()
// length/self.cursor usage
#[cfg(any(esp32s2, esp32s3, em_memsp32c2, esp32c3))]
impl Sha {
    pub fn new(sha: SHA, mode: ShaMode) -> Self {
        // Setup SHA Mode
        sha.mode.write(|w| unsafe {
            w.mode().bits(mode_as_bits(mode))
        });
        Self {
            sha,
            mode,
            cursor: 0,
            first_run: true,
            finished: false,
            alignment_helper: AlignmentHelper::default() 
        }
    }

    pub fn first_run(&self) -> bool {
        self.first_run
    }

    pub fn finished(&self) -> bool {
        self.finished
    }

    fn process_buffer(&mut self) {
        // FIXME: SHA_START_REG & SHA_CONTINUE_REG are wrongly marked as RO (they are WO)
        if self.first_run {
            // Set SHA_START_REG
            unsafe {
                self.sha.start.as_ptr().write_volatile(1u32);
            }
            self.first_run = false;
        } else {
            // SET SHA_CONTINUE_REG
            unsafe {
                self.sha.continue_.as_ptr().write_volatile(1u32);
            }
        }
    }

    fn chunk_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 | ShaMode::SHA224 | ShaMode::SHA256 => 64,
            _ => 128,
        };
    }

    pub fn digest_length(&self) -> usize {
        return match self.mode {
            ShaMode::SHA1 => 20,
            ShaMode::SHA224 => 28,
            ShaMode::SHA256 => 32, 
            ShaMode::SHA384 => 48,
            ShaMode::SHA512 => 64,
            #[cfg(any(esp32s2, esp32s3))]
            ShaMode::SHA512_224 => 28,
            #[cfg(any(esp32s2, esp32s3))]
            ShaMode::SHA512_256 => 32, 
        }
    }

    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }
        unsafe {
            let dst_ptr = (self.sha.m_mem.as_ptr() as *mut u32).add((self.cursor % self.chunk_length()) / ALIGN_SIZE);
            let flushed =  self.alignment_helper.flush_to(dst_ptr);
            if flushed != 0 {
                self.cursor = self.cursor.wrapping_add(ALIGN_SIZE - flushed);
                if self.cursor % self.chunk_length() == 0 {
                    self.process_buffer();
                }
            }
        }
        
        Ok(())
    }

    // This function ensures that incoming data is aligned to u32 (due to issues with cpy_mem<u8>)
    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> nb::Result<&'a [u8], Infallible>
    {
        let mod_cursor = self.cursor % self.chunk_length();
        unsafe {
            // NOTE: m_mem is 64,u8; but datasheet (esp32s2/esp32s3@>=SHA384) says 128, u8
            let ptr = (self.sha.m_mem[0].as_ptr() as *mut u32).add(mod_cursor / ALIGN_SIZE);
            let (remaining, bound_reached) = self.alignment_helper.aligned_volatile_copy(ptr, 
                                                                         incoming, 
                                                                         self.chunk_length()-mod_cursor);
            self.cursor = self.cursor.wrapping_add(incoming.len() - remaining.len());
            if bound_reached {
                self.process_buffer();
            }
            Ok(remaining)
        }
    }

    pub fn update<'a>(&mut self, buffer: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }

        let remaining = self.write_data(buffer)?;
        Ok(remaining)
    }

    pub fn finish(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        // The main purpose of this function is to dynamically generate padding for the input.
        // Padding: Append "1" bit, Pad zeros until 512/1024 filled
        // then set the message length in the LSB (overwriting the padding)
        // If not enough free space for length+1, add length at end of a new zero'd block

        if self.sha.busy.read().bits() != 0 {
            return Err(nb::Error::WouldBlock);
        }

        let chunk_len = self.chunk_length();

        if !self.finished {
            // Store message length for padding 
            let length = self.cursor * 8;
            block!(self.update(&[0x80]))?; // Append "1" bit
            block!(self.flush_data())?;    // Flush partial data, ensures aligned cursor 
            //assert!(self.cursor % 4 == 0);

            let mod_cursor = self.cursor % chunk_len;

            if chunk_len - mod_cursor < chunk_len / 8 {
                // Zero out remaining data if buffer is almost full (>=448/896), and process buffer
                let pad_len = chunk_len - mod_cursor;
                unsafe {
                    let m_cursor_ptr = (self.sha.m_mem[0].as_ptr() as *mut u32).add(mod_cursor/ALIGN_SIZE);
                    self.alignment_helper.volatile_write_bytes(m_cursor_ptr, 0, pad_len/ALIGN_SIZE);
                    //core::intrinsics::volatile_set_memory(m_cursor_ptr, 0, pad_len/4);
                }
                self.process_buffer();
                self.cursor = self.cursor.wrapping_add(pad_len);

                // Spin-wait for finish 
                while self.sha.busy.read().bits() != 0 {}
            }

            let mod_cursor = self.cursor % chunk_len; // Should be zero if branched above
            unsafe {
                let m_cursor_ptr = self.sha.m_mem[0].as_ptr() as *mut u32;
                // Pad zeros 
                let pad_ptr = m_cursor_ptr.add(mod_cursor/ALIGN_SIZE);
                let pad_len = (chunk_len - mod_cursor) - ALIGN_SIZE; 

                self.alignment_helper.volatile_write_bytes(pad_ptr, 0, pad_len/ALIGN_SIZE);

                // Write length (BE) to end
                // NOTE: aligned_volatile_copy does not work here
                // The decompiler suggest volatile_copy_memory/write_volatile is optimized to a simple *v = *pv;
                // While the aligned_volatile_copy makes an actual call to memcpy, why this makes a
                // difference when memcpy does works in other places, I don't know
                let end_ptr = m_cursor_ptr.add((chunk_len/ALIGN_SIZE) -1);
                end_ptr.write_volatile(length.to_be() as u32);
                //let len_buf = length.to_be_bytes();
                //core::intrinsics::volatile_copy_nonoverlapping_memory::<u32>(end_ptr, len_buf.as_ptr() as *const u32, len_buf.len()/alignment);
            }

            self.process_buffer();
            // Spin-wait for final buffer to be processed
            while self.sha.busy.read().bits() != 0 {}
            self.finished = true;
        }

        unsafe {
            core::ptr::copy_nonoverlapping::<u32>(
                self.sha.h_mem.as_ptr() as *const u32,
                output.as_mut_ptr() as *mut u32,
                core::cmp::min(self.digest_length(), output.len())/ALIGN_SIZE,
            );
        }
        Ok(())
    }

    pub fn free(self) -> SHA {
        self.sha
    }
}
