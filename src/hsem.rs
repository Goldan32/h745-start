use stm32h7xx_hal as hal;

#[derive(Debug)]
pub enum Error {
    InvalidHsemIndex,
}

fn get_core_id() -> u8 {
    if cfg!(core = "0") {
        0x03
    } else {
        0x01
    }
}

pub trait Hsem {
    fn release(&self, sem_id: usize) -> Result<(), Error>;
    fn lock(&self, sem_id: usize, proc_id: u8) -> Result<u32, Error>;
    fn fast_lock(&self, sem_id: usize) -> Result<u32, Error>;
    fn is_taken(&self, sem_id: usize) -> Result<bool, Error>;
    fn release_all(&self, key: u16);
    fn set_clear_key(&self, key: u16);
    fn get_clear_key(&self) -> u16;
    fn enable_interrupt(&self, sem_mask: u32);
    fn disable_interrupt(&self, sem_mask: u32);
}

impl Hsem for stm32h7xx_hal::stm32::HSEM {
    fn release(&self, sem_id: usize) -> Result<(), Error> {
        if sem_id > 31 {
            Err(Error::InvalidHsemIndex)
        } else {
            self.r[sem_id].write(|w| unsafe { w
                .procid().bits(0)
                .masterid().bits(get_core_id())
                .lock().bit(false)});
            Ok(())
        }
    }

    fn lock(&self, sem_id: usize, proc_id: u8) -> Result<u32, Error> {
        if sem_id > 31 {
            Err(Error::InvalidHsemIndex)
        } else {
            self.r[sem_id].write(|w| unsafe { w
                .procid().bits(proc_id)
                .masterid().bits(get_core_id())
                .lock().bit(true)});
            Ok(self.r[sem_id].read().bits())
        }
    }

    fn fast_lock(&self, sem_id: usize) -> Result<u32, Error> {
        if sem_id > 31 {
            Err(Error::InvalidHsemIndex)
        } else {
            Ok(self.rlr[sem_id].read().bits())
        }
    }

    fn is_taken(&self, sem_id: usize) -> Result<bool, Error> {
        if sem_id > 31 {
            Err(Error::InvalidHsemIndex)
        } else {
            Ok(self.r[sem_id].read().lock().bit())
        }
    }

    fn release_all(&self, key: u16) {
        self.cr.write(|w| unsafe { w
            .key().bits(key)
            .masterid().bits(get_core_id())
        });
    }

    fn set_clear_key(&self, key: u16) {
        self.keyr.modify(|_, w| unsafe { w
            .key().bits(key)
        });
    }

    fn get_clear_key(&self) -> u16 {
        self.keyr.read().key().bits()
    }

    fn enable_interrupt(&self, sem_mask: u32) {
        self.ier.modify(|r, w| unsafe { w.bits(r.bits() | sem_mask) });
    }

    fn disable_interrupt(&self, sem_mask: u32) {
        self.ier.modify(|r, w| unsafe { w.bits(r.bits() & !sem_mask)})
    }
}
