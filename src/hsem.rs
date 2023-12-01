use stm32h7xx_hal as hal;

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
    fn unlock(&self, sem_id: usize) -> Result<(), Error>;
    fn lock_2_step(&self, sem_id: usize, proc_id: u8) -> Result<u32, Error>;
    fn lock_1_step(&self, sem_id: usize) -> Result<u32, Error>;
}

impl Hsem for stm32h7xx_hal::stm32::HSEM {
    fn unlock(&self, sem_id: usize) -> Result<(), Error> {
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

    fn lock_2_step(&self, sem_id: usize, proc_id: u8) -> Result<u32, Error> {
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

    fn lock_1_step(&self, sem_id: usize) -> Result<u32, Error> {
        if sem_id > 31 {
            Err(Error::InvalidHsemIndex)
        } else {
            Ok(self.r[sem_id].read().bits())
        }
    }
}
