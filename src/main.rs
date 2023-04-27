#![no_main]
#![no_std]

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};
use stm32ral::{write_reg, rcc, gpio};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    loop {
        led.set_high();
        delay.delay_ms(500_u16);

        led.set_low();
        delay.delay_ms(500_u16);
    }

}

unsafe extern "C" fn main2() -> ! {
    write_reg!(rcc, RCC, AHB4ENR, GPIOBEN: Enabled);
    write_reg!(gpio, GPIOB, MODER, MODER0: Output, MODER14: Output);
    write_reg!(gpio, GPIOB, ODR, 0);
    loop {
        write_reg!(gpio, GPIOB, BSRR, BS0: 1);
        cortex_m::asm::delay(16_000_000);
        write_reg!(gpio, GPIOB, BSRR, BR0: 1);
        cortex_m::asm::delay(16_000_000);
    }
}

#[no_mangle]
#[link_section=".flash2.reset_vector"]
pub static CPU2_RESET_VECTOR: unsafe extern "C" fn() -> ! = main2;
