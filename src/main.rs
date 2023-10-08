#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU8, Ordering};
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, time::*, rcc};
use panic_halt as _;
use microamp::shared;
use core::fmt::Write;

const PLL3_P: Hertz = Hertz::Hz(48_000 * 256);

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().vos0(&dp.SYSCFG).freeze();

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(480.MHz()) // system clock @ 480 MHz
              .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
              .pll3_p_ck(PLL3_P) // sai clock @ 12.288 MHz
             //.use_hse_crystal()                              // TODO hse oscillator @ 25 MHz
              .freeze(pwrcfg, &dp.SYSCFG);

    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();
    let mut led2 = gpiob.pb0.into_push_pull_output();

    let tx = gpiod.pd8.into_alternate();
    let rx = gpiod.pd9.into_alternate();

    let serial = dp
        .USART3
        .serial((tx, rx), 19200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
        .unwrap();

    let (mut tx, mut rx) = serial.split();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    //let core_based_num = if cfg!(core = "0") {250_u16} else {1000_u16};

    const CORE0: u8 = 0;
    const CORE1: u8 = 1;
    const LOCKED: u8 = 2;

    #[shared]
    static mut SHARED: u32 = 0;

    #[shared]
    static SEMAPHORE: AtomicU8 = AtomicU8::new(CORE0);

    let mut delay_time = 500_u16;

    let (our_turn, next_core) = if cfg!(core = "0") {
        (CORE0, CORE1)
    } else {
        (CORE1, CORE0)
    };

    loop {
        match () {
            #[cfg(core = "0")]
            () => {
                writeln!(tx, "Hello World!\r\n").unwrap();
                // Busy wait for semaphore
                unsafe {if SHARED > 5 {delay_time = 1000_u16}}
                led.set_high();
                delay.delay_ms(delay_time);
        
                led.set_low();
                delay.delay_ms(delay_time);
            }
            #[cfg(not(core = "0"))]
            () => {
                // Busy wait for semaphore
                unsafe {SHARED += 1;}
                led2.set_high();
                delay.delay_ms(500_u16);
        
                led2.set_low();
                delay.delay_ms(500_u16);
            }
        }

    }

}
