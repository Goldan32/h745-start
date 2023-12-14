#![no_main]
#![no_std]
#![allow(unused_imports)]

mod utils;
mod hsem;

use hal::device::stk::cvr::CURRENT_R;
use hal::gpio::{Edge, ExtiPin, Input, Output, PushPull};
use stm32h7xx_hal as hal;
use panic_halt as _;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use core::fmt::Write;
use core::cell::{Cell, RefCell};
use cortex_m;
use cortex_m::peripheral::NVIC;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::{entry, exception};
use hal::{pac, prelude::*, time::*, rcc, adc, rcc::rec::AdcClkSel, traits::DacOut};
use hal::rcc::CoreClocks;
use hal::{ethernet, ethernet::PHY};
use hal::hal::Direction;
use pac::interrupt;
use microamp::shared;
use smoltcp;
use smoltcp::iface::{
    Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes, SocketStorage,
};
use smoltcp::socket::{TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint, Ipv4Address, Ipv6Cidr};
use httparse;
use crate::utils::write_to;
use crate::hsem::Hsem;

// - logging ------------------------------------------------------------------

const LOG_LEVEL: u32 = 1;

macro_rules! log_serial {
    ($tx: expr, $($arg:tt)*) => {{
        if LOG_LEVEL > 0 {
            write!($tx, $($arg)*).unwrap();
        }
    }};
}

// - global constants ---------------------------------------------------------

const PLL3_P: Hertz = Hertz::Hz(48_000 * 256);

#[cfg(core = "0")]
const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
#[cfg(core = "0")]
const IP_LOCAL: [u8; 4] = [192, 168, 0, 139];
#[cfg(core = "0")]
const MAX_PACKET_SIZE: usize = 576;
#[cfg(core = "0")]
const LOCAL_PORT: u16 = 6970;
#[cfg(core = "0")]
const WEBPAGE_UPPER: &str =
"HTTP/1.1 200 OK
Content-Type: text/html

<!DOCTYPE html>
<html>
<head>
    <title>STM32H745 Dual Core Demo App</title>
</head>
<body>
    <h1>STM32H745 Dual Core Demo App</h1>";

#[cfg(core = "0")]
const WEBPAGE_LOWER: &str =
r#"
<p>The filtering will use the formula: y[0] = alpha*u[0]+(1-alpha)*y[-1] + beta*u[-1]
<form action="/" method="POST">
  <input type="text" id="alpha" name="alpha" placeholder="alpha">
  <input type="text" id="beta" name="beta" placeholder="beta">
  <input type="submit" value="Set">
</form>
</body>
</html>"#;

// - global static variables ---------------------------------------------------

#[shared]
static GreenLed: Mutex<RefCell<Option<hal::gpio::PB0<Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));

#[cfg(core = "0")]
static ATOMIC_TIME: AtomicU32 = AtomicU32::new(0);

#[cfg(core = "0")]
static mut ETHERNET: Option<Net> = None;
#[cfg(core = "0")]
static mut ETHERNET_STORAGE: EthernetStorage = EthernetStorage::new();

#[cfg(core = "0")]
#[link_section = ".sram3.eth"]
static mut ETHERNET_DESCRIPTOR_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

// - entry points -------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut _cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().vos0(&dp.SYSCFG).freeze();

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    // Enable HSEM clk
    dp.RCC.ahb4enr.modify(|_, w| w.hsemen().set_bit());

    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(200.MHz()) // system clock
        .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
        .pll3_p_ck(PLL3_P) // sai clock @ 12.288 MHz
        .hclk(200.MHz())
        .pll1_r_ck(100.MHz())
      //.use_hse_crystal()                              // TODO hse oscillator @ 25 MHz
        .freeze(pwrcfg, &dp.SYSCFG);

    // Switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);

    // - shared variables ---------------------------------------------------------

    const UNLOCKED: u8 = 0;
    const LOCKED: u8 = 1;

    #[shared]
    static mut SHARED_DATA: SharedObject = SharedObject {
        voltage: 0.0,
        alpha: 0.1,
        beta: 0.2,
    };

    #[shared]
    static MUTEX: AtomicU8 = AtomicU8::new(UNLOCKED);

    MUTEX.store(UNLOCKED, Ordering::Release);

    match () {
        #[cfg(core = "0")]
        () => {
            _cp.SCB.invalidate_icache();
            _cp.SCB.enable_icache();
            _cp.DWT.enable_cycle_counter();

            // - leds -----------------------------------------------------------------

            let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

            let mut led_red = gpiob.pb14.into_push_pull_output();
            let mut led_yellow = gpioe.pe1.into_push_pull_output();
            let mut _led_green = gpiob.pb0.into_push_pull_output();
            led_red.set_high();
            led_yellow.set_low();

            free(|cs| {
                GreenLed.borrow(cs).replace(Some(_led_green));
            });

            // - uart -----------------------------------------------------------------

            let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
            let tx = gpiod.pd8.into_alternate();
            let _rx = gpiod.pd9.into_alternate();
            let serial = dp
                .USART3
                .serial((tx, _rx), 19200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
                .unwrap();
            let (mut tx, _rx) = serial.split();

            // - ethernet -------------------------------------------------------------

            use hal::gpio::gpioa;
            use hal::gpio::gpiob;
            use hal::gpio::gpioc;
            use hal::gpio::gpiog;
            type AlternateFunction11 = hal::gpio::Alternate<11>;

            let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
            let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
            let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

            let _rmii_ref_clk: gpioa::PA1<AlternateFunction11> = gpioa.pa1.into_alternate();
            let _rmii_mdio: gpioa::PA2<AlternateFunction11> = gpioa.pa2.into_alternate();
            let _rmii_mdc: gpioc::PC1<AlternateFunction11> = gpioc.pc1.into_alternate();
            let _rmii_crs_dv: gpioa::PA7<AlternateFunction11> = gpioa.pa7.into_alternate();
            let _rmii_rxd0: gpioc::PC4<AlternateFunction11> = gpioc.pc4.into_alternate();
            let _rmii_rxd1: gpioc::PC5<AlternateFunction11> = gpioc.pc5.into_alternate();
            let _rmii_tx_en: gpiog::PG11<AlternateFunction11> = gpiog.pg11.into_alternate();
            let _rmii_txd0: gpiog::PG13<AlternateFunction11> = gpiog.pg13.into_alternate();
            let _rmii_txd1: gpiob::PB13<AlternateFunction11> = gpiob.pb13.into_alternate();

            assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
            assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
            assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
            assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

            let mac_addr = EthernetAddress::from_bytes(&MAC_LOCAL);
            let (eth_dma, eth_mac) = unsafe {
                ethernet::new_unchecked(
                    dp.ETHERNET_MAC,
                    dp.ETHERNET_MTL,
                    dp.ETHERNET_DMA,
                    &mut ETHERNET_DESCRIPTOR_RING,
                    mac_addr.clone(),
                    ccdr.peripheral.ETH1MAC,
                    &ccdr.clocks,
                )
            };

            let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));

            // initialise PHY and wait for link to come up
            lan8742a.phy_reset();
            lan8742a.phy_init();
            while !lan8742a.poll_link() {}

            // enable ethernet interrupt
            unsafe {
                ethernet::enable_interrupt();
                _cp.NVIC.set_priority(pac::Interrupt::ETH, 196); // mid prio
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ETH);
            }

            // create global static ETHERNET object
            unsafe {
                ETHERNET = Some(Net::new(&mut ETHERNET_STORAGE, eth_dma, mac_addr));
            }

            // - tcp socket -----------------------------------------------------------

            let store = unsafe { &mut ETHERNET_STORAGE };
            let tcp_rx_buffer = TcpSocketBuffer::new(&mut store.tcp_rx_buffer_storage[..]);
            let tcp_tx_buffer = TcpSocketBuffer::new(&mut store.tcp_tx_buffer_storage[..]);
            let tcp_socket_handle = unsafe { ETHERNET.as_mut().unwrap().interface.add_socket(TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)) };

            let mut hello_world_buf = [0u8; 512];

            // - timer ----------------------------------------------------------------

            systick_init(&mut _cp.SYST, &ccdr.clocks); // 1ms tick
            let mut delay = _cp.SYST.delay(ccdr.clocks);

            led_red.set_low();

            let mut alpha: f32 = 0.1;
            let mut beta: f32 = 0.2;

            // - hsem -----------------------------------------------------------------

            let hsem = &dp.HSEM;

            let _ = hsem.fast_lock(0);
            delay.delay_ms(100u16);
            if hsem.is_taken(0).unwrap() {
                log_serial!(tx, "Taken\r\n");
            } else {
                log_serial!(tx, "Free\r\n");
            }
            let _ = hsem.release(0);
            delay.delay_ms(100u16);
            if hsem.is_taken(0).unwrap() {
                log_serial!(tx, "Taken\r\n");
            } else {
                log_serial!(tx, "Free\r\n");
            }

            // - main loop ------------------------------------------------------------
            loop {
                match lan8742a.poll_link() {
                    true => led_yellow.set_high(),
                    _ => led_yellow.set_low(),
                }

                let tcp_socket = unsafe {
                    ETHERNET.as_mut().unwrap().interface
                            .get_socket::<TcpSocket>(tcp_socket_handle)
                };

                if !tcp_socket.is_open() {
                    tcp_socket.listen(LOCAL_PORT).unwrap();
                }

                if tcp_socket.may_recv() {
                    let mut headers = [httparse::EMPTY_HEADER; 16];
                    let mut req = httparse::Request::new(&mut headers);
                    let mut data_len = 0;
                    let data = tcp_socket.recv(|buffer| {
                        let mut data = [0u8; 576];
                        data_len = buffer.len();
                        for i in 0..buffer.len() {
                            data[i] = buffer[i];
                        }
                        (buffer.len(), data)
                    }).unwrap();

                    if data_len > 0 {
                        for c in data {
                            log_serial!(tx, "{}", c as char);
                        }
                        let _res = req.parse(&data).unwrap();
                        match req.method.unwrap() {
                            "GET" => {
                                log_serial!(tx, "In GET arm\r\n");
                                hsem.r[0].read().lock().bit();
                                delay.delay_ms(100u16);
                                hsem.r[0].write(|w| unsafe { w
                                    .procid().bits(0)
                                    .masterid().bits(3)
                                    .lock().bit(false) }
                                );
                            }
                            "POST" => {
                                log_serial!(tx, "In POST arm\r\n");
                                (alpha, beta) = extract_coefs(&data);
                                log_serial!(tx, "Got {} and {}\r\n", alpha, beta);

                            }
                            _ => {
                                log_serial!(tx, "In ERROR arm\r\n");
                                tcp_socket.close();
                            }
                        }
                        if tcp_socket.may_send() {
                            while MUTEX
                                .compare_exchange(UNLOCKED, LOCKED, Ordering::AcqRel, Ordering::Relaxed)
                                .is_err()
                                {}

                            // MUTEX START
                            let display_voltage;
                            unsafe {
                                display_voltage = SHARED_DATA.voltage;
                                SHARED_DATA.alpha = alpha;
                                SHARED_DATA.beta = beta;
                            }

                            MUTEX.store(UNLOCKED, Ordering::Release);
                            // MUTEX END

                            let hello_world = write_to::show(&mut hello_world_buf,
                                format_args!("{}Average voltage: {}{}", WEBPAGE_UPPER, display_voltage, WEBPAGE_LOWER))
                                .unwrap()
                                .as_bytes();
                            tcp_socket.send_slice(hello_world).unwrap();
                            tcp_socket.close();
                        }
                    }
                }

                delay.delay_ms(2000_u16);
            }
        }
        #[cfg(not(core = "0"))]
        () => {
            let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
            let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
            let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
            let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

            // - leds -----------------------------------------------------------------

            let mut _led_yellow = gpioe.pe1.into_push_pull_output();
            let mut _led_green = gpiob.pb0.into_push_pull_output();
            let mut _led_red = gpiob.pb14.into_push_pull_output();

            // - timer ----------------------------------------------------------------

            let mut delay = _cp.SYST.delay(ccdr.clocks);

            // - uart -----------------------------------------------------------------

            let tx = gpiod.pd8.into_alternate();
            let _rx = gpiod.pd9.into_alternate();

            let serial = dp
                .USART3
                .serial((tx, _rx), 19200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
                .unwrap();

            let (mut tx, _rx) = serial.split();

            // - adc -----------------------------------------------------------------

            let mut adc1 = adc::Adc::adc1(
                dp.ADC1,
                16.MHz(),
                &mut delay,
                ccdr.peripheral.ADC12,
                &ccdr.clocks,
            )
            .enable();

            adc1.set_resolution(adc::Resolution::SixteenBit);

            // - dac -----------------------------------------------------------------

            let dac = dp.DAC.dac(gpioa.pa4, ccdr.peripheral.DAC12);
            let mut dac = dac.calibrate_buffer(&mut delay).enable();
            let mut channel = gpioc.pc0.into_analog(); // ANALOG IN 10

            dac.set_value(2048); // set to 50% of vdda

            // - main loop ------------------------------------------------------------

            let mut nops: u32 = 0;
            let mut y1: f32 = 0.0;
            let mut u1: f32 = 0.0;
            let mut y0: f32 = 0.0;
            let mut u0: f32 = 0.0;
            let mut alpha: f32 = 0.2;
            let mut beta: f32 = 0.0;

            loop {
                let reading: u32 = adc1.read(&mut channel).unwrap();
                u0 = reading as f32;
                y0 = alpha * u0 + (1.0 - alpha) * y1 + beta * u1;

                let output: u16 = (y0 / 16.0) as u16;
                dac.set_value(output);
                u1 = u0;
                y1 = y0;
                //log_serial!(tx, "{} to {}\r\n", u0, y0);
            }
        }
    }

}

// - systick ------------------------------------------------------------------

#[cfg(core = "0")]
fn systick_init(syst: &mut pac::SYST, clocks: &CoreClocks) {
    let c_ck_mhz = clocks.c_ck().raw() / 1_000_000;
    let syst_calib = 0x3E8;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

// - interrupts and exceptions ------------------------------------------------

#[cfg(core = "0")]
#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() };

    if let Some(ethernet) = unsafe { ETHERNET.as_mut() } {
        let time = ATOMIC_TIME.load(Ordering::Relaxed);
        ethernet.poll(time as i64);
    }
}

#[cfg(core = "0")]
#[exception]
fn SysTick() {
    ATOMIC_TIME.fetch_add(1, Ordering::Relaxed);
}

// - NetStaticStorage ---------------------------------------------------------

#[cfg(core = "0")]
pub struct EthernetStorage<'a> {
    ip_addrs: [IpCidr; 1],
    socket_storage: [SocketStorage<'a>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],

    // network buffers
    tcp_rx_buffer_storage: [u8; MAX_PACKET_SIZE],
    tcp_tx_buffer_storage: [u8; MAX_PACKET_SIZE],
}

#[cfg(core = "0")]
impl<'a> EthernetStorage<'a> {
    pub const fn new() -> Self {
        EthernetStorage {
            ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
            socket_storage: [SocketStorage::EMPTY; 8],
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 1],

            tcp_rx_buffer_storage: [0u8; MAX_PACKET_SIZE],
            tcp_tx_buffer_storage: [0u8; MAX_PACKET_SIZE],
        }
    }
}

// - Net ----------------------------------------------------------------------

#[cfg(core = "0")]
pub struct Net<'a> {
    interface: Interface<'a, ethernet::EthernetDMA<'a, 4, 4>>,
}

#[cfg(core = "0")]
impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut EthernetStorage<'a>,
        ethdev: ethernet::EthernetDMA<'a, 4, 4>,
        ethernet_addr: EthernetAddress,
    ) -> Self {
        store.ip_addrs = [IpCidr::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 0)];

        let neighbor_cache = NeighborCache::new(&mut store.neighbor_cache_storage[..]);
        let routes = Routes::new(&mut store.routes_storage[..]);
        let interface = InterfaceBuilder::new(ethdev, &mut store.socket_storage[..])
            .hardware_addr(ethernet_addr.into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();

        Net { interface }
    }

    // poll ethernet interface
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        match self.interface.poll(timestamp) {
            Ok(_) => (),
            Err(smoltcp::Error::Exhausted) => (),
            Err(smoltcp::Error::Unrecognized) => (),
            Err(_) => (),
        };
    }
}

#[cfg(core = "0")]
pub fn extract_coefs(data_buffer: &[u8]) -> (f32, f32) {
    let mut alpha: f32 = 0.0;
    let mut beta: f32 = 0.0;
    for line in core::str::from_utf8(&data_buffer).unwrap().lines() {
        match get_variable_value(&line, "alpha=") {
            Some(num) => {alpha = num;}
            None => ()
        }
        match get_variable_value(&line, "beta=") {
            Some(num) => {beta = num;}
            None => ()
        }
    }
    (alpha, beta)
}

#[cfg(core = "0")]
pub fn get_variable_value(line: &str, var_name: &str) -> Option<f32> {
    let var: Option<f32> = match line.find(var_name) {
        Some(idx) => {
            let numidx = idx + var_name.len();
            let endidx = match line[numidx..].find('&') {
                Some(idx) => numidx + idx,
                None => line.len()
            };
            let numstr = &line[numidx..endidx].trim_end_matches('\0');
            match numstr.parse::<f32>() {
                Ok(num) => Some(num),
                Err(_) => None
            }
        }
        None => None
    };
    var
}

#[repr(C)]
struct SharedObject {
    alpha: f32,
    beta: f32,
    voltage: f32
}

fn toggle_green_led() {
    free(|cs| {
        if let Some(pin) = GreenLed.borrow(cs).borrow_mut().as_mut() {
            pin.toggle();
        }
    });
}

#[cfg(core = "0")]
#[interrupt]
fn HSEM0() {
    toggle_green_led();
}
