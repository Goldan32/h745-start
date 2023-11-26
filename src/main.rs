#![no_main]
#![no_std]
#![allow(unused_imports)]

mod utils;

use hal::device::stk::cvr::CURRENT_R;
use stm32h7xx_hal as hal;
use panic_halt as _;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use core::fmt::Write;
use cortex_m;
use cortex_m_rt::{entry, exception};
use hal::{pac, prelude::*, time::*, rcc, adc, rcc::rec::AdcClkSel, traits::DacOut};
use hal::rcc::CoreClocks;
use hal::{ethernet, ethernet::PHY};
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
    <title>Hello, World!</title>
</head>
<body>
    <h1>";

#[cfg(core = "0")]
const WEBPAGE_LOWER: &str =
r#"</h1>
<form action="/" method="POST">
  <input type="text" id="voltage" name="voltage">
  <input type="submit" value="Set">
</form>
</body>
</html>"#;

// - global static state ------------------------------------------------------
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
    static mut CURRENT_VOLTAGE: f32 = 0.0;

    #[shared]
    static CRITICAL: AtomicU8 = AtomicU8::new(UNLOCKED);

    CRITICAL.store(UNLOCKED, Ordering::Release);

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
                        // Call function here
                        let _res = req.parse(&data).unwrap();
                        match req.method.unwrap() {
                            "GET" => {
                                log_serial!(tx, "In GET arm\r\n");
                            }
                            "POST" => {
                                log_serial!(tx, "In POST arm\r\n");
                                for line in core::str::from_utf8(&data).unwrap().lines() {
                                    let target_voltage:u16 = match line.find("voltage=") {
                                        Some(idx) => {
                                            let numidx = idx+"voltage=".len();
                                            let numstr = &line[numidx..].trim_end_matches('\0');
                                            match numstr.parse::<u16>() {
                                                Ok(num) => {log_serial!(tx, "Got target: {}\r\n", num); num},
                                                Err(_) => {log_serial!(tx, "Error parsing, original was: {}\r\n", &numstr);
                                                for x in numstr.bytes() {
                                                    log_serial!(tx, "{}", x as usize);
                                                }
                                                1}
                                            }
                                        },
                                        None => 0
                                    };
                                }
                            }
                            _ => {
                                log_serial!(tx, "In ERROR arm\r\n");
                                tcp_socket.close();
                            }
                        }
                        if tcp_socket.may_send() {
                            while CRITICAL
                                .compare_exchange(UNLOCKED, LOCKED, Ordering::AcqRel, Ordering::Relaxed)
                                .is_err()
                                {}

                            // CRITICAL SECTION START
                            let display_voltage;
                            unsafe {
                                display_voltage = CURRENT_VOLTAGE;
                            }

                            CRITICAL.store(UNLOCKED, Ordering::Release);
                            // CRITICAL SECTION END

                            let hello_world = write_to::show(&mut hello_world_buf,
                                format_args!("{}Voltage: {}{}", WEBPAGE_UPPER, display_voltage, WEBPAGE_LOWER))
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

            let mut counter = 0usize;
            let mut samples: [f32; 128] = [0f32; 128];

            const ALPHA: f32 = 0.1;
            const BETA: f32 = 0.2;

            let mut adc_value: f32 = 0.0;
            let mut filtered_value: f32 = 0.0;
            let mut prev_adc_value: f32 = 0.0;
            let mut prev_filtered_value: f32 = 0.0;

            loop {
                let reading: u32 = adc1.read(&mut channel).unwrap();
                samples[counter] = reading as f32 * (3.3 / adc1.slope() as f32);

                adc_value = samples[counter];
                filtered_value = ALPHA * adc_value
                                 + (1.0 - ALPHA) * prev_filtered_value
                                 + BETA * prev_adc_value;

                prev_adc_value = adc_value;
                prev_filtered_value = filtered_value;

                let out_value: u16 = if filtered_value < 0.0 { 0 }
                    else if filtered_value > 3.3 { 4095 }
                    else { ((filtered_value /  3.3) * 4096.0) as u16};

                dac.set_value(out_value);

                // Delay so adc is not handled too often
                delay.delay_us(10u16);

                if counter == 127 {
                    counter = 0;
                    let mut avg = 0f32;
                    for v in samples {
                        avg += v;
                    }

                    avg /= 128.0;

                    while CRITICAL
                        .compare_exchange(UNLOCKED, LOCKED, Ordering::AcqRel, Ordering::Relaxed)
                        .is_err()
                        {log_serial!(tx, "Stuck: {}\r", CRITICAL.load(Ordering::Relaxed));}

                    // CRITICAL SECTION START
                    unsafe {
                        CURRENT_VOLTAGE = avg;
                    }

                    CRITICAL.store(UNLOCKED, Ordering::Release);
                    // CRITICAL SECTION END
                }

                counter += 1;
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
