#![no_main]
#![no_std]

use stm32h7xx_hal as hal;
use panic_halt as _;
use core::sync::atomic::{AtomicU32, Ordering};
use core::fmt::Write;
use cortex_m;
use cortex_m_rt::{entry, exception};
use hal::{pac, prelude::*, time::*, rcc};
use hal::rcc::CoreClocks;
use hal::{ethernet, ethernet::PHY};
use pac::interrupt;
use microamp::shared;
use smoltcp;
use smoltcp::iface::{
    Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes, SocketStorage,
};
use smoltcp::socket::{UdpPacketMetadata, UdpSocket, UdpSocketBuffer, TcpSocket, TcpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint, Ipv4Address, Ipv6Cidr};

// Start udp server: nc -u -l 34254

macro_rules! log_serial {
    ($tx: expr, $($arg:tt)*) => {{
        write!($tx, $($arg)*).unwrap()
    }};
}

// - global constants ---------------------------------------------------------

const PLL3_P: Hertz = Hertz::Hz(48_000 * 256);
const MAC_LOCAL: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];
const IP_LOCAL: [u8; 4] = [192, 168, 0, 139];
const IP_REMOTE: [u8; 4] = [192, 168, 0, 65];
const IP_REMOTE_PORT: u16 = 34254;
const MAX_UDP_PACKET_SIZE: usize = 576;

// - global static state ------------------------------------------------------

static ATOMIC_TIME: AtomicU32 = AtomicU32::new(0);

static mut ETHERNET: Option<Net> = None;
static mut ETHERNET_STORAGE: EthernetStorage = EthernetStorage::new();

#[link_section = ".sram3.eth"]
static mut ETHERNET_DESCRIPTOR_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

// - entry points -------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().vos0(&dp.SYSCFG).freeze();

    // link SRAM3 power state to CPU1
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz()) // system clock
        .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
        .pll3_p_ck(PLL3_P) // sai clock @ 12.288 MHz
        .hclk(200.MHz())
        .pll1_r_ck(100.MHz())
      //.use_hse_crystal()                              // TODO hse oscillator @ 25 MHz
        .freeze(pwrcfg, &dp.SYSCFG);

    match () {
        #[cfg(core = "0")]
        () => {
            cp.SCB.invalidate_icache();
            cp.SCB.enable_icache();
            cp.DWT.enable_cycle_counter();

            // - leds -----------------------------------------------------------------

            let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

            let mut led_user = gpiob.pb14.into_push_pull_output(); // LED3, red
            let mut led_link = gpioe.pe1.into_push_pull_output(); // LED2, yellow
            let mut led_green = gpiob.pb0.into_push_pull_output(); // LED1, green
            led_user.set_high();
            led_link.set_low();

            // - uart -----------------------------------------------------------------

            let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
            let tx = gpiod.pd8.into_alternate();
            let rx = gpiod.pd9.into_alternate();
            let serial = dp
                .USART3
                .serial((tx, rx), 19200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
                .unwrap();
            let (mut tx, mut rx) = serial.split();

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
            log_serial!(tx, "Waiting for connection\r\n");
            while !lan8742a.poll_link() {}
            log_serial!(tx, "Connection established\r\n");

             // enable ethernet interrupt
            unsafe {
                ethernet::enable_interrupt();
                cp.NVIC.set_priority(pac::Interrupt::ETH, 196); // mid prio
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::ETH);
            }

            // create global static ETHERNET object
            unsafe {
                ETHERNET = Some(Net::new(&mut ETHERNET_STORAGE, eth_dma, mac_addr));
            }

            // - udp socket -----------------------------------------------------------

            let store = unsafe { &mut ETHERNET_STORAGE };
            let udp_rx_buffer = UdpSocketBuffer::new(
                &mut store.udp_rx_metadata[..],
                &mut store.udp_rx_buffer_storage[..],
            );
            let udp_tx_buffer = UdpSocketBuffer::new(
                &mut store.udp_tx_metadata[..],
                &mut store.udp_tx_buffer_storage[..],
            );
            let mut udp_socket = UdpSocket::new(udp_rx_buffer, udp_tx_buffer);

            let endpoint_local = IpEndpoint::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 12345);
            let endpoint_remote =
                IpEndpoint::new(Ipv4Address::from_bytes(&IP_REMOTE).into(), IP_REMOTE_PORT);
            if !udp_socket.is_open() {
                udp_socket.bind(endpoint_local).unwrap();
            }

            let udp_socket_handle = unsafe { ETHERNET.as_mut().unwrap().interface.add_socket(udp_socket) };

            // - tcp socket -----------------------------------------------------------

            let store = unsafe { &mut ETHERNET_STORAGE };
            let tcp_rx_buffer = TcpSocketBuffer::new(&mut store.tcp_rx_buffer_storage[..]);
            let tcp_tx_buffer = TcpSocketBuffer::new(&mut store.tcp_tx_buffer_storage[..]);
            let mut tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

            let mut tcp_recv_buffer = [0u8; 2048];
            let tcp_endpoint = IpEndpoint::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 6970);

            let tcp_socket_handle = unsafe { ETHERNET.as_mut().unwrap().interface.add_socket(tcp_socket) };

            let hello_world_u = "HTTP/1.1 200 OK
Content-Type: text/html

<!DOCTYPE html>
<html>
<head>
    <title>Hello, World!</title>
</head>
<body>
    <h1>Hello, World!</h1>
</body>
</html>";

let hello_world = hello_world_u.as_bytes();

            // - timer ----------------------------------------------------------------

            systick_init(&mut cp.SYST, &ccdr.clocks); // 1ms tick
            let mut delay = cp.SYST.delay(ccdr.clocks);

            led_user.set_low();

            let mut tcp_active: bool = false;

            // - main loop ------------------------------------------------------------
            loop {
                log_serial!(tx, "Ethernet main loop\r\n");
                match lan8742a.poll_link() {
                    true => led_link.set_high(),
                    _ => led_link.set_low(),
                }

                let tcp_socket = unsafe {
                    ETHERNET.as_mut().unwrap().interface
                            .get_socket::<TcpSocket>(tcp_socket_handle)
                };

                log_serial!(tx, "Starting to listen\r\n");
                if !tcp_socket.is_open() {
                    log_serial!(tx, "Socket was not open\r\n");
                    tcp_socket.listen(6970).unwrap();
                }
                log_serial!(tx, "Finished listening\r\n");

                if tcp_socket.is_active() && !tcp_active {
                    log_serial!(tx, "tcp connected\r\n");
                } else if tcp_socket.is_active() && tcp_active {
                    log_serial!(tx, "tcp disconnected\r\n");
                }
                tcp_active = tcp_socket.is_active();

                if tcp_socket.may_recv() {
                    tcp_socket.recv(|buffer| {
                        if !buffer.is_empty() {
                            log_serial!(tx, "Recieved {} octets\r\n", buffer.len());
                            for c in &mut *buffer {
                                log_serial!(tx, "{}", *c as char);
                            }
                        }
                        (buffer.len(), ())
                    }).unwrap();
                }

                if tcp_socket.may_send() {
                    log_serial!(tx, "May send tcp\r\n");
                    tcp_socket.send(|hello_world| {
                        if !hello_world.is_empty() {
                            log_serial!(tx, "TCP sending {} octets\r\n", hello_world.len());
                        }
                        (hello_world.len(), ())
                    }).unwrap();
                    tcp_socket.close();
                }
        
                // send a packet
                let udp_socket = unsafe {
                    ETHERNET
                        .as_mut()
                        .unwrap()
                        .interface
                        .get_socket::<UdpSocket>(udp_socket_handle)
                };
                match udp_socket.send_slice("hello there\n".as_bytes(), endpoint_remote) {
                    Ok(()) => log_serial!(tx, "Success sending packet\r\n"),
                    Err(e) => log_serial!(tx, "Error sending packet\r\n"),
                }
        
                delay.delay_ms(2000_u16);
            
            }
        }
        #[cfg(not(core = "0"))]
        () => {
            let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
            let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

            let mut led_yellow = gpioe.pe1.into_push_pull_output();
            let mut led_green = gpiob.pb0.into_push_pull_output();
            let mut led_red = gpiob.pb14.into_push_pull_output();

            let tx = gpiod.pd8.into_alternate();
            let rx = gpiod.pd9.into_alternate();

            let serial = dp
                .USART3
                .serial((tx, rx), 19200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
                .unwrap();

            let (mut tx, mut rx) = serial.split();

            // Get the delay provider.
            let mut delay = cp.SYST.delay(ccdr.clocks);

            loop {
                log_serial!(tx, "Blinky main loop\r\n");
                led_green.set_high();
                delay.delay_ms(5000_u16);
        
                led_green.set_low();
                delay.delay_ms(5000_u16);
            }
        }
    }

}

// - systick ------------------------------------------------------------------

fn systick_init(syst: &mut pac::SYST, clocks: &CoreClocks) {
    let c_ck_mhz = clocks.c_ck().raw() / 1_000_000;
    let syst_calib = 0x3E8;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

// - interrupts and exceptions ------------------------------------------------

#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() };

    if let Some(ethernet) = unsafe { ETHERNET.as_mut() } {
        let time = ATOMIC_TIME.load(Ordering::Relaxed);
        ethernet.poll(time as i64);
    }
}

#[exception]
fn SysTick() {
    ATOMIC_TIME.fetch_add(1, Ordering::Relaxed);
}

// - NetStaticStorage ---------------------------------------------------------

pub struct EthernetStorage<'a> {
    ip_addrs: [IpCidr; 2],
    socket_storage: [SocketStorage<'a>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 2],

    // network buffers
    udp_rx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_tx_metadata: [PacketMetadata<IpEndpoint>; 1],
    udp_rx_buffer_storage: [u8; MAX_UDP_PACKET_SIZE],
    udp_tx_buffer_storage: [u8; MAX_UDP_PACKET_SIZE],
    tcp_rx_buffer_storage: [u8; MAX_UDP_PACKET_SIZE],
    tcp_tx_buffer_storage: [u8; MAX_UDP_PACKET_SIZE],
}

impl<'a> EthernetStorage<'a> {
    pub const fn new() -> Self {
        EthernetStorage {
            ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX); 2],
            socket_storage: [SocketStorage::EMPTY; 8],
            neighbor_cache_storage: [None; 8],
            routes_storage: [None; 2],

            udp_rx_metadata: [UdpPacketMetadata::EMPTY],
            udp_tx_metadata: [UdpPacketMetadata::EMPTY],
            udp_rx_buffer_storage: [0u8; MAX_UDP_PACKET_SIZE],
            udp_tx_buffer_storage: [0u8; MAX_UDP_PACKET_SIZE],
            tcp_rx_buffer_storage: [0u8; MAX_UDP_PACKET_SIZE],
            tcp_tx_buffer_storage: [0u8; MAX_UDP_PACKET_SIZE],
        }
    }
}

// - Net ----------------------------------------------------------------------

pub struct Net<'a> {
    interface: Interface<'a, ethernet::EthernetDMA<'a, 4, 4>>,
}

impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut EthernetStorage<'a>,
        ethdev: ethernet::EthernetDMA<'a, 4, 4>,
        ethernet_addr: EthernetAddress,
    ) -> Self {
        store.ip_addrs = [IpCidr::new(Ipv4Address::from_bytes(&IP_LOCAL).into(), 0); 2];

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