//! ENC28J60 demo: pong server + UDP echo server
//!
//! This program:
//!
//! - Responds to ARP requests
//! - Responds to ICMP echo requests, thus you can `ping` the device
//! - Responds to *all* UDP datagrams by sending them back
//!
//! You can test this program by running the following commands:
//!
//! - `ping 192.168.1.33`. The device should respond and toggle the state of the LED on every `ping`
//! request.
//! - `nc -u 192.168.1.33 1337` and sending any string. The device should respond back by sending
//! back the received string; the LED will toggle each time a UDP datagram is sent.

#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(nll)]
#![no_std]
#![no_main]

extern crate cortex_m_rt as rt;
#[macro_use]
extern crate cortex_m;
extern crate enc28j60;
extern crate heapless;
extern crate jnet;
extern crate panic_itm;
extern crate stm32f1xx_hal as hal;

use enc28j60::Enc28j60;
use hal::delay::Delay;
use hal::prelude::*;
use hal::spi::Spi;
use hal::stm32f103xx;
use heapless::consts::*;
use heapless::FnvIndexMap;
use jnet::{arp, ether, icmp, ipv4, mac, udp, Buffer};
use rt::{entry, exception, ExceptionFrame};

// uncomment to disable tracing
// macro_rules! iprintln {
//     ($($tt: tt)*) => {};
// }

/* Configuration */
const MAC: mac::Addr = mac::Addr([0x20, 0x18, 0x03, 0x01, 0x00, 0x00]);
const IP: ipv4::Addr = ipv4::Addr([192, 168, 1, 33]);

/* Constants */
const KB: u16 = 1024; // bytes

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f103xx::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let _stim = &mut cp.ITM.stim[0];

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    cp.DWT.enable_cycle_counter();

    // LED
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // turn the LED off during initialization
    led.set_high();

    // SPI
    let mut ncs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    ncs.set_high();
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        enc28j60::MODE,
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    // ENC28J60
    let mut reset = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    reset.set_high();
    let mut delay = Delay::new(cp.SYST, clocks);
    let mut enc28j60 = Enc28j60::new(
        spi,
        ncs,
        enc28j60::Unconnected,
        reset,
        &mut delay,
        7 * KB,
        MAC.0,
    )
    .ok()
    .unwrap();

    // LED on after initialization
    led.set_low();

    // FIXME some frames are lost when sent right after initialization
    delay.delay_ms(100_u8);

    // ARP cache
    let mut cache = FnvIndexMap::<_, _, U8>::new();

    let mut buf = [0; 256];
    loop {
        let mut buf = Buffer::new(&mut buf);
        let len = enc28j60.receive(buf.as_mut()).ok().unwrap();
        buf.truncate(len);

        if let Ok(mut eth) = ether::Frame::parse(buf) {
            iprintln!(_stim, "\nRx({})", eth.as_bytes().len());
            iprintln!(_stim, "* {:?}", eth);

            let src_mac = eth.get_source();

            match eth.get_type() {
                ether::Type::Arp => {
                    if let Ok(arp) = arp::Packet::parse(eth.payload_mut()) {
                        match arp.downcast() {
                            Ok(mut arp) => {
                                iprintln!(_stim, "** {:?}", arp);

                                if !arp.is_a_probe() {
                                    cache.insert(arp.get_spa(), arp.get_sha()).ok();
                                }

                                // are they asking for us?
                                if arp.get_oper() == arp::Operation::Request && arp.get_tpa() == IP
                                {
                                    // reply to the ARP request
                                    let tha = arp.get_sha();
                                    let tpa = arp.get_spa();

                                    arp.set_oper(arp::Operation::Reply);
                                    arp.set_sha(MAC);
                                    arp.set_spa(IP);
                                    arp.set_tha(tha);
                                    arp.set_tpa(tpa);
                                    iprintln!(_stim, "\n** {:?}", arp);
                                    let arp_len = arp.len();

                                    // update the Ethernet header
                                    eth.set_destination(tha);
                                    eth.set_source(MAC);
                                    eth.truncate(arp_len);
                                    iprintln!(_stim, "* {:?}", eth);

                                    iprintln!(_stim, "Tx({})", eth.as_bytes().len());
                                    enc28j60.transmit(eth.as_bytes()).ok().unwrap();
                                }
                            }
                            Err(_arp) => {
                                // Not a Ethernet/IPv4 ARP packet
                                iprintln!(_stim, "** {:?}", _arp);
                            }
                        }
                    } else {
                        // malformed ARP packet
                        iprintln!(_stim, "Err(A)");
                    }
                }
                ether::Type::Ipv4 => {
                    if let Ok(mut ip) = ipv4::Packet::parse(eth.payload_mut()) {
                        iprintln!(_stim, "** {:?}", ip);

                        let src_ip = ip.get_source();

                        if !src_mac.is_broadcast() {
                            cache.insert(src_ip, src_mac).ok();
                        }

                        match ip.get_protocol() {
                            ipv4::Protocol::Icmp => {
                                if let Ok(icmp) = icmp::Packet::parse(ip.payload_mut()) {
                                    match icmp.downcast::<icmp::EchoRequest>() {
                                        Ok(request) => {
                                            // is an echo request
                                            iprintln!(_stim, "*** {:?}", request);

                                            let src_mac = cache
                                                .get(&src_ip)
                                                .unwrap_or_else(|| unimplemented!());

                                            let _reply: icmp::Packet<_, icmp::EchoReply, _> =
                                                    request.into();
                                            iprintln!(_stim, "\n*** {:?}", _reply);

                                            // update the IP header
                                            let mut ip = ip.set_source(IP);
                                            ip.set_destination(src_ip);
                                            let _ip = ip.update_checksum();
                                            iprintln!(_stim, "** {:?}", _ip);

                                            // update the Ethernet header
                                            eth.set_destination(*src_mac);
                                            eth.set_source(MAC);
                                            iprintln!(_stim, "* {:?}", eth);

                                            led.toggle();
                                            iprintln!(_stim, "Tx({})", eth.as_bytes().len());
                                            enc28j60.transmit(eth.as_bytes()).ok().unwrap();
                                        }
                                        Err(_icmp) => {
                                            iprintln!(_stim, "*** {:?}", _icmp);
                                        }
                                    }
                                } else {
                                    // Malformed ICMP packet
                                    iprintln!(_stim, "Err(B)");
                                }
                            }
                            ipv4::Protocol::Udp => {
                                if let Ok(mut udp) = udp::Packet::parse(ip.payload_mut()) {
                                    iprintln!(_stim, "*** {:?}", udp);

                                    if let Some(src_mac) = cache.get(&src_ip) {
                                        let src_port = udp.get_source();
                                        let dst_port = udp.get_destination();

                                        // update the UDP header
                                        udp.set_source(dst_port);
                                        udp.set_destination(src_port);
                                        udp.zero_checksum();
                                        iprintln!(_stim, "\n*** {:?}", udp);

                                        // update the IP header
                                        let mut ip = ip.set_source(IP);
                                        ip.set_destination(src_ip);
                                        let ip = ip.update_checksum();
                                        let ip_len = ip.len();
                                        iprintln!(_stim, "** {:?}", ip);

                                        // update the Ethernet header
                                        eth.set_destination(*src_mac);
                                        eth.set_source(MAC);
                                        eth.truncate(ip_len);
                                        iprintln!(_stim, "* {:?}", eth);

                                        led.toggle();
                                        iprintln!(_stim, "Tx({})", eth.as_bytes().len());
                                        enc28j60.transmit(eth.as_bytes()).ok().unwrap();
                                    }
                                } else {
                                    // malformed UDP packet
                                    iprintln!(_stim, "Err(C)");
                                }
                            }
                            _ => {}
                        }
                    } else {
                        // malformed IPv4 packet
                        iprintln!(_stim, "Err(D)");
                    }
                }
                _ => {}
            }
        } else {
            // malformed Ethernet frame
            iprintln!(_stim, "Err(E)");
        }
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
