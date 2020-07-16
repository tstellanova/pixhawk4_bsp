/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

//! This example exercises the PMW3901 optical flow sensor
//! attached to the Pixhawk 4's external SPI port.
//!

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const LOOP_RATE_HZ: u16 = 1;
const LOOP_INTERVAL_MS: u16 = 1000 / LOOP_RATE_HZ;

use pmw3901_ehal::{self, PMW3901};
use pixhawk4_bsp::board::Board;

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

    let mut board = Board::new();

    let loop_interval = LOOP_INTERVAL_MS as u8;
    rprintln!("min loop_interval: {}", loop_interval);

    let _ = board.user_leds.0.set_high();
    let _ = board.user_leds.1.set_low();
    let _ = board.user_leds.2.set_high();

    let mut flow = PMW3901::new(
        board.ext_spi.acquire(),
        board.ext_spi_csn.0
    );
    flow.init(&mut board.delay_source).expect("could not init flow");

    loop {
        if let Ok(sample) = flow.get_motion() {
            rprintln!("flow {:?}",sample);
        }
        let _ = board.user_leds.0.toggle();
        let _ = board.user_leds.1.toggle();
        let _ = board.user_leds.2.toggle();
        board.delay_source.delay_ms(loop_interval);
    }
}
