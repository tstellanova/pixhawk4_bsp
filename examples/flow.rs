/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const LOOP_RATE_HZ: u16 = 200;
const LOOP_INTERVAL_MS: u16 = 1000 / LOOP_RATE_HZ;

use pmw3901_ehal::{self, Builder};
use pixhawk4_bsp::board::Board;
use pixhawk4_bsp::peripherals::Spi5CS1;

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

    let flow = pmw3901_ehal::PMW3901::new(
        board.ext_spi.acquire(),
        board.ext_spi_csn.0
    );

    flow.init();

    let mut loop_count: u32 = 0;
    loop {

        rprintln!("baro [{}]: {} ", loop_count, sample.pressure);


        if board.baro.is_some() {
            if let Ok(sample) =
                board.baro.as_mut().unwrap().get_second_order_sample(
                    Oversampling::OS_2048,
                    &mut board.delay_source,
                )
            {
                rprintln!("baro [{}]: {} ", loop_count, sample.pressure);
            }
        }

        let _ = board.user_leds.0.toggle();
        let _ = board.user_leds.1.toggle();
        board.delay_source.delay_ms(loop_interval);

        loop_count += 1;
    }
}
