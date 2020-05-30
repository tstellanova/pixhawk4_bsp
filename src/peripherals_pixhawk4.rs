/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use p_hal::device as pac;
use stm32f7xx_hal as p_hal;

use pac::{I2C1, USART1};

// use p_hal::flash::FlashExt;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::GpioExt;
use p_hal::rcc::{HSEClock, HSEClockMode, RccExt};
use p_hal::time::{Hertz, U32Ext};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// Initialize peripherals for Pixracer.
/// Pixhawk4 FMU chip is stm32f765 216 MHz
pub fn setup_peripherals() -> (
    // LED output pins
    (
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
    ),
    // delay source
    impl DelayMs<u8> + DelayUs<u32>,
    Gps1PortType,
    // // Rng,
    // I2C1PortType,
    // Spi1PortType,
    // Spi2PortType,
    // // SpiPinsImu,  // imu
    // SpiPins6Dof, // 6dof
    // SpiPinsMag,  // mag
    // SpiCsBaro,   // baro
    // SpiCsFram, // ferro ram
    // Spi1PowerEnable,
    // Tim1PwmChannels,
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .hse(HSEClock::new(16.mhz(), HSEClockMode::Oscillator)) // 16 MHz xtal
        .sysclk(216.mhz()) // HCLK
        .timclk1(54.mhz()) // APB1 clock (PCLK1) is HCLK/4
        .timclk2(108.mhz()) // APB2 clock (PCLK2) is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);
    // let mut rand_source = dp.RNG.constrain(clocks);

    // let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();
    // let gpioe = dp.GPIOE.split();
    // let gpiog = dp.GPIOG.split();
    // let gpioi = dp.GPIOI.split();

    let user_led1 = gpiob.pb1.into_push_pull_output(); //red
    let user_led2 = gpioc.pc6.into_push_pull_output(); //green
    let user_led3 = gpioc.pc7.into_push_pull_output(); //blue

    // USART1 is GPS port:
    // let gps_port = {
    //     let config = p_hal::serial::config::Config::default().baudrate(115200.bps());
    //     let rx = gpiob.pb7.into_alternate_af7();
    //     let tx = gpiob.pb6.into_alternate_af7();
    //     dp.USART1.usart((tx, rx), config, &mut ccdr).unwrap()
    // };

    let gps1_port = {
        let rx = gpiob.pb7.into_alternate_af7();
        let tx = gpiob.pb6.into_alternate_af7();
        p_hal::serial::Serial::new(
            dp.USART1,
            (tx, rx),
            clocks,
            p_hal::serial::Config {
                baud_rate: 115_200.bps(),
                oversampling: p_hal::serial::Oversampling::By8,
            },
        )
    };

    // SPI1 connects to internal sensors
    // SPI2 connects to FRAM
    // SPI4 connects to internal barometer only

    //    let mut spi = Spi::new(p.SPI3, (sck, miso, mosi)).enable::<u8>(
    //         &mut rcc,
    //         spi::ClockDivider::DIV32,
    //         spi::Mode {
    //             polarity: spi::Polarity::IdleHigh,
    //             phase: spi::Phase::CaptureOnSecondTransition,
    //         },
    //     );

    // let spi1_port = {
    //     let sck = gpiog.pg11.into_alternate_af5();
    //     let miso = gpioa.pa6.into_alternate_af5();
    //     let mosi = gpiod.pd7.into_alternate_af5();
    //
    //     p_hal::spi::Spi::new(
    //         dp.SPI1,
    //         (sck, miso, mosi)).enable::<u8>(
    //             &mut rcc,
    //             p_hal::spi::ClockDivider::DIV32,
    //         p_hal::spi::Mode::MODE_3,
    //     );
    // };

    // let spi2_port = {
    //     let sck = gpioi.pi1.into_alternate_af5();
    //     let miso = gpioi.pi2.into_alternate_af5();
    //     let mosi = gpioi.pi3.into_alternate_af5();
    //
    //     p_hal::spi::Spi::spi2(
    //         dp.SPI2,
    //         (sck, miso, mosi),
    //         embedded_hal::spi::MODE_3,
    //         20_000_000.hz(),
    //         clocks,
    //     )
    // };
    //
    // let spi4_port = {
    //     let sck = gpioe.pe2.into_alternate_af5();
    //     let miso = gpioe.pe13.into_alternate_af5();
    //     let mosi = gpioe.pe6.into_alternate_af5();
    //
    //     p_hal::spi::Spi::spi4(
    //         dp.SPI4,
    //         (sck, miso, mosi),
    //         embedded_hal::spi::MODE_3,
    //         20_000_000.hz(),
    //         clocks,
    //     )
    // };
    //
    // let i2c1_port = {
    //     let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
    //     let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
    //     p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), p_hal::i2c::Mode::fast(400.khz()), clocks, 0)
    // };

    // SPI chip select and data ready pins

    // // ICM20602 or ICM20608G
    // let mut spi_cs_6dof = gpioc.pc15.into_push_pull_output();
    // let _ = spi_cs_6dof.set_high();
    // let spi_drdy_6dof = gpioc.pc14.into_pull_up_input();
    //
    // // HMC5883 (hmc5983) or LIS3MDL
    // let mut spi_cs_mag = gpioe.pe15.into_push_pull_output();
    // let _ = spi_cs_mag.set_high();
    // let spi_drdy_mag = gpioe.pe12.into_pull_up_input();
    //
    // let mut spi_cs_baro = gpiod.pd7.into_push_pull_output();
    // let _ = spi_cs_baro.set_high();
    //
    // let mut spi_cs_fram = gpiod.pd10.into_push_pull_output();
    // let _ = spi_cs_fram.set_high();
    //
    // //enables power to spi1 bus devices
    // let spi1_power_enable = gpioe.pe3.into_push_pull_output();

    //TODO setup ports & pins for these devices:

    (
        (user_led1, user_led2, user_led3),
        delay_source,
        gps1_port,
        // rand_source,
        // i2c1_port,
        // spi1_port,
        // spi2_port,
        // (spi_cs_imu, spi_drdy_imu),
        // (spi_cs_6dof, spi_drdy_6dof),
        // (spi_cs_mag, spi_drdy_mag),
        // spi_cs_baro,
        // spi_cs_fram,
        // spi1_power_enable,
        // pwm_tim1_channels
    )
}

pub type I2C1PortType = p_hal::i2c::I2c<
    I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
    ),
>;

pub type Spi1PortType = p_hal::spi::Spi<
    pac::SPI1,
    (
        p_hal::gpio::gpioa::PA5<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpioa::PA6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpioa::PA7<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
    p_hal::spi::Enabled<u8>,
>;

pub type Spi2PortType = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpiob::PB10<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpiob::PB14<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpiob::PB15<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
    p_hal::spi::Enabled<u8>,
>;

// pub type SpiPinsImu = (
//     p_hal::gpio::gpioc::PC2<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
//     p_hal::gpio::gpiod::PD15<p_hal::gpio::Input<p_hal::gpio::PullUp>>,
// );
pub type SpiPins6Dof = (
    p_hal::gpio::gpioc::PC15<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
    p_hal::gpio::gpioc::PC14<p_hal::gpio::Input<p_hal::gpio::PullUp>>,
);
pub type SpiPinsMag = (
    p_hal::gpio::gpioe::PE15<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
    p_hal::gpio::gpioe::PE12<p_hal::gpio::Input<p_hal::gpio::PullUp>>,
);

pub type SpiCsBaro =
    p_hal::gpio::gpiod::PD7<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
pub type SpiCsFram =
    p_hal::gpio::gpiod::PD10<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

pub type Spi1PowerEnable =
    p_hal::gpio::gpioe::PE3<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

//
// pub type Tim1PwmChannels = (
//     p_hal::pwm::PwmChannels<pac::TIM1, p_hal::pwm::C1>,
//     p_hal::pwm::PwmChannels<pac::TIM1, p_hal::pwm::C2>,
//     p_hal::pwm::PwmChannels<pac::TIM1, p_hal::pwm::C3>,
//     p_hal::pwm::PwmChannels<pac::TIM1, p_hal::pwm::C4>
// );

type Usart1PortType = p_hal::serial::Serial<
    USART1,
    (
        p_hal::gpio::gpiob::PB6<p_hal::gpio::Alternate<p_hal::gpio::AF7>>, //tx
        p_hal::gpio::gpiob::PB7<p_hal::gpio::Alternate<p_hal::gpio::AF7>>, //rx
    ),
>;

//stm32f7xx_hal::serial::Serial<
// stm32f7::stm32f765::USART1,
// (
// stm32f7xx_hal::gpio::gpiob::PB6<stm32f7xx_hal::gpio::Alternate<stm32f7xx_hal::gpio::AF7>>,
// stm32f7xx_hal::gpio::gpiob::PB7<stm32f7xx_hal::gpio::Alternate<stm32f7xx_hal::gpio::AF7>>
// )>`

//GPS1: /dev/ttyS0 ?
pub type Gps1PortType = Usart1PortType;
