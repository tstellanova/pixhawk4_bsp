/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use stm32f7xx_hal as p_hal;
use p_hal::device as pac;

use pac::I2C1;

// use p_hal::flash::FlashExt;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;
use p_hal::time::{Hertz, U32Ext};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
// use p_hal::pwm;
// use stm32f4xx_hal::rng::{RngExt, Rng};
// use pac::I2C1;

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
    impl DelayMs<u8>,
    // Rng,
    I2C1PortType,
    Spi1PortType,
    Spi2PortType,
    // SpiPinsImu,  // imu
    SpiPins6Dof, // 6dof
    SpiPinsMag,  // mag
    SpiCsBaro,   // baro
    SpiCsFram, // ferro ram
    Spi1PowerEnable,
    // Tim1PwmChannels,
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let mut rcc = dp.RCC.constrain();
    //    rcc::{HSEClock, HSEClockMode},

    let clocks = rcc
        .cfgr
        .hse(HSEClock::new(16.mhz(), HSEClockMode::Oscillator)) // 16 MHz xtal
        .sysclk(216.mhz()) // HCLK
        .timclk1(54.mhz()) // APB1 clock (PCLK1) is HCLK/4
        .timclk2(108.mhz()) // APB2 clock (PCLK2) is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);
    // let mut rand_source = dp.RNG.constrain(clocks);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();
    let gpiog = dp.GPIOG.split();
    let gpioi = dp.GPIOI.split();


    let user_led1 = gpiob.pb1.into_push_pull_output(); //red
    let user_led2 = gpioc.pc6.into_push_pull_output(); //green
    let user_led3 = gpioc.pc7.into_push_pull_output(); //blue


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

    let spi1_port = {
        let sck = gpiog.pg11.into_alternate_af5();
        let miso = gpioa.pa6.into_alternate_af5();
        let mosi = gpiod.pd7.into_alternate_af5();

        p_hal::spi::Spi::new(
            dp.SPI1,
            (sck, miso, mosi)).enable::<u8>(
                &mut rcc,
                p_hal::spi::ClockDivider::DIV32,
            p_hal::spi::Mode::MODE_3,
        );

    };


    let spi2_port = {
        let sck = gpioi.pi1.into_alternate_af5();
        let miso = gpioi.pi2.into_alternate_af5();
        let mosi = gpioi.pi3.into_alternate_af5();

        p_hal::spi::Spi::spi2(
            dp.SPI2,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_3,
            20_000_000.hz(),
            clocks,
        )
    };

    let spi4_port = {
        let sck = gpioe.pe2.into_alternate_af5();
        let miso = gpioe.pe13.into_alternate_af5();
        let mosi = gpioe.pe6.into_alternate_af5();

        p_hal::spi::Spi::spi4(
            dp.SPI4,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_3,
            20_000_000.hz(),
            clocks,
        )
    };

    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), p_hal::i2c::Mode::fast(400.khz()), clocks, 0)
    };

    // SPI chip select and data ready pins

    // ICM20602 or ICM20608G
    let mut spi_cs_6dof = gpioc.pc15.into_push_pull_output();
    let _ = spi_cs_6dof.set_high();
    let spi_drdy_6dof = gpioc.pc14.into_pull_up_input();

    // HMC5883 (hmc5983) or LIS3MDL
    let mut spi_cs_mag = gpioe.pe15.into_push_pull_output();
    let _ = spi_cs_mag.set_high();
    let spi_drdy_mag = gpioe.pe12.into_pull_up_input();

    let mut spi_cs_baro = gpiod.pd7.into_push_pull_output();
    let _ = spi_cs_baro.set_high();

    let mut spi_cs_fram = gpiod.pd10.into_push_pull_output();
    let _ = spi_cs_fram.set_high();

    //enables power to spi1 bus devices
    let spi1_power_enable = gpioe.pe3.into_push_pull_output();

    //TODO setup ports & pins for these devices:

    // --- SPI1 ---
    // Invensense® ICM-20608 Accel / Gyro (4 KHz)
    // MPU9250 Accel / Gyro / Mag (4 KHz)
    // HMC5983 magnetometer with temperature compensation
    // --- SPI2 ---
    // MS5611 barometer Measurement Specialties MS5611 barometer on internal SPI (spi2??)
    // SPI microsd
    // FRAM? spi2, cs
    // --- ??? ---
    // RC port (s.bus ?) for FrSky
    // Pixracer R15 has LIS3MDL for mag

    // PWM output pins
    // pixracer actually provides six pwm pins, but for now we only setup four
    // because stm32f4xx_hal lacks type erasure for the GPIO port letter,
    // which means we can't create a heterogeneous array of gpioe and gpiod pins.

    // TODO support final two PWM pins (on gpiod):
    // gpiod.pd13.into_push_pull_output().downgrade(), // TIM4_CH2
    // gpiod.pd14.into_push_pull_output().downgrade() // TIM4_CH3

    // Note that this channel order is different from the external pin order
    // let pwm_pins = (
    //     gpioe.pe9.into_alternate_af1(), // TIM1_CH1 -> pin4 ?
    //     gpioe.pe11.into_alternate_af1(), // TIM1_CH2 -> pin3 ?
    //     gpioe.pe13.into_alternate_af1(), // TIM1_CH3 -> pin2 ?
    //     gpioe.pe14.into_alternate_af1(), // TIM1_CH4 -> pin1 ?
    // );

    // //TODO we use 400 Hz by default for PWM output...may be able to drive faster with some ESCs
    // let pwm_tim1_channels = pwm::tim1(dp.TIM1, pwm_pins, clocks, 400.hz());


    (
        (user_led1, user_led2, user_led3),
        delay_source,
        // rand_source,
        i2c1_port,
        spi1_port,
        spi2_port,
        // (spi_cs_imu, spi_drdy_imu),
        (spi_cs_6dof, spi_drdy_6dof),
        (spi_cs_mag, spi_drdy_mag),
        spi_cs_baro,
        spi_cs_fram,
        spi1_power_enable,
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
    p_hal::spi::Enabled<u8>
>;

pub type Spi2PortType = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpiob::PB10<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpiob::PB14<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpiob::PB15<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
    p_hal::spi::Enabled<u8>
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
