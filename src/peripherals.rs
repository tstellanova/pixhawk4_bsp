/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use stm32f7xx_hal as p_hal;
use p_hal::pac as pac;

use pac::{I2C1, USART1};

// use p_hal::flash::FlashExt;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::GpioExt;
use p_hal::rcc::{HSEClock, HSEClockMode, RccExt};
use p_hal::time::{Hertz, U32Ext};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use stm32f7xx_hal::dma::DMA;

//  Main FMU Processor: STM32F765
// 32 Bit Arm® Cortex®-M7, 216MHz, 2MB memory, 512KB RAM
// IO Processor: STM32F100
// 32 Bit Arm® Cortex®-M3, 24MHz, 8KB SRAM

/// Initialize peripherals for Pixracer.
/// Pixhawk4 FMU chip is stm32f765 216 MHz
pub fn setup() -> (
    UserLeds,
    DelaySource,
    GpsPortUart,
    I2c1Port,
    Spi1Port,
    Spi2Port,
    Spi4Port,
    Spi5Port,
    SpiPins6Dof, // 6dof
    SpiPinsMag,  // mag
    SpiCsBaro,   // baro
    SpiCsFram, // ferro ram
    Spi1PowerEnable,
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();
    let gpiof = dp.GPIOF.split();
    let gpiog = dp.GPIOG.split();
    let gpioi = dp.GPIOI.split();

    let user_led1 = gpiob.pb1.into_push_pull_output(); //red
    let user_led2 = gpioc.pc6.into_push_pull_output(); //green
    let user_led3 = gpioc.pc7.into_push_pull_output(); //blue

    // SPI1 connects to internal sensors
    let spi1_port: Spi1Port = {
        let sck = gpiog.pg11.into_alternate_af5();
        let cipo = gpioa.pa6.into_alternate_af5();
        let copi = gpiod.pd7.into_alternate_af5();

        p_hal::spi::Spi::new(dp.SPI1,(sck, cipo, copi)).enable::<u8>(
            &mut rcc,
            p_hal::spi::ClockDivider::DIV32,
            embedded_hal::spi::MODE_3
        )
    };

    // SPI2 connects to FRAM
    let spi2_port: Spi2Port = {
        let sck = gpioi.pi1.into_alternate_af5();
        let cipo = gpioi.pi2.into_alternate_af5();
        let copi = gpioi.pi3.into_alternate_af5();

        p_hal::spi::Spi::new( dp.SPI2,(sck, cipo, copi)).enable::<u8>(
            &mut rcc,
            p_hal::spi::ClockDivider::DIV32, //TODO s/b 20 MHz
            embedded_hal::spi::MODE_3
        )
    };

    // SPI4 connects to internal barometer only
    let spi4_port: Spi4Port = {
        let sck = gpioe.pe2.into_alternate_af5();
        let cipo = gpioe.pe13.into_alternate_af5();
        let copi = gpioe.pe6.into_alternate_af5();

        p_hal::spi::Spi::new(
            dp.SPI4,
            (sck, cipo, copi)).enable::<u8>(
            &mut rcc,
            p_hal::spi::ClockDivider::DIV32, //TODO s/b 20 MHz
            embedded_hal::spi::MODE_3
        )
    };

    let spi5_port: Spi5Port = {
        let sck = gpiof.pf7.into_alternate_af5();
        let cipo = gpiof.pf8.into_alternate_af5();
        let copi = gpiof.pf9.into_alternate_af5();

        p_hal::spi::Spi::new(
            dp.SPI5,
            (sck, cipo, copi)).enable::<u8>(
            &mut rcc,
            p_hal::spi::ClockDivider::DIV32, //TODO s/b 20 MHz
            embedded_hal::spi::MODE_3
        )
    };

    // this sucks, but we need to delay capturing clocks until we've used rcc for SPI...for some reason
    let mut clocks = rcc
        .cfgr
        .hse(HSEClock::new(16.mhz(), HSEClockMode::Oscillator)) // 16 MHz xtal
        .sysclk(216.mhz()) // HCLK
        .timclk1(54.mhz()) // APB1 clock (PCLK1) is HCLK/4
        .timclk2(108.mhz()) // APB2 clock (PCLK2) is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // let mut rand_source = dp.RNG.constrain(clocks);
    // USART1 is GPS port:
    let gps1_port = {
        let rx = gpiob.pb7.into_alternate_af7();
        let tx = gpiob.pb6.into_alternate_af7();
        p_hal::serial::Serial::new(dp.USART1, (tx, rx), clocks, p_hal::serial::Config::default())
    };

    let i2c1_port: I2c1Port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1,
                              (scl, sda),
                              p_hal::i2c::Mode::fast(400.khz()),
                              clocks,
                              &mut rcc.apb1
                              )
    };

    // SPI chip select and data ready pins

    // // ICM20602 or ICM20608G
    let mut spi_cs_6dof = gpioc.pc15.into_push_pull_output();
    let _ = spi_cs_6dof.set_high();
    let spi_drdy_6dof = gpioc.pc14.into_pull_up_input();

    // HMC5883 (hmc5983) or LIS3MDL
    let mut spi_cs_mag = gpioe.pe15.into_push_pull_output();
    let _ = spi_cs_mag.set_high();
    let spi_drdy_mag = gpioe.pe12.into_pull_up_input();

    let mut spi_cs_baro = gpiof.pf10.into_push_pull_output();
    let _ = spi_cs_baro.set_high();

    let mut spi_cs_fram: SpiCsFram = gpiod.pd10.into_push_pull_output();
    let _ = spi_cs_fram.set_high();

    //enables power to spi1 bus devices
    let spi1_power_enable = gpioe.pe3.into_push_pull_output();

    //TODO setup ports & pins for these devices:

    (
        (user_led1, user_led2, user_led3),
        delay_source,
        gps1_port,
        i2c1_port,
        spi1_port,
        spi2_port,
        spi4_port,
        spi5_port,
        (spi_cs_6dof, spi_drdy_6dof),
        (spi_cs_mag, spi_drdy_mag),
        spi_cs_baro,
        spi_cs_fram,
        spi1_power_enable,
    )

}

pub type I2c1Port = p_hal::i2c::I2c<
    I2C1,
    p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
    p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
>;

/// The AF pin type used for SPI peripherals on this MCU
pub type SpiDataPin = p_hal::gpio::Alternate<p_hal::gpio::AF5>;

/// Internal SPI1 bus: connects to various onboard sensors
pub type Spi1Port = p_hal::spi::Spi<
    pac::SPI1,
    (
        p_hal::gpio::gpiog::PG11<SpiDataPin>, //SCLK
        p_hal::gpio::gpioa::PA6<SpiDataPin>, //CIPO
        p_hal::gpio::gpiod::PD7<SpiDataPin>, //COPI
    ),
    p_hal::spi::Enabled<u8>,
>;

/// Internal SPI2 bus: connects to FRAM
pub type Spi2Port = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpioi::PI1<SpiDataPin>, //SCLK
        p_hal::gpio::gpioi::PI2<SpiDataPin>, //CIPO
        p_hal::gpio::gpioi::PI3<SpiDataPin>, //COPI
    ),
    p_hal::spi::Enabled<u8>,
>;

/// Internal SPI4 bus: connects to internal barometer only
pub type Spi4Port = p_hal::spi::Spi<
    pac::SPI4,
    (
        p_hal::gpio::gpioe::PE2<SpiDataPin>,  //SCLK
        p_hal::gpio::gpioe::PE13<SpiDataPin>, //CIPO
        p_hal::gpio::gpioe::PE6<SpiDataPin>,  //COPI
    ),
    p_hal::spi::Enabled<u8>,
>;

///External port marked "SPI":  is SPI5:
/// Pinout left-right: (Vcc, SCK, CIPO, COPI, CS1, CS2, GND)
pub type Spi5Port = p_hal::spi::Spi<
    pac::SPI5,
    (
        p_hal::gpio::gpiof::PF7<SpiDataPin>, //SCLK
        p_hal::gpio::gpiof::PF8<SpiDataPin>, //CIPO
        p_hal::gpio::gpiof::PF9<SpiDataPin>, //COPI
    ),
    p_hal::spi::Enabled<u8>,
>;


// pub type SpiPinsImu = (
//     p_hal::gpio::gpioc::PC2<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
//     p_hal::gpio::gpiod::PD15<p_hal::gpio::Input<p_hal::gpio::PullUp>>,
// );

pub type SpiCs6Dof = p_hal::gpio::gpioc::PC15<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
pub type SpiPins6Dof = (
    SpiCs6Dof, //CSN
    p_hal::gpio::gpioc::PC14<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DRDY
);


pub type SpiCsMag =
    p_hal::gpio::gpioe::PE15<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
pub type SpiPinsMag = (
    SpiCsMag, //CSN
    p_hal::gpio::gpioe::PE12<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DRDY
);

pub type SpiCsBaro =
    p_hal::gpio::gpiof::PF10<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
pub type SpiCsFram =
    p_hal::gpio::gpiod::PD10<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

/// Enables power on the SPI1 bus
pub type Spi1PowerEnable =
    p_hal::gpio::gpioe::PE3<p_hal::gpio::Output<p_hal::gpio::PushPull>>;


type Usart1PortType = p_hal::serial::Serial<
    USART1,
    (
        p_hal::gpio::gpiob::PB6<p_hal::gpio::Alternate<p_hal::gpio::AF7>>, //tx
        p_hal::gpio::gpiob::PB7<p_hal::gpio::Alternate<p_hal::gpio::AF7>>, //rx
    ),
>;



//GPS1: /dev/ttyS0 ?
/// External port marked "GPS MODULE" combines this UART
pub type GpsPortUart = Usart1PortType;

pub type Dma1Type = DMA<pac::DMA1>;

pub type DelaySource = p_hal::delay::Delay;

pub type UserLedOutPin = p_hal::gpio::Output<p_hal::gpio::PushPull>;
pub type UserLed1 =     p_hal::gpio::gpiob::PB1<UserLedOutPin>;
pub type UserLed2 =     p_hal::gpio::gpioc::PC6<UserLedOutPin>;
pub type UserLed3 =     p_hal::gpio::gpioc::PC7<UserLedOutPin>;

pub type UserLeds = (UserLed1, UserLed2, UserLed3);