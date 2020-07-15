use crate::peripherals::*;

use core::ops::DerefMut;
use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::interrupt::{self, Mutex};
use cortex_m::singleton;

/// Onboard sensors
use hmc5983::HMC5983;
use icm20689::{Builder, ICM20689};
use ms5611_spi::Ms5611;

use crate::peripherals;
use core::cell::RefCell;


use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::PwmPin;


use shared_bus::BusMutex;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// The main Board type for this hardware
/// TODO: handle R12 / R15 differences in onboard sensors
pub struct Board<'a> {
    pub user_leds: UserLeds,
    pub delay_source: DelaySource,
    pub ext_i2c1: I2c1BusManager,
    pub mag: Option<InternalMagnetometer<'a>>,
    pub six_dof: Option<Internal6Dof<'a>>,
    pub baro: Option<InternalBarometer<'a>>,
    pub fram: Option<InternalFram<'a>>,
}

impl Board<'_> {
    pub fn new() -> Self {
        let (
            user_leds,
            mut delay_source,
            _gps1_port,
            i2c1_port,
            // spi1_port,
            spi2_port,
            _spi4_port,
            (spi_cs_6dof, _spi_drdy_6dof),
            (spi_cs_mag, _spi_drdy_mag),
            spi_cs_baro,
            spi_cs_fram,
            mut spi1_power_enable,
        ) = peripherals::setup();

        // let spi1_bus_mgr: &'static mut Spi1BusManager =
        //     singleton!(:Spi1BusManager =
        //         shared_bus::CortexMBusManager::new(spi1_port)
        //     )
        //     .unwrap();

        let spi2_bus_mgr: &'static mut Spi2BusManager =
            singleton!(:Spi2BusManager =
                shared_bus::CortexMBusManager::new(spi2_port)
            )
            .unwrap();

        let mut i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

        //TOOD use ist8310 instead of HMC5983??
        // let mut mag_int_opt = {
        //     let mut mag_int = HMC5983::new_with_interface(
        //         hmc5983::interface::SpiInterface::new(spi1_bus_mgr.acquire(), spi_cs_mag),
        //     );
        //     let rc = mag_int.init(&mut delay_source);
        //     if mag_int.init(&mut delay_source).is_ok() {
        //         Some(mag_int)
        //     }
        //     else {
        //         #[cfg(feature = "rttdebug")]
        //         rprintln!("mag setup fail: {:?}" , rc);
        //         None
        //     }
        // };


        // let tdk_6dof_opt = {
        //     let mut tdk_6dof =
        //         icm20689::Builder::new_spi(spi1_bus_mgr.acquire(), spi_cs_6dof);
        //     let rc = tdk_6dof.setup(&mut delay_source);
        //     if rc.is_ok() {
        //         Some(tdk_6dof)
        //     } else {
        //         #[cfg(feature = "rttdebug")]
        //         rprintln!("6dof setup failed: {:?}", rc);
        //         None
        //     }
        // };

        let mut fram_opt = {
            let proxy = spi2_bus_mgr.acquire();
            let rc = spi_memory::series25::Flash::init(proxy, spi_cs_fram);
            if let Ok(fram) = rc {
                Some(fram)
            } else {
                #[cfg(feature = "rttdebug")]
                rprintln!("fram setup failed");
                None
            }
        };

        // if fram_opt.is_some() {
        //     let flosh = fram_opt.as_mut().unwrap();
        //     if let Ok(ident) = flosh.read_jedec_id() {
        //         #[cfg(feature = "rttdebug")]
        //         rprintln!("FRAM ident: {:?}", ident);
        //         // Identification([c2, 22, 00])
        //         // maybe FM25V02-G per ramtron:
        //         // F-RAM 256 kilobit (32K x 8 bit = 32 kilobytes)
        //         // dump_fram(flosh, &mut delay_source);
        //     }
        // }

        // power cycle spi devices
        let _ = spi1_power_enable.set_low();
        delay_source.delay_ms(250u8);
        let _ = spi1_power_enable.set_high();
        // wait a bit for sensors to power up
        delay_source.delay_ms(250u8);

        // // setup pwm
        // let max_duty = tim1_pwm_chans.0.get_max_duty();
        // tim1_pwm_chans.0.set_duty(0);
        // tim1_pwm_chans.0.enable();
        // //TODO provide PWM channels in the Board struct

        // let mag_int_opt = {
        //     let mut mag_int = HMC5983::new_with_interface(
        //         hmc5983::interface::SpiInterface::new(
        //             spi1_bus_mgr.acquire(),
        //             spi_cs_mag,
        //         ),
        //     );
        //     let rc = mag_int.init(&mut delay_source);
        //     if mag_int.init(&mut delay_source).is_ok() {
        //         Some(mag_int)
        //     } else {
        //         #[cfg(feature = "rttdebug")]
        //         rprintln!("mag setup fail: {:?}", rc);
        //         None
        //     }
        // };


        //TODO BMI055

        let baro_int_opt = {
            let proxy = spi2_bus_mgr.acquire();
            let rc = Ms5611::new(proxy, spi_cs_baro, &mut delay_source);
            if let Ok(mut baro) = rc {
                Some(baro)
            } else {
                #[cfg(feature = "rttdebug")]
                rprintln!("baro setup failed");
                None
            }
        };


        Self {
            user_leds,
            delay_source,
            ext_i2c1: i2c_bus1,
            baro: baro_int_opt,
            mag: None, //mag_int_opt,
            six_dof: None, //tdk_6dof_opt,
            fram: fram_opt,
        }
    }
}


pub type BusManager<Port> = shared_bus::proxy::BusManager<
    cortex_m::interrupt::Mutex<core::cell::RefCell<Port>>,
    Port,
>;

pub type BusProxy<'a, Port> = shared_bus::proxy::BusProxy<
    'a,
    cortex_m::interrupt::Mutex<core::cell::RefCell<Port>>,
    Port,
>;

pub type I2c1BusManager = BusManager<I2c1Port>;
pub type I2c1BusProxy<'a> = BusProxy<'a, I2c1Port>;

pub type Spi1BusManager = BusManager<Spi1Port>;
pub type Spi1BusProxy<'a> = BusProxy<'a, Spi1Port>;

pub type Spi2BusManager = BusManager<Spi2Port>;
pub type Spi2BusProxy<'a> = BusProxy<'a, Spi2Port>;


pub type InternalBarometer<'a> = Ms5611<Spi2BusProxy<'a>, SpiCsBaro>;
pub type InternalMagnetometer<'a> =
    HMC5983<hmc5983::interface::SpiInterface<Spi1BusProxy<'a>, SpiCsMag>>;
pub type Internal6Dof<'a> =
    ICM20689<icm20689::SpiInterface<Spi1BusProxy<'a>, SpiCs6Dof>>;

//<shared_bus::proxy::BusProxy<'a, M, SPI> as embedded_hal::blocking::spi::Transfer<u8>>
pub type InternalFram<'a> =
    spi_memory::series25::Flash<Spi2BusProxy<'a>, SpiCsFram>;

