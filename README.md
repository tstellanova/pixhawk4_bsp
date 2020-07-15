# pixhawk4_bsp

Embedded Rust board support package (BSP / BSC) for the 
[Pixhawk 4 autopilot hardware](https://docs.px4.io/v1.9.0/en/flight_controller/pixhawk4.html),
originally produced by Holybro, comprising the PX4 FMUv5 design.


### Note on Bootloaders
Most boards that support PX4 come preinstalled with a bootloader derived from 
the [PX4 Bootloader](https://github.com/PX4/Bootloader).  This bootloader
is stored in the first approximately 16-32 kB of the FMU microcontroller's
flash memory, and facilitates firmware updates via USB (among other things).
Rather than overwriting this bootloader when we build our BSP, by default
we go through some work to preserve it (see the [build.rs](./build.rs) file).
The benefit of this is that if you want to revert to other firmware, you 
may do so simply by eg connecting to QGroundControl via USB. 

## License
BSD-3-Clause: See LICENSE file

## Status
Work in progress. Currently the only thing preventing crates.io publication of
this crate is that the [stm32f4xx-hal is missing some SPI1 pins](https://github.com/stm32-rs/stm32f7xx-hal/pull/66)

- [x] Basic support for onboard (internal) sensors
- [x] Support for some external ports
- [x] Preserve PX4 bootloader in mcu flash (to allow easy revert of firmware)
 
