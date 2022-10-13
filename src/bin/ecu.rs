#![no_main]
#![no_std]

//Needed for memory and panic handler
use phnx_aeb as _;

use hal::timer::PwmExt;
use ld06_embed::error::ParseError;
use ld06_embed::{LD06Pid, nb};
use stm32f7xx_hal as hal;
use stm32f7xx_hal::{prelude::*, serial, serial::Serial};
use stm32f7xx_hal::prelude::_embedded_hal_digital_ToggleableOutputPin;

type Can1 = bxcan::Can<hal::can::Can<hal::pac::CAN1>>;

#[rtic::app(
device = crate::hal::pac,
dispatchers = [SDMMC1, DCMI]
)]
mod app {
    use bxcan::ExtendedId;
    use bxcan::filter::Mask32;
    use stm32f7xx_hal::{
        rcc::{HSEClock, HSEClockMode},
    };
    use stm32f7xx_hal::gpio::{Output, Pin};
    use stm32f7xx_hal::rcc::RccExt;
    use stm32f7xx_hal::serial::{DataBits, Parity, Rx, Event};
    use systick_monotonic::*;
    use systick_monotonic::fugit::RateExtU32;

    use crate::Can1;
    use stm32f7xx_hal::pac::{TIM1, UART4};
    use crate::read_can;
    use ld06_embed::*;

    use super::*;
    use super::hal;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1000>;

    #[shared]
    struct Shared {
        /// The current velocity in m/s
        velocity: f32,
        /// The current steering angle of the virtual ackermann wheel
        steering_angle: f32,
    }

    #[local]
    struct Local {
        can: Can1,
        led: Pin<'B', 7, Output>,
        lidar_pwm: stm32f7xx_hal::timer::pwm::PwmChannel<TIM1, 0>,
        lidar: LD06Pid<Rx<UART4>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = cx.device.RCC.constrain();

        // To meet CAN clock accuracy requirements, an external crystal or ceramic
        // resonator must be used.
        let _clocks = rcc
            .cfgr
            .hse(HSEClock::new(8_000_000.Hz(), HSEClockMode::Bypass))
            .sysclk(216_000_000.Hz())
            .hclk(216_000_000.Hz())
            .freeze();

        defmt::info!("init");

        let systick = cx.core.SYST;
        let mono = Systick::new(systick, 216_000_000);

        let gpioa = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOA);
        let gpiob = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOB);
        let gpioe = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOE);
        let gpiod = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOD);

        // CAN SETUP
        let mut can = {
            // Use alternative pins on the pre-soldered headers.
            let rx = gpiob.pb8.into_alternate();
            let tx = gpiob.pb9.into_alternate();

            let can = hal::can::Can::new(cx.device.CAN1, &mut rcc.apb1, (tx, rx));

            bxcan::Can::builder(can)
                .set_bit_timing(0x001e0005) //1mb: 0x001e0002, 250k: 0x001e000, 500kb: 0x001e0005
                .leave_disabled()
        };
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        let mut filters = can.modify_filters();
        //TODO change this to steering and encoder ID
        filters.enable_bank(0, Mask32::frames_with_ext_id(ExtendedId::new(0x0000005).unwrap(), ExtendedId::MAX));
        core::mem::drop(filters);

        if can.enable_non_blocking().is_err() {
            defmt::info!("CAN enabling in background...");
        }
        // END CAN

        // Ye-old debug LED
        let mut led = gpiob.pb7.into_push_pull_output();
        led.set_high();

        //PWM SETUP
        let (mut ch1, _ch2) = {
            let pe9 = gpioe.pe9.into_alternate();
            let pe11 = gpioe.pe11.into_alternate();
            let channels = (pe9, pe11);
            cx.device.TIM1.pwm_hz(channels, 24.kHz(), &_clocks).split()
        };
        ch1.enable();
        //END PWM

        //UART SETUP
        let (_, rx) = {
            let tx = gpiod.pd1.into_alternate();
            let rx = gpiod.pd0.into_alternate();

            let mut serial = Serial::new(
                cx.device.UART4,
                (tx, rx),
                &_clocks,
                serial::Config {
                    baud_rate: 230_400.bps(),
                    data_bits: DataBits::Bits8,
                    parity: Parity::ParityNone,
                    ..Default::default()
                },
            );
            // Call interrupt when new data is available (I think)
            serial.listen(Event::Rxne);
            serial.split()
        };
        //END UART

        let ld06 = LD06::new(rx).into_pid();

        //Start LiDAR motor with a reasonable value
        ch1.set_duty((ch1.get_max_duty() as f32 * 0.26) as u16);

        (
            Shared { velocity: 0.0, steering_angle: 0.0 },
            Local { can, led, lidar_pwm: ch1, lidar: ld06 },
            init::Monotonics(mono),
        )
    }

    use super::lidar_read;

    extern "Rust" {
        #[task(binds = CAN1_RX0, local = [can, led], shared = [velocity, steering_angle], priority = 2)]
        fn read_can(_cx: read_can::Context);

        #[task(binds = UART4, local = [lidar_pwm, lidar], priority = 1)]
        fn lidar_read(_cx: lidar_read::Context);
    }
}

/// Receives CAN frame on interrupt, either a velocity value to update or a steering value
fn read_can(_cx: app::read_can::Context) {
    //TODO redo for encoder and steering frames later, read vel and steering angle, convert steering angle to ackermann wheel

    defmt::trace!("CAN interrupt fired");

    // How else could we know we got a frame?
    let led = _cx.local.led;
    led.toggle();

    let can = _cx.local.can;
}

/// Called when the lidar has a new byte to read. Will spawn the AEB software task if a full scan is read.
fn lidar_read(_cx: app::lidar_read::Context) {
    let lidar = _cx.local.lidar;
    let pwm = _cx.local.lidar_pwm;

    match lidar.read_next_byte() {
        Ok(Some((scan, pid_update))) => {
            defmt::trace!("Received full scan");

            // Update speed of the LiDAR motor using the new PID value
            if pid_update != 0 {
                let speed_perc = (lidar.get_max_lidar_speed() / pid_update) as f32;
                pwm.set_duty((pwm.get_max_duty() as f32 * speed_perc) as u16);

                defmt::trace!("Set lidar speed to {}%", speed_perc);
            }

            // Accept any scans that could have any points in our grid
            if (scan.start_angle >= (360.0 - 25.0) && scan.start_angle <= 360.0) || (scan.start_angle <= 25.0 && scan.start_angle >= 0.0) {
                defmt::trace!("Accepted scan");
                //TODO spawn AEB task with scan
            }
        }
        Err(err) => {
            match err {
                nb::Error::Other(err) => {
                    match err {
                        ParseError::SerialErr => {
                            defmt::error!("Serial error");
                        }
                        ParseError::CrcFail => {
                            defmt::error!("CRC error");
                        }
                    }
                }
                _ => {}
            }
        }
        _ => {}
    }
}