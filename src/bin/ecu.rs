#![no_main]
#![no_std]

//Needed for memory and panic handler
use aeb_rs::grid::KartPoint;
use phnx_aeb as _;

use crate::app::run_aeb;
use hal::timer::PwmExt;
use ld06_embed::error::ParseError;
use ld06_embed::nb;
use phnx_candefs::CanMessage;
use rtic::Mutex;
use stm32f7xx_hal as hal;
use stm32f7xx_hal::{prelude::*, serial, serial::Serial};

type Can1 = bxcan::Can<hal::can::Can<hal::pac::CAN1>>;

#[rtic::app(
device = crate::hal::pac,
dispatchers = [SDMMC1, DCMI]
)]
mod app {
    use aeb_rs::Aeb;
    use bxcan::filter::Mask32;
    use bxcan::{ExtendedId, Fifo};
    use stm32f7xx_hal::gpio::{Output, Pin};
    use stm32f7xx_hal::rcc::RccExt;
    use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
    use stm32f7xx_hal::serial::{DataBits, Event, Parity, Rx};
    use systick_monotonic::fugit::RateExtU32;
    use systick_monotonic::*;

    use crate::read_can;
    use crate::Can1;
    use ld06_embed::*;
    use stm32f7xx_hal::pac::{TIM1, UART4};

    use super::hal;
    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1000>;

    #[shared]
    struct Shared {
        /// AEB algorithm
        aeb: Aeb<51>,
    }

    #[local]
    struct Local {
        can: Can1,
        led: Pin<'B', 7, Output>,
        lidar_pwm: stm32f7xx_hal::timer::pwm::PwmChannel<TIM1, 0>,
        lidar: LD06Pid<Rx<UART4>>,
        last_scan_in_bounds: bool,
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

        //let gpioa = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOA);
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

        // Enable steering and encoder ids
        let mut filters = can.modify_filters();
        filters.enable_bank(
            0,
            Fifo::Fifo0,
            Mask32::frames_with_ext_id(ExtendedId::new(0x0000005).unwrap(), ExtendedId::MAX),
        );
        filters.enable_bank(
            1,
            Fifo::Fifo0,
            Mask32::frames_with_ext_id(ExtendedId::new(0x0000007).unwrap(), ExtendedId::MAX),
        );
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
            // Call interrupt when new data is available
            serial.listen(Event::Rxne);
            serial.split()
        };
        //END UART

        let ld06 = LD06::new(rx).into_pid();

        //Start LiDAR motor with a reasonable value
        ch1.set_duty((ch1.get_max_duty() as f32 * 0.26) as u16);

        //V len = 202cm
        //H width = 135cm
        //Axel to Lidar = 143cm
        let aeb = Aeb::<51>::new(
            0.0,
            0.0,
            1.08,
            ((-0.675, 1.43), (0.675, -0.59)),
            KartPoint(1.43, 0.0),
            3.0,
        );

        (
            Shared { aeb },
            Local {
                can,
                led,
                lidar_pwm: ch1,
                lidar: ld06,
                last_scan_in_bounds: false,
            },
            init::Monotonics(mono),
        )
    }

    use super::lidar_read;

    extern "Rust" {
        #[task(shared = [aeb], priority = 3)]
        fn run_aeb(_cx: run_aeb::Context);

        #[task(binds = CAN1_RX0, local = [can, led], shared = [aeb], priority = 2)]
        fn read_can(_cx: read_can::Context);

        #[task(binds = UART4, local = [lidar_pwm, lidar, last_scan_in_bounds], shared = [aeb], priority = 1)]
        fn lidar_read(_cx: lidar_read::Context);
    }
}

/// Receives CAN frame on interrupt, either a velocity value to update or a steering value
fn read_can(_cx: app::read_can::Context) {
    defmt::trace!("CAN interrupt fired");
    let mut aeb = _cx.shared.aeb;

    // How else could we know we got a frame?
    let led = _cx.local.led;
    led.toggle();

    let can = _cx.local.can;
    let frame = can.receive().unwrap();

    // Lock AEB for write, we only prevent aeb from spawning
    aeb.lock(|aeb| {
        let conv = CanMessage::from_frame(frame).unwrap();

        match conv {
            CanMessage::GetAngle(angle) => {
                defmt::trace!("Read raw steering angle {}", angle.angle);

                // We need to convert the steering wheel angle to wheel angle
                let ackermann_angle = angle.ackermann_angle();
                defmt::trace!("Converted to ackermann angle {}", ackermann_angle);

                aeb.update_steering(ackermann_angle);
            }
            CanMessage::EncoderCount(ec) => {
                defmt::trace!("Updated velocity to {}m/s", ec.velocity);

                aeb.update_velocity(ec.velocity);
            }
            _ => {
                defmt::error!("Invalid CAN ID received! This is an error in filters or candefs.")
            }
        }
    });
}

/// Called when the lidar has a new byte to read. Will spawn the AEB software task if a full scan is read.
fn lidar_read(mut _cx: app::lidar_read::Context) {
    let lidar = _cx.local.lidar;
    let pwm = _cx.local.lidar_pwm;
    let last = _cx.local.last_scan_in_bounds;

    match lidar.read_next_byte() {
        Ok(Some((scan, pid_update))) => {
            defmt::trace!("Received full scan");

            // Update speed of the LiDAR motor using the new PID value
            if pid_update != 0 {
                let speed_perc = (lidar.get_max_lidar_speed() / pid_update) as f32;
                pwm.set_duty((pwm.get_max_duty() as f32 * speed_perc) as u16);

                defmt::trace!("Set lidar speed to ${}$%", speed_perc);
            }

            // Run AEB after we have multiple scans over our desired area
            if (360.0 - 25.0..=360.0).contains(&scan.start_angle)
                || (..=25.0).contains(&scan.start_angle)
                || (360.0 - 25.0..=360.0).contains(&scan.end_angle)
                || (..=25.0).contains(&scan.end_angle)
            {
                defmt::trace!("Accepted scan");
                *last = true;

                // Add points to AEB. AEB cannot be running while this CS is running, but this is garrentied by priorities.
                _cx.shared.aeb.lock(|aeb| {
                    for (i, p) in scan.data.into_iter().enumerate() {
                        // Filter out close or unlikely points
                        if p.dist < 150 || p.confidence < 50 {
                            continue;
                        }

                        // Convert LiDAR points into standard polar points
                        let polar = scan.get_range_in_polar(i as u16);
                        // Transform from polar to cartiesian
                        let kart = KartPoint::from_polar(polar.0 / 1000.0, polar.1);

                        aeb.add_points(&[kart])
                    }
                });
            }
            // Spawn AEB if we have had scans in the range, but we now left the range
            else if *last {
                *last = false;
                defmt::trace!("Full scan received, spawning AEB");
                run_aeb::spawn().unwrap()
            }
        }
        Err(nb::Error::Other(err)) => match err {
            ParseError::SerialErr => {
                defmt::error!("Serial error");
            }
            ParseError::CrcFail => {
                defmt::error!("CRC error");
            }
        },
        _ => {}
    }
}

/// Runs AEB, popping the e-stop if a collision would occur.
fn run_aeb(mut _cx: run_aeb::Context) {
    _cx.shared.aeb.lock(|aeb| {
        let (should_estop, _) = aeb.collision_check(None);
        defmt::trace!("Collision check result: {}", should_estop);

        if should_estop {
            defmt::error!("Firing ESTOP!");
            //TODO fire estop
        }
    })
}
