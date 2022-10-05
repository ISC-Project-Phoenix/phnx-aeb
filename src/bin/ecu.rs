//! Program that takes throttle messages from CAN and outputs a voltage 0-3.1V from DAC, to
//! control the ESC.
//!
//! Author: Andrew Ealovega

#![no_main]
#![no_std]

use hal::dac::DacOut;

// global logger + panicking-behavior + memory layout
use phnx_throttle as _;

use stm32f7xx_hal as hal;
use stm32f7xx_hal::prelude::_embedded_hal_digital_ToggleableOutputPin;

type Can1 = bxcan::Can<hal::can::Can<hal::pac::CAN1>>;

#[rtic::app(
device = crate::hal::pac,
dispatchers = [SDMMC1, DCMI]
)]
mod app {
    use super::hal;
    use crate::Can1;
    use stm32f7xx_hal::rcc::RccExt;
    use stm32f7xx_hal::dac::DacPin;
    use systick_monotonic::*;
    use systick_monotonic::fugit::RateExtU32;
    use bxcan::filter::Mask32;
    use bxcan::ExtendedId;

    use stm32f7xx_hal::{
        rcc::{HSEClock, HSEClockMode},
    };
    use stm32f7xx_hal::gpio::{Output, Pin};

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1000>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        dac: hal::dac::C1,
        can: Can1,
        led: Pin<'B', 7, Output>,
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

        // Accept only throttle messages
        let mut filters = can.modify_filters();
        filters.enable_bank(0, Mask32::frames_with_ext_id(ExtendedId::new(0x0000005).unwrap(), ExtendedId::MAX));
        core::mem::drop(filters);

        if can.enable_non_blocking().is_err() {
            defmt::info!("CAN enabling in background...");
        }

        // Configure DAC output
        let dac_pin = gpioa.pa4.into_analog();
        let dac = cx.device.DAC;
        let mut dac = hal::dac::dac(dac, dac_pin);
        dac.enable();

        // Ye-old debug LED
        let mut led = gpiob.pb7.into_push_pull_output();
        led.set_high();

        (
            Shared {},
            Local { dac, can, led },
            init::Monotonics(mono),
        )
    }

    use crate::read_can;
    use crate::write_throttle;

    //Extern tasks to make the autocomplete work
    extern "Rust" {
        #[task(local = [dac], capacity = 5, priority = 2)]
        fn write_throttle(_cx: write_throttle::Context, throttle: u8);

        #[task(binds = CAN1_RX0, local = [can, led], priority = 1)]
        fn read_can(_cx: read_can::Context);
    }
}

/// Writes 0-3.1V to the ESC, with the percent passed to the task.
fn write_throttle(_cx: app::write_throttle::Context, throttle: u8) {
    // This should be a percent, so just throw out invalid values
    if throttle > 100 {
        defmt::error!("Ignoring invalid throttle percent");
        return;
    }

    let dac = _cx.local.dac;

    //Percent of 3.1V
    let out_val = (throttle as f32 / 100.0) * 4092.0;

    defmt::trace!("Writing {} to DAC", out_val as u16);

    dac.set_value(out_val as u16);
}

/// Receives CAN frame on interrupt, then starts throttle write.
fn read_can(_cx: app::read_can::Context) {
    defmt::trace!("CAN interrupt fired");

    // How else could we know we got a frame?
    let led = _cx.local.led;
    led.toggle();

    let can = _cx.local.can;

    if let Ok(frame) = can.receive() {
        //Percent should be encoded in the first data byte
        let percent = u8::from_le_bytes(frame.data().unwrap()[..1].try_into().unwrap());

        defmt::trace!("Got frame percent: {}", percent);

        app::write_throttle::spawn(percent).unwrap();
    } else {
        defmt::error!("Lost a frame :(");
    }
}
