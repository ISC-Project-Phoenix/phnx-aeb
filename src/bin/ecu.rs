#![no_main]
#![no_std]

use hal::dac::DacOut;
use phnx_throttle as _;
// global logger + panicking-behavior + memory layout
use stm32f7xx_hal as hal;
use stm32f7xx_hal::prelude::{_embedded_hal_digital_ToggleableOutputPin, OutputPin};

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

    use stm32f7xx_hal::{
        can::Can,
        pac,
        prelude::*,
        rcc::{HSEClock, HSEClockMode},
    };
    use stm32f7xx_hal::gpio::{Output, Pin};

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<100>;

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
                // APB1 (PCLK1): 130MHz, Bit rate: 512kBit/s, Sample Point 87.5%
                .set_bit_timing(0x001e_000b)
                .enable()
        };
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        let mut filters = can.modify_filters();
        filters.enable_bank(0, Mask32::accept_all());
        core::mem::drop(filters);

        //if can.enable_non_blocking().is_err() {
        //    defmt::info!("CAN enabling in background...");
        //}

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

    //TODO test and see if we can remove this without removing RTT
    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {}
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

/// Writes 0-5V to the ESC.
fn write_throttle(_cx: app::write_throttle::Context, throttle: u8) {
    // This should be a percent, so just throw out invalid values
    if throttle > 100 {
        defmt::error!("Ignoring invalid throttle percent");
        return;
    }

    let dac = _cx.local.dac;

    //Percent of 3.3V
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
        let percent = u8::from_le_bytes(frame.data().unwrap()[..1].try_into().unwrap());

        defmt::trace!("Got frame percent: {}", percent);

        app::write_throttle::spawn(percent).unwrap();
    } else {
        defmt::error!("Lost a frame :(");
    }
}
