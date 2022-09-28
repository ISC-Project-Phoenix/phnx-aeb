#![no_main]
#![no_std]

use hal::dac::DacOut;
use phnx_throttle as _;
// global logger + panicking-behavior + memory layout
use stm32f7xx_hal as hal;

type Can1 = bxcan::Can<hal::can::Can<hal::pac::CAN1>>;

#[rtic::app(
device = crate::hal::pac,
dispatchers = [SDMMC1]
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

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<100>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        dac: hal::dac::C1,
        can: Can1,
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
            let rx = gpioa.pa11.into_alternate();
            let tx = gpioa.pa12.into_alternate();

            let can = hal::can::Can::new(cx.device.CAN1, &mut rcc.apb1, (tx, rx));

            bxcan::Can::builder(can)
                // APB1 (PCLK1): 130MHz, Bit rate: 250kBit/s, Sample Point 87.5%
                .set_bit_timing(0x00050040)
                .leave_disabled()
        };
        can.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);

        let mut filters = can.modify_filters();
        filters.enable_bank(0, Mask32::accept_all());
        core::mem::drop(filters);

        can.enable_non_blocking();

        let dac_pin = gpioa.pa4.into_analog();
        let dac = cx.device.DAC;
        let mut dac = hal::dac::dac(dac, dac_pin);
        dac.enable();

        // Ye-old debug LED
        let mut led = gpiob.pb7.into_push_pull_output();
        led.set_high();

        (
            Shared {},
            Local { dac, can },
            init::Monotonics(mono),
        )
    }

    //TODO test and see if we can remove this without removing RTT
    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
        }
    }

    use crate::read_can;
    use crate::write_throttle;

    //Extern tasks to make the autocomplete work
    extern "Rust" {
        #[task(local = [dac], capacity = 5, priority = 2)]
        fn write_throttle(_cx: write_throttle::Context, throttle: u8);

        #[task(binds = CAN1_RX0, local = [can], priority = 1)]
        fn read_can(_cx: read_can::Context);
    }
}

/// Writes 0-5V to the ESC.
fn write_throttle(_cx: app::write_throttle::Context, throttle: u8) {
    let dac = _cx.local.dac;

    //Percent of 3.3V
    let out_val = (throttle as f32 / 100.0) * 4092.0;

    defmt::debug!("Writing {} to DAC", out_val);

    dac.set_value(out_val as u16);
}

/// Receives CAN frame on interrupt, then starts throttle write.
fn read_can(_cx: app::read_can::Context) {
    defmt::trace!("CAN interrupt fired");

    let can = _cx.local.can;

    if let Ok(frame) = can.receive() {
        let percent = u8::from_le_bytes(frame.data().unwrap()[..0].try_into().unwrap());

        defmt::debug!("Got frame percent: {}", percent);

        app::write_throttle::spawn(percent).unwrap();
    }
}
