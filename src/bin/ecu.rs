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
    use systick_monotonic::*;

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
        defmt::info!("init");

        let mut rcc = cx.device.RCC.constrain();

        let systick = cx.core.SYST;
        let mono = Systick::new(systick, 12_000_000); //TODO set proper tick

        let gpioa = hal::prelude::_stm327xx_hal_gpio_GpioExt::split(cx.device.GPIOA);

        let can = {
            let rx = gpioa.pa11.into_alternate();
            let tx = gpioa.pa12.into_alternate();

            let can = hal::can::Can::new(cx.device.CAN1, &mut rcc.apb1, (tx, rx));

            bxcan::Can::builder(can)
                // APB1 (PCLK1): 130MHz, Bit rate: 512kBit/s, Sample Point 87.5%
                // Value was calculated with http://www.bittiming.can-wiki.info/
                .set_bit_timing(0x001e_000b) //TODO set up
                .enable()

            //TODO set up filters
        };

        let dac_pin = gpioa.pa4.into_analog();
        let dac = cx.device.DAC;
        let dac = hal::dac::dac(dac, dac_pin);

        (
            Shared {},
            Local { dac, can },
            init::Monotonics(mono),
        )
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

    let out_val = 0; //TODO calculate analog out value
    dac.set_value(out_val);
}

/// Receives CAN frame on interrupt, then starts throttle write.
fn read_can(_cx: app::read_can::Context) {
    let can = _cx.local.can;

    if let Ok(frame) = can.receive() {
        let percent = u8::from_le_bytes(frame.data().unwrap()[..0].try_into().unwrap());

        app::write_throttle::spawn(percent).unwrap();
    }
}
