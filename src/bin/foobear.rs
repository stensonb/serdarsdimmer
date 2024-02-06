#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtt_target::debug_rtt_init_print;
//, debug_rprintln};
use stm32f1xx_hal::{adc, pac};
use stm32f1xx_hal::gpio::{Analog, Pin, PinState};

use stm32f1xx_hal::gpio::{gpioc::PC13, Output, PushPull};
use stm32f1xx_hal::prelude::*;
use systick_monotonic::{fugit::Duration, Systick};

use cortex_m_semihosting::hprintln;

const HIGH_POT_SENSITIVITY: u16 = 30; // higher value makes pot less sensitive
const LOW_POT_SENSITIVITY: u16 = 30; // higher value makes pot less sensitive
const CYCLE_POT_SENSITIVITY: u16 = 10; // higher value makes pot less sensitive

const CYCLE_MIN_TIME_MS: u16 = 1_000; // smallest time a complete cycle could take, in seconds (1 second)
const CYCLE_MAX_TIME_MS: u16 = 60_000; // longest time a complete cycle could take, in seconds (10 minutes)

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        state: bool,

        adc: adc::Adc<pac::ADC1>,

        high_pot_chan: Pin<'B', 0, Analog>, // b0 on board
        high_pot_last_value: u16,

        low_pot_chan: Pin<'B', 1, Analog>, // b1 on board
        low_pot_last_value: u16,

        cycle_pot_chan: Pin<'A', 7, Analog>, // a7 on board
        cycle_pot_last_value: u16,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Setup clocks
        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        // todo: where is this printing?!
        debug_rtt_init_print!();
        //debug_rprintln!("init yay");

        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .adcclk(2.MHz())
            .freeze(&mut flash.acr);

        // Setup LED
        let mut gpioc = cx.device.GPIOC.split();
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);

        // Setup ADC
        let adc = adc::Adc::adc1(cx.device.ADC1, _clocks);

        // high_pot setup
        // Setup GPIOB
        let mut gpiob = cx.device.GPIOB.split();

        // Configure pb0 as an analog input
        let high_pot_chan = gpiob.pb0.into_analog(&mut gpiob.crl);

        let high_pot_last_value: u16 = 0;

        // low_pot setup
        // re-use gpiob from above

        // Configure pb1 as an analog input
        let low_pot_chan = gpiob.pb1.into_analog(&mut gpiob.crl);

        let low_pot_last_value: u16 = 0;

        // cycle_pot setup
        // Setup GPIOA
        let mut gpioa = cx.device.GPIOA.split();

        // Configure pa7 as an analog input
        let cycle_pot_chan = gpioa.pa7.into_analog(&mut gpioa.crl);

        let cycle_pot_last_value: u16 = 0;

        // Schedule the blinking task
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
        // schedule printing high_pot_value
        output_pot_values::spawn().unwrap();

        (
            Shared {},
            Local {
                led,
                state: false,

                adc: adc,

                high_pot_chan: high_pot_chan,
                high_pot_last_value: high_pot_last_value,

                low_pot_chan: low_pot_chan,
                low_pot_last_value: low_pot_last_value,

                cycle_pot_chan: cycle_pot_chan,
                cycle_pot_last_value: cycle_pot_last_value,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led, state])]
    fn blink(cx: blink::Context) {
        if *cx.local.state {
            cx.local.led.set_high();
            *cx.local.state = false;
        } else {
            cx.local.led.set_low();
            *cx.local.state = true;
        }
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(250)).unwrap();
    }

    #[task(local = [
        adc,

        high_pot_chan,
        high_pot_last_value,

        low_pot_chan,
        low_pot_last_value,

        cycle_pot_chan,
        cycle_pot_last_value,
        ])]
    fn output_pot_values(cx: output_pot_values::Context) {
        // schedule this again here to ignore how long this actually takes
        output_pot_values::spawn_after(Duration::<u64, 1, 1000>::from_ticks(100)).unwrap();

        let mut changed: bool = false;

        let high_pot_data: u16 = cx.local.adc.read(cx.local.high_pot_chan).unwrap();
        let low_pot_data: u16 = cx.local.adc.read(cx.local.low_pot_chan).unwrap();
        let cycle_pot_data: u16 = cx.local.adc.read(cx.local.cycle_pot_chan).unwrap();

        // if any of these changed, changed == true
        changed |= set_last(cx.local.high_pot_last_value, high_pot_data, HIGH_POT_SENSITIVITY, cx.local.adc.max_sample());
        changed |= set_last(cx.local.low_pot_last_value, low_pot_data, LOW_POT_SENSITIVITY, cx.local.adc.max_sample());
        changed |= set_last(cx.local.cycle_pot_last_value, cycle_pot_data, CYCLE_POT_SENSITIVITY, cx.local.adc.max_sample());

        if changed {
            hprintln!("high_pot:{}, low_pot:{}, cycle_pot:{}", *cx.local.high_pot_last_value, *cx.local.low_pot_last_value, *cx.local.cycle_pot_last_value);

            // must protect against div/0 above
            // rounding towards zero: https://doc.rust-lang.org/reference/expressions/operator-expr.html#numeric-cast

            // todo: rework this to make pot impact % more at lower levels
            // time spent is (cycle_pot_val / max_pot_val * (CYCLE_MAX_TIME - CYCLE_MIN_TIME)) + CYCLE_MIN_TIME
            // if pot_val = 1024, eg. .25233434
            let cycle_percentage: f32 = (*cx.local.cycle_pot_last_value as f32) / (cx.local.adc.max_sample() as f32);

            hprintln!("pot: {}", *cx.local.cycle_pot_last_value);
            hprintln!("%: {}", cycle_percentage);

            let mut product: u16 = 0;
            if cycle_percentage > 0.0 {
                product = (((CYCLE_MAX_TIME_MS - CYCLE_MIN_TIME_MS) as f32) * cycle_percentage) as u16;
            }
            hprintln!("time:{}", product + CYCLE_MIN_TIME_MS);

            //hprintln!("%: {}\n-------------------------", (*cx.local.low_pot_last_value + *cx.local.high_pot_last_value + *cx.local.cycle_pot_last_value) as f32);

        }
    }

    fn set_last(last: &mut u16, data: u16, sensitivity: u16, max: u16) -> bool {
        if last.abs_diff(data) > sensitivity {
            // set min (0) if we're below sensitivity
            if data < sensitivity {
                *last = 0;
            // set max if we're above (max - sensitivity)
            } else if data > (max - sensitivity) {
                *last = max;
            // set data
            } else {
                *last = data;
            }
            return true;
        }
        return false;
    }
}
