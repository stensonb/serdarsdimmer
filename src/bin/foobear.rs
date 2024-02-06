#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtt_target::{debug_rtt_init_print, debug_rprintln};
use stm32f1xx_hal::{adc, pac};
use stm32f1xx_hal::gpio::{Analog, Pin, PinState};

use stm32f1xx_hal::gpio::{gpioc::PC13, Output, PushPull};
use stm32f1xx_hal::prelude::*;
use systick_monotonic::{fugit::Duration, Systick};

//
use cortex_m_semihosting::*;

const HIGH_POT_MAX: u16 = 4092; // observed
const HIGH_POT_SENSITIVITY: u16 = 30; // higher value makes pot less sensitive

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        /*
        #[lock_free]
        high_pot_value: bool,  // <- lock-free shared resource

        #[lock_free]
        low_pot_value: bool,  // <- lock-free shared resource

        #[lock_free]
        cycle_pot_value: bool,  // <- lock-free shared resource
        */
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        state: bool,

        high_pot_adc: adc::Adc<pac::ADC1>,
        high_pot_chan: Pin<'B', 0, Analog>, // pb0 on board
        high_pot_last_value: u16,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let p = cx.device;

        // Setup clocks
        let mut flash = p.FLASH.constrain();
        let rcc = p.RCC.constrain();

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        // todo: where is this printing?!
        debug_rtt_init_print!();
        debug_rprintln!("init yay");

        // todo: all hprintln!() can go away if we can get rprintln!() to print in openocd console via gdb
        hprintln!("initializing clocks...");

        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .adcclk(2.MHz())
            .freeze(&mut flash.acr);

        // Setup LED
        let mut gpioc = p.GPIOC.split();
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);

        // Setup ADC
        let adc1 = adc::Adc::adc1(p.ADC1, _clocks);

        // Setup GPIOB
        let mut gpiob = p.GPIOB.split();

        // Configure pb0 as an analog input
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

        let high_pot_last_value: u16 = 0;

        hprintln!("scheduling tasks...");

        // Schedule the blinking task
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        // schedule printing high_pot_value
        output_pot_values::spawn().unwrap();


        (
            Shared {
                /*
                high_pot_value: false,
                low_pot_value: false,
                cycle_pot_value: false,
                */
                /*
                high_pot_adc: adc1,
                high_pot_chan: ch0,
 */
            },
            Local {
                led,
                state: false,

                high_pot_adc: adc1,
                high_pot_chan: ch0,
                high_pot_last_value: high_pot_last_value,

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
        // where 250 * 1 / 1000 = 0.25 seconds
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(250)).unwrap();
    }

    #[task(local = [high_pot_adc, high_pot_chan, high_pot_last_value])]
    fn output_pot_values(cx: output_pot_values::Context) {
        // schedule this again here to ignore how long this actually takes
        output_pot_values::spawn_after(Duration::<u64, 1, 1000>::from_ticks(100)).unwrap();

        let data: u16 = cx.local.high_pot_adc.read(cx.local.high_pot_chan).unwrap();

        if (cx.local.high_pot_last_value.abs_diff(data)) > HIGH_POT_SENSITIVITY {
            if data < HIGH_POT_SENSITIVITY {
                *cx.local.high_pot_last_value = 0;
            } else if data > (HIGH_POT_MAX - HIGH_POT_SENSITIVITY) {
                *cx.local.high_pot_last_value = HIGH_POT_MAX;
            } else {
                *cx.local.high_pot_last_value = data;
            }

            hprintln!("high_pot_value: {}", *cx.local.high_pot_last_value);
        }
    }
}
