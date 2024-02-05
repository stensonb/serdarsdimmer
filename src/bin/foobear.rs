#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtic::app;
use rtt_target::{debug_rtt_init_print, debug_rprintln};
use stm32f1xx_hal::adc;
use stm32f1xx_hal::gpio::PinState;

use stm32f1xx_hal::gpio::{gpioc::PC13, Output, PushPull};
use stm32f1xx_hal::prelude::*;
use systick_monotonic::{fugit::Duration, Systick};

//
use cortex_m_semihosting::*;

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
        /*
        high_pot_adc: adc::Adc<pac::ADC1>,
        high_pot_chan: Pin<'B', 0, Analog>, // pb0 on board
         */
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
        debug_rprintln!("init");

        // todo: all hprintln!() can go away if we can get rprintln!() to print in openocd console via gdb
        hprintln!("init");

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
        let mut adc1 = adc::Adc::adc1(p.ADC1, _clocks);

        // Setup GPIOB
        let mut gpiob = p.GPIOB.split();

        // Configure pb0 as an analog input
        let mut ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

        let data: u16 = adc1.read(&mut ch0).unwrap();
        hprintln!("adc1: {}", data);

        // Schedule the blinking task
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        // schedule printing pot_value
        //output_pot_values::spawn().unwrap();

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
                /*
                high_pot_adc: adc1,
                high_pot_chan: ch0,
                 */
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

    /*
    #[task(local = [high_pot_adc, high_pot_chan])]
    fn output_pot_values(cx: output_pot_values::Context) {
        // schedule this again here to ignore how long this actually takes
        output_pot_values::spawn_after(Duration::<u64, 1, 1000>::from_ticks(2000)).unwrap();

        let data: u16 = cx.local.high_pot_adc.read(cx.local.high_pot_chan).unwrap();
        hprintln!("adc1: {}", data);
    }
             */
}
