//! A49xx, DRV88xx stepper motor driver.
//!
//! This is an implementation of the [`embedded-hal`] traits for the A49xx and DRV88xx families of
//! stepper motor drivers.
//!
//! [`embedded-hal`]: https://github.com/japaric/embedded-hal
//!
//! # Examples
//!
//! ```rust,no-run
//! #![deny(unsafe_code)]
//! #![deny(warnings)]
//! #![no_std]
//!
//! extern crate panic_abort;
//! extern crate cortex_m;
//! extern crate embedded_hal;
//! extern crate tm4c123x_hal as hal;
//! extern crate stepper_rs;
//!
//! use hal::delay::Delay;
//! use hal::gpio::GpioExt;
//! use hal::sysctl::SysctlExt;
//!
//! fn main() {
//!     let p = hal::Peripherals::take().unwrap();
//!     let sysctl = p.SYSCTL.constrain();
//!     let portc = p.GPIO_PORTC.split(&sysctl.power_control);
//!     let clocks = sysctl.clock_setup.freeze();
//!
//!     let cp = cortex_m::Peripherals::take().unwrap();
//!     let mut driver = stepper_rs::MotorDriver::a4988(
//!         Delay::new(cp.SYST, &clocks),
//!         portc.pc6.into_push_pull_output(),
//!         portc.pc7.into_push_pull_output(),
//!         200,
//!         16,
//!         100f32
//!     );
//!
//!     loop {
//!         driver.set_speed(100f32);
//!         driver.set_direction(true);
//!         driver.move_instant(600);
//!         driver.set_direction(false);
//!         driver.move_instant(600);
//!
//!         driver.set_speed(300f32);
//!         driver.set_direction(true);
//!         driver.move_smooth(1600, 150, 150);
//!         driver.set_direction(false);
//!         driver.move_smooth(1600, 150, 150);
//!     }
//! }
//! ```

#![no_std]

#![deny(missing_debug_implementations)]
#![deny(missing_docs)]
#![deny(warnings)]
#![deny(trivial_casts)]
#![deny(trivial_numeric_casts)]
#![deny(unsafe_code)]
#![deny(unstable_features)]
#![deny(unused_import_braces)]
#![deny(unused_qualifications)]

extern crate embedded_hal as hal;

use core::marker::PhantomData;
use hal::digital::OutputPin;
use hal::blocking::delay::DelayUs;

// TODO: support EN pin
//trait Enablable {
//    pub fn enable();
//    pub fn disable();
//}

/// Stepping mode (1:step_division)
static STEP_DIVISION: [u8; 8] = [1,2,4,8,16,32,64,128];

/// A stepper motor driver generic struct
#[derive(Debug)]
pub struct MotorDriver<D, DIR, STEP, CHIP>
where
    D: DelayUs<u32>,
    DIR: OutputPin,
    STEP: OutputPin,
    CHIP: Params,
{
    delay: D,
    dir_pin: DIR,
    step_pin: STEP,
//    TODO: support EN pin
//    enable_pin: OutputPin,
//    TODO: support driver specific stepping mode
//    driver_impl: Some(),
    _chip: PhantomData<CHIP>,

    /// usually 200
    number_of_steps: u16,
    /// stepping mode (1:step_division) [1,2,4,8,16,32,64,128]
    step_division: u8,
    /// step pulse duration (microseconds)
    step_interval: u32,
}

impl<D, DIR, STEP, CHIP> MotorDriver<D, DIR, STEP, CHIP>
where
    D: DelayUs<u32>,
    DIR: OutputPin,
    STEP: OutputPin,
    CHIP: Params,
{
    /// Sets the speed in revolutions per minute (1-200 is a reasonable range)
    pub fn set_speed(&mut self, rpm: f32) {
        self.step_interval =
            (60000000f32 / self.number_of_steps as f32 / rpm / self.step_division as f32) as u32;
    }

    /// Moves the motor steps_to_move steps
    pub fn move_instant(&mut self, steps_to_move: u64) {
        let steps_to_move = steps_to_move * self.step_division as u64;
        (0..steps_to_move).for_each(|_| self.step(None));
    }

    /// Moves the motor smoothly `steps_to_move` steps.
    /// Increasing speed during the `steps_acc` and decreasing during `steps_dec` steps.
    pub fn move_smooth(&mut self,
                       steps_to_move: u64,
                       steps_acc: u64,
                       steps_dec: u64) {
        let steps_to_move = (steps_to_move - steps_acc - steps_dec) * self.step_division as u64;
        let steps_acc = steps_acc * self.step_division as u64;
        let steps_dec = steps_dec * self.step_division as u64;

        (1..=steps_acc).for_each(|i| self.step(Some((i, steps_acc))));
        (0..steps_to_move).for_each(|_| self.step(None));
        (1..=steps_dec).rev().for_each(|i| self.step(Some((i, steps_dec))));
    }

    /// Set the direction
    pub fn set_direction(&mut self, clock_work: bool) {
        if clock_work {
            self.dir_pin.set_low();
        } else {
            self.dir_pin.set_high();
        }
    }

    /// Toggle step and yield to step control.
    ///
    /// !!!FIXME!!!
    /// Super naive implementation due to limitaions of the `embedded-hal` crate.
    /// One should use a timer instead of delay when `timer` and `time` API stabilize.
    fn step(&mut self, s: Option<(u64, u64)>) {
        self.step_pin.set_high();

        let mut step_interval = self.step_interval;
        if let Some((s1, s2)) = s {
            let r1: f64 = s1 as f64 / s2 as f64;
            let r2: f64 = 0.1 + 0.2*r1 + 2.2*r1*r1 - 1.5*r1*r1*r1;
            step_interval = (self.step_interval as f64 / r2) as u32;
        }

        // Wait at least step_min_time
        self.delay.delay_us(CHIP::STEP_MIN_TIME);
        self.step_pin.set_low();

        // Wait the rest of step_interval but at least step_min_time
        let rest = if step_interval > CHIP::STEP_MIN_TIME {
            step_interval - CHIP::STEP_MIN_TIME
        } else {
            CHIP::STEP_MIN_TIME
        };
        self.delay.delay_us(rest);
    }

    /// Generic version of constructor
    fn new(delay: D,
           mut dir_pin: DIR,
           mut step_pin: STEP,
           number_of_steps: u16,
           step_division: u8,
           rpm: f32) -> Self {
        dir_pin.set_high();
        step_pin.set_low();

        MotorDriver {
            delay,
            dir_pin,
            step_pin,
            _chip: PhantomData,
            number_of_steps,
            step_division,
            step_interval: (60000000f32 / number_of_steps as f32
                / rpm / step_division as f32) as u32,
        }
    }
}

/// Trait for motor driver parameters.
/// Currently only `STEP_MIN_TIME`.
pub trait Params {
    /// STEP high/low min value (microseconds)
    const STEP_MIN_TIME: u32;
}

macro_rules! driver {
    ($name:ident, $time:expr) => {
        #[allow(non_camel_case_types)]
        #[derive(Debug)]
        struct $name;

        impl Params for $name {
            const STEP_MIN_TIME: u32 = $time;
        }

        impl<D, DIR, STEP> MotorDriver<D, DIR, STEP, $name>
        where
            D: DelayUs<u32>,
            DIR: OutputPin,
            STEP: OutputPin
        {
            /// Specialized constructor
            pub fn $name(delay: D,
                         dir_pin: DIR,
                         step_pin: STEP,
                         number_of_steps: u16,
                         mut step_division: u8,
                         rpm: f32) -> Self {
                if !STEP_DIVISION.contains(&step_division) {
                    step_division = 1;
                }

                Self::new(delay, dir_pin, step_pin, number_of_steps, step_division, rpm)
            }
        }
    };
}

driver!(a4988, 1);
driver!(drv8825, 2);
driver!(drv8834, 2);
driver!(drv8880, 1);
