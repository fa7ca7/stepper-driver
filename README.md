# A49xx/DRV88xx stepper motor driver

Stepper motor driver that supports A49xx and DRV88xx families of
stepper drivers.

## Example

```rust
#![deny(unsafe_code)]
#![deny(warnings)]
#![no_std]

extern crate panic_abort;
extern crate cortex_m;
extern crate embedded_hal;
extern crate tm4c123x_hal as hal;
extern crate stepper_rs;

use hal::delay::Delay;
use hal::gpio::GpioExt;
use hal::sysctl::SysctlExt;

fn main() {
    let p = hal::Peripherals::take().unwrap();
    let sysctl = p.SYSCTL.constrain();
    let portc = p.GPIO_PORTC.split(&sysctl.power_control);
    let clocks = sysctl.clock_setup.freeze();

    let cp = cortex_m::Peripherals::take().unwrap();
    let mut driver = stepper_rs::MotorDriver::a4988(
        Delay::new(cp.SYST, &clocks),
        portc.pc6.into_push_pull_output(),
        portc.pc7.into_push_pull_output(),
        200,
        16,
        100f32
    );

    loop {
        driver.set_speed(100f32);
        driver.set_direction(true);
        driver.move_instant(600);
        driver.set_direction(false);
        driver.move_instant(600);

        driver.set_speed(300f32);
        driver.set_direction(true);
        driver.move_smooth(1600, 150, 150);
        driver.set_direction(false);
        driver.move_smooth(1600, 150, 150);
    }
}
```

## TODO

- [ ] Implement EN pin handling (enable/disable the driver)
- [ ] Implement driver specific functions (for example, setting a step
division by pins)
- [ ] Refactor `hal::delay` into properly configured timer when
[embedded-hal](https://github.com/japaric/embedded-hal) will get more
consistent `CountDowm::Time` restrictions.

## License
Licensed at your option under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
- [MIT license](http://opensource.org/licenses/MIT)
