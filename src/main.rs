#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_uno::adc;
use arduino_uno::prelude::*;
use arduino_uno::wdt;
use avr_device::interrupt;

use panic_abort as _;

mod led;
mod motor;
mod serial;

#[arduino_uno::entry]
fn main() -> ! {
    let dp = arduino_uno::Peripherals::take().unwrap();

    let mut pins = arduino_uno::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);

    let mut serial = arduino_uno::Serial::new(
        dp.USART0,
        pins.d0,
        pins.d1.into_output(&mut pins.ddr),
        9600.into_baudrate(),
    );
    #[cfg(debug_assertions)]
    ufmt::uwriteln!(serial, "started").void_unwrap();

    let mut adc = adc::Adc::new(
        dp.ADC,
        adc::AdcSettings {
            clock_divider: adc::ClockRateDivision::Factor64,
            ref_voltage: adc::ReferenceVoltage::AVcc,
        },
    );

    let mut led = led::Led::init(pins.d13.into_output(&mut pins.ddr));

    let pin_stepper_c0 = pins.d5.into_output(&mut pins.ddr);
    let pin_stepper_c1 = pins.d6.into_output(&mut pins.ddr);
    let pin_stepper_c2 = pins.d7.into_output(&mut pins.ddr);
    let pin_stepper_c3 = pins.d8.into_output(&mut pins.ddr);
    let pin_hall_sensor = pins.a0.into_analog_input(&mut adc);
    let mut motor = motor::Motor::init(
        dp.TC1,
        pin_stepper_c0,
        pin_stepper_c1,
        pin_stepper_c2,
        pin_stepper_c3,
        adc,
        pin_hall_sensor,
    );
    motor.set_target_digit(0);
    motor.start();

    let (mut commands, mut writer) = serial::Serial::init(serial);

    let mut watchdog = wdt::Wdt::new(&dp.CPU.mcusr, dp.WDT);
    watchdog.start(wdt::Timeout::Ms8000);

    unsafe {
        interrupt::enable();
    }

    loop {
        watchdog.feed();
        commands.update(&mut writer, &mut motor);
        motor.update(&mut writer, &mut led);
        led.update();
    }
}
