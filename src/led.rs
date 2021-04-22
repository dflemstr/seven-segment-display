use arduino_uno::hal::port;
use arduino_uno::prelude::*;
use embedded_hal::digital::v2::OutputPin;

pub type Pin = port::portb::PB5<port::mode::Output>;

pub struct Led {
    pin_led: Pin,
    blink_counter: u8,
}

impl Led {
    pub fn init(pin_led: Pin) -> Self {
        let blinks = 0;
        Led {
            pin_led,
            blink_counter: blinks,
        }
    }

    pub fn blink(&mut self, times: u8) {
        if times > 0 {
            self.blink_counter = times * 2 - 1;
        }
    }

    pub fn update(&mut self) {
        let high;
        if self.blink_counter > 0 {
            high = (self.blink_counter & 0b1) == 1;
            self.blink_counter -= 1;
        } else {
            high = false;
        }

        if high {
            self.pin_led.set_high().void_unwrap();
        } else {
            self.pin_led.set_low().void_unwrap();
        }
    }
}
