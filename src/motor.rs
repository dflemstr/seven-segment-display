use crate::led;
use crate::serial;
use arduino_uno::adc;
use arduino_uno::hal::port;
use arduino_uno::prelude::*;
use core::sync::atomic;

const STEP_FREQ: u32 = 550;

const STEPS_PER_REVOLUTION: u32 = 64 * 32;
const STEPS_PER_DIGIT: u32 = (STEPS_PER_REVOLUTION * 200 / 360) as u32;
// 200 degrees per digit
const STEPS_PER_ROUNDTRIP: u32 = (STEPS_PER_REVOLUTION * 2000 / 360) as u32; // 10 digits

type PinStepperC0 = port::portd::PD5<port::mode::Output>;
type PinStepperC1 = port::portd::PD6<port::mode::Output>;
type PinStepperC2 = port::portd::PD7<port::mode::Output>;
type PinStepperC3 = port::portb::PB0<port::mode::Output>;
type PinHallSensor = port::portc::PC0<port::mode::Analog>;

pub struct Motor {
    timer: arduino_uno::pac::TC1,
}

#[derive(Clone, Copy, Default)]
struct PositioningState {
    current_pos: u32,
    target_pos: Option<u32>,
    correction_offset: Option<u32>,
    next_correction_offset: Option<u32>,
}

struct Positioning {
    state: PositioningState,
    pin_stepper_c0: PinStepperC0,
    pin_stepper_c1: PinStepperC1,
    pin_stepper_c2: PinStepperC2,
    pin_stepper_c3: PinStepperC3,
}

#[derive(Clone, Copy, Default)]
struct CalibrationState {
    adc_min_val: Option<u16>,
}

struct Calibration {
    state: CalibrationState,
    adc: adc::Adc,
    pin_hall_sensor: PinHallSensor,
}

struct MotorInterruptState {
    positioning: Positioning,
    calibration: Calibration,
}

static FLAG_AT_ZERO: atomic::AtomicBool = atomic::AtomicBool::new(false);
static FLAG_ROUNDTRIP_COMPLETED: atomic::AtomicBool = atomic::AtomicBool::new(false);
static FLAG_INITIAL_CALIBRATION_DONE: atomic::AtomicBool = atomic::AtomicBool::new(false);

static MOTOR_INTERRUPT_STATE: bare_metal::Mutex<core::cell::RefCell<Option<MotorInterruptState>>> =
    bare_metal::Mutex::new(core::cell::RefCell::new(None));

impl Motor {
    pub fn init(
        timer: arduino_uno::pac::TC1,
        pin_stepper_c0: PinStepperC0,
        pin_stepper_c1: PinStepperC1,
        pin_stepper_c2: PinStepperC2,
        pin_stepper_c3: PinStepperC3,
        adc: adc::Adc,
        pin_hall_sensor: PinHallSensor,
    ) -> Self {
        use arduino_uno::hal::clock::Clock;
        // timer mode 4 (CTC on OCA aka ClearOnTimerMatchOutputCompare)
        timer.tccr1b.write(|w| w.wgm1().bits(0b01));
        timer.tccr1a.write(|w| w.wgm1().bits(0));
        // frequency
        timer.ocr1a.write(|w| unsafe {
            w.bits((arduino_uno::hal::clock::MHz16::FREQ / STEP_FREQ) as u16)
        });
        // reset counter
        timer.tcnt1.reset();
        // enable interrupt when timer reaches ocr1a
        timer.timsk1.write(|w| w.ocie1a().set_bit());

        let motor_interrupt_state = MotorInterruptState {
            positioning: Positioning {
                state: PositioningState::default(),
                pin_stepper_c0,
                pin_stepper_c1,
                pin_stepper_c2,
                pin_stepper_c3,
            },
            calibration: Calibration {
                state: CalibrationState::default(),
                adc,
                pin_hall_sensor,
            },
        };

        avr_device::interrupt::free(|cs| {
            *MOTOR_INTERRUPT_STATE.borrow(cs).borrow_mut() = Some(motor_interrupt_state);
        });

        Self { timer }
    }

    pub fn set_target_digit(&mut self, digit: u8) {
        if digit <= 9 {
            avr_device::interrupt::free(|cs| {
                if let Some(state) = MOTOR_INTERRUPT_STATE.borrow(cs).borrow_mut().as_mut() {
                    // digit 0
                    //const OFFSET: i32 = 40;
                    // digit 1
                    //const OFFSET: i32 = 10;
                    // digit 2
                    //const OFFSET: i32 = 0;
                    // digit 3
                    const OFFSET: i32 = -500;

                    let mut target_pos = STEPS_PER_DIGIT as i32 * digit as i32 + OFFSET;
                    if target_pos < 0 {
                        target_pos += STEPS_PER_ROUNDTRIP as i32;
                    }
                    if target_pos > STEPS_PER_ROUNDTRIP as i32 {
                        target_pos -= STEPS_PER_ROUNDTRIP as i32;
                    }

                    state.positioning.set_target_pos(target_pos as u32);
                }
            });
        }
    }

    pub fn start(&mut self) {
        self.timer.tccr1b.write(|w| w.cs1().direct());
        // reset counter
        self.timer.tcnt1.reset();
    }

    pub fn stop(&mut self) {
        self.timer.tccr1b.write(|w| w.cs1().no_clock());
    }

    pub fn update(&mut self, serial: &mut serial::UsartWriter, led: &mut led::Led) {
        // The AVR target actually doesn't have compare-and-swap support right now, so this is
        // actually not atomic.  However, even though this is racey, at least this means we avoid
        // locks.
        if FLAG_ROUNDTRIP_COMPLETED.load(atomic::Ordering::Relaxed) {
            FLAG_ROUNDTRIP_COMPLETED.store(false, atomic::Ordering::Relaxed);

            #[cfg(debug_assertions)]
            ufmt::uwriteln!(serial, "roundtrip completed").void_unwrap();
            led.blink(1);
            Self::dump_interrupt_state(serial);
        }
        if FLAG_AT_ZERO.load(atomic::Ordering::Relaxed) {
            FLAG_AT_ZERO.store(false, atomic::Ordering::Relaxed);
            #[cfg(debug_assertions)]
            ufmt::uwriteln!(serial, "at zero").void_unwrap();
            led.blink(2);
            Self::dump_interrupt_state(serial);
        }
        if FLAG_INITIAL_CALIBRATION_DONE.load(atomic::Ordering::Relaxed) {
            FLAG_INITIAL_CALIBRATION_DONE.store(false, atomic::Ordering::Relaxed);

            #[cfg(debug_assertions)]
            ufmt::uwriteln!(serial, "initial calibration completed").void_unwrap();
            led.blink(3);
            Self::dump_interrupt_state(serial);
        }
    }

    fn dump_interrupt_state(serial: &mut serial::UsartWriter) {
        #[cfg(debug_assertions)]
        if let Some((positioning_state, calibration_state)) = avr_device::interrupt::free(|cs| {
            MOTOR_INTERRUPT_STATE
                .borrow(cs)
                .borrow_mut()
                .as_ref()
                .map(|interrupt| (interrupt.positioning.state, interrupt.calibration.state))
        }) {
            // Use copy of positioning_data to not hold the lock for too long
            ufmt::uwriteln!(
                serial,
                "state: current_pos: {:?}, target_pos: {:?}, corrected_pos: {:?}, correction_offset: {:?}, next_correction_offset: {:?}, adc_min_val: {:?}",
                positioning_state.current_pos,
                positioning_state.target_pos,
                positioning_state.corrected_pos(),
                positioning_state.correction_offset,
                positioning_state.next_correction_offset,
                calibration_state.adc_min_val
            ).void_unwrap();
        }
    }
}

impl Drop for Motor {
    fn drop(&mut self) {
        avr_device::interrupt::free(|cs| {
            *MOTOR_INTERRUPT_STATE.borrow(cs).borrow_mut() = None;
        });
    }
}

impl PositioningState {
    fn corrected_pos(&self) -> Option<u32> {
        self.correction_offset.map(|offset| {
            let current_pos = self.current_pos;
            let current_pos_avoiding_wrapping = if offset > current_pos {
                current_pos + STEPS_PER_ROUNDTRIP
            } else {
                current_pos
            };
            current_pos_avoiding_wrapping - offset
        })
    }

    fn is_at_target(&self) -> bool {
        self.corrected_pos() == self.target_pos
    }

    fn has_correction(&self) -> bool {
        self.correction_offset.is_some()
    }

    fn store_next_correction(&mut self) {
        self.next_correction_offset = Some(self.current_pos);
    }

    fn update_correction(&mut self) {
        if let Some(offset) = self.next_correction_offset.take() {
            self.correction_offset = Some(offset);
        }
    }
}

impl Positioning {
    fn set_target_pos(&mut self, position: u32) {
        self.state.target_pos = Some(position);
    }

    // Returns true when calibration should be reset (this is a bit ugly, should have a cleaner
    // interface...)
    fn step(&mut self) -> bool {
        use embedded_hal::digital::v2::OutputPin;

        let mut current_pos = self.state.current_pos;
        let step = 3 - (current_pos % 4);

        if Some(current_pos) == self.state.correction_offset {
            FLAG_AT_ZERO.store(true, atomic::Ordering::Relaxed);
        }

        current_pos = current_pos + 1;

        let pattern = 0b110011u8 << step;
        if pattern & 0b00010000 != 0 {
            self.pin_stepper_c0.set_high().void_unwrap();
        } else {
            self.pin_stepper_c0.set_low().void_unwrap();
        }
        if pattern & 0b00100000 != 0 {
            self.pin_stepper_c1.set_high().void_unwrap();
        } else {
            self.pin_stepper_c1.set_low().void_unwrap();
        }
        if pattern & 0b01000000 != 0 {
            self.pin_stepper_c2.set_high().void_unwrap();
        } else {
            self.pin_stepper_c2.set_low().void_unwrap();
        }
        if pattern & 0b10000000 != 0 {
            self.pin_stepper_c3.set_high().void_unwrap();
        } else {
            self.pin_stepper_c3.set_low().void_unwrap();
        }

        let result = if current_pos >= STEPS_PER_ROUNDTRIP {
            current_pos = 0;
            FLAG_ROUNDTRIP_COMPLETED.store(true, atomic::Ordering::Relaxed);
            if !self.state.has_correction() {
                FLAG_INITIAL_CALIBRATION_DONE.store(true, atomic::Ordering::Relaxed);
            }
            self.state.update_correction();
            true
        } else {
            false
        };

        self.state.current_pos = current_pos;

        result
    }

    fn turn_off(&mut self) {
        use embedded_hal::digital::v2::OutputPin;
        self.pin_stepper_c0.set_low().void_unwrap();
        self.pin_stepper_c1.set_low().void_unwrap();
        self.pin_stepper_c2.set_low().void_unwrap();
        self.pin_stepper_c3.set_low().void_unwrap();
    }
}

impl CalibrationState {
    // Returns whether we are at a new lowest point
    fn add_sample(&mut self, value: u16) -> bool {
        if self.adc_min_val.map(|v| value < v).unwrap_or(true) {
            self.adc_min_val = Some(value);
            true
        } else {
            false
        }
    }

    fn reset(&mut self) {
        self.adc_min_val = None;
    }
}

impl Calibration {
    // Returns whether we are at a new lowest point
    fn update(&mut self) -> bool {
        use embedded_hal::adc::OneShot;

        match self.adc.read(&mut self.pin_hall_sensor) {
            Ok(value) => self.state.add_sample(value),
            Err(nb::Error::WouldBlock) => false,
            Err(nb::Error::Other(_)) => false,
        }
    }
}

impl MotorInterruptState {
    fn reconcile(&mut self) {
        if self.calibration.update() {
            self.positioning.state.store_next_correction();
        }
        if self.positioning.state.is_at_target() && self.positioning.state.has_correction() {
            self.positioning.turn_off();
        } else {
            if self.positioning.step() {
                self.calibration.state.reset();
            }
        }
    }
}

#[allow(non_snake_case)]
#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    avr_device::interrupt::free(|cs| {
        if let Some(motor_interrupt_state) = MOTOR_INTERRUPT_STATE.borrow(cs).borrow_mut().as_mut()
        {
            motor_interrupt_state.reconcile();
        }
    });
}
