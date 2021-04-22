use crate::motor;
use crate::serial;
use arduino_uno::hal::clock;
use arduino_uno::hal::port;
use arduino_uno::hal::usart;
use arduino_uno::pac;
use arduino_uno::prelude::*;
use core::cell;

const BUFFER_CAPACITY: usize = 32;

pub type Usart = usart::Usart0<clock::MHz16, port::mode::Floating>;

pub type UsartReader = usart::UsartReader<
    pac::USART0,
    port::portd::PD0<port::mode::Input<port::mode::Floating>>,
    port::portd::PD1<port::mode::Output>,
    clock::MHz16,
>;
pub type UsartWriter = usart::UsartWriter<
    pac::USART0,
    port::portd::PD0<port::mode::Input<port::mode::Floating>>,
    port::portd::PD1<port::mode::Output>,
    clock::MHz16,
>;

pub struct Serial {
    _unused: (),
}

pub struct UartRxInterruptState {
    rx_buffer: arrayvec::ArrayVec<u8, BUFFER_CAPACITY>,
    messages: ringbuffer::ConstGenericRingBuffer<arrayvec::ArrayVec<u8, BUFFER_CAPACITY>, 8>,
    reader: UsartReader,
}

static RX_INTERRUPT_STATE: bare_metal::Mutex<cell::RefCell<Option<UartRxInterruptState>>> =
    bare_metal::Mutex::new(cell::RefCell::new(None));

impl Serial {
    pub fn init(mut serial: Usart) -> (Self, UsartWriter) {
        serial.listen(usart::Event::RxComplete);

        let (reader, writer) = serial.split();
        let rx_buffer = Default::default();
        let messages = Default::default();

        let rx_interrupt_state = UartRxInterruptState {
            reader,
            messages,
            rx_buffer,
        };
        avr_device::interrupt::free(|cs| {
            *RX_INTERRUPT_STATE.borrow(cs).borrow_mut() = Some(rx_interrupt_state)
        });
        (Self { _unused: () }, writer)
    }

    pub fn update(&mut self, serial: &mut serial::UsartWriter, motor: &mut motor::Motor) {
        if let Some(rx_buffer) = self.take_message() {
            if let [b'$', body @ .., b'\n'] = &rx_buffer[..] {
                // Sender might use "\r\n" line endings; strip that too
                let mut body = body;
                if let [new_body @ .., b'\r'] = body {
                    body = new_body;
                }

                if let [b'D', more_digits @ .., digit] = body {
                    let digit = digit - b'0';
                    #[cfg(debug_assertions)]
                    ufmt::uwriteln!(serial, "received new target digit: {}", digit).void_unwrap();
                    motor.set_target_digit(digit);

                    // If there are remaining digits
                    if !more_digits.is_empty() {
                        nb::block!(serial.write(b'$')).void_unwrap();
                        nb::block!(serial.write(b'D')).void_unwrap();
                        for &digit in more_digits {
                            nb::block!(serial.write(digit)).void_unwrap();
                        }
                        nb::block!(serial.write(b'\n')).void_unwrap();
                        nb::block!(serial.flush()).void_unwrap();
                    }
                }
            }
        }
    }

    fn take_message(&mut self) -> Option<arrayvec::ArrayVec<u8, 32>> {
        use ringbuffer::RingBufferRead;
        avr_device::interrupt::free(|cs| {
            if let Some(rx_interrupt_state) = RX_INTERRUPT_STATE.borrow(cs).borrow_mut().as_mut() {
                rx_interrupt_state.messages.dequeue()
            } else {
                None
            }
        })
    }
}

#[allow(non_snake_case)]
#[avr_device::interrupt(atmega328p)]
fn USART_RX() {
    use core::mem;
    use embedded_hal::serial::Read;
    use ringbuffer::RingBufferWrite;

    avr_device::interrupt::free(|cs| {
        if let Some(rx_interrupt_state) = RX_INTERRUPT_STATE.borrow(cs).borrow_mut().as_mut() {
            if let Ok(byte) = rx_interrupt_state.reader.read() {
                let _ = rx_interrupt_state.rx_buffer.try_push(byte);
                if byte == b'\n' {
                    rx_interrupt_state.messages.push(mem::replace(
                        &mut rx_interrupt_state.rx_buffer,
                        Default::default(),
                    ));
                }
            }
        }
    });
}
