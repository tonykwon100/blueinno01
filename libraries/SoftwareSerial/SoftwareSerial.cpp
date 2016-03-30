/*
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13

#define DEBUG_SERIAL	0
#define DEBUG_LED		0

//#if DEBUG_LED
static int count;
//#endif
// 
// Includes
// 
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "nrf_gpiote.h"

#define cli()		//
//#define cli		__disable_irq

static uint8_t SREG;
//
// Lookup table
//
typedef struct _DELAY_TABLE
{
  long baud;
  unsigned short rx_delay_centering;
  unsigned short rx_delay_intrabit;
  unsigned short rx_delay_stopbit;
  unsigned short tx_delay;
} DELAY_TABLE;

#if F_CPU == 16000000

static const DELAY_TABLE PROGMEM table[] = 
{
  //  baud    rxcenter   rxintra    rxstop    tx
  //{ 115200,   1,         17,        17,       12,    },
  //{ 115200,   1,         3,        3,       3,    },
  //{ 115200,   1,         3,        3,       3,    },
  { 115200,   10,         30,        30,       1,    },
  { 57600,    10,        37,        37,       33,    },
  { 38400,    25,        57,        57,       54,    },
  { 31250,    31,        70,        70,       68,    },
  { 28800,    34,        77,        77,       74,    },
  //{ 19200,    25,        48,       47,      47,   },
  { 19200,    25,        46,       46,      46,   },
  { 14400,    74,        156,       156,      153,   },
  //{ 9600,     114,       236,       236,      233,   },
  //{ 9600,     50,       104,       104,      104,   },
  //{ 9600,     47,       101,       101,      100,   },
  //{ 9600,     30,       100,       100,      100,   },
  //{ 9600,     50,       100,       100,      100,   },//success
  //{ 9600,     50,       101,       100,      100,   },//success
  { 9600,     47,       100,       100,      100,   },
  //{ 9600,     47,       100,       100,      150,   },
  { 4800,     233,       474,       474,      471,   },
  { 2400,     471,       950,       950,      947,   },
  { 1200,     947,       1902,      1902,     1899,  },
  { 600,      1902,      3804,      3804,     3800,  },
  { 300,      3804,      7617,      7617,     7614,  },
};

const int XMIT_START_ADJUSTMENT = 5;

#elif F_CPU == 8000000

static const DELAY_TABLE table[] PROGMEM = 
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,          5,         5,      3,      },
  { 57600,    1,          15,        15,     13,     },
  { 38400,    2,          25,        26,     23,     },
  { 31250,    7,          32,        33,     29,     },
  { 28800,    11,         35,        35,     32,     },
  { 19200,    20,         55,        55,     52,     },
  { 14400,    30,         75,        75,     72,     },
  { 9600,     50,         114,       114,    112,    },
  { 4800,     110,        233,       233,    230,    },
  { 2400,     229,        472,       472,    469,    },
  { 1200,     467,        948,       948,    945,    },
  { 600,      948,        1895,      1895,   1890,   },
  { 300,      1895,       3805,      3805,   3802,   },
};

const int XMIT_START_ADJUSTMENT = 4;

#elif F_CPU == 20000000

// 20MHz support courtesy of the good people at macegr.com.
// Thanks, Garrett!

static const DELAY_TABLE PROGMEM table[] =
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   3,          21,        21,     18,     },
  { 57600,    20,         43,        43,     41,     },
  { 38400,    37,         73,        73,     70,     },
  { 31250,    45,         89,        89,     88,     },
  { 28800,    46,         98,        98,     95,     },
  { 19200,    71,         148,       148,    145,    },
  { 14400,    96,         197,       197,    194,    },
  { 9600,     146,        297,       297,    294,    },
  { 4800,     296,        595,       595,    592,    },
  { 2400,     592,        1189,      1189,   1186,   },
  { 1200,     1187,       2379,      2379,   2376,   },
  { 600,      2379,       4759,      4759,   4755,   },
  { 300,      4759,       9523,      9523,   9520,   },
};

const int XMIT_START_ADJUSTMENT = 6;

#else

#error This version of SoftwareSerial supports only 20, 16 and 8MHz processors

#endif

//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#endif
}

//#define _DEBUG_LED		13
#ifdef _DEBUG_LED
inline void DebugLed()
{
	if(count++ & 1) digitalWrite(_DEBUG_LED, HIGH);
	else digitalWrite(_DEBUG_LED, LOW);
}
#endif
//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay) {
#if 0  
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
#endif

	delayMicroseconds(delay);
	
	/*while (delay--)
    {
        __ASM(" NOP\n\t");
    };*/
	
	
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerial::listen()
{
  if (active_object != this)
  {
    _buffer_overflow = false;
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    SREG = oldSREG;
    return true;
  }

  return false;
}

//
// The receive routine called by the interrupt handler
//
void SoftwareSerial::recv()
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

#if DEBUG_SERIAL
  //Serial.write('-');
#endif
  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
	  #ifdef _DEBUG_LED
	  DebugLed();
	  #endif
    // Wait approximately 1/2 of a bit width to "center" the sample
    //tunedDelay(_rx_delay_centering);
    //DebugPulse(_DEBUG_PIN2, 1);
	//DebugLed();

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      tunedDelay(_rx_delay_intrabit);
      //DebugPulse(_DEBUG_PIN2, 1);
	  #ifdef _DEBUG_LED
	  DebugLed();
	  #endif
      uint8_t noti = ~i;
      if (rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);
    //DebugPulse(_DEBUG_PIN2, 1);
	#ifdef _DEBUG_LED
	  DebugLed();
	  #endif

    if (_inverse_logic)
      d = ~d;

	//Serial.printf("[%x]",d);
	
    // if buffer full, set the overflow flag and return
    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
    else 
    {
#if _DEBUG // for scope: pulse pin as overflow indictator
      DebugPulse(_DEBUG_PIN1, 1);
#endif
		#ifdef _DEBUG_LED
	  DebugLed();
	  #endif
      _buffer_overflow = true;
    }
  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

void SoftwareSerial::tx_pin_write(uint8_t pin_state)
{
#if 0
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
#endif
	digitalWrite(_transmitPin, pin_state);
}

uint8_t SoftwareSerial::rx_pin_read()
{
  //return *_receivePortRegister & _receiveBitMask;
  return digitalRead(_receivePin);
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handle_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic)
{
  setTX(transmitPin);
  setRX(receivePin);
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  _transmitPin = tx;
  //_transmitBitMask = digitalPinToBitMask(tx);
  //uint8_t port = digitalPinToPort(tx);
  //_transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT_PULLUP);
  if (!_inverse_logic)
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  //_receiveBitMask = digitalPinToBitMask(rx);
  //uint8_t port = digitalPinToPort(rx);
  //_receivePortRegister = portInputRegister(port);
}

//
// Public methods
//

static int SerialPinCallback(uint32_t ulPin)
//static int SerialPinCallback()
{
#if DEBUG_LED
	if(count++ & 1) digitalWrite(_DEBUG_LED, HIGH);
	else digitalWrite(_DEBUG_LED, LOW);
#endif
#if DEBUG_SERIAL
  //Serial.write('*');
#endif
	SoftwareSerial::handle_interrupt();
	return 1;
}

static void Callback()
{
#if DEBUG_LED
	if(count++ & 1) digitalWrite(_DEBUG_LED, HIGH);
	else digitalWrite(_DEBUG_LED, LOW);
#endif
#if DEBUG_SERIAL
  Serial.write('+');
#endif
	SoftwareSerial::handle_interrupt();
}

void SoftwareSerial::begin(long speed)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
  {
    long baud = pgm_read_dword(&table[i].baud);
    if (baud == speed)
    {
      _rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
      _rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
      _rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
      _tx_delay = pgm_read_word(&table[i].tx_delay);
      break;
    }
  }
#if DEBUG_SERIAL
  Serial.printf("_tx_delay %d\n",_tx_delay);
#endif
  // Set up RX interrupts, but only if we have a valid RX baud rate
  #if 0
  if (_rx_delay_stopbit)
  {
    if (digitalPinToPCICR(_receivePin))
    {
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
      *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
    }
    tunedDelay(_tx_delay); // if we were low this establishes the end
  }
  #endif

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif
	#ifdef _DEBUG_LED
	pinMode(_DEBUG_LED, OUTPUT);
	#endif
	/*
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    //NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_HIGH);
    	
	NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (_receivePin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);
									  
	NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
	
	NVIC_EnableIRQ(GPIOTE_IRQn);
	*/
  //nrf_drv_gpiote_in_event_enable(_receivePin, true);
  //attachInterrupt(GPIOTE_IRQn, Callback);
  //RFduino_pinWakeCallback(_receivePin, CHANGE, SerialPinCallback);
  
  //attachInterrupt(_receivePin, Callback, GPIOTE_CONFIG_POLARITY_Toggle);
  //attachInterruptPin(_receivePin, SerialPinCallback, CHANGE);
  RFduino_pinWakeCallback(_receivePin, LOW, SerialPinCallback);
  
  listen();
}
/*
void GPIOTE_IRQHandler()
{
#if DEBUG_LED
if(count++ & 1) digitalWrite(_DEBUG_LED, HIGH);
else digitalWrite(_DEBUG_LED, LOW);
#endif
#if DEBUG_SERIAL
  //Serial.write('+');
#endif

	SoftwareSerial::handle_interrupt();

	NRF_GPIOTE->EVENTS_PORT = 0;
	NRF_GPIOTE->EVENTS_IN[0] = 0;
	NRF_GPIOTE->EVENTS_IN[1] = 0;
	NRF_GPIOTE->EVENTS_IN[2] = 0;
	NRF_GPIOTE->EVENTS_IN[3] = 0;
}
*/
void SoftwareSerial::end()
{
  //if (digitalPinToPCMSK(_receivePin))
  //  *digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));

	detachInterrupt(_receivePin);
}


// Read data from buffer
int SoftwareSerial::read()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial::available()
{
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
#if DEBUG_SERIAL
  //Serial.println("write");
#endif
  if (_tx_delay == 0) {
    setWriteError();
#if DEBUG_SERIAL
	Serial.println("error");
#endif
    return 0;
  }

  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit
  __disable_irq();

  // Write the start bit
  tx_pin_write(_inverse_logic ? HIGH : LOW);
  tunedDelay(_tx_delay + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  if (_inverse_logic)
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(LOW); // send 1
      else
        tx_pin_write(HIGH); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(LOW); // restore pin to natural state
  }
  else
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(HIGH); // restore pin to natural state
  }

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(_tx_delay);
  __enable_irq();
  
  return 1;
}

void SoftwareSerial::flush()
{
  if (!isListening())
    return;

  uint8_t oldSREG = SREG;
  cli();
  _receive_buffer_head = _receive_buffer_tail = 0;
  SREG = oldSREG;
}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
