/*
 Copyright (c) 2013 OpenSourceRF.com.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "WInterrupts.h"

#define DEBUG_SERIAL	0

typedef void (*app_gpiote_input_event_handler_t)(void);

static app_gpiote_input_event_handler_t m_app_gpiote_input_event_handlers[4] = {0};

void attachInterrupt(uint8_t IRQn, callback_t callback)
{
  dynamic_handlers[IRQn] = callback;
  rfduino_nvic_enableirq(IRQn);
}

void detachInterrupt(uint8_t IRQn)
{
  rfduino_nvic_disableirq(IRQn);
  dynamic_handlers[IRQn] = NULL;
}

void attachPinInterrupt(uint32_t pin, pin_callback_t callback, uint32_t mode)
{
  RFduino_pinWakeCallback(pin, mode, callback);
}

void detachPinInterrupt(uint32_t pin)
{
  RFduino_pinWakeCallback(pin, DISABLE, NULL);
}
/*
void attachInterruptPin(uint32_t pin, pin_callback_t callback, uint32_t mode)
{
	uint32_t polarity;
	
	__disable_irq();
	
	BlueInnoPinMap(pin);
	
	if(mode==LOW) polarity = GPIOTE_CONFIG_POLARITY_HiToLo;
	else if(mode==HIGH) polarity = GPIOTE_CONFIG_POLARITY_LoToHi;
	else if(mode==CHANGE) polarity = GPIOTE_CONFIG_POLARITY_Toggle;
	else if(mode==RISING) polarity = GPIOTE_CONFIG_POLARITY_LoToHi;
	else if(mode==FALLING) polarity = GPIOTE_CONFIG_POLARITY_HiToLo;
#if 0
	if((NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk)==0) {
		#if DEBUG_SERIAL
		Serial.printf("attachPinInterrupt0 %d\n",pin);
		#endif
		m_app_gpiote_input_event_handlers[0] = callback;
		
		NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
	} else if((NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN1_Msk)==0) {
		#if DEBUG_SERIAL
		Serial.printf("attachPinInterrupt1 %d\n",pin);
		#endif
		m_app_gpiote_input_event_handlers[1] = callback;
		
		NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos);
	} else if((NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN2_Msk)==0) {
		#if DEBUG_SERIAL
		Serial.printf("attachPinInterrupt2 %d\n",pin);
		#endif
		m_app_gpiote_input_event_handlers[2] = callback;
		NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN2_Enabled << GPIOTE_INTENSET_IN2_Pos);
	} else if((NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN3_Msk)==0) {
		#if DEBUG_SERIAL
		Serial.printf("attachPinInterrupt3 %d\n",pin);
		#endif
		m_app_gpiote_input_event_handlers[3] = callback;
		NRF_GPIOTE->CONFIG[3] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN3_Enabled << GPIOTE_INTENSET_IN3_Pos);
	}
#else
	//RFduino_pinWakeCallback(pin, mode, callback);
	
	if(pin==2) {
	  	m_app_gpiote_input_event_handlers[1] = callback;
		
		NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos);
	} else if(pin==3) {
		m_app_gpiote_input_event_handlers[2] = callback;
		NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN2_Enabled << GPIOTE_INTENSET_IN2_Pos);
	} else if(pin==4) {
		m_app_gpiote_input_event_handlers[3] = callback;
		NRF_GPIOTE->CONFIG[3] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN3_Enabled << GPIOTE_INTENSET_IN3_Pos);
	} else if(pin==1) {		
		m_app_gpiote_input_event_handlers[0] = callback;
		
		NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                      (pin << GPIOTE_CONFIG_PSEL_Pos) |
                                      (polarity << GPIOTE_CONFIG_POLARITY_Pos);
									  
		NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos);
	}
#endif
	NVIC_EnableIRQ(GPIOTE_IRQn);
	__enable_irq();
}

void detachInterruptPin(uint32_t pin)
{
	int i, found = 0;
	
	__disable_irq();
	
	BlueInnoPinMap(pin);
#if 0
	for(i=0;i<4;i++) {
		if((NRF_GPIOTE->CONFIG[i] & GPIOTE_CONFIG_PSEL_Msk)==(pin<<GPIOTE_CONFIG_PSEL_Pos)) {
			found = 1;
			break;
		}		
	}
	
	if(found) {
		if(i==0) NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN0_Msk);
		else if(i==1) NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN1_Msk);
		else if(i==2) NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN2_Msk);
		else if(i==3) NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN3_Msk);
	}
#else
	if(pin==2) {
	  	m_app_gpiote_input_event_handlers[1] = 0;
		NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN1_Msk);
	} else if(pin==3) {
		m_app_gpiote_input_event_handlers[2] = 0;
		NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN2_Msk);
	} else if(pin==4) {
		m_app_gpiote_input_event_handlers[3] = 0;
		NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN3_Msk);
	} else if(pin==1) {		
		m_app_gpiote_input_event_handlers[0] = 0;
		NRF_GPIOTE->INTENCLR = (GPIOTE_INTENSET_IN0_Msk);
	}
#endif

	__enable_irq();
}
*/
#define DEBUG_LED	1
static int count;
/*
void GPIOTE_IRQHandler()
{
	bool gpiote_in_evt   = false;
    bool gpiote_port_evt = false;
	
#if DEBUG_LED
pinMode(13, OUTPUT);
if(count++ & 1) digitalWrite(13, HIGH);
else digitalWrite(13, LOW);
#endif
#if DEBUG_SERIAL
  //Serial.write('+');
#endif

	if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        gpiote_in_evt            = true;

        if (m_app_gpiote_input_event_handlers[0])
        {
            m_app_gpiote_input_event_handlers[0]();
        }
    }

    if ((NRF_GPIOTE->EVENTS_IN[1] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN1_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[1] = 0;
        gpiote_in_evt            = true;

        if (m_app_gpiote_input_event_handlers[1])
        {
            m_app_gpiote_input_event_handlers[1]();
        }
    }

    if ((NRF_GPIOTE->EVENTS_IN[2] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN2_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[2] = 0;
        gpiote_in_evt            = true;

        if (m_app_gpiote_input_event_handlers[2])
        {
            m_app_gpiote_input_event_handlers[2]();
        }
    }

    if ((NRF_GPIOTE->EVENTS_IN[3] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN3_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[3] = 0;
        gpiote_in_evt            = true;

        if (m_app_gpiote_input_event_handlers[3])
        {
            m_app_gpiote_input_event_handlers[3]();
        }
    }
	
	
}
*/