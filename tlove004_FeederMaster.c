/*  MasterServant, by Tyson Loveless
 *    Sends control signal to a servant connected to a feeder box
 *
 *  Can be modified to monitor several feeder boxes by updating the
 *    tx_address.  Multiple boxes can be controlled using the same logic
 *
 *  Master is actuated by either a button press on the board, through
 *    a timer, or through the receipt of foodLevel data from a feeder
 *    box.
 *  
 *  Button press and/or a timer sends a signal to all servant boxes
 *    to run their motors for however long those are set to run.
 *
 *  Receipt of foodLevel data from a feeder box initiates an alert
 *    so you can know to refill feed when it's low.
 * 
 *  Project completed for CS122A at UCR, Fall 2016
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "./nrf24.h"  /* credit: https://github.com/kehribar/nrf24L01_plus */

uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
volatile uint8_t feedIsLow;
volatile uint8_t getFlag;
volatile uint8_t sendFlag;
volatile uint16_t tot_overflow;
volatile uint8_t hours;
//uint8_t i, led = 0;
uint8_t *data;
uint8_t sendData[1] = {0x77};


/* Sends 0x77 to feeder box to initiate feed motors */
void SendMotorSignal()
{
	SendRadioData(sendData);
// 	for (i = 0; i < 10; i++) {
// 		led = !led;
// 		PORTA = led;
// 		_delay_ms(500);
// 	}
//	_delay_ms(1500);
// 	nrf24_csn_digitalWrite(LOW);
// 	/* Write cmd to flush transmit FIFO */
// 	spi_transfer(FLUSH_TX);
// 	/* Pull up chip select */
// 	nrf24_csn_digitalWrite(HIGH);
	sendFlag = 0;
}

void GetFeedLevel()
{
	data = GetRadioData();		// get incoming data from servant
//	PORTC = data[0];
	if (data[0] == 0x55) {     //signal from servant?
// 		for (i = 0; i < 6; i++) {
// 			led = !led;
// 			PORTA = led;
// 			_delay_ms(500);
// 		}
		//check feed level
		feedIsLow = 1;
	}
	else if (data[0] == 0x66) {
		feedIsLow = 0;
	}
	while (nrf24_dataReady()) {
		data = GetRadioData();
	}
	getFlag = 0;
}



enum MasterState {INIT, IDLE} state;
void MasterTick()
{
	//Actions
	switch (state) {
		case INIT:
			//setup nRF
			InitRadio(NRF_CH, PL_SIZE);			//setup channel and payload
			SetTransmitAddress(tx_address);		//transfer address set
			SetRadioAddress(rx_address);		//receive address set
		
			//disable unneeded functions
			power_adc_disable();
			power_spi_disable();
			power_twi_disable();
			//power_timer0_disable();
			//power_timer1_disable();
			//power_timer2_disable();
		
			//get ready for sleep
			set_sleep_mode(SLEEP_MODE_PWR_SAVE);
			cli();									//disable interrupts
			#if defined(BODS) && defined(BODSE)
			sleep_bod_disable();				//brown out detection
			#endif
			sei();									//enable interrupts
			state = IDLE;
			break;
		case IDLE:
			//_delay_ms(10000);
			//PORTA = PORTB = PORTC = PORTD = 0x00;
			sleep_mode();	// sets sleep enable bit, goes to sleep
			//wake up here,  sleep enable bit cleared at wakeup
			cli();
			if (sendFlag) {
				SendMotorSignal();
				//sei();
			}
			if (getFlag) {
				GetFeedLevel();
				if (feedIsLow) {
					PORTA = 0x01;
				}
				else {
					PORTA = 0x00;
				}
			}
			sei();
			break;
		default:
			state = INIT;
			break;
	}
}


int main(void)
{
	//Setup Ports
	DDRA = 0x01; PORTA = 0xFE; // PA1 output, other input pull-ups
	DDRB = 0xF8; PORTB = 0x07; // PB0:3 input pull-ups, other outputs
	DDRC = 0x00; PORTC = 0xFF; // all input pull-ups
	DDRD = 0x00; PORTD = 0xFF; // inputs, all pull-down (ext interrupts)
	//PORTD |= (1<<3);		   // pull-up (btn interrupt)
	
	// enable external interrupt on any logic change on PD2(int0) and PD3(int1)
	SREG |= (1<<7);
	//INT0
	EICRA |= (1<<ISC00);
//	EICRA &= ~(1<<ISC01);
	//INT1
 	EICRA |= (1<<ISC10);
// 	EICRA &= ~(1<<ISC11);
	//enable INT0 and INT1
	EIMSK |= (1<<INT0)|(1<<INT1);  
	
	// set up timer interrupt
	TCCR1B |= (1<<CS12)|(1<<CS10); // 1024 prescaler
	TCNT1 = 0;					   // init counter
	TIMSK1 |= (1<<TOIE1);           // enable timer1 overflow interrupt
	tot_overflow = 0;			   // init overflow counter
	while(1)
	{
		MasterTick();
	}
	return 0;
}


//interrupt on PD2 (INT0)
ISR(INT0_vect) 
{
// 	for (i = 0; i < 10; i++) {
// 		led = !led;
// 		PORTA = led;
// 		_delay_ms(200);
// 	}
//	_delay_ms(1500);
	sendFlag = 1;
//	_delay_ms(1000);
}

//interrupt on PD3 (INT1)
ISR(INT1_vect)
{
	if (nrf24_dataReady()) {
// 		for (i = 0; i < 10; i++) {
// 			led = !led;
// 			PORTA = led;
// 			_delay_ms(2500);
// 		}
//		_delay_ms(1500);
		getFlag = 1;
		
	}
// 	_delay_ms(1500);
// 	getFlag = 1;
}

ISR(TIMER1_OVF_vect)
{
	// keep a track of number of overflows
	tot_overflow++;
	
	// check for number of overflows here itself
	// 429 overflows == 1 hour delay (approx.)
	if (tot_overflow >= 429)
	{
		if (hours++ == 12) { // every 12 hours send feed signal
			sendFlag = 1;
			hours = 0;
		}
		tot_overflow = 0;   // reset overflow counter
	}
}