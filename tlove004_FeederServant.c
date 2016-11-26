/*  FeederServant, by Tyson Loveless
 *    Sleeps until receiving data from master to start motors
 *     after releasing feed, it checks the level of feed remaining
 *     in the box.  If the feed level has changed (logic low->high or
 *     high->low), the new level is sent to the Master to alert me
 *     to refill the box
 *  
 *  Project completed for CS122A at UCR, Fall 2016
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include "./nrf24.h"  /* credit: https://github.com/kehribar/nrf24L01_plus */

#define MOTORTIME 30000 //~30 seconds

uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint8_t startMotor;
volatile uint8_t getFlag = 0;
volatile uint8_t checkFeedFlag = 0;
volatile uint8_t feedLevel[1] = {0x00};
volatile uint8_t prevFeedLevel[1] = {0x00};
//uint8_t i, led = 0;
uint8_t* data;
uint8_t sendData[1] = {0x55};

void SendFeedLevel();

void RunMotors();

void GetData();

enum ServantState {INIT, IDLE, GO, SENDLEVEL} state;
void ServantTick()
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
			power_timer0_disable();
			power_timer1_disable();
			power_timer2_disable();
			
			//get read for sleep
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);	//as low as 5uA power usg
			cli();									//disable interrupts
			#if defined(BODS) && defined(BODSE)
				sleep_bod_disable();				//brown out detection
			#endif
			sei();									//enable interrupts
			state = IDLE;
			break;
		case IDLE:	
			sleep_mode();	// sets sleep enable bit, goes to sleep
			//wake up here,  sleep enable bit cleared at wakeup
			cli();
			if (getFlag) {
				GetData();
			}
			_delay_ms(1500);
			if (startMotor) {
				state = GO;   // start motors if received data to do so
				sei();
				break;
			}
			if (checkFeedFlag) {
				state = SENDLEVEL;
			}
			sei();
			break;
		case GO:
			cli();			  // disable interrupts
			RunMotors();
			sei();			  // enable interrupts
			state = SENDLEVEL;
			break;
		case SENDLEVEL:
			cli();
			_delay_ms(1500);
			SendFeedLevel();
			sei();
			state = IDLE;
			break;
		default:
			state = INIT;
			break;
	}
}


int main(void)
{
	//Setup Ports
	DDRA = 0xFF; PORTA = 0x00; // outputs, init to zero (Motor control)
	DDRB = 0xFF; PORTB = 0x00; // outputs, init to zero (SPI control)
	DDRC = 0x00; PORTC = 0x00; // input, pull-up (IR sensor[s])
	DDRD = 0x00; PORTD = 0xFF; // inputs, pull-down (ext interrupt)
	
	// enable external interrupt on any logic change for PD2(int0) and PD3(int1)
	SREG |= (1<<7);
	//int0
	EICRA &= ~(1<<ISC00);  
	EICRA |= (1<<ISC01);
	//int1
	EICRA &= ~(1<<ISC10);
	EICRA |= (1<<ISC11);
	EIMSK |= (1<<INT0)|(1<<INT1);  //enable int0 and int1
	while(1)
	{
		ServantTick();
	}
}

void SendFeedLevel()
{
	feedLevel[0] = (PINC & 0x01);			// get feed level reading
	// if feed level has changed, send new level
	if (feedLevel != prevFeedLevel)
	{
		if (feedLevel[0]) {
			sendData[0] = 0x55;
		}
		else {
			sendData[0] = 0x66;
		}
//		cli();
		SendRadioData(sendData);
//		_delay_ms(1500);
// 		nrf24_csn_digitalWrite(LOW);
// 		/* Write cmd to flush transmit FIFO */
// 		spi_transfer(FLUSH_TX);
// 		/* Pull up chip select */
// 		nrf24_csn_digitalWrite(HIGH);
//		sei();
		prevFeedLevel[0] = feedLevel[0];   //update feed memory
	}
	checkFeedFlag = 0;
}

void RunMotors()
{
	PORTA = 0x03;     // set bits 0 and 1 high on PORTA
	_delay_ms(MOTORTIME); // run motor for MOTORTIME ms
	PORTA = 0x00;
	startMotor = 0;
//	_delay_ms(1500);
//	checkFeedFlag = 1;
}

void GetData()
{
// 	if (getFlag) {
// //		for (i = 0; i < 10; i++) {
// // 			led = !led;
// // 			PORTA = led;
// // 			_delay_ms(500);
// // 		}
	data = GetRadioData();
//	_delay_ms(1500);
	if (data[0] == 0x77) {
		startMotor = 1;
	}
	while (nrf24_dataReady()) {
		data = GetRadioData();
	}
//	}
	getFlag = 0;
}

//interrupt service routine run when interrupted on PD2 (INT0)
ISR(INT0_vect)
{
	//make sure the interrupt is due to data arriving
	if (nrf24_dataReady()) {
// 		for (i = 0; i < 10; i++) {
// 			led = !led;
// 			PORTA = led;
// 			_delay_ms(200);
// 		}
//		_delay_ms(1500);
		getFlag = 1;
	}
}

ISR(INT1_vect) 
{
	checkFeedFlag = 1;
}