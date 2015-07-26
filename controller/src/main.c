#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/**
 * Serial baud and calculator
 */
#define BAUD 57600
#define BRC (F_CPU/16/BAUD-1)

/**
 * the char that signifies the end of a serial packet incommint
 */
#define EOP '\n'

/**
 * The CE pin on the nRF
 */
#define NRFCEHIGH() (PORTB |= 0x01)
#define NRFCELOW() (PORTB &= 0xFE)

uint16_t throttlepoint = 100;
uint8_t inv0 = 0, inv1 = 0, inv2 = 0, inv3 = 1;

char string[64];

uint8_t throttle = 62;

uint8_t calibrate = 0;

#define a0min calibrations[0]
#define a0max calibrations[1]
#define a0mid calibrations[2]
#define a1min calibrations[3]
#define a1max calibrations[4]
#define a1mid calibrations[5]
#define a2min calibrations[6]
#define a2max calibrations[7]
#define a2mid calibrations[8]
#define a3min calibrations[9]
#define a3max calibrations[10]
#define a3mid calibrations[11]
uint8_t calibrations[12];

void writechar( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

inline void writestring(const char *s)
{
	int i=0;
	while(s[i]!=0)
		writechar(s[i++]);
}

void SPITransmit(char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
}

void NRFStart()
{
	PORTD &= ~(1<<DDD4);
	_delay_us(80);
}

void NRFStop()
{
	PORTD |= (1<<DDD4);
	_delay_us(80);
}

int main(void)
{
	double range0, range1, range2, range3;

	DDRB = 0xFF;
	PORTB = 0xFE;//keep CE on NRF low

	DDRD = 0xFF;
	PORTD = 0xFF;
	//enable extermal interrupts
	sei();

	sleep_disable();

	/**
	 * This sets the boud rate to the value defined above,
	 * it calculates the value needed in UBRR0 with the BRC macro.
	 *
	 * Serial transmit is then enabled, and the serial data recieve interrupt is enabled
	 * with UCSR0B.
	 *
	 * 8 bit serial with 2 stop bits is then enabled with UCSR0C,
	 * the parity generator/checker is commented out.
	 */
	UBRR0H = (BRC >> 8);
	UBRR0L = BRC;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (3 << UCSZ00);// | (1 << UPM01);

	/**
	 * Init NRF
	 */
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

	/* Power on, enable error checker thing, RX */
	NRFStart();
	SPITransmit(0x20);
	SPITransmit(0x0E);
	NRFStop();

	/* Enable pipe 1 */
	NRFStart();
	SPITransmit(0x31);
	SPITransmit(17);
	NRFStop();

	/* turn off ShockBurst autoACK shit */
	NRFStart();
	SPITransmit(0x21);
	SPITransmit(0x00);
	NRFStop();

	/* 256Kbpsi, high power */
	NRFStart();
	SPITransmit(0x26);
	SPITransmit(0x2F);
	NRFStop();

	/* actually power up */
	NRFCEHIGH();

	eeprom_read_block(calibrations, 0x00, sizeof(calibrations));

	sprintf(string, "a0:%i:%i:%i\na1:%i:%i:%i\na2:%i:%i:%i\na3:%i:%i:%i\n", a0min, a0max, a0mid,
			a1min, a1max, a1mid,
			a2min, a2max, a2mid,
			a3min, a3max, a3mid);
	writestring(string);

	/* actually a coefficent derrived from range */
	range0 = 1.0 / (double)(a0max - a0min);
	range1 = 63.0 / (double)(a1max - a1mid);
	range2 = 1.0 / (double)(a2max - a2min);
	range3 = 1.0 / (double)(a3max - a3min);

	while(1)
	{
		int16_t a0, a1, a2, a3;
		static uint8_t calibrating = 0;
		ADMUX = 0x60;
		ADCSRA = 0xC6;
		while(ADCSRA & 0x40);
		a0 = inv0 ? 255-ADCH : ADCH;
		ADMUX = 0x61;
		ADCSRA = 0xC6;
		while(ADCSRA & 0x40);
		a1 = inv1 ? 255-ADCH : ADCH;
		ADMUX = 0x62;
		ADCSRA = 0xC6;
		while(ADCSRA & 0x40);
		a2 = inv2 ? 255-ADCH: ADCH;
		ADMUX = 0x63;
		ADCSRA = 0xC6;
		while(ADCSRA & 0x40);
		a3 = inv3 ? 255-ADCH : ADCH;

		if(calibrating)
		{
			a0min = a0 < a0min ? a0 : a0min;
			a1min = a1 < a1min ? a1 : a1min;
			a2min = a2 < a2min ? a2 : a2min;
			a3min = a3 < a3min ? a3 : a3min;

			a0max = a0 > a0max ? a0 : a0max;
			a1max = a1 > a1max ? a1 : a1max;
			a2max = a2 > a2max ? a2 : a2max;
			a3max = a3 > a3max ? a3 : a3max;

			if(!calibrate)
			{
				a0mid = a0;
				a1mid = a1;
				a2mid = a2;
				a3mid = a3;

				calibrating = calibrate;
				eeprom_update_block(calibrations, 0x00, sizeof(calibrations));

				range0 = 1.0 / (double)(a0max - a0min);
				range1 = 63.0 / (double)(a1max - a1mid);
				range2 = 1.0 / (double)(a2max - a2min);
				range3 = 1.0 / (double)(a3max - a3min);
			}
		} else {
			if(calibrate)
			{
				calibrating = 1;

				a0max = 0;
				a0min = 255;
				a1max = 0;
				a1min = 255;
				a2max = 0;
				a2min = 255;
				a3max = 0;
				a3min = 255;
			}

			double z = (double)(a0 - a0mid) * range0;
			uint8_t throttle = a1 < a1mid ? 62 : (a1 - a1mid) * range1 + 62;
			double x = (double)(a2 - a2mid) * range2;
			double y = (double)(a3 - a3mid) * range3;

			int16_t axisx = -y;
			int16_t axisy = x;
			//axisz = 0;

			double angle = sqrt(y*y + x*x);
			double sinfix = sin(angle);

			double q0 = cos(angle);
			double q1 = axisx * sinfix;
			double q2 = axisy * sinfix;
			double q3 = 0;

			sprintf(string, "0:%f\t1:%f\t2:%f\t3:%f\n", q0, q1, q2, q3);
			writestring(string);

			NRFStart();
			SPITransmit(0xE1);
			NRFStop();

			NRFStart();
			SPITransmit(0xA0);
			SPITransmit(0xAA);//just to check on the other side

			SPITransmit(throttle);

			SPITransmit(((char *)&q0)[0]);
			SPITransmit(((char *)&q0)[1]);
			SPITransmit(((char *)&q0)[2]);
			SPITransmit(((char *)&q0)[3]);

			SPITransmit(((char *)&q1)[0]);
			SPITransmit(((char *)&q1)[1]);
			SPITransmit(((char *)&q1)[2]);
			SPITransmit(((char *)&q1)[3]);

			SPITransmit(((char *)&q2)[0]);
			SPITransmit(((char *)&q2)[1]);
			SPITransmit(((char *)&q2)[2]);
			SPITransmit(((char *)&q2)[3]);

			SPITransmit(((char *)&q3)[0]);
			SPITransmit(((char *)&q3)[1]);
			SPITransmit(((char *)&q3)[2]);
			SPITransmit(((char *)&q3)[3]);

			NRFStop();

			_delay_ms(1);

		}
	}
}

/**
 * This is the serial input subroutine
 *
 * each group of charcters between new lines is a 'packet'
 *
 * a packet-type is denoted by the first charcter
 */
ISR(USART_RX_vect)
{
	static uint8_t newpacket=1;
	static uint8_t packettype;
	static uint8_t packetcounter;
	static char packetbuffer[20];

	char c = UDR0;
	if(newpacket)
	{
		newpacket=0;
		packettype = c;
		packetcounter=0;
	} else {
		if(c != EOP)
		{
			packetbuffer[packetcounter++] = c;
		} else {
			packetbuffer[packetcounter]=0;
			switch(packettype)
			{
				case 'c':
					if(packetbuffer[0] == 's')
						calibrate = 1;
					else if(packetbuffer[0] == 'c')
						calibrate = 0;
					break;
			default:
				break;
			}
			newpacket=1;
		}
	}
}
