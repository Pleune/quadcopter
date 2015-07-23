/**
 * INPUT TO ARDUINO OVER SERIAL
 * ****************************
 *
 * send "pc\n" to re calibrate the gyros
 *
 * send "pr\n" to reset the quaternion to (1,0,0,0)
 *
 * send "p\n" to do both
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

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

unsigned char SPITransmit(char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	return SPDR;
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
	 * Timer 1, 16 bit
	 * enable by setting prescaler only.
	 * everything else left normal (counter only)
	 *
	 * A prescalar of 256 overflows almost exactly once a seccond at 16Mhz
	 */
	TCCR1B = 0x03;//64

	/**
	 * Init NRF
	 */
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);

	/* Power on, enable error checker thing, RX */
	NRFStart();
	SPITransmit(0x20);
	SPITransmit(0x0F);
	NRFStop();

	/* Enable pipe 1, 1 byte */
	NRFStart();
	SPITransmit(0x31);
	SPITransmit(0x01);
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

	/* pipe 1 RX address */
	NRFStart();
	SPITransmit(0x2A);
	SPITransmit(0x44);
	SPITransmit(0x44);
	SPITransmit(0x44);
	SPITransmit(0x44);
	SPITransmit(0x44);
	NRFStop();

	/* actually power up */
	NRFCEHIGH();

	uint16_t lastclock = TCNT1;//16-bit read handled by compiler
	while(1)
	{
		NRFStart();
		unsigned char status = SPITransmit(0x61);
		if(status & 0x40)
		{
			/* just to generate the clock */
			unsigned char data = SPITransmit(0x00);
			NRFStop();

			/* clear the interrupt */
			NRFStart();
			SPITransmit(0x27);
			SPITransmit(0x40);
			NRFStop();

			writechar('a');
		} else {
			NRFStop();
		}
	}

	_delay_ms(1);
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
				/**
				 * this is the code that gets exxectued after the entire packet
				 * is received, based on the first charcter of the packet
				 */
			default:
				break;
			}
			newpacket=1;
		}
	}
}
