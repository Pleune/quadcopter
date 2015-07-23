#include <avr/io.h>
#include <util/delay.h>

/**
 * Serial baud and calculator
 */
#define BAUD 57600
#define BRC (F_CPU/16/BAUD-1)

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

	while(1)
	{
		NRFStart();
		SPITransmit(0x17);
		unsigned char status = SPITransmit(0x00);
		if(!(status & 0x01))
		{
			NRFStop();

			NRFStart();
			SPITransmit(0x61);
			/* just to generate the clock */
			unsigned char data = SPITransmit(0x00);
			NRFStop();

			writechar(data);
		} else {
			NRFStop();
		}
	}
}
