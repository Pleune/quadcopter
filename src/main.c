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
 * The I2C speed and calculator
 * F_SCL is the clock speed
 */
#define F_SCL 400000
#define TWBR_CALC ((F_CPU-16*F_SCL)/(2*F_SCL))

/*The charcter that seperates serial input packets*/
#define EOP '\n'

/**
 * I2C address of the MPU6050
 * can be changed to 0x69 by pulling
 * the ADD pin high
 */
#define MPU6050ADDR 0x68

/**
 * These are the values read from the IMU.
 * The z_ values are zero-state correction values.
 */
double ax, ay, az = 0;
double gx, gy, gz = 0;
double z_gx, z_gy, z_gz = 0;
double temp = 0;

/*multiplyer to the accelerometer error part added to the qyro data*/
double accKp = .1;

/*the orientation quaternion*/
double q0 = 1, q1 = 0, q2 = 0, q3 = 0;

/**
 * Shared variables:
 *
 * These are used by the serial interrupt to communicate with
 * the main loop.
 *
 * flag bits:
 * 7: print deltaT
 * 6: reset q[0-3]
 */
uint8_t flags = 0x00;

/**************************************************************************************/

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

uint8_t I2CGetStatus(void)
{
	uint8_t status;
	status = TWSR & 0xF8;
	return status;
}

/*true if sucess*/
int I2CStart(void)
{
	TWCR |= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	uint8_t status = I2CGetStatus();
	if(status == 0x28)
	writechar('h');
	return (status == 0x08) || (status == 0x10);
}

void I2CStop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

/*true if ack*/
int I2CWrite(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return (I2CGetStatus() == 0x28);
}

uint8_t I2CReadACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t I2CReadNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

int I2CStartWriteTo(uint8_t u8data)
{
	if(!I2CStart())
		{ I2CStop(); return -1; }

	TWDR = u8data<<1;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	if(I2CGetStatus()!=0x18)
		{ I2CStop(); return -1; }

	return 0;
}

int I2CStartReadFrom(uint8_t u8data)
{
	if(!I2CStart())
		{ I2CStop(); return -1; }

	TWDR = (u8data<<1) | 0x01;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	if(I2CGetStatus()!=0x40)
		{ I2CStop(); return -1; }

	return 0;
}

int mpu6050_init()
{
	//set 2000 deg/s
	if(I2CStartWriteTo(MPU6050ADDR))
		return -1;

	if(!I2CWrite(0x1b))
		{ I2CStop(); return -1; }

	if(!I2CWrite(0x18))
		{ I2CStop(); return -1; }

	I2CStop();


	//turn on
	if(I2CStartWriteTo(MPU6050ADDR))
		return -1;

	if(!I2CWrite(0x6b))//PWR_MGMT_1 Register
		{ I2CStop(); return -1; }

	if(!I2CWrite(0))//wake system
		{ I2CStop(); return -1; }

	I2CStop();

	return 0;
}

int mpu6050_updateall()
{
	if(I2CStartWriteTo(MPU6050ADDR))
		return -1;

	if(!I2CWrite(0x3b))
		{ I2CStop(); return -1; }

	if(I2CStartReadFrom(MPU6050ADDR))
		return -1;

	/**
	 * Begin reading values sequentally
	 * all values are 16 bit, transfered as big endian.
	 */
	ax = (I2CReadACK()<<8) | I2CReadACK();
	ay = (I2CReadACK()<<8) | I2CReadACK();
	az = (I2CReadACK()<<8) | I2CReadACK();
	temp = (double)((I2CReadACK()<<8) | I2CReadACK()) / 340.00 + 36.53;
	gx = ((I2CReadACK()<<8) | I2CReadACK()) - z_gx;
	gy = ((I2CReadACK()<<8) | I2CReadACK()) - z_gy;
	gz = ((I2CReadACK()<<8) | I2CReadNACK()) - z_gz;

	I2CStop();

	return 0;
}

void mpu6050_calibrate()
{
	z_gx = 0;
	z_gy = 0;
	z_gz = 0;

	long ax = 0;
	long ay = 0;
	long az = 0;

	int i;
	for(i=0; i<500; i++)
	{
		mpu6050_updateall();
		ax += gx;
		ay += gy;
		az += gz;

		_delay_ms(10);
	}

	z_gx = (long double)ax / 500;
	z_gy = (long double)ay / 500;
	z_gz = (long double)az / 500;
}

int main(void)
{
	//values for use in the trapizoydal sum
	double lastgx = gx, lastgy = gy, lastgz = gz;

	//print information in the loop every x loops,
	int p = 0;

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
	 * this starts the I2C unit on the atmega chip at 400Khz clock
	 */
	TWSR = 0;
	TWBR = TWBR_CALC;
	TWCR = (1<<TWEN);

	if(mpu6050_init())
	{
		writestring("mpu init failed\n");
	}

	_delay_ms(2000);

	mpu6050_calibrate();

	uint16_t lastclock = TCNT1;//16-bit read handled by compiler
	while(1)
	{
		uint16_t clock;
		uint16_t dt;
		long double gyroK;

		clock = TCNT1;

		/**
		 * Read sensors into memory here for the most accurate deltaT/sensor-data combination.
		 */
		mpu6050_updateall();

		/**
		 * Calculate deltaT here,
		 * and reset it for next loop.
		 *
		 * deltaT is calculated based on the 16-bit timer0, which effectly counts clock cycles.
		 */
		dt = clock - lastclock;
		lastclock = clock;

		/******************************************************************
		 * Start calculations
		 */
		gyroK = .25*(long double)dt * (((long double)/* timer1 prescalar */64*(long double)/* max deg/s */2000)/((long double)F_CPU*(long double)/* 2^15 */32768)) * /*Deg to rad*/(3.14159265359/180);

		if(flags&0x40)
		{
			q0 = 1;
			q1 = 0;
			q2 = 0;
			q3 = 0;

			flags &= 0xBF;
		}
		else
		{
			/**
			 * This is the code that keeps track of the orientation of the quad copter.
			 *
			 * HOWEVER, the quarternion keeps track of how to rotate the world-space realitive to the quadcopter-space,
			 * NOT the other way around.
			 *
			 * That means the common equasion vnew = Q * vorig * Q^-1 will get you a vector in world space from a vector in quad space.
			 *
			 * vorig is a quarternion that is (0, v1, v2, v3)
			 * v[1-3] are just a vector.
			 *
			 * The same thing for vnew, q0 will always end up as 0.
			 *
			 * world-space is realitive to the world/gravity/etc.
			 * quad-space is realitive to the axies drawn on the IMO
			 *
			 * All quarternions q are (q0, q1, q2, q3)
			 */

			/**
			 * Trapizoydal sum for greater accuracy.
			 */
			double gx_ = (lastgx+gx) * gyroK;
			double gy_ = (lastgy+gy) * gyroK;
			double gz_ = (lastgz+gz) * gyroK;

			double mag = sqrt(ax*ax + ay*ay + az*az);

			/**
			 * Normalized UP!!! vector
			 * According to any accelerometer, everything is accelerating up at 1g.
			 *
			 * Works best with the normalized quarternions and
			 * cross product to produce onsistant results.
			 */
			double ax_ = ax / mag;
			double ay_ = ay / mag;
			double az_ = az / mag;

			/**
			 * This uses the oppisite of the
			 * vnew = Q * vorig * Q^-1
			 * It uses:
			 * vnew = Q-1 * vorig * Q
			 * to go from world-space to quad-space
			 *
			 * For any quarternions Q,
			 * Q * Q^-1 = 1, or the multiplicitive quarternions idenity (1, 0, 0, 0)
			 * this only happens if the multiplications are right after each other,
			 * so Q * B * Q^-1 != B
			 *
			 * The inverse of a quaternions is however the oppisite of the non-inverted quaternion...
			 * This means that when using the quaternion in the equasion above with inverted quaternions,
			 * it preforms the reverse of the rotation.
			 *
			 * Also note that the inverse of a unit quaternion is also the conjugate
			 * to conjugate a quaternion, invert q1, q2 and q3.
			 *
			 * * * *
			 *
			 * What the below block of code does is converts the world-space vector (0, 0, 1)
			 * into the same vector in quad-sapce by the equasion Q-1 * (0, [0, 0, 1]) * Q...
			 *
			 * estx are parts 1-3 of the resulting quaternion, part 0 will always be 0.
			 *
			 * This should be the same vector that the accelerometer reads out
			 * if the gyros were perfect.
			 *
			 * Each component of est is then divided by 2, simply to save instructions.
			 */
			double estx = 2*q1*q3 - 2*q0*q2;
			double esty = 2*q0*q1 + 2*q2*q3;
			double estz = (q0*q0 - q1*q1 - q2*q2 + q3*q3);

			/**
			 * The error calculated here is the cross product of (the accelerometer vector) x (the vector from above).
			 *
			 * The cross product essentally calculates rotational velocities around each axis
			 * with the correct sign to move the vector above to the accelerometer vector
			 * according to the right hand rule.
			 *
			 * To get a more proportional correction to how far apart the vectors are,
			 * use atan2.
			 */
			double errorx = ay_*estz - az_*esty;
			double errory = az_*estx - ax_*estz;
			double errorz = ax_*esty - ay_*estx;

			/**
			 * The error from above is now just added to the gyro readiong
			 * for this iteration...
			 *
			 * This means that the larger the qyro readings,
			 * then the larger of a weight they have becasue the
			 * error part is never scaled proportionally to
			 * the qyro readings.
			 *
			 * Ex:
			 * in
			 * 2+10
			 * 2 has a 2/12 weight
			 *
			 * While in
			 * 2 + 100
			 * 2 has a 2/102 weight
			 */
			gx_ += accKp * errorx * dt * 64 / 16000000;
			gy_ += accKp * errory * dt * 64 / 16000000;
			gz_ += accKp * errorz * dt * 64 / 16000000;

			/**
			 * This block takes integrated qyro velocities (change in angle) and integrates a quaternion with them.
			 * it can be done seperatly from the accelerometer junk above.
			 *
			 * It uses a formula that can be derrived from an extension of Euler's formula
			 * shown here: https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
			 *
			 * Basically,
			 * Qold = Qold + Qold*(0, gx, gy, gz)*dT
			 * integrates a quaternion based on the gyro velocities.
			 *
			 * The gyro values must be in radians.
			 */
			q0 += -q1*gx_ - q2*gy_ - q3*gz_;
			q1 += q0*gx_ + q2*gz_ - q3*gy_;
			q2 += q0*gy_ - q1*gz_ + q3*gx_;
			q3 += q0*gz_ + q1*gy_ - q2*gx_;

			/**
			 * we must now shrink the quaternion back down to a unit vector
			 * this does not affect the rotation that it describes
			 */
			mag = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

			q0 /= mag;
			q1 /= mag;
			q2 /= mag;
			q3 /= mag;

			/**
			 * uncomment this to print the estimated up vector from the last
			 * iteration
			 */
			//char string[128];
			//sprintf(string, "ex: %f\tey: %f\tez: %f\n", estx, esty, estz);
			//writestring(string);
		}

		lastgx = gx;
		lastgy = gy;
		lastgz = gz;

		/*print the quaternion every 100 loops*/
		if(p++ == 100)
		{
			p = 0;
			char string[128];
			sprintf(string, "q0: %f\tq1: %f\tq2: %f\tq3: %f\tdt: %u\n", q0, q1, q2, q3, dt);
			writestring(string);
		}

		if(flags&0x20)
		{
			mpu6050_calibrate();
			flags &= 0xDF;
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
				/**
				 * this is the code that gets exxectued after the entire packet
				 * is received, based on the first charcter of the packet
				 */
			case 'p':
				if(packetcounter==0)
					flags=0xFF;
				else {
					int i;
					for(i=0; i<packetcounter; i++)
					{
						switch(packetbuffer[i])
						{
						case 't'://print deltaT
							flags |= 0x80;
							break;
						case 'r':
							flags |=0x40;
							break;
						case 'c':
							flags |=0x20;
							break;
						default:
							break;
						}
					}
				}
				break;
			default:
				break;
			}
			newpacket=1;
		}
	}
}
