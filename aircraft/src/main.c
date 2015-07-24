#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define GYRO

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
 * The CE pin on the nRF
 */
#define NRFCEHIGH() (PORTB |= 0x01)
#define NRFCELOW() (PORTB &= 0xFE)

/**
 * DEBUG LEDS
 */
#define LED1ON() (PORTD |= 0x80)
#define LED1OFF() (PORTD &= 0x7F)
#define LED2ON() (PORTB |= 0x02)
#define LED2OFF() (PORTB &= 0xFD)
#define LED3ON() (PORTB |= 0x04)
#define LED3OFF() (PORTB &= 0xFB)
#define LED4ON() (PORTB |= 0x10)
#define LED4OFF() (PORTB &= 0xEF)

int enabled = 0;

int throttle = 62;

uint8_t motor1 = 62;
uint8_t motor2 = 62;
uint8_t motor3 = 62;
uint8_t motor4 = 62;

double kProll = 5.0;
double kIroll = 3.0;
double kDroll = 0.3;

double kPpitch = 5.0;
double kIpitch = 3.0;
double kDpitch = 0.3;

double kPyaw = 5.0;
double kIyaw = 10.0;
double kDyaw = 0.05;

double iMaxroll = 20;
double iMaxpitch = 20;
double iMaxyaw = 20;

double iMinroll = -20;
double iMinpitch = -20;
double iMinyaw = -20;

double maxcommandroll = 25;
double maxcommandpitch = 25;
double maxcommandyaw = 25;

double mincommandroll = -25;
double mincommandpitch = -25;
double mincommandyaw = -25;

double smoothness = 10;

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

int mpu6050Init()
{


	//turn on
	if(I2CStartWriteTo(MPU6050ADDR))
		return -1;

	if(!I2CWrite(0x6b))//PWR_MGMT_1 Register
		{ I2CStop(); return -1; }

	if(!I2CWrite(0))//wake system
		{ I2CStop(); return -1; }

	I2CStop();
	//set 2000 deg/s
	if(I2CStartWriteTo(MPU6050ADDR))
		return -1;

	if(!I2CWrite(0x1b))
		{ I2CStop(); return -1; }

	if(!I2CWrite(0x18))
		{ I2CStop(); return -1; }

	I2CStop();

	return 0;
}

int mpu6050UpdateAll()
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

void mpu6050Calibrate()
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
		mpu6050UpdateAll();
		ax += gx;
		ay += gy;
		az += gz;

		_delay_ms(10);
	}

	z_gx = (long double)ax / 500;
	z_gy = (long double)ay / 500;
	z_gz = (long double)az / 500;
}

unsigned char SPITransmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

void NRFStart()
{
	PORTD &= ~(1<<DDD2);
	_delay_us(80);
}

void NRFStop()
{
	PORTD |= (1<<DDD2);
	_delay_us(80);
}

void NRFInit()
{
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

	/* Transmit address 0x44444444 */
	NRFStart();
	SPITransmit(0x30);
	SPITransmit(0x44);
	SPITransmit(0x44);
	SPITransmit(0x44);
	SPITransmit(0x44);
	SPITransmit(0x44);
	NRFStop();

	/* actually power up */
	NRFCEHIGH();
}

void msg(char *s)
{
	int index = 0;

	NRFCELOW();

	_delay_us(80);

	/* switch to TX mode */
	NRFStart();
	SPITransmit(0x20);
	SPITransmit(0x0E);
	NRFStop();

	NRFStart();
	SPITransmit(0x25);
	SPITransmit(0x0F);
	NRFStop();

	NRFCEHIGH();

	_delay_us(80);

	while(s[index]!=0)
	{
		while(1) {
			NRFStart();
			unsigned char status = SPITransmit(0xA0);

			if(status & 0x01)
				NRFStop();
			else
				break;
		}
		SPITransmit(s[index++]);
		NRFStop();

		/* give the RX time to read a constant stream */
		_delay_us(250);
	}

	while(1)
	{
		SPITransmit(0x17);
		unsigned char status = SPITransmit(0x00);

		if(status & 0x10)
			break;
	}

	NRFCELOW();

	_delay_us(80);

	/* switch to RX mode */
	NRFStart();
	SPITransmit(0x20);
	SPITransmit(0x0F);
	NRFStop();

	NRFStart();
	SPITransmit(0x25);
	SPITransmit(0x02);
	NRFStop();


	NRFCEHIGH();

	_delay_us(80);

}

int main(void)
{
	//values for use in the trapizoydal sum
	double lastgx = gx, lastgy = gy, lastgz = gz;

	//print information in the loop every x loops,
	int p = 0;

	DDRB = 0xFF;
	PORTB = 0xFE;//keep CE on NRF low

	DDRD = 0xFF;
	PORTD = 0xFF;

	sei();
	sleep_disable();

	/**
	 * Timer 1, 16 bit
	 * enable by setting prescaler only.
	 * everything else left normal (counter only)
	 *
	 * A prescalar of 256 overflows almost exactly once a seccond at 16Mhz
	 */
#define T1PSK 64//set this the same as the prescalar below
	TCCR1B = 0x03;//64

	/**
	 * PWM
	 */
	/* enable fastpwm modes, all 4 pins */
	TCCR0A = 0xA3;
	TCCR2A = 0xA3;

	/* interrupt on overflow */
	TIMSK0 = 0x01;
	TIMSK2 = 0x01;

	/* 1ms on each pin */
	OCR0A = motor1;
	OCR0B = motor2;
	OCR2A = motor3;
	OCR2B = motor4;

	/* 256 prescalar, 244hz, starts clocks */
	TCCR0B = 0x04;
	TCCR2B = 0x06;

	/* give things a chance to boot */
	_delay_ms(1000);

	/**
	 * this starts the I2C unit on the atmega chip at 400Khz clock
	 */
	TWSR = 0;
	TWBR = TWBR_CALC;
	TWCR = (1<<TWEN);

	mpu6050Init();

	/**
	 * Init NRF
	 */
	UBRR0 = 0;
	/* Setting the XCKn port pin as output, enables master
	 * mode. */
	//done above --> XCKn_DDR |= (1<<XCKn);
	/* Set MSPI mode of operation and SPI data mode 0. */
	UCSR0C = (1<<UMSEL01)|(1<<UMSEL00)|(0<<UCPHA0)|(0<<UCPOL0);
	/* Enable receiver and transmitter. */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the
	 * transmitter is enabled
	 *
	 * Fcpu/4
	 */
	UBRR0 = 1;

	NRFInit();

	_delay_ms(2600);

	motor1 = 62;
	motor2 = 62;
	motor3 = 62;
	motor4 = 62;

	msg("starting\n");

	_delay_ms(100);
	LED1OFF();
	_delay_ms(100);
	LED2OFF();
	_delay_ms(100);
	LED3OFF();
	_delay_ms(100);
	LED4OFF();
	_delay_ms(200);
	LED4ON();
	_delay_ms(100);
	LED3ON();
	_delay_ms(100);
	LED2ON();
	_delay_ms(100);
	LED1ON();

#ifdef GYRO

	mpu6050Calibrate();
	msg("calibrated\n");
#endif

	uint16_t lastclock = TCNT1;//16-bit read handled by compiler
	while(1)
	{
		uint16_t clock;
		uint16_t dt;

		clock = TCNT1;
#ifdef GYRO
		/**
		 * Read sensors into memory here for the most accurate deltaT/sensor-data combination.
		 */
		mpu6050UpdateAll();
#endif
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
#ifdef GYRO
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
#define gyroK (.25*(long double)dt * (((long double)/* timer1 prescalar */ T1PSK *(long double)/* max deg/s */2000)/((long double)F_CPU*(long double)/* 2^15 */32768)) * /*Deg to rad*/(3.14159265359/180))
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
			gx_ += accKp * errorx * dt * T1PSK / 16000000;
			gy_ += accKp * errory * dt * T1PSK / 16000000;
			gz_ += accKp * errorz * dt * T1PSK / 16000000;

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
		}
#endif
		/**
		 * http://arxiv.org/pdf/0811.2889.pdf
		 */
		//double qd0;
		double qd1;
		double qd2;
		double qd3;

		//target orientation
		double qt0 = 1.0;
		double qt1 = 0.0;
		double qt2 = 0.0;
		double qt3 = 0.0;

		if(qt0*q0 + qt1*q1 + qt2*q2 + qt3*q3 > 0)
		{
			qt0 -= q0;
			qt1 -= q1;
			qt2 -= q2;
			qt3 -= q3;
			//qd0 = q0*qt0 + q1*qt1 + q2*qt2 + q3*qt3;
			qd1 = q0*qt1 - q1*qt0 + q2*qt3 - q3*qt2;
			qd2 = q0*qt2 - q1*qt3 - q2*qt0 + q3*qt1;
			qd3 = q0*qt3 + q1*qt2 - q2*qt1 - q3*qt0;
		} else {
			qt0 -= q0;
			qt1 -= q1;
			qt2 -= q2;
			qt3 -= q3;
			//qd0 = -q0*qt0 - q1*qt1 - q2*qt2 - q3*qt3;
			qd1 = -q0*qt1 + q1*qt0 - q2*qt3 + q3*qt2;
			qd2 = -q0*qt2 + q1*qt3 + q2*qt0 - q3*qt1;
			qd3 = -q0*qt3 - q1*qt2 + q2*qt1 + q3*qt0;
		}


#define Kdt ((double)64.0 / (double)F_CPU)
#define gyroK (long double)/* max deg/s */2000 / ((long double)/* 2^15 */32768) * /*Deg to rad*/(3.14159265359/180)
		long double dT_ = (double)dt * Kdt;
		/**
		 * ask the nrf for a RXx packet,
		 * meanwhile it shifts out the status register.
		 *
		 * if the status reg says there is a packet, it will be shifted out next.
		 */
		static long double time = 0;

		NRFStart();
		unsigned char status = SPITransmit(0x61);

		if(status & 0x40)
		{
			/* just to generate the clock */
			unsigned char data = SPITransmit(0x00);

			throttle = data;

			NRFStop();

			NRFStart();
			SPITransmit(0x27);
			SPITransmit(0x40);
			NRFStop();
			time = 0;
		} else {
			time += dT_;

			if(time > .5)
				throttle = 62;

			NRFStop();
		}

		if(flags&0x20)
		{
			mpu6050Calibrate();
			flags &= 0xDF;
		}

		if(throttle > 62)
		{
			double commandpitch;
			double commandroll;
			double commandyaw;

			double gx_ = gx * gyroK;
			double gy_ = gy * gyroK;
			double gz_ = gz * gyroK;

			static double gx_s = 0;
			static double gy_s = 0;
			static double gz_s = 0;

			double weight = smoothness * dT_;

			gx_s = gx_s * (1.0 - weight) + gx_ * (weight);
			gy_s = gy_s * (1.0 - weight) + gy_ * (weight);
			gz_s = gz_s * (1.0 - weight) + gz_ * (weight);

			double dInput;
			double error;

			static double iTermroll = 0;
			static double iTermpitch = 0;
			static double iTermyaw = 0;

			static double lastinputroll = 0;
			static double lastinputpitch = 0;
			static double lastinputyaw = 0;

			qd1 *= 5.0;
			qd2 *= 5.0;
			qd3 *= 5.0;

			static double qd1_s = 0;
			static double qd2_s = 0;
			static double qd3_s = 0;

			double qd_sweight = .6;

			qd1_s = qd1_s * (1.0 - qd_sweight) + qd1 * (qd_sweight);
			qd2_s = qd2_s * (1.0 - qd_sweight) + qd2 * (qd_sweight);
			qd3_s = qd3_s * (1.0 - qd_sweight) + qd3 * (qd_sweight);

			error = qd1_s - gx_;
			dInput = gx_s - lastinputroll;
			iTermroll += kIroll * dT_ * error;
			if(iTermroll > iMaxroll)
				iTermroll = iMaxroll;
			else if(iTermroll < iMinroll)
				iTermroll = iMinroll;
			commandroll = kProll * error + iTermroll - kDroll * dInput / dT_;
			if(commandroll > maxcommandroll)
				commandroll = maxcommandroll;
			else if(commandroll < mincommandroll)
				commandroll = mincommandroll;
			lastinputroll = gx_s;

			error = qd2_s - gy_;
			dInput = gy_s - lastinputpitch;
			iTermpitch += kIpitch * dT_ * error;
			if(iTermpitch > iMaxpitch)
				iTermpitch = iMaxpitch;
			else if(iTermpitch < iMinpitch)
				iTermpitch = iMinpitch;
			commandpitch = kPpitch * error + iTermpitch - kDpitch * dInput / dT_;
			if(commandpitch > maxcommandpitch)
				commandpitch = maxcommandpitch;
			else if(commandpitch < mincommandpitch)
				commandpitch = mincommandpitch;
			lastinputpitch = gy_s;

			error = qd3_s - gz_;
			dInput = gz_s - lastinputyaw;
			iTermyaw += kIyaw * dT_ * error;
			if(iTermyaw > iMaxyaw)
				iTermyaw = iMaxyaw;
			else if(iTermyaw < iMinyaw)
				iTermyaw = iMinyaw;
			commandyaw = kPyaw * error + iTermyaw - kDyaw * dInput / dT_;
			if(commandyaw > maxcommandyaw)
				commandyaw = maxcommandyaw;
			else if(commandyaw < mincommandyaw)
				commandyaw = mincommandyaw;
			lastinputyaw = gz_s;

		if(p++ == 200)
		{
			p = 0;
			char string[64];
			sprintf(string, "1: %f\t2: %f\t3: %f\t4: %i\n", commandroll, commandpitch, commandyaw, throttle);
			msg(string);
		}

			motor1 = (int)(throttle + commandroll - commandpitch + commandyaw);
			motor2 = (int)(throttle - commandroll - commandpitch - commandyaw);
			motor3 = (int)(throttle - commandroll + commandpitch + commandyaw);
			motor4 = (int)(throttle + commandroll + commandpitch - commandyaw);

			if(motor1 > 125)
				motor1 = 125;
			else if(motor1 < 62)
				motor1 = 62;
			if(motor2 > 125)
				motor2 = 125;
			else if(motor2 < 62)
				motor2 = 62;
			if(motor3 > 125)
				motor3 = 125;
			else if(motor3 < 62)
				motor3 = 62;
			if(motor4 > 125)
				motor4 = 125;
			else if(motor4 < 62)
				motor4 = 62;
		} else {
			motor1 = 62;
			motor2 = 62;
			motor3 = 62;
			motor4 = 62;

		if(p++ == 200)
		{
			p = 0;
			char string[64];
			sprintf(string, "1: %i\n", throttle);
			msg(string);
		}
		}

		lastgx = gx;
		lastgy = gy;
		lastgz = gz;
	}
}

ISR(TIMER0_OVF_vect)
{
	OCR0A = motor1;
	OCR0B = motor2;
}

ISR(TIMER2_OVF_vect)
{
	OCR2A = motor3;
	OCR2B = motor4;
}
