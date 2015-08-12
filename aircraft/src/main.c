#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

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
#define T1PSK 64//set this the same as the prescalar below

int enabled = 0;

int throttle = 62;

uint8_t motor1 = 62;
uint8_t motor2 = 62;
uint8_t motor3 = 62;
uint8_t motor4 = 62;

typedef struct {
	double kP;
	double kI;
	double kD;
	double iMax;
	double iMin;
	double max;
	double min;
} gains_t;

gains_t pitchrollgains = {
	5.0, 0.0, 0.1,
	10.0, -10.0,
	20.0, -20.0
};

gains_t yawgains = {
	5.0, 0.0, 0.05,
	20.0, -20.0,
	20.0, -20.0
};

/* target orientation */
double qt0 = 1.0;
double qt1 = 0.0;
double qt2 = 0.0;
double qt3 = 0.0;

/* target rates */
double gxt = 0.0;
double gyt = 0.0;
double gzt = 0.0;

/**
 * These are the values read from the IMU.
 * The z_ values are zero-state correction values.
 */
double ax, ay, az = 0;
double gx, gy, gz = 0;
double z_gx, z_gy, z_gz = 0;
double temp = 0;
double lastgx = 0.0, lastgy = 0.0, lastgz = 0.0;

/*multiplyer to the accelerometer error part added to the qyro data*/
double accKp = .1;

/*the orientation quaternion*/
double q0 = 1;
double q1 = 0;
double q2 = 0;
double q3 = 0;

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

	const double gyroK = (long double)/* max deg/s */2000 / ((long double)/* 2^15 */32768) * /*Deg to rad*/(3.14159265359/180);
	gx *= gyroK;
	gy *= gyroK;
	gz *= gyroK;

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

	/* Enable pipe 1 */
	NRFStart();
	SPITransmit(0x31);
	SPITransmit(18);
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

void integraterot(double dt)
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
	double gyroK = .25 * (long double)dt * (long double)T1PSK / (long double)F_CPU;
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
	gx_ += accKp * errorx * dt;
	gy_ += accKp * errory * dt;
	gz_ += accKp * errorz * dt;

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

void recievepacket(double dt)
{
	static long double time = 0;

	NRFStart();
	unsigned char status = SPITransmit(0x61);

	if(status & 0x40)
	{
		/* just to generate the clock */
		unsigned char data = SPITransmit(0x00);

		if(data == 0xAA)
		{
			throttle = SPITransmit(0x00);

			char build[4];
			build[0] = SPITransmit(0x00);
			build[1] = SPITransmit(0x00);
			build[2] = SPITransmit(0x00);
			build[3] = SPITransmit(0x00);
			qt0 = *((double *)build);

			build[0] = SPITransmit(0x00);
			build[1] = SPITransmit(0x00);
			build[2] = SPITransmit(0x00);
			build[3] = SPITransmit(0x00);
			qt1 = *((double *)build);

			build[0] = SPITransmit(0x00);
			build[1] = SPITransmit(0x00);
			build[2] = SPITransmit(0x00);
			build[3] = SPITransmit(0x00);
			qt2 = *((double *)build);

			build[0] = SPITransmit(0x00);
			build[1] = SPITransmit(0x00);
			build[2] = SPITransmit(0x00);
			build[3] = SPITransmit(0x00);
			qt3 = *((double *)build);
		}
		NRFStop();

		NRFStart();
		SPITransmit(0x27);
		SPITransmit(0x40);
		NRFStop();
		time = 0;
	} else {
		time += dt;
		if(time > .5)
			throttle = 62;
		NRFStop();
	}
}

void calculaterates()
{
	double qt0_ = qt0 - q0;
	double qt1_ = qt1 - q1;
	double qt2_ = qt2 - q2;
	double qt3_ = qt3 - q3;

	if(qt0*q0 + qt1*q1 + qt2*q2 + qt3*q3 > 0)
	{
		gxt = q0*qt1_ - q1*qt0_ + q2*qt3_ - q3*qt2_;
		gyt = q0*qt2_ - q1*qt3_ - q2*qt0_ + q3*qt1_;
		gzt = q0*qt3_ + q1*qt2_ - q2*qt1_ - q3*qt0_;
	} else {
		gxt = -q0*qt1_ + q1*qt0_ - q2*qt3_ + q3*qt2_;
		gyt = -q0*qt2_ + q1*qt3_ + q2*qt0_ - q3*qt1_;
		gzt = -q0*qt3_ - q1*qt2_ + q2*qt1_ + q3*qt0_;
	}

	gxt *= 5.0;
	gyt *= 5.0;
	gzt *= 5.0;
}

void calculatemotors(double dt)
{
			double commandpitch;
			double commandroll;
			double commandyaw;

			double dInput;
			double error;

			static double iTermroll = 0;
			static double iTermpitch = 0;
			static double iTermyaw = 0;

			static double lastinputroll = 0;
			static double lastinputpitch = 0;
			static double lastinputyaw = 0;

			error = gxt - gx;
			dInput = gx - lastinputroll;
			iTermroll += pitchrollgains.kI * dt * error;
			if(iTermroll > pitchrollgains.iMax)
				iTermroll = pitchrollgains.iMax;
			else if(iTermroll < pitchrollgains.iMin)
				iTermroll = pitchrollgains.iMin;
			commandroll = pitchrollgains.kP * error + iTermroll - pitchrollgains.kD * dInput / dt;
			if(commandroll > pitchrollgains.max)
				commandroll = pitchrollgains.max;
			else if(commandroll < pitchrollgains.min)
				commandroll = pitchrollgains.min;
			lastinputroll = gx;

			error = gyt - gy;
			dInput = gy - lastinputpitch;
			iTermpitch += pitchrollgains.kI * dt * error;
			if(iTermpitch > pitchrollgains.iMax)
				iTermpitch = pitchrollgains.iMax;
			else if(iTermpitch < pitchrollgains.iMin)
				iTermpitch = pitchrollgains.iMin;
			commandpitch = pitchrollgains.kP * error + iTermpitch - pitchrollgains.kD * dInput / dt;
			if(commandpitch > pitchrollgains.max)
				commandpitch = pitchrollgains.max;
			else if(commandpitch < pitchrollgains.min)
				commandpitch = pitchrollgains.min;
			lastinputpitch = gy;

			error = gzt - gz;
			dInput = gz - lastinputyaw;
			iTermyaw += yawgains.kI * dt * error;
			if(iTermyaw > yawgains.iMax)
				iTermyaw = yawgains.iMax;
			else if(iTermyaw < yawgains.iMin)
				iTermyaw = yawgains.iMin;
			commandyaw = yawgains.kP * error + iTermyaw - yawgains.kD * dInput / dt;
			if(commandyaw > yawgains.max)
				commandyaw = yawgains.max;
			else if(commandyaw < yawgains.min)
				commandyaw = yawgains.min;
			lastinputyaw = gz;

			uint8_t motor1_ = (int)(throttle + commandroll - commandpitch + commandyaw);
			uint8_t motor2_ = (int)(throttle - commandroll - commandpitch - commandyaw);
			uint8_t motor3_ = (int)(throttle - commandroll + commandpitch + commandyaw);
			uint8_t motor4_ = (int)(throttle + commandroll + commandpitch - commandyaw);

			if(motor1_ > 125)
				motor1 = 125;
			else if(motor1_ < 62)
				motor1 = 62;
			else
				motor1 = motor1_;
			if(motor2_ > 125)
				motor2 = 125;
			else if(motor2_ < 62)
				motor2 = 62;
			else
				motor2 = motor2_;
			if(motor3_ > 125)
				motor3 = 125;
			else if(motor3_ < 62)
				motor3 = 62;
			else
				motor3 = motor3_;
			if(motor4_ > 125)
				motor4 = 125;
			else if(motor4_ < 62)
				motor4 = 62;
			else
				motor4 = motor4_;
}

int main(void)
{
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

	/**
	 * Init spi
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

	mpu6050Init();
	NRFInit();

	_delay_ms(2600);

	motor1 = 62;
	motor2 = 62;
	motor3 = 62;
	motor4 = 62;

	msg("starting\n");

	mpu6050Calibrate();
	msg("calibrated\n");

	uint16_t lastclock = TCNT1;//16-bit read handled by compiler
	while(1)
	{
		uint16_t clock;
		double dt;

		clock = TCNT1;

		/**
		 * Read sensors into memory here for the most accurate deltaT/sensor-data combination.
		 */
		mpu6050UpdateAll();

		/**
		 * Calculate deltaT here,
		 * and reset it for next loop.
		 *
		 * deltaT is calculated based on the 16-bit timer0, which effectly counts clock cycles.
		 */
		dt = (double)(clock - lastclock) * (double)T1PSK / (double)F_CPU;
		lastclock = clock;\

		/******************************************************************
		 * Start calculations
		 */

		integraterot(dt);
		recievepacket(dt);

		if(throttle > 62)
		{
			calculaterates();
			calculatemotors(dt);

		} else {
			motor1 = 62;
			motor2 = 62;
			motor3 = 62;
			motor4 = 62;
		}

		if(p++ == 200)
		{
			p = 0;
			char string[64];
			sprintf(string, "1: %f\t2: %f\t3: %f\t4: %f\t%i\n", gxt, gyt, gzt, 0.0, throttle);
			msg(string);
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
