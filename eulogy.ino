#include <avr/interrupt.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 2

// MPU control/status vars
uint8_t  mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64]; // FIFO storage buffer

#include "pitches.h"

/* Sine wave parameters */
#define PI2     6.283185 // 2 * PI - saves calculating it later
#define AMP     127      // Multiplication factor for the sine wave
#define OFFSET  128      // Offset shifts wave to just positive values

/* Waveform lookup table */
#define LENGTH  256  // The length of the waveform lookup table
byte wave[LENGTH];

/* Note stuff */
int gDecayTime = 200;
float gVolume = 1.0;

/* MPU related variables */
MPU6050 mpu;
float ypr[3];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

void SetupMPU () {
	Wire.begin();
	Wire.setClock(400000);

	mpu.initialize();

	pinMode(INTERRUPT_PIN, INPUT);

	if (mpu.testConnection())
		Serial.println("MPU6050 connection successful");
	else
		Serial.println("MPU6050 connection failed");

	int devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

}

void SetupSynth () {
	/******** Populate the waveform lookup table with a sine wave ********/
	for (int i = 0; i < LENGTH; i++) {
		float v = (AMP * sin((PI2 / LENGTH) * i)); // Calculate current entry
		wave[i] = int(v + OFFSET);            // Store value as integer
	}

	/******** Set timer1 for 8-bit fast PWM output ********/
	pinMode(9, OUTPUT);       // Make timer's PWM pin an output
	TCCR1B  = (1 << CS10);    // Set prescaler to full 16MHz
	TCCR1A |= (1 << COM1A1);  // PWM pin to go low when TCNT1=OCR1A
	TCCR1A |= (1 << WGM10);   // Put timer into 8-bit fast PWM mode
	TCCR1B |= (1 << WGM12);

	/******** Set up timer 2 to call ISR ********/
	TCCR2A = 0;               // We need no options in control register A
	TCCR2B = (1 << CS21);     // Set prescaller to divide by 8
	TIMSK2 = (1 << OCIE2A);   // Set timer to call ISR when TCNT2 = OCRA2
	OCR2A = 32;               // sets the frequency of the generated wave
	sei();                    // Enable interrupts to generate waveform!
}

void setup() {
	Serial.begin(9600);

	SetupMPU();
	SetupSynth();

  NoTone();
}


void loop()
{
	while (!mpuInterrupt && fifoCount < packetSize) {
		const int notes[] = { nC4, nD4, nE4, nF4, nG4, nA4, nB4, nC5 };
		Serial.println("Estou no while!");
		for (int i = 0; i < 1; i++) {
			//Serial.println(notes[i]);
			//Tone(notes[i]);
      Tone((ypr[0] + 180 + nB0) * 30);
			//delay(1000);
			//Decay(500);
			//NoTone();
			//delay(500);

			//Serial.println("Decay time!");
			//Decay();
		}
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;


		Quaternion  quaternion;
		VectorFloat gravity;
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &quaternion);
		mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);

//		Serial.print("ypr\t");
//		Serial.print(ypr[0] * 180 / M_PI);
//		Serial.print("\t");
//		Serial.print(ypr[1] * 180 / M_PI);
//		Serial.print("\t");
		Serial.println(ypr[2] * 180 / M_PI);
	}
 mpuInterrupt = false;
}

void Tone (int frequency) {
	gVolume = 1.0;
	OCR2A = 2000000 / ((frequency + 0.001) * LENGTH);
}

void NoTone () {
	gVolume = 0.0f;
}

/******** Called every time TCNT2 = OCR2A ********/
ISR(TIMER2_COMPA_vect) {  // Called each time TCNT2 == OCR2A
	static byte index = 0;  // Points to successive entries in the wavetable
	OCR1AL = (int)(wave[index++] * gVolume); // Update the PWM output
	//  OCR1AL = wave[index++]; // Update the PWM output
	asm("NOP;NOP");         // Fine tuning
	TCNT2 = 6;              // Timing to compensate for time spent in ISR
}

void Decay (int decayTime) {
	int timeDecayStarted = millis();
	int timeSinceDecayStarted = millis() - timeDecayStarted;
	int dx = decayTime - timeSinceDecayStarted;

	while (dx > 0) {
		gVolume = (float)dx / (float)decayTime;

		timeSinceDecayStarted = millis() - timeDecayStarted;
		dx = decayTime - timeSinceDecayStarted;
	}
	gVolume = 0;
}
