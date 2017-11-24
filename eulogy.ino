#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define INTERRUPT_PIN 2 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


#include "pitches.h"

/******** Load AVR timer interrupt macros ********/
#include <avr/interrupt.h>

/******** Sine wave parameters ********/
#define PI2     6.283185 // 2 * PI - saves calculating it later
#define AMP     127      // Multiplication factor for the sine wave
#define OFFSET  128      // Offset shifts wave to just positive values

/******** Lookup table ********/
#define LENGTH  256  // The length of the waveform lookup table
byte wave[LENGTH];   // Storage for the waveform

/* Note stuff */
int gDecayTime = 200;
float gVolume = 1.0;


MPU6050 mpu;
int gOffsetX, gOffsetY, gOffsetZ;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
    
  pinMode(INTERRUPT_PIN, INPUT);
  
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  devStatus = mpu.dmpInitialize();

      if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
}

  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 100; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    sumX += gx;
    sumY += gy;
    sumZ += gz;
  }
  Serial.print(sumX / 100); Serial.print("\t");
  Serial.print(sumY / 100); Serial.print("\t");
  Serial.println(sumZ / 100);
  gOffsetX = sumX / 100;
  gOffsetY = sumY / 100;
  gOffsetZ = sumZ / 100;
  
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

#define OUTPUT_READABLE_ACCELGYRO
float ypr[3];
Quaternion q;
VectorFloat gravity; 
void loop()
{
  while (!mpuInterrupt && fifoCount < packetSize) {
  
  const int notes[] = {nC4, nD4, nE4, nF4, nG4, nA4, nB4, nC5};
  for (int i = 0; i < 8; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    
    // display tab-separated accel x/y/z values
    #ifdef OUTPUT_READABLE_ACCELGYRO
      // display tab-separated accel/gyro x/y/z values
      Serial.print((long)gx - (long)gOffsetX); Serial.print("\t");
      Serial.print((long)gy - (long)gOffsetY); Serial.print("\t");
      Serial.println((long)gz - (long)gOffsetZ);
    #endif
    delay(500);
//    Serial.println(notes[i]);
//    Tone(notes[i]);
//    delay(1000);
//    Decay(500);
//    NoTone();
//    delay(200);

    //Serial.println("Decay time!");
    //    Decay();
  }
  
  // other program behavior stuff here
  // .
  // if you are really paranoid you can frequently test in between other
  // stuff to see if mpuInterrupt is true, and if so, "break;" from the
  // while() loop to immediately process the MPU data
  // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
Serial.println(ypr[2] * 180/M_PI);

  
}
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

void DecayUpdate() {
  static int timeDecayStarted = 0;
  int timeSinceDecayStarted = millis() - timeDecayStarted;

  int dx = gDecayTime - timeSinceDecayStarted;
  if (dx < 0) {
    timeDecayStarted = millis();
    dx = 0;
  }

  gVolume = (float)dx / (float)gDecayTime;
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






