#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INTERRUPT_PIN 19

// MPU control/status vars
uint8_t  mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64]; // FIFO storage buffer
/* MPU related variables */
MPU6050 mpu;
float ypr[3];


#define LCD_CS    A3 // Chip Select goes to Analog 3
#define LCD_CD    A2 // Command/Data goes to Analog 2
#define LCD_WR    A1 // LCD Write goes to Analog 1
#define LCD_RD    A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include <Adafruit_TFTLCD.h>

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif


#include "TouchScreen.h"

uint8_t YP = A1;  // must be an analog pin, use "An" notation!
uint8_t XM = A2;  // must be an analog pin, use "An" notation!
uint8_t YM = 7;   // can be a digital pin
uint8_t XP = 6;   // can be a digital pin

uint16_t TS_LEFT = 160;
uint16_t TS_RT  = 925;
uint16_t TS_TOP = 160;
uint16_t TS_BOT = 955;


// YMax = 955
// YMin = 160
// XMax = 925
// XMin = 160

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);



#include "pitches.h"

/* Sine wave parameters */
#define PI2     6.283185 // 2 * PI - saves calculating it later
#define AMP     120      // Multiplication factor for the sine wave
#define OFFSET  128      // Offset shifts wave to just positive values

/* Waveform lookup table */
#define LENGTH  256  // The length of the waveform lookup table
byte wave[LENGTH];


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
int dataReadyCalled = 0;
void dmpDataReady() {
	mpuInterrupt = true;
  dataReadyCalled += 1;
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
//  for (int i = 0; i < LENGTH; i++) {
//    if (i < LENGTH / 2)
//      wave[i] = tft.height();
//    else 
//      wave[i] = 0;
//  }
}

void setup() {
	Serial.begin(9600);
  Serial2.begin(9600);

	SetupMPU();
  
  tft.reset();

  Serial.print("TFT size is ");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());
  
  uint16_t id = tft.readID();
  Serial.print("ID = 0x");
  Serial.println(id, HEX);
  id = 0x4532;
  tft.begin(id);
    
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setTextColor(RED);
  tft.setCursor(0, 0);
  tft.print(".-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.\n");
  tft.setCursor(0, 0);


  SetupSynth();
  
  for (int i = 0; i < LENGTH; i++) {
//    tft.drawPixel(i, tft.height() / 2 + wave[i] - 127, MAGENTA);  
    tft.drawFastVLine(i, tft.height() / 2 + wave[i] - 127, 8, RED);
  }
}

void loop()
{

  TSPoint tp = ts.getPoint();
  
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  pinMode(XP, OUTPUT);
  pinMode(YM, OUTPUT);
  
  int ypos = map(tp.x, TS_LEFT, TS_RT, 0, tft.height());
  int xpos = map(tp.y, TS_TOP, TS_BOT, 0, tft.width() + 20);
  
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if (tp.z > 50) {
    if (xpos > tft.width()) {
      
    }
    else if (xpos >= LENGTH && xpos <= tft.width()) {
    }
    else if (xpos > 0 && xpos < LENGTH) {
      Serial.print("X = ");   Serial.print(xpos);
      Serial.print("\tY = "); Serial.println(ypos);

      tft.drawFastVLine(xpos, wave[xpos] - 9, 20, BLACK);
      int h = tft.height() - ypos;
      tft.drawFastVLine(xpos, h, 9, RED); 
      wave[xpos] = h;
      
      byte w[2] = {xpos, h};
      Serial2.write(w, 2);
      
    }
  }

  
  
  float angle = ypr[2] * 180 / M_PI;
	//Serial.println(angle);
  
  
  ////////////////////////////////////////////////////////

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();
  dataReadyCalled = 0;

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
//		Serial.println(ypr[2] * 180 / M_PI);
	}
}


