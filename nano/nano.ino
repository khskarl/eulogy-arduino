#include <avr/interrupt.h>

#include "pitches.h"

/* Sine wave parameters */
#define PI2     6.283185 // 2 * PI - saves calculating it later
#define AMP     127      // Multiplication factor for the sine wave
#define OFFSET  128      // Offset shifts wave to just positive values

/* Waveform lookup table */
#define LENGTH  256  // The length of the waveform lookup table
byte wave[LENGTH];

/* Note stuff */
float gVolume = 1.0;

void SetupSynth () {
	/******** Populate the waveform lookup table with a sine wave ********/
	for (int i = 0; i < LENGTH; i++) {
		float v = (AMP * sin((PI2 / LENGTH) * i)); // Calculate current entry
		wave[i] = int(v + OFFSET);            // Store value as integer
	}
  
  Serial.write(0xFF); 
  if (Serial.available() > 0) {
    byte ack;
    Serial.readBytes(wave, LENGTH);
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

	SetupSynth();

  NoTone();
}

int gMelody[] = {
  G3, AS3, D4, G4, G4, F4, D4,
  G3, AS3, D4, G4, G4, F4, D4,
  G3, AS3, D4, G4, G4, F4, D4,
  G3, AS3, D4, G4, G4, F4, D4,

  G3, A3, AS3, D4,
  0,  D4, E4,  F4, D4, E4, C4, D4,
  A3,

  0,
  G3, A3, AS3, D4,
  0,  D4, E4,  F4, D4, E4, C4, D4,
  A3,

  
};

int gNoteDurations[] = {
  8, 8, 8, 4, 8, 8, 8,
  8, 8, 8, 4, 8, 8, 8,
  8, 8, 8, 4, 8, 8, 8,
  8, 8, 8, 4, 4, 4, 4,

  8, 8, 8, 2, 
  8, 8, 8, 8, 8, 8, 8, 8,
  1,

  1,
  8, 8, 8, 2, 
  8, 8, 8, 8, 8, 8, 8, 8,
  1,
  
};

int gCurrNote = 0;

void loop()
{
//  const int notes[] = { nC4, nD4, nE4, nF4, nG4, nA4, nB4, nC5 };

  int noteDuration = 1000 / gNoteDurations[gCurrNote];
  int pauseBetweenNotes = noteDuration * 1.50;
//  delay(pauseBetweenNotes);  
  int note = gMelody[gCurrNote];
  if (note != 0) {
    Tone(note);
    delay(noteDuration);
    Decay(300, pauseBetweenNotes);
  }
  else { 
    NoTone();
    delay(noteDuration + pauseBetweenNotes);
  }
  delay(100);
  gCurrNote += 1;
  if (gCurrNote == 55) {
    gCurrNote = 0;
    delay(1000);
  }
  
  while (Serial.available() > 0) {
    byte w[2];
    Serial.readBytes(w, 2);
    
    byte b1 = w[0];
    byte b2 = w[1];

    wave[b1] = b2;
  }
}

void Tone (int frequency) {
	gVolume = 1.0;
	OCR2A = 2000000 / ((frequency + 0.1) * LENGTH);
}

void NoTone () {
	gVolume = 0.1f;
}

/******** Called every time TCNT2 = OCR2A ********/
ISR(TIMER2_COMPA_vect) {  // Called each time TCNT2 == OCR2A
	static byte index = 0;  // Points to successive entries in the wavetable
	OCR1AL = (int)(wave[index++] * gVolume); // Update the PWM output
	//  OCR1AL = wave[index++]; // Update the PWM output
	asm("NOP;NOP");         // Fine tuning
	TCNT2 = 6;              // Timing to compensate for time spent in ISR
}

void Decay (int decayTime, int threshould) {
	int timeDecayStarted = millis();
	int timeSinceDecayStarted = millis() - timeDecayStarted;
	int dx = decayTime - timeSinceDecayStarted;

	while (dx > 0 && timeSinceDecayStarted <= threshould) {
		gVolume = (float)dx / (float)decayTime;

		timeSinceDecayStarted = millis() - timeDecayStarted;
		dx = decayTime - timeSinceDecayStarted;
	}
	gVolume = 0;
}
