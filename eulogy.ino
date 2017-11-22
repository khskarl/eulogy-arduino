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

void setup() {
  Serial.begin(9600);
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

void loop()
{
  int notes[] = {nC4, nD4, nE4, nF4, nG4, nA4, nB4, nC5};
  for (int i = 0; i < 8; i++) {
    Serial.println(notes[i]);
    Tone(notes[i]);
    delay(1000);
    Decay(500);
    NoTone();
    delay(200);

    //Serial.println("Decay time!");
    //    Decay();
  }
}

void Tone (int frequency) {
  gVolume = 1.0;
  OCR2A = 2000000 / ((frequency + 0.001) * LENGTH);
}

void NoTone () {
  gVolume = 0.0f;
}

r
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






