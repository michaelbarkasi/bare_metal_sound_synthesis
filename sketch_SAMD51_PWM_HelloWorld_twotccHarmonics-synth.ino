/*
 * The code for the base (240kHz) wave form (from TCC1) is taken largely from a post by MartinL (Jan 2019) on the Arduino forums; 
 *    See: https://forum.arduino.cc/t/metro-m4-express-atsamd51-pwm-frequency-and-resolution/566491/2
 *  It was originally written written for the Metro M4, to run at some other frequency 
 *    (I changed the timer speeds, prescalers, and PER/CC0 values to adjust frequency to 240kHz).
 *  I modified the code to work with the SparkFun Thing Plus (SAMD51) by looking up the necessary registery values in the SAMD51 datasheet (see page numbers below).
 *  It's written to work with pin D11 of the SparkFun Thing Plus, which is pin PA16 on the SAMD51; which (I believe) must be used with TCC1/WO[0]
 *    (This latter part is also a modification from MartinL, who used TCC0.)
 *  Finally, to get the base wave form to actually generate a signal on D11, 
 *    I needed two more lines of code, to set DIRSET and OUTCLR, which I got from Shawn Hymel's write-up (Dec 22, 2018);
 *    See: https://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
 *  Shawn notes that his write-up is also based on posts by MartinL.
 *  
 *  The code for the timer controlling the hearable oscilation (TCC2) is inspired by a mixature of the above sources and from: 
 *    https://emalliab.wordpress.com/2021/04/16/comparing-timers-on-samd21-and-samd51-microcontrollers/
 */ 
 
/* How the sound is generated: Two timers are used. The first (TCC1) controls a base wave form of digital pulses with 240 kHz oscilation. 
    Since we're using two TCC timers, we can do PWM (pulse-width modulation). 240 kHz is too fast for a speaker, which (acting as a low-pass filter)
    only "sees" the average voltage, which is digital HIGH * duty cycle. (Duty cycle is pulse width, e.g. pulse on for 70% of the period and off 
    for 30%.) The second timer (TCC2) oscilates pulse width (duty cycle) of this wave at a hearable frequency, flipping between 0 and some second 
    nonzero value. This second nonzero value essentially ends up controlling volume, since it covaries with the voltage "seen" on each pulse by the speaker.
    Since we're using a TCC timer (not a TC), we have more control than just a "top" or overflow value; For a TC timer, we'd be limited to having the pulse width
    of the base wave (from TCC1) flip on every overflow, e.g. flipping between 0 and some non-zero volume level. The duty cycle, though, of this hearable oscilation 
    must remain a fixed 50/50 with the TC method. By switching to a TCC timer, we can have the timer start/overflow initiate a non-zero value for the TCC1 duty cycle (volume),
    then have the timer's top value initiate a zero value, allowing us to control the duty cycle of the hearable oscilation. 
    
    By running the second timer (TCC2) at twice the intended frequency, we can run separate volume levels for every other wave form, allowing for under tones. 
    For example, running at 880Hz, every other wave form could be a 80% duty cycle for TCC1, while the rest are at 30%, giving a main 440Hz sound with undertones at 880Hz.
    
    If you don't want the undertones (harmonics) but want control of the sound's duty cycle, eliminate the harmonics variable and run all the TCC2 overflows the same.*/

/* This version of the code includes some extra bits and timers just meant to demonstrate the range of sounds that can be produced. */ 

volatile int VOL = 499; /* As just explained, this value controls volume. 
                            It should range between 0 and 499.
                            499 is the TOP value of the counter controlling the base 240 kHz wave form;
                            duty cycle ends up being VOL / 499. */
volatile float harmonic_ratio = 0.15;
volatile int VOL_harmonic = (int)( VOL * harmonic_ratio );
volatile int TF = 440;                                  /* Tone frequency; Mathematically, anything from 183 to 6 million are valid 
                                                            given the code/math/architecture. Values below 183 will overflow the 16-bit register, 
                                                            and values over 6 million end up in nonsense like dividing by zero or negative numbers.
                                                            Of course, since a good audible range is something like 200 to 2000, this is fine. 
                                                            Why initialize at 440? It's sort of pleasing. */
volatile long TCTOPL = 12000000L / (2 * TF) - 1;        /* Variable stores CC0 (TOP) value of counter TC0, which is the counter ultimately controlling the tone pitch. 
                                                            Every time this timer overflows, pulse width of the base waveform is flipped between 0 and some nonzero value,
                                                            hence this counter must overflow twice for one period of the tone frequency. Frequency (pitch) of the tone is 
                                                            thus equal to oscilator speed (cycles per sec) divided by the number of times the counter overflows twice. Since
                                                            the oscilator used for TC0 runs at 12 MHz, this means that TF = 12000000 / (2 * (TCTOP - 1)). We have to subtract 1 
                                                            because the counter starts at 0, not 1. A simple rearrangement of this equation gets us the formula 
                                                            for TCTOP. 
                                                            Note: We need this extra step because 12000000 is longer than 16 bit; 
                                                            a 32 kHz oscilator would save us this step, but would offer poor control/resolution of tone pitch. */
volatile int TCTOP = (uint16_t)TCTOPL;                  /* Finally, convert into int16 for register */

volatile float pulsefraction = 0.6;

bool harmonic = true;

int readHarmonicRatio = 1;
int readVOL = 1;
int readPulseFraction = 1;
int readTF = 1;

bool LFO = true;
float LFOvalue = 1.0;
bool ULFO = true;
float ULFOvalue = 1.0;

const int noisefilter = 20;

long time1 = 1;
long time2 = 1;
long time3 = 1;
long time4 = 1;
long time5 = 1;
long time6 = 1;

int count = 0;
int count2 = 0;
int count3 = 0;

void setup() 
{
  Serial.begin(115200);
  // Setup TCC1, which will generate the base pulse wave
  // Set up the generic clock (GCLK7) to clock timer TCC1 
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120 MHz clock source by divisor 1: 120 MHz/1 = 120 MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization  

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC1 peripheral channel, see p. 169; 25 = GCLK_TCC1
                          GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC1

  // below: Port (ulPort) is PORTA; pin (ulPin) is PORT_PA16 (= D11)
  PORT->Group[g_APinDescription[11].ulPort].DIRSET.reg = g_APinDescription[11].ulPin; // Set pin as output
  PORT->Group[g_APinDescription[11].ulPort].OUTCLR.reg = g_APinDescription[11].ulPin; // Set pin as low

  // Enable the peripheral multiplexer on pin D11
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
  
  // Set the D11 (PORT_PA16) peripheral multiplexer to the correct peripheral (even port number)
  PORT->Group[g_APinDescription[11].ulPort].PMUX[8].reg |= 0x5; // see p. 900, 923
  
  TCC1->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 1, 120 MHz/1 = 120 MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC1->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC1 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC1->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

  TCC1->PER.reg = 499;                        // Set-up the PER (period) register; this value is chosen to produce a pulse at 240 kHz (see formula below)
  while (TCC1->SYNCBUSY.bit.PER);                    // Wait for synchronization
                                                     // Formula: f_PWM = f_GCLKTCC / (PRESCALER (PER + 1))
                                                     // In this case, f_GCLKTCC = 120 MHz, PRESCALER = 1; 120 x 10^6 / 500 = 240 x 10^3
  
  TCC1->CC[0].reg = VOL;                    // Set-up the CC (counter compare), channel 0 register; Recall, this controls volume. 
  while (TCC1->SYNCBUSY.bit.CC0);                    // Wait for synchronization

  TCC1->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC1
  while (TCC1->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization

  // Setup TCC2, which will generate a hearable oscilation off the base pulse wave: 
  // Set up a generic clock (GCLK8) to clock timer TCC2 
  GCLK->GENCTRL[8].reg = GCLK_GENCTRL_DIV(4) |       // Divide the 48MHz clock source by divisor 4: 48MHz/1 = 12MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK8
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL8);               // Wait for synchronization  

  GCLK->PCHCTRL[29].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC2 peripheral channel, see p. 169; 9 = GCLK_TCC2
                          GCLK_PCHCTRL_GEN_GCLK8;    // Connect generic clock 8 to TCC2

  TCC2->CTRLA.bit.ENABLE = 0;            // Disable TCC2
  while (TCC2->SYNCBUSY.bit.ENABLE);     // Wait for sync
  TCC2->CTRLA.reg = TC_CTRLA_SWRST;      // Reset TCC2
  while (TCC2->SYNCBUSY.bit.SWRST);      // Wait for sync

  TCC2->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;   // Set TCC2 wave generation mode to Match Frequency Generation
  TCC2->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE; // No prescaler (Divide by 1)
  TCC2->PER.reg = TCTOP;
  TCC2->CC[0].reg = (int) TCTOP * pulsefraction;                 // Write the counter value controlling wave form frequency
  while (TCC2->SYNCBUSY.reg > 0);          // Wait for sync

  // Setup the interrupts used to adjust pulse width of the base 240 kHz wave form
  // Basic idea: MC0 is triggered on a CC0 match, OVF when the counter hits top. See pp. 1766, 1976, 1984
  NVIC_DisableIRQ(TCC2_1_IRQn); // TCC2_1_IRQn is 98, the number for the MC0 channel (see p. 73)
  NVIC_ClearPendingIRQ(TCC2_1_IRQn);
  NVIC_SetPriority(TCC2_1_IRQn, 0);
  NVIC_EnableIRQ(TCC2_1_IRQn);

  // Set interrupt register
  TCC2->INTENSET.bit.MC0 = 1;
  while (TCC2->SYNCBUSY.reg > 0);

  // Setup the interrupts used to adjust pulse width of the base 240 kHz wave form
  NVIC_DisableIRQ(TCC2_0_IRQn); // TCC2_0_IRQn is 97, the number for the overflow (OVF) channel (see p. 73)
  NVIC_ClearPendingIRQ(TCC2_0_IRQn);
  NVIC_SetPriority(TCC2_0_IRQn, 0);
  NVIC_EnableIRQ(TCC2_0_IRQn);

  // Set interrupt register
  TCC2->INTENSET.bit.OVF = 1;
  while (TCC2->SYNCBUSY.reg > 0);

  pinMode(A3,INPUT);
  pinMode(A0,INPUT);
  pinMode(A2,INPUT);
  pinMode(A4,INPUT);
  time1 = micros();
  time4 = millis();
}

void loop() {
    for ( int i = 0 ; i < 1000 ; i++ ){ 
  time2 = micros();
  time3 = time2 - time1;
  int LFOtime = (int)( 600000 * (float)(readPulseFraction / 1023.0) );
  if (time3 >= LFOtime){
    LFO = !LFO;
    time1 = time2;
//    count += 1;
//    if (count == 3) count = 0;
    count2 += 1;
    if (count2 == 5) count2 = 0;
//    count3 += 1;
//    if (count3 == 3) count3 = 0;
  }
//  time5 = millis();
//  time6 = time5 - time4;
//  if (time6 >= 2000){
//    ULFO = !ULFO;
//    time4 = time5;
//    count3 += 1;
//    if (count3 == 3) count3 = 0;
//  }
  if (LFO) LFOvalue = 1.0; else LFOvalue = 0.5;
//  if (ULFO) ULFOvalue = 1.0; else ULFOvalue = 0.5;

  int readHarmonicRation = analogRead(A3);
  int readVOLn = analogRead(A4);
  int readPulseFractionn = analogRead(A2);
  int readTFn = analogRead(A0);
  if (abs(readHarmonicRation - readHarmonicRatio) > noisefilter) readHarmonicRatio = readHarmonicRation;
  if (abs(readVOLn - readVOL) > noisefilter) readVOL = readVOLn;
  if (abs(readPulseFractionn - readPulseFraction) > noisefilter) readPulseFraction = readPulseFractionn;
  if (abs(readTFn - readTF) > noisefilter) readTF = readTFn;
  
  harmonic_ratio = (readHarmonicRatio/1023.0) * (count2 / 4.0);
  VOL = (int)((float)(readVOL/1023.0) * (499.0 - (i/2 - 1)));
  VOL_harmonic = (int)( VOL * (float)harmonic_ratio );
  TF = readTF + (120 * count) + (i * 6);
  pulsefraction = (readPulseFraction/1023.0) * LFOvalue;
  UpdateTF(TF); // also updates pulse fraction
  delay(2);
  Serial.print("Harmonics: ");
  Serial.print(harmonic_ratio,3);
  Serial.print(", Volume: ");
  Serial.print(VOL,1);
  Serial.print(", Wave Width: ");
  Serial.print(pulsefraction,3);
  Serial.print(", Pitch (Hz): ");
  Serial.println(TF,1);
  }
}

// This handler is what actually uses the second timer (TCC2) to generate the hearable oscilation of the pulse wave generated by the first timer (TCC1).
void TCC2_0_Handler(void) {
  if (TCC2->INTFLAG.bit.OVF == 1) //Test if an OVF-Interrupt has occured
  {
    TCC2->INTFLAG.bit.OVF = 1;  //Clear the Interrupt-Flag
    if (harmonic) {
      TCC1->CC[0].reg = VOL_harmonic;
      while (TCC1->SYNCBUSY.bit.CC0); 
      harmonic = !harmonic;
    }
    else {
      TCC1->CC[0].reg = VOL;
      while (TCC1->SYNCBUSY.bit.CC0); 
      harmonic = !harmonic;
    }
  }
}

void TCC2_1_Handler(void) {
  if (TCC2->INTFLAG.bit.MC0 == 1)  //Test if an MC0-Interrupt has occured
  {
    TCC2->INTFLAG.bit.MC0 = 1;  //Clear the Interrupt-Flag
    TCC1->CC[0].reg = 0;
   while (TCC1->SYNCBUSY.bit.CC0);
  }
}

void UpdateTF (uint16_t newTF) {
  TCTOPL = 12000000L / newTF - 1;  // Do the needed math to get new TOP value for TC0
  TCTOP = (uint16_t)TCTOPL; // Just double check and make sure we're passing the right data type to the register
    // Note that TF (or "newTF") really is the hearable frequency, e.g. if it's 440, it will be a 440 Hz tone.
  // Pitch needs to be updated now: 
  TCC2->PER.reg = TCTOP;                             // Set-up the PER (period) register, controlling wave form frequency (here, controls frequency of hearable oscilation);
  //while (TCC2->SYNCBUSY.bit.PER);                    // Wait for synchronization
  TCC2->CC[0].reg = (int)( TCTOP * pulsefraction );                 // Set-up the CC (counter compare), channel 0 register; this controls duty cycle (wave width) of the hearable frequency. 
  //while (TCC2->SYNCBUSY.bit.CC0);                    // Wait for synchronization
  while (TCC2->SYNCBUSY.reg > 0);          // Wait for sync
}
