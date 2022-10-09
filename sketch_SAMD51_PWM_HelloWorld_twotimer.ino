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
 *  The code for the timer controlling the hearable oscilation (TC0) is adapted from: 
 *    https://emalliab.wordpress.com/2021/04/16/comparing-timers-on-samd21-and-samd51-microcontrollers/
 *    I've changed it a bit, including switching from the slow 32 kHz oscilator for a fast clock, to give better pitch control/resolution. 
 */ 
 
/* How the sound is generated: Two timers are used. The first (TCC1) controls a base wave form of digital pulses with 240 kHz oscilation. 
    Since we're using a TCC and TC, we can do PWM (pulse-width modulation). 240 kHz is too fast for a speaker, which (acting as a low-pass filter)
    only "sees" the average voltage, which is digital HIGH * duty cycle. (Duty cycle is pulse width, e.g. pulse on for 70% of the period and off 
    for 30%.) The second timer (TC0) oscilates pulse width (duty cycle) of this wave at a hearable frequency, flipping between 0 and some second 
    nonzero value. This second nonzero value essentially ends up controlling volume, since it covaries with the voltage "seen" on each pulse by the speaker.*/

volatile int VOL = 499; /* As just explained, this value controls volume. 
                            It should range between 0 and 499.
                            499 is the TOP value of the counter controlling the base 240 kHz wave form;
                            duty cycle ends up being VOL / 499. */
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

void setup() 
{
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

  // Setup TC0, which will generate a hearable oscilation off the base pulse wave: 
  // Set up a generic clock (GCLK8) to clock timer TC0 
  GCLK->GENCTRL[8].reg = GCLK_GENCTRL_DIV(4) |       // Divide the 48MHz clock source by divisor 4: 48MHz/1 = 12MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK8
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL8);               // Wait for synchronization  

  GCLK->PCHCTRL[9].reg = GCLK_PCHCTRL_CHEN |        // Enable the TC0 peripheral channel, see p. 169; 9 = GCLK_TC0
                          GCLK_PCHCTRL_GEN_GCLK8;    // Connect generic clock 8 to TC0

  TC0->COUNT16.CTRLA.bit.ENABLE = 0;            // Disable TC0
  while (TC0->COUNT16.SYNCBUSY.bit.ENABLE);     // Wait for sync
  TC0->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;      // Reset TC0
  while (TC0->COUNT16.SYNCBUSY.bit.SWRST);      // Wait for sync

  TC0->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16; // Set TC0 for 16 bit mode? (Isn't that default?)
  TC0->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;   // Set TC0 wave generation mode to Match Frequency Generation
  TC0->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE; // No prescaler (Divide by 1)
  TC0->COUNT16.CC[0].reg = TCTOP;                 // Write the counter value controlling wave form frequency
  while (TC0->COUNT16.SYNCBUSY.reg > 0);          // Wait for sync

  // Setup the interrupts used to adjust pulse width of the base 240 kHz wave form
  NVIC_DisableIRQ(TC0_IRQn);
  NVIC_ClearPendingIRQ(TC0_IRQn);
  NVIC_SetPriority(TC0_IRQn, 0);
  NVIC_EnableIRQ(TC0_IRQn);

  // Set interrupt register
  TC0->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC0->COUNT16.SYNCBUSY.reg > 0);
  
}

void loop() {}

// This handler is what actually uses the second timer (TC0) to generate the hearable oscilation off the pulse wave generated by the first timer (TCC1).
void TC0_Handler(void) {
  // If this interrupt is due to the compare register matching the timer count
  if (TC0->COUNT16.INTFLAG.bit.MC0 == 1) { // p. 64
    TC0->COUNT16.INTFLAG.bit.MC0 = 1;

    if ( TCC1->CC[0].reg == 0 ) {
      TCC1->CC[0].reg = VOL;
      while (TCC1->SYNCBUSY.bit.CC0);
    }
    else {
      TCC1->CC[0].reg = 0;
      while (TCC1->SYNCBUSY.bit.CC0);
    }
    
  }
}
