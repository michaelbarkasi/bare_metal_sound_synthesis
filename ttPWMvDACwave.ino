/*
  Setup base wave form @ 152,380 Hz using the ESP32's hardware PWM
      152,380 Hz is the fastest oscilation its hardware PWM can achieve with at least 9 bit of resolution
        Get this with: clk_src = LEDC_APB_CLK (80 MHz).
        ... then duty resolution is integer (log 2 (LEDC_APB_CLK / frequency))
      see: 
        https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
        https://github.com/espressif/esp-idf/blob/5893797bf068ba6f72105fff289ead370b4591a3/examples/peripherals/ledc/ledc_basic/main/ledc_basic_example_main.c
  Initial code to get PWM working from: https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  Note: We really want this PWM to run in LEDC_HIGH_SPEED_MODE; the code from the esp-idf gives this control, but not the code for arduino;
    Currently; assuming arduino puts PWM into high-speed mode by default
*/

#define DACwave // uncomment to use DAC to generate base pulse wave instead of ttPWM

const int update_rate = 1000; // Controls both sound updates and when to process sensor data/ update motion models

// Variables controlling base pulse wave production / volume control
#ifdef DACwave
const int soundPin = A1;  // DAC is set to use DAC1, which is A1; DAC2 is A0
const int amp_max = 255; // DAC is 8-bit, so only 256 levels
#else 
const int soundPin = 13; 
const int base_wave_freq = 152380; // highest availabe with 9 bit resolution is 152380
const int pwm_Channel = 0;
const int pwm_resolution = (int) log2( 80000000 / base_wave_freq ) ;
const int amp_max = pow(2,pwm_resolution) - 1; // for 9 bit resolution: 2^9 - 1 = 511; for 8 bit 2^8 - 1 = 255
#endif
volatile int amp = (int) amp_max * 0.5;
// setting the timer for generating hearable oscilation 
const int pitch_initial = 440;
volatile int pitch = pitch_initial;
hw_timer_t * timerSW = NULL;
portMUX_TYPE timerSWMux = portMUX_INITIALIZER_UNLOCKED;
const int timerSWprescaler = 2; // Timers are 80Mhz; counters seem to be at least 32bit, so, don't need to prescale down low
const long timerSize = 80000000;
const int timerSizeprescaler = timerSize/timerSWprescaler;
volatile int timerSW_top = (int) timerSizeprescaler / (pitch * 2); // controls frequency of hearable oscilation
volatile bool up = true;

void update_pitch ( int new_pitch ) {
  timerSW_top = (int) timerSizeprescaler / (new_pitch * 2);
  timerAlarmWrite(timerSW, timerSW_top, true); // true = reload automatically
}

// interrupt handlers for timers generating hearable oscilation: 
void IRAM_ATTR onTimerSW() {
  portENTER_CRITICAL_ISR(&timerSWMux);
  #ifdef DACwave
    if (up) {
      dacWrite(DAC1, amp);
    } else {
      dacWrite(DAC1, 0);
    }
  #else 
    if (up) {
      ledcWrite(pwm_Channel, amp);
    } else {
      ledcWrite(pwm_Channel, 0);
    }
  #endif
  up = !up;
  portEXIT_CRITICAL_ISR(&timerSWMux);
}

// Variables for the timer controlling sound updates / motion processing
hw_timer_t * timerSU = NULL;
portMUX_TYPE timerSUMux = portMUX_INITIALIZER_UNLOCKED;
const int timerSU_top = (int) timerSizeprescaler / update_rate; 
volatile bool update_now = false;

// interrupt handler for timer generating sound updates / motion processing: 
void IRAM_ATTR onTimerSU() {
  portENTER_CRITICAL_ISR(&timerSWMux);
  update_now = true;
  portEXIT_CRITICAL_ISR(&timerSWMux);
}
 
void setup(){

  // Setup base pulse wav
  #ifdef DACwave
    // print notice that DAC is used; nothing else to run
  #else
    // print notice that ttPWM is beeing used
    ledcSetup(pwm_Channel, base_wave_freq, pwm_resolution); // configure PWM functionalitites
    ledcAttachPin(soundPin, pwm_Channel); // attach the channel to the pin generating the wave
  #endif
  // Setup timer used for hearable oscilation
  timerSW = timerBegin(0, timerSWprescaler, true); // true indicates counter goes up
  timerAttachInterrupt(timerSW, &onTimerSW, true); // true indicates edge interrupt (false for level)
  timerAlarmWrite(timerSW, timerSW_top, true); // true for reload automatically
  timerAlarmEnable(timerSW);
  // Setup timer for sound updates/ motion processing
  timerSU = timerBegin(1, timerSWprescaler, true); // true indicates counter goes up
  timerAttachInterrupt(timerSU, &onTimerSU, true); // true indicates edge interrupt (false for level)
  timerAlarmWrite(timerSU, timerSU_top, true); // true for reload automatically
  timerAlarmEnable(timerSU);
rdui
  amp = amp_max;
}
 
void loop(){  

  if (update_now) {
    if ( pitch < 2000 ) pitch += 1;
    else pitch = pitch_initial;
    update_pitch(pitch);

    if ( amp < 5 ) amp = amp_max;
    else amp -= 1;

    update_now = false;
  }
  
}