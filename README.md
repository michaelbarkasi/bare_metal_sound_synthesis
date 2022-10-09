# bare_metal_sound_synthesis
two-timer PWM for snythesizing sound with pitch, volume, and duty cycle control on SAMD51 and ESP32

This is code I created in 2021 for use in motion sonification experiments. 
I needed light weight sound synthesis on small embedded microprocessors that would respond with only a few microseconds of latency and leave the vast majority of the processor time available for motion processing.

sketch_SAMD51_PWM_HellowWorld_twotimer - Basic sketch for Sparkfun Thing Plus SAMD51 (Cortex M4F) using a TCC and TC

sketch_SAMD51_PWM_HellowWorld_twotimerHarmonics - Same, but with a TCC and TCC allowing control of hearable duty cycle and some extra code for harmonic undertones

sketch_SAMD51_PWM_HellowWorld_twotimerHarmonics-snyth - Same as above, but with some extra code to play varying sounds depending on input from potentiometers

ttPWMvDACwave - a version of the basic code for the ESP32
