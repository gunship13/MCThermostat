/* 
	Editor: http://www.visualmicro.com
	        arduino debugger, visual micro +, free forum and wiki
	
	Hardware: Arduino Uno, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328P__
#define ARDUINO 101
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
extern "C" void __cxa_pure_virtual() {;}

//
//
int get_key(unsigned int input);
float getRmTemp(void);
void runThermostat(void);
void commandRelays(void);
void initializeTimers(void);

#include "C:\Users\MikeAdmin\Documents\Arduino\arduino-1.0.5\hardware\arduino\variants\standard\pins_arduino.h" 
#include "C:\Users\MikeAdmin\Documents\Arduino\arduino-1.0.5\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\MikeAdmin\Documents\Arduino\MikeTrmStat_V1_0\MikeTrmStat_V1_0.ino"
