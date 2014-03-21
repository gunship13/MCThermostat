     // DATE 3/21/14
// Thermostat Project by Mike Cipolla
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <stdio.h>
// NOTE:  The relay board does not work with SERIAL I/O So Do Not Set DEBUG if connected to Furnace
// #define DEBUG          // Comment this line out if you do not want to include Debug Code
#define VERSION  0        // This is minor version displayed at startup
#define TOP_LINE 0
#define BOT_LINE 1
#define COL_ONE  0

#define CMD_IDLE      0   //  Mode Commands Output to Relays by Thermostat
#define CMD_COOL      1
#define CMD_HEAT      2
#define CMD_EMR_HEAT  3

// Interface to the Furnace is done with these relays
//
//                          Furnace Wire Connections
//                          Relay Common connects to Red              MODE Commands X = Closed, 0 = Open
//                          Relay Ground connects to Brown            Off Cool Heat EmrHeat Fan 
#define RELAY1  0       //  Relay 1 Normaly Open connects to Green     O   X    X    X       X
#define RELAY2  1       //  Relay 2 Normaly Open connects to Yellow    O   X    X    X       O
#define RELAY3  2       //  Relay 3 Normaly Open connects to Orange    O   X    O    O       O
#define RELAY4  3       //  Relay 4 Normaly Open connects to White     O   0    0    X       0

// Form these Relay Modes into Hex Codes
#define RELAY_COOL     0x07
#define RELAY_HEAT     0x03
#define RELAY_EMR_HEAT 0x0B
#define RELAY_FAN      0x01
#define RELAY_OFF      0x00

#define MIN_INPUT false              // Input State for Min Input Temp with Auto Mode
#define MAX_INPUT true               // Input State for Max Input Temp with Auto Mode

// Define Constanse for EEPROM and API interface
#define HEADER_SYNC      0xA5A55A5A
#define PROTOCAL_VERSION     0x0001 
#define THERMOSTAT_CMD       0x00F0



// #define LCD_HW_V0 0               // This would be for unmodified HW
#define LCD_HW_V1   1                // 470 Ohm resistor on Analog In Pin 0

#ifdef LCD_HW_V0
#define REFERANCE_VOLTAGE 5.0
#endif 

#ifdef LCD_HW_V1
#define REFERANCE_VOLTAGE 1.1
#endif 

#define ANALOG_LSB REFERANCE_VOLTAGE/1023.0        // define LSB for 10 bit A/D conversion

#define TEMP_IN_CONVERT 0.010  // 10mV per Deg C
#define TEMP_DIFF 0.9          // This provides the hysteresis for the Heating and Cooling Cycling On and Off

// External Declarations
extern volatile unsigned long timer0_millis;   // Get direct access to the Arduino timer used by millis() so 
                                               //   a graceful reset can be performed before it rolls over.

//LiquidCrystal(rs, rw, enable, d4, d5, d6, d7)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
 
// Function Prototypes, Note The Arduino SDK does not require these to be defined here but ANSI C would.
float getRmTemp(void);
void  runThermostat(void);
void  commandRelays(void); 
void  initializeTimers(void);

char myMsg[80];      // This is a temporary holder for strings being formated for eventual output to the LCD.

#ifdef LCD_HW_V0 
int  adc_key_val[5] ={30, 150, 360, 535, 760 };    // use 5 Volt A/D Reference
#endif
#ifdef LCD_HW_V1 
int  adc_key_val[5] ={200, 520, 710, 810, 875 };   // use 1.1 Volt A/D Reference and 470 Ohm HW Modification
#endif

int NUM_KEYS = 5;
enum KEYS {RT_KEY, UP_KEY, DN_KEY, LT_KEY, SEL_KEY};  // This is the order of the Keys or Buttons on the LCD 
                                                      //   Display
int adc_key_in;
int key=-1;
int oldkey=-1;
int KEY_PRESS_PIN = A0;
int TEMP_SENSOR_PIN = A1;
 
// Mode Menue Items
#define MAX_MODE_NUMBER   6
#define MIN_TEMP_CMD     40         // This program will limit the Temp Cmd to min of 40 deg Fahrenheit
#define MAX_TEMP_CMD     99         // This program will limit the Temp Cmd to max of 99 deg Fahrenheit 

String menuString[MAX_MODE_NUMBER] = {"OFF     ","COOL    ","HEAT    ","EMG HEAT", "AUTO    ","SET FAN "};
enum SYS_MODES {SYS_OFF, SYS_COOL, SYS_HEAT, SYS_EMR_HEAT, SYS_AUTO, FAN_SWITCH};

int relayCommand = RELAY_OFF;     //  State of the output relays, Initialize to OFF
int backLightPin = 10;            // LCD Back Light Control on Pin 10

float rmTemp;                     // Best Estimate of current room Temperature in Degs. Fahrenheit

boolean twoSecondStartupFlag = true;     // Force system to do mode change logic on startup  

// Define Structure To Hold Values in EEProm Memory across loss of power or restarts
// This will also be used in the future as an API Interface for remote control of the thermostat.
struct flashSave { 
  unsigned long  headerSync;           // 0xA5A55A5A
  int            msgLength;            // This length tells how many bytes follow. (does not include headerSync or msgLength)
  int            protocallVersion;     // 0x0001 is the first version
  int            msgType;              // 0x00F0 is the only message for now
  enum SYS_MODES menuMode;             // Index to the current menu being displayed
  enum SYS_MODES bufferedMode;         // Tentative Mode Command
  enum SYS_MODES operateMode;          // Final Active Mode Command
  enum SYS_MODES returnFromAutoMode;   // Hold the mode AUTO MODE was entered from 
  int            minAutoTemp;          // Minimum Temp Cmd for Auto Mode (Where Heat is switched on)
  int            maxAutoTemp;          // Maximum Temp Cmd for Auto Mode (Where Cool is switched on)
  boolean        sysFanOn;             // False - set the fan off except when another mode needs it.  Often called "Fan Auto".
                                       //   So the "Fan Always On" would be indicated with this flag set "true"
  int            tempCmd[MAX_MODE_NUMBER]; // The user commanded temperature for each mode
  int            spare_FFFF;           // Set this to 0xFFFF to help with debug
} currentState, eeWrittenState;


// Timers
// Caution:  Be sure to add any new clock timers to the initializeTimers() rotine so that clock rollover is prevented.
unsigned long previousClk_ms       =    0L;
unsigned long previousTwoSecClk_ms =    0L;
unsigned long previousTempClk_ms   =    0L;
unsigned long countDown90Min_ms    =    0L;

// Define several timer durations scaled to mili-seconds
const unsigned long time250_ms   =  250L; 
const unsigned long timeOneSec   = 1000L;
const unsigned long timeTwoSec   = 2000L;
const unsigned long time90Min    = 90*60*1000L;       // 90 Minutes
const unsigned long time30Days   = 30*24*60*60*1000;  // 30 days is a time a little short of 50 days when the long timer will overflow

#define KEY_HOLD_HALF_SEC 2            // 250ms * 2 = Half Second 

/*
 *****************************************************************************************
 *  Function: setup()
 */
  
void setup() {

  initializeTimers();   // Set up all the timers
  int ii;
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  pinMode(KEY_PRESS_PIN,INPUT);
  pinMode(TEMP_SENSOR_PIN,INPUT);
  
  #ifdef LCD_HW_V1
  analogReference(INTERNAL);      // If new H/W mod then use internal 1.1 Volt A/D reference           
  #endif
  
  // Recover the EEPROM State Of Thermostat
  char *pEEWrittenState = (char*)&eeWrittenState;
  for(ii=0; ii<sizeof(eeWrittenState); ii++){
    *pEEWrittenState++ = EEPROM.read(ii);
  }
  
  // Protect from any bad mode command by setting all sytem and menu states to System Off.
  // This is good recovery for bad EEPROM Data.  The API Interface should have a more robust
  // error check and status return to the remote device than what is here.  This can stay as
  // a second level of protection.
  if( (eeWrittenState.menuMode > MAX_MODE_NUMBER) || (eeWrittenState.bufferedMode > MAX_MODE_NUMBER) ||  
    (eeWrittenState.operateMode > MAX_MODE_NUMBER) || (eeWrittenState.returnFromAutoMode > MAX_MODE_NUMBER) )
    {
      eeWrittenState.menuMode = eeWrittenState.bufferedMode = 
         eeWrittenState.operateMode = eeWrittenState.returnFromAutoMode = SYS_OFF;
    }
  // Protect from any temperature commands that are out of range
  for(ii=0; ii < MAX_MODE_NUMBER; ii++){
    if(eeWrittenState.tempCmd[ii] < MIN_TEMP_CMD)
      eeWrittenState.tempCmd[ii] = MIN_TEMP_CMD;
    else if(eeWrittenState.tempCmd[ii] > MAX_TEMP_CMD)
      eeWrittenState.tempCmd[ii] = MAX_TEMP_CMD;
  }
  
  if(eeWrittenState.maxAutoTemp > MAX_TEMP_CMD)
     eeWrittenState.maxAutoTemp = MAX_TEMP_CMD;
  if(eeWrittenState.minAutoTemp < MIN_TEMP_CMD)
    eeWrittenState.minAutoTemp = MIN_TEMP_CMD;
  
  
  // Now that we have done sanity checks and corrections save EEPROM commands to currentState.
  pEEWrittenState = (char*)&eeWrittenState;
  char *pCurrentState = (char*)&currentState;
  for(ii=0; ii<sizeof(eeWrittenState); ii++){
    *pCurrentState++ = *pEEWrittenState++;
  }
  
  // Start the Screen Displays
  lcd.begin(16, 2);
  lcd.setCursor(TOP_LINE, 1);
  lcd.setCursor(COL_ONE, TOP_LINE);  // left Col Bot Line
  lcd.print("Version: 1.");
  lcd.print((String)VERSION);
  delay(2000);
  pinMode(backLightPin,OUTPUT);//Back Light LED is on Pin 10
  analogWrite(backLightPin,50);// dim the display
  lcd.clear();
  
  lcd.print(currentState.tempCmd[currentState.operateMode]);
  lcd.setCursor(COL_ONE, TOP_LINE);  // left Col Top Line
  lcd.print(menuString[currentState.menuMode]);

 // set the digital pin as output:
  pinMode(RELAY1, OUTPUT); 
  pinMode(RELAY2, OUTPUT);   
  pinMode(RELAY3, OUTPUT);   
  pinMode(RELAY4, OUTPUT);  

}  // end setup()

/*
*****************************************************************************************
*  Function loop() 
*/
 
void loop() {
 
   unsigned long currentClk_ms = millis();
   
   // The 250ms Code Block ////////////////////////////////////////////////////////////////////
   if(currentClk_ms - previousClk_ms > time250_ms) {
    previousClk_ms = currentClk_ms;
 
    int button = analogRead(KEY_PRESS_PIN);
    int key = get_key(button);
    static int keyHeldCnt;
    static boolean inputToggle = MIN_INPUT;  // Start with MIN_INPUT
    


    if(button >= adc_key_val[NUM_KEYS-1]){  // Compare to Last adc_key_val
        keyHeldCnt = 0;   // Restet the Key Held Count when no key is pressed 
      }
 
      if((key == RT_KEY && (currentState.menuMode != FAN_SWITCH))) {  // Right Button -  Temperature Cmd Up
        currentState.tempCmd[currentState.menuMode]++;
        if( currentState.tempCmd[currentState.menuMode] > MAX_TEMP_CMD)
          currentState.tempCmd[currentState.menuMode] = MAX_TEMP_CMD;
        if((currentState.menuMode == SYS_AUTO) && (inputToggle == MIN_INPUT)){
          currentState.minAutoTemp = currentState.tempCmd[currentState.menuMode];
          lcd.setCursor(4, BOT_LINE);   // place brackets around Min Auto Temp Display Numbers
          lcd.print(currentState.minAutoTemp);
        }
        if((currentState.menuMode == SYS_AUTO) && (inputToggle == MAX_INPUT)){
          currentState.maxAutoTemp = currentState.tempCmd[currentState.menuMode];
          lcd.setCursor(8, BOT_LINE);   // place brackets around Min Auto Temp Display Numbers
          lcd.print(currentState.maxAutoTemp);
        }         
        if(currentState.menuMode != SYS_AUTO){
          lcd.setCursor(COL_ONE, BOT_LINE);  // Left Col Bottom Line
          lcd.print((String)currentState.tempCmd[currentState.menuMode]);
        }
      }  // end key RIGHT
     
      if((key == LT_KEY) && (currentState.menuMode != FAN_SWITCH)) {  // Left Button  -  Temperature Cmd Down
       currentState.tempCmd[currentState.menuMode]--;
        if( currentState.tempCmd[currentState.menuMode] < MIN_TEMP_CMD)
          currentState.tempCmd[currentState.menuMode] = MIN_TEMP_CMD;
        if((currentState.menuMode == SYS_AUTO) && (inputToggle == MIN_INPUT)){
          currentState.minAutoTemp = currentState.tempCmd[currentState.menuMode];
          lcd.setCursor(4, BOT_LINE);   // place brackets around Min Auto Temp Display Numbers
          lcd.print(currentState.minAutoTemp);
        }
        if((currentState.menuMode == SYS_AUTO) && (inputToggle == MAX_INPUT)){
          currentState.maxAutoTemp = currentState.tempCmd[currentState.menuMode];
          lcd.setCursor(8, BOT_LINE);   // place brackets around Min Auto Temp Display Numbers
          lcd.print(currentState.maxAutoTemp);
        }         
        if(currentState.menuMode != SYS_AUTO){
          lcd.setCursor(COL_ONE, BOT_LINE);  // Left Col Bottom Line
          lcd.print((String)currentState.tempCmd[currentState.menuMode]);
        }   
      }  // end key LEFT
     
      if(key == SEL_KEY) {  // SELect Button  -  Thermostat Mode Toggle to next mode
        if(keyHeldCnt >= 0){
          keyHeldCnt = -1;   //  only toggle once per key press.
          int myMenuMode = (int)currentState.menuMode;
          myMenuMode++;
          myMenuMode %= MAX_MODE_NUMBER;                // Perform wrap around back to SYS_OFF
          currentState.menuMode = (enum SYS_MODES)myMenuMode;      // Save as enum of SYSMODES
          lcd.setCursor(COL_ONE, TOP_LINE);               // Send mode text to left Col Top Line
          lcd.print(menuString[currentState.menuMode]); 
          lcd.setCursor(COL_ONE, BOT_LINE); 
          lcd.print("                ");              // Clear the bottom line
          if((currentState.menuMode != SYS_OFF) && (currentState.menuMode != FAN_SWITCH)){
            lcd.print(currentState.tempCmd[currentState.menuMode]);             // Display the Selection Temperature
          }
        }
      }  // end key SEL
      
     if((key == DN_KEY) && (currentState.menuMode == SYS_AUTO)) { // Down  - For Auto Mode Toggle between Min and Max Input Selection
        if(keyHeldCnt > 0){
          inputToggle ^= 1;
          keyHeldCnt = -1;   //  only toggle once per key press.        
          if(inputToggle == MIN_INPUT){
            currentState.tempCmd[currentState.menuMode] = currentState.minAutoTemp;  // Start back with last minAutoTemp
            lcd.setCursor(7, BOT_LINE);    // remove brackets from around Max Auto Temp Display Numbers
            lcd.print(" ");
            lcd.setCursor(10, BOT_LINE);
            lcd.print(" ");
            lcd.setCursor(3, BOT_LINE);   // place brackets around Min Auto Temp Display Numbers
            lcd.print("<");
            lcd.print(currentState.tempCmd[currentState.menuMode]);
            lcd.setCursor(6, BOT_LINE); 
            lcd.print(">");
          }
          else{        // MAX_INPUT
            currentState.tempCmd[currentState.menuMode] = currentState.maxAutoTemp;  // Start back with last maxAutoTemp
            lcd.setCursor(3, BOT_LINE);    // remove brackets from around Min Auto Temp Display Numbers
            lcd.print(" ");
            lcd.setCursor(6, BOT_LINE);
            lcd.print(" ");
            lcd.setCursor(7, BOT_LINE);   // place brackets around Max Auto Temp Display Numbers
            lcd.print("<");
            lcd.print(currentState.tempCmd[currentState.menuMode]);
            lcd.setCursor(10, BOT_LINE);
            lcd.print(">");
          }
        }  // end KEY_HOLD_HALF_SEC
        else if (keyHeldCnt >= 0)
        {
          keyHeldCnt++;           
        }
     } // end key DOWN
      
    if((key == UP_KEY) && (currentState.menuMode == FAN_SWITCH)) { // UP  -  Select Fan Auto or On
      if(keyHeldCnt >= 0){
        keyHeldCnt = -1;   //  only toggle once per key press.
        lcd.setCursor(COL_ONE, BOT_LINE);
        currentState.sysFanOn ^= 1;    // Toggle The Fan Setting
        if (currentState.sysFanOn == true) {
          lcd.print("ON  ");
        }
        else
        {
          lcd.print("AUTO");
        } 
      }  
    } // end key UP and Fan
      
    //
    // The UP key is used to easily toggle the Auto Mode on and off
    //  
    // UP  -  Toggle Auto Mode OFF
    if((key == UP_KEY) && (currentState.menuMode != FAN_SWITCH) && (currentState.menuMode == SYS_AUTO) && (keyHeldCnt >= 0)) { 
      keyHeldCnt = -1;                                           //  only toggle once per key press.
      currentState.menuMode = currentState.returnFromAutoMode;   // Resore Menu from which Auto Mode was entered from
      lcd.setCursor(COL_ONE, TOP_LINE);                          // Send mode text to left Col Top Line
      lcd.print(menuString[currentState.menuMode]); 
      lcd.setCursor(COL_ONE, BOT_LINE); 
      lcd.print("                ");                             // Clear the bottom line  
    }
    
    // UP  -  Toggle Auto Mode ON
    else if((key == UP_KEY) && (currentState.menuMode != SYS_AUTO)&& (currentState.menuMode != FAN_SWITCH)&& (keyHeldCnt >= 0)) {
      keyHeldCnt = -1;                                         //  only toggle once per key press. 
      currentState.returnFromAutoMode = currentState.menuMode; //  save mode to return to
      currentState.menuMode = (enum SYS_MODES)SYS_AUTO;        //  swich to AUTO Mode
      lcd.setCursor(COL_ONE, TOP_LINE);                        // Send mode text to left Col Top Line
      lcd.print(menuString[currentState.menuMode]); 
      lcd.setCursor(COL_ONE, BOT_LINE); 
      lcd.print("                ");                           // Clear the bottom line 
    }      
}  // end of 250 ms block
 
// One Second Code Block  ////////////////////////////////////////////////////////////////////////////////////
    currentClk_ms = millis(); 
    
    if(currentClk_ms - previousTempClk_ms > timeOneSec) {
        previousTempClk_ms = currentClk_ms;
        rmTemp = getRmTemp();
        
        int wholTemp = (int)(rmTemp);
        int fracTemp = (int)((rmTemp-wholTemp)*10);
        sprintf(myMsg, "%3d.%dF",wholTemp, fracTemp);  // Temperature in "xxx.y" format followed by "F"
        lcd.setCursor(10, TOP_LINE);                   // Col 11 Top Line
        lcd.print((String)myMsg);                      // Write the current room tempurature to the LCD 
       
        runThermostat();
        
        if (relayCommand == CMD_EMR_HEAT){
          lcd.setCursor(11, BOT_LINE);         // Display That Emergency Heat is Commanded
          lcd.print("EM");          
        }
        else {         
          lcd.setCursor(11, BOT_LINE);         // Clear to show That Emergency Heat is NOT Commanded
          lcd.print("  ");
        }
        
        commandRelays();                      // Send our commands to the Furnace through the relays
        
        if (millis() > time30Days)            // If we are getting close to timer overflow, then reset all timers
            initializeTimers();
  
      }  // end One Second Block
      
// Two Second Code Block ///////////////////////////////////////////////////////////////////////////////////////
    currentClk_ms = millis(); 
    
    if(currentClk_ms - previousTwoSecClk_ms > timeTwoSec) {
        previousTwoSecClk_ms = currentClk_ms;
       
       // Do we need a mode change?
       if (((currentState.bufferedMode == currentState.menuMode) && (currentState.operateMode != currentState.menuMode))
              || (twoSecondStartupFlag == true)){ 
         twoSecondStartupFlag = false;                            // only done one time at startup.              
         currentState.operateMode = currentState.menuMode;        // Command a mode change
         lcd.setCursor(COL_ONE, BOT_LINE);
         lcd.print("                ");                           // Clear Bottom Line
         if ((currentState.menuMode != SYS_OFF)&&(currentState.menuMode != SYS_AUTO)&&(currentState.menuMode != FAN_SWITCH )){
           lcd.setCursor(COL_ONE, BOT_LINE);
           lcd.print(currentState.tempCmd[currentState.operateMode]);  // Restore the Saved Temp Command for new non-off mode
         }
         if (currentState.menuMode == SYS_AUTO){
            currentState.tempCmd[currentState.menuMode] = currentState.minAutoTemp;  // Start back with last minAutoTemp
            lcd.setCursor(7, BOT_LINE);                        // remove brackets from around Max Auto Temp Display Numbers
            lcd.print(" ");
            lcd.setCursor(10, BOT_LINE);
            lcd.print(" ");
            lcd.setCursor(3, BOT_LINE);                        // place brackets around Min Auto Temp Display Numbers
            lcd.print("<");
            lcd.setCursor(6, BOT_LINE); 
            lcd.print(">");
           
           lcd.setCursor(4, BOT_LINE);
           lcd.print(currentState.minAutoTemp);
           lcd.setCursor(8, BOT_LINE);
           lcd.print(currentState.maxAutoTemp);
         }        
       } // end of mode change
       
     // Display the Fan On Indicator
     lcd.setCursor(13, BOT_LINE);
     if (currentState.sysFanOn == true)
        lcd.print("FAN");    // signal the fan is commanded to be on all the time
      else
        lcd.print("   ");   // clear the "FAN" when in AUTO Fan
 
      currentState.bufferedMode = currentState.menuMode;  // ??? I'm not sure if this line is needed
       
     // Test if new mode commands need to be sent to the EEPROM
     
      char* pEEWrittenState = (char*)&eeWrittenState;
      char *pCurrentState = (char*)&currentState;
      boolean needUpdate = false;
      int ii;
      
      for(ii=0; ii<sizeof(eeWrittenState); ii++){
        if( *pCurrentState++ != *pEEWrittenState++)
        {
         needUpdate = true;
         break;                 // found difference so go do update now
        }
      }
      
      if(needUpdate)
       {
          // Now that we know we need to update EEPROM, save EEPROM commands to currentState
          // and save currentState to EEPROM.
          // Set up Header and Size
          currentState.headerSync = 0xA5A55A5A;           // 0xA5A55A5A
          currentState.msgLength = sizeof(currentState)- sizeof(long int) - sizeof(int) ; // This length tells how many bytes
          currentState.protocallVersion = 0x0001;     // 0x0001 is the first version
          currentState.msgType = 0x00F0;              // 0x00F0 is the only message for now
          currentState.spare_FFFF = 0xFFFF;           // 0xFFFF Spare to help with debug
          
          pEEWrittenState = (char*)&eeWrittenState;
          pCurrentState   = (char*)&currentState;

          for(ii=0; ii<sizeof(eeWrittenState); ii++){               
            EEPROM.write(ii,*pCurrentState);
            *pEEWrittenState++ = *pCurrentState++;   
          }
       }  // end needUpdate
    }  // end 2 second block             
}  // end of loop()

/*
**********************************************************************************************
*  Function: get_key(input)
*/

int get_key(unsigned int input)
{
  int k;
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
       return k;   // Return the index of the key pressed
  }
  if (k >= NUM_KEYS)
    k = -1;          // No valid key pressed
  return k;
}  // end get_key

/*
**********************************************************************************************
*  Function: getRmTemp()
*/
 
float getRmTemp(void)
{
  static float tempSmooth;
  int tempRead;
  float tempReadF;
  tempRead = analogRead(TEMP_SENSOR_PIN);  // Read in voltage in format
  tempReadF = ANALOG_LSB  * (float)tempRead;
  tempReadF /= TEMP_IN_CONVERT;
  tempReadF = tempReadF * 9.0/5.0 + 32;       // convert from deg. C to deg. F
  if(twoSecondStartupFlag == false)
    tempSmooth = tempSmooth * 0.875 + tempReadF * 0.125;  // filter to smooth the temperature reading.
  else
    tempSmooth = tempReadF;   // No filtering for the first 2 seconds to aquire rough temperature quickly

#ifdef DEBUG
    int wholTemp = (int)(tempReadF);
    int fracTemp = (int)((tempReadF-wholTemp)*10);
    int wholSmTemp = (int)(tempSmooth);
    int fracSmTemp = (int)((tempSmooth-wholSmTemp)*10);
    sprintf(myMsg, "tempReadF %d.%d A/D %d TempSmooth %d.%d \n",wholTemp, fracTemp, tempRead, wholSmTemp,fracSmTemp );
    Serial.println(myMsg);
#endif

   return tempSmooth;
} // end getRmTemp

/*
**********************************************************************************************
*  Function: runThermostat()
*/
void runThermostat(void) {
  switch ( currentState.operateMode){
    
    case SYS_COOL:
      if(rmTemp > (currentState.tempCmd[currentState.operateMode] + TEMP_DIFF))
        relayCommand = CMD_COOL;
      else if(rmTemp < (currentState.tempCmd[currentState.operateMode] - TEMP_DIFF))
        relayCommand = CMD_IDLE;
      break;
      
    case SYS_HEAT:
      if(rmTemp > (currentState.tempCmd[currentState.operateMode] + TEMP_DIFF))
        relayCommand = CMD_IDLE;
      else if(rmTemp < (currentState.tempCmd[currentState.operateMode] - TEMP_DIFF - TEMP_DIFF))  // IF way behind in heating room
        relayCommand = CMD_EMR_HEAT;
      else if(rmTemp < (currentState.tempCmd[currentState.operateMode] - TEMP_DIFF))
        relayCommand = CMD_HEAT;
      break; 
      
    case SYS_EMR_HEAT:
       if(rmTemp > (currentState.tempCmd[currentState.operateMode] + TEMP_DIFF))
        relayCommand = CMD_IDLE;
       else if(rmTemp < (currentState.tempCmd[currentState.operateMode] - TEMP_DIFF))
        relayCommand = CMD_EMR_HEAT;
       break;
      
    case SYS_AUTO:
      if((rmTemp > (currentState.minAutoTemp + TEMP_DIFF)) && (rmTemp < (currentState.maxAutoTemp - TEMP_DIFF)))
        relayCommand = CMD_IDLE;
      else if(rmTemp < (currentState.minAutoTemp - TEMP_DIFF - TEMP_DIFF))  // IF way behind in heating the room
        relayCommand = CMD_EMR_HEAT;  
      else if((rmTemp < (currentState.minAutoTemp - TEMP_DIFF))) 
        relayCommand = CMD_HEAT;
      else if(rmTemp > (currentState.maxAutoTemp + TEMP_DIFF))
        relayCommand = CMD_COOL;
      break;
        
    default: 
    case FAN_SWITCH:
    case SYS_OFF:
       relayCommand = CMD_IDLE;
     break;
  } // end switch  
  
#ifdef DEBUG
      Serial.print("        RUN: ");
      Serial.print(operateMode);
      Serial.print(" CMD: ");
      Serial.print(relayCommand);
      Serial.print(" Menu: ");
      Serial.print(menuMode);
      Serial.print(" save: ");
      Serial.println(returnFromAutoMode);     
#endif

}  // end runThermostat()

/*
**********************************************************************************************
*  Function: commanRelays()
*/
void commandRelays(void)
{
  int i;
  byte relayOut;
  boolean  relaySwitch;
  
  switch (relayCommand){
    
    case CMD_COOL: {
      relayOut = RELAY_COOL;
      break;
    }
      
    case CMD_HEAT: {
      relayOut = RELAY_HEAT;
      break;
    }
      
    case CMD_EMR_HEAT: {
      relayOut = RELAY_EMR_HEAT;
      // ??? Display a signal that the emergency heat is being used.  This could be an LED driven by relay discreet out pin
      break;
    }
    
    default:
    case CMD_IDLE: {
      relayOut = RELAY_OFF;
      break;
    } 
  }  // end switch
  
  // if  "FAN ON" is selected, then turn the fan on fultime, even when off is selected.
  if (currentState.sysFanOn == true)
      relayOut |= RELAY_FAN;
  #ifdef DEBUG_KILL
    sprintf(myMsg, "Relay 0x%x = %d \n", int(relayOut), int(relayOut));
              Serial.print(myMsg);
  #endif
  for (i=0; i < 4; i++) {
    relaySwitch = relayOut & 0x01;   // Mask off LSB
    digitalWrite((RELAY1+i), relaySwitch); // Send command for one relay at a time
    #ifdef DEBUG_KILL
    sprintf(myMsg, "Relay %d = %d ", i, int(relaySwitch));
              Serial.print(myMsg);
    #endif
    relayOut >>= 1;             // Shift next bit to LSB
  }
  #ifdef DEBUG_KILL
    sprintf(myMsg, "/n ");
              Serial.print(myMsg);
    #endif
}

/*
**********************************************************************************************
*  Function: initializeTimers()
*/
// Call this at startup and after 30 days so as to gracefully reset all the timers before an overflow occurs.
// Caution:  Please add any other timers that are created to this routine so they are also reset.
void initializeTimers(void){
  
noInterrupts();     // Turn off interrupts

// Shift All Timers to new time base of Zero
previousClk_ms       -=  timer0_millis;
previousTempClk_ms   -=  timer0_millis;
previousTwoSecClk_ms -=  timer0_millis;
countDown90Min_ms    -=  timer0_millis;
timer0_millis        =   0L;   // this is the global Arduino timer being reset to zero

interrupts();      // re-enable interrupts

}
      
// End File

