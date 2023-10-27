//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "StriveON_Bobra"


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 222 bytes
  { 255,1,0,37,0,215,0,16,31,1,71,56,0,7,63,63,4,2,24,135,
  0,0,0,0,0,0,160,66,0,0,160,65,0,0,32,65,0,0,0,64,
  24,75,109,47,104,0,135,0,0,0,0,0,0,208,65,189,0,0,200,65,
  0,0,16,66,36,0,0,12,66,0,0,56,66,24,0,0,52,66,0,0,
  160,66,3,3,2,71,9,24,2,26,67,5,20,46,22,7,2,31,11,129,
  0,0,1,62,6,17,77,97,110,97,66,111,98,114,97,32,67,111,110,116,
  114,111,108,108,101,114,0,129,0,13,89,9,4,1,82,97,99,101,0,129,
  0,13,81,27,4,6,78,111,114,116,104,32,65,109,101,114,105,99,97,0,
  129,0,13,74,13,4,135,69,117,114,111,112,101,0,67,6,26,61,30,6,
  119,16,11,129,0,0,62,24,4,8,84,114,105,112,32,100,105,115,116,97,
  110,99,101,0,129,0,57,62,4,5,8,77,0,67,4,38,89,20,5,1,
  26,11 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t multi = 1; // =0 if select position A, =1 if position B, =2 if position C, ... 

    // output variables
  float real_speed;  // from 0 to 80 
  char realspeed[11];  // string UTF8 end zero 
  char distance[11];  // string UTF8 end zero 
  char report_speed[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)


#define REED_SENSOR_PIN GPIO_NUM_14
#define SETUP_PIN GPIO_NUM_12
#define ELECTROMAGNET_PIN GPIO_NUM_13
#define REDLED_LOWPRESSCOUNT_PIN GPIO_NUM_25
#define MAGPLUS_PIN GPIO_NUM_32
#define MAGMIN_PIN GPIO_NUM_33

byte ButtonState;
byte LastButtonState;
byte Trigger = HIGH;
byte SetupState;

unsigned long LastPress = 0;
unsigned long CurrentPress = 0;
unsigned long EndMagnet = 0;
double MagnetDuration = 47;
unsigned long Delta;
unsigned long DeltaStatic;
unsigned long ActivateMagnetStart = 0;
unsigned long ActivateMagnetTemp = 0;
double MultiplierOut = 1;
unsigned long MagnetOnTime = 0;
int MultiplierCutOff = 1398; //1398 = 6kph/3.5mph 
unsigned long halfmag = 0;
int Pressed = 0;
int PressCount = 0;
int MagnetSwitch = 0;
int MagnetOn = 0;
unsigned long BridgeTime = 0;
unsigned long previousBridgeTime = 0;
unsigned long Sleep = 0;
float distance;
float report_speed;
const double WHEEL_CIRCUMFERENCE = 2.247;

////////////////////
//change this MultiplierOut to alter max assistance speed. MultiplierOut 1.4 gives: 25kph * 1.4 = 35kph new max assitance
double MultiplierIn = 1.4;
int SetupDelay = 513;
int SetupWidth = 47;

void hbridge(int Sleep) {
  BridgeTime = millis();
  if (BridgeTime - previousBridgeTime < (Sleep/2)) {
    digitalWrite(MAGMIN_PIN, LOW);
    digitalWrite(MAGPLUS_PIN, HIGH);
  }
  if ((BridgeTime - previousBridgeTime >= (Sleep/2)) && (BridgeTime - previousBridgeTime <= (Sleep))) {
    digitalWrite(MAGPLUS_PIN, LOW);
    digitalWrite(MAGMIN_PIN, HIGH);
  }
  previousBridgeTime = BridgeTime;
}

void hOff() {
  digitalWrite(MAGPLUS_PIN, LOW);
  digitalWrite(MAGMIN_PIN, LOW);
//  bitWrite(PORTB, 1, 0);
//  bitWrite(PORTB, 2, 0);
}

void setup() {
  RemoteXY_Init (); 
  pinMode(MAGPLUS_PIN, OUTPUT);
  pinMode(MAGMIN_PIN, OUTPUT);
  pinMode(ELECTROMAGNET_PIN, OUTPUT);
  pinMode(REED_SENSOR_PIN, INPUT_PULLUP);
//  pinMode(REED_SENSOR_PIN, INPUT);
  pinMode(SETUP_PIN, INPUT_PULLUP);
  pinMode(REDLED_LOWPRESSCOUNT_PIN, OUTPUT);
  LastButtonState = digitalRead(REED_SENSOR_PIN);
  SetupState = digitalRead(SETUP_PIN);
}

void loop() {
  RemoteXY_Handler ();
  SetupState = digitalRead(SETUP_PIN);
  //if we're in setup mode, run the consistent pulsing
  if (SetupState == LOW) {
    //activate coil
    digitalWrite(ELECTROMAGNET_PIN, HIGH);
    digitalWrite(REDLED_LOWPRESSCOUNT_PIN, HIGH);
    digitalWrite(MAGMIN_PIN, LOW);
    digitalWrite(MAGPLUS_PIN, HIGH);
    RemoteXY_delay(SetupWidth/2);
    digitalWrite(MAGPLUS_PIN, LOW);
    digitalWrite(MAGMIN_PIN, HIGH);
    RemoteXY_delay(SetupWidth/2);
    digitalWrite(ELECTROMAGNET_PIN, LOW);
    digitalWrite(REDLED_LOWPRESSCOUNT_PIN, LOW);
    hOff();
    RemoteXY_delay(SetupDelay);
  }
  //outside of setup mode, function as normal
  if (SetupState == HIGH) {
    switch (RemoteXY.multi) {
      case 0:
        MultiplierIn = 1;
        break;
      case 1:
        MultiplierIn = 1.4;
        break;
      case 2:
        MultiplierIn = 1.8;
        break;
    }
    ButtonState = digitalRead(REED_SENSOR_PIN);
    if ((ButtonState != LastButtonState) && (millis() - CurrentPress > 100)) {
      //Button pressed
      if (ButtonState == LOW) {
        LastPress = CurrentPress;
        CurrentPress = millis();
        Delta = CurrentPress - LastPress;
        RemoteXY.real_speed = (WHEEL_CIRCUMFERENCE / Delta) * 3600;
        report_speed = ((WHEEL_CIRCUMFERENCE / Delta) * 3600) / MultiplierIn;
        dtostrf(report_speed, 3, 1, RemoteXY.report_speed);
        dtostrf(RemoteXY.real_speed, 3, 1, RemoteXY.realspeed);
        distance += WHEEL_CIRCUMFERENCE;
        dtostrf(distance, 10, 1, RemoteXY.distance);
        Pressed = 2;
        digitalWrite(REDLED_LOWPRESSCOUNT_PIN, HIGH);
        if ((PressCount < 5) && ((CurrentPress - ActivateMagnetStart >= Delta) || PressCount > 0)) {
          digitalWrite(ELECTROMAGNET_PIN, HIGH);
          if ((MagnetDuration > 0) && (MagnetDuration < 100)) {
            Sleep = MagnetDuration * 1000;
          }
          else {
            Sleep = SetupWidth * 1000;
          }
          //BridgeTime = millis();
          if (micros() - previousBridgeTime < (Sleep/2)) {
            digitalWrite(MAGMIN_PIN, LOW);
            digitalWrite(MAGPLUS_PIN, HIGH);
          }
          if ((micros() - previousBridgeTime >= (Sleep/2)) && (micros() - previousBridgeTime <= (Sleep))) {
            digitalWrite(MAGPLUS_PIN, LOW);
            digitalWrite(MAGMIN_PIN, HIGH);
          }
          previousBridgeTime = micros();
          Trigger = HIGH;
          ActivateMagnetStart = CurrentPress;
          MultiplierOut = MultiplierIn;
          MagnetOnTime = millis();
          MagnetOn = 1;
        }
      }
      //Button released
      if (ButtonState == HIGH) {
        EndMagnet = millis();
        MagnetDuration = EndMagnet - CurrentPress;
	      halfmag = MagnetDuration/2;
        digitalWrite(REDLED_LOWPRESSCOUNT_PIN, LOW);
        if (PressCount < 5) {
          digitalWrite(ELECTROMAGNET_PIN, LOW);
	        hOff();
          MagnetOn = 0;
        }
        PressCount = PressCount + 1;
      }
      LastButtonState = ButtonState;
    }
 //if travelling slowly, fire coil 1:1
    if (millis() - ActivateMagnetStart > MultiplierCutOff) {
      PressCount = 0;
    }    

    //If wheel stops on magnet after travelling slowly, turn off the coil after 2seconds
    if ((MagnetOn == 1) && (millis() - MagnetOnTime > 2000)) {
        digitalWrite(ELECTROMAGNET_PIN, LOW);
        digitalWrite(REDLED_LOWPRESSCOUNT_PIN, LOW);
	      hOff();
      }

    //Record one Delta to avoid continual changes to interval during multiplied interval below
    if ((Trigger == HIGH) && (Delta > 0)) {
      DeltaStatic = Delta;
      Trigger = LOW;
    }

    //setup multiplier ramping
    if (PressCount >= 5) {
      if (PressCount < 10) {
        MultiplierOut = (((MultiplierIn - 1) / 5) * (PressCount - 4)) + 1;
      } else {
        MultiplierOut = MultiplierIn;
      }


      if (millis() >= ActivateMagnetStart + DeltaStatic * MultiplierOut) {
        if ((millis() < ActivateMagnetStart + DeltaStatic * MultiplierOut + MagnetDuration * MultiplierOut) && (Pressed > 0 )) {
          //Magnet On
          if (MagnetSwitch == 0) {
            ActivateMagnetTemp = millis();
          }
          MagnetSwitch = 1;
          digitalWrite(ELECTROMAGNET_PIN, HIGH);
	        if ((MagnetDuration > 0) && (MagnetDuration < 100)) {
            Sleep = MagnetDuration;
          }
          else {
            Sleep = SetupWidth;
          }
          if (micros() - previousBridgeTime < (Sleep/2)) {
            digitalWrite(MAGMIN_PIN, LOW);
            digitalWrite(MAGPLUS_PIN, HIGH);
          }
          if ((micros() - previousBridgeTime >= (Sleep/2)) && (micros() - previousBridgeTime <= (Sleep))) {
            digitalWrite(MAGPLUS_PIN, LOW);
            digitalWrite(MAGMIN_PIN, HIGH);
          }
          previousBridgeTime = micros();
          MagnetOnTime = millis();
          MagnetOn = 1;

        } else {
          //Magnet Off
          digitalWrite(ELECTROMAGNET_PIN, LOW);
	        hOff();
          ActivateMagnetStart = ActivateMagnetTemp;
          Trigger = HIGH;
          Pressed = Pressed - 1;
          MagnetSwitch = 0;
          MagnetOn = 0;
        }
      }
    }
  }
}
