// Many thanks to orly.andico@gmail.com, for the original command parser, others for bits on code picked up here and there on the net
///////////////////////////////////////////////
// cchristman - Modified to use NEMA-17 stepper and EasyDriver
//
//
//
// Since the MoonLite focuser only works with positive integers, I center (zero) my system at 30000. The range is from
// 0 to 65535, so 30000 is a good round number for the center.
// If the Current Position, or New Position is set to 0, this code will set the values at 30000. The reason for this
// is the system is designed to be set at center, manually focused, then focused in a +/- fashion by the controller.

// from adafruit site, I *think* this is the order they go on the easydriver:
// Coil #1: Red & Yellow wire pair. Coil #2 Green & Brown/Gray wire pair
// 
/*
From the EasyDriver site:

On a bi-polar stepper motor, there are four wires that connect to two coils. Each coils has two 
wires. The Easy Driver calls those two coils A and B. So one coils' wires are to be connected 
across the two A pins, and the other coils' wires are to be connected across the B pins. The driver 
chip datasheet refers to these two coils as coil 1 and coil 2.
*/
#include <DHT.h>
#include <AccelStepper.h>


#define HOME 30000
#define MAXCOMMAND 8
#define DHTTYPE DHT22

/////////////////////////
// Pin locations
#define RUNLED 13     // Amber LED lights whenever motor is active, 13 for nano 
#define DHTPIN 11     // DHT22 is on D11
#define STEP_PIN  7
#define DIRECTION_PIN  8
#define ENABLE_PIN  9

// m0/m1/m2 sets stepping mode 000 = F, 100 = 1/2, 010 = 1/4, 110 = 1/8, 001 = 1/16, 101 = 1/32
// steps per revolution = 200, 400, 800, 1600, 6400
// must set the current limiting for microstepping to work correctly
#define myM0      4  // microstepping lines
#define myM1      5
#define myM2      6

#define MAXSPEED  100
#define SPEEDMULT 3

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
char tempString[6];
unsigned int hashCmd;

int isRunning = 0;
int speed = 2;
int eoc = 0;
int idx = 0;
long millisLastMove = 0;
int testPin = 12;
int coilPwr = true;

// Temperature measurement
int temperatureChannel = 0;
unsigned int wADC;

#define SCALE 0.488 /* ADC>centigrade scale for my particular Teensy */
#define OFFSET 100 /* 2 * 50 offset for TPM-36 */

AccelStepper  stepper(AccelStepper::DRIVER,STEP_PIN, DIRECTION_PIN );
  
// Temperature sensor
//
DHT dht( DHTPIN, DHTTYPE);

int GetTemp(void) {
  // supposed to be "four-digit signed (2's complement) hex number
  // but docs are inconsistent
  //
  float temp;
  int itemp;
  temp  = dht.readTemperature();
  if ( isnan(temp) )  {
    temp = 0;
  } 
  itemp = (int) temp * 2;
  return itemp;
}

long hexstr2long(char *line) {
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// start of program

void setup()  {
  Serial.begin(9600);
  
  pinMode(RUNLED, OUTPUT); /* yellow LED */
  pinMode(testPin, OUTPUT);

  stepper.setSpeed( MAXSPEED );
  stepper.setMaxSpeed( MAXSPEED );
  stepper.setAcceleration( 10 );
  stepper.enableOutputs();

  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();
}

// Forever Loop
void loop() { 
  int tempTemp;
  long pos;

  if (!Serial.available()) {
  
    // run the stepper if there's no pending command and if there are pending movements
    //
    if (isRunning) {
      stepper.run();
      millisLastMove = millis(); /* reset idle timer */
      digitalWrite( RUNLED, HIGH );
    } else { /* Check to see if idle time is up */

      if ((millis() - millisLastMove) > 15000) {
        // if so, turn off motor
        if (coilPwr = false)  {
          stepper.disableOutputs();
        }
      }

    }
    
    // if motion is complete
    if (stepper.distanceToGo() == 0) {  
      stepper.run();
      isRunning = 0;
      digitalWrite(RUNLED, LOW);
    }
  } else {

    // read the command until the terminating # character
    //
    while (Serial.available() && !eoc) {
      inChar = Serial.read();
      if (inChar != '#' && inChar != ':') {
        line[idx++] = inChar;

        if (idx >= MAXCOMMAND)
          idx = MAXCOMMAND - 1;
      }
      else {
        if (inChar == '#')
          eoc = 1;
      }
    }
  } // end if (!Serial.available())

  //////////////////////////////////////
  // process the command we got
  //
  if (eoc) {
    digitalWrite(testPin, LOW);

    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(line);

    if (len >= 2)
      strncpy(cmd, line, 2);

    if (len > 2)
      strncpy(param, line + 2, len - 2);

    memset(line, 0, MAXCOMMAND);
    eoc = 0;
    idx = 0;

    // the stand-alone program sends :C# :GB# on startup
    // :C# is to start a temperature conversion, doesn't require any response (we don't use it)
    hashCmd = (byte(cmd[0]) | (byte(cmd[1]) << 8)); /* combine the two command charaters into an unsigned int */

    switch (hashCmd) {
      case ('P'<<8 | 'G'):
        // GP command Get current position
        //
        pos = stepper.currentPosition();
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('T'<<8 | 'G'):        
        // GT command Get Temperature
        //
        tempTemp = GetTemp();
        sprintf(tempString, "%04X", tempTemp);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('I'<<8 | 'G'):        
        // GI command 01 if motor running, 00 if not
        //
        if ( abs(stepper.distanceToGo()) > 0 )
          Serial.print("01#");
        else
          Serial.print("00#");
        break;

      case ('B'<<8 | 'G'):        
        // GB command Get current backlight value, always 00
        //
        Serial.print("00#");
        break;

      case ('H'<<8 | 'P'):
        // PH command Find motor home
        //
        stepper.setCurrentPosition( HOME );
        stepper.moveTo( 0 );
        isRunning = 1;
        digitalWrite(RUNLED, HIGH);
        break;

      case ('V'<<8 | 'G'):
        // GV command Get software version, always 10
        //
        Serial.print("10#");
        break;

      case ('N'<<8 | 'G'):
        // GN command Get new (target) position
        //
        pos = stepper.targetPosition();
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('C'<<8 | 'G'):
        // GC command Get temerature coefficient, always 2
        //
        Serial.print("02#");
        break;

      case ('D'<<8 | 'G'):
        // GD command Get motor speed
        //
        sprintf(tempString, "%02X", speed);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('D'<<8 | 'S'):
        // SD command Set motor speed
        //
        speed = hexstr2long(param);
        stepper.setSpeed( MAXSPEED );
        stepper.setMaxSpeed( MAXSPEED );
        break;

      case ('H'<<8 | 'G'):
        // GH command Get half step mode, always 00
        //
        Serial.print("00#");
        break;

      case ('P'<<8 | 'S'):
        // SP command Set current position
        //
        pos = hexstr2long(param);
        stepper.setCurrentPosition( pos );        
        break;

      case ('N'<<8 | 'S'):
        // SN command Set new position
        //
        pos = hexstr2long(param);
        stepper.moveTo( pos );
        break;

      case ('G'<<8 | 'F'):
        // FG command Start motor command
        //
        stepper.enableOutputs();
        isRunning = 1;
        digitalWrite(RUNLED, HIGH);
        break;

      case ('Q'<<8 | 'F'):
        // FQ command Stop motor command
        //
        isRunning = 0;
        stepper.moveTo( stepper.currentPosition() );
        stepper.run();
        digitalWrite(RUNLED, LOW);
        break;

    }
    digitalWrite(testPin, HIGH);

  } // end process command
} // end forever loop


