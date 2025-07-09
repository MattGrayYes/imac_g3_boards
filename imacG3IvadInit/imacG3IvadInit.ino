
//Rocky Hill


/*
   This sketch waits for button presses to turn on/off the iMac G3 CRT
   circuitry and send the init sequence to the IVAD board.
   if you are planning to just send the init sequence on startup, you
   can uncomment "initIvadBoard();"  in setup(). It uses software i2c
   lines.

   it also sends edid information when requested via i2c on port 0x50.
   The Edid information is for an iMac G3 DV and it sends the three
   supported modes.

   1024x768 @ 75 Hz
   800x600 @ 95 Hz
   640x480 @ 117 Hz

   In order for this program to work, the i2c transmit buffer length
   constants must be changed in two files. The Wire library has two
   buffers it uses for i2c transmissions

   "BUFFER_LENGTH" in
   "arduino_install_folder/hardware/arduino/avr/libraries/Wire/src/Wire.h"
   "/Users/mattg/Library/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/Wire/src/Wire.h"
   
   and

   "TWI_BUFFER_LENGTH" in
   "arduino_install_folder/hardware/arduino/avr/libraries/Wire/src/utility/twi.h"
   "/Users/mattg/Library/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/Wire/src/utility/twi.h"

   Both of these must be changed from 32 to 128 to be able to transmit
   the edid byte array in one shot.

   Editing the libraries with an ifndef, this should let us define the new values here, rather than
   change the library for every use.
   
   For example in Wire.h
   #ifndef BUFFER_LENGTH
   #define BUFFER_LENGTH 32
   #endif

   To test this program, you can directly wire SDA(pin 12),SCL(pin 15)
   and GND(pin 6) pins from your computers VGA portbdirectly into the
   corresponding pins on the arduino.


*/


/*
  Uses SoftwareWire from the arduino libraries. install with library
  manager or from https://github.com/Testato/SoftwareWire
*/

#define BUFFER_LENGTH 128
#define TWI_BUFFER_LENGTH 128

#include "ivad.h"
#include "imacG3IvadInit.h"
#include <EEPROMWearLevel.h>
#include <SoftwareWire.h>
#include <Wire.h>

#if BUFFER_LENGTH < 128
#error Must patch the Wire library to send EDID correctly.
#endif

#if TWI_BUFFER_LENGTH < 128
#error Must patch the TWI library to send EDID correctly.
#endif

byte SERIAL_BUFFER[SERIAL_BUFFER_MAX_SIZE];
byte SERIAL_BUFFER_DATA_SIZE;
byte CURRENT_CONFIG[CONFIG_EEPROM_SLOTS];
byte FIRST_RUN = 0x79;

byte data = -1;

//define solid state relay and power button pins
byte solid_state_relay_Pin = 7;

byte powerButtonPin = 3;
//int powerButtonPin = 13;

//define state variables
byte externalCircuitState = LOW;
byte buttonState = LOW;

//vsync pin
byte vsyncPin = 10;

//vsync power off countdown in seconds
byte vsync_off_time = 180;

//counters
byte buttonPressedTime = 0;
byte vsyncDetect = 0;
unsigned long currentTime = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

// The init sequence is sent on a software i2c bus.
// sda is on 4 and scl is on 5
SoftwareWire softWire( 4, 5);

void setup() {

  //define pin direction
  pinMode(solid_state_relay_Pin, OUTPUT);
  pinMode(powerButtonPin, INPUT);
  pinMode(vsyncPin, INPUT);//this pin is used to monitor VSYNC.
  pinMode(8, INPUT);//this pin is on the J5 connector for general use PB0.
  pinMode(9, INPUT);//this pin is on the J5 connector for general use PB1.

  EEPROMwl.begin(CONFIG_EEPROM_VERSION, CONFIG_EEPROM_SLOTS + 1);
  Wire.begin(0x50);     //  VGA: join as slave and wait for EDID requests
  softWire.begin();     // IVAD: join as master and send init sequence to IVAD board
  Serial.begin(115200); // SELF: start built in serial to recieve commands & output status
  Serial.setTimeout(1000);

  Serial.println("");
  Serial.println("Hello, Turning on!");

  Wire.onRequest(requestEvent); // event handler for requests from master
  Wire.onReceive(receiveData);  // event handler when receiving from  master
  
  // turn it all off
  externalCircuitOff();

  //check to see if it's the 1st time running after burning firmware
  FIRST_RUN = EEPROMwl.read(CONFIG_EEPROM_SLOTS);
  if (FIRST_RUN != 0x79 ) {
    Serial.println("First Run, configuring EEPROM");
    EEPROMwl.update(CONFIG_EEPROM_SLOTS, 0x79);
    settings_reset_default();
    settings_store();
    settings_load();
    ivad_write_settings();
  }//end if


  //externalCircuitOn();


}//end setup

//byte x = 0;
/*
   This loops looks for button presses, turns the circuit on or off, and
   listens for characters on the serial port to make screen adjustments.
*/
void loop() {

  buttonState = digitalRead(powerButtonPin);

  // do stuff only when the CRT is on
  if ( externalCircuitState == HIGH ) {

    currentTime = millis();
    elapsedTime = currentTime - startTime;

    serial_processing();

  }//end if


  if (buttonState == LOW )
  {
    if (buttonPressedTime <= 10) {
      buttonPressedTime++;
    }//end if
  }
  else
  {
    //buttonPressedTime = 0;
  }

  //turn everything off if button is pressed for 10 ms
  if (buttonPressedTime > 0 && externalCircuitState == HIGH && buttonState == HIGH) {
    externalCircuitOff();
    buttonPressedTime = 0;
  }

  //turn everything on if button is pressed for 10 ms
  if (buttonPressedTime > 0 && externalCircuitState == LOW  && buttonState == HIGH) {
    externalCircuitOn();
    buttonPressedTime = 0;
    startTime = millis();
    currentTime = millis();
    vsyncDetect = vsync_off_time;
  }

}//end loop



void handleSerial(char incoming) {
  /*
                        ROTATE    PIN_CUSHION   PWR_OFF  PRINT_INFO
     MOVE   SQUISH       t y         u i           o         p
      w       r        
     a s     d f         g h         j k
      z       c        CONTRAST   BRIGHTNESS
      
            x   v        b n
           PARALLEL    KEYSTONE           
     

    a = move left
    s = move right
    w = move up
    z = move down
    
    d = skinnier
    f = fatter
    r = taller
    c = shorter
    
    g = contrast down
    h = contrast up
    
    j = brightness down
    k = brightness up
    
    x = paralellogram left
    v = paralellogram right
    
    b = keystone top
    n = keystone bottom
      
    t = rotate left
    y = rotate right
    
    u = pin cushion out
    i = pin cushion in

  */


  // Echo received text back to the sender.
  Serial.print(incoming);
  Serial.print(" ");
  
  int index = -1;
  bool increment = true;
  switch (incoming) {
    case 'a'://move left
      //moveHorizontal(+1);
      index = IVAD_SETTING_HORIZONTAL_POS;
      break;
    case 's'://move right
      //moveHorizontal(-1);
      index = IVAD_SETTING_HORIZONTAL_POS;
      increment = false;
      break;
    case 'w'://move up
      //moveVertical(-1);
      index = IVAD_SETTING_VERTICAL_POS;
      increment = false;
      break;
    case 'z'://move down
      //moveVertical(+1);
      index = IVAD_SETTING_VERTICAL_POS;
      break;
    case 'd'://make skinnier
      //changeWidth(+1);
      index = IVAD_SETTING_WIDTH;
      break;
    case 'f'://make fatter
      //changeWidth(-1);
      index = IVAD_SETTING_WIDTH;
      increment = false;
      break;
    case 'r'://make taller
      //changeHeight(+1);
      index = IVAD_SETTING_HEIGHT;
      break;
    case 'c'://make shorter
      //changeHeight(-1);
      index = IVAD_SETTING_HEIGHT;
      increment = false;
      break;
    case 'g'://decrease contrast
      //changeContrast(-1);
      index = IVAD_SETTING_CONTRAST;
      increment = false;
      break;
    case 'h'://increase contrast
      //changeContrast(+1);
      index = IVAD_SETTING_CONTRAST;
      break;
    case 'j'://decrease brightness
      //changeBrightness(-1);
      index = IVAD_SETTING_BRIGHTNESS;
      increment = false;
      break;
    case 'k'://increase brightness
      // changeBrightness(+1);
      index = IVAD_SETTING_BRIGHTNESS;
      break;
    case 'x'://tilt paralellogram left
      //changeParallelogram(+1);
      index = IVAD_SETTING_PARALLELOGRAM;
      break;
    case 'v'://tilt paralellogram right
      //changeParallelogram(-1);
      index = IVAD_SETTING_PARALLELOGRAM;
      increment = false;
      break;
    case 'b'://keystone pinch top
      //changeKeystone(-1);
      index = IVAD_SETTING_KEYSTONE;
      increment = false;
      break;
    case 'n'://keystone pinch bottom
      //changeKeystone(+1);
      index = IVAD_SETTING_KEYSTONE;
      break;
    case 't'://rotate left
      //changeRotation(+1);
      index = IVAD_SETTING_ROTATION;
      break;
    case 'y'://rotate right
      //changeRotation(-1);
      index = IVAD_SETTING_ROTATION;
      increment = false;
      break;
    case 'u'://pincushion pull corners out
      //changePincushion(-1);
      index = IVAD_SETTING_PINCUSHION;
      increment = false;
      break;
    case 'i'://pincushion pull corners in
      //changePincushion(+1);
      index = IVAD_SETTING_PINCUSHION;
      break;
    case 'p':
      printCurrentSettings();
      break;
    case 'o'://power off
      if ( externalCircuitState == HIGH ) {
        externalCircuitOff();
      }//end if
      break;
    default:
      Serial.println("unknown command");
      break;
  }

  if (index > -1) {
    int val = CURRENT_CONFIG[index];
    if (increment) {
      val++;
    }
    else
    {
      val--;
    }
    
    ivad_change_setting(index, val);

    char hexChar[2];
    sprintf(hexChar, "%02X", val);
    Serial.print(hexChar); //report changed value to user
    Serial.println("");
  }//end if
  else {
    Serial.println("");
  }
  
}//end handleSerial

void printCurrentSettings() {
  Serial.println("Serial commands for controlling the CRT Screen:");
  Serial.println("");
  Serial.println("                    ROTATE    PIN_CUSHION   PWR_OFF  PRINT_INFO");
  Serial.println(" MOVE   SQUISH       t y         u i           o         p     ");
  Serial.println("  w       r                                                    ");
  Serial.println(" a s     d f         g h         j k                           ");
  Serial.println("  z       c        CONTRAST   BRIGHTNESS                       ");
  Serial.println("                                                               ");
  Serial.println("        x   v        b n                                       ");
  Serial.println("       PARALLEL    KEYSTONE                                    ");
  Serial.println("");
  Serial.println("Code for displaying current settings does not exist.");
}

void writeToIvad(byte address, byte message) {
  softWire.beginTransmission(address);
  softWire.write(message);
  softWire.endTransmission();

}//end method

void writeToIvad(byte address, byte message1, byte message2) {
  softWire.beginTransmission(address);
  softWire.write(message1);
  softWire.write(message2);
  softWire.endTransmission();

}//end method

void  readFromIvad(byte address, byte bytes) {
  char buf[bytes + 1];
  byte bytesRead = 0;
  softWire.requestFrom(address, bytes);
  while (softWire.available())
  {
    char c = softWire.read();
    buf[bytesRead++] = c;
  }
  buf[bytesRead] = '\0';

}//end method


void initIvadBoard() {

//  //init sequence 2 <---this is the one that works well with my iMac G3, Rocky Hill
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_WIDTH, 0x00);
//  readFromIvad(IVAD_REGISTER_PROPERTY, 1);
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_VERTICAL_POS, 0x00);
//  writeToIvad( 0x53, 0x33);
//  readFromIvad(0x53, 1);
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_WIDTH, 0x0B);
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_CONTRAST, 0x00); //setting contrast to 0x00 seems to turn something on.
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_HEIGHT, 0xE4);
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_ROTATION, 0xC9);
//  writeToIvad( 0x53, 0x00);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x0A);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x14);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x1E);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x28);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x32);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x3C);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x46);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x50);
//  readFromIvad(0x53, 10);
//  writeToIvad( 0x53, 0x5A);
//  readFromIvad(0x53, 2);
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_RED_CUTOFF,         VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_RED_CUTOFF ]        );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_GREEN_CUTOFF,       VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_GREEN_CUTOFF ]      );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_BLUE_CUTOFF,        VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_BLUE_CUTOFF ]       ); 
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_HORIZONTAL_POS,     VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_HORIZONTAL_POS ]    );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_HEIGHT,             VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_HEIGHT ]            );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_VERTICAL_POS,       VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_VERTICAL_POS ]      );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_S_CORRECTION,       VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_S_CORRECTION ]      );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_KEYSTONE,           VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_KEYSTONE ]          );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_PINCUSHION,         VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_PINCUSHION ]        );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_WIDTH,              VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_WIDTH ]             );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_PINCUSHION_BALANCE, VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_PINCUSHION_BALANCE ]);
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_PARALLELOGRAM,      VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_PARALLELOGRAM ]     );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_RESERVED6,          VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_RESERVED6 ]         ); // brightness
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_BRIGHTNESS,         VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_BRIGHTNESS ]        );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_ROTATION,           VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_ROTATION ]          );
//  writeToIvad( IVAD_REGISTER_PROPERTY, IVAD_SETTING_CONTRAST,           VIDEO_CONFIG_DEFAULT[ IVAD_SETTING_CONTRAST ]          );



     //provided by anothere
      writeToIvad( 0x46,0x13,0x00);
      writeToIvad(0x46,0x13,0x00);
      readFromIvad(0x46,1);
      writeToIvad(0x46,0x09,0x00);
      writeToIvad(0x53,0x33);
      readFromIvad(0x53,1);
      writeToIvad(0x46,0x13,0x0b);
      writeToIvad(0x46,0x00,0x00);
      writeToIvad(0x46,0x08,0xe4);
      writeToIvad(0x46,0x12,0xc9);
      writeToIvad(0x53,0x00);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x0a);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x14);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x1e);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x28);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x32);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x3c);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x46);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x50);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x5a);
      readFromIvad(0x53,10);
      writeToIvad(0x46,0x01,0x82);
      writeToIvad(0x46,0x02,0x82);
      writeToIvad(0x46,0x03,0x82);
      writeToIvad(0x46,0x04,0xa0);
      writeToIvad(0x46,0x05,0xa0);
      writeToIvad(0x46,0x06,0xa0);
      writeToIvad(0x46,0x07,0xad);
      writeToIvad(0x46,0x08,0xe4);
      writeToIvad(0x46,0x09,0x3d);
      writeToIvad(0x46,0x0a,0x9e);
      writeToIvad(0x46,0x0b,0xb4);
      writeToIvad(0x46,0x0c,0xc4);
      writeToIvad(0x46,0x0d,0x27);
      writeToIvad(0x46,0x0e,0xbf);
      writeToIvad(0x46,0x0f,0xc0);
      writeToIvad(0x46,0x10,0x40);
      writeToIvad(0x46,0x11,0x0a);
      writeToIvad(0x46,0x12,0x5b);
      writeToIvad(0x46,0x00,0xff);
      writeToIvad(0x53,0x00);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x10);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x20);
      readFromIvad(0x53,10);
      writeToIvad(0x53,0x30);
      readFromIvad(0x53,10);
      writeToIvad(0x46,0x11,0x05);
      writeToIvad(0x46,0x00,0xff);
      writeToIvad(0x46,0x00,0x00);
      writeToIvad(0x46,0x07,0xb1);
      writeToIvad(0x46,0x0d,0x10);
      writeToIvad(0x46,0x0c,0xc7);
      writeToIvad(0x46,0x09,0x4a);
      writeToIvad(0x46,0x08,0xea);
      writeToIvad(0x46,0x0f,0xc0);
      writeToIvad(0x46,0x0b,0xae);
      writeToIvad(0x46,0x12,0x5b);
      writeToIvad(0x46,0x00,0xff);
      writeToIvad(0x46,0x11,0x05);
      writeToIvad(0x46,0x00,0xff);
      writeToIvad(0x46,0x10,0x40);
      writeToIvad(0x46,0x06,0xa0);
      writeToIvad(0x46,0x05,0xa0);
      writeToIvad(0x46,0x04,0xa0);
      writeToIvad(0x46,0x03,0x82);
      writeToIvad(0x46,0x02,0x82);
      writeToIvad(0x46,0x01,0x82);
      writeToIvad(0x46,0x11,0x05);
      writeToIvad(0x46,0x00,0xff);
      writeToIvad(0x46,0x11,0x05);
      writeToIvad(0x46,0x00,0xff);
      writeToIvad(0x46,0x10,0x40);
      writeToIvad(0x46,0x06,0xa0);
      writeToIvad(0x46,0x05,0xa0);
      writeToIvad(0x46,0x04,0xa0);
      writeToIvad(0x46,0x03,0x82);
      writeToIvad(0x46,0x02,0x82);
      writeToIvad(0x46,0x01,0x82);
      writeToIvad(0x46,0x11,0x05);
      writeToIvad(0x46,0x00,0xff);
}


void solid_state_relayOn() {
  digitalWrite(solid_state_relay_Pin, HIGH);

}

void solid_state_relayOff() {
  digitalWrite(solid_state_relay_Pin, LOW);

}




//these are probably too much but they are here in case I would lke to add more stuff to turn on and off
void externalCircuitOn() {
  solid_state_relayOn();
  delay(500);
  initIvadBoard();
  settings_load();
  ivad_write_settings();
  externalCircuitState = HIGH;


}

void externalCircuitOff() {
  solid_state_relayOff();
  externalCircuitState = LOW;

}



// function that executes whenever data is requested by master
// this function is registered as an event.
void requestEvent() {
  //delay(500);
  Wire.write(edid, 128);
  Serial.println("VGA I2C Request recieved, so sending EDID.");
  printEDID();

  Serial.println("");
}//end method
// function that executes whenever data is received by the slave


void receiveData(byte byteCount) {
  while (Wire.available()) {
    data = Wire.read();
  }
}


/*
  This is my implementation of oshimai's communications protocol.
  Clearly I borrowed heavily from his sketch and I purposely
  kept the same variable names to make it easier to port any
  changes he might make to his sletch. There are parts I didn't
  bother to port, verifying the checksum for example because of
  time constraints. I might implement the rest in the future.

*/
void serial_processing()
{

  byte b ;

  if (Serial.available()) {
    
    do
    {
      b = Serial.read();
      SERIAL_BUFFER[SERIAL_BUFFER_DATA_SIZE++] = b;
    }
    while (Serial.available() &&  b != SERIAL_EOL_MARKER);


    //call other serial handler
    if (SERIAL_BUFFER_DATA_SIZE != 9)
    {
      if (SERIAL_BUFFER[0] != 0x07) {
        SERIAL_BUFFER_DATA_SIZE = 0;
        handleSerial((char)b);
      }
      return;

    }


    SERIAL_BUFFER_DATA_SIZE = 0;

    byte id = SERIAL_BUFFER[1];
    byte cmd = SERIAL_BUFFER[2];
    byte valA = SERIAL_BUFFER[3];
    byte valB = SERIAL_BUFFER[4];

    switch (cmd)
    {

      case 0x01: // Get EEPROM Version
        {
          byte ret[8] { 0x06, id, 0x01, CONFIG_EEPROM_VERSION, 0x03, 0xFF, 0x04, SERIAL_EOL_MARKER };
          ret[5] = checksum(ret, 5);
          Serial.write(ret, 8);
        }
        break;

      case 0x02: // Dump SRAM Config
        {
          byte ret[7 + CONFIG_EEPROM_SLOTS];
          ret[0] = 0x06;
          ret[1] = SERIAL_BUFFER[1];
          ret[2] = CONFIG_EEPROM_SLOTS;

          for (int i = 0; i < CONFIG_EEPROM_SLOTS; i++)
            ret[3 + i] = CURRENT_CONFIG[i];

          ret[2 + CONFIG_EEPROM_SLOTS + 1] = 0x03;
          ret[2 + CONFIG_EEPROM_SLOTS + 2] = checksum(ret, 2 + CONFIG_EEPROM_SLOTS + 1 + 1);
          ret[2 + CONFIG_EEPROM_SLOTS + 3] = 0x04;
          ret[2 + CONFIG_EEPROM_SLOTS + 4] = SERIAL_EOL_MARKER;
          Serial.write(ret, 7 + CONFIG_EEPROM_SLOTS);
        }
        break;

      case 0x03: // IVAD Change Setting
        {
          ivad_change_setting(valA, valB);
          byte ret[7] { 0x06, id, 0x00, 0x03, 0xFF, 0x04, SERIAL_EOL_MARKER };
          ret[4] = checksum(ret, 4);
          Serial.write(ret, 7);
        }
        break;

      case 0x04: // IVAD Reset from EEPROM
        {
          settings_load();
          byte ret[7] { 0x06, id, 0x00, 0x03, 0xFF, 0x04, SERIAL_EOL_MARKER };
          ret[4] = checksum(ret, 4);
          Serial.write(ret, 7);
        }
        break;

      case 0x05: // EEPROM Reset to Default
        {
          settings_reset_default();
          settings_store();
          settings_load();
          ivad_write_settings();
          byte ret[7] { 0x06, id, 0x00, 0x03, 0xFF, 0x04, SERIAL_EOL_MARKER };
          ret[4] = checksum(ret, 4);
          Serial.write(ret, 7);
        }
        break;

      case 0x06: // Write SRAM to EEPROM
        {
          settings_store();
          byte ret[7] { 0x06, id, 0x00, 0x03, 0xFF, 0x04, SERIAL_EOL_MARKER };
          ret[4] = checksum(ret, 4);
          Serial.write(ret, 7);
        }
    }//end switch

    SERIAL_BUFFER[1] = 0xFF;

  }

}//end if









//===================================



byte checksum(const byte arr[], const int len)
{
  int sum = 1; // Checksum may never be 0.

  for (int i = 0; i < len; i++)
    sum += arr[i];

  byte ret = 256 - (sum % 256);

  return ret;
}




int ivad_change_setting(const int ivad_setting,  const byte value)
{

  CURRENT_CONFIG[ivad_setting] = value;

  if (CURRENT_CONFIG[ivad_setting] < VIDEO_CONFIG_MIN[ivad_setting]) CURRENT_CONFIG[ivad_setting] = VIDEO_CONFIG_MIN[ivad_setting];
  if (CURRENT_CONFIG[ivad_setting] > VIDEO_CONFIG_MAX[ivad_setting]) CURRENT_CONFIG[ivad_setting] = VIDEO_CONFIG_MAX[ivad_setting];

  writeToIvad(IVAD_REGISTER_PROPERTY, ivad_setting, CURRENT_CONFIG[ivad_setting]);
  CURRENT_CONFIG[CONFIG_OFFSET_CHECKSUM] = checksum(CURRENT_CONFIG, CONFIG_EEPROM_SLOTS - 1);

  return 0;
}



/*
  This function loads the monitor property values from the EEPROM
  into variables.
*/
void settings_load()
{
  Serial.println("loading settings from eeprom");
  // Set something so a checksum mismatch can trigger if there's nothing in the EEPROM.

  for (byte eeprom_memory_offset = 0 ; eeprom_memory_offset < CONFIG_EEPROM_SLOTS ; eeprom_memory_offset++) {
    CURRENT_CONFIG[eeprom_memory_offset] = EEPROMwl.read(eeprom_memory_offset);
  }//end for

  byte loaded_checksum = CURRENT_CONFIG[CONFIG_OFFSET_CHECKSUM];
  byte expected_checksum = checksum(CURRENT_CONFIG, CONFIG_EEPROM_SLOTS - 1);

  if (loaded_checksum != expected_checksum)
  {
    //settings_reset_default();
    settings_store();


  }


}

void ivad_write_settings()
{

  for (int IVAD_SETTING = 0 ; IVAD_SETTING < IVAD_SETTING_END ;  IVAD_SETTING ++ )
  {
    writeToIvad(IVAD_REGISTER_PROPERTY, IVAD_SETTING, CURRENT_CONFIG[IVAD_SETTING]);
  }

}



void settings_store()
{
  Serial.println("saving settings to eeprom");
  //compute current config checksum and store it.
  byte current_config_checksum = checksum(CURRENT_CONFIG, CONFIG_EEPROM_SLOTS - 1);

  CURRENT_CONFIG[CONFIG_OFFSET_CHECKSUM] = current_config_checksum;

  for (byte eeprom_memory_offset = 0 ; eeprom_memory_offset < CONFIG_EEPROM_SLOTS ; eeprom_memory_offset++) {
    EEPROMwl.update(eeprom_memory_offset, CURRENT_CONFIG[eeprom_memory_offset]);
  }//end for

}


void settings_reset_default()
{
  Serial.println("resetting settings to default");

  //compute current config checksum and store it.
  byte current_config_checksum = checksum(CURRENT_CONFIG, CONFIG_EEPROM_SLOTS - 1);

  CURRENT_CONFIG[CONFIG_OFFSET_CHECKSUM] = current_config_checksum;

  for (byte eeprom_memory_offset = 0 ; eeprom_memory_offset < CONFIG_EEPROM_SLOTS ; eeprom_memory_offset++) {
    CURRENT_CONFIG[eeprom_memory_offset] = VIDEO_CONFIG_DEFAULT[eeprom_memory_offset];
  }//end for

}

void printEDID()
{
  Serial.println("*** EDID ***");
  for(int i=0; i<sizeof(edid); i++){
    printHexByte(edid[i]);
  }
  Serial.println("");
}

void printHexByte(uint8_t num) {
  char hexChar[2];

  sprintf(hexChar, "%02X", num);
  Serial.print(hexChar);
}