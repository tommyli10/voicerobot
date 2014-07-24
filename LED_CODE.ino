#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #include "Platform.h"
  #include "SoftwareSerial.h"
#ifndef CDC_ENABLED
  // Shield Jumper on SW
  SoftwareSerial port(12,13);
#else
  // Shield Jumper on HW (for Leonardo)
  #define port Serial1
#endif
#else // Arduino 0022 - use modified NewSoftSerial
  #include "WProgram.h"
  #include "NewSoftSerial.h"
  NewSoftSerial port(12,13);
#endif

#include "EasyVR.h"

EasyVR easyvr(port);

#define SND_BEEP                     1

//Groups and Commands
enum Groups
{
  GROUP_0  = 0,
  GROUP_1  = 1,
};

enum Group0 
{
  G0_HELLO = 0,
};

enum Group1 
{
  G1_ON = 0,
  G1_OFF = 1,
};


EasyVRBridge bridge;

int8_t group, idx;

void setup()
{
    pinMode(8, OUTPUT); // For an LED to go 
  digitalWrite(8, LOW);  // initializing LED to be off
#ifndef CDC_ENABLED
  // bridge mode?
  if (bridge.check())
  {
    cli();
    bridge.loop(0, 1, 12, 13);
  }
  // run normally
  Serial.begin(9600);
  Serial.println("Bridge not started!");
#else
  // bridge mode?
  if (bridge.check())
  {
    port.begin(9600);
    bridge.loop(port);
  }
  Serial.println("Bridge connection aborted!");
#endif
  port.begin(9600);

  while (!easyvr.detect())
  {
    Serial.println("EasyVR not detected!");
    delay(1000);
  }

  easyvr.setPinOutput(EasyVR::IO1, LOW);
  Serial.println("EasyVR detected!");
  easyvr.setTimeout(5);
  easyvr.setLanguage(0);
  group = EasyVR::TRIGGER;
//<-- start group (customize)
}


void loop()
{
  easyvr.setPinOutput(EasyVR::IO1, HIGH); // LED on (listening)
 pinMode(EasyVR::IO1, INPUT);
  pinMode(8, OUTPUT);
  // digitalWrite(8,LOW);
  
  Serial.print("Say a command in Group ");
  easyvr.playSound(SND_BEEP,EasyVR::VOL_FULL);
  Serial.println(group);
  easyvr.recognizeCommand(group);

  do
  {
    // can do some processing while waiting for a spoken command
  }
  while (!easyvr.hasFinished());
  
  easyvr.setPinOutput(EasyVR::IO1, LOW); // LED off

  idx = easyvr.getWord();
  if (idx >= 0)
  {
    // built-in trigger (ROBOT)
    // group = GROUP_X; <-- jump to another group X
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    // print debug message
    uint8_t train = 0;
    char name[32];
    Serial.print("Command: ");
    Serial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train))
    {
      Serial.print(" = ");
      Serial.println(name);
    }
    else
      Serial.println();
    easyvr.playSound(0, EasyVR::VOL_FULL);
    // perform some action
    action();
  }
  else // errors or timeout
  {
    if (easyvr.isTimeout())
      Serial.println("Timed out, try again...");
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      Serial.print("Error ");
      Serial.println(err, HEX);
    }
  }
}

void action()
{
    switch (group)
    {
    case GROUP_0:
      switch (idx)
      {
      case G0_HELLO:
        // write your action code here
        Serial.println("Command recieved");
        easyvr.playSound(SND_BEEP,EasyVR::VOL_FULL);
       group = GROUP_1; // group = GROUP_X; <-- or jump to another group X for composite commands
        break;
      }
      break;
    case GROUP_1:
      switch (idx)
      {
      case G1_ON:
        pinMode(8, OUTPUT);
        digitalWrite(8, HIGH); // LED On
  
        Serial.println("LED On n G1On ");
       // write your action code here
        // group = GROUP_X; <-- or jump to another group X for composite commands
      break;
       case G1_OFF:
        pinMode(8, OUTPUT);
        digitalWrite(8, LOW); // LED Off
       Serial.println("LED OFF n G1On");
        // write your action code here
        // group = GROUP_X; <-- or jump to another group X for composite commands
        break;
      }
      break;
    }
}
