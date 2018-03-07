// **********************************************************************************************************
// Moteino gateway/base sketch that works with Moteinos equipped with RFM69W/RFM69HW/RFM69CW/RFM69HCW
// This is a basic gateway sketch that receives packets from end node Moteinos, formats them as ASCII strings
//      with the end node [ID] and passes them to Pi/host computer via serial port
//     (ex: "messageFromNode" from node 123 gets passed to serial as "[123] messageFromNode")
// It also listens to serial messages that should be sent to listening end nodes
//     (ex: "123:messageToNode" sends "messageToNode" to node 123)
// Make sure to adjust the settings to match your transceiver settings (frequency, HW etc).
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                  
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/RFM69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE (www.arduino.cc)

//****************************************************************************************************************
//**** IMPORTANT RADIO SETTINGS - YOU MUST CHANGE/CONFIGURE TO MATCH YOUR HARDWARE TRANSCEIVER CONFIGURATION! ****
//****************************************************************************************************************
#define NODEID          1 //the ID of this node
#define NETWORKID     100 //the network ID of all nodes this node listens/talks to
#define FREQUENCY     RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY    "sampleEncryptKey" //identical 16 characters/bytes on all nodes, not more not less!
//#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ACK_TIME       30  // # of ms to wait for an ack packet
//*****************************************************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75  //target RSSI for RFM69_ATC (recommended > -80)
//*****************************************************************************************************************************
// Serial baud rate must match your Pi/host computer serial port baud rate!
#undef SERIAL_EN     //comment out if you don't want any serial verbose output
#define SERIAL_BAUD  115200
//*****************************************************************************************************************************

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef SERIAL_EN
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit Windbond FlashMEM chip

String inputString;
bool stringComplete;
void mySerialEvent();

void setup() {

  inputString.reserve(130);
  inputString = "";

  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif
  
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", (int)(radio.getFrequency()/1000000));
  DEBUGln(buff);

  if (flash.initialize())
  {
    DEBUGln("SPI Flash Init OK!");
  }
  else
  {
    DEBUGln("SPI FlashMEM not found (is chip onboard?)");
  }
}

byte ackCount=0;

void printDataAsHex(uint8_t* dataBuffer, int len)
{
    for(int i = 0; i < len; ++i)
    {
      char twoChar[3];

      sprintf(twoChar, "%02x", dataBuffer[i]);
      Serial.print(twoChar);
    }
}

int g_radioSendBufferLen = 0;
uint8_t g_radioSendBuffer[70] = {0};
byte g_radioSendTargetId = 0;

void loop() 
{
  
  mySerialEvent();
  

  // Data received from the radio gets delivered to Serial as 
  // hexidecimal 2 chars per byte no spaces.
  // Serial output format: [senderid][rssi:value][ack-requested][data(5):5A5A5A5A5A]\n
  if (radio.receiveDone())
  {
    pinMode(LED, OUTPUT);
    digitalWrite(LED,HIGH);

    // Copy the data from the radio before transmitting ACK.
    uint8_t radioDataBuffer[70];
    int radioDataLen = radio.DATALEN;
    memcpy(radioDataBuffer,const_cast<const uint8_t*>(&radio.DATA[0]),radioDataLen);

    int rssi = radio.RSSI;
    int senderId = radio.SENDERID;
    bool ackRequested = radio.ACKRequested();
    if (ackRequested)
    {
      radio.sendACK();
    }


    // Send if we have data waiting.
    if (g_radioSendBufferLen != 0 && senderId == g_radioSendTargetId)
    {
      // Seems that a delay is needed here.  The other side must not be ready to receive so soon after sending.
      // Instead of always delay, let it retry sending for longer.
      // Measured time to send and receive ack (with debug builds):
      //     525ms, with retry wait 255ms
      //     324ms, with retry wait 60ms
      //     315ms, with retry wait 150ms
      //     310ms, with default retry wait (40ms)
      //delay(500);
      DEBUG("Radio send len:");DEBUGln(g_radioSendBufferLen);
      #ifdef SERIAL_EN
      unsigned long startTime = millis();
      #endif
      radio.sendWithRetry(g_radioSendTargetId, g_radioSendBuffer, g_radioSendBufferLen,10);
      if (radio.ACK_RECEIVED)
      {
           #ifdef SERIAL_EN
           unsigned long endTime = millis();
           #endif
          DEBUG("ack received ");DEBUGln(endTime-startTime);
      }
     g_radioSendBufferLen = 0;
    }

    // After transmitting ack, write the data to the serial port.
    Serial.print('[');Serial.print(senderId);Serial.print("] ");
    Serial.print("[RSSI:");Serial.print(rssi);Serial.print("]");
    if (ackRequested)
    {
      Serial.print("[ACK-requested]"); 
    }
    Serial.print("[data("); Serial.print(radioDataLen);Serial.print("):");
    printDataAsHex(radioDataBuffer, radioDataLen);
    Serial.print("]\n");
  }

  digitalWrite(LED,LOW);
}

bool copySerialInputToString(String& s)
{
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      //Serial.print("char=");
      //char buf[3];
      //sprintf(buf,"%02x",(int)inChar);
      //Serial.println(buf);
        
      if (isxdigit(inChar) || inChar == ':') 
      {
        s += inChar;
      }

      if (inChar == '\n' || inChar == '\r') 
      {
        if (s.length()>0)
        {
            return true;
        }
      }
    }
    return false;
}

int stringToData(const char* inputString, uint8_t* dataBuffer)
{
    int len = strlen(inputString)/2;
    char twoChar[3];
    twoChar[2] = '\0';
    for(int i = 0; i < len; ++i)
    {

      twoChar[0] = *inputString;
      twoChar[1] = *(inputString+1);
      inputString += 2;

      dataBuffer[i] = strtoul(twoChar, 0, 16);
    }
    return len;
}

void mySerialEvent()
{
   if (Serial.available())
   {
      pinMode(LED, OUTPUT);
      digitalWrite(LED,HIGH);

      // add it to the inputString:
      stringComplete = copySerialInputToString(inputString);

      // if the incoming character is a newline, set a flag so the main loop can
      // do something about it:
   }

   if (stringComplete) 
   {
    //Serial.print("inputString=");
    //Serial.println(inputString.c_str());
    byte targetId = inputString.toInt();
    byte colonIndex = inputString.indexOf(":"); //find position of first colon

    if (targetId > 0)
    {
      inputString = inputString.substring(colonIndex+1); //trim "ID:" if any
    }

    // The string is bytes encoded as hexidecimal, 2 chars per byte, no spaces
    // Translate to bytes before transmission.
    // Radio supports up to 61 bytes.
    
    // BEWARE:  If there is a message queued to send, then this will overwrite the queued message.
    g_radioSendBufferLen = stringToData(inputString.c_str(),g_radioSendBuffer);
    g_radioSendTargetId = targetId;
DEBUG("Message to send, target:");DEBUG(g_radioSendTargetId);DEBUG(", len:");DEBUGln(g_radioSendBufferLen);
    // clear the string:
    inputString = "";
    stringComplete = false;

  // Don't send until we receive something.
    // send over radio
   // radio.send(targetId, sendBuf, sendLen);
   }
}