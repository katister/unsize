/*********************************************************************
Generate Product Develop Studio
Project: SizeU
Spring 2017
Christopher Tannock - Product Engineer
**********************************************************************
Header File Info
*********************************************************************/
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
#define BLUE_LED 8
#define GREEN_LED 7
#define RED_LED 6
#define INTERNAL_LED_PIN 13
#define BUTTON_PIN 4
#define ROTARY_SWITCH A0
#define ROTARY_ENCODER_1 A4
#define ROTARY_ENCODER_2 A5

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
*/
/**************************************************************************/

// variables will change.
int buttonState = 0;
int lastButtonState = 0;
int rotarySwitchState = 0;

// JT
int count;
int Rotations;
int resolutionx2=15;      //using 2x the resolution in degrees to keep data in int type
int Angle;
float inchpoly,inchpwr,inchlin,inchround;

void setup(void)
{ 
  configureLights();
  configureEncoder();
  pinMode(BUTTON_PIN, INPUT);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(ROTARY_SWITCH, INPUT);
  digitalWrite(ROTARY_SWITCH, HIGH);
  
  Serial.begin(9600);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Generate Hardware Prototype Demo"));

  BTLEserial.setDeviceName("SizeU");

  BTLEserial.begin();
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
*/
/**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{ 

  runEncoder();
}

void useless(void)
{
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();
  
  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();



  if (status != laststatus) {
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    bluetoothConnected();

    // ALL OF THIS IS NOT NECESSARY AS WE ARE ONLY SENDING DATA AND NOT LISTENING FOR DATA
    // Lets see if there's any data for us!
    // if (BTLEserial.available()) {
    //  Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    // }
    // OK while we still have something to read, get a character and print it out
    // while (BTLEserial.available()) {
    //  char c = BTLEserial.read();
    //  Serial.print(c);
    // }

    // We need to convert the line to bytes
    rotarySwitchState = getRotaryState();
     
    buttonState = digitalRead(BUTTON_PIN);
    String s = determineMeasurementSize(rotarySwitchState);
    uint8_t sendbuffer[70];
    s.getBytes(sendbuffer, 70);
    char sendbuffersize = min(70, s.length());
    
    if (buttonState != lastButtonState) {
      if (buttonState == HIGH) {
        BTLEserial.write(sendbuffer, sendbuffersize);
      }
      delay(50);
    }
    lastButtonState = buttonState;
 

    /*
    // Next up, see if we have any data to get from the Serial console
    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
    */
  }

  if (status == ACI_EVT_DEVICE_STARTED) {
    bluetoothAdvertising();
  }

  if (status == ACI_EVT_DISCONNECTED) {

  }
  
}
/*********************************************************************
Bluetooth nRF8001 Functions
*********************************************************************/
aci_evt_opcode_t getConnectionState(void)
{
  aci_evt_opcode_t status = BTLEserial.getState();
}
/*********************************************************************
RGB LED Functions
*********************************************************************/
void configureLights(void)
{
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
}

void lightsOff(void)
{
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
}

void bluetoothConnected(void)
{
  digitalWrite(BLUE_LED, HIGH);
}

void bluetoothAdvertising(void)
{
  digitalWrite(BLUE_LED, HIGH);
  delay(1000);
  digitalWrite(BLUE_LED, LOW);
  delay(1000);
}

void successfulSend(void)
{
  lightsOff();
  digitalWrite(GREEN_LED, HIGH);
  delay(200);
  digitalWrite(GREEN_LED, LOW);
  delay(200);
  digitalWrite(GREEN_LED, HIGH);
  delay(200);
  digitalWrite(GREEN_LED, LOW);
}

void failedSend(void)
{
  lightsOff();
  digitalWrite(RED_LED, HIGH);
  delay(200);
  digitalWrite(RED_LED, LOW);
  delay(200);
  digitalWrite(RED_LED, HIGH);
  delay(200);
  digitalWrite(RED_LED, LOW);
}
/*********************************************************************
Rotary Switch Functions
*********************************************************************/
int getRotaryState(void)
{
  int mode=0;
  int R= analogRead(A0);
  Serial.println(R);
  for(int i=0;i<140;i+=20){
    if((i-20)<R&&R<i){
      mode=i/18;
      delay(100);
    }
  }  
  delay(100);
  return mode;
}

String determineMeasurementSize(int rotaryMode)
{
  String currentState;
  if(rotaryMode == 1)
  {
    currentState = "Neck: ";
  }
  else if(rotaryMode == 2)
  {
    currentState = "Chest: ";
  }
  else if(rotaryMode == 3)
  {
    currentState = "Sleeve: ";
  }
  else if(rotaryMode == 4)
  {
    currentState = "Waist: ";
  }
  else if(rotaryMode == 5)
  {
    currentState = "Hip: ";
  }
  else if(rotaryMode == 6)
  {
    currentState = "Inseam: ";
  }
  else
  {
    currentState = "Error Unknown State!";
  }

 return currentState;
}

/*********************************************************************
Rotary Encoder Functions
*********************************************************************/
void configureEncoder(void){
  pinMode(ROTARY_ENCODER_1,INPUT);
  pinMode(ROTARY_ENCODER_2,INPUT);
  digitalWrite(ROTARY_ENCODER_1,HIGH);
  digitalWrite(ROTARY_ENCODER_2,HIGH);
}

void ReadEncoder(int &rotate, int& buttonPress){    //register encoder state
  rotate=2*digitalRead(ROTARY_ENCODER_1)+digitalRead(ROTARY_ENCODER_2);
  //buttonPress=digitalRead(pinButton);
}

void runEncoder(void)
{
  int Position, Press;
  int Direction=0;
  //ReadEncoder(Position, Press);
  int rotate=2*digitalRead(ROTARY_ENCODER_1)+digitalRead(ROTARY_ENCODER_2);
  while (!Serial.available()){
    int Position2, Press2;
    do{
      //ReadEncoder(Position2, Press2);
      rotate=2*digitalRead(ROTARY_ENCODER_1)+digitalRead(ROTARY_ENCODER_2);
    } while((Position2==Position)&&(Press2==Press));
    if(Position2!=Position){
      int Direction=((Position == 0) && (Position2 == 1)) ||
        ((Position == 1) && (Position2 == 3)) ||
        ((Position == 3) && (Position2 == 2)) ||
        ((Position == 2) && (Position2 == 0));         
      if(Direction==1){
        count++;
      }      
     else{
        count-- ;
      }
      inchpoly=-0.00002*pow(count,2)+.1571*count-.0947;
      inchpwr=.1525*pow(count,1.0007);
      inchlin=.1516*count+.1965;
      Serial.println(inchpoly);
    if (Press2 != Press)
    {
      Serial.println(Press ? "Press" : "Release - Reset");    //reset the counter when button is pressed
      count=0;
    }
    
    Position = Position2;
    Press = Press2;
    }
  }
}
  




