
#include <Arduino.h>

//#include <TFT_eSPI_Setups/Setup60_RP2040_ST7735.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <mcp_can.h>
#include <vesc_can_bus_arduino.h>

TFT_eSPI tft = TFT_eSPI(); 
CAN can;             // get torque sensor data, throttle for now

#define CAN0_INT 11                              // Set INT to pin 2

bool print_realtime_data = true;
long last_print_data;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(2);
  tft.setTextSize(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);
  // initialize the digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TFT_BL,OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  //delay(3000);
  Serial.println("Let's begin..");
  if(can.initialize() == CAN_OK){
    tft.print("MCP2515 Initialized Successfully! ");
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else{
    tft.print("Error Initializing MCP2515.. ");
    Serial.println("Error Initializing MCP2515...");  
    // digitalWrite(TFT_BL, LOW);
  }
  //delay(3000);
  
}

// the loop routine runs over and over again forever:
void loop() {
  static uint i = 0;
  static bool up_down=1;
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    can.spin();
    if (millis() - last_print_data > 200)
    {
      if(up_down)
      {
        i++;
      }
      else 
      {
        i--;
      }

      if (i >= 100) 
      {
        i = 100; 
        up_down=0;
      }
      if (i == 0)
      {
        up_down = 1;
      }
      float cmd = float(i)*100.0;
      can.vesc_set_erpm(cmd); //2 amps of current

      tft.setCursor(0,0);
      //tft.fillScreen(TFT_BLACK);
      tft.print(i,DEC); tft.print("   "); tft.print(cmd); tft.print("   \n");
      tft.print("erpm = "); tft.print(can.erpm); tft.print("   \n");
      tft.print("inpVoltage = "); tft.print(can.inpVoltage); tft.print("   \n");
      tft.print("dutyCycleNow = "); tft.print(can.dutyCycleNow); tft.print("   \n");
      tft.print("avgInputCurrent = "); tft.print(can.avgInputCurrent); tft.print("   \n");
      tft.print("avgMotorCurrent = "); tft.print(can.avgMotorCurrent); tft.print("   \n");
      tft.print("tempFET = "); tft.print(can.tempFET); tft.print("   \n");
      tft.print("tempMotor = ");tft.print(can.tempMotor); tft.print("   \n");

      Serial.print(can.erpm); Serial.print(',');
      Serial.print(can.inpVoltage); Serial.print(',');
      Serial.print(can.dutyCycleNow); Serial.print(',');
      Serial.print(can.avgInputCurrent); Serial.print(',');
      Serial.print(can.avgMotorCurrent); Serial.print(',');
      Serial.print(can.tempFET); Serial.print(',');
      Serial.println(can.tempMotor);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      last_print_data = millis();
    }
  }
}

/*

  tft.setTextColor(TFT_WHITE);
  int pixel_height = 30;
  tft.setTextFont(1);
  tft.setCursor(10,pixel_height);
  pixel_height+=8;
  tft.print("Hello World! 1");
  tft.setTextFont(2);
  tft.setCursor(10,pixel_height);
  pixel_height+=16;
  tft.print("Hello World! 2");
  tft.setTextFont(4);
  tft.setCursor(10,pixel_height);
  pixel_height+=26;
  tft.print("Hello World! 4");
  tft.setTextFont(6);
  tft.setCursor(10,pixel_height);
  pixel_height+=48;
  tft.setTextColor(TFT_RED);
  tft.print("6");
  tft.setTextFont(7);
  tft.setCursor(10,pixel_height);
  pixel_height+=48;
  tft.setTextColor(TFT_GREEN);
  tft.print("7");
  tft.setTextFont(8);
  tft.setCursor(10,pixel_height);
  pixel_height+=75;
  tft.setTextColor(TFT_BLUE);
  tft.print("8");
  */

/*
#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
// The Pico has two SPI peripherals, SPI and SPI1. Either (or both) can be used.
// The are no default pin assignments so they must be set explicitly.
// Testing was done with Earle Philhower's arduino-pico core:
// https://github.com/earlephilhower/arduino-pico
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK  = 14 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 15 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12 ; // SDO output of MCP2515 (adapt to your design)

static const byte MCP2515_CS   = 13 ;  // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT  = 11 ;  // INT output of MCP2515 (adapt to your design)

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI1, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
  //--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  //--- Start serial
  Serial.begin (115200) ;
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  //--- There are no default SPI1 pins so they must be explicitly assigned
  SPI1.setSCK (MCP2515_SCK);
  SPI1.setTX (MCP2515_MOSI);
  SPI1.setRX (MCP2515_MISO);
  SPI1.setCS (MCP2515_CS);
  //--- Begin SPI1
  SPI1.begin () ;
  //--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 500UL * 1000UL) ; // CAN bit rate 500 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select loopback mode
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//----------------------------------------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    frame.ext = true ;
    frame.id = 0x1FFFFFFF ;
    frame.len = 8 ;
    frame.data [0] = 0x11 ;
    frame.data [1] = 0x22 ;
    frame.data [2] = 0x33 ;
    frame.data [3] = 0x44 ;
    frame.data [4] = 0x55 ;
    frame.data [5] = 0x66 ;
    frame.data [6] = 0x77 ;
    frame.data [7] = 0x88 ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentFrameCount) ;
    }else{
      Serial.println ("Send failure") ;
    }
  }
  if (can.receive (frame)) {
    gReceivedFrameCount ++ ;
    Serial.print ("  id: ");Serial.println (frame.id,HEX);
    Serial.print ("  ext: ");Serial.println (frame.ext);
    Serial.print ("  rtr: ");Serial.println (frame.rtr);
    Serial.print ("  len: ");Serial.println (frame.len);
    Serial.print ("  data: ");
    for(int x=0;x<frame.len;x++) {
      Serial.print (frame.data[x],HEX); Serial.print(":");
    }
    Serial.println ("");
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
}
*/