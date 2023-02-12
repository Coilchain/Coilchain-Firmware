#include "common_inc.h"
#include "bcu_disp.h"

//#include <TFT_eSPI_Setups/Setup60_RP2040_ST7735.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <mcp_can.h>
#include <vesc_can_bus_arduino.h>
#include <math_helper.h>
#define SERIAL_PRINT 0

TFT_eSPI tft = TFT_eSPI(); 
CAN can;             // get torque sensor data, throttle for now

#define CAN0_INT 8                              // Set INT to pin 2
#define LED_GREEN 9
#define ENABLE_12V 7
#define TACH_GPIO 11
#define TORQUE_ADC A0

#define BUT_MID 18

// Max voltage and current input from generator
#define CURRENT_MAX_IN 50
#define VOLTAGE_MAX_IN 60

//207, 460
#define TORQUE_MIN 512
#define TORQUE_MAX 1024
#define CURRENT_MAX_OUT (80*1000)
bool print_realtime_data = true;
long last_print_data;


/*-------------------------------  BCU Display -----------------------------------
a. How to run sample code
1. Enable  OPEN_BCU_DISP_SAMPLE  macro define to use sample code.
2. Before use this sample code, you need change button keys(BUT_UP......) pin map.
3. Build and copy ulf2 firmware to your bcu board.

b. How to use BcuDisp class.
1). Declare BcuDisp class
  |namespace|class type| variable  = |namespace|class type| [initial list]
  BcuDisplay:: BcuDisp     bcu_disp =   BcuDisplay::BcuDisp(param1, param2)

  >param1: TFT_eSPI class instance.
  >param2: {BUT_UP, BUT_DN,BUT_MID, RISING, TFT_BLACK, TFT_WHITE, TFT_VIOLET}
    BUT_UP: button up key.
    BUT_DN: button down key.
    BUT_MID: button middle key.
    RISING: button isr action: RISING|CHANGE|LOW|HIGH|.....
    TFT_BLACK: panel display background color.
    TFT_WHITE: panel display font color.
    TFT_VIOLET: selected params highlight.

  // example
  BcuDisplay::BcuDisp bcu_disp = BcuDisplay::BcuDisp(tft, {BUT_UP, BUT_DN,BUT_MID, RISING, TFT_BLACK, TFT_WHITE, TFT_VIOLET});

2) Init bcu display
  bcu_disp.init();

3) Update display value
  bcu_disp.collect(param1, param2)
  >param1: display item name:
  "SPD"|"PWR"|"CAD"|"LVL"|"Po"|"Pi"|"To"|"Km"|"Ke"|"Kt"|"Lf"
    "Po" short for "Power out"
    "Pi" short for "Power in"
    "To" short for "Torque out"
  >param2: value (Todo: need to use template)
  input value corresponding to display item .

  //example
  auto spd = a * b;
  bcu_disp.collect("SPD", spd)

4) Get parameters
  bcu_disp.get(param1, param2)
  >param1: display item name:
  "SPD"|"PWR"|"CAD"|"LVL"|"Po"|"Pi"|"To"|"Km"|"Ke"|"Kt"|"Lf"
    "Po" short for "Power out"
    "Pi" short for "Power in"
    "To" short for "Torque out"
  >param2: value
  sync parameter and return.
  //example
  auto Km_value = disp.get("Km")

  5. Print message to panel.
  bcu_disp.print()

  print all message to panel.
  
  example:
  bcu_disp.collect("SPD", 10)
  .....
  bcu_disp.print()

By jarry.wu123456
---------------------------------------------------------------------------------------*/

// #define OPEN_BCU_DISP_SAMPLE
#ifdef OPEN_BCU_DISP_SAMPLE

// need to change to bcu board pin map
#define BUT_UP    15            // Up key
#define BUT_DN    3             // Down key
#define BUT_MID   2             // Mid key

BcuDisplay::BcuDisp bcu_disp = BcuDisplay::BcuDisp(tft, {BUT_UP, BUT_DN,BUT_MID, RISING, TFT_BLACK, TFT_WHITE, TFT_VIOLET});

void setup() {
  Serial.begin(115200);
  bcu_disp.init();
  delay(100);
}

void loop() {
  //generate data;
  srand((int)time(0));
  bcu_disp.collect("SPD",  rand()%100);
  bcu_disp.collect("PWR",  rand()%100);
  bcu_disp.collect("CAD",  rand()%100);
  bcu_disp.collect("LVL",  rand()%100);
  bcu_disp.collect("Po",   rand()%100);
  bcu_disp.collect("Pi",   rand()%100);
  bcu_disp.collect("To",   rand()%100);

  //Get params
  Serial.print(">> Sync params:");
  Serial.print("Sync param [Km] "); Serial.print(bcu_disp.get("Km"));Serial.print("\n");
  Serial.print("Sync param [Ke] "); Serial.print(bcu_disp.get("Ke"));Serial.print("\n");
  Serial.print("Sync param [Kt] "); Serial.print(bcu_disp.get("Kt"));Serial.print("\n");
  Serial.print("Sync value [Lf] "); Serial.print(bcu_disp.get("Lf"));Serial.print("\n");

  //Print data
  bcu_disp.print();
  delay(200);
}
#else
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(1);
  // initialize the digital pin as an output.
  pinMode(LED_GREEN, OUTPUT);
  pinMode(ENABLE_12V, OUTPUT);
  digitalWrite(ENABLE_12V, HIGH);
  pinMode(TFT_BL,OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  pinMode(BUT_MID, INPUT);
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  delay(1000); // adding some delay to allow for serial print
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
  delay(500);
  tft.fillScreen(TFT_BLACK);
  
}

// the loop routine runs over and over again forever:
void loop() {
  
  // Get the input torque from the crank torque sensor
  uint32_t raw_measured_torque = analogRead(TORQUE_ADC);
  float measured_torque = (float) raw_measured_torque;
  measured_torque = mapf(raw_measured_torque, TORQUE_MIN, TORQUE_MAX, 0, 1);
  measured_torque = constrainf(measured_torque, 0, 1);

  // Get the power input from the generator 
  float current_in_amp = can.vesc_data_1.avgInputCurrent / 1000;
  float raw_elec_power_input = current_in_amp * (float) can.vesc_data_1.inpVoltage;
  float elec_power_input = mapf(raw_elec_power_input, 0, CURRENT_MAX_IN * VOLTAGE_MAX_IN, 0, 1);
  elec_power_input = constrainf(elec_power_input, 0, 1);

  // Combine those two values to feed into the motor
  uint32_t k_meca = 8;
  uint32_t k_elec = 8;
  float motor_power = k_meca * measured_torque + k_elec * elec_power_input;
  motor_power = mapf(motor_power, 0, 2, 0, 1);
  motor_power = constrainf(motor_power, 0, 1);

  uint32_t motorCurrent = (uint32_t) (mapf(motor_power, 0, 1, 0, CURRENT_MAX_OUT));

  tft.setTextSize(2);

  tft.setCursor(0,0);
  tft.print("SPD "); tft.print(can.vesc_data_1.dutyCycleNow); tft.print("   ");
  tft.setCursor(150,0);
  tft.print("CAD "); tft.print(can.vesc_data_2.dutyCycleNow); tft.print("   ");


  tft.setCursor(0,60);
  tft.print(measured_torque); tft.print(" "); // blank space to clean previous higher value
  tft.setCursor(100,60);
  tft.print(elec_power_input); tft.print(" ");
  tft.setCursor(200,60);
  tft.print(motorCurrent); tft.print("  ");

  
  static uint i = 0;
  static uint32_t cadence = 33000;
  static bool up_down=1;

  // If CAN0_INT pin is low, read receive buffer
  if(!digitalRead(CAN0_INT))
  {
    can.spin();
    if (millis() - last_print_data > 100)
    {
      can.vesc_set_erpm(1, cadence); //set generator rpm
      can.vesc_set_current(2, motorCurrent); //set motor current

      tft.setCursor(0,80);
      //tft.fillScreen(TFT_BLACK);
      tft.setTextSize(1);
      tft.print(i,DEC); tft.print("   "); tft.print(measured_torque); tft.print("   \n");
      tft.print("erpm vesc 1= "); tft.print(can.vesc_data_1.erpm); tft.print("   \n");
      tft.print("inpVoltage = "); tft.print(can.vesc_data_1.inpVoltage); tft.print("   \n");
      tft.print("dutyCycleNow = "); tft.print(can.vesc_data_1.dutyCycleNow); tft.print("   \n");
      tft.print("avgInputCurrent = "); tft.print(can.vesc_data_1.avgInputCurrent); tft.print("   \n");
      tft.print("avgMotorCurrent = "); tft.print(can.vesc_data_1.avgMotorCurrent); tft.print("   \n");

      tft.print("erpm vesc 2= "); tft.print(can.vesc_data_2.erpm); tft.print("   \n");
      tft.print("dutyCycleNow vesc 2= "); tft.print(can.vesc_data_2.dutyCycleNow); tft.print("   \n");
      if(SERIAL_PRINT){
        Serial.print(can.vesc_data_1.erpm); Serial.print(',');
        Serial.print(can.vesc_data_1.inpVoltage); Serial.print(',');
        Serial.print(can.vesc_data_1.dutyCycleNow); Serial.print(',');
        Serial.print(can.vesc_data_1.avgInputCurrent); Serial.print(',');
        Serial.print(can.vesc_data_1.avgMotorCurrent); Serial.print(',');
        Serial.print(can.vesc_data_1.tempFET); Serial.print(',');
        Serial.println(can.vesc_data_1.tempMotor);
      }
      digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
      last_print_data = millis();
    }
  }
}
#endif

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
  pinMode (LED_GREEN, OUTPUT) ;
  digitalWrite (LED_GREEN, HIGH) ;
  //--- Start serial
  Serial.begin (115200) ;
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_GREEN, !digitalRead (LED_GREEN)) ;
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
    digitalWrite (LED_GREEN, !digitalRead (LED_GREEN)) ;
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