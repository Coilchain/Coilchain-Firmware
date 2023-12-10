#include "common_inc.h"
#include "bcu_disp.h"

//#include <TFT_eSPI_Setups/Setup60_RP2040_ST7735.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <mcp_can.h>
#include <vesc_can_bus_arduino.h>
#include <math_helper.h>
#define SERIAL_PRINT 1

#include "led.cpp"
#define LED_DOUT 29
#define NUM_LEDS 14
void Ring1Complete();
NeoPatterns Ring1(NUM_LEDS, LED_DOUT, NEO_GRB + NEO_KHZ800, &Ring1Complete);

TFT_eSPI tft = TFT_eSPI();
CAN can;             // get torque sensor data, throttle for now

#define CAN0_INT 8                              // Set INT to pin 2
#define LED_GREEN 9
#define ENABLE_12V 7
#define TACH_GPIO 11
#define TORQUE_ADC A0

#define BUT_SEL   17
#define BUT_UP    18            // Up key
#define BUT_DN    19             // Down key
#define BUT_MID   14             // Mid key

BcuDisplay::BcuDisp bcu_disp = BcuDisplay::BcuDisp(tft, {BUT_SEL, FALLING, BUT_UP, BUT_DN,BUT_MID, RISING, TFT_BLACK, TFT_WHITE, TFT_VIOLET});

// Max voltage and current input from generator
#define CURRENT_MAX_IN 50
#define VOLTAGE_MAX_IN 60

//207, 460
#define TORQUE_MIN 512
#define TORQUE_MAX 1024
#define CURRENT_MAX_OUT (80*1000)
bool print_realtime_data = true;
long last_print_data;
long last_proccess_data;

//Bike configuration
#define MOTOR_POLAR_PAIRS  42         // Polar pairs in motor.
#define GEARBOX_RATIO     1.0         // GearBox ratios.
#define WHEEL_DIAMETER    2.0         // Bike diameter.
#define ADC2NM            1.0         // torque adc signal convert to nm depending on sensors.

void setup() {
  Serial.begin(115200);
  // init bcu display module.
  bcu_disp.init();
  Serial.println("bcu display init success");
  // init can bus
  pinMode(LED_GREEN, OUTPUT);
  pinMode(ENABLE_12V, OUTPUT);
  digitalWrite(ENABLE_12V, HIGH);
  pinMode(CAN0_INT, INPUT);

  // need add connected signal state.
  if(can.initialize() == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  delay(200);
}
void setup1() {
  // Initialize all the pixelStrips
  Ring1.begin();
  // Kick off a pattern
  Ring1.RainbowCycle(5);
  delay(300);
}
void loop1(){
  
  // Update the rings.
  Ring1.Update();
}
// the loop routine runs over and over again forever:
void loop() {
  long time_now = millis();
  static uint32_t motorCurrent = 0;
  // If CAN0_INT pin is low, read receive buffer
  if(!digitalRead(CAN0_INT))
  {
    can.spin();
    Serial.println("Spin!");
  }
  
  if (time_now - last_proccess_data > 100)
  {
    last_proccess_data = time_now;
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
    auto k_meca = bcu_disp.get("Km");
    auto k_elec = bcu_disp.get("Ke");
    //bcu_disp.collect("kt", k_meca+k_elec);

    float motor_power = k_meca * measured_torque + k_elec * elec_power_input;
    motor_power = mapf(motor_power, 0, 2, 0, 1);
    motor_power = constrainf(motor_power, 0, 1);
    motorCurrent = (uint32_t) (mapf(motor_power, 0, 1, 0, CURRENT_MAX_OUT));

    float cur  = motorCurrent; //float cad = can.vesc_data_1.erpm / MOTOR_POLAR_PAIRS;
    float rpm2 = can.vesc_data_2.erpm / MOTOR_POLAR_PAIRS;
    float spd = 0.06 * rpm2 * GEARBOX_RATIO * PI * WHEEL_DIAMETER;      // Kmh
    float pwd = can.vesc_data_2.avgInputCurrent * can.vesc_data_2.inpVoltage;
    float vbat = can.vesc_data_2.inpVoltage;
    bcu_disp.collect("SPD", spd);
    bcu_disp.collect("CUR", motor_power*80);
    bcu_disp.collect("PWR", pwd);
    bcu_disp.collect("VBAT", vbat);

    // torque: now is analog input.
    float torque  = measured_torque * ADC2NM;
    //bcu_disp.collect("To", measured_torque);
    //bcu_disp.collect("Pi", elec_power_input);
    //bcu_disp.collect("Po", motor_power);

    //can.vesc_set_erpm(1, cadence); //set generator rpm
    can.vesc_set_current(2, motorCurrent); //set motor current
  }
  static uint loopctr = 0;
  static uint i = 0;
  static uint32_t cadence = 33000;
  static bool up_down=1;
  if (time_now - last_print_data > 500)
  {

    if(SERIAL_PRINT){
      Serial.print(can.vesc_data_1.erpm); Serial.print(',');
      Serial.print(can.vesc_data_1.inpVoltage); Serial.print(',');
      Serial.print(can.vesc_data_1.dutyCycleNow); Serial.print(',');
      Serial.print(can.vesc_data_1.avgInputCurrent); Serial.print(',');
      Serial.print(can.vesc_data_1.avgMotorCurrent); Serial.print(',');
      Serial.print(can.vesc_data_1.tempFET); Serial.print(',');
      Serial.print(can.vesc_data_1.tempMotor);
      Serial.print(can.vesc_data_2.erpm); Serial.print(',');
      Serial.print(can.vesc_data_2.inpVoltage); Serial.print(',');
      Serial.print(can.vesc_data_2.dutyCycleNow); Serial.print(',');
      Serial.print(can.vesc_data_2.avgInputCurrent); Serial.print(',');
      Serial.print(can.vesc_data_2.avgMotorCurrent); Serial.print(',');
      Serial.print(can.vesc_data_2.tempFET); Serial.print(',');
      Serial.print(can.vesc_data_2.tempMotor);
      Serial.println("");
    }
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    last_print_data = time_now;
    
    //Serial.print(millis()); Serial.print(',');
    //Serial.print(micros()); Serial.print(',');
    bcu_disp.print();
    
    //Serial.print(loopctr); Serial.print(',');
    //Serial.print(millis()-time_now);
    //Serial.println("");
    loopctr=0;
  }
  else
  {
    loopctr++;
  }
  
}

// Ring1 Completion Callback
void Ring1Complete()
{
        // Alternate color-wipe patterns with Ring2
        //Ring1.Color1 = Ring1.Wheel(random(255));
        //Ring1.Interval = 100;
}