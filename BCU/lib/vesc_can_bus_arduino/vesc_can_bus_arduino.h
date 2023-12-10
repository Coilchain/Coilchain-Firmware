#include <Arduino.h> //required for PD# definitions

struct Vesc_data {
  float inpVoltage;
  float dutyCycleNow; 
  float avgInputCurrent; 
  float avgMotorCurrent;
  float tempFET;
  float tempMotor;
  long erpm;
};

class CAN
{

public:

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];// Array to store serial string

struct Vesc_data vesc_data = {
  0.0, // inpVoltage;
  0.0, // dutyCycleNow; 
  0.0, // avgInputCurrent; 
  0.0, // avgMotorCurrent;
  0.0, // tempFET;
  0.0, // tempMotor;
  0, // erpm;
};
struct Vesc_data vesc_data_1 = {
  0.0, // inpVoltage;
  0.0, // dutyCycleNow; 
  0.0, // avgInputCurrent; 
  0.0, // avgMotorCurrent;
  0.0, // tempFET;
  0.0, // tempMotor;
  0, // erpm;
};
struct Vesc_data vesc_data_2 = {
  12.3, // inpVoltage;
  0.0, // dutyCycleNow; 
  2.0, // avgInputCurrent; 
  0.0, // avgMotorCurrent;
  0.0, // tempFET;
  0.0, // tempMotor;
  0, // erpm;
};

INT8U initialize();
void spin();
void get_frame(); // populates rxId and rxBuf with latest can frame 
void can_send(byte data[8]); //transmits the send commands to the sensor
void print_raw_can_data(); //output raw can data to terminal (debug)


void vesc_set_duty(float duty);
void vesc_set_current(uint8_t vesc_id, uint32_t current);
void vesc_set_erpm(uint8_t vesc_id, uint32_t erpm);
float process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2);
int hex2int(char buf[]);

};
