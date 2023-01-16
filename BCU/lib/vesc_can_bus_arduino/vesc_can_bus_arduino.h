#include <Arduino.h> //required for PD# definitions

class CAN
{

public:

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];// Array to store serial string


float inpVoltage, dutyCycleNow, avgInputCurrent, avgMotorCurrent, tempFET, tempMotor;
long erpm;


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
