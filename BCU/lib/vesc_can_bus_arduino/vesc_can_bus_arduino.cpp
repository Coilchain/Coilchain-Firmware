#include <mcp_can.h>
#include <vesc_can_bus_arduino.h>
#include <SPI.h>

MCP_CAN CAN0(13);                               // Set CS to pin 10

INT8U CAN::initialize() {
  INT8U begin_result = CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  return (begin_result);
}

void CAN::spin() {
  get_frame();

  print_raw_can_data();  // uncomment to see raw can messages

  if ((rxId & 0xFFFFFFFE) == 0x80000900) { //  if (rxId == 0x8000090A) {
    vesc_data.dutyCycleNow = process_data_frame_vesc('D', rxBuf[6], rxBuf[7]);
    vesc_data.avgMotorCurrent = process_data_frame_vesc('C', rxBuf[4], rxBuf[5]);
    unsigned char erpmvals[4];
    erpmvals[0] = rxBuf[3];
    erpmvals[1] = rxBuf[2];
    erpmvals[2] = rxBuf[1];
    erpmvals[3] = rxBuf[0];
    vesc_data.erpm = *(long *)erpmvals;

    if ((rxId & 0x1) == 0x1){
      vesc_data_1.dutyCycleNow = vesc_data.dutyCycleNow;
      vesc_data_1.avgMotorCurrent = vesc_data.avgMotorCurrent;
      vesc_data_1.erpm = vesc_data.erpm;
    }
    else if ((rxId & 0x1) == 0x2){
      vesc_data_2.dutyCycleNow = vesc_data.dutyCycleNow;
      vesc_data_2.avgMotorCurrent = vesc_data.avgMotorCurrent;
      vesc_data_2.erpm = vesc_data.erpm;
    }
    //need to add in the rpm conversion function for 4 byte values
  }
  else if ((rxId & 0xFFFFFFFE) == 0x80001000) { //
    vesc_data.tempFET = process_data_frame_vesc('F', rxBuf[0], rxBuf[1]);
    vesc_data.tempMotor = process_data_frame_vesc('T', rxBuf[2], rxBuf[3]);
    vesc_data.avgInputCurrent = process_data_frame_vesc('I', rxBuf[4], rxBuf[5]);

    if ((rxId & 0x1) == 0x1){
      vesc_data_1.tempFET = vesc_data.tempFET;
      vesc_data_1.tempMotor = vesc_data.tempMotor;
      vesc_data_1.avgInputCurrent = vesc_data.avgInputCurrent;
    }
    else if ((rxId & 0x1) == 0x2){
      vesc_data_2.tempFET = vesc_data.tempFET;
      vesc_data_2.tempMotor = vesc_data.tempMotor;
      vesc_data_2.avgInputCurrent = vesc_data.avgInputCurrent;
    }
  }
  else if (rxId == 0x80001B00 + vesc_id) {
    char receivedByte[4], *p;
    sprintf(receivedByte, "%02X%02X", rxBuf[4], rxBuf[5]);
    vesc_data.inpVoltage = hex2int(receivedByte) * 0.1;

    if ((rxId & 0x1) == 0x1){
      vesc_data_1.inpVoltage = vesc_data.inpVoltage;
    }
    else  if ((rxId & 0x1) == 0x2){
      vesc_data_2.inpVoltage = vesc_data.inpVoltage;
    }
  }
}

void CAN::print_raw_can_data() {
  int len = 8;
  sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  Serial.print(msgString);
  for (byte i = 0; i < len; i++) {
    sprintf(msgString, " 0x%.2X", rxBuf[i]);
    Serial.print(msgString);
  }
  Serial.println();
}

float CAN::process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2) {
  char receivedByte[4], *p;
  sprintf(receivedByte, "%02X%02X", byte1, byte2);
  float output = hex2int(receivedByte);

  switch (datatype) {
    case 'D': output *= 0.001; break; //dutyCycleNow
    case 'C': output *= 0.1; break; //avgMotorCurrent
    case 'F': output *= 0.1; break; //tempFET
    case 'T': output *= 0.1; break; //tempMotor
    case 'I': output *= 0.1; break; //avgInputCurrent
    case 'V': output *= 0.1; break; //inpVoltage
  }
  return output;
}

int CAN::hex2int(char buf[])
{
  return (short) strtol(buf, NULL, 16);
}

void CAN::vesc_set_duty(float duty) {
  uint32_t set_value = duty * 100000;
  uint8_t buffer[4];
  buffer[0] = (set_value >> 24) & 0xFF;
  buffer[1] = (set_value >> 16) & 0xFF;
  buffer[2] = (set_value  >> 8  )  & 0xFF;
  buffer[3] = set_value & 0xFF;
  byte sndStat = CAN0.sendMsgBuf(0x00000002, 1, 4, buffer);
  sndStat = CAN0.sendMsgBuf(0x00000001, 1, 4, buffer);
}

void CAN::vesc_set_current(uint8_t vesc_id, uint32_t set_value) {
  // set_value = current in mA
  uint8_t buffer[4];
  buffer[0] = (set_value >> 24) & 0xFF;
  buffer[1] = (set_value >> 16) & 0xFF;
  buffer[2] = (set_value  >> 8  )  & 0xFF;
  buffer[3] = set_value & 0xFF;
  byte sndStat = CAN0.sendMsgBuf(0x00000100+vesc_id, 1, 4, buffer);
}

void CAN::vesc_set_erpm(uint8_t vesc_id, uint32_t erpm) {
  uint32_t set_value = erpm;
  uint8_t buffer[4];
  buffer[0] = (set_value >> 24) & 0xFF;
  buffer[1] = (set_value >> 16) & 0xFF;
  buffer[2] = (set_value  >> 8  )  & 0xFF;
  buffer[3] = set_value & 0xFF;
  byte sndStat = CAN0.sendMsgBuf(0x00000300+vesc_id, 1, 4, buffer);
}
void CAN::get_frame() {
  CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
}
