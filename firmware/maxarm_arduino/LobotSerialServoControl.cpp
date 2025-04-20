//﻿
//* FileName:      LobotSerialServoControl.h
//* Company:     Hiwonder
//* Date:          2020/05/13  16:53
// *Last Modification Date: 202005131938
//* www.hiwonder.com

#include "LobotSerialServoControl.h"
#include <Stream.h>

LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &A)
{
  isAutoEnableRT = true;
	isUseHardwareSerial = true;
	SerialX = (Stream*)(&A);
}
LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &A,int receiveEnablePin, int transmitEnablePin)
{
  isAutoEnableRT = false;
  this->receiveEnablePin = receiveEnablePin;
  this->transmitEnablePin = transmitEnablePin;
  
  isUseHardwareSerial = true;
  SerialX = (Stream*)(&A);
}

void LobotSerialServoControl::OnInit(void)
{
  if(!isAutoEnableRT)
  {
    pinMode(receiveEnablePin, OUTPUT);
    pinMode(transmitEnablePin, OUTPUT);
    RxEnable();
  }
}

inline void LobotSerialServoControl::RxEnable(void)
{
  digitalWrite(receiveEnablePin, HIGH);
  digitalWrite(transmitEnablePin, LOW);
}
inline void LobotSerialServoControl::TxEnable(void)
{
  digitalWrite(receiveEnablePin, LOW);
  digitalWrite(transmitEnablePin, HIGH);
}

byte LobotSerialServoControl::LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoControl::LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoStopMove(uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
}

void LobotSerialServoControl::LobotSerialServoSetID(uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoControl::LobotSerialServoSetMode(uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoLoad(uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoControl::LobotSerialServoUnload(uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

int LobotSerialServoControl::LobotSerialServoReceiveHandle(byte *ret)
{
  bool frameStarted = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  const unsigned long timeout = 50; // ms
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    if (SerialX->available()) {
      rxBuf = SerialX->read();
      delayMicroseconds(100);  // tiny delay helps serial stability

      if (!frameStarted) {
        if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
          frameCount++;
          if (frameCount == 2) {
            frameStarted = true;
            recvBuf[0] = LOBOT_SERVO_FRAME_HEADER;
            recvBuf[1] = LOBOT_SERVO_FRAME_HEADER;
            dataCount = 2;
          }
        } else {
          frameCount = 0;
        }
      } else {
        recvBuf[dataCount++] = rxBuf;

        if (dataCount == 4) {
          dataLength = recvBuf[3];
          if (dataLength < 3 || dataCount > sizeof(recvBuf) - 1) {
            frameStarted = false;
            frameCount = 0;
            dataCount = 0;
            continue;
          }
        }

        if (dataCount == dataLength + 3) {
          if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
            memcpy(ret, recvBuf + 4, dataLength - 1); // exclude checksum
            return 1;
          }
          return -1; // checksum failed
        }
      }
    } else {
      delay(1); // give CPU and serial some breathing room
    }
  }
  return -1; // timeout
}


int LobotSerialServoControl::LobotSerialServoReadPosition(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);


#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX->available())
    SerialX->read();
    
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
  if(isUseHardwareSerial)
  {
    delayMicroseconds(600);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -1;
  }



  if (LobotSerialServoReceiveHandle(buf) > 0){
    ret = BYTE_TO_HW(buf[2], buf[1]);
  }
  else{
    ret = -2048;
  }

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadVin(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false)
    TxEnable();
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(600);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
