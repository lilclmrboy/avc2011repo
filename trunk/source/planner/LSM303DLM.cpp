#include "avc.h"
#include "compass.h"

#define ACCEL_IIC_ADDR 0x30
// Accelerometer registers
#define LSM303DLM_CTRL_REG1_A 0x20
#define LSM303DLM_CTRL_REG2_A 0x21
#define LSM303DLM_CTRL_REG3_A 0x22
#define LSM303DLM_CTRL_REG4_A 0x23
#define LSM303DLM_CTRL_REG5_A 0x24
#define LSM303DLM_HP_FILTER_RESET_A 0x25
#define LSM303DLM_REFERENCE_A 0x26
#define LSM303DLM_STATUS_REG_A 0x27
#define LSM303DLM_OUT_X_L_A 0x28
#define LSM303DLM_OUT_X_H_A 0x29
#define LSM303DLM_OUT_Y_L_A 0x2A
#define LSM303DLM_OUT_Y_H_A 0x2B
#define LSM303DLM_OUT_Z_L_A 0x2C
#define LSM303DLM_OUT_Z_H_A 0x2D
#define LSM303DLM_INT1_CFG_A 0x30
#define LSM303DLM_INT1_SOURCE_A 0x31
#define LSM303DLM_INT1_THS_A 0x32
#define LSM303DLM_INT1_DURATION_A 0x33
#define LSM303DLM_INT2_CFG_A 0x34
#define LSM303DLM_INT2_SOURCE_A 0x35
#define LSM303DLM_INT2_THS_A 0x36
#define LSM303DLM_INT2_DURATION_A 0x37

#define LSM303DLM_MAG_IIC_ADDR 0x3C
// Magnetometer registers
#define LSM303DLM_CRA_REG_M 0x00
#define LSM303DLM_CRB_REG_M 0x01
#define LSM303DLM_MR_REG_M 0x02
#define LSM303DLM_OUT_X_H_M 0x03
#define LSM303DLM_OUT_X_L_M 0x04
#define LSM303DLM_OUT_Y_H_M 0x07
#define LSM303DLM_OUT_Y_L_M 0x08
#define LSM303DLM_OUT_Z_H_M 0x05
#define LSM303DLM_OUT_Z_L_M 0x06
#define LSM303DLM_SR_REG_Mg 0x09
#define LSM303DLM_IRA_REG_M 0x0A
#define LSM303DLM_IRB_REG_M 0x0B
#define LSM303DLM_IRC_REG_M 0x0C
#define LSM303DLM_WHO_AM_I_M 0x0F

/////////////////////////////////////////////////////////////////////////////
// Constructor that calls parent constructor
compassLSM303DLM::compassLSM303DLM(acpStem *pStem, aSettingFileRef settings)
	: avcCompass(pStem, settings)
{
	m_compassHwType = kCompass_LSM303DLM;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
compassLSM303DLM::~compassLSM303DLM(){
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::init(){
  // setup the various control registers
  aUInt8 reg_buffer[2];
  aPacketRef regPacket;
  
  // set the update rate
  reg_buffer[0] = LSM303DLM_CRA_REG_M;
  reg_buffer[1] = 0x14; // 15Hz update rate
  regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 2, reg_buffer);
  m_pStem->sendPacket(regPacket);
  
  // set the gain
  // earth's field is 0.25-0.65 gauss
  reg_buffer[0] = LSM303DLM_CRB_REG_M;
  reg_buffer[1] = 0x20; // ±1.3 gauss range
  reg_buffer[1] = 0x80; // ±4.0 gauss range
  reg_buffer[1] = 0xE0; // ±8.1 gauss range
  regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 2, reg_buffer);
  m_pStem->sendPacket(regPacket);
  
  // set the operating mode
  reg_buffer[0] = LSM303DLM_MR_REG_M;
  reg_buffer[1] = 0x00; // continuous conversion mode
  regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 2, reg_buffer);
  m_pStem->sendPacket(regPacket);
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagnetometerReadings(int *x, int *y, int *z){
  // check the points
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  try {
    getMagneticX(x);
    getMagneticY(y);
    getMagneticZ(z);
  } catch (int &e) {
    m_logger->log(ERROR, "%s: error while reading magnetometer", __FUNCTION__);
    return -1;
  }
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getHeadingDeg(float *headingDeg){
  // check pointer
  if(!headingDeg)
		return -1;
  
  m_logger->log(DEBUG, "%s: this function is not complete", __FUNCTION__);
  // need to add dot product to get difference from north for "heading"
  // need to add calibration information
  
  int x=0;
  getMagneticX(&x);
  
  *headingDeg = (float)x;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::readTwoByteTwosCompliment(unsigned int firstReg, int *reading){
	// Check the passed pointer
  if(!reading)
    throw -1;
  
  short ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  aUInt8 reg_buffer[1] = {firstReg};
  
  aPacketRef regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 1, reg_buffer);
  m_pStem->sendPacket(regPacket);
  m_pStem->IIC_RD(aUSBSTEM_MODULE, LSM303DLM_MAG_IIC_ADDR+1, 2, read_buffer);
  
  // value is stored as two byte two's compliment little endian
  ret_value = (short)((read_buffer[1] << 8) | read_buffer[0]);
  
  // sign extension by cast
  *reading = (int) ret_value;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagneticX(int *x){
  // check pointers
  if(!x)
		throw -1;
  
	readTwoByteTwosCompliment(LSM303DLM_OUT_X_H_M, x);
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagneticY(int *y){
  // check pointers
  if(!y)
		throw -1;
  
	readTwoByteTwosCompliment(LSM303DLM_OUT_Y_H_M, y);
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagneticZ(int *z){
  // check pointers
  if(!z)
		throw -1;
  
	readTwoByteTwosCompliment(LSM303DLM_OUT_Z_H_M, z);
  
  return 0;
}



