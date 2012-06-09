#include "avc.h"
#include "compass.h"

#define CMPS10_IIC_ADDR 0xC0
#define CMPS10_SW_VER 0          // Software version
#define CMPS10_BEARING_SHORT 1   // Compass Bearing as a byte, i.e. 0-255 for a full circle
#define CMPS10_BEARING_INT 2     // Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
#define CMPS10_PITCH 4           // Pitch angle - signed byte giving angle in degrees from the horizontal plane
#define CMPS10_ROLL 5            // Roll angle - signed byte giving angle in degrees from the horizontal plane
#define CMPS10_MAGX 10           // Magnetometer X axis raw output, 16 bit signed integer with register 10 being the upper 8 bits
#define CMPS10_MAGY 12           // Magnetometer Y axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
#define CMPS10_MAGZ 14           // Magnetometer Z axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
#define CMPS10_ACCX 16           // Accelerometer  X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
#define CMPS10_ACCY 18           // Accelerometer  Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
#define CMPS10_ACCZ 20           // Accelerometer  Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
#define CMPS10_CMD 22            // Command register

/////////////////////////////////////////////////////////////////////////////
// Constructor that calls parent constructor
compassCMPS10::compassCMPS10(acpStem *pStem, aSettingFileRef settings)
	: avcCompass(pStem, settings)
{
	m_compassHwType = kCompass_CMPS10;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
compassCMPS10::~compassCMPS10(){
}

/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::init(){
  // setup the various control registers
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::getMagnetometerReadings(int *x, int *y, int *z){
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
int compassCMPS10::getHeadingDeg(float *headingDeg){
  // check pointer
  if(!headingDeg)
		return -1;
  
  int iHeadingDeg = 0;
	readTwoByteTwosCompliment(CMPS10_BEARING_INT, &iHeadingDeg);
  
  //convert the two int to a float
  *headingDeg = ((float)iHeadingDeg)/10.0;
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::readTwoByteTwosCompliment(unsigned int firstReg, int *reading){
	// Check the passed pointer
  if(!reading)
    throw -1;
  
  short ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  aUInt8 reg_buffer[1] = {firstReg};
  
  aPacketRef regPacket = m_pStem->createPacket(CMPS10_IIC_ADDR, 1, reg_buffer);
  m_pStem->sendPacket(regPacket);
  m_pStem->IIC_RD(aUSBSTEM_MODULE, CMPS10_IIC_ADDR+1, 2, read_buffer);
  
  // value is stored as two byte two's compliment big endian
  ret_value = (short)((read_buffer[0] << 8) | read_buffer[1]);
  
  // sign extension by cast
  *reading = (int) ret_value;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::getMagneticX(int *x){
  // check pointers
  if(!x)
		throw -1;
  
	readTwoByteTwosCompliment(CMPS10_MAGX, x);
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::getMagneticY(int *y){
  // check pointers
  if(!y)
		throw -1;
  
	readTwoByteTwosCompliment(CMPS10_MAGY, y);
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::getMagneticZ(int *z){
  // check pointers
  if(!z)
		throw -1;
  
	readTwoByteTwosCompliment(CMPS10_MAGZ, z);
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassCMPS10::resetCalToFactory(){
  aUInt8 reg_buffer[2] = {CMPS10_CMD, 0x20};
  
  aPacketRef regPacket = m_pStem->createPacket(CMPS10_IIC_ADDR, 2, reg_buffer);
  m_pStem->sendPacket(regPacket);
  m_pStem->sleep(100);
	
  reg_buffer[1] = 0x2A;
  regPacket = m_pStem->createPacket(CMPS10_IIC_ADDR, 1, reg_buffer);
  m_pStem->sendPacket(regPacket);
  m_pStem->sleep(100);
	
  reg_buffer[1] = 0x60;
  regPacket = m_pStem->createPacket(CMPS10_IIC_ADDR, 1, reg_buffer);
  m_pStem->sendPacket(regPacket);
  
  return 0;
  
}


