#include "avc.h"
#include "compass.h"
#include <math.h>

#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)

#define LSM303DLM_ACCEL_IIC_ADDR 0x30
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

#define CHECK_INITIALIZATION if(0==m_beenInitialized){m_logger->log(ERROR, "%s: LSM303DLM is not initialized", __FUNCTION__);return -1;}

/////////////////////////////////////////////////////////////////////////////
// Constructor that calls parent constructor
compassLSM303DLM::compassLSM303DLM(acpStem *pStem, aSettingFileRef settings)
	: avcCompass(pStem, settings)
{
	m_compassHwType = kCompass_LSM303DLM;
  m_beenInitialized = 0;
  
  // set the compass calibration values
  m_compassCalMin.x = -544;
  m_compassCalMin.y = -608;
  m_compassCalMin.z = -432;
  
  m_compassCalMax.x = 312;
  m_compassCalMax.y = 304;
  m_compassCalMax.z = 473;
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
  
  // check to see if we've already run initialization
  // this is needed since the initialization routine is shared between the compass
  // and accelerometer classes (compass requires accelerometer readings to compute
  // heading). We don't want someone calling init while somewhere else is making
  // readings
  if(1 == m_beenInitialized){
    m_logger->log(INFO, "%s: LSM303DLM has already been initialized", __FUNCTION__);
    return 0;
  }
  
  try {
    // set the update rate
    reg_buffer[0] = LSM303DLM_CRA_REG_M;
    reg_buffer[1] = 0x14; // 15Hz update rate
    regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 2, reg_buffer);
    m_pStem->sendPacket(regPacket);
    
    // set the gain
    // earth's field is 0.25-0.65 gauss
    reg_buffer[0] = LSM303DLM_CRB_REG_M;
    reg_buffer[1] = 0x20; // ±1.3 gauss range
    //reg_buffer[1] = 0x80; // ±4.0 gauss range
    //reg_buffer[1] = 0xE0; // ±8.1 gauss range
    regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 2, reg_buffer);
    m_pStem->sendPacket(regPacket);
    
    // set the operating mode
    reg_buffer[0] = LSM303DLM_MR_REG_M;
    reg_buffer[1] = 0x00; // continuous conversion mode
    regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 2, reg_buffer);
    m_pStem->sendPacket(regPacket);
    
    // also need to configure the accelerometer
    // set the power mode to normal (001x xxxx)
    // set the update rate to 50Hz (xxx0 0xxx), 100Hz (xxx0 1xxx)
    // enable all three axes (xxxx x111)
    reg_buffer[0] = LSM303DLM_CTRL_REG1_A;
    reg_buffer[1] = 0x27;
    regPacket = m_pStem->createPacket(LSM303DLM_ACCEL_IIC_ADDR, 2, reg_buffer);
    m_pStem->sendPacket(regPacket);
    
    // change block data update to not update MSB/LSB during reads
    reg_buffer[0] = LSM303DLM_CTRL_REG4_A;
    reg_buffer[1] = 0x80;
    regPacket = m_pStem->createPacket(LSM303DLM_ACCEL_IIC_ADDR, 2, reg_buffer);
    m_pStem->sendPacket(regPacket);
  } catch (acpException &e){
    m_logger->log(ERROR, "%s: Stem error while initializing LSM303DLM: %s", __FUNCTION__, e.msg());
  }
  
  m_beenInitialized = 1;
  
  // get and store the inital acc readings to be used in compass heading calc
  int accX=0, accY=0, accZ=0;
  if(0 != getAccelerometerReadings(&accX, &accY, &accZ)){
    m_logger->log(ERROR, "%s: Error while getting initial accelerometer readings from the LSM303DLM", __FUNCTION__);
    return -1;
  }
  
  m_initalAccelerometerReadings = vector3D((double)accX, (double)accY, (double)accZ);
  m_logger->log(DEBUG, "%s: Initial accV: %3.2f, %3.2f, %3.2f", __FUNCTION__, (double)accX, (double)accY, (double)accZ);
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagnetometerReadings(int *x, int *y, int *z){
  // check the points
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  CHECK_INITIALIZATION;
  
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
  
  CHECK_INITIALIZATION;

  // get the magnetometer readings
  int magX=0, magY=0, magZ=0;
  if(0 != getMagnetometerReadings(&magX, &magY, &magZ)){
    m_logger->log(ERROR, "%s: error while reading magnetometer", __FUNCTION__);
    return -1;
  }
  
  // store the mag readings into a vector for easy computing later
  vector3D magV = vector3D((double)magX, (double)magY, (double)magZ);

  // the accelerometer is read when the class is initialized
  // stored in vector3D m_initalAccelerometerReadings
  
  calculateHeadingDeg(magV, headingDeg);
  
  // offset for declination
  //*headingDeg -= 14.6;
  
  //if(*headingDeg < 0) *headingDeg += 360;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// read the accelarometer values
int compassLSM303DLM::getAccelerometerReadings(int *accX, int *accY, int *accZ){
  // check the pointers
  if(!accX || !accY || !accZ){
    m_logger->log(ERROR, "%s: null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  CHECK_INITIALIZATION;
  
  try {
    readTwoByteTwosComplimentLittleEndian(LSM303DLM_OUT_X_L_A, accX);
    readTwoByteTwosComplimentLittleEndian(LSM303DLM_OUT_Y_L_A, accY);
    readTwoByteTwosComplimentLittleEndian(LSM303DLM_OUT_Z_L_A, accZ);
  } catch (int &e) {
    m_logger->log(ERROR, "%s: error while reading accelerometer", __FUNCTION__);
    return -1;
  }
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// Description of heading algorithm: 
// Shift and scale the magnetic reading based on calibration data to
// to find the North vector. Use the acceleration readings to
// determine the Down vector. The cross product of North and Down
// vectors is East. The vectors East and North form a basis for the
// horizontal plane. The chipOrientationV vector is projected into the horizontal
// plane and the angle between the projected vector and north is
// returned.
int compassLSM303DLM::calculateHeadingDeg(vector3D magV, float *headingDeg){
  if(!headingDeg)
    return -1;
  
  // defines the orientation of the chip
  // heading is 
  vector3D chipOrientationV = vector3D(1,0,0);
  
  vector3D scaledMagV, normalizedAcc;
  // shift and scale the magnetometer readings
  scaledMagV.x = ((double)magV.x - m_compassCalMin.x) / (m_compassCalMax.x - m_compassCalMin.x) * 2.0 - 1.0;
  scaledMagV.y = ((double)magV.y - m_compassCalMin.y) / (m_compassCalMax.y - m_compassCalMin.y) * 2.0 - 1.0;
  scaledMagV.z = ((double)magV.z - m_compassCalMin.z) / (m_compassCalMax.z - m_compassCalMin.z) * 2.0 - 1.0;
  
  //int x, y, z;
  //getAccelerometerReadings(&x, &y, &z);
  //m_initalAccelerometerReadings.x = (double)x;
  //m_initalAccelerometerReadings.y = (double)y;
  //m_initalAccelerometerReadings.z = (double)z;
  normalizedAcc = m_initalAccelerometerReadings;
  normalizeVector3D(&normalizedAcc);
                   
  // compute E and refine (normalize) N
  vector3D EastV, NorthV;
  crossProductVector3D(&scaledMagV, &normalizedAcc, &EastV);
  normalizeVector3D(&EastV);
  crossProductVector3D(&normalizedAcc, &EastV, &NorthV); // north should be normalized (unit X unit = unit)
  
  //compute the heading
  *headingDeg = (float)RAD_TO_DEG * atan2( dotProductVector3D(&EastV, &chipOrientationV), dotProductVector3D(&NorthV, &chipOrientationV));
  
  if(*headingDeg < 0) *headingDeg += 360.0;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
void compassLSM303DLM::normalizeVector3D(vector3D *vec){
  double mag = sqrt(dotProductVector3D(vec, vec));
  if(fabs(mag) < 1e-9){
    m_logger->log(ERROR, "%s: magnitude is quite small: %3.2e", __FUNCTION__, mag);
  }
  vec->x /= mag;
  vec->y /= mag;
  vec->z /= mag;
}

/////////////////////////////////////////////////////////////////////////////
void compassLSM303DLM::crossProductVector3D(const vector3D *a, const vector3D *b, vector3D *result){
  result->x = a->y*b->z - a->z*b->y;
  result->y = a->z*b->x - a->x*b->z;
  result->z = a->x*b->y - a->y*b->x;
}

/////////////////////////////////////////////////////////////////////////////
double compassLSM303DLM::dotProductVector3D(const vector3D *a, const vector3D *b){
  return a->x*b->x + a->y*b->y + a->z*b->z;
}




/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::readTwoByteTwosComplimentLittleEndian(unsigned int firstReg, int *reading){
	// Check the passed pointer
  if(!reading)
    throw -1;
  
  short ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  aUInt8 reg_buffer[1] = {firstReg};
  
  // assert the MSB of the address to get the gyro 
	// to do slave-transmit register auto-increment
  reg_buffer[0] = firstReg | (1 << 7);
  
  try {
    aPacketRef regPacket = m_pStem->createPacket(LSM303DLM_ACCEL_IIC_ADDR, 1, reg_buffer);
    m_pStem->sendPacket(regPacket);
    m_pStem->IIC_RD(aUSBSTEM_MODULE, LSM303DLM_ACCEL_IIC_ADDR+1, 2, read_buffer);
  } catch (acpException &e){
    m_logger->log(ERROR, "%s: Stem error reading LSM303DLM: %s", __FUNCTION__, e.msg());
    throw -1;
  }
  
  // if the IIC device doesn't respond, the brainstem will stuff 0xFF into the buffer elements
  // check for this and throw an error
  if (read_buffer[0] == 0xFF && read_buffer[1] == 0xFF){
    m_logger->log(ERROR, "%s: time out or IIC read error", __FUNCTION__);
    throw -1;
  }

  
  // value is stored as two byte two's compliment little endian
  ret_value = (short)((read_buffer[1] << 8) | read_buffer[0]);
  
  // sign extension by cast
  *reading = (int) ret_value;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::readTwoByteTwosComplimentBigEndian(unsigned int firstReg, int *reading){
	// Check the passed pointer
  if(!reading)
    throw -1;
  
  short ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  aUInt8 reg_buffer[1] = {firstReg};
  
  try {
    aPacketRef regPacket = m_pStem->createPacket(LSM303DLM_MAG_IIC_ADDR, 1, reg_buffer);
    m_pStem->sendPacket(regPacket);
    m_pStem->IIC_RD(aUSBSTEM_MODULE, LSM303DLM_MAG_IIC_ADDR+1, 2, read_buffer);
  } catch (acpException &e){
    m_logger->log(ERROR, "%s: Stem error reading LSM303DLM: %s", __FUNCTION__, e.msg());
    throw -1;
  }
  
  // if the IIC device doesn't respond, the brainstem will stuff 0xFF into the buffer elements
  // check for this and throw an error
  if (read_buffer[0] == 0xFF && read_buffer[1] == 0xFF){
    m_logger->log(ERROR, "%s: time out or IIC read error", __FUNCTION__);
    throw -1;
  }
  
  
  // value is stored as two byte two's compliment little endian
  ret_value = (short)((read_buffer[0] << 8) | read_buffer[1]);
  
  // sign extension by cast
  *reading = (int) ret_value;
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagneticX(int *x){
  // check pointers
  if(!x)
		throw -1;
  
  try {
    readTwoByteTwosComplimentBigEndian(LSM303DLM_OUT_X_H_M, x);
  } catch (int &e) {
    m_logger->log(ERROR, "%s: error while reading magnetometer", __FUNCTION__);
    return -1;
  }
	
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagneticY(int *y){
  // check pointers
  if(!y)
		throw -1;
  
  try {
    readTwoByteTwosComplimentBigEndian(LSM303DLM_OUT_Y_H_M, y);
  } catch (int &e) {
    m_logger->log(ERROR, "%s: error while reading magnetometer", __FUNCTION__);
    return -1;
  }
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int compassLSM303DLM::getMagneticZ(int *z){
  // check pointers
  if(!z)
		throw -1;
  try {
    readTwoByteTwosComplimentBigEndian(LSM303DLM_OUT_Z_H_M, z);
  } catch (int &e) {
    m_logger->log(ERROR, "%s: error while reading magnetometer", __FUNCTION__);
    return -1;
  }
  
  return 0;
}






