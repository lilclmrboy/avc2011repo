#include "avc.h"
#include "gyro.h"

#define L3G4200D_IIC_ADDR 0xD2

#define L3G4200D_WHO_AM_I 0x0F
#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_CTRL_REG3 0x22
#define L3G4200D_CTRL_REG5 0x24
#define L3G4200D_OUT_TEMP 0x26
#define L3G4200D_OUT_X_L 0x28
#define L3G4200D_OUT_Y_L 0x2A
#define L3G4200D_OUT_Z_L 0x2C
#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_INT1_CFG 0x30
#define L3G4200D_INT1_TSH_XH 0x32
#define L3G4200D_INT1_TSH_YH 0x34
#define L3G4200D_INT1_TSH_ZH 0x36
#define L3G4200D_INT1_DURATION 0x38
/////////////////////////////////////////////////////////////////////////////
gyroL3G4200D::gyroL3G4200D(acpStem *pStem, aSettingFileRef settings)
	: avcGyro(pStem, settings)
{
  m_gyroHWtype = kGyro_L3G4200D;
}

/////////////////////////////////////////////////////////////////////////////
gyroL3G4200D::~gyroL3G4200D(){
  
}


/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200D::init(){
  // setup the various control registers

	m_logger->log(DEBUG, "%s: Here", __FUNCTION__);
  // CTRL_REG1
  aUInt8 reg_buffer[2] = {L3G4200D_CTRL_REG1, 0x0F}; // Normal power mode, all axes enabled
  
  aPacketRef regPacket = m_pStem->createPacket(L3G4200D_IIC_ADDR, 2, reg_buffer);
  m_pStem->sendPacket(regPacket);

  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200D::gyroL3G4200DreadTwoByteTwosCompliment(unsigned int firstReg, int *reading){
  // Check the passed pointer
  if(!reading)
    return -1;
  
  short ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  // assert the MSB of the address to get the gyro 
	// to do slave-transmit register auto-increment
  aUInt8 reg_buffer[1] = {firstReg | (1 << 7)};
  
  aPacketRef regPacket = m_pStem->createPacket(L3G4200D_IIC_ADDR, 1, reg_buffer);
  m_pStem->sendPacket(regPacket);
  m_pStem->IIC_RD(aUSBSTEM_MODULE, L3G4200D_IIC_ADDR+1, 2, read_buffer);
  
  // value is stored as two byte two's compliment; stuff and extend
  ret_value = (short)((read_buffer[1] << 8) | read_buffer[0]);

  *reading = (int) ret_value;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200D::gyroL3G4200DgetX(int *dddx){
  if(!dddx)
    throw -1;
  
  if(0 != gyroL3G4200DreadTwoByteTwosCompliment(L3G4200D_OUT_X_L, dddx))
    throw -1;
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200D::gyroL3G4200DgetY(int *dddy){
  if(!dddy)
    throw -1;

  if(0 != gyroL3G4200DreadTwoByteTwosCompliment(L3G4200D_OUT_Y_L, dddy))
    throw -1;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200D::gyroL3G4200DgetZ(int *dddz){
  if(!dddz)
    throw -1;
  
  if(0 != gyroL3G4200DreadTwoByteTwosCompliment(L3G4200D_OUT_Z_L, dddz))
    throw -1;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200D::getAngularRateData(int *dddx, int *dddy, int *dddz){
  // Check the pointers
  if(!dddx || !dddy || !dddz){
    m_logger->log(ERROR, "%s: Null pointer passed in", __FUNCTION__);
    return -1;
  }
  try {
    gyroL3G4200DgetX(dddx);
    gyroL3G4200DgetY(dddy);
    gyroL3G4200DgetZ(dddz);
  }
  catch (int &e){
    m_logger->log(ERROR, "%s: Error while getting gyro readings", __FUNCTION__);
    return -1;
  }
  
  return 0;

}


