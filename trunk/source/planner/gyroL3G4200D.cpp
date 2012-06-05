#include "gyroL3G4200D.h"

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
int gyroL3G4200Dinit(acpStem *pStem){
  // setup the various control registers
  
  // CTRL_REG1
  aUInt8 reg_buffer[2] = {L3G4200D_CTRL_REG1, 0x0F}; // Normal power mode, all axes enabled
  
  aPacketRef regPacket = pStem->createPacket(L3G4200D_IIC_ADDR, 2, reg_buffer);
  pStem->sendPacket(regPacket);

  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200DreadTwoByteTwosCompliment(acpStem *pStem, unsigned int firstReg, int *reading){
  // Check the passed pointer
  if(!reading)
    return -1;
  
  short ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  // assert the MSB of the address to get the gyro 
	// to do slave-transmit register auto-increment
  aUInt8 reg_buffer[1] = {firstReg | (1 << 7)};
  
  aPacketRef regPacket = pStem->createPacket(L3G4200D_IIC_ADDR, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, L3G4200D_IIC_ADDR+1, 2, read_buffer);
  
  // value is stored as two byte two's compliment; stuff and extend
  ret_value = (short)((read_buffer[1] << 8) | read_buffer[0]);

  *reading = (int) ret_value;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200DgetX(acpStem *pStem, int *dddx){
  if(!dddx)
    return -1;
  
  if(0 != gyroL3G4200DreadTwoByteTwosCompliment(pStem, L3G4200D_OUT_X_L, dddx))
    return -1;
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200DgetY(acpStem *pStem, int *dddy){
  if(!dddy)
    return -1;

  if(0 != gyroL3G4200DreadTwoByteTwosCompliment(pStem, L3G4200D_OUT_Y_L, dddy))
    return -1;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
int gyroL3G4200DgetZ(acpStem *pStem, int *dddz){
  if(!dddz)
    return -1;
  
  if(0 != gyroL3G4200DreadTwoByteTwosCompliment(pStem, L3G4200D_OUT_Z_L, dddz))
    return -1;
  
  return 0;
}


