#include "gps.h"

/////////////////////////////////////////////////////////////////////////////
int aGPM_Get2DigitInt(acpStem* pStem, char addr, char reg)
{
  int ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  aUInt8 reg_buffer[1] = {reg};
  
  aPacketRef regPacket = pStem->createPacket(addr, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, addr, 2, read_buffer);

  ret_value = (int)(read_buffer[0]*10 + read_buffer[1]);
  return ret_value;
}

int aGPM_Get3DigitInt(acpStem* pStem, char addr, char reg)
{
  int ret_value=0;
  aUInt8 read_buffer[3]={0,0,0};
  aUInt8 reg_buffer[1] = {reg};
  
  aPacketRef regPacket = pStem->createPacket(addr, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, addr, 3, read_buffer);
  
  /* typecasts avoid overflow */
  ret_value = ((int)read_buffer[0])*100 + ((int)read_buffer[1])*10 + (int)read_buffer[2];
  return ret_value;
}

int aGPM_Get4DigitInt(acpStem* pStem, char addr, char reg)
{
  int ret_value=0;
  aUInt8 read_buffer[4] = {0,0,0,0};
  aUInt8 reg_buffer[1] = {reg};
  
  aPacketRef regPacket = pStem->createPacket(addr, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, addr, 4, read_buffer);
  
  /* typecasts avoid overflow */
  ret_value = ((int)read_buffer[0])*1000 + ((int)read_buffer[1])*100 + ((int)read_buffer[2])*10 + (int)read_buffer[3];
  return ret_value;
}

int aGPM_GetAltitude(acpStem* pStem, char addr)
{
  int altm;
  altm =   aGPM_Get2DigitInt(pStem, addr, 39) * 1000
  + aGPM_Get3DigitInt(pStem, addr, 41);
  return altm;
}

int aGPM_GetAnalogInput(acpStem* pStem, char addr, int n)
{
  int ret_value = 0;
  aUInt8 read_buffer[1] = {0};
  aUInt8 reg_buffer[1] = {107 + n};
  
  aPacketRef regPacket = pStem->createPacket(addr, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, addr, 1, read_buffer);

  ret_value = read_buffer[0];

  return ret_value;
}


int aGPM_GetHours(acpStem* pStem){ return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 0);}
int aGPM_GetMinutes(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 2);}
int aGPM_GetSeconds(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 4);}
int aGPM_GetDay(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 6);}
int aGPM_GetMonth(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 8);}
int aGPM_GetYear(acpStem* pStem){return aGPM_Get4DigitInt(pStem, GPS_IIC_ADDR, 10);}

int aGPM_GetLatitudeDegrees(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 14);}
int aGPM_GetLatitudeMinutes(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 16);}
int aGPM_GetLatitudeFrac(acpStem* pStem){return aGPM_Get4DigitInt(pStem, GPS_IIC_ADDR, 18);}

int aGPM_GetLongitudeDegrees(acpStem* pStem){return aGPM_Get3DigitInt(pStem, GPS_IIC_ADDR, 23);}
int aGPM_GetLongitudeMinutes(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 26);}
int aGPM_GetLongitudeFrac(acpStem* pStem){return aGPM_Get4DigitInt(pStem, GPS_IIC_ADDR, 28);}

int aGPM_GetSatellitesInUse(acpStem* pStem){return aGPM_Get2DigitInt(pStem, GPS_IIC_ADDR, 34);}
int aGPM_GetHDOP(acpStem* pStem){return aGPM_Get3DigitInt(pStem, GPS_IIC_ADDR, 36);}

int aGPM_GetTrueHeading(acpStem* pStem){return aGPM_Get3DigitInt(pStem, GPS_IIC_ADDR, 44);}
int aGPM_GetMagHeading(acpStem* pStem){return aGPM_Get3DigitInt(pStem, GPS_IIC_ADDR, 48);}

int aGPM_GetSpeed(acpStem* pStem){return aGPM_Get3DigitInt(pStem, GPS_IIC_ADDR, 52);}




