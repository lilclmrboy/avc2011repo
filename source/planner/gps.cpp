#include "gps.h"
#include "logger.h"
////////////////////////////////////////////////////////////////////////////
// This is a private static member of the gps class and cannot be 
// accessed directly. To access the gps, use gps::getInstance();
gps* gps::m_pInstance = NULL;

///////////////////////////////////////////////////////////////////////////
// This is simple, In a multithreaded environment we'd need a handle on a 
// lock before entering this function.
gps*
gps::getInstance() {
	
	if (!m_pInstance) //only one instance is ever created.
		m_pInstance = new gps;
	
	return m_pInstance;
	
}


/////////////////////////////////////////////////////////////////////////////

aErr gps::init(const acpString& port, const int baudrate) {
	
    //Start up the serial stream.

    aErr e = aErrNone;

    if(!m_bInit) {
        m_log = logger::getInstance();

        if (aStream_CreateSerial(m_ioRef, port, baudrate, &m_serialStream, &e))
            m_log->log(ERROR, "Error creating serial stream %d", e);
        m_bInit = true;
    }

    return e;
}


aErr gps::getPosition(float *lon, float* lat, float* heading) {
    m_gps.f_get_position(lat, lon);
    (*heading) = m_gps.f_course();

    if((*lat) == TinyGPS::GPS_INVALID_ANGLE
       || (*lon) == TinyGPS::GPS_INVALID_ANGLE
       || (*heading) == TinyGPS::GPS_INVALID_ANGLE)
      return aErrNotReady;
    else
      return aErrNone;
}

// Returns the HDOP as an indicator of quality
unsigned int gps::getQuality(){
  unsigned long hdop = m_gps.hdop();

  m_log->log(DEBUG, "HDOP from gps: %f", hdop / 100.0f);

  // No valid sentences, or HDOP > 10.0
  if (hdop == TinyGPS::GPS_INVALID_HDOP || hdop > 1000)
    return 0;
  else if (hdop > 200) // HDOP in bad range > 3.0
    return 1;
  else // HDOP good.
    return 2;
}

/////////////////////////////////////////////////////////////////////////////

int gps::run(void) {
    aErr e = aErrNone;
    while (m_bRunning) {
        char s;
        aStream_Read(m_ioRef, m_serialStream, &s, 1, &e);
        if(e == aErrNone) {
            m_gps.encode(s);
            //m_log->log(RAW, "%c", s);
        } else if(e != aErrNotReady) {
          m_bRunning = false;
          return aErrIO;
        }
    }
    m_bRunning = false;
	return 0;
}

#if 0
/////////////////////////////////////////////////////////////////////////////
int aGPM_Get2DigitInt(acpStem* pStem, char addr, char reg)
{
  int ret_value = 0;
  aUInt8 read_buffer[2]={0,0};
  aUInt8 reg_buffer[1] = {reg};
  
  aPacketRef regPacket = pStem->createPacket(addr, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, addr+1, 2, read_buffer);

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
  pStem->IIC_RD(aUSBSTEM_MODULE, addr+1, 3, read_buffer);
  
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
  pStem->IIC_RD(aUSBSTEM_MODULE, addr+1, 4, read_buffer);
  
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
  pStem->IIC_RD(aUSBSTEM_MODULE, addr+1, 1, read_buffer);

  ret_value = read_buffer[0];

  return ret_value;
}

int aGPM_GetGPSQuality(acpStem* pStem)
{
  int ret_value = 0;
  aUInt8 read_buffer[1] = {0};
  aUInt8 reg_buffer[1] = {33};
  
  aPacketRef regPacket = pStem->createPacket(GPS_IIC_ADDR, 1, reg_buffer);
  pStem->sendPacket(regPacket);
  pStem->IIC_RD(aUSBSTEM_MODULE, GPS_IIC_ADDR+1, 1, read_buffer);
  
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
#endif

#ifdef GPS_TEST

////////////////////////////////////////
// main testing routine for motModule
int
main(int argc,
     const char* argv[])
{

   aIOLib ioRef;
   aIO_GetLibRef(&ioRef, NULL);

  acpThread* gpsThread = acpOSFactory::thread("gps");
  logger*  mlog= logger::getInstance();
  mlog->log(INFO, "Starting GPS test");
  gps* gps = gps::getInstance();
  gps->init("ttyUSB1", 57600);
  gpsThread->start(gps);
  float lat = 0.0f, lon = 0.0f, head = 0.0f;
  while(1) {
      gps->getPosition(&lon, &lat, &head);
      mlog->log(INFO, "Lon, Lat, Head: %lf, %lf, %lf", lon,lat,head);
    aIO_MSSleep(ioRef, 200, NULL);
  }

  aIO_ReleaseLibRef(ioRef, NULL);

}

#endif

