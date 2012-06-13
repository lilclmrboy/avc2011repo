#include "avc.h"
#include "accelerometer.h"


#define STEM_A2D_VREF       3.3f     // V
#define GRAVITATIONAL_CONST 9.80665f // (m/s^2)/g

/* From datasheet, page 12:
 VCC(3.6V) = 360 mV/g
 VCC(2.0V) = 195 mV/g
 thus
 VCC(3.3V) = 329.0625 mV/g   
 */
#define ADXL335_SCALE_FACTOR 0.3290625f // V/g
#define ADXL335_AVERAGE_SAMPLES 3
#define ADXL335_AVERAGE_DELAY 3


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// Accelerometer stuff
/////////////////////////////////////////////////////////////////////////////
accelerometerADXL335::accelerometerADXL335 (acpStem *pStem, aSettingFileRef settings)
: avcAccelerometer(pStem, settings)
{
	m_accelerometerHwType = kAccelerometer_LSM303DLM;
  
}

/////////////////////////////////////////////////////////////////////////////  
accelerometerADXL335::~accelerometerADXL335(){};


/////////////////////////////////////////////////////////////////////////////
int accelerometerADXL335::init(){
  // the ADXL335 does not require initialization
  
  return 0;
}
  
/////////////////////////////////////////////////////////////////////////////
// Read the accelerometer sensor data
// See the datasheet for more information
// http://www.analog.com/static/imported-files/data_sheets/ADXL335.pdf

int accelerometerADXL335::getAccelerometerReadings (float *ddx, float *ddy, float *ddz) {
  // check the pointers
  if(!ddx || !ddy || !ddz){
    m_logger->log(ERROR, "%s: null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  if(!ddx || !ddy || !ddz){
    m_logger->log(ERROR, "%s: Null pointer passed to getAccelerometerReadings", __FUNCTION__);
    return -1;
  }
  
#ifdef aACCEL_BULK_CAPTURE
  
  aUInt8 slot = 9;
  unsigned int samples = 50;
  unsigned short pace = 10;
  aStreamRef s = NULL;
  aErr e = aErrNone;
  aMemSize len = 0;
  char *pData;
  unsigned int index = 0;
  
  m_logger->log(INFO, "Starting bulk capture\n");
  
  // Tell the stem to fire off the readings
  m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_X_CHAN, slot, pace, samples);
  
  // Read the data back from the Stem
  // First create a memory buffer to write the capture to
  if (aStreamBuffer_Create(m_ioRef, 4092, &s, &e))
    throw acpException(e, "creating bulk read buffer");
  
  // Unload the slot data capture into a buffer
  m_pStem->unloadSlot(aUSBSTEM_MODULE, slot, s);
  
  // Put the data into the storage array
  if (aStreamBuffer_Get(m_ioRef, s, &len, &pData, &e))
    throw acpException(e, "getting buffer data");
  
  // Debug what we have read from the stem
  m_logger->log(INFO, "Retreived %d data points from bulk capture\n", len);
  
  // Once we get data back, then we can chip through it
  // The bulk raw data is stored in 2 byte pairs creating a 16 bit word
  if (len > 1) {
    
    for (unsigned int i = 0; i < samples; i++) {
      m_logger->log(DEBUG, "[%d]: %d\n", 
                    i, 
                    pData[index] << 8 + pData[index + 1]);
      index+=2;
    }
    
  }
  
  // Clean up the stream reference
  if (aStream_Destroy(m_ioRef, s, &e))
    throw acpException(e, "destroying bulk stream");
  
  // let us know we are finished
  m_logger->log(INFO, "Ending bulk capture\n");
  
#endif // end of buik capture
  
  // Scaling is radiometric
  // ((m/s^2 / g) * V) / (V/g) = m/sec^2
  float scale_factor = GRAVITATIONAL_CONST *
    STEM_A2D_VREF *
    (1.0f / ADXL335_SCALE_FACTOR);   
	
  //  we get 0 g from the sensor
  //  when less than half vreferenc, we get negative acceleration
  //  when greater than half vreference, we get positive accel
  
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  
  // Get the readings from the Stem. Running average if we want
  int nSamples = ADXL335_AVERAGE_SAMPLES;
  do {
    
    // A2D delay if doing more than one sample
    if (ADXL335_AVERAGE_SAMPLES > 1)
      m_pStem->sleep(ADXL335_AVERAGE_DELAY);
    
    // Add up some readings from each channel
    ax += m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_X_CHAN) - 0.5f;
    ay += m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Y_CHAN) - 0.5f;
    az += m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Z_CHAN) - 0.5f;
    
  } while (--nSamples);
  
  // Compute the running average result
  ax = ax / (float) ADXL335_AVERAGE_SAMPLES;
  ay = ay / (float) ADXL335_AVERAGE_SAMPLES;
  az = az / (float) ADXL335_AVERAGE_SAMPLES;

  
  // Ship the result off to who ever wants it
  *ddx = scale_factor * ax;
  *ddy = scale_factor * ay;
  *ddz = scale_factor * az;
  
  if (-1 == *ddx || -1 == *ddy || -1 == *ddz){
    m_logger->log(ERROR, "%s: A2D timeout while reading accelerometer", __FUNCTION__);
    return -1;
  }
  
  return 0;
  
}  


