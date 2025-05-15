#ifndef _MEAS_H_
#define _MEAS_H_

// Maximum factor of oversampling
#define OVERSAMPLING_MAX 10000

// Maximum reasonable value of calibration.pllDelay - not really important, it only avoids system seemed "dead" if no value is stored in FLASH (ie 0xFFFFFFFF)
#define PLLDELAY_MAX 20000

void fastScan(meastable_t* table, settings_t* settings);
void fastScanPipeline(volatile settings_t* settings);


retval_t getMeasurementResult(long frequency, meas_t* result);
void sendMeasType(meastype_t mt);
void sendDataTable(meastable_t* table);
void sendUncalPll();
void sendMeasurement2(meas_t* meas);
void sendMeasurement(meas_t* meas);
float Adc2dBm(int adc);

unsigned char isGeneratorLocked(int frequency, int tresholdFrequency);
void configureGenerators(int frequency, int tresholdFrequency);

float getOversampledAdcValue(int oversampling);
short getPwrAdcValue(void);
void sendAdcRawData();
unsigned int reverseNBits(unsigned int x, int n);


#endif /* _MEAS_H_ */
