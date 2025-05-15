// Q-measurements routines 
#include "stm32f10x_conf.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "si570.h"
#include "detector_calibration.h"
#include "main.h"
#include "meas.h"
#include "ad9833.h"
#include "adf4107.h"
#include "adf4350.h"
#include "packet.h"
#include "stxetx.h"
#include "gpio.h"

extern volatile settings_t settings;
extern calibration_t calibration;

// Function makes fast scan of the resonator (step is approx. 1 MHz)
void fastScan(meastable_t* table, settings_t* settings)
{
 /*   long freq;
    int retval;

    table->count = 0;

    for (freq = settings->start_frequency; (freq <= settings->stop_frequency && table->count<MEAS_TABLE_SIZE); freq += settings->fast_step)
    {
        retval = getMeasurementResult(freq, &table->data[table->count]);
        if (retval != ERR_OK)
            continue;
        table->count++;    
    }      */
}

void fastScanPipeline(volatile settings_t* settings)
{
	meas_t meas;
    int timeout;
    unsigned long freq;
    unsigned long sampleFrequency;
    char scanNumber;
    meastype_t oldMeasType;
    float averagedSample;
    int preciseN=0;
    int preciseM=0;
    int preciseCorr;
    static int lastStopFrequency=0;

    setLeds(LEDS_GreenSlowBlink);

    clbReadFromFlash();  // Again: get current calibration data from FLASH before measurement starts

    // to avoid a "dead lock" when no valid value is stored in flash (i.e. 0xffffffff)
    if (calibration.pllDelay > PLLDELAY_MAX || calibration.pllDelay < 0)
    	calibration.pllDelay = PLLDELAY_MAX;

    sendMeasType(settings->scanning.meastype);
    oldMeasType = settings->scanning.meastype;

    freq = settings->scanning.start_frequency;

    configureGenerators(freq, calibration.tresholdFrequency);
    meas.frequency = freq;

	// wait until PLL is locked
    timeout = PLL_TIMEOUT;
	while (timeout--!=0 && !isGeneratorLocked(freq, calibration.tresholdFrequency));


	averagedSample = getOversampledAdcValue(settings->scanning.oversampling);
    meas.power = clbGetCalibratedPower(freq, averagedSample);
    if (settings->scanning.meastype==mtS21)  // choose between meas results given in dBm (power) or dB (S21)
    	meas.power = clbGetCalibratedS21(freq, meas.power);
    meas.scanNumber = 0;
    scanNumber = 0;

    do
    {
        freq += settings->scanning.fast_step;    // change frequency

        configureGenerators(freq, calibration.tresholdFrequency);

        if (settings->scanning.meastype != oldMeasType)  // reaction for meas-type change during scanning
        {
        	oldMeasType = settings->scanning.meastype;
        	sendMeasType(settings->scanning.meastype);
        }

        if (!meas.not_locked)
        {
        	if (settings->scanning.start_frequency != settings->scanning.stop_frequency)  // do not send results, when SPAN is set to 0
        	{
                meas.frequency = sampleFrequency;  // DEBUG
        		meas.power = clbGetCalibratedPower(meas.frequency, averagedSample);
                if (settings->scanning.meastype==mtS21)  // choose between meas results given in dBm (power) or dB (S21)
                	meas.power = clbGetCalibratedS21(meas.frequency, meas.power);
        		sendMeasurement2(&meas);         // send previous result
        	}
        }
        else
        	sendUncalPll();   // send packet indicating "out-of-lock"

        meas.frequency = freq;
        meas.scanNumber = scanNumber;

        // Additional delay for PLL fine stabilisation - can be changed with calibration procedure (so user also has such ability)
        timeout = calibration.pllDelay;
        while (timeout--)
        { }

        // wait until PLL is locked
        timeout = PLL_TIMEOUT;
    	while (timeout--!=0 && !isGeneratorLocked(freq, calibration.tresholdFrequency))
    		setLedRedOn();    // this is executed inside the loop to very little light in the "in-lock" state

    	setLedRedOff();

        if (timeout==-1)
            meas.not_locked = TRUE;
        else
        {
            meas.not_locked = FALSE;
            averagedSample = getOversampledAdcValue(settings->scanning.oversampling);
            sampleFrequency = freq;
        }


        // Loop condition
        // (freq<start_freq) may happen during range change made by user
        if (freq > settings->scanning.stop_frequency || freq < settings->scanning.start_frequency)
        {
        	freq = settings->scanning.start_frequency;
            scanNumber++;

            // Precise Scanning correction:
            if (freq < settings->scanning.start_frequency || settings->scanning.stop_frequency!=lastStopFrequency)
            {
            	preciseN = 0;
            	preciseM = 0;
            	lastStopFrequency = settings->scanning.stop_frequency;
            }
            preciseCorr = (reverseNBits(preciseM, preciseN-1)<<1)|0x01 ;
            freq +=  preciseCorr * settings->scanning.fast_step / (1<<preciseN);
            preciseM++;
            if (preciseM >= 1<<(preciseN-1))
            {
            	preciseM = 0;
            	preciseN++;
            	if (settings->scanning.fast_step < settings->scanning.preciseScanning*(1<<(preciseN-1)))
            	{
            		preciseN = 0;
            	}
            }

        }

    }
    while(settings->state == S_FASTSCAN);

    

}

// Automatically selects generator depending on the given frequency and switch-treshold (frequencies given in [kHz])
void configureGenerators(int frequency, int tresholdFrequency)
{
	int DDSfrequency;  // DDS frequency given in [0.01Hz] units
	int N;
	static int last_N=0;
	unsigned char divider=0;
	int delay;

	// Let's calculate integer N as
	//   N = fVCO[kHz] / fDDSdefault[Hz]
	// Exact value of DDS-generated reference is:
	//   fDDS[kHz] = fVCO[kHz] / N
	//   fDDS[0.01Hz] = fDDS[kHz] * 100000

#if PCB_VERSION==PCBVER_15G
	frequency /= 2;    //  because HMC531 has internal /2 prescaler
	#define R_COUNTER 4
#else
	#define R_COUNTER 2  //2
#endif

//	N = frequency / (DDS_CENTER_FREQ/1000);
//	DDSfrequency = (long long)frequency*100000 / N;
	while(((frequency<<divider)<2150000)&&(divider<5))  // 2150000 (previously 2100000, but there was a problem that VCO couldn't go so low with frequency - change done 12.12.2012) CHANGED from original 2200000 due to problems with VCO stability in the range between 4300...4400 MHz (and scaled down); reason of VCO unstability is unknown!!!
		divider++;

	N = (frequency<<divider) / (DDS_CENTER_FREQ/1000 / R_COUNTER);  // R=2 or R=4

#if PCB_VERSION==PCBVER_15G
	//setSi570Frequency(DDSfrequency*R_COUNTER);
	DDSfrequency = (long long)(frequency<<divider)*1000 / N;   /* Si570_setFreq() requires frequency given in Hz */
	Si570_setFreq(DDSfrequency*R_COUNTER);
#else
	DDSfrequency = (long long)(frequency<<divider)*100000 / N;
	setDDSfrequency(DDSfrequency*R_COUNTER);					/* setDDSfrequency() requires frequency given in 0.01Hz */
#endif

	if (frequency < tresholdFrequency) // ADF4350 should be choosen if frequency below treshold (about 4.4 GHz)
	{
		//setGeneratorSwitch(OUT4350);
		//set4107Power(DISABLE);   // DEBUG
		set4350Power(ENABLE);  // DEBUG
		if (N != last_N)      // Reconfigure PLL only when N (PLL settings) should be changed
		{
			if (frequency > MAX_FREQ_FOR_PRESC_45)
				set4350Pll(N, divider, PRESCALER89);
			else
				set4350Pll(N, divider, PRESCALER45);
			delay = 0x100;
			while(delay--);   // wait for PLL stabilisation after N change
		}
	}
	else				// ADF4107 when frequency over 4.4 GHz
	{
		//setGeneratorSwitch(OUT4107);
		//set4350Power(ENABLE);  // DEBUG
		set4107Power(ENABLE);   // DEBUG
		if (N != last_N)   // DEBUG
			set4107Pll(N, FALSE);

	}
	last_N = N;
}

// returns TRUE, when PLL (choosen depending on current frequency and treshold) is in "locked" stated
unsigned char isGeneratorLocked(int frequency, int tresholdFrequency)
{
	if (frequency < tresholdFrequency) // ADF4350 should is choosen
		return is4350Locked();
	else				// ADF4107 when frequency over 4.4 GHz
		return is4107Locked();
}
// Function starts new ADC conversion and returns sampled value
short getPwrAdcValue(void)
{
#define EOC 1

	ADC1->SR &= ~(1<<EOC);				/* Clear the EndOfConversion flag     */
    ADC1->CR2 |= 1 << 22;               /* Start new conversion               */
    while (!(ADC1->SR & (1 << EOC)));   /* wait until conversion has finished */

    return  ADC1->DR & 0x0FFF;   
}

float getOversampledAdcValue(int oversampling)
{
    int i;
    double sum = 0.0;     // TODO: mo¿e float by wystarczy³?

    if (oversampling>OVERSAMPLING_MAX)
		oversampling = OVERSAMPLING_MAX;
    if (oversampling<1)
    	oversampling = 1;

    i = oversampling;

    while(i--)
    	sum += getPwrAdcValue();

    return sum / oversampling * 10;
}


#define UREF 3.3
#define UCAL1 1.492
#define PCAL1 -50
#define UCAL2 0.565
#define PCAL2 -10

// Function converts ADC value to dBm with predefined calibration factor
float Adc2dBm(int adc)
{
	float u;
	u = UREF*adc/(1<<12);
	return (u*(PCAL2-PCAL1)-PCAL2*UCAL1+PCAL1*UCAL2)/(UCAL2-UCAL1);
}

// New version of SendMeasurement function (with STX/ETX packets)
void sendMeasurement2(meas_t* meas)
{
	const int DATA_SIZE = 8;

	  // DATA PACKET:
	  // [STX]              - packet start
	  // [status]			- scanNumber
	  // [type]				- ptCalibratedMeasDbm
	  // [length]           = 8
	  // [frequency] (4 bytes)  - int
	  // [adc - in dBm] 4 bytes - float
	  // [checksum]
	  // [ETX]

//	memcpy(data, meas, DATA_SIZE);

	stxetxSend(meas->scanNumber, ptCalibratedMeasDbm, DATA_SIZE, (unsigned char*)meas);

}

void sendUncalPll()
{
	stxetxSend(0, ptUncalPll, 0, 0);
}

// Sends given measurement result to PC host via UART (or USB emulated UART)
void sendMeasurement(meas_t* meas)
{
// PROWIZORKA
//    printf("%d, %f, %d\n", meas->frequency, Adc2dBm(meas->adc), meas->not_locked);   

  int i;
  unsigned char checksum=0;
  unsigned char packet[sizeof(meas_t)+2];  

  // DATA PACKET:
  // 0xff              - packet start
  // frequency         - [kHz]
  // adc               - [adc]
  // checksum          - sum (mod 256) of all previous bytes in the packet

  packet[0] = 0xff;    
  memcpy(packet+1, meas, sizeof(meas_t));     
//  memcpy(packet+1, meas->frequency, sizeof(meas->frequency));
//  memcpy(packet+1+sizeof(meas->frequency), meas->adc, sizeof(meas->adc));

  for (i=0; i<sizeof(meas_t)-1; i++)
    checksum += packet[i];

  packet[sizeof(meas_t)-1] = checksum;  

  for (i=0; i<sizeof(packet)-2; i++)    // It is not necessary to send meas->not_locked so I don't send this
  {
      USART_SendData(/*EVAL_COM1*/USART2, packet[i]);
   
      /* Loop until the end of transmission */
      while(USART_GetFlagStatus(/*EVAL_COM1*/ USART2, USART_FLAG_TXE) == RESET)
      {
      }
  }
}

// Sends to PC the typeof meas to be performed (as a result PC changes axis legend of the graph and clears receive buffer)
void sendMeasType(meastype_t mt)
{
	stxetxSend((int)mt, ptMeasType, 0, NULL);
}

void sendDataTable(meastable_t* table)
{
    int i;
    printf("WYSYLAM FASTSCAN\n");
/*    for (i=0; i<table->count; i++)
    {
        printf("%d, %d, %f\n", i, table->data[i].frequency, Adc2dBm(table->data[i].adc));
    }
 */   printf("KONIEC\n");
}



void sendAdcRawData()
{
	int adc, i;
	float adcf;
	unsigned char buffer[8];

	set4107Power(DISABLE);
	set4350Power(DISABLE);
	setLeds(LEDS_YellowFastBlink);

	while (settings.state==S_ADCRAW)
	{
		adcf = getOversampledAdcValue(OVERSAMPLING_MAX);
		adc = (int)adcf;
		//adc = getPwrAdcValue();
		memcpy(buffer, &adc, 4);
		memcpy(buffer+4, &adcf, 4);

		stxetxSend(0, ptAdcRawValue, 8, buffer); //(unsigned char*)&adc);
		for (i=0; i<10000; i++)
		{ asm("nop");}

	}
}

// Function reverses order of N lowest significant bits in given number
unsigned int reverseNBits(unsigned int x, int n)
{
	unsigned int h=0;
	int i=0;

	for (h=i=0; i<n; i++)
	{
		h = (h<<1) + (x & 1);
		x >>= 1;
	}

	return h;
}
