#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <I2Cdev.h>

/*###############################################################*/
/*                     !!!!!!!!!!!!!!!                           */
/*                     !!! WARNING !!!                           */
/*                     !!!!!!!!!!!!!!!                           */
/* The ms5611 altimeter is very sensitive to measure frequency.  */
/* So this library use TIMER2 interrupts to make I2C requests    */
/* at very stable frequency.                                     */
/* Therefore :                                                   */
/* 1) you can't use timer2 while using this library              */
/* 2) To use I2C with other device you need to link to a special */
/*    I2CDev library provided as I2CDev_ms5611. Try to make the  */
/*    I2C request as short as possible.                          */
/* 3) To get the higtest precision, init fastwire at 400Htz with */
/*    Fastwire::setup(400,0) before initializing the device.     */
/* From your point of view, you DON'T need to get the ms5611     */
/* data at stable frequency. The timer's interrupts update the   */
/* global variables for you. You can get the values when you     */ 
/* want.                                                         */ 
/*###############################################################*/

#define MS5611_ADDRESS (0x76)
#define MS5611_CMD_RESET (0x1E)
#define MS5611_CMD_READ_PROM (0xA2)
#define MS5611_CMD_CONV_D1 (0x46)
#define MS5611_CMD_CONV_D2 (0x58)
#define MS5611_CMD_ADC_READ (0x00)

#define MS5611_RESET_DELAY 3
#define MS5611_CONV_DELAY 9

#define MS5611_STEP_READ_TEMP 0
#define MS5611_STEP_READ_PRESSURE 1

/* with a 64 prescale, 125 ticks give 0.5 ms */
/* and 17 * 0.5 = 8.5 ms the conv delay */
#define MS5611_INTERRUPT_COMPARE 125
#define MS5611_INTERRUPT_TIME 17
#define MS5611_INTERRUPT_START_DELAY 1000

#define MS5611_BASE_SEA_PRESSURE 1013.25
// 10 meters correction
#define MS5611_PRESSURE_CORRECTION_FACTOR 0.9988151618

/* ################################ */

/* first init the altimeter */
void ms5611_init();

/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/* once the variometer is initialized */
/* you can't use i2c anymore without  */
/* locking the variometer             */
/* !! USE THE MS5611 i2cdev LIBRARY!! */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
void ms5611_lock();
void ms5611_release();

/* check if you have new data */
boolean ms5611_dataReady(void);

/* then compute compensated temperature, pressure and alti values */
void ms5611_updateData(void);

/* and finally get computed values */
double ms5611_getTemperature();
double ms5611_getPressure();
double ms5611_getAltitude();

/* you can change the base pressure at any time */
/* each step change =~ 10 meter */
void ms5611_setCurrentAltitude(double currentAlti); //if altitude is known

#endif
