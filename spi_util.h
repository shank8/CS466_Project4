/*******************************************************************************
 * Programmer: Shwetha Niddodi                                                 *
 * Class: CptS 466                                                             *
 * Lab Project: Project 3                                                      *
 * Date: 03/02/2014                                                            *
 *                                                                             *
 * Description:  This project captures the temperature from Pmod Tmp2 using    *
 *               I2C and displays current and average temperature on Pmod CLS  *
 *               using SPI. It is continuation of Project 2                    *
 ******************************************************************************/
#ifndef SPI_UTIL_H
#define	SPI_UTIL_H

#include "system_config.h"
#include <peripheral/spi.h>

void setup_SPI2(void);
void initializeCLS_SPI2(void);
void display_SPI2(double speed, double distance, double totalTime, float currentTemp, float avgTemp);

#endif	/* SPIUTIL_H */

