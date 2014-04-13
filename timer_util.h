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
#ifndef TIMER_UTIL_H
#define	TIMER_UTIL_H

#include <peripheral/timer.h> // Timers 1 - 5
#include "system_config.h"
#define TOGGLES_PER_SEC        1
#define TIMER3_PRESCALE        256
#define TIME_STEP              128E-3
#define T1_TICK               (SYS_FREQ/PB_DIV/TIMER3_PRESCALE/TOGGLES_PER_SEC)
#define T4_TICK               (SYS_FREQ/PB_DIV/TIMER3_PRESCALE)*TIME_STEP



void delay(unsigned long int period);
void setup_timer2(void);
void setup_timer3(void);
void setup_timer4(void);
void setup_timer5(void);

#endif	/* TIMERUTIL_H */

