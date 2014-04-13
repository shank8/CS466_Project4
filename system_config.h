/*******************************************************************************
 * Programmer: Shwetha Niddodi                                                 *
 * Class: CptS 466                                                             *
 * Lab Project: Project 3                                                      *
 * Date: 02/26/2014                                                            *
 *                                                                             *
 * Description: This project is required to move the robot at precise speed    *
 *              and distance. If switch 1 is pressed, the robot moves 10 feet  *
 *              distance and then another fifteen feet. If switch 2 is pressed,*
 *              the robot moves 5 feet, makes 90 degree turn and then moves    *
 *              another 5 feet.The distance travelled and speed of the robot is*
 *              displayed on the motor.                                        *
 ******************************************************************************/
#ifndef SYSTEM_CONFIG_H
#define	SYSTEM_CONFIG_H

#include <peripheral/ports.h> // PORTSetPinsDigitalIn (), ConfigINT (), etc.
#include <peripheral/int.h> // Enable interrupts - INTEnableInterrupts()
#include <peripheral/uart.h> // Enable UARTs 1 for reception and transmission of serial data
#include <peripheral/system.h>	// Set up the system and perihperal clocks for best performance
#include <proc/p32mx460f512l.h> // Vector table constants
#include <sys/attribs.h> // Very important for ISR...

#define SYS_FREQ             (80000000L)
#define PB_DIV                 8
#define PB_CLOCK              SYS_FREQ/PB_DIV
#define TRUE                    1
#define FALSE                   0
#define PWM_PERIOD              0x3e7

#define GetPeripheralClock()    (PB_CLOCK)
#endif	/* SYSTEMCONFIG_H */

