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
#ifndef MOTOR_CONTROL_H
#define	MOTOR_CONTROL_H

#define PRESCALE               1

void set_hbridge(void);
void setup_switches(void);
void configure_interrupts(void);
void setup_IC2(void);
void setup_IC3(void);
void straight_line_control(void);
void right_angle_control(void);
void control_motor(void);
void display_statistics(void);
void control_motor_after_delay(void);
#endif