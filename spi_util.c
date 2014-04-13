
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
#include "spi_util.h"
#include "pmodACL.h"

// Globals for setting up pmod CLS
// Globals for setting up pmod CLS
unsigned char enable_display_spi[] = {27, '[', '3', 'e', '\0'};
unsigned char set_cursor_spi[] = {27, '[', '1', 'c', '\0'};
unsigned char home_cursor_spi[] = {27, '[', 'j', '\0'};
unsigned char wrap_line_spi[] = {27, '[', '0', 'h', '\0'};
unsigned char second_line_spi[] = {27, '[', '1',';','0','H','\0'};


/*************************************************************
 * Function:          setup_SPI2                             *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description: It function is used to set up SPI2.          *
 *              Microcontroller is configured as the master. *
 *              Pmod CLS is the slave                        *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           It is used for displaying the speed,    *
 *                   distance travelled by the robot,        *
 *                   temperature and total running time of   *
 *                   the motors                              *
 * Preconditions: System is powered On                       *
 * Postconditions:   SPI2 is configured                      *
 *************************************************************/
void setup_SPI2(void)
{
    // Master Mode
    /* SDO2 - Output - RG8
       SDI2 - Input - RG7
       SCK2 - Ouput - RG6
       SS2  - Output - RG9 */

       /* Set up SPI 2 for communication */
       PORTSetPinsDigitalOut (IOPORT_G, BIT_6 | BIT_8 | BIT_9);
       PORTSetPinsDigitalIn (IOPORT_G, BIT_7);

       /* Setup interrupts EXT-INT1 */
       PORTSetPinsDigitalIn (IOPORT_E, BIT_8);

       PmodACLInitSpi(SPI_CHANNEL2, PB_CLOCK, 625000);
           //Place the PmodACL in measure mode
       PmodACLSetPowerCtl(SPI_CHANNEL2,PMODACL_BIT_POWER_CTL_MEASURE);
     //Bypass the FIFO
       PmodACLSetFIFOCtl(SPI_CHANNEL2,PMODACL_BIT_FIFO_CTL_BYPASS);
     //Set the data output rate to 100Hz
       PmodACLSetBwRate(SPI_CHANNEL2,PMODACL_BIT_BW_RATE_100HZ);

       PmodACLCalibrate(SPI_CHANNEL2, 100, PMODACL_CALIBRATE_Z_AXIS);

       
//       SpiChnOpen (SPI_CHANNEL2, SPI_OPEN_MSTEN | SPI_OPEN_MODE8 | SPI_OPEN_CKP_HIGH, 256);
//
//       // Create a falling edge pin SS to enable SPI mode
//       PORTSetBits (IOPORT_G, BIT_9);
//       delay (1000);
//       PORTClearBits (IOPORT_G, BIT_9);
}

/*************************************************************
 * Function:          initializeCLS_SPI2                     *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description: This function is for setting up CLS with     *
 *              initial display conditions like enabing the  *
 *              display and setting the cursor to home       *
 *              position etc                                 *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           It is used for displaying statistics    *
 * Preconditions:  SPI2 is configured                        *
 * Postconditions: The display properties of CLS is          *
 *                 configured properly                       *
 ************************************************************/
void initializeCLS_SPI2(void)
{
    SpiChnPutS (SPI_CHANNEL2, (int *)enable_display_spi, 4);
    SpiChnPutS (SPI_CHANNEL2, (int *)set_cursor_spi, 4);
    SpiChnPutS (SPI_CHANNEL2, (int *)home_cursor_spi, 3);
    SpiChnPutS (SPI_CHANNEL2, (int *)wrap_line_spi, 4);
}

/*************************************************************
 * Function:          display_SPI2                           *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description: It provides required delay                   *
 *                                                           *
 * Input parameters: speed - instantanious speed in mph      *
 *              distance - distance travelled in feet        *
 *              totalTime - total time in sec the motors     *
 *                          have been running                *
 *             currentTemp - current temperature in F        *
 *             avgTemp - average temperature since last reset*
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to display speed, distance, total  *
 *                   time of motors running, temperature     *
 * Preconditions: SPI2 is configured                         *
 * Postconditions: The strings are displayed in proper format*
 *************************************************************/
//void display_SPI2(double speed, double distance,double totalTime,
//        float currentTemp, float avgTemp)
//{
//    unsigned char line1[0x27];
//    unsigned char line2[0x27];
//
//    sprintf(line1,"S%2.1f D%2.1f T%1.4f", speed, distance, totalTime);
//    sprintf(line2,"TE:%2.1f AT:%2.2f",  currentTemp, avgTemp);
//
//    SpiChnPutS (SPI_CHANNEL2, (int *)home_cursor_spi, 3);
//    SpiChnPutS (SPI_CHANNEL2, (int *)line1, strlen(line1));
//    SpiChnPutS (SPI_CHANNEL2, (int *)second_line_spi, 6);
//    SpiChnPutS (SPI_CHANNEL2, (int *)line2, strlen(line2));
//}
//

/* -------------------------------------------------------------------- */
/*                                                                      */
/*  pmod_spi_common.c -- Implimentation for common Pmod Spi functions   */
/*                                                                      */
/*                                                                      */
/* -------------------------------------------------------------------- */
/*	Author: 	Ryan Hoffman											*/
/*              Copyright (C) 2011 Ryan Hoffman                         */
/************************************************************************/
/*  Module Description: 												*/
/*  Commnon SPI IO tables and functions are defined in this module.     */
/* -------------------------------------------------------------------- */
/*  Revision History:													*/
/*                                                                      */
/*  10/18/2011(RyanH):                                                  */
/*                                                                      */
/* -------------------------------------------------------------------- */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */
#include "pmodSPI_Common.h"


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

//SpiIO contains SPI port to IO_PORT/SS pin mappings for use in
//SPI IO operations such as driving SS high and low
static const SpiPortSS SpiIO[] = {
	{0,0},  // SPI 0 is invalid
#if  ((__PIC32_FEATURE_SET__ >= 300) && (__PIC32_FEATURE_SET__ <= 499))
	{IOPORT_D,BIT_9}, //SPI1
	{IOPORT_G,BIT_9}  //SPI2
#elif ((__PIC32_FEATURE_SET__ >= 500) && (__PIC32_FEATURE_SET__ <= 799))
	{IOPORT_D,BIT_9}, //SPI1
	{IOPORT_G,BIT_9},//SPI2/SPI2A
    {IOPORT_D,BIT_14},//SPI1A/SPI3
	{IOPORT_F,BIT_12} //SPI3A/SPI4
#endif
};

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/*  PmodSPISetSSLow
**
**	Synopsis:
**  Sets the slave select bit on the selected SPI channel to low
**  
**  Input: SpiChannel chn  - spi channel to set SS bit low on
**
**  Returns: none
*/
void PmodSPISetSSLow(SpiChannel chn)
{
	PORTClearBits(SpiIO[chn].portSS,SpiIO[chn].ssMask);
}

/*  PmodSPISetSSHigh
**
**	Synopsis:
**  Sets the slave select bit on the selected SPI channel to high
**
**  Input: SpiChannel chn  - spi channel to set SS bit high on
**
**  Returns: none
**
**	Errors:	none
*/
void PmodSPISetSSHigh(SpiChannel chn)
{
	PORTSetBits(SpiIO[chn].portSS,SpiIO[chn].ssMask);
}