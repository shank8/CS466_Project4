
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

#include "system_config.h"
#include <peripheral/i2c.h>

void setup_I2C(void);
BOOL getCurrentTemperature(float* pTemperature);
void getAverageTemperature(float* avg);




// CLS Constants
#define TMP2_I2C_BUS 			I2C2
#define TMP2_ADDRESS			0x4B	// 0b1001010 7-bits
#define I2C_CLOCK_FREQ          100000  // 100 khZ CLOCK FREQ
#define MSB_SHIFT_COUNTER       8
#define TMP_REG1                0x00
#define TMP_REG2                0x01
#define F_FACTOR1               9/5.0
#define F_FACTOR2               32
#define F_FACTOR3               0.0625
float totalTemperature = 0.0;
int    totalCount = 0;
BOOL ReceiveOneByte(UINT8* data, int regAddress);

/*************************************************************
 * Function:          setup_I2C                              *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:                                       *
 * Description: This function sets up I2C communication with *
 *              Pmod Tmp2                                    *
 * Input parameters: direction: direction of motor           *
 *                   (clockwise or anti clockwise            *
 * Returns:          none                                    *
 * Usages:           Used to communicate with Pmod Tmp2 to   *
 *                   get temperature                         *
 * Preconditions:    System is powered on                    *
 * Postconditions: I2C2 communication is setup               *
 *************************************************************/
void setup_I2C(void)
{
    UINT32 actualClock;
    // I2C Port #2 - connected to pmod TMP2
    // RA02 SCL2/RA2
    // RA03 SDA2/RA3
    PORTSetPinsDigitalOut (IOPORT_A, BIT_2 | BIT_3);

    // Set the I2C baudrate
    actualClock = I2CSetFrequency(TMP2_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
        DBPRINTF("Error: I2C2 clock frequency (%u) error exceeds 10%%.\n", (unsigned)actualClock);
    }

    // Enable the I2C bus
    I2CEnable(TMP2_I2C_BUS, TRUE);
}

/*************************************************************
 * Function:          StartTransfer                          *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:                                       *
 * Description: This routine starts (or restarts) a transfer *
 *              to the Tmp2, waiting (in a blocking loop)    *
 *              until the start (or re-start) condition has  *
 *              completed.                                   *
 * Input parameters: If FALSE, send a "Start" condition      *
 *                    - If TRUE, send a "Restart" condition  *
 * Returns:          If TRUE, start transfer is successful   *
 *                   If FALSE, start transfer is unsuccessful*
 * Usages:           Used to start a data/address transfer   *
 *                   get temperature                         *
 * Preconditions:    I2C module must have been initialized.  *
 * Postconditions:   The device is ready to send/receive data*
 *************************************************************/
BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(TMP2_I2C_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(TMP2_I2C_BUS) );

        if(I2CStart(TMP2_I2C_BUS) != I2C_SUCCESS)
        {
            DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(TMP2_I2C_BUS);

    } while ( !(status & I2C_START) );

    return TRUE;
}

/*************************************************************
 * Function:          TransmitOneByte                        *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:                                       *
 * Description: This transmits one byte to the Pmod Tmp2, and*
 *  reports errors for any bus collisions.                   *
 * Input parameters:  data    - Data byte to transmit        *
 * Returns:           TRUE    - Data was sent successfully   *
 *                    FALSE   - A bus collision occured      *
 * Usages:           Used to start a data/address transfer   *
 *                   get temperature                         *
 * Preconditions:    The transfer must have been previously  *
 *                   started.                                *
 * Postconditions:   1 byte of data is transferred else error*
 *************************************************************/
BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(TMP2_I2C_BUS));

    // Transmit the byte
    if(I2CSendByte(TMP2_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(TMP2_I2C_BUS));

    return TRUE;
}

/*************************************************************
 * Function:          StopTransfer                           *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description:This routine Stops a transfer to the Pmod Tmp2*
 *              waiting (in a blocking loop) until the Stop  *
 *              condition has completed.                     *
 *                                                           *
 * Input parameters: none                                    *
 *                   none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to stop transfer                   *
 * Preconditions: The I2C module must have been initialized  *
 *                 & a transfer started.                     *
 * Postconditions: Another transfered needs to be "started"  *
 *                                                           *
 *************************************************************/
void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(TMP2_I2C_BUS);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(TMP2_I2C_BUS);

    } while ( !(status & I2C_STOP) );
}

/*************************************************************
 * Function:          ReceiveOneByte                         *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description:This routine receives one byte from Pmod Tmp2 *
 *             It sends the device address and temperature   *
 *             register address and waits for the data reply.*
 *             It then stops the transfer after receiving the*
 *             temperature vale                              *
 * Input parameters: data - temperature byte                 *
 *                   regAddress - register address           *
 *                                                           *
 * Returns:          TRUE - if data read is successful       *
 *                   FALSE - if data read is unsuccessful    *
 * Usages:           Used to receive temperature bytes       *
 *                   from Pmod Tmp2                          *
 * Preconditions: The I2C module must have been initialized  *
 * Postconditions: Data is received                          *
 *************************************************************/
BOOL ReceiveOneByte(UINT8* data, int regAddress)
{
    UINT8               i2cData[3];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 DataSz;
    BOOL                Success = TRUE;
    int Index = 0;
    UINT8               i2cbyte;
    int i = 0;

    // Initialize the data buffer
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, TMP2_ADDRESS, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = regAddress;  // TMP2 reg location to read (high address byte)
    DataSz = 2;

    // Start the transfer to read the EEPROM.
    if( !StartTransfer(FALSE) )
    {
        while(1);
    }

    // Address the TMP2.
    Index = 0;
    while( Success & (Index < DataSz) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            Success = FALSE;
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(TMP2_I2C_BUS))
        {
            DBPRINTF("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
        }
    }

    // Restart and send the TMP2's internal address to switch to a read transfer
    if(Success)
    {
        // Send a Repeated Started condition
        if( !StartTransfer(TRUE) )
        {
            while(1);
        }

        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, TMP2_ADDRESS, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(TMP2_I2C_BUS))
            {
                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // Read the data from the desired address
    if(Success)
    {
        if(I2CReceiverEnable(TMP2_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            DBPRINTF("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            //while(i < 2)
            {
                while(!I2CReceivedDataIsAvailable(TMP2_I2C_BUS));
                i2cbyte = I2CGetByte(TMP2_I2C_BUS);
                *data = i2cbyte;
            }
        }
    }

    // End the transfer (stop here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }
    return Success;
}

/*************************************************************
 * Function:          getCurrentTemperature                  *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description: This routine reads the current temperature   *
 *              from pmodTmp2 and converts to fahrenheit and *
 *              returns it                                   *
 * Input parameters: *pTemprature - temperature in F         *
 *                                                           *
 * Returns:          TRUE - if data read is successful       *
 *                   FALSE - if data read is unsuccessful    *
 * Usages:           Used to receive temperature bytes       *
 *                   from Pmod Tmp2                          *
 * Preconditions: The I2C module must have been initialized  *
 * Postconditions: Data is received                          *
 *************************************************************/
BOOL getCurrentTemperature(float* pTemperature)
{
    I2C_7_BIT_ADDRESS   SlaveAddress;
    UINT8               i2cData[2] = {0, 0};
    int                 byteCount = 0;
    BOOL                Success = TRUE;
    UINT16              currentTemp = 0;
    BOOL isAck = FALSE;
    //
    // Send the address of pmod Tmp2 register
    //
    Success = ReceiveOneByte(&i2cData[0], TMP_REG1);
    Success = ReceiveOneByte(&i2cData[1], TMP_REG2);
    currentTemp = (i2cData[0] << 8)|i2cData[1];
    currentTemp = currentTemp >> 3;

    *pTemperature = currentTemp * F_FACTOR3;
    *pTemperature = (*pTemperature * F_FACTOR1) + F_FACTOR2;
    totalTemperature += *pTemperature;
    totalCount++;
    return Success;
}

/*************************************************************
 * Function:          getAverageTemperature                  *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description: This routine returns the average temperature *
 *                                                           *
 * Input parameters: avg - average temperature               *
 *                                                           *
 * Returns:          none                                    *
 * Usages:         To display the average temperature on CLS *
 * Preconditions: The I2C module must have been initialized  *
 *                                                           *
 * Postconditions: Average temperature is returned           *
 *                                                           *
 *************************************************************/
void getAverageTemperature(float * avg)
{
    if(avg)
    {
        *avg = totalTemperature/(float)totalCount;
    }
}
