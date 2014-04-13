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
#include "system_config.h"
#include <peripheral/uart.h> // Enable UARTs 1 for reception and transmission of serial data


void setup_UART1 (void);
void initialize_CLS(void);
void display_string(const char *string);
void display_CLS(double speed, double distance,double totalTime,
        float currentTemp, float avgTemp);

#define DESIRED_BAUD_RATE      9600

// Globals for setting up pmod CLS
unsigned char enable_display[] = {27, '[', '3', 'e', '\0'};
unsigned char set_cursor[] = {27, '[', '1', 'c', '\0'};
unsigned char home_cursor[] = {27, '[', 'j', '\0'};
unsigned char wrap_line[] = {27, '[', '0', 'h', '\0'};
unsigned char second_line[] = {27, '[', '1',';','0','H','\0'};
/*************************************************************
 * Function:          setup_UART1                            *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function is for setting up UART1 with   *
 *              baudrate of 9600                             *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           It is used for displaying the speed and *
 *                   distance travelled by the robot         *
 *                   the left motor                          *
 * Preconditions:                                            *
 * Postconditions: UART1 is configured with baudrate 9600    *
 *************************************************************/
void setup_UART1 (void)
{
    // UART 1 port pins - connected to PC
    /* JE-01 CN20/U1CTS/RD14 		RD14
     * 	   JE-02 U1TX/RF8                       RF8
     *    JE-03 U1RX/RF2 		        RF2
     *    JE-04  U1RTS/BCLK1/CN21/RD15         RD15 */
    PORTSetPinsDigitalIn (IOPORT_F, BIT_2);
    PORTSetPinsDigitalOut (IOPORT_F, BIT_8);

    // OpenUART2( config1, config2, ubrg)
	OpenUART1 (UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS |
               UART_MODE_FLOWCTRL | UART_DIS_BCLK_CTS_RTS | UART_NORMAL_RX | UART_BRGH_SIXTEEN,
               UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_ENABLE | UART_INT_TX | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS	| UART_RX_OVERRUN_CLEAR,
			   mUARTBRG(40000000, DESIRED_BAUD_RATE));
}

/*************************************************************
 * Function:          initialize_CLS                         *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function is for setting up CLS with     *
 *              initial display conditions like enabing the  *
 *              display and setting the cursor to home       *
 *              position etc                                 *
 *                                                           *
 * Input parameters: none                                    *
 * Returns:          none                                    *
 * Usages:           It is used for displaying the speed and *
 *                   distance travelled by the robot         *
 *                   the left motor                          *
 * Preconditions:                                            *
 * Postconditions: The display properties of CLS is          *
 *                 configured properly                       *
 *************************************************************/
void initialize_CLS (void)
{
	putsUART1 (enable_display);
	putsUART1 (set_cursor);
	putsUART1 (home_cursor);
	putsUART1 (wrap_line);
}

/*************************************************************
 * Function:          displayString                          *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function string on UART 1               *
 *                                                           *
 *                                                           *
 * Input parameters: none                                    *
 *                   none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to display string on UART          *
 * Preconditions:                                            *
 * Postconditions:                                           *
 *                                                           *
 *************************************************************/
void display_string(const char *string)
{
    while(*string != '\0')
    {
        while(!UARTTransmitterIsReady(UART1))
            ;

        UARTSendDataByte(UART1, *string);

        string++;

        while(!UARTTransmissionHasCompleted(UART1));
    }
}

/*************************************************************
 * Function:          displayCLS                             *
 * Date Created:      02/22/2014                             *
 * Date Last Modified:02/22/2014                             *
 * Description: This function will display speed and distance*
 *              on CLS                                       *
 *                                                           *
 * Input parameters: none                                    *
 *                   none                                    *
 *                                                           *
 * Returns:          none                                    *
 * Usages:           Used to display speed and distance on   *
 *                   UART 1                                  *
 * Preconditions: UART 1 should be configured and initialized*
 * Postconditions: Speed and distance is displayed on CLS    *
 *                                                           *
 *************************************************************/
void display_CLS(double speed, double distance,double totalTime,
        float currentTemp, float avgTemp)
{
    unsigned char line1[0x27];
    unsigned char line2[0x27];

    sprintf(line1,"S%2.1f D%2.1f T%1.4f", speed, distance, totalTime);
    //sprintf(line1, "X:%2.1f Y:%2.1f Z:%2.1f", xAxis, yAxis, zAxis);
    sprintf(line2,"TE:%2.1f AT:%2.2f",  currentTemp, avgTemp);

    display_string(home_cursor);
    display_string(line1);
    display_string(second_line);
    display_string(line2);
}
