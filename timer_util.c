#include "timer_util.h"
#include "FreeRTOS.h"
#include "semphr.h"


/*************************************************************
 * Function:          delay                                  *
 * Date Created:      03/02/2014                             *
 * Date Last Modified:03/02/2014                             *
 * Description: It provides required delay                   *
 *                                                           *
 * Input parameters:                                         *
 * period - no. of iterations to spin                        *
 * Returns:          none                                    *
 * Usages:           Used to provide delay                   *
 * Preconditions:                                            *
 * Postconditions:   delay is provided                       *
 *************************************************************/

void delay(unsigned long int period)
{
    unsigned long int count = 0;
    for(count = 0; count < 40 *period; count++)
    {
        asm("nop");
    }
}


