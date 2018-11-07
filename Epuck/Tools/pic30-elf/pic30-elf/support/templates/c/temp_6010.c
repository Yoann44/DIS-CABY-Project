/***********************************************************************
 * This file is a basic template for creating C code for a dsPIC30F    *
 * device.  Copy this file into your project directory and modify or   *
 * add to it as needed.                                                *  
 * Add the suitable linker script (e.g., p30f6010.gld) to the project. *
 *                                                                     *
 * If interrupts are not used, all code presented for that interrupt   *
 * can be removed or commented out with C-style comment declarations.  *
 *                                                                     *
 * For additional information about dsPIC architecture and language    *
 * tools, refer to the following documents:                            *
 *                                                                     *
 * MPLAB C30 Compiler User's Guide                        : DS51284    *
 * dsPIC 30F MPLAB ASM30, MPLAB LINK30 and Utilites                    *
 *                                           User's Guide : DS51317    *
 * Getting Started with dsPIC DSC Language Tools          : DS51316    *
 * dsPIC 30F Language Tools Quick Reference Card          : DS51322    *
 * dsPIC 30F 16-bit MCU Family Reference Manual           : DS70046    *
 * dsPIC 30F Motor Control and Power Conversion Family                 * 
 *                                             Data Sheet : DS70082    *
 * dsPIC 30F Programmer's Reference Manual                : DS70030    *
 *                                                                     *
 * Template file has been compiled with MPLAB C30 v1.30.               *
 *                                                                     *
 ***********************************************************************
 *                                                                     * 
 *    Author:                                                          *
 *    Company:                                                         * 
 *    Filename:       temp_6010.c                                      *
 *    Date:           8/20/2003                                        *
 *    File Version:   1.30                                             *
 *    Other Files Required: p30f6010.gld, libpic30.a                   *
 *    Tools Used: MPLAB GL  -> 6.60                                    *
 *                Compiler  -> 1.30                                    *
 *                Assembler -> 1.30                                    *
 *                Linker    -> 1.30                                    *
 *                                                                     *
 *    Devices Supported:                                               *
 *                dsPIC30F2010                                         *
 *                dsPIC30F3010                                         *
 *                dsPIC30F4012                                         *
 *                dsPIC30F3011                                         *
 *                dsPIC30F4011                                         *
 *                dsPIC30F5015                                         *
 *                dsPIC30F6010                                         *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 * Other Comments:                                                     *
 *                                                                     *
 * 1) C attributes, designated by the __attribute__ keyword, provide a *
 *    means to specify various characteristics of a variable or        *
 *    function, such as where a particular variable should be placed   *
 *    in memory, whether the variable should be aligned to a certain   *
 *    address boundary, whether a function is an Interrupt Service     *
 *    Routine (ISR), etc. If no special characteristics need to be     *
 *    specified for a variable or function, then attributes are not    *
 *    required. For more information about attributes, refer to the    *
 *    C30 User's Guide.                                                *
 *                                                                     *
 * 2) The __space__(xmemory) and __space__(ymemory) attributes are     *
 *    used to place a variable in X data space and Y data space,       *
 *    respectively. Variables accessed by dual-source DSP instructions *
 *    must be defined using these attributes.                          *
 *                                                                     *
 * 3) The aligned(k) attribute, used in variable definitions, is used  *
 *    to align a variable to the nearest higher 'k'-byte address       * 
 *    boundary. 'k' must be substituted with a suitable constant       *
 *    number when the ModBuf_X(k) or ModBuf_Y(k) macro is invoked.     *
 *    In most cases, variables are aligned either to avoid potential   *
 *    misaligned memory accesses, or to configure a modulo buffer.     *                           
 *                                                                     *
 * 4) The __interrupt__ attribute is used to qualify a function as an  *
 *    interrupt service routine. An interrupt routine can be further   *
 *    configured to save certain variables on the stack, using the     *
 *    __save__(var-list) directive.                                    *
 *                                                                     *
 * 5) The __shadow__ attribute is used to set up any function to       *  
 *    perform a fast context save using shadow registers.              *
 *                                                                     *
 * 6) Note the use of double-underscores (__) at the start and end of  *
 *    all the keywords mentioned above.                                *
 *                                                                     *
 **********************************************************************/


/* Include the appropriate header (.h) file, depending on device used */
/* Example (for dsPIC30F3010): #include <p30f3010.h>                  */

/* Alternatively, the header file may be inserted from the Project    */
/* window in the MPLAB IDE                                            */                  

#include <p30f6010.h>


/* Define constants here                            */

#define CONSTANT1 10
#define CONSTANT2 20


/************* START OF GLOBAL DEFINITIONS **********/


/* Define arrays: array1[], array2[], etc.          */
/* with attributes, as given below                  */

/* either using the entire attribute                */
int array1[CONSTANT1] __attribute__((__space__(xmemory), __aligned__(32)));
int array2[CONSTANT1] __attribute__((__space__(ymemory), __aligned__(32)));

/* or using macros defined in the header file       */
int array3[CONSTANT1] _XBSS(32);
int array4[CONSTANT1] _YBSS(32);


/* Define arrays without attributes                 */

int array5[CONSTANT2];   /* array5 is NOT an aligned buffer */

/* ------------------------------------------------ */

/* Define global variables with attributes          */

int variable1 __attribute__((__space__(xmemory)));
int variable2 __attribute__((__space__(ymemory)));

/* Define global variables without attributes       */

int variable3;


/************** END OF GLOBAL DEFINITIONS ***********/



/************* START OF MAIN FUNCTION ***************/

int main ( void )
{


/* Code goes here                                   */


}



/****** START OF INTERRUPT SERVICE ROUTINES *********/

/* Replace the interrupt function names with the    */
/* appropriate names depending on interrupt source. */

/* The names of various interrupt functions for     */
/* each device are defined in the linker script.    */


/* Interrupt Service Routine 1                      */
/* No fast context save, and no variables stacked   */

void __attribute__((__interrupt__)) _ADCInterrupt(void)
{


/* Interrupt Service Routine code goes here         */
        
}


/* Interrupt Service Routine 2                      */
/* Fast context save (using push.s and pop.s)       */

void __attribute__((__interrupt__, __shadow__)) _T1Interrupt(void)
{

/* Interrupt Service Routine code goes here         */
        
}


/* Interrupt Service Routine 3: INT0Interrupt       */
/* Save and restore variables var1, var2, etc.      */

void __attribute__((__interrupt__(__save__(variable1,variable2)))) _INT0Interrupt(void)
{

/* Interrupt Service Routine code goes here         */

}


/********* END OF INTERRUPT SERVICE ROUTINES ********/
