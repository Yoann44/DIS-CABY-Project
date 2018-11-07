/***********************************************************************
 * This file provides a basic template for writing dsPIC30F trap       *
 * handlers in C language for the C30 compiler                         *
 *                                                                     *
 * Add this file into your MPLAB project. Build your project, program  *
 * the device and run. If any trap occurs during code execution, the   *
 * processor vectors to one of these routines.                         *
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
 * dsPIC 30F General Purpose and Sensor Families                       *
 *                                           Data Sheet   : DS70083    *
 * dsPIC 30F Programmer's Reference Manual                : DS70030    *
 *                                                                     *
 * Template file has been compiled with MPLAB C30 v1.10.               *
 *                                                                     *
 ***********************************************************************
 *                                                                     *
 *    Author:                                                          *
 *    Company:                                                         *
 *    Filename:       traps.c                                          *
 *    Date:           09/12/2004                                       *
 *    File Version:   1.00                                             *
 *    Devices Supported: All dsPIC30F devices                          *
 *                                                                     *
 **********************************************************************/

        #include "p30fxxxx.h"



/* ****************************************************************
* Standard Exception Vector handlers if ALTIVT (INTCON2<15>) = 0  *
*                                                                 *
* Not required for labs but good to always include                *
******************************************************************/
void _ISR _OscillatorFail(void)
{

        INTCON1bits.OSCFAIL = 0;
        while(1);
}

void _ISR _AddressError(void)
{

        INTCON1bits.ADDRERR = 0;
        while(1);
}

void _ISR _StackError(void)
{

        INTCON1bits.STKERR = 0;
        while(1);
}

void _ISR _MathError(void)
{

        INTCON1bits.MATHERR = 0;
        while(1);
}




/* ****************************************************************
* Alternate Exception Vector handlers if ALTIVT (INTCON2<15>) = 1 *
*                                                                 *
* Not required for labs but good to always include                *
******************************************************************/
void _ISR _AltOscillatorFail(void)
{

        INTCON1bits.OSCFAIL = 0;
        while(1);
}

void _ISR _AltAddressError(void)
{

        INTCON1bits.ADDRERR = 0;
        while(1);
}

void _ISR _AltStackError(void)
{

        INTCON1bits.STKERR = 0;
        while(1);
}

void _ISR _AltMathError(void)
{

        INTCON1bits.MATHERR = 0;
        while(1);
}



