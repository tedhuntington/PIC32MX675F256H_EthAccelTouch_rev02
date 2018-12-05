/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"
#include "accel.h"  //acelerometers data structures (which I2C and external interrupts each accel uses)
#include "touchsensors.h"  //touch sensor data structures (which PORT PINs each touch sensor uses)

extern EthAccelStatus EAStatus; //status of EthAccelTouch PCB
extern SYSTEM_OBJECTS sysObj;
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************


void __ISR(_I2C_3_VECTOR, ipl4AUTO) _IntHandlerDrvI2CInstance0(void)
{
    DRV_I2C_Tasks(sysObj.drvI2C0);
 
}
     
 
   

void __ISR(_I2C_1_VECTOR, ipl4AUTO) _IntHandlerDrvI2CInstance1(void)
{
    DRV_I2C_Tasks(sysObj.drvI2C1);
 
}
     
 
   
void __ISR(_I2C_5_VECTOR, ipl4AUTO) _IntHandlerDrvI2CInstance2(void)
{
    DRV_I2C_Tasks(sysObj.drvI2C2);
 
}
     

 
void __ISR(_UART_3_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
//`void __ISR(_UART_3A_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);
    DRV_USART_TasksReceive(sysObj.drvUsart0);
}
 
 
 

 
 
 
void __ISR(_EXTERNAL_0_VECTOR, IPL4AUTO) _IntHandlerExternalInterruptInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_0);
}
void __ISR(_EXTERNAL_4_VECTOR, IPL4AUTO) _IntHandlerExternalInterruptInstance1(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_4);
}
void __ISR(_EXTERNAL_1_VECTOR, IPL4AUTO) _IntHandlerExternalInterruptInstance2(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
}
 

void __ISR(_TIMER_1_VECTOR, ipl5AUTO) IntHandlerDrvTmrInstance0(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr0);
}
 void __ISR(_ETH_VECTOR, ipl5AUTO) _IntHandler_ETHMAC(void)
{
    DRV_ETHMAC_Tasks_ISR((SYS_MODULE_OBJ)0);
}

 
 
void __ISR(_TIMER_3_VECTOR,ipl3AUTO)  Timer3_ISR(void)   //IPL=Interrupt Priority Level
{
    uint8_t i;

//    LATGbits.LATG8=!LATGbits.LATG8; //SCL4

    //Process Accelerometer polling
    //for each accel, if they are being monitored 
    //get their current x,y,z reading
    //compare to the last reading and the threshold
    //if greater than the threshold, send a UDP packet

    if (EAStatus.flags&ETHACCEL_STATUS_ACCEL_POLLING) {
        //Get_Accelerometer_Samples(ACCEL_STATUS_POLLING);  //only for polling, not single sample
        Get_Accelerometer_Samples();
    } //if (EAStatus.flags&ETHACCEL_STATUS_ACCEL_POLLING) {

    //I was processing interrupts in the ext int isrs,
    //but UDP packets are sent too quickly
    if (EAStatus.flags&ETHACCEL_STATUS_ACCEL_INTERRUPT) {
        Process_Accelerometer_Interrupt();
    } //if (EAStatus.flags&ETHACCEL_STATUS_ACCEL_POLLING) {

    if ((EAStatus.flags&ETHACCEL_STATUS_TOUCH_SENSOR_POLLING) ||
        (EAStatus.flags&ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT)) {
        IFS1bits.AD1IF=0; //clear AD1 interrupt flag
        IEC1bits.AD1IE=1; //enable AD1 interrupt
        AD1CON1bits.ASAM=1; //autostart sampling
    } //

    IFS0bits.T3IF=0; //clear interrupt- needs to be done early or else exceptions ccur -perhaps because other interrupts need to be serviced?
}	//void __ISR(_TIMER_3_VECTOR,IPL3AUTO) Timer3_ISR(void) {  //IPL=Interrupt Priority Level



/* This function is used by ETHMAC driver */
bool SYS_INT_SourceRestore(INT_SOURCE src, int level)
{
    if(level)
    {
        SYS_INT_SourceEnable(src);
    }

    return level;
}

/*******************************************************************************
 End of File
*/
