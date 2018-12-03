/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "driver/i2c/drv_i2c.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

    //define which accelerometer is being used:
#define USE_MPU6050 1
//#define USE_FXOS8700CQ 1
    
    
#define GetSystemClock() (200000000UL)/* Fcy = 200MHz */
#define us_SCALE   (GetSystemClock()/2000000)
#define ms_SCALE   (GetSystemClock()/2000)    

    
#define I2C_TIMEOUT 100 //100 milliseconds
#define I2C_INST_TIMEOUT    0x0 //I2C transaction timed out
#define I2C_INST_COMPLETE   0x1 //I2C transaction completed
#define I2C_INST_ERROR      0x2 //I2C transaction had error

#define ACCEL_POLL_SEND_SIZE 150  //size of buffer that is sent to client with accel polling samples
#define ACCEL_INTR_SEND_SIZE 150  //size of buffer that is sent to client with accel interrupt samples
//#define TOUCH_SEND_SIZE 150  //size of buffer that is sent to client with touch sensor samples
    
/* EthAccel Status*/    
#define ETHACCEL_STATUS_TOUCH_SENSOR_POLLING 0x1 //polling one or more touch sensors
#define ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT 0x2 //touch sensors interrupt
//todo: probably do away with the above flag, because touch sensor interrupts are polled with a timer enabling the adc interrupt
#define ETHACCEL_STATUS_ACCEL_POLLING 0x4 //polling one or more accelerometers
#define ETHACCEL_STATUS_ACCEL_INTERRUPT 0x8 //one or more accelerometers is set to send an interrupt
#define ETHACCEL_STATUS_GPS_ENABLED 0x10 //GPS is enabled
#define ETHACCEL_STATUS_SEND_ALL_GPS_DATA 0x20 //Send all GPS data is enabled
typedef struct {
    uint32_t flags;
}EthAccelStatus;

    

typedef enum
{
    APP_TCPIP_WAIT_FOR_IP,
    APP_TCPIP_OPENING_SERVER,
    APP_TCPIP_WAIT_FOR_CONNECTION,
    APP_TCPIP_SERVING_CONNECTION,
    APP_TCPIP_CLOSING_CONNECTION
    //APP_TCPIP_WAIT_FOR_RESPONSE           
} APP_TCP_SERVER_TXRX_STATES;

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
	APP_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */
	APP_STATE_ERROR
} APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    APP_STATES state;

    /* TODO: Define any additional data used by the application. */
    DRV_HANDLE handleUSART0;

    uint8_t                   usartBQRxData[APP_DRV_USART_BQ_RX_SIZE];
    //uint8_t                   usartBQRxData[DRV_USART_RCV_QUEUE_SIZE_IDX0];    
    uint8_t                   usartInst[100];  //tph stores accumulated inst from usart
    uint8_t                   usartCurIndex; //current index into usartInst
    size_t                    usartBQRxRead;
    size_t                    usartBQRxWritten;
    DRV_USART_BUFFER_HANDLE   usartBQRxBufferHandle;
    size_t                    usartBQTxWritten;
    DRV_USART_BUFFER_HANDLE   usartBQTxBufferHandle;

    
	TCP_SOCKET socket;
	TCP_PORT port;
	APP_TCP_SERVER_TXRX_STATES txrxTaskState;

    
    uint8_t ReturnInst[50];  //instruction to send to client using UDP
    uint8_t ReadBuffer[40];  //holds bytes received from I2C device
    uint32_t ReturnInstLen; //length of return instruction in bytes
    uint32_t ExpectedReadLen; //length in bytes expected to be read
    uint8_t LastAccelSample; //tells callback when to send UDP packet 

} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
void I2CCallBack_Accel0(DRV_I2C_BUFFER_EVENT event, void * context);
void I2CCallBack_Accel1(DRV_I2C_BUFFER_EVENT event, void * context);
void I2CCallBack_Accel2(DRV_I2C_BUFFER_EVENT event, void * context);
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );

static void BufferQueueEventHandler(DRV_USART_BUFFER_EVENT  bufEvent,
                                    DRV_USART_BUFFER_HANDLE bufHandle,
                                    uintptr_t               context);
static void USART_Task(void);
static void TCP_Server_TXRX_Task(void);
uint32_t ReadCoreTimer(void);
uint8_t WaitForI2C(DRV_HANDLE I2CHandle,DRV_I2C_BUFFER_HANDLE BufferHandle);
void DelayMs(unsigned long int msDelay);
void DelayUs(unsigned long int usDelay);
void Process_Instruction(uint16_t InstDataLen);


#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

