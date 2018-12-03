/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

//#include <proc/p32mz0512efe064.h>
#include <proc/p32mx675f256h.h>

#include "app.h"
#include "robot_accelmagtouchgps_pic_instructions.h" //robot EthAccelTouch PCB instruction codes
#include "accel.h"  //acelerometers data structures (which I2C and external interrupts each accel uses)
#include "touchsensors.h"  //touch sensor data structures (which PORT PINs each touch sensor uses)
//#include "i2c_master.h" //I2C functions
#include "FXOS8700CQ.h"  //accelerometer+magnetometer address and registers
#include "MPU6050.h" //accelerometer+magnetometer address and registers
#include "system/console/src/sys_console_uart.h"  //for handle to uart

#if Linux
#include "osal/osal.h" //for TCPIP_NET_IF
#include "tcpip/src/link_list.h" //for TCPIP_NET_IF
#include "tcpip/src/tcpip_manager_control.h"  //for TCPIP_NET_IF
#endif
#if Win32
#include "osal\osal.h" //for TCPIP_NET_IF
#include "tcpip\src\link_list.h" //for TCPIP_NET_IF
#include "tcpip\src\tcpip_manager_control.h"  //for TCPIP_NET_IF
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/


extern CONS_UART_DATA consUartData;

APP_DATA appData;
uint8_t appWriteString[] = "Hello World";


static uint8_t usartBQTxData[] = "Test serial port Send\r\n";
static enum 
{
    USART_BQ_INIT,
    USART_BQ_WORKING,
    USART_BQ_DONE,
} usartBQState;

static char appMsgToClient[] = "Hello Client!\n\r";
static uint8_t appMsgFromClient[80];
static uint8_t appReadBuffer[80]; //tph

static TCPIP_NET_HANDLE    	app_netH;
static SYS_STATUS          	app_tcpipStat;
static int                 	app_nNets;


#define PCB_NAME_LENGTH 5
static const char *PCB_Name = "Accel";
//For instructions sent over the network (UDP) (and eventually possibly over USB)
#define INSTRUCTION_PORT_NUM    53510 //the port of the MAC sending instructions, and the port of this MAC receiving instructions
uint16_t TimerInterval; //the time (in us) of each timer2 interrupt

uint16_t UDPNumuint8_ts,InstDataLen;
uint8_t ReturnInst[80]; //currently just 50 bytes but probably will change

//Accelerometer variables:
uint8_t NumAccelerometers;//,NumEnabledAccelerometers;
AccelStatus Accel[MAX_NUM_ACCEL]; //status of each accelerometer
uint8_t IntAccel[5];  //for ext int isr- which accel uses a given ext int
uint8_t AccelTimerSend[ACCEL_POLL_SEND_SIZE];  //accel polling and interrupt packet data to send back to requester
uint32_t AccelTimerSendLen; //length of accel polling and interrupt send data packet

//Touch Sensor variables:
uint8_t NumTouchSensors,NumActiveTouchSensors;
TouchSensorStatus TouchSensor[MAX_NUM_TOUCH_SENSORS]; //status of each touch sensor
uint8_t ActiveTouchSensor[MAX_NUM_TOUCH_SENSORS]; //list of all active touch sensors in order, for a quick reference
uint32_t A2DCount; //counts up ADC interrupts

uint8_t TouchSensorSend[TOUCH_SENSOR_SEND_SIZE];  //touch sensor packet data to send back to requester
uint32_t TouchSensorSendLen; //length of touch sensor send data packet
#define GPS_SEND_SIZE 255
uint8_t GPSSend[GPS_SEND_SIZE];  //touch sensor packet data to send back to requester
uint32_t GPSSendLen; //length of touch sensor send data packet
EthAccelStatus EAStatus; //status of EthAccelTouch PCB

int sendoutput;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    uint8_t i;
    
    EAStatus.flags=0;

    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

       for(i=0;i<NumAccelerometers;i++) {
        //appData.handleI2C0 = DRV_HANDLE_INVALID;
        Accel[i].handleI2C=DRV_HANDLE_INVALID;
    } //for i
    //appData.handleTimer1 = DRV_HANDLE_INVALID;
    appData.handleUSART0 = DRV_HANDLE_INVALID;
    
    /* Initialize the receive buffer as owned by the Application with no data */
    appData.usartBQRxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;

    appData.usartBQTxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;
	appData.port = 53510;
 
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
        sendoutput=0;

//#if 0     
    Accel[0].flags|=ACCEL_STATUS_ENABLED;
    Accel[1].flags|=ACCEL_STATUS_ENABLED;
    Accel[2].flags|=ACCEL_STATUS_ENABLED;
//#endif 

    Initialize_Accelerometers();
    Initialize_TouchSensors();
    

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void)
{
    int i;
    bool appInitialized;

    /* Check the application's current state. */
    switch (appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
            appInitialized = true;
#if 0             
            for(i=0;i<NumAccelerometers;i++) {
                if (Accel[i].handleI2C == DRV_HANDLE_INVALID)
                {   //i=DRV_I2C_INDEX_0, DRV_I2C_INDEX_1, etc.
                    Accel[i].handleI2C = DRV_I2C_Open(i, DRV_IO_INTENT_EXCLUSIVE);
                    //Accel[i].handleI2C = DRV_I2C_Open(i, DRV_IO_INTENT_SHARED|DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);
                    //Accel[i].handleI2C = DRV_I2C_Open(i, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);
                    //DelayMs(100);
                    if (Accel[i].handleI2C==DRV_HANDLE_INVALID) {
                        SYS_CONSOLE_PRINT("DRV_I2C_Open failed for Accel %d\n\r",i);
                    } else {
                        SYS_CONSOLE_PRINT("DRV_I2C_Open succeeded for Accel %d\n\r",i);
                    }
                    //Accel[i].handleI2C = DRV_I2C_Open(0, DRV_IO_INTENT_EXCLUSIVE);

//#if 0 
                    switch(i) {
                        
                        case 0:
                            if (Accel[i].handleI2C!=DRV_HANDLE_INVALID) {
                                DRV_I2C_BufferEventHandlerSet(Accel[i].handleI2C,(DRV_I2C_BUFFER_EVENT_HANDLER)I2CCallBack_Accel0,i2cOpStatus);
                            }
                        break;
                        case 1:
                            if (Accel[i].handleI2C!=DRV_HANDLE_INVALID) {
                                DRV_I2C_BufferEventHandlerSet(Accel[i].handleI2C,(DRV_I2C_BUFFER_EVENT_HANDLER)I2CCallBack_Accel1,i2cOpStatus);
                            }
                        break;
                        case 2:
                            if (Accel[i].handleI2C!=DRV_HANDLE_INVALID) {
                                DRV_I2C_BufferEventHandlerSet(Accel[i].handleI2C,(DRV_I2C_BUFFER_EVENT_HANDLER)I2CCallBack_Accel2,i2cOpStatus);
                            }
                        break;
                    } //switch(i)
                            //#endif

                    //Initialize any enabled accelerometers
                    if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
                        //start each accelerometer
                        if (Reset_Accelerometer(i)) {
                            Accel[i].Threshold=DEFAULT_ACCEL_THRESHOLD; //set interrupt acceleration threshold (absolute or relative depending on flag)
                            Accel[i].flags|=ACCEL_STATUS_AUTOCALIBRATE; //autocalibrate is on by default
                            Initialize_Accelerometer(i);
                            //Accel[i].flags|=ACCEL_STATUS_INITIALIZED; //testing

                            // if accel was successfully initialized move to ready state
                            if (Accel[i].flags&ACCEL_STATUS_INITIALIZED) {
                              //  appData.i2cState[i] = APP_I2C_READY;
                                SYS_CONSOLE_PRINT("Initialized accelerometer %d.\r\n",i);
                            } else {
                                SYS_CONSOLE_PRINT("Failed to initialize accelerometer %d.\r\n",i);
                                //Accel[i].flags=ACCEL_STATUS_NOT_ENABLED;
                            }
                        } //if (Reset_Accelerometer(i)) {
                    } //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {

//                    Initialize_TouchSensors();
                    
                    appInitialized &= ( DRV_HANDLE_INVALID != Accel[i].handleI2C );
                        
                }
            } //for i
#endif 
#if 0             
            if (appData.handleTimer1 == DRV_HANDLE_INVALID)
            {
                appData.handleTimer1 = DRV_TMR_Open(APP_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                if (appData.handleTimer1!= DRV_HANDLE_INVALID) {
                    SYS_CONSOLE_PRINT("DRV_TMR_Open succeeded.\r\n");
                    //appInitialized &= ( DRV_HANDLE_INVALID != appData.handleTimer1 );
                } else {
                    SYS_CONSOLE_PRINT("DRV_TMR_Open failed.\r\n");
                    appInitialized=0;
                }
                
            }
#endif
            /* Open the USART driver if it has not been previously opened */
            if (appData.handleUSART0 == DRV_HANDLE_INVALID)
            {

                 //for Pic32MX EthAccel there is only 1 USART
                //and it is opened by the sys console driver
                //todo: probably should use a different driver instance?
//                appData.handleUSART0=consUartData.deviceHandle;

                appData.handleUSART0 = DRV_USART_Open(APP_DRV_USART, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                //appInitialized &= ( DRV_HANDLE_INVALID != appData.handleUSART0 );
                if (appData.handleUSART0!= DRV_HANDLE_INVALID) {
                    //cannot send text to console until we have a buffer callback
                    /* Establish the Buffer Queue event handler */
                    DRV_USART_BufferEventHandlerSet(appData.handleUSART0,
                                                    BufferQueueEventHandler,
                                                    NULL);

                    usartBQState = USART_BQ_INIT;
                    //U3TXREG=0x70;
                    //U2TXREG=0x70;
                    SYS_CONSOLE_PRINT("DRV_USART_Open succeeded.\r\n");
                    //appInitialized &= ( DRV_HANDLE_INVALID != appData.handleTimer1 );
                } else {
                    SYS_CONSOLE_PRINT("DRV_USART_Open failed.\r\n");
                    appInitialized=0;
                }

            }
            
            
            app_tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(app_tcpipStat < 0)
            {   // some error occurred
                appData.state = APP_STATE_ERROR;
				appInitialized = false;
            }
            else if(app_tcpipStat == SYS_STATUS_READY)
            {
                // now that the stack is ready we can check the
                // available interfaces
                app_nNets = TCPIP_STACK_NumberOfNetworksGet();
                for(i = 0; i < app_nNets; i++)
                {
                    app_netH = TCPIP_STACK_IndexToNet(i);
                }
                appData.txrxTaskState = APP_TCPIP_WAIT_FOR_IP;
            }
          

            if (appInitialized)
            {
                //TimerSetup();
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;


        case APP_STATE_SERVICE_TASKS:
		    /* Run the state machine for servicing I2C */
            USART_Task();
            TCP_Server_TXRX_Task();
            
            if (!sendoutput) {
                SYS_CONSOLE_MESSAGE("Test SYS_CONSOLE_MESSAGE\n\r");
                //SYS_CONSOLE_PRINT("Testing SYS_CONSOLE_PRINT\n\r");
                //SYS_DEBUG(0, "Test SYS_DEBUG\n\r"); //tph            
                //SYS_ERROR_PRINT(SYS_ERROR_ERROR, "Test SYS_ERROR_PRINT appData.state= %d\r\n", appData.state);
                sendoutput=1;
            }
        break;

        /* TODO: implement your application state machine.*/
        
        case APP_STATE_ERROR:
 		break;


        /* The default state should never be executed. */
        default:
            /* TODO: Handle error in application's state machine. */
       break;
    }
} //void APP_Tasks(void)



// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

//use  top 2 bits of accel number for: 00=single sample, 01=poll, 11=intr
//sample can be actual raw data and/or pitch, yaw, and roll


//note that all I2C0 (Accel0) writes and reads reach here)
//are initiated by (10ms) timer 
//for now do static - 3 callbacks for each i2c bus/accel
void I2CCallBack_Accel0(DRV_I2C_BUFFER_EVENT event, void * context)
{
    uint8_t RegAddr,ReturnValue;
    
    
    
    switch (event)
    {
        
        //SYS_CONSOLE_PRINT("%d\r\n",event);
/*
        case DRV_I2C_SEND_STOP_EVENT:              
            DRV_I2C_StopEventSend(Accel[0].handleI2C);
            SYS_CONSOLE_PRINT("stop\r\n");
            break;
        case DRV_I2C_SEND_RESTART_EVENT:      
            DRV_I2C_RestartEventSend(Accel[0].handleI2C);
            SYS_CONSOLE_PRINT("restart\r\n");
            break;
*/
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            
            //
            //SYS_CONSOLE_PRINT("C\r\n");
            //Get_Accelerometer_Samples() has already checked to make sure the accel is enabled
            //Timercallback checks EAStatus for polling or interrupting flags
            if (Accel[0].flags&ACCEL_STATUS_POLLING) {
                //SYS_CONSOLE_PRINT("cb0\r\n");
                //SYS_CONSOLE_PRINT("P\r\n");
                //accel is polling or has an interrupt
                //samples are already in the return buffer
                //in the datasheet example the status is never checked
                //perhaps just reading the reg is enough?
                //I decided to just check above to remove any doubt

                
                //if (AccelTimerSend[AccelTimerSendLen]&FXOS8700CQ_STATUS_ZYXDR) {
#if USE_FXOS8700CQ                
                if (Accel[0].Buffer[0]&FXOS8700CQ_STATUS_ZYXDR) {
                    ReturnValue=1; //data is ready
                } else {
                    ReturnValue=0;  //data is not ready
                }


                if (ReturnValue) {

                    //if (1) {
 
                    //Accel Data
                    //[0]=MSB 7:0 are 8 MSB [1]=LSB 7:2 are 6 LSB,
                    //ReturnSampleX=((I2CData[1]<<0x8)| I2CData[2])>>0x2; //14-bit data
                    //ReturnSampleY=((I2CData[3]<<0x8)| I2CData[4])>>0x2; //14-bit data
                    //ReturnSampleZ=((I2CData[5]<<0x8)| I2CData[6])>>0x2; //14-bit data
                    //lock
                    AccelTimerSendLen+=13;//7;
                    //unlock
                    
                    //add the accel # to the return UDP packet
                    AccelTimerSend[AccelTimerSendLen-13]=0;

                    //add the samples to the UDP packet
                    memcpy(AccelTimerSend+AccelTimerSendLen-12,&Accel[0].Buffer[1],12);                                                             

                } else { //if (ReturnValue) {
                    SYS_CONSOLE_PRINT("Accel 0 poll data not ready\n\r");
                    //just skip this accel then
                    break;  //exit switch
                } //if (ReturnValue) {
#endif //USE_FXOS8700CQ   
#if USE_MPU6050     
                AccelTimerSendLen+=15;
                //add the accel # to the return UDP packet
                AccelTimerSend[AccelTimerSendLen-15]=0;

                //add the samples to the UDP packet
                memcpy(AccelTimerSend+AccelTimerSendLen-14,&Accel[0].Buffer[0],14);

#endif //USE_MPU6050   

                //if only a single sample was requested,
                //reset the polling and single sample flags on this accel
                if (Accel[0].flags&ACCEL_STATUS_SINGLE_SAMPLE) { //clear single sample flag
                    Accel[0].flags&=~ACCEL_STATUS_SINGLE_SAMPLE; //clear single sample flag
                    Accel[0].flags&=~ACCEL_STATUS_POLLING; //clear polling flag too
                    
                    //only for single sample might we need to stop polling here
                    if (!(Accel[0].flags&ACCEL_STATUS_POLLING) &&
                        !(Accel[1].flags&ACCEL_STATUS_POLLING) &&
                        !(Accel[2].flags&ACCEL_STATUS_POLLING) ) {
                            //no accels are polling or interrupting so
                            //stop polling (getting samples each timer interrupt)
                            EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_POLLING;                        
                    } //if (Accel[0].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) {
                    
                } //if (Accel[0].flags&ACCEL_STATUS_SINGLE_SAMPLE) { //clear single sample flag
                
            //possible problem: other interrupt flag could be cleared by this time
            if (appData.LastAccelSample==0) {                
                //accels 1 and 2 are not polling and have no interrupts so
                //send the UDP packet now
                SendTimerUDPPacket();               
            } //if (!(Accel[1].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) &&
            break;     
        } //if (EAStatus.flags&ETHACCEL_STATUS_ACCEL_POLLING) {
#if USE_FXOS8700CQ      
        if (Accel[0].flags&ACCEL_STATUS_GOT_INTERRUPT) {
            //if not polling it can only be an interrupt
            //SYS_CONSOLE_PRINT("I\r\n");
            //accel is polling or has an interrupt
            //samples are already in the return buffer
            //in the datasheet example the status is never checked
            //perhaps just reading the reg is enough?
            //I decided to just check above to remove any doubt

            //if (AccelTimerSend[AccelTimerSendLen]&FXOS8700CQ_STATUS_ZYXDR) {
            if (Accel[0].Buffer[0]&FXOS8700CQ_STATUS_ZYXDR) {
                ReturnValue=1; //data is ready
            } else {
                ReturnValue=0;  //data is not ready
            }


            if (ReturnValue) {
                //SYS_CONSOLE_PRINT("IM\r\n");
                //Accel Data
                //[0]=MSB 7:0 are 8 MSB [1]=LSB 7:2 are 6 LSB,
                //ReturnSampleX=((I2CData[1]<<0x8)| I2CData[2])>>0x2; //14-bit data
                //ReturnSampleY=((I2CData[3]<<0x8)| I2CData[4])>>0x2; //14-bit data
                //ReturnSampleZ=((I2CData[5]<<0x8)| I2CData[6])>>0x2; //14-bit data
                //lock
                AccelTimerSendLen+=13;//7;
                //unlock

                //add the accel # to the return UDP packet
                AccelTimerSend[AccelTimerSendLen-13]=0;

                //add the samples to the UDP packet
                memcpy(AccelTimerSend+AccelTimerSendLen-12,&Accel[0].Buffer[1],12);                    


            } else { //if (ReturnValue) {
                SYS_CONSOLE_PRINT("Accel 0 intr data not ready\n\r");
                //just skip this accel then
            } //if (ReturnValue) {

#if 0               
            if (!(Accel[0].flags&ACCEL_STATUS_INTERRUPT)) &&
                !(Accel[1].flags&ACCEL_STATUS_INTERRUPT)) &&
                !(Accel[2].flags&ACCEL_STATUS_INTERRUPT)) ) {
                    //no accels are polling or interrupting so
                    //stop polling (getting samples each timer interrupt)
                    EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_INTERRUPT;                        
            } //if (Accel[0].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) {
#endif            

            if (appData.LastAccelSample==0) {                
                //accels 1 and 2 are not polling and have no interrupts so
                //send the UDP packet now
                //SYS_CONSOLE_PRINT("send\n\r");
                SendTimerUDPPacket();
            } else { //if (!(Accel[1].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) &&
                //SYS_CONSOLE_PRINT("a1=%d a2=%d\n\r",Accel[1].flags,Accel[2].flags);
            } 
            Accel[0].flags&=~ACCEL_STATUS_GOT_INTERRUPT; //clear interrupt flag                

        } else { //if (Accel[0].flags&ACCEL_STATUS_GOT_INTERRUPT) {
            //cleared accel flag
            if (Accel[0].flags&ACCEL_STATUS_CLEAR_INTERRUPT) {
                //so now get interrupt data
                Accel[0].flags&=~ACCEL_STATUS_CLEAR_INTERRUPT; 
                //Accel[0].flags|=ACCEL_STATUS_GOT_INTERRUPT;
            }
        }//if (Accel[0].flags&ACCEL_STATUS_GOT_INTERRUPT) {
#endif //#if USE_FXOS8700CQ      
        break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            SYS_CONSOLE_PRINT("I2C Error Accel 0\r\n");
            if (Accel[0].flags&ACCEL_STATUS_CLEAR_INTERRUPT) {
                Accel[0].flags&=~ACCEL_STATUS_CLEAR_INTERRUPT; 
            }
        break;
    } //switch(event)
}

void I2CCallBack_Accel1(DRV_I2C_BUFFER_EVENT event, void * context)
{
    uint8_t RegAddr,ReturnValue;
    
    
    switch (event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:    

            
            if (Accel[1].flags&ACCEL_STATUS_POLLING) {
                //SYS_CONSOLE_PRINT("cb1\r\n");
                //accel is polling or got interrupt
                //samples are already in the return buffer
                //in the datasheet example the status is never checked
                //perhaps just reading the reg is enough?
                //I decided to just check above to remove any doubt
#if USE_FXOS8700CQ            
                //if (AccelTimerSend[AccelTimerSendLen]&FXOS8700CQ_STATUS_ZYXDR) {
                if (Accel[1].Buffer[0]&FXOS8700CQ_STATUS_ZYXDR) {    
                    ReturnValue=1; //data is ready
                } else {
                    ReturnValue=0;  //data is not ready
                }


                if (ReturnValue) {
                //if (1) {
                    //Accel Data
                    //[0]=MSB 7:0 are 8 MSB [1]=LSB 7:2 are 6 LSB,
                    //ReturnSampleX=((I2CData[1]<<0x8)| I2CData[2])>>0x2; //14-bit data
                    //ReturnSampleY=((I2CData[3]<<0x8)| I2CData[4])>>0x2; //14-bit data
                    //ReturnSampleZ=((I2CData[5]<<0x8)| I2CData[6])>>0x2; //14-bit data

                    //lock AccelTimerSendLen
                    AccelTimerSendLen+=13;
                    //unlock AccelTimerSendLen
                    
                    //add the accel # to the return UDP packet
                    AccelTimerSend[AccelTimerSendLen-13]=1;

                    //add the samples to the UDP packet
                    memcpy(AccelTimerSend+AccelTimerSendLen-12,&Accel[1].Buffer[1],12);                    
                    
                    
                } else { //if (ReturnValue) {
                    SYS_CONSOLE_PRINT("Accel 1 poll data not ready\n\r");
                    break;  //exit switch
                } //if (ReturnValue) {
#endif //#endif //USE_FXOS8700CQ   
#if USE_MPU6050     
                AccelTimerSendLen+=15;
                //add the accel # to the return UDP packet
                AccelTimerSend[AccelTimerSendLen-15]=1;

                //add the samples to the UDP packet
                memcpy(AccelTimerSend+AccelTimerSendLen-14,&Accel[1].Buffer[0],14);

#endif //USE_MPU6050
            

                //if only a single sample was requested,
                //reset the polling and single sample flags on this accel
                if (Accel[1].flags&ACCEL_STATUS_SINGLE_SAMPLE) { //clear single sample flag
                    Accel[1].flags&=~ACCEL_STATUS_SINGLE_SAMPLE; //clear single sample flag
                    Accel[1].flags&=~ACCEL_STATUS_POLLING; //clear polling flag too

                    //we would only clear EAStatus polling flag here for single sample
                    if (!(Accel[0].flags&ACCEL_STATUS_POLLING) &&
                        !(Accel[1].flags&ACCEL_STATUS_POLLING) &&
                        !(Accel[2].flags&ACCEL_STATUS_POLLING) ) {
                            //no accels are polling or interrupting so
                            //stop polling (getting samples each timer interrupt)
                            EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_POLLING;                        
                    } //if (Accel[0].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) {                
                } //if (Accel[1].flags&ACCEL_STATUS_SINGLE_SAMPLE) { //clear single sample flag
 
         

            if (appData.LastAccelSample==1) {
                //accel 2 is not polling and has no interrupt so
                //send the UDP packet now
                SendTimerUDPPacket();                                    
            } //if (!(Accel[2].flags&ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) {
            break;
        } //if (Accel[1].flags&ACCEL_STATUS_POLLING) {
#if USE_FXOS8700CQ    
        if (Accel[1].flags&ACCEL_STATUS_GOT_INTERRUPT) {            
            //Accel 1 must have interrupt to get here

            //if (AccelTimerSend[AccelTimerSendLen]&FXOS8700CQ_STATUS_ZYXDR) {
            if (Accel[1].Buffer[0]&FXOS8700CQ_STATUS_ZYXDR) {    
                ReturnValue=1; //data is ready
            } else {
                ReturnValue=0;  //data is not ready
            }


            if (ReturnValue) {
                //lock AccelTimerSendLen
                AccelTimerSendLen+=13;
                //unlock AccelTimerSendLen

                //add the accel # to the return UDP packet
                AccelTimerSend[AccelTimerSendLen-13]=1;

                //add the samples to the UDP packet
                memcpy(AccelTimerSend+AccelTimerSendLen-12,&Accel[1].Buffer[1],12);                    


            } else { //if (ReturnValue) {
                SYS_CONSOLE_PRINT("Accel 1 intr data not ready\n\r");
            } //if (ReturnValue) {


            if (appData.LastAccelSample==1) {
                //accel 2 is not polling and has no interrupt so
                //send the UDP packet now
                SendTimerUDPPacket();                                    
            } //if (!(Accel[2].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_GOT_INTERRUPT)) {

            //clear interrupt flag - as late as possible because above comparisons use this flag
            Accel[1].flags&=~ACCEL_STATUS_GOT_INTERRUPT;                 
        } //if (Accel[1].flags&ACCEL_STATUS_GOT_INTERRUPT) {
#endif //FXOS_8700CQ    
        break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            SYS_CONSOLE_PRINT("I2C Error Accel 1\r\n");
        break;
    } //switch(event)    
}

void I2CCallBack_Accel2(DRV_I2C_BUFFER_EVENT event, void * context)
{
    uint8_t RegAddr,ReturnValue;
    
    
    switch (event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
            
    
            if (Accel[2].flags&ACCEL_STATUS_POLLING) { 
                //&&(!(Accel[2].flags&ACCEL_STATUS_PROCESSING))) {
                //SYS_CONSOLE_PRINT("cb2\r\n");
//                SYS_CONSOLE_PRINT("%d\r\n",event);
              
                //Accel[2].flags|=ACCEL_STATUS_PROCESSING;
                //accel is polling
             
#if FXOS8700CQ
                //if (AccelTimerSend[AccelTimerSendLen]&FXOS8700CQ_STATUS_ZYXDR) {
                if (Accel[2].Buffer[0]&FXOS8700CQ_STATUS_ZYXDR) {    
                    ReturnValue=1; //data is ready
                } else {
                    ReturnValue=0;  //data is not ready
                }


                if (ReturnValue) {
                //if (1) {
                    //lock AccelTimerSendLen
                    AccelTimerSendLen+=13;
                    //unlock AccelTimerSendLen
                    
                    //add the accel # to the return UDP packet
                    AccelTimerSend[AccelTimerSendLen-13]=2;
                    //add the samples to the UDP packet
                    memcpy(AccelTimerSend+AccelTimerSendLen-12,&Accel[2].Buffer[1],12);                    

                } else { //if (ReturnValue) {
                    SYS_CONSOLE_PRINT("Accel 2 poll data not ready\n\r");
                    break;  //exit switch                    
                } //if (ReturnValue) {
#endif //FXOS8700CQ
#if USE_MPU6050     
                AccelTimerSendLen+=15;
                //add the accel # to the return UDP packet
                AccelTimerSend[AccelTimerSendLen-15]=2;

                //add the samples to the UDP packet
                memcpy(AccelTimerSend+AccelTimerSendLen-14,&Accel[2].Buffer[0],14);

#endif //USE_MPU6050   
                
                //if only a single sample was requested,
                //reset the polling and single sample flags on this accel
                if (Accel[2].flags&ACCEL_STATUS_SINGLE_SAMPLE) { //clear single sample flag
                    Accel[2].flags&=~ACCEL_STATUS_SINGLE_SAMPLE; //clear single sample flag
                    Accel[2].flags&=~ACCEL_STATUS_POLLING; //clear polling flag too

                    //only need to clear EAStatus polling flag for single sample
                    if (!(Accel[0].flags&ACCEL_STATUS_POLLING) &&
                        !(Accel[1].flags&ACCEL_STATUS_POLLING) &&
                        !(Accel[2].flags&ACCEL_STATUS_POLLING) ) {
                            //no accels are polling so
                            //stop polling (getting samples each timer interrupt)
                            EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_POLLING;                        
                    } //if (!(Accel[0].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_INTERRUPT)) &&

                } //if (Accel[2].flags&ACCEL_STATUS_SINGLE_SAMPLE) { //clear single sample flag
                
                //since this is the last accel
                //send the UDP packet
                SendTimerUDPPacket();
//#endif //0 for testing                
            break;    
        } //if (Accel[2].flags&ACCEL_STATUS_POLLING) {
#if FXOS8700CQ            
        if (Accel[2].flags&ACCEL_STATUS_INTERRUPT) {
            //accel got interrupt

               //if (AccelTimerSend[AccelTimerSendLen]&FXOS8700CQ_STATUS_ZYXDR) {
            if (Accel[2].Buffer[0]&FXOS8700CQ_STATUS_ZYXDR) {    
                ReturnValue=1; //data is ready
            } else {
                ReturnValue=0;  //data is not ready
            }


            if (ReturnValue) {
                //lock AccelTimerSendLen
                AccelTimerSendLen+=13;
                //unlock AccelTimerSendLen

                //add the accel # to the return UDP packet
                AccelTimerSend[AccelTimerSendLen-13]=2;
                //add the samples to the UDP packet
                memcpy(AccelTimerSend+AccelTimerSendLen-12,&Accel[2].Buffer[1],12);                    

            } else { //if (ReturnValue) {
                SYS_CONSOLE_PRINT("Accel 2 intr data not ready\n\r");
            } //if (ReturnValue) {

            if (ReturnValue) {
                //since this is the last accel
                //send the UDP packet
                SendTimerUDPPacket();
            } //if (ReturnValue) {

            //clear interrupt flag last because above callbacks use it
            Accel[2].flags&=~ACCEL_STATUS_GOT_INTERRUPT;                 
        } //if (Accel[2].flags&ACCEL_STATUS_INTERRUPT) {
#endif //FXOS8700CQ            
        break;
        case DRV_I2C_BUFFER_EVENT_ERROR:
            SYS_CONSOLE_PRINT("I2C Error Accel 2\r\n");
        break;
    } //switch(event)    
    
}


/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


#if 0 
/* Application's Timer Setup Function */
static void TimerSetup(void)
{
    DRV_TMR_AlarmRegister(
        appData.handleTimer1, 
        APP_TMR_DRV_PERIOD, 
        APP_TMR_DRV_IS_PERIODIC,
        (uintptr_t)NULL, 
        TimerCallback);
    DRV_TMR_Start(appData.handleTimer1);


    //set the timer (timer 2) to the default time interval)
    //note that for some unknown reason the timer is initially set to 
    //0xF423F 999,999, not 1,000,000 (which is 10ms or 100,000 timer clk ticks @100mhz)
    //cannot be done in AppInit() because that is before there is a timer handle
    TimerInterval=DEFAULT_TIMER_INTERVAL;
    //DRV_TMR_AlarmPeriodSet(appData.handleTimer1,TimerInterval*100000);  //1 prescaler      
    DRV_TMR_AlarmPeriodSet(appData.handleTimer1,TimerInterval*391); //256 prescaler


}
#endif 
/******************************************************************************
  Function:
    void BufferQueueEventHandler ( DRV_USART_BUFFER_EVENT,
                DRV_USART_BUFFER_HANDLE,  uintptr_t );

   Remarks:
    This routine is callback function for the USART buffer queue events. Driver
    uses the application Tx or Rx buffer reference along with the client handle
    to notify the buffer events to the application. This call back function 
    to be registerd with the driver using DRV_USART_BufferAddRead(); 
    or DRV_USART_BufferAddWrite();
 */

static void BufferQueueEventHandler(DRV_USART_BUFFER_EVENT  bufEvent,
                                    DRV_USART_BUFFER_HANDLE bufHandle,
                                    uintptr_t               context )
{
    int i;
 
    /*  Does bufHandle identify an RX buffer?  */
    if (appData.usartBQRxBufferHandle == bufHandle)
    {
        /* Transfer buffer to the Application by invalidating the buffer handle */
        appData.usartBQRxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;

        /* Zero the number of characters written in preparation for transmitting them */
        appData.usartBQRxWritten = 0;
          
        /* Set the number of characters read */              
        if (bufEvent == DRV_USART_BUFFER_EVENT_COMPLETE)
        {
            /* Successful read */
            appData.usartBQRxRead = APP_DRV_USART_BQ_RX_SIZE;
            //tph: send the data back
            DRV_USART_BufferAddWrite(appData.handleUSART0,
                            (DRV_USART_BUFFER_HANDLE * const)&appData.usartBQTxBufferHandle,
                            &appData.usartBQRxData[0], sizeof(appData.usartBQRxData));
            //if this character = 10 (line feed), then echo command and 
            //send to TCPIP Command stack to interpret
            appData.usartInst[appData.usartCurIndex]=appData.usartBQRxData[0];
            appData.usartCurIndex++;
            if (appData.usartBQRxData[0]==13) { //line feed
                //send output to terminal too
                DRV_USART_BufferAddWrite(appData.handleUSART0,
                                (DRV_USART_BUFFER_HANDLE * const)&appData.usartBQTxBufferHandle,
                                appData.usartInst, appData.usartCurIndex);
                //send command to console for command processing
                appData.usartInst[appData.usartCurIndex-1]=0; //replace 13 with 0
                //only echos to terminal: SYS_CMD_MESSAGE(appData.usartInst); //send inst to command module
                appData.usartCurIndex=0; //reset usart inst index                
            }
            
            //tph get a handle to a new read buffer
            appData.usartBQRxRead = 0;
        }
        else
        {
            /* Read was not successful */
            appData.usartBQRxRead = 0;
        }
        
        return;
    }

    /*  Does the buffer handle identify a TX buffer?  */
    if (appData.usartBQTxBufferHandle == bufHandle)
    {
        /* Transfer buffer to the Application by invalidating the buffer handle */
        appData.usartBQTxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;
        appData.usartBQTxWritten = sizeof(usartBQTxData);
        return;
    }
} //static void BufferQueueEventHandler(DRV_USART_BUFFER_EVENT  bufEvent,


/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter.
*/
static void USART_Task(void)
{
    switch (usartBQState)
    {
        default:
        case USART_BQ_INIT:
        {
            appData.usartBQTxWritten = 0;
            appData.usartBQRxRead    = 0;
            appData.usartBQRxWritten = 0;

            usartBQState = USART_BQ_WORKING;
            
            appData.usartCurIndex=0; //set usart inst index to 0
            break;
        }

        case USART_BQ_WORKING:
        {
            /* Check to see if the Application owns the TX buffer */
            if (appData.usartBQTxBufferHandle == DRV_USART_BUFFER_HANDLE_INVALID)
            {
                /* If we have data to transmit, then queue the buffer */
                if (appData.usartBQTxWritten == 0)
                {
                    DRV_USART_BufferAddWrite(appData.handleUSART0,
                            (DRV_USART_BUFFER_HANDLE * const)&appData.usartBQTxBufferHandle,
                            &usartBQTxData[0], sizeof(usartBQTxData));
                }
            }

            /* Has the RX buffer been read by the USART driver and transferred to the application by the BufferQueueEventHandler function? */
            if (appData.usartBQRxBufferHandle == DRV_USART_BUFFER_HANDLE_INVALID)
            {
                /* Process this buffer */
                if (appData.usartBQRxRead == 0)
                {
                    DRV_USART_BufferAddRead(appData.handleUSART0,
                        (DRV_USART_BUFFER_HANDLE * const)&appData.usartBQRxBufferHandle,
                        &appData.usartBQRxData[0], APP_DRV_USART_BQ_RX_SIZE);
                }
            }

            if ((appData.usartBQTxWritten == sizeof(usartBQTxData)) && (appData.usartBQRxRead == APP_DRV_USART_BQ_RX_SIZE))
            {
                //usartBQState = USART_BQ_DONE;
            }
            break;
        }

        case USART_BQ_DONE:
        {
            break;
        }
    }
}
/******************************************************************************
  Function:
    static void TCP_Server_TXRX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void TCP_Server_TXRX_Task(void)
{
	static IPV4_ADDR    		dwLastIP[2] = { {-1}, {-1} };
	static IPV4_ADDR           	ipAddr;
	int                 		i;
    uint16_t wMaxGet;

	switch (appData.txrxTaskState)
	{
        case APP_TCPIP_WAIT_FOR_IP:
        {
            app_nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < app_nNets; i++)
            {
                app_netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(app_netH);
                if (TCPIP_STACK_NetIsReady(app_netH))
                {                
                    if(dwLastIP[i].Val != ipAddr.Val)
                    {                         
                         SYS_CONSOLE_MESSAGE(TCPIP_STACK_NetNameGet(app_netH));
                         SYS_CONSOLE_MESSAGE(" IP Address: ");
                         SYS_CONSOLE_PRINT("%d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                         //SYS_CONSOLE_PRINT("%d.%d.%d.%d \r\n", dwLastIP[i].v[0], dwLastIP[i].v[1], dwLastIP[i].v[2], dwLastIP[i].v[3]);
                         
                         dwLastIP[i].Val = ipAddr.Val;
                         
                    } //if(dwLastIP[i].Val != ipAddr.Val)                

                    appData.txrxTaskState = APP_TCPIP_OPENING_SERVER;
                } //if (TCPIP_STACK_NetIsReady(app_netH))
            }
            break;
        }
        case APP_TCPIP_OPENING_SERVER:
        {
            //appData.socket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, appData.port, 0);
            appData.socket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, appData.port, 0);
            if (appData.socket == INVALID_SOCKET)                
            {
                SYS_CONSOLE_PRINT("Failed to open UDP Socket %d\r\n",appData.port);
                break;
            }
            appData.txrxTaskState = APP_TCPIP_WAIT_FOR_CONNECTION;;
        }
        break;

        case APP_TCPIP_WAIT_FOR_CONNECTION:
        {
            //if (!TCPIP_TCP_IsConnected(appData.socket))
            if (!TCPIP_UDP_IsConnected(appData.socket))
            {
                break;
            }
            else
            {
                // We got a connection
				//TCPIP_TCP_ArrayPut(appData.socket, appMsgToClient, sizeof(appMsgToClient));
                //appData.txrxTaskState = APP_TCPIP_WAIT_FOR_RESPONSE;
                appData.txrxTaskState = APP_TCPIP_SERVING_CONNECTION;
            }
        }
        break;

        case APP_TCPIP_SERVING_CONNECTION:
        {
            if (!TCPIP_UDP_IsConnected(appData.socket))
            {
                appData.state = APP_TCPIP_CLOSING_CONNECTION;
                SYS_CONSOLE_MESSAGE("Connection was closed\r\n");
                break;
            }
            //if any data has been received process it
            //In theory there could be more or less than one complete
            //instruction, but instructions come one in a packet
            //and are only a few bytes, and this loop checks for new data
            //so rapidly, that a single instruction is
            //always read at once.
            //todo: probably there should be code to check for 
            //a second instruction or extra data at the end of a 
            //received UDP buffer.
            
            wMaxGet = TCPIP_UDP_GetIsReady(appData.socket);	// Get UDP RX FIFO byte count

            if (wMaxGet>0) {
                Process_Instruction(wMaxGet);
            }

//          appData.state = APP_TCPIP_CLOSING_CONNECTION;
//          TCPIP_UDP_Discard(appData.socket);
        }
        break;

        case APP_TCPIP_CLOSING_CONNECTION:
        {
         	// Close the socket connection.
            TCPIP_UDP_Close(appData.socket);

            appData.state = APP_TCPIP_OPENING_SERVER;

        }
        break;

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/* TODO:  Add any necessary local functions.
*/



uint32_t ReadCoreTimer(void)
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return(timer);
}


uint8_t WaitForI2C(DRV_HANDLE I2CHandle,DRV_I2C_BUFFER_HANDLE BufferHandle) 
{
    uint8_t result,status;
    uint32_t tI2CTimeout;        

    result=I2C_INST_TIMEOUT;
    tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
    
    while ((result==I2C_INST_TIMEOUT) && (ReadCoreTimer()<=tI2CTimeout)) {
        status=DRV_I2C_TransferStatusGet(I2CHandle,BufferHandle);
        if (status==DRV_I2C_BUFFER_EVENT_COMPLETE) {
            result=I2C_INST_COMPLETE;
        } else {
            if (status==DRV_I2C_BUFFER_EVENT_ERROR) {        
                result=I2C_INST_ERROR;
            }
        }
    } //while
    
    if (result!=DRV_I2C_BUFFER_EVENT_COMPLETE) {
        SYS_CONSOLE_PRINT("status=%d\n\r",status);
    }
    return(result);
} //uint8_t WaitForI2C(I2CHandle,BufferHandle) {


/***********************************************************
 *   Millisecond Delay function using the Count register
 *   in coprocessor 0 in the MIPS core.
 *   When running 200 MHz, CoreTimer frequency is 100 MHz
 *   CoreTimer increments every 2 SYS_CLK, CoreTimer period = 10ns
 *   1 ms = N x CoreTimer_period;
 *   To count 1ms, N = 100000 counts of CoreTimer
 *   1 ms = 10 ns * 100000 = 10e6 ns = 1 ms
 *   When running 80 MHz, CoreTimer frequency is 40 MHz 
 *   CoreTimer increments every 2 SYS_CLK, CoreTimer period = 25ns
 *   To count 1ms, N = 40000 counts of CoreTimer
 *   1ms = 25 ns * 40000 = 10e6 ns = 1 ms
 *   ms_SCALE = (GetSystemClock()/2000) @ 200 MHz = 200e6/2e3 = 100e3 = 100000
 *   ms_SCLAE = (GetSystemClock()/2000) @ = 80e6/2e3 = 40e3 = 40000 
 */
 
void DelayMs(unsigned long int msDelay)
{
      register unsigned int startCntms = ReadCoreTimer();
      register unsigned int waitCntms = msDelay * ms_SCALE;
 
      while( ReadCoreTimer() - startCntms < waitCntms );
}

void DelayUs(unsigned long int usDelay)
{
      register unsigned int startCntms = ReadCoreTimer();
      register unsigned int waitCntms = usDelay * us_SCALE;
 
      while( ReadCoreTimer() - startCntms < waitCntms );
}


//Process any kind of instruction
void Process_Instruction(uint16_t InstDataLen)
{
#define MAX_INSTRUCTION_LEN 80 //max of 80 bytes in any sent instruction, was 512
    uint8_t InstData[MAX_INSTRUCTION_LEN]; //Received instruction
    uint8_t ReturnInst[MAX_INSTRUCTION_LEN];  //Return instruction
    TCPIP_NET_IF *pNetIf;
    //TCPIP_NET_IF  *pMyNetIf;
    uint32_t* MemAddr;
    uint32_t MemData;
    uint32_t InstValue;
    uint8_t *SendData;
    uint8_t DevRegValue,ReturnValue;
    uint8_t I2CData[20];
    uint16_t Sample,Threshold,AccelMask,AccelThreshold;
    uint32_t I2C_Status;
    uint32_t ReturnInstLen;
    uint32_t NumBytes;
    uint8_t DevNum; //device number
    uint8_t AccelNum; 
    uint8_t i,temp,result;
    uint32_t TouchSensorMask,AccelFlags;
    static uint8_t GPSToBinary[26] = "$PSRF100,0,9600,8,1,0*0C\r\n"; //put sirf from NMEA to binary mode
    static uint8_t GPSEnableStaticNav[10] = {0xa0,0xa2,0,0x2,0x8f,1,0,0x90,0xb0,0xb3};
    static uint8_t GPSDisableStaticNav[10] = {0xa0,0xa2,0,0x2,0x8f,0,0,0x8f,0xb0,0xb3};
    static uint8_t GPSToNMEA[32] = {0xa0,0xa2,0,0x18,0x81,2,1,1,0,1,1,1,5,1,1,1,0,1,0,1,0,1,0,1,0,1,0x25,0x80,1,0x3A,0xB0,0xB3};
    uint16_t TouchMin,TouchMax;
 

    // Transfer the data out of the TCP RX FIFO and into our local processing buffer.
    NumBytes= TCPIP_UDP_ArrayGet(appData.socket, InstData, InstDataLen);

    //SYS_CONSOLE_PRINT("\tReceived %d bytes\r\n",NumBytes);
    
    
//currently instructions are returned:
    //sending IP[0-3] Inst[1]
    //switch(SetupPkt.bRequest)  //USB
    switch(InstData[4]) //Robot Instruction
    {
    ////=====START PIC RELATED INSTRUCTIONS
    case ROBOT_ACCELMAGTOUCH_TEST: //send back 0x12345678
        memcpy(ReturnInst,InstData,5); //copy IP + inst byte to return instruction
        ReturnInst[5]=0x12;
        ReturnInst[6]=0x34;
        ReturnInst[7]=0x56;
        ReturnInst[8]=0x78;
        while (TCPIP_UDP_PutIsReady(appData.socket)<8) {};
        TCPIP_UDP_ArrayPut(appData.socket,ReturnInst, 8);  //little endian
        TCPIP_UDP_Flush(appData.socket); //send the packet
    break;
    case ROBOT_ACCELMAGTOUCH_PCB_NAME: //01 send back name/id
        memcpy(ReturnInst,InstData,5); //copy IP + inst byte to return instruction
        ReturnInstLen=5;
        //get the MAC address from the default network interface
        //this presumes that there is only 1 net
        //in the future there could be more than 1 net, 
        //for example a wireless net too
        //netH = TCPIP_STACK_GetDefaultNet();
        app_netH = TCPIP_STACK_IndexToNet(0);  //presumes net 0 is the wired net
        //pMyNetIf = _TCPIPStackHandleToNet(netH);
        pNetIf = _TCPIPStackHandleToNetUp(app_netH);
        memcpy(ReturnInst+ReturnInstLen,(pNetIf)->netMACAddr.v,sizeof(pNetIf->netMACAddr));//copy mac
        ReturnInstLen+=sizeof(pNetIf->netMACAddr);
        memcpy(ReturnInst+ReturnInstLen,PCB_Name,PCB_NAME_LENGTH);//copy name
        ReturnInstLen+=PCB_NAME_LENGTH; //MOTOR
        //while (UDPIsTxPutReady(UDPSendSock,ReturnInstLen)<ReturnInstLen) {};
        //UDPPutArray(UDPSendSock,(uint8_t *)ReturnInst,ReturnInstLen);  //little endian
        //UDPFlush(UDPSendSock); //send the packet
        while (TCPIP_UDP_PutIsReady(appData.socket)<ReturnInstLen) {};
        TCPIP_UDP_ArrayPut(appData.socket,ReturnInst, ReturnInstLen);  //little endian       
        TCPIP_UDP_Flush(appData.socket); //send the packet

    break;
    case ROBOT_ACCELMAGTOUCH_GET_MEM: //get PIC memory value
        MemAddr=(uint32_t *)((InstData[8]<<0x18)|(InstData[7]<<0x10)|(InstData[6]<<0x8)|InstData[5]);
        memcpy(ReturnInst,InstData,5); //copy IP + inst byte to return instruction
        memcpy(ReturnInst+5,MemAddr,4); //little endian
        while (TCPIP_UDP_PutIsReady(appData.socket)<8) {};
        TCPIP_UDP_ArrayPut(appData.socket,ReturnInst,8);  //little endian       
        TCPIP_UDP_Flush(appData.socket); //send the packet
    break;
    case ROBOT_ACCELMAGTOUCH_SET_MEM: //set PIC memory value
        //write to PIC memory
        //presumes 2 32-bit little-endian integer values
        MemAddr=(uint32_t *)((InstData[8]<<0x18)|(InstData[7]<<0x10)|(InstData[6]<<0x8)|InstData[5]);
        MemData=(uint32_t)((InstData[12]<<0x18)|(InstData[11]<<0x10)|(InstData[10]<<0x8)|InstData[9]);
#if 0 //currently disabled 
        *MemAddr=MemData;
#endif
    break;
    case ROBOT_ACCELMAGTOUCH_GET_TIMER_INTERVAL_IN_MSEC:
        //ip(0-3) inst(4)
        //send back SourceIP+data so data is sent to correct requesting machine
        memcpy(ReturnInst,InstData,5); //copy IP + inst byte to return instruction

        //in theory should read PR2 and PR3
        //tph get timer interval (period) 
        //TimerInterval=(uint16_t)(DRV_TMR_AlarmPeriodGet(appData.handleTimer1)/100000);       
        //MemData=(uint32_t)(((PR3<<0x10)|PR2)*10)/3906;
        //the above returns 9- for 10ms period 
        //and 19 for 20ms, etc.
        //which is somewhat misleading

        //Timers use PBCLK3, which is at 100Mhz
        memcpy(ReturnInst+5,&TimerInterval,2); //little endian
        //memcpy(ReturnInst+5,&MemData,2); //little endian
        while (TCPIP_UDP_PutIsReady(appData.socket)<7) {};
        TCPIP_UDP_ArrayPut(appData.socket,ReturnInst,7);  //little endian       
        TCPIP_UDP_Flush(appData.socket); //send the packet        
    break;
    case ROBOT_ACCELMAGTOUCH_SET_TIMER_INTERVAL_IN_MSEC:
        //ip(0-3) inst(4) interval (5-6)
        //set the polling timer interrupt in milliseconds
        //has 1 parameter=4 uint8_t Little Endian int
        InstValue=(uint16_t)((InstData[6]<<0x8)|InstData[5]);        
        //InstValue=*(uint32_t *)&InstData[1];  //doesn't work
        //PB2CLK is 100MHz, 1:1 div, so 1 tick every 10ns
        //1tick=10ns, 1000ticks=10us, 1e6 ticks=10ms
        //10ms * 100,000 = 1,000,000 ticks
        //100ms * 100,000 = 10,000,000 ticks
        //1 s = 100,000,000 ticks / 100=10ms=1e6ticks/256prescaler=
        if (InstValue!=TimerInterval) {
            //DRV_TMR_AlarmPeriodSet(appData.handleTimer1,InstValue*100000);        
            //DRV_TMR_AlarmPeriodSet(appData.handleTimer1,InstValue*391);        
            TimerInterval=InstValue;
            
            temp=T2CONbits.ON;
            T2CONbits.ON=0; //disable timer2+3 (for 32-bit)
            //PR2=(TimerInterval*100000)&0xffff;
            //PR3=((TimerInterval*100000)&0xffff0000)>>0x10;
            PR2=((TimerInterval*3906)/10)&0xffff;
            PR3=(((TimerInterval*3906)/10)&0xffff0000)>>0x10;
            TMR2=0;  //reset timer count
            TMR3=0;
            if (temp) {  //Timer2+3 was enabled
                T2CONbits.ON=1; //re-enable timer2+3 (for 32-bit)
            }
            
        }
    break;

    ////=====END PIC RELATED INSTRUCTIONS    
    
    ////=====START ACCELEROMETER+MAGNETOMETER RELATED INSTRUCTIONS
    case ROBOT_ACCELMAGTOUCH_GET_ACCEL_REG: //get a register value from an accelerometer
        //command = 20 (GET_ACCEL_REG) 00 (Accelerator #0) (ACCEL REG=0d)
        //should return c7 for FXOS, 4a = device id for MMA or 6a for FXLS
        //[0-3]=IP
        //[4]=20 get accel reg
        //[5]=accel #
        //[6]=reg #
        AccelNum=InstData[5];
        //I2CData[0]=Accel[DevNum].I2CAddress;
        //I2CData[0] = InstData[6];  //reg address
        
        //DevRegValue=ReadI2CData(I2CData,2,Accel[DevNum].I2CBus);
        if (Accel[AccelNum].flags&ACCEL_STATUS_ENABLED) {
//#if 0 
            //send back:
            //SourceIP(0:3)+command(4)+accel(5)+returnbyte(6)
            memcpy(ReturnInst,InstData,6); //copy IP + inst byte to return instruction
            ReturnInstLen=6;
            Accel[AccelNum].I2CBufferHandle=DRV_I2C_TransmitThenReceive(Accel[AccelNum].handleI2C, 
                                                            Accel[AccelNum].I2CAddress,
                                                            &InstData[6], //byte to send: register addr
                                                            1,  
                                                            &ReturnInst[ReturnInstLen],//appData.ReadBuffer,
                                                            1,
                                                            NULL); 
            DelayMs(1);

            //wait for read to finish
            result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
            //if (Accel[AccelNum].I2CBufferHandle == NULL) {
            if (result==I2C_INST_COMPLETE) {   
                //add read bytes to return inst
               //memcpy(ReturnInst+ReturnInstLen,&ReturnValue,1);
                ReturnInstLen++;

                while (TCPIP_UDP_PutIsReady(appData.socket)<ReturnInstLen) {};
                TCPIP_UDP_ArrayPut(appData.socket,ReturnInst,ReturnInstLen);  //little endian       
                TCPIP_UDP_Flush(appData.socket); //send the packet                                        

            } else {
                SYS_CONSOLE_PRINT("GetReg Error %d on a%d\r\n",result,AccelNum);    
                SYS_CONSOLE_PRINT("handle=%d\r\n",Accel[AccelNum].handleI2C);    
                
            }   //if (result==I2C_INST_COMPLETE) {   
                
        } //if (Accel[DevNum].flags&ACCEL_STATUS_ENABLED) {

//        memcpy(appData.ReturnInst,InstData,5); //copy IP + inst byte to return instruction
//        appData.ReturnInstLen=5;

        //ReturnInst[4]=InstData[5]; //send original reg
        //ReturnInst[5]=DevRegValue;

/*        while (TCPIP_UDP_PutIsReady(appData.socket)<6) {};
        TCPIP_UDP_ArrayPut(appData.socket,ReturnInst,6);  //little endian       
        TCPIP_UDP_Flush(appData.socket); //send the packet        
*/
    break;
    case ROBOT_ACCELMAGTOUCH_SET_ACCEL_REG: //set a register value in an accelerometer
        //returns Sending IP [4]+result [1]
        //InstData[5] has the accelerometer #
        //InstData[6] has the register address
        //InstData[7] has the register value
        AccelNum=InstData[5];
        if (Accel[AccelNum].flags&ACCEL_STATUS_ENABLED) {
            //memcpy(appData.ReturnInst,InstData,5); //copy IP + inst byte to return instruction
            Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C, 
                                                            Accel[AccelNum].I2CAddress,
                                                            &InstData[6], //byte to send: register addr
                                                            2,  
                                                            NULL); 
            DelayMs(1);

            //wait for read to finish
            result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
        //if (Accel[AccelNum].I2CBufferHandle == NULL) {
        //could send result
        //if (result==I2C_INST_COMPLETE) {                    
        } //if (Accel[AccelNum].flags&ACCEL_STATUS_ENABLED) {
    break;        
    case ROBOT_ACCELMAGTOUCH_GET_ACCELEROMETER_VALUES:
        memcpy(AccelTimerSend,InstData,5); //copy IP + inst byte to return instruction
        memcpy(&AccelMask,(uint16_t *)&InstData[5],2);  //copy 16-bit mask
        //SYS_CONSOLE_PRINT("AccelMask=%d\r\n",AccelMask);
        //ACCEL_STATUS_POLLING needs to be set currently because the sample is retrieved by the timer ISR
        if (EAStatus.flags&ETHACCEL_STATUS_ACCEL_INTERRUPT) {
            for(i=0;i<NumAccelerometers;i++) {
                if (Accel[i].flags&ACCEL_STATUS_INTERRUPT) {
                    //trigger a sample send in the next timer interrupt callback
                    Accel[i].flags|=ACCEL_STATUS_CLEAR_INTERRUPT;
                }
            } //for i
        } else {
            ConfigureAccelerometers(AccelMask,ACCEL_STATUS_ENABLED|ACCEL_STATUS_SINGLE_SAMPLE|ACCEL_STATUS_POLLING,0);
            EAStatus.flags|=ETHACCEL_STATUS_ACCEL_POLLING; //so timer will call GetSample        
        }
        //Get_Accelerometer_Samples();
    break;
    case ROBOT_ACCELMAGTOUCH_START_POLLING_ACCELEROMETER:
        //sends a UDP packet at every timer interval
        //ip[0-3] robot_inst[4] accel# mask[5-6]
        //for example a600: send a UDP packet (to the original requester) anytime accelerometer #0 detects a large enough change in acceleration        
        if (InstDataLen>=7) {
            memcpy(AccelTimerSend,InstData,5); //copy IP + inst byte to return instruction
            memcpy(&AccelMask,(uint16_t *)&InstData[5],2);
            //IEC0bits.T3IE=0; //disable timer while enabling polling
            ConfigureAccelerometers(AccelMask,ACCEL_STATUS_ENABLED|ACCEL_STATUS_POLLING,0);

            EAStatus.flags|=ETHACCEL_STATUS_ACCEL_POLLING;

        } //if (InstDataLen>=7) {
    break;
    case ROBOT_ACCELMAGTOUCH_STOP_POLLING_ACCELEROMETER:
        //ip[0-3] robot_inst[4] accel# mask[5-6]
 
        memcpy(&AccelMask,(uint16_t *)&InstData[5],2);
        if (AccelMask&1) {
            Accel[0].flags&=~ACCEL_STATUS_POLLING;
        }
        if (AccelMask&2) {
            Accel[1].flags&=~ACCEL_STATUS_POLLING;
        }
        if (AccelMask&4) {
            Accel[2].flags&=~ACCEL_STATUS_POLLING;
        }
        
        if (!(Accel[0].flags&ACCEL_STATUS_POLLING) &&
            !(Accel[1].flags&ACCEL_STATUS_POLLING) &&
            !(Accel[2].flags&ACCEL_STATUS_POLLING)) {
            
            //stop checking for polling in the timer interrupt callback
            EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_POLLING;
        }
        
        //Currently there is a problem: stopping the accels
        //is not allowing the accels to be restarted
        //so for now, just stop all polling. This will 
        //not disable the accelerometer, but will simply
        //not request samples any more.
        //If I had to guess, I would say that
        //the timer is still enabled and trying to get 
        //samples from an accel that has been deactivated.
        
        //Delay_ms(100);
        
/*        
        //you may be only stopping a single accel
        if (InstDataLen>=7) {
            memcpy(&AccelMask,(uint16_t *)&InstData[5],2);
            //also sets accel flags=0
            SetActiveAccelerometers(AccelMask,ACCEL_STATUS_NOT_ACTIVE,0);
        } else {
            //deactivate all
            SetActiveAccelerometers(0xffff,ACCEL_STATUS_NOT_ACTIVE,0);
        }
*/                
    break;
    case ROBOT_ACCELMAGTOUCH_START_ACCELEROMETER_INTERRUPT:
        //sends a UDP packet when the accel triggers an interrupt because 
        //there is a large enough change in acceleration
        //(starts up to 16 accelerometers at once): 
        //ip[0-3] robot_inst[4] accel# 16-bit mask[5-6] abs/rel[7] threshold (in mg) [8-9]
        //was: only starts 1 accelerometer at a time
        //ip[0-3] inst[4] accel#[5] abs/rel[6] threshold (in mg) [7-8]

        
        AccelThreshold=0;
        if (InstDataLen>=6) {
            //need to initialize AccelSend here?
            memcpy(AccelTimerSend,InstData,5); //copy IP + inst byte to return instruction
            //for now use Polling instruction
            AccelTimerSend[4]=ROBOT_ACCELMAGTOUCH_START_POLLING_ACCELEROMETER;
            memcpy(&AccelMask,(uint16_t *)&InstData[5],2);
            //AccelMask=(uint16_t)((InstData[6]<<8)|InstData[5]);
            //DevNum=InstData[5];
            //todo: later perhaps the return ip will be connected to each individual accelerometer
            AccelFlags=ACCEL_STATUS_ENABLED|ACCEL_STATUS_INTERRUPT;
            if (InstDataLen>=7) { //absolute/relative byte was sent
                //if (InstData[6]) {
                if (InstData[7]) {
                    AccelFlags|=ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION;
                }
                //if (InstDataLen>=9) { //threshold was sent
                if (InstDataLen>=8) { //threshold was sent
                    //AccelThreshold=(uint16_t)((InstData[8]<<8)|InstData[7]);
                    AccelThreshold=(uint16_t)((InstData[9]<<8)|InstData[8]);
                }
            }
                                  
            ConfigureAccelerometers(AccelMask,AccelFlags,AccelThreshold);
            //convert AccelNum into mask
            //SetActiveAccelerometers(1<<DevNum,AccelFlags,AccelThreshold);
            //EAStatus.flags|=ETHACCEL_STATUS_ACCEL_INTERRUPT;
        } //if (InstDataLen>=7) {
    break;
    case ROBOT_ACCELMAGTOUCH_STOP_ACCELEROMETER_INTERRUPT:
        //ip[0-3] robot_inst[4] accel# 16-bit mask[5-6]
        
        //EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_INTERRUPT;
        //Delay_ms(100);
        //there is currently a problem: the code below causes
        //interrupts not to be started again

        //probably the Accel should not be disabled
        //but only interrupt sample sending stopped
        if (InstDataLen>=7) {
            memcpy(&AccelMask,(uint16_t *)&InstData[5],2);
            ConfigureAccelerometers(AccelMask,ACCEL_STATUS_NOT_ENABLED,0);
        } else {
            //deactivate all accels
            //EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_INTERRUPT;
            ConfigureAccelerometers(0xffff,ACCEL_STATUS_NOT_ENABLED,0);
        }//if (InstDataLen>=7) {
#if 0 
        /* did not stop int
        Accel[0].flags&=~ACCEL_STATUS_INTERRUPT;
        Accel[1].flags&=~ACCEL_STATUS_INTERRUPT;
        Accel[2].flags&=~ACCEL_STATUS_INTERRUPT;
         */ 
        /* unbelievably the below does not stop the interrupt
        Disable_Accelerometer_Interrupt(0);
        Reset_Accelerometer(0);
        Disable_Accelerometer_Interrupt(1);
        Reset_Accelerometer(1);
        Disable_Accelerometer_Interrupt(2);
        Reset_Accelerometer(2);
        */
#endif        
    break;
    case ROBOT_ACCELMAGTOUCH_RESET_ACCELEROMETER:
        //ip(0-3) inst(4) accel# 16-bit mask(5-6)
        if (InstDataLen>=7) {
            memcpy(&AccelMask,(uint16_t *)&InstData[5],2);
            //go through mask and reset and initialize each accelerometer
            for(i=0;i<NumAccelerometers;i++) {
                  if (AccelMask&(1<<i)) { //accel is selected
                    Reset_Accelerometer(i);
                    Initialize_Accelerometer(i);
                } //if (AccelMask&*1<<i)) {
            } //for i
        } //if (InstDataLen>=7) 
    break;
    //=====START TOUCH SENSOR RELATED INSTRUCTIONS
    case ROBOT_ACCELMAGTOUCH_ENABLE_TOUCH_SENSORS:
        //turn on ADC module?
     //   IEC1bits.AD1IE=1; //enable AD1 interrupt
        //
        //ip[0-3] robot_inst[4] touch#[5]
    break;
    case ROBOT_ACCELMAGTOUCH_DISABLE_TOUCH_SENSORS:
        //turn off ADC module?
     //   IEC1bits.AD1IE=0; //disable AD1 interrupt
        //ip[0-3] robot_inst[4] touch#[5]
    break;
    case ROBOT_ACCELMAGTOUCH_GET_TOUCH_SENSOR_VALUES:
        //ip[0-3] robot_inst[4] touch sensor mask[5-8]
        //todo: change to just polling and letting the ADC ISR 
        //deactivate any accels that are single sample only
        //then polling, interrupt, and single sample can all operate at the same time.
        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_POLLING; //stop any polling
        //stop interrupt too?
        memcpy(TouchSensorSend,InstData,5); //copy IP + inst byte to return instruction
        TouchSensorSendLen=5;
        //touch sensor adc module is currently always on
        //so just poll the adc?
           //the reference manual states to poll AD1IF (not DONE)
        //reset which pins to sample
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
            //1=activate, and flags are set for single sample (cleared in ADC ISR))
            SetActiveTouchSensors(~TouchSensorMask,0); //disable any touch sensors not selected
            SetActiveTouchSensors(TouchSensorMask,1); //enable touch sensors that are selected
            //SetActiveTouchSensors(TouchSensorMask,1,TOUCH_SENSOR_SINGLE_SAMPLE);
        }
///*pic32mx        
        IFS1bits.AD1IF=0;//.AD1IF=0; //clear interrupt flag
        IEC1bits.AD1IE=1;//.AD1IE=1; //enable AD1 interrupt
        AD1CON1bits.ASAM=1; //autostart sampling
 //*/
        //SendSerial("get sample",10);
        //SYS_CONSOLE_PRINT("get sample\r\n");    
        /*pic32mz        
        IFS6bits.ADCEOSIF=0; //clear end of scan interrupt flag
        IEC6bits.ADCEOSIE=1; //enable end of scan interrupt
        ADCCON3bits.GSWTRG=1;//Trigger a conversion
        */
        
        //ISR will send data
        //Note there is no need for the timer interrupt for a single set of samples
    break;
    case ROBOT_ACCELMAGTOUCH_START_POLLING_TOUCH_SENSORS:
//        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT; //set stop polling flag
        memcpy(TouchSensorSend,InstData,5); //copy IP + inst byte to return instruction
        EAStatus.flags|=ETHACCEL_STATUS_TOUCH_SENSOR_POLLING;
        //set which sensors to poll
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
            SetActiveTouchSensors(TouchSensorMask,1); //1=activate
        }
        //enable the timer that starts the ADC (and also polls accels)
        T2CONbits.ON=1; //enable timer2+3 (for 32-bit)
        //there is no need to enable the interrupt, because it is always enabled
        //but just to make sure
        IEC0bits.T3IE=1;        //enable Timer 2+3 interrupt- doesn't enable the timer
    break;
    case ROBOT_ACCELMAGTOUCH_STOP_POLLING_TOUCH_SENSORS:
        
        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_POLLING; //set stop polling flag
        
        //todo: this will get a 32-bit mask to determine which touch sensors
        //to stop polling
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
            SetActiveTouchSensors(TouchSensorMask,0); //0=deactivate
        } else {
            //deactivate all
            SetActiveTouchSensors(0xffffffff,0); //0=deactivate
        }


        //EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_POLLING; //set stop polling flag
        IEC1bits.AD1IE=0; //disable AD1 interrupt
        IFS1bits.AD1IF=0; //clear interrupt flag
        //pic32mz IEC6bits.ADCEOSIE=0; //disable end of scan interrupt
        //pic32mz IFS6bits.ADCEOSIF=0; //clear end of scan interrupt flag
        if (!(EAStatus.flags&ETHACCEL_STATUS_ACCEL_POLLING) &&
            !(EAStatus.flags&ETHACCEL_STATUS_ACCEL_INTERRUPT) &&
            !(EAStatus.flags&ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT)) {
            //only stop timer if no touch sensor polling or interrupt is enabled
            T2CONbits.ON=0; //disable timer2+3 (for 32-bit)
        } //

    break;
    case ROBOT_ACCELMAGTOUCH_START_TOUCH_SENSORS_INTERRUPT:
        //touch sensor interrupts still poll the touch sensor (default=every 100ms),
        //but only send a UDP when there is a large change in voltage
        //set which sensors to send interrupt UDP packets for
        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_POLLING; //set stop polling flag
        memcpy(TouchSensorSend,InstData,5); //copy IP + inst byte to return instruction
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
            SetActiveTouchSensors(TouchSensorMask,1); //1=activate
        }

        EAStatus.flags|=ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT;
        T2CONbits.ON=1; //enable timer2+3 (for 32-bit)
        break;
    case ROBOT_ACCELMAGTOUCH_STOP_TOUCH_SENSORS_INTERRUPT:
        //todo: this will get a 32-bit mask to determine which touch sensors
        //to stop interrupt
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
            SetActiveTouchSensors(TouchSensorMask,0); //0=deactivate
        } else {
            //deactivate all touch sensors
            SetActiveTouchSensors(0xffffffff,0);
        }

        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT; //set stop polling flag
        IEC1bits.AD1IE=0; //disable AD1 interrupt
        IFS1bits.AD1IF=0; //clear interrupt flag
        //pic32mz IEC6bits.ADCEOSIE=0; //disable end of scan interrupt
        //pic32mz IFS6bits.ADCEOSIF=0; //clear end of scan interrupt flag        
    break;
    case ROBOT_ACCELMAGTOUCH_GET_TOUCH_SENSOR_THRESHOLD: 
        //get 1 or more touch sensor thresholds
        //threshold is how much of a voltage change there needs to be
        //before an interrupt is triggered
      	//probably should be mask of only 16 bits 
    	//because currently there can only be 15 touch sensors/PCB

        //ip(0-3) inst(4) touch sensor mask(5-8)
        memcpy(ReturnInst,InstData,5); //copy IP + inst byte to return instruction + sensor #
        ReturnInstLen=5;
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint16_t *)&InstData[5],4);
            for(i=0;i<NumTouchSensors;i++) {
                if (TouchSensorMask&(1<<i)) {
                    ReturnInst[ReturnInstLen+1]=i;
                    Threshold=TouchSensor[i].Threshold;
                    memcpy(&ReturnInst[ReturnInstLen+1],(uint8_t *)&Threshold,2);
                    ReturnInstLen+=3;
                } //if (TouchSensorMask&(1<<i)) {
            } //for(i=0;i<NumTouchSensors;i++) {
            if (TouchSensorMask!=0) {
                while (TCPIP_UDP_PutIsReady(appData.socket)<ReturnInstLen) {};
                TCPIP_UDP_ArrayPut(appData.socket,ReturnInst,ReturnInstLen);  //little endian       
                TCPIP_UDP_Flush(appData.socket); //send the packet        
            } //if (TouchSensorMask!=0) {
        } //if (InstDataLen>=9) {
    break;
    case ROBOT_ACCELMAGTOUCH_SET_TOUCH_SENSOR_THRESHOLD: //set one of more touch sensor thresholds
        //ip(0-3) inst(4) sensor#(5) threshold (6-7) sensor#(8) threshold(9-10), etc.
        //could be ip(0-4) inst(5) mask (6-9) threshold (10-11) threshold(12-13)
        //or even just a single threshold
        ReturnInstLen=5;
        while(ReturnInstLen<InstDataLen) {
            DevNum=InstData[ReturnInstLen];
            Threshold=(uint16_t)((InstData[ReturnInstLen+2]<<8)|InstData[ReturnInstLen+1]);
            TouchSensor[DevNum].Threshold=Threshold;
            ReturnInstLen+=3;
        } //while
    break;
    case ROBOT_ACCELMAGTOUCH_GET_TOUCH_MINMAX:         
        //send the user the min and max voltage for 1 or more touch sensors
        //ip(0-3) inst(4) touch sensor mask (5-9)        
        //returns:
        //ip(0-3) inst(4) num(5)min(6-7)max(8-9)num(10)min(11-12)max(13-14)...
        memcpy(ReturnInst,InstData,5); //copy IP + inst byte to return instruction + sensor #
        ReturnInstLen=5;
        if (InstDataLen>=9) {
            memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
            for(i=0;i<NumTouchSensors;i++) {
                if (TouchSensorMask&(1<<i)) {
                    ReturnInst[ReturnInstLen]=i;
                    memcpy(&ReturnInst[ReturnInstLen+1],(uint8_t *)&TouchSensor[i].Min,2);
                    memcpy(&ReturnInst[ReturnInstLen+3],(uint8_t *)&TouchSensor[i].Max,2);
                    ReturnInstLen+=5;
                } //if (TouchSensorMask&(1<<i)) {
            } //for(i=0;i<NumTouchSensors;i++) {
            if (TouchSensorMask!=0) {
                while (TCPIP_UDP_PutIsReady(appData.socket)<ReturnInstLen) {};
                TCPIP_UDP_ArrayPut(appData.socket,ReturnInst,ReturnInstLen);  //little endian       
                TCPIP_UDP_Flush(appData.socket); //send the packet        
            } //if (TouchSensorMask!=0) {
        } //if (InstDataLen>=9) {
    break;
    case ROBOT_ACCELMAGTOUCH_SET_TOUCH_MINMAX:         
        //set the min and max voltage for 1 or more touch sensors
        //ip(0-3) inst(4) touch sensor mask(5-8) min(9-10) max(11-12)        
        //(note that only one min and max is sent)
        memcpy(&TouchSensorMask,(uint32_t *)&InstData[5],4);
        TouchMin=(uint16_t)((InstData[10]<<8)|InstData[9]);
        TouchMax=(uint16_t)((InstData[12]<<8)|InstData[11]);
        for(i=0;i<NumTouchSensors;i++) { //only 15 sensors
            if (TouchSensorMask&(1<<i)) {
                TouchSensor[i].Min=TouchMin;
                TouchSensor[i].Max=TouchMax;
            } //if (TouchSensorMask&(1<<i)) {
        } //for i
    break;

    //=====END TOUCH SENSOR RELATED INSTRUCTIONS
    
    //=====START GPS MODULE RELATED INSTRUCTIONS
#if 0     
    case ROBOT_ACCELMAGTOUCH_GET_GPS_DATA: //start sending GPS data
//really unusual- this line is needed UART3 to work (with external 
        //5v power supply with common ground)
//        if (U3STAbits.OERR != 0) {  //testing
//                U3STAbits.OERR = 0;
//        }
        //if already getting GPS do nothing
        if (!(EAStatus.flags&ETHACCEL_STATUS_GPS_ENABLED)) {
            //GPS not enabled yet
            //GPS is already configured but needs:
            //1. Put a header in the packet to send to the user
            //2. the UART3 receive interrupt enabled
            //3. GPS module powered on

            //put header into GPS return packet
            memcpy(GPSSend,InstData,5); //copy IP + inst byte to return instruction
            GPSSendLen=5;
  
//            U3STAbits.OERR = 0; //this was needed because I enabled the UART before enabling the interrupt
  
            
            IFS1bits.U3RXIF=0; //clear the UART3 interrupt flag
            IEC1bits.U3RXIE=1; //enable UART3 receive interrupt            

            //Note: do not enable the UART until the interrupt is enabled
            //because otherwise an overrun error will occur
            U3STASET = 0x1400;            // Enable Transmit and Receive
            
            
            //power the GPS on pin RG6
            LATGbits.LATG6=1;
            EAStatus.flags|=ETHACCEL_STATUS_GPS_ENABLED;
        }
        
    break;
    case ROBOT_ACCELMAGTOUCH_STOP_GPS_DATA:  //stop sending GPS data
        

       //Note: it is important to disable the UART 
       //before disabling the interrupt
       //because otherwise an overrun error will occur
        U3STACLR = 0x1400; // Disable Transmit and Receive


        LATGbits.LATG6=0; //power off the GPS module on pin RG6
        
        
        IEC1bits.U3RXIE=0; //disable UART3 receive interrupt                    
        IFS1bits.U3RXIF=0; //clear the UART3 interrupt flag
        
        
        EAStatus.flags&=~ETHACCEL_STATUS_GPS_ENABLED;
    break;
    case ROBOT_ACCELMAGTOUCH_SET_SEND_ALL_GPS_DATA:
        EAStatus.flags|=ETHACCEL_STATUS_SEND_ALL_GPS_DATA;
    break;
    case ROBOT_ACCELMAGTOUCH_UNSET_SEND_ALL_GPS_DATA:
        EAStatus.flags&=~ETHACCEL_STATUS_SEND_ALL_GPS_DATA;
    break;
    case ROBOT_ACCELMAGTOUCH_ENABLE_STATIC_NAVIGATION:
        SendSerial(GPSToBinary,26);
        Delay_ms(200);        
        SendSerial(GPSEnableStaticNav,10);
        Delay_ms(200);                        
        SendSerial(GPSToNMEA,32);
    break;
    case ROBOT_ACCELMAGTOUCH_DISABLE_STATIC_NAVIGATION:

        SendSerial(GPSToBinary,26);
        Delay_ms(200);        
        SendSerial(GPSDisableStaticNav,10);
        Delay_ms(200);                        
        SendSerial(GPSToNMEA,32);
        
    break;      
#endif     
    //=====END GPS MODULE RELATED INSTRUCTIONS    
    default:
    break;
    }//end switch
} //void Process_Instruction(uint16_t InstDataLen)


 

/*******************************************************************************
 End of File
 */
