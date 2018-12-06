/*******************************************************************************
  Board Support Package Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    bsp.c

  Summary:
    Board Support Package implementation for PIC32MX Embedded Connectivity (EC)
    Starter Kit.

  Description:
    This file contains routines that implement the board support package for
    PIC32MX Embedded Connectivity (EC) Starter Kit.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include <proc/p32mx675f256h.h>
//#include <proc/p32mz0512efe064.h>

#include "bsp.h"
#include "robot_accelmagtouchgps_pic_instructions.h" //robot EthAccelTouch PCB instruction codes
#include "accel.h"  //acelerometers data structures (which I2C and external interrupts each accel uses)
#include "touchsensors.h"  //touch sensor data structures (which PORT PINs each touch sensor uses)
//tph #include "i2c_master.h" //I2C functions
#include "app.h"  //for EthAccelStatus
#include "FXOS8700CQ.h"  //accelerometer+magnetometer address and registers
#include "MPU6050.h" //accelerometer+magnetometer address and registers


extern EthAccelStatus EAStatus; 
extern uint8_t NumAccelerometers;
extern AccelStatus Accel[MAX_NUM_ACCEL]; 
extern uint8_t IntAccel[5]; 
extern uint16_t TimerInterval; //the time (in us) of each timer2 interrupt

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function: 
    void BSP_Initialize(void)

  Summary:
    Performs the necessary actions to initialize a board
  
  Description:
    This function initializes the LED, Switch and other ports on the board.
    This function must be called by the user before using any APIs present in
    this BSP.  

  Remarks:
    Refer to bsp.h for usage information.
*/

void BSP_Initialize(void )
{
    
    uint8_t i;

    /* Configure the target for maximum performance at 80 MHz */
    //SYSTEMConfigPerformance(80000000UL);
    //mOSCSetPBDIV(OSC_PB_DIV_1); //set PBCLK=SYSCLK 80 MHZ

    //DDPCONbits.JTAGEN = 0;                  // This turns off the JTAG and allows PORTB pins to be used
    //DDPCONbits.TROEN=0;  //turn off trace enable bits

/*
    //tph do hardware reset on PHY nRST- active low
    TRISFbits.TRISF0=0;  //configure port F pin 0 as output (PHY nReset)
    LATFbits.LATF0=0;
    SYS_TMR_DelayMS(1);  //delay 1 ms, min=100us  //min=100us
    LATFbits.LATF0=1;
    SYS_TMR_DelayMS(1);  //delay 1 ms, min=100us  //wait before sending anything for the PHY to initialize

*/
    
    EAStatus.flags=0; //clear all EthAccelTouch PCB flags

    //for debugging
    //LATB=0x5555;

    //ACCEL PORT1 - top of PCB
    TRISDbits.TRISD2=1; //SDA3 - input
    TRISDbits.TRISD3=0; //SCL3 - output
    TRISDbits.TRISD0=1;  //INT0 - input
    TRISDbits.TRISD6=0;  //power pin - output
    //set RD2 and RD3 as open drain for I2C communication
    ODCDbits.ODCD2=1;
    ODCDbits.ODCD3=1;
    ODCDbits.ODCD6=0; //no open drain for power pin
    
   //ACCEL PORT2 - top of PCB on right of PORT2
    TRISDbits.TRISD9=1; //SDA1 - input
    TRISDbits.TRISD10=0; //SCL1 - output
    TRISDbits.TRISD11=1;  //INT4 - input
    TRISDbits.TRISD5=0;  //power pin - output
    //set RD9 and RD10 as open drain for I2C communication
    ODCDbits.ODCD9=1;
    ODCDbits.ODCD10=1;
    ODCDbits.ODCD5=0; //no open drain for power pin

   //ACCEL PORT3 - right side of PCB
    TRISFbits.TRISF4=1; //SDA5 - input
    TRISFbits.TRISF5=0; //SCL5 - output
    TRISDbits.TRISD8=1;  //INT1 - input
    TRISDbits.TRISD4=0;  //power pin - output
    //set RF4 and RF5 as open drain for I2C communication
    ODCFbits.ODCF4=1;
    ODCFbits.ODCF5=1;
    ODCDbits.ODCD4=0; //no open drain for power pin


//#endif 
    

    NumAccelerometers=3;//3;//3; //Number of accelerometers currently in use
    //ACCEL PORT1 - top of PCB (I2C3)
    memset(Accel,0,NumAccelerometers*sizeof(AccelStatus));  //clear Accel[0]

    
    
#if USE_FXOS8700CQ    
    Accel[0].I2CAddress=FXOS8700CQ_ADDRESS;
#endif     
#ifdef USE_MPU6050
    Accel[0].I2CAddress=MPU6050_ADDRESS;
#endif     
    Accel[0].I2CBus=I2C_ID_3;//tph I2C3; //set which I2C bus this accel uses
    //I2CxBRG=[(1/2*FSCK - TPGD)*PBCLK]-2]
    //see formula in PIC32 ref manual
    //I2C3BRG=0x70;//400khz  0x1e7;//100khz  with 100mhz PBCLK 0x182;//386 0x60;  //0x60 to be clearly less than 400khz-
    //possibly 100khz would cause less errors and still be fast enough?
    //at 100,000 bits/sec 1 bit every 10us, 20 bits still only takes 200us .2ms
    //I2C3CONbits.SMEN=1; //enable SMBus I/O pin threshold levels
    //I2C3CONbits.SIDL=0; //continue in idle mode
    //I2C3CONbits.A10M=0; //register is a 7-bit slave address
    //I2C3CONbits.I2CEN=1; //turn on I2C3 module
    //I2C3CONbits.ON=1; //turn on I2C3 module
    Accel[0].IntNum=0; //which ext interrupt this accel uses
    Accel[0].PowerPort=&LATD;  //RD6 powers accel#0
    Accel[0].PowerPinMask=0x0040; //pin 6
    IntAccel[0]=0; //int 0 is used by accel 0, reverse lookup table for external interrupt

   //ACCEL PORT2 - top of PCB on right of PORT2 (I2C1)
    //memset(&Accel[2],0,sizeof(AccelStatus));  //clear Accel[1]


#ifdef USE_FXOS8700CQ    
    Accel[1].I2CAddress=FXOS8700CQ_ADDRESS;
#endif     
#ifdef USE_MPU6050
    Accel[1].I2CAddress=MPU6050_ADDRESS;
#endif     
    Accel[1].I2CBus=I2C_ID_1;//tph I2C1; //set which I2C bus this accel uses
    //I2C1BRG=0x70; //400khz 0x1e7;//100khz
    //I2C1CONbits.SMEN=1; //enable SMBus I/O pin threshold levels
    //I2C1CONbits.SIDL=0; //continue in idle mode
    //I2C1CONbits.A10M=0; //register is a 7-bit slave address
    //I2C1CONbits.I2CEN=1; //turn on I2C1 module
    //I2C1CONbits.ON=1; //turn on I2C1 module
    Accel[1].IntNum=4; //which ext interrupt this accel uses
    Accel[1].PowerPort=&LATD;  //RD5 powers accel#0
    Accel[1].PowerPinMask=0x0020; //pin 5
    IntAccel[4]=1; //int 4 is used by accel 1, reverse lookup table for external interrupt

   //ACCEL PORT3 - right side of PCB (I2C5)
    //memset(&Accel[1],0,sizeof(AccelStatus));  //clear Accel[1]


#if USE_FXOS8700CQ    
    Accel[2].I2CAddress=FXOS8700CQ_ADDRESS;
#endif     
#if USE_MPU6050
    Accel[2].I2CAddress=MPU6050_ADDRESS;
#endif     
    Accel[2].I2CBus=I2C_ID_5;//tph I2C5; //set which I2C bus this accel uses
    //I2C5BRG=0x70; //400khz    0x1e7;//100khz
    //I2C5CONbits.SMEN=1; //enable SMBus I/O pin threshold levels
    //I2C5CONbits.SIDL=0; //continue in idle mode
    //I2C5CONbits.A10M=0; //register is a 7-bit slave address
    //I2C5CONbits.I2CEN=1; //turn on I2C5 module
    //I2C5CONbits.ON=1; //turn on I2C5 module
    Accel[2].IntNum=1; //which ext interrupt this accel uses
    Accel[2].PowerPort=&LATD;  //RD4 powers accel#0
    Accel[2].PowerPinMask=0x0010; //pin 4
    IntAccel[1]=2; //int 1 is used by accel 2, reverse lookup table for external interrupt

   //ACCEL PORT4 - bottom of PCB (I2C4)
    //memset(&Accel[3],0,sizeof(AccelStatus));  //clear Accel[1]
/*
    Accel[3].I2CBus=I2C4; //set which I2C bus this accel uses
    I2C4BRG=0x60;  //0x60 to be clearly less than 400khz
    I2C4CONbits.SMEN=1; //enable SMBus I/O pin threshold levels
    I2C4CONbits.SIDL=0; //continue in idle mode
    I2C4CONbits.A10M=0; //register is a 7-bit slave address
    I2C4CONbits.I2CEN=0; //turn on I2C4 module
*/
    //#endif


    //to initialize I2C a timeout is used which depends on SYS_TICK_Get()
    //and so call SYS_Initialize() first
    //tph this func is called from SYS_Initailize() SYS_Initialize();

/*
 * //moved to appInit() - needs I2C driver to be opened
    //start each accelerometer
    for(i=0;i<NumAccelerometers;i++) {
        Accel[i].handleI2C=DRV_HANDLE_INVALID;
        Reset_Accelerometer(i);
        Accel[i].Threshold=DEFAULT_ACCEL_THRESHOLD; //set interrupt acceleration threshold (absolute or relative depending on flag)
        Accel[i].flags|=ACCEL_STATUS_AUTOCALIBRATE; //autocalibrate is on by default
        Activate_Accelerometer(i);
    }
*/

    //enable timer 3 to poll the accelerometer and
    //touch sensor ADC channels
    //moved to app.c TimerInterval=DEFAULT_TIMER_INTERVAL;
    //currently default is 10 times a second (100ms) 100,000us
//#if 0 //   done by harmony
    //initialize Timer 2 count for motor PWM
    T2CON=0;//stop any 16/32-bit timer2 operation
    T3CON=0; //stop any 16-bit timer3 operation
    T2CONbits.ON=0; //clear the ON control bit to disable the timer
    //because the peripheral clock is 1:1 with the SYSCLK it is 80MHz (12.5ns)
    //for 80MHz: 12ns
    //prescale of 8=100ns
    //prescale of 64=800ns
    //prescale of 256=3.2us  but seems to be 1.8us ~7ns 6ns
    //T2CONbits.TCKPS=0x3; //prescale of 1:8
    T2CONbits.TCKPS=0x7; //prescale of 1:256, one tick every 3.2us
    T2CONbits.T32=1;  //32 bit timer
    //convert to 3.2us counts
    //ACCEL_POLL_INTERVAL is 100 (ms) by default
    //PR2=((int)(ACCEL_POLL_INTERVAL*312.5))&0xffff;
    //PR3=(((int)(ACCEL_POLL_INTERVAL*312.5))&0xffff0000)>>0x10;
    //there are 312.5 timer ticks in 1ms 3125 is 10ms, etc.

    //TimerInterval is used for both accelerometers and
    //for touch sensors to start the ADC interrupt
    //(which sends back a sample from all active touch sensors)

    TimerInterval=DEFAULT_TIMER_INTERVAL; //10 (ms)
    
    //convert from us to TIMER/8 100ns
    //duty cycle must also be divided by 2
    //in order to set the on and off transitions
    //PR2=((TimerInterval*10)>>1)&0xffff;
    //PR3=(((TimerInterval*10)>>1)&0xffff0000)>>0x10;
    PR2=(TimerInterval*3125/10)&0xffff;
    PR3=((TimerInterval*3125/10)&0xffff0000)>>0x10;
   // PR2=((TimerInterval*3906)/10)&0xffff;
   // PR3=(((TimerInterval*3906)/10)&0xffff0000)>>0x10;

    TMR2=0;
    TMR3=0;


    IFS0bits.T3IF=0; //clear the timer 2 interrupt flag
    //important: interrupt priority must match isr priority
    IPC3bits.T3IP=4;//3;//pic32mx had pri=4;//.INT2IP=4;  //priority - 3 bits
    IPC3bits.T3IS=3;//.INT2IS=2;  //subpriority -2 bits    
    IEC0bits.T3IE=1;        //enable Timer 2+3 interrupt- doesn't enable the timer
    //T2CONbits.ON=1; //enable timer2 (for 32-bit)
    //T3CONbits.ON=1; //enable timer3 (16-bit)
    //INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR); //configure multi vector
    //INTEnableSystemMultiVectoredInt(); //enable interrupts
    //ConfigIntTimer1(T1_INT_ON | T2_INT_PRIOR_3 | T2_INT_SUB_PRIOR_3);
    //ConfigIntTimer1(T2_INT_PRIOR_3 | T2_INT_SUB_PRIOR_3);
    //INTCONbits.MVEC=1; //interrupt controller configured for Multi-vector mode
    //LATGbits.LATG8=0; //pin for testing timer

    //set number of clocks in motor duty cycle
    //is 7 (but was 14), 7 timer2 interrupts make 1 full motor duty cycle
    //NumClocksInMotorDutyCycle=ROBOT_DEFAULT_NUM_CLKS_IN_MOTOR_DUTY_CYCLE;

//#endif
    
    
/* tph for now

    Initialize_TouchSensors();
*/

 
    //Initialize GPS
    //GPS pins
    TRISGbits.TRISG6=0; //RG6= GPS power pin
    TRISGbits.TRISG9=1; //RG9= GPS boot -leave floating as input
    
    //set up the output and input pins for UART1
    TRISGbits.TRISG8 = 0;  //RG8 = TX
    TRISGbits.TRISG7 = 1; //RG7 = RX 

    LATGbits.LATG6=0;  //make sure GPS is powered off initially


    
    //initialize UART3
/*
    //configure UART3 interrupt
    //U3RXIE
    IFS1bits.U3RXIF=0; //clear the UART3 rx interrupt flag
    //IFS1bits.U3TXIF=0; //clear the UART3 tx interrupt flag

    //important: interrupt priority must match isr priority
    IPC7bits.U3IP=5;//4; //priority - 3 bits
    IPC7bits.U3IS=2;//1; //subpriority -2 bits    
    //IEC1bits.U3RXIE=1; //enable UART3 receive interrupt
    
    
    IFS1bits.U3RXIF=0; //clear the UART3 rx interrupt flag
    //IFS1bits.U3TXIF=0; //clear the UART3 tx interrupt flag
    IEC1bits.U3RXIE=1; //enable UART3 receive interrupt            
    //IEC1bits.U3TXIE=1; //enable UART3 tx interrupt            

            //Note: do not enable the UART until the interrupt is enabled
            //because otherwise an overrun error will occur
    
    //9600bps is default baud but others are possible- see datasheet
    //for a peripheral bus (PBCLK) of 40MHz
    //UxBRG=Fpb/16*BaudRate -1 40e6/16*9600 -1 = 259.4166
    //for a peripheral bus (PBCLK) of 80MHz
    //UxBRG=Fpb/16*BaudRate -1 80e6/16*9600 -1 = 519.833
    U3BRG=520;//259; //9600bps
    U3STA   =   0;
    U3MODE  =   0x8000;            // Enable UART for 8-bit data
    // no parity, 1 Stop bit
    //Note: do not enable UART until interrupt is enabled
    //otherwise the receive buffer will not get cleared
    //and an overrun error will occur
    U3STASET = 0x1400;            // Enable Transmit and Receive
    //U3STASET = 0x1000;    
    // Enable Receive only 
    U3STAbits.UTXISEL=1; //interrupt is generated when the transmit buffer becomes empty
*/

    


    // configure for multi-vectored mode
    //INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    // enable interrupts
    //INTEnableInterrupts();

    
//#endif     

}

/*******************************************************************************
 End of File
*/
