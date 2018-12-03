//accelerometer.c
//Accelerometer (and gyroscope) functions


#include "accel.h"
#include "app.h" //for delayMs, ReadCoreTimer
#include "FXOS8700CQ.h"
#include "MPU6050.h"
#include "robot_accelmagtouchgps_pic_instructions.h" //robot EthAccelTouch PCB instruction codes

extern EthAccelStatus EAStatus; 
extern uint8_t NumAccelerometers;
extern AccelStatus Accel[MAX_NUM_ACCEL]; 
extern APP_DATA appData; 
extern uint8_t AccelTimerSend[ACCEL_POLL_SEND_SIZE];  //touch sensor packet data to send back to requester
extern uint32_t AccelTimerSendLen; //length of touch sensor send data packet
extern uint8_t NumActiveTouchSensors;

uint8_t Power_On_Accelerometer(uint8_t AccelNum)
{

    //power on the accelerometer
    *Accel[AccelNum].PowerPort|=Accel[AccelNum].PowerPinMask;
    DelayMs(1);

    return(1);
}

uint8_t Power_Off_Accelerometer(uint8_t AccelNum)
{
    //power off the accelerometer
    *Accel[AccelNum].PowerPort&=~Accel[AccelNum].PowerPinMask;
    DelayMs(1);

    return(1);
}

uint8_t Initialize_Accelerometers(void) 
{
//uintptr_t i2cOpStatus; //Operation status of I2C operation returned from callback
uint8_t i;  

    for(i=0;i<NumAccelerometers;i++) {
        Disable_Accelerometer_Interrupt(i); //disable the external interrupt initially
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
                        DRV_I2C_BufferEventHandlerSet(Accel[i].handleI2C,(DRV_I2C_BUFFER_EVENT_HANDLER)I2CCallBack_Accel0,NULL);
                    }
                break;
                case 1:
                    if (Accel[i].handleI2C!=DRV_HANDLE_INVALID) {
                        DRV_I2C_BufferEventHandlerSet(Accel[i].handleI2C,(DRV_I2C_BUFFER_EVENT_HANDLER)I2CCallBack_Accel1,NULL);
                    }
                break;
                case 2:
                    if (Accel[i].handleI2C!=DRV_HANDLE_INVALID) {
                        DRV_I2C_BufferEventHandlerSet(Accel[i].handleI2C,(DRV_I2C_BUFFER_EVENT_HANDLER)I2CCallBack_Accel2,NULL);
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

        }
    } //for i

} //uint8_t Initialize_Accelerometers(void) {

uint8_t Reset_Accelerometer(uint8_t AccelNum)
{
    uint8_t I2CData[10],result;    

    //this is a hardware reset- because that is the only way
    //to clear the accel I2C
    Power_Off_Accelerometer(AccelNum);   
    DelayMs(10);

    //power on the accelerometer
    Power_On_Accelerometer(AccelNum);
    DelayMs(10);

    //followed by a software reset    
#if USE_FXOS8700CQ 
    //I2CData[0]=Accel[AccelNum].I2CAddress;
    I2CData[0] = FXOS8700CQ_CTRL_REG2; //CNTL_REG2
    I2CData[1] = FXOS8700CQ_CTRL_REG2_RST;  //Reset
#endif //USE_FXOS8700CQ
#if USE_MPU6050
    I2CData[0] = MPU6050_PWR_MGMT_1; //CNTL_REG2
    I2CData[1] = MPU6050_PWR_MGMT_1_DEVICE_RESET;  //Reset    
#endif //USE_MPU6050
    
    
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);


    DelayMs(1); //definitely needed for I2C driver to start the 
    //transaction if no while after it because otherwise the i2c queue 
    //fills up. Need for waiting too- because complete may be returned before
    //inst is queued apparently.

    // Wait for the signal to complete
//#if 0     
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);
    
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {
    switch(result) {
        case I2C_INST_TIMEOUT:
            SYS_CONSOLE_PRINT("I2C write of Accel %d timed out. Disabling.\r\n",AccelNum);
            Power_Off_Accelerometer(AccelNum); 
            Accel[AccelNum].flags&=~ACCEL_STATUS_ENABLED;            
            return(0);
        break;
        case I2C_INST_ERROR: 
            SYS_CONSOLE_PRINT("I2C write of Accel %d had error. Disabling.\r\n",AccelNum);
            Power_Off_Accelerometer(AccelNum); 
            Accel[AccelNum].flags&=~ACCEL_STATUS_ENABLED;
            return(0);
        break;
        default:
        break;
    } //switch

    DelayMs(100);  //wait for device to fully reset
    
    return(1);

} //uint8_t Reset_Accelerometer(uint8_t num)


uint8_t Initialize_Accelerometer(uint8_t AccelNum)
{
    uint8_t I2CData[10],result,ReturnByte;
    //uint8_t ReturnValue;
    //uint32_t Timeout;

    //start accelerometer

    //the firmware thinks there are 3 accels but some may be
    //disconnected so we want to make sure this failing doesn't
    //cause any problem

    
    if (!(Accel[AccelNum].flags&ACCEL_STATUS_ENABLED)) {
        //not enabled (but INITIALIZED flag can still be set)
        Accel[AccelNum].flags&=~ACCEL_STATUS_INITIALIZED;
    }
    
//    if (Accel[AccelNum].flags&ACCEL_STATUS_INITIALIZED) {
        //already enabled and initialized
//        return(1);
//    }



    //commented out in case I want to test with a different accel
    /*
     //in the data sheet example,
    //the FXOS8700CQ WHOAMI register data is verified here
    I2CData[1] = FXOS8700CQ_WHOAMI;

    ReturnValue=0;

    //wait  10ms at most
    Timeout= SYS_TICK_Get() + ((10 * SYS_TICK_ResolutionGet()) + 999)/1000;


    while(!ReturnValue) {

        //write 2 bytes read 1 byte
        ReturnValue=ReadI2CData(I2CData,2,Accel[num].I2CBus);
        if (ReturnValue==0xc7) {
            ReturnValue=1;
        } else {
            return(0);
        }

        if (SYS_TICK_Get()>=Timeout) {
            //timed out - do not try to reset or re-activate accel,
             //because an accel might not be connected
              //and the program would endlessly try to reach it
            //Reset_Accelerometer(num);
            //Activate_Accelerometer(num);
            return(0);
        }
    } //while(!ReturnValue) {
*/

    
#if USE_FXOS8700CQ    
    //put the accelerometer into standby
    Accelerometer_StandByMode(AccelNum);

    //I2CData[0]=Accel[num].I2CAddress;


    //activate the magnetometer
    // write 0001 1111 = 0x1F to magnetometer control register 1
    // [7]: m_acal=0: auto calibration disabled
    //if 1: Auto-calibration feature is enabled; the ASIC uses the maximum and minimum magnetic data to determine the
    //hard iron offset value. The M_OFF_X/Y/Z registers are automatically loaded with (MAX_X/Y/Z + MIN_X/Y/Z)/2 for
    //each axis at the end of every ODR cycle.
    // [6]: m_rst=0: no one-shot magnetic reset
    // [5]: m_ost=0: no one-shot magnetic measurement
    // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
    // [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
    I2CData[0] = FXOS8700CQ_M_CTRL_REG1;
    //I2CData[2] = 0x1f;  //see above
    //currently m_raw is always enabled, so the M_OFF offsets
    //are always added to the final value
    //turning off autocalibration stops M_OFF from being
    //overwritten by min+max/2
    if (Accel[AccelNum].flags&ACCEL_STATUS_AUTOCALIBRATE) {
        //autocalibrate enabled
        I2CData[1] = 0x9f;  //see above       
    } else {
        //autocalibrate not enabled
        //I2CData[2] = 0x80;  //see above    
        I2CData[1] = 0x1f;  //see above    
    }

//    I2CData[2] = 0x9f;  //see above - uses autocalibration - use  M_CTRL_REG3 m_raw to stop the adding of an autocalculated hard iron offset
    //found Z is always negative, X has -15 to 17, and Y has -18 to +12.
    //but it seems to take a minute to stabilize, and then the max/min points seemed to change- were not always the same
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    //note that this is a transmit buffer handle and so we never use it again
    //after this
    
    DelayMs(1); //need or i2c queue may become full unless waiting here
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
    //if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        return(0);
    }

    // write 0010 0000 = 0x20 to magnetometer control register 2
    // [7]: reserved
    // [6]: reserved
    // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the
    // accelerometer registers
    // [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
    // [3]: m_maxmin_dis_ths=0
    // [2]: m_maxmin_rst=0
    // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
    I2CData[0] = FXOS8700CQ_M_CTRL_REG2;
    I2CData[1] = 0x20;  //see above
    //I2CData[2] = 0x00;  //see above
    
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
//    if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        return(0);
    }

    
    

    // write 1000 0000 = 0x80 to magnetometer control register 3
    // [7]: Magnetic measurement RAW mode enable:
    //0: Values stored in the M_OFF_X/Y/Z registers are applied to the magnetic sample data. This bit must be cleared 
    //in order for the automatic hard-iron com
    //pensation function to have any effect. 
    //1: Values stored in M_OFF_X/Y/Z are not applied to the magnetic sample data; automatic hard-iron compensation 
    //function does not have any effect on the output data.reserved
//    I2CData[1] = FXOS8700CQ_M_CTRL_REG3;
//    if (Accel[num].flags&ACCEL_STATUS_AUTOCALIBRATE) {
        //mraw enabled
//        I2CData[2] = 0x00;  //see above       
//    } else {
        //mraw not enabled
        //I2CData[2] = 0x80;  //see above    
//    }
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
//        return(0);
//    }

    //no need to write to XYZ_DATA_CFG register because default is +-2g
    //but possibly a low-pass filter should be added because
    //robot movements may change acceleration readings
    //see register after XYZ_DATA_CFG, HP_FILTER_CUTOFF

//#if 0 
    I2CData[0] = FXOS8700CQ_CTRL_REG2; //CNTL_REG1
    //I2CData[2] = FXOS8700CQ_CTRL_REG1_DR_2|FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE;  //12.5 samples/sec
      //200 samples/sec Hybrid mode, reduced noise mode
    I2CData[1] = 0x2; //high resolution mode

    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        return(0);
    }
//#endif 
    
//#if 0     
    I2CData[0] = FXOS8700CQ_HP_FILTER_CUTOFF; 
    //for now trying: ODR=400hz Cutoff=16hz (ODR=200hz Cutoff=16Hz)
    //for high resolution mode
    //hopefully this will remove high changing accelerations, 
    //but recognize when there is a fast and large change due to gravity.
    //it's not clear to me how to equate cutoff frequency to change in g.
    //a cust support rep said these filters are in place for all sampling,
    //not just pulse processing.
    I2CData[1] = 0x30; //HPF bypassed, LPF enabled (sel=00 odr=400hz cutoff 16hz, 
    //but because of hybrid mode odr=200hz with cutoff of 8hz))

    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        return(0);
    }
//#endif 
    
    //configure and enable accelerometer by writing to Control Register 1
    //if using the acceleration vector-magnitude function,
    //the output data rate (ODR) can be no faster than 400hz (2.5ms period)
    //appnote AN4692 sets this to 12.5hz (80ms)
    //the default and maximum possible is 800hz (1.25ms)
    //for balancing a robot body segment I doubt I would need anything
    //faster than 100/second (10ms)
    //I could set this faster, and raise the debouncing count
    //I currently set this to 400hz (400 samples/second but is 200hz in hybrid mode- 
    //both accel and mag sampling) dr=001
    //in hybrid mode (both accel+magnetic) sample rate is half just reading one or the other
    //currently (400hz regular) 200Hz 5ms period is max sample speed I set (but it can be faster if I need it to)
    I2CData[0] = FXOS8700CQ_CTRL_REG1; //CNTL_REG1
    //I2CData[2] = FXOS8700CQ_CTRL_REG1_DR_2|FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE;  //12.5 samples/sec
      //200 samples/sec Hybrid mode, reduced noise mode
    //I2CData[1] = FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE|FXOS8700CQ_CTRL_REG1_LNOISE;
    //400 sample/sec Hybrid mode, reduced noise mode
    I2CData[1] = FXOS8700CQ_CTRL_REG1_ACTIVE|FXOS8700CQ_CTRL_REG1_LNOISE;
#endif //USE_FXOS8700CQ    
#if USE_MPU6050

//#if 0 
    //enable digital low pass filter
    I2CData[0] = MPU6050_CONFIG;
    I2CData[1] = MPU6050_CONFIG_DLPF_CFG_1;

    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        1, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        SYS_CONSOLE_PRINT("Write to MPU6050_CONFIG failed.\n\r");
        return(0);
    }
//#endif 
    
#if 0     
    I2CData[0] = MPU6050_PWR_MGMT_1;

    Accel[AccelNum].I2CBufferHandle=DRV_I2C_TransmitThenReceive(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        1, 
                                                        &ReturnByte,
                                                        1,
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        SYS_CONSOLE_PRINT("Read of MPU6050_PWR_MGMT_1 failed.\n\r");
        return(0);
    }

    SYS_CONSOLE_PRINT("MPU6050_PWR_MGMT_1= %d\n\r",ReturnByte);
#endif

//#if 0 
//Sample rate divider    
    I2CData[0] = MPU6050_SMPRT_DIV;
    I2CData[1] = 4;//9;  //equals sample rate/1+9 = for accel 1khz/10 = 100hz

    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        1, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        SYS_CONSOLE_PRINT("Write to MPU6050_SMPRT_DIV failed.\n\r");
        return(0);
    }
//#endif 
    
#if 0     
//enable fifo      
    I2CData[0] = MPU6050_FIFO_EN;
    I2CData[1] = MPU6050_FIFO_EN_TEMP_FIFO_EN|
            MPU6050_FIFO_EN_XG_FIFO_EN|
            MPU6050_FIFO_EN_YG_FIFO_EN|
            MPU6050_FIFO_EN_ZG_FIFO_EN|
            MPU6050_FIFO_EN_ACCEL_FIFO_EN;

    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        1, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        SYS_CONSOLE_PRINT("Write to MPU6050_SMPRT_DIV failed.\n\r");
        return(0);
    }
#endif 
    
    
    //take accel out of sleep/power save mode and bring into regular operation mode
    //and set clock to PLL with Z axis gyroscope reference  (0x3)
    I2CData[0] = MPU6050_PWR_MGMT_1;
    I2CData[1] = 3;//3;//0;


#endif //USE_MPU6050
    
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result==I2C_INST_COMPLETE) {
//    if (WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
       // Accel[AccelNum].flags|=ACCEL_STATUS_ACTIVE; //set active flag
        Accel[AccelNum].flags|=ACCEL_STATUS_INITIALIZED; //set active flag        
        return(1);
    } else {
        return(0);
    }



}//uint8_t Initialize_Accelerometer(uint8_t AccelNum)

uint8_t Accelerometer_StandByMode(uint8_t AccelNum)
{
    uint8_t I2CData[10];
    uint8_t I2CByte,result;

    
#if USE_FXOS8700CQ
    //I2CData[0]=Accel[num].I2CAddress;
    //start accelerometer
    I2CData[0] = FXOS8700CQ_CTRL_REG1; //CNTL_REG1
    I2CData[1] = 0;
    Accel[AccelNum].I2CBufferHandle=DRV_I2C_Transmit(Accel[AccelNum].handleI2C,
                                                        Accel[AccelNum].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1);    
//    if (Accel[AccelNum].I2CBufferHandle != NULL) {
    result=WaitForI2C(Accel[AccelNum].handleI2C,Accel[AccelNum].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result==I2C_INST_COMPLETE) {   
//    if (WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        Accel[AccelNum].flags&=~ACCEL_STATUS_INITIALIZED; //clear initialized flag
        //not needed but just in case: wait 1ms
        //DelayMs(1);
        return(1);
    } else {
        Reset_Accelerometer(AccelNum);
        return(0);
    }
#endif //USE_FXOS8700CQ
    return(1);
}//uint8_t Accelerometer_StandByMode(uint8_t AccelNum)




uint8_t Enable_Accelerometer_Interrupt(uint8_t num)
{

    uint8_t I2CData[20];
    uint8_t I2CByte,result,NumBytes;
    uint8_t   RelativeInt;
    uint16_t  AccelThresh;

#if USE_FXOS8700CQ
    //SYS_CONSOLE_PRINT("Enable_Accelerometer_Interrupt %d\r\n",num);
    //SYS_CONSOLE_PRINT("En %i\r\n",num);
    
    RelativeInt=0;
    if (Accel[num].flags&ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION) {
        //This accelerometer is being set to interrupt on a change in relative acceleration
        RelativeInt=1;
    }


    //put the accelerometer into standby mode
    Accelerometer_StandByMode(num);

    //enable the interrupt on the accelerometer
    //I2CData[0]=Accel[num].I2CAddress;

    if (RelativeInt) {
        //relative acceleration (vector-magnitude) interrupt
        I2CData[0] = FXOS8700CQ_A_VECM_CFG; //acceleration-vector config
        //a_vecm_ele=1: event latch enable, int flag is latched and held until host app reads INT_SOURCE reg (0x0C)
        //a_vecm_init=0: initial reference is current x,y,z,
        //a_vecm_updm=0: ref value is updated with x,y,z data after int occurs
        //a_vecm_en=1: the accelerometer vector-magnitude function is enabled
        //I2CData[1] = FXOS8700CQ_A_VECM_CFG_ELE|FXOS8700CQ_A_VECM_CFG_EN;
        I2CData[1] = FXOS8700CQ_A_VECM_CFG_EN;
        //note that ELE doesn't matter for the vector-magnitude interrupt
        //don't bother to latch the interrupt values
        //I2CData[2] = FXOS8700CQ_A_VECM_CFG_EN;
    } else {
        //absolute acceleration (freefall and motion) interrupt
        I2CData[0] = FXOS8700CQ_FF_MT_CFG; //Freefall and motion config
        //ele= event latch enable
        //oae= 0=freefall AND of low-g and X,Y,Z event flags
        //     1=motion OR of high-g and X,Y,Z event flags
        //I2CData[1] = FXOS8700CQ_FF_MT_CFG_ELE|FXOS8700CQ_FF_MT_CFG_OAE;
        I2CData[1] = FXOS8700CQ_FF_MT_CFG_OAE;
        //don't bother to latch the values
        //I2CData[2] = FXOS8700CQ_FF_MT_CFG_OAE;
        //monitor the accel on X and Y
        I2CData[1]|=FXOS8700CQ_FF_MT_CFG_XEFE|FXOS8700CQ_FF_MT_CFG_YEFE;
        //I2CData[2]|=FXOS8700CQ_FF_MT_CFG_ZEFE;
    } //if (RelativeInt) {

//    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
//        Reset_Accelerometer(num);
//        return(0);
//    }
    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        return(0);
    }



    //set the threshold value 0g=0 1g=0x3ff -1g=c00
    //CData[0]=Accel[num].I2CAddress;

    if (RelativeInt) {
        I2CData[0]= FXOS8700CQ_A_VECM_THS_MSB; //vector threshold
        //resolution of the 13-bit unsigned A_VECM_THS is = to the resolution set in XYZ_DATA_CFG (fs)
        //which is by default the most precise= +-0.244mg/LSB
        //user threshold is in mg, so to convert:
        //(desired accel)/0.244 or *4.4643 (note appnote AN4692 has 0.224)
        //so for 100mg,  100/0.244=~410
        //410*0.244=100mg
        //note: debounce 0x80 on MSB =0, the debounce counter is decremented by 1
        //when the vector-magnitude result is below the programmed threshold value
        //when=1 the debounce counter is cleared when the vector-magnitude result
        //is below the programmed threshold value
        //my debounce count is 1 so it doesn't matter much
        AccelThresh=(uint16_t)(((float)Accel[num].Threshold)/0.244);
        //AccelThresh=258;
        //SYS_CONSOLE_PRINT("AccelThresh=%d\r\n",AccelThresh);
        I2CData[1]=(uint8_t)((AccelThresh&0xff00)>>8);
        I2CData[2]=(uint8_t)(AccelThresh&0xff);
        //note: 16-bit threshold
        NumBytes=3;
//        if (!WriteI2CData(I2CData,4,Accel[num].I2CBus)) {
//            Reset_Accelerometer(num);
//            return(0);
//        }
    } else {
        I2CData[0] = FXOS8700CQ_FF_MT_THS; //Freefall and motion config
        //threshold is 0.063g/LSB range 0-127 counts
        //Threshold is in mg, to convert/63
        I2CData[1]=(uint8_t)(Accel[num].Threshold/63);
        //I2CData[2]=(uint8_t)(Accel[num].Threshold/0.063);
        //I2CData[2]=(uint8_t)(DEFAULT_ACCEL_THRESHOLD/0.063);
        //note: 8-bit threshold
        //if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        //    Reset_Accelerometer(num);
        //    return(0);
        //}
        NumBytes=2;
    } //if (RelativeInt) {
    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        return(0);
    }

    
    //set the debounce count value
    //I2CData[0]=Accel[num].I2CAddress;

    //The debounce timer period is determined by the ODR and = to A_VECM_CNT/ODR
    //for example, when ODR=400Hz, A_VEM_CNT=16 results in a debounce period of 40ms
    //currently, like example =1 1*80ms=80ms
    if (RelativeInt) {
        I2CData[0] = FXOS8700CQ_A_VECM_CNT; //Acceleration vector-magnitude count
    } else {
        I2CData[0] = FXOS8700CQ_FF_MT_COUNT; //Freefall and motion count
    } //if (RelativeInt) {

    //I2CData[1] = 0x01;  //number of sample counts for the event trigger
    //at 400hz, 4samples=10ms - important to be>1samp because otherwise 
    //interrupt flag will not clear and data will not get sent
    I2CData[1] = 0x04;  //number of sample counts for the event trigger
    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        return(0);
    }

    //configure which interrupt will be set, int1 or int2
    //I2CData[0]=Accel[num].I2CAddress;

    I2CData[0] = FXOS8700CQ_CTRL_REG5;
    if (RelativeInt) {
        I2CData[1] = FXOS8700CQ_CTRL_REG5_INT_CFG_A_VECM; //0=int2 1=int1
    } else {
        I2CData[1] = FXOS8700CQ_CTRL_REG5_INT_CFG_FF_MT; //0=int2 1=int1
    }

    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        return(0);
    }


    //enable the freefall/motion or acceleration vector-magnitude interrupt
    //I2CData[0]=Accel[num].I2CAddress;

    I2CData[0] = FXOS8700CQ_CTRL_REG4;
    if (RelativeInt) {
        //enable the acceleration vector-magnitude interrupt
        I2CData[1] = FXOS8700CQ_CTRL_REG4_INT_EN_A_VECM;
    } else {
        //enable the freefall/motion interrupt
        I2CData[1] = FXOS8700CQ_CTRL_REG4_INT_EN_FF_MT;
    } //if (RelativeInt) {

    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        return(0);
    }

   

/*    
    //enable accel
    I2CData[0] = FXOS8700CQ_CTRL_REG1; //CNTL_REG1
    //I2CData[2] = FXOS8700CQ_CTRL_REG1_DR_2|FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE;  //12.5 samples/sec
      //200 samples/sec Hybrid mode, reduced noise mode
    //I2CData[1] = FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE|FXOS8700CQ_CTRL_REG1_LNOISE;
    //400 sample/sec Hybrid mode, reduced noise mode
    I2CData[1] = FXOS8700CQ_CTRL_REG1_ACTIVE|FXOS8700CQ_CTRL_REG1_LNOISE;

    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result==I2C_INST_COMPLETE) {
//    if (WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
       // Accel[AccelNum].flags|=ACCEL_STATUS_ACTIVE; //set active flag
        Accel[num].flags|=ACCEL_STATUS_INITIALIZED; //set active flag        
        return(1);
    } else {
        return(0);
    }
*/

    
    
    //put the accelerometer into active mode
    Initialize_Accelerometer(num);
#if 0     
    //configure and enable accelerometer by writing to Control Register 1
    //if using the acceleration vector-magnitude function,
    //the output data rate (ODR) can be no faster than 400hz (2.5ms period)
    //appnote AN4692 sets this to 12.5hz (80ms)
    //the default and maximum possible is 800hz (1.25ms)
    //for balancing a robot body segment I doubt I would need anything
    //faster than 100/second (10ms)
    //I could set this faster, and raise the debouncing count
    //I currently set this to 400hz (400 samples/second but is 200hz in hybrid mode- 
    //both accel and mag sampling) dr=001
    //in hybrid mode (both accel+magnetic) sample rate is half just reading one or the other
    //currently (400hz regular) 200Hz 5ms period is max sample speed I set (but it can be faster if I need it to)
    I2CData[0] = FXOS8700CQ_CTRL_REG1; //CNTL_REG1
    //I2CData[2] = FXOS8700CQ_CTRL_REG1_DR_2|FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE;  //12.5 samples/sec
      //200 samples/sec Hybrid mode, reduced noise mode
    //I2CData[1] = FXOS8700CQ_CTRL_REG1_DR_0|FXOS8700CQ_CTRL_REG1_ACTIVE|FXOS8700CQ_CTRL_REG1_LNOISE;
    //400 sample/sec Hybrid mode, reduced noise mode
    I2CData[1] = FXOS8700CQ_CTRL_REG1_ACTIVE|FXOS8700CQ_CTRL_REG1_LNOISE;

    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result==I2C_INST_COMPLETE) {
//    if (WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
       // Accel[AccelNum].flags|=ACCEL_STATUS_ACTIVE; //set active flag
        Accel[num].flags|=ACCEL_STATUS_INITIALIZED; //set active flag        
        return(1);
    } else {
        return(0);
    }

#endif     
#endif //USE_FXOS8700CQ
    

#if USE_MPU6050
    I2CData[0] = MPU6050_INT_PIN_CFG; 
    I2CData[1] = MPU6050_INT_PIN_CFG_INT_LEVEL|
            MPU6050_INT_PIN_CFG_LATCH_INT_EN|
            MPU6050_INT_PIN_CFG_INT_RD_CLEAR;

    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        SYS_CONSOLE_PRINT("Failed to write to INT_PIN_CFG reg, accel=%d\r\n",num);
        return(0);
    }
    

    I2CData[0] = MPU6050_INT_ENABLE; 
    I2CData[1] = MPU6050_INT_ENABLE_DATA_RDY_EN;
    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        SYS_CONSOLE_PRINT("Failed to write to INT_PIN_CFG reg, accel=%d\r\n",num);
        return(0);
    }

    
#endif //USE_MPU6050

    
    
    //enable the external interrupt on the PIC
    //todo: see if just storing actual registers and bit masks is a better solution here
    switch(Accel[num].IntNum) {
        case 0: //Interrupt 0
            //SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_0);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_0);
            //SYS_INT_VectorPrioritySet(INT_VECTOR_INT0, INT_PRIORITY_LEVEL4);
            //SYS_INT_VectorSubprioritySet(INT_VECTOR_INT0, INT_SUBPRIORITY_LEVEL2);
            //SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE0,INT_EDGE_TRIGGER_FALLING);
            SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_0);
/*            
            IEC0bits.INT0IE=0; //disable interrupt
            INTCONbits.INT0EP=0; //int polarity 0=falling edge
            IFS0bits.INT0IF=0; //clear int flag
            //IPC0bits.INT0IP=4; //int priority
            IPC0bits.INT0IP=4; //int priority
            IPC0bits.INT0IS=2;  //int subpriority
            IEC0bits.INT0IE=1;  //enable the interrupt
 */
        break;
        case 1: //Interrupt 1
            //SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_1);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
            //SYS_INT_VectorPrioritySet(INT_VECTOR_INT1, INT_PRIORITY_LEVEL4);
            //SYS_INT_VectorSubprioritySet(INT_VECTOR_INT1, INT_SUBPRIORITY_LEVEL2);
            //SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE1,INT_EDGE_TRIGGER_FALLING);
            SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_1);
/*
            IEC0bits.INT1IE=0; //disable interrupt
            INTCONbits.INT1EP=0; //int polarity 0=falling edge
            IFS0bits.INT1IF=0; //clear int flag
            IPC2bits.INT1IP=4; //int priority
            IPC2bits.INT1IS=2;  //int subpriority
            IEC0bits.INT1IE=1;  //enable the interrupt
 */
        break;
        case 4: //Interrupt 4
            //SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_4);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_4);
            //SYS_INT_VectorPrioritySet(INT_VECTOR_INT4, INT_PRIORITY_LEVEL4);
            //SYS_INT_VectorSubprioritySet(INT_VECTOR_INT4, INT_SUBPRIORITY_LEVEL2);
            //SYS_INT_ExternalInterruptTriggerSet(INT_EXTERNAL_INT_SOURCE4,INT_EDGE_TRIGGER_FALLING);
            SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_4);
/*          IEC0bits.INT4IE=0; //disable interrupt
            INTCONbits.INT4EP=0; //int polarity 0=falling edge
            IFS0bits.INT4IF=0; //clear int flag
            IPC5bits.INT4IP=4; //int priority
            IPC5bits.INT4IS=2;  //int subpriority
            IEC0bits.INT4IE=1;  //enable the interrupt
*/
            break;
    } //switch(num)
    
    return(1);
}//uint8_t Enable_Accelerometer_Interrupt(uint8_t num)

uint8_t Disable_Accelerometer_Interrupt(uint8_t num)
{

    uint8_t I2CData[10];
    uint8_t I2CByte,result;


    //disable the external interrupt on the PIC
    //todo: see if just storing actual registers and bit masks is a better solution here
    switch(Accel[num].IntNum) {
        case 0: //Interrupt 0
            SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_0);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_0);
            //IEC0bits.INT0IE=0; //disable interrupt
            //IFS0bits.INT0IF=0; //clear int flag
        break;
        case 1: //Interrupt 1
            SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_1);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
            //IEC0bits.INT1IE=0; //disable interrupt
            //IFS0bits.INT1IF=0; //clear int flag
        break;
        case 4: //Interrupt 4
            SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_4);
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_4);            
            //IEC0bits.INT4IE=0; //disable interrupt
            //IFS0bits.INT4IF=0; //clear int flag
        break;
    } //switch(num)

    //wait for any accelerometer interrupt handler to stop processing (and send last UDP packet)
    //WaitForAccelInterrupt(num,100); //100 ms timeout
    
    
    //wait for the I2C bus to become idle
    //WaitForI2CBusIdle(Accel[num].I2CBus,TimerInterval*2);
    DelayMs(1);

#if USE_FXOS8700CQ    
    
    //put the accelerometer into standby mode
    Accelerometer_StandByMode(num);

    //disable (all accel interrupts) including the freefall/motion interrupt
    //I2CData[0]=Accel[num].I2CAddress;

    I2CData[0] = FXOS8700CQ_CTRL_REG4;

    I2CData[1] = 0;
/*    if (!WriteI2CData(I2CData,3,Accel[num].I2CBus)) {
        Reset_Accelerometer(num);
        return(0);
    }*/
    Accel[num].I2CBufferHandle=DRV_I2C_Transmit(Accel[num].handleI2C,
                                                        Accel[num].I2CAddress,
                                                        I2CData,
                                                        2, 
                                                        NULL);
    DelayMs(1); //need or i2c queue may become full
    //if (Accel[AccelNum].I2CBufferHandle != NULL) {        
    result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (Accel[AccelNum].I2CBufferHandle == NULL) {
    if (result!=I2C_INST_COMPLETE) {
        return(0);
    }
    

    //put the accelerometer into active mode
    Initialize_Accelerometer(num);
#endif //USE_FXOS8700CQ    
    
    return(1);
}//uint8_t Disable_Accelerometer_Interrupt(uint8_t num)


//Configure one or more accelerometers
//Note ENABLED flag needs to be included, or the accel selected will be disabled
//accelerometers may be enabled or disabled by using a 16-bit mask
//if enable=1 any 1s in the mask enable the sensor, 0s remain in the same state
//if enable=0 any 1s in the mask disable the sensor, 0s remain in the same state
//flags:
// - enable, polling, interrupt
//Threshold sets the interrupt threshold acceleration if >0
uint8_t ConfigureAccelerometers(uint16_t mask,uint32_t flags,uint16_t Threshold)
{
    uint8_t i,NumEnabledAccelerometers,NumPolling,NumInterrupting;

    //reset number of active accelerometer
    NumEnabledAccelerometers=0;    
    NumPolling=0;  
    NumInterrupting=0;
    

    for(i=0;i<NumAccelerometers;i++) {

        if (mask&(1<<i)) { //accel is selected
            //enable or disable the accel
            if (flags&ACCEL_STATUS_ENABLED) { //enable accel
                if (!(Accel[i].flags&ACCEL_STATUS_ENABLED)) { //Accel not already enabled
                    //power on and initialize accel
                    Power_On_Accelerometer(i);
                    if (!Initialize_Accelerometer(i)) {
                        //could not initialize accelerometer
                        //so disable it
                        SYS_CONSOLE_PRINT("Accel %d cannot be initialized, disabling it\r\n",i);
                        Accel[i].flags&=~ACCEL_STATUS_ENABLED;                            
                    } else {
                        Accel[i].flags|=ACCEL_STATUS_ENABLED;
                        SYS_CONSOLE_PRINT("Accel %d was enabled\r\n",i);
                    }                    
                } else { //if (!(Accel[i].flags&ACCEL_STATUS_ENABLED)) {
                    //Accel is already enabled
                    if ((Accel[i].flags&ACCEL_STATUS_INTERRUPT) && (flags&ACCEL_STATUS_POLLING)) {
                        //accel is changing from INTERRUPT to POLLING
                        //future: possibly disable the accel and PIC interrupts
                        //unset interrupt flag
                        Accel[i].flags&=~ACCEL_STATUS_INTERRUPT;
                        Accel[i].flags&=~ACCEL_STATUS_GOT_INTERRUPT;
                        //WaitForI2CBusIdle(Accel[i].I2CBus,TimerInterval*2);
                        //Disable_Accelerometer_Interrupt(i); //will deactivate
                        Reset_Accelerometer(i);
                        if (!Initialize_Accelerometer(i)) {
                            //return(0);
                        } //if (!Activate_Accelerometer(i)) {
                    } //if ((Accel[i].flags&ACCEL_STATUS_INTERRUPT) && (flags&ACCEL_STATUS_POLLING)) {
                } ////if (!(Accel[i].flags&ACCEL_STATUS_ENABLED)) {
                //ActiveAccel[NumEnabledAccelerometers]=i; //remember accel num
                if (flags&ACCEL_STATUS_INTERRUPT) { //set accel to interrupt
                    if (Threshold>0) { //user sent threshold
                        Accel[i].Threshold=Threshold; //in mg
                    } //if (Threshold>0) { //user sent threshold
                    if (Accel[i].flags&ACCEL_STATUS_POLLING) { //accel was polling
                        //disable the polling flag and wait 2*Timer for I2C bus to be idle
                        //in case a sample is in the process of being sent or received
                        Accel[i].flags&=~ACCEL_STATUS_POLLING;
                        DelayMs(1);
                        //WaitForI2CBusIdle(Accel[i].I2CBus,TimerInterval*2);                    
                    }//if (Accel[AccelNum].flags&ACCEL_STATUS_POLLING) {
                    //if (!(Accel[i].flags&ACCEL_STATUS_INTERRUPT)) { //Accel not already set to interrupt
                    //enable the PIC external interrupt and accelerometer interrupt
                    Accel[i].flags|=ACCEL_STATUS_INTERRUPT;
                    if (flags&ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION) {  //have to set relative flag here for Enable_Accelerometer_Interrupt(i) function
                        Accel[i].flags|=ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION;
                    } else {
                        Accel[i].flags&=~ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION;
                    }                       
                    //}
                    Enable_Accelerometer_Interrupt(i);
                    
                } //if (flags&ACCEL_STATUS_INTERRUPT) { //set accel to interrupt
                if (flags&ACCEL_STATUS_POLLING) { 
                    Accel[i].flags|=ACCEL_STATUS_POLLING;
                    if (flags&ACCEL_STATUS_SINGLE_SAMPLE) {
                        Accel[i].flags|=ACCEL_STATUS_SINGLE_SAMPLE;
                        //SYS_CONSOLE_PRINT("Accel %d set to single sample\r\n",i);
                    }
                    
                }
            } else { //if (flags&ACCEL_STATUS_ENABLED) 
                //disable these accelerometers

                //To deactivate an accel, caution is needed
                //because the timer interrupt may be in the middle
                //of talking to the accel when the accel is disabled.              
                             
                if (Accel[i].flags&ACCEL_STATUS_INTERRUPT) {
                    //disable the PIC external interrupt and accelerometer interrupt
                    Disable_Accelerometer_Interrupt(i);
                    Accel[i].flags&=~ACCEL_STATUS_INTERRUPT;
                    Accel[i].flags&=~ACCEL_STATUS_GOT_INTERRUPT;
                }
                //Accelerometer_StandByMode(i); //set accel to standby using i2c
                if (flags&ACCEL_STATUS_POLLING) { 
                    Accel[i].flags&=~ACCEL_STATUS_POLLING;
                    if (flags&ACCEL_STATUS_SINGLE_SAMPLE) {
                        Accel[i].flags&=~ACCEL_STATUS_SINGLE_SAMPLE;
                    }
                }
                DelayMs(1); //wait for all I2C communication to stop              
                Accel[i].flags&=~ACCEL_STATUS_ENABLED;
                Power_Off_Accelerometer(i); 
            } //if (flags&ACCEL_STATUS_ENABLED) { //enable these accels
        } else { //if (mask&(1<<i)) {
            //no mask, these sensors stay in the same state
            if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
             //   ActiveAccel[NumEnabledAccelerometers]=i;
                NumEnabledAccelerometers++;
            }
        } //if (mask&(1<<i)) {

        if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
            NumEnabledAccelerometers++;
        }
        if (Accel[i].flags&ACCEL_STATUS_POLLING) {
            NumPolling++;
        }
        if (Accel[i].flags&ACCEL_STATUS_INTERRUPT) {
            NumInterrupting++;
        }

    } //for i

    
    
    if (NumInterrupting>0) {
        EAStatus.flags|=ETHACCEL_STATUS_ACCEL_INTERRUPT;
        //SYS_CONSOLE_PRINT("EAStatus interrupt\r\n");
    } else {
        EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_INTERRUPT;
    }
    if (NumPolling>0) {
        EAStatus.flags|=ETHACCEL_STATUS_ACCEL_POLLING;
        //SYS_CONSOLE_PRINT("EAStatus polling\r\n");
    } else {
        EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_POLLING;
    }
//#if 0     
    if (NumEnabledAccelerometers==0 && NumActiveTouchSensors==0) {
        //SYS_CONSOLE_PRINT("no enabled accels- stopping EAStatus interrupt and polling\r\n");
        //no active accelerometers clear poll and interrupt flags
        EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_INTERRUPT;
        EAStatus.flags&=~ETHACCEL_STATUS_ACCEL_POLLING;        
        if (!(EAStatus.flags&ETHACCEL_STATUS_TOUCH_SENSOR_POLLING) &&
            !(EAStatus.flags&ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT)) {
            //no interrupt or polling on touch sensors or accels, so stop timer
            T2CONbits.ON=0; //disable timer2+3 (for 32-bit)
            //IFS0bits.T3IF=0; //clear interrupt flag
            //IEC0bits.T3IE=0; //disable Timer 2+3 interrupt- doesn't enable the timer
        } else {
            if (!T2CONbits.ON) {
                T2CONbits.ON=1; //enable timer2+3 (for 32-bit)    
            }
        }//
  
    } else {
        //enable the timer interrupt that checks for the accel interrupt flag
        if (!T2CONbits.ON) {
            T2CONbits.ON=1; //enable timer2+3 (for 32-bit)
        } 
        //the timer interrupt always stays enabled, but just in case enable again:
        //IFS0bits.T3IF=0; //clear interrupt flag
        //IEC0bits.T3IE=1; //enable Timer 2+3 interrupt- doesn't enable the timer
    } //if (!NumEnabledAccelerometers) {
//#endif     
return(1);
} //uint8_t ConfigureAccelerometers(uint16_t mask,uint32_t flags,uint16_t Threshold)

uint8_t Clear_Accelerometer_Interrupt(uint8_t num) {

    uint8_t I2CData[10],ReturnByte,result;

    //I2CData[0] = Accel[num].I2CAddress;
#if USE_FXOS8700CQ
    
    if (Accel[num].flags & ACCEL_STATUS_INTERRUPT_RELATIVE_ACCELERATION) {
        I2CData[0] = FXOS8700CQ_INT_SOURCE; //reg address
    } else {
        I2CData[0] = FXOS8700CQ_FF_MT_SRC; //reg address
    }

    //returns read byte
    //return (ReadI2CData(I2CData, 2, Accel[num].I2CBus));
    Accel[num].I2CBufferHandle=DRV_I2C_TransmitThenReceive(Accel[num].handleI2C, 
                                                    Accel[num].I2CAddress,
                                                    I2CData, //byte to send: register addr
                                                    1,  
                                                    &ReturnByte,
                                                    1,
                                                    NULL); 
    //DelayMs(1);
    DelayUs(500);
#endif //USE_FXOS8700CQ
    //wait for read to finish
    //result=WaitForI2C(Accel[num].handleI2C,Accel[num].I2CBufferHandle);    
    //if (result==I2C_INST_COMPLETE) {
        return(1);
    //} else {
    //    SYS_CONSOLE_PRINT("Err:%d clr A%d int\n\r",result,num);
   // }
    
    //return(0); 
    

} //uint8_t Clear_Accelerometer_Interrupt(uint8_t num) {


//Currently accelerometer interrupts are processed and a UDP packet
//with their X,Y data sent one at a time as they arrive
//uint8_t Process_Accelerometer_Interrupt(uint8_t AccelNum) {
uint8_t Process_Accelerometer_Interrupt(void) 
{

    uint8_t I2CData[20];
    //uint8_t GotIntData;
    uint8_t ReturnValue,i,RegAddr,result;
    uint16_t ReturnSampleX;
    uint16_t ReturnSampleY;
    uint16_t ReturnSampleZ;

#if USE_FXOS8700CQ
    //timer calls this function every time if any interrupt is enabled

    //SYS_CONSOLE_PRINT("ProcInt\r\n");
    
    AccelTimerSendLen=5;
    //for(i=0;i<NumActiveAccelerometers;i++) {
    for(i=0;i<NumAccelerometers;i++) {
        //AccelNum=ActiveAccel[i];
        if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
            if (Accel[i].flags&ACCEL_STATUS_GOT_INTERRUPT) {

               //according to app note AN4083 the accel int flag should be cleared first
               //and check the accel int source register to see that data is ready

                //clear accel interrupt by reading the status register
                //no need to do this now because ELE bit is not set (data is not latched at interrupt)
                //note: in ELE=0 mode, the INT1 bit only goes low at the threshold transition
                //(from below to above- not from above to below)
                //so only 1 interrupt is recognized when the accel crosses the threshold
                //even if the acceleration continues to grow
                //so restoring balance would require polling back to balance
                //but multiple interrupts are wanted ELE must = 1
                //it's not enough to just clear the interrupt
                //with ELE=1 mode I don't get the positive X or Y interrupt
                Accel[i].flags&=~ACCEL_STATUS_GOT_INTERRUPT; //clear interrupt flag
                ReturnValue=Clear_Accelerometer_Interrupt(i);  //w/o ELE set ReturnValue==0                
                if (ReturnValue==0) {
                    //read failed- reset accelerometer
                    //Reset_Accelerometer(i);
                    //Activate_Accelerometer(i);
                    //Enable_Accelerometer_Interrupt(i);
                    //return(0);
                } else {

                //in theory this interrupt could come from some other source
                //or the data may not be ready for some reason

    /*
                //no need to clear the interrupt if the data is not latched
                GotIntData=0;
                if (Accel[i].flags&ACCEL_FLAGS_RELATIVE_ACCELERATION) {
                    if (ReturnValue&(FXOS8700CQ_CTRL_REG5_INT_CFG_A_VECM|FXOS8700CQ_CTRL_REG5_INT_CFG_DRDY)) {
                        GotIntData=1;
                    }
                } else {
                    if (ReturnValue&(FXOS8700CQ_CTRL_REG5_INT_CFG_FF_MT|FXOS8700CQ_CTRL_REG5_INT_CFG_DRDY)) {
                        GotIntData=1;
                    }
                }
     */
    //            if (GotIntData) {
                    //I2CData[0]=Accel[i].I2CAddress;
                    //I2CData[0] = FXOS8700CQ_OUT_X_MSB;  //start reg address X=1
                    RegAddr = FXOS8700CQ_OUT_X_MSB;  //start reg address X=1

                    //ReturnValue=ReadMultipleI2CData(I2CData,2,4,Accel[i].I2CBus); //write 2 bytes read 4 bytes
//                    ReturnValue=ReadMultipleI2CData(I2CData,2,13,Accel[i].I2CBus); //write 2 bytes read 6 bytes
                    //write the client address, the reg address
                    //then read 12 bytes
                    Accel[i].I2CBufferHandle=DRV_I2C_TransmitThenReceive(Accel[i].handleI2C, 
                                                            Accel[i].I2CAddress,
                                                            &RegAddr, //byte to send: register addr
                                                            1,  
                                                            I2CData,
                                                            12,
                                                            NULL); 
                    //DelayMs(1); //probably try delay microseconds
                    //DelayUs(400); //
                    //at 400khz, each bit is 2.5us
                    
                    //wait for read to finish
                    result=WaitForI2C(Accel[i].handleI2C,Accel[i].I2CBufferHandle);    
                    //if (Accel[i].I2CBufferHandle == NULL) {
                    if (result==I2C_INST_COMPLETE) {
                    //if (ReadMultipleI2CData(I2CData,2,13,Accel[i].I2CBus)) {
                        //in the datasheet example the status is never checked
                        //perhaps just reading the reg is enough?
                        //I decided to just check above to remove any doubt
                   

                        //Accel Data
                        //[0]=MSB 7:0 are 8 MSB [1]=LSB 7:2 are 6 LSB,
                        //ReturnSampleX=((I2CData[0]<<0x8)| I2CData[1])>>0x2; //14-bit data
                        //ReturnSampleY=((I2CData[2]<<0x8)| I2CData[3])>>0x2; //14-bit data
                        //ReturnSampleZ=((I2CData[4]<<0x8)| I2CData[5])>>0x2; //14-bit data
                        //send UDP packet with x,y,z values
                //        memcpy(AccelSend,&Accel[i].ReturnIP,4); //copy IP to return instruction
                 //       AccelSend[4]=ROBOT_ACCELMAGTOUCH_START_ACCELEROMETER_INTERRUPT; //add int instruction
                        AccelTimerSendLen=5;
                        //add the accel # to the return UDP packet
                        AccelTimerSend[AccelTimerSendLen]=i;

                        //add the samples to the UDP packet
                        memcpy(AccelTimerSend+AccelTimerSendLen+1,&I2CData[0],12);
                        //add the x,y,z values to the return UDP packet
                        //memcpy(AccelSend+AccelSendLen+1,&ReturnSampleX,2); //little endian
                        //memcpy(AccelSend+AccelSendLen+3,&ReturnSampleY,2); //little endian
                        //memcpy(AccelSend+AccelSendLen+5,&ReturnSampleZ,2); //little endian
                        //AccelSendLen+=7;

                        //Magnetic Data
                        //[0]=MSB 7:0 are 8 MSB [1]=LSB 7:0 are 8 LSB,
                        //ReturnSampleX=(I2CData[6]<<0x8)|I2CData[7]; //16-bit data
                        //ReturnSampleY=(I2CData[8]<<0x8)|I2CData[9]; //16-bit data
                        //ReturnSampleZ=(I2CData[10]<<0x8)|I2CData[11]; //16-bit data

                        //add the x,y,z values to the return UDP packet
                        //memcpy(AccelSend+AccelSendLen+7,&I2CData[6],6);
                        //memcpy(AccelSend+AccelSendLen+7,&ReturnSampleX,2); //little endian
                        //memcpy(AccelSend+AccelSendLen+9,&ReturnSampleY,2); //little endian
                        //memcpy(AccelSend+AccelSendLen+11,&ReturnSampleZ,2); //little endian
                        AccelTimerSendLen+=13; //1 accel num + 12 sample bytes

                        //Accel[i].flags&=~ACCEL_STATUS_GOT_INTERRUPT; //clear interrupt flag

                        //while (UDPIsTxPutReady(UDPSendSock,AccelSendLen)<AccelSendLen) {};
                        //UDPPutArray(UDPSendSock,(uint8_t *)AccelSend,AccelSendLen);  //little endian
                        //UDPFlush(UDPSendSock); //send the packet
                        //send the UDP packet
                        while (TCPIP_UDP_PutIsReady(appData.socket)<AccelTimerSendLen) {};
                        TCPIP_UDP_ArrayPut(appData.socket,AccelTimerSend,AccelTimerSendLen);  //little endian       
                        TCPIP_UDP_Flush(appData.socket); //send the packet                                                          
                    } else { //if (result==I2C_INST_COMPLETE) {
                        //read failed- reset accelerometer
                        //if (result==I2C_INST_ERROR) {
                        SYS_CONSOLE_PRINT("Accel %d process interrupt error (%d), disabling.\n\r",i,result);
                        //probably an accel went offline
                        //just set to NOT ENABLED
                        //and if user tries to get a new sample
                        //it will get reset
                        //otherwise - perhaps 
                        //trying to power on once every second might be
                        //an idea
                        Accel[i].flags&=~ACCEL_STATUS_ENABLED;
                        //ReturnValue=1; //exit while
                    } //if (result==I2C_INST_COMPLETE) {
                } //if (ReturnValue==0) {
            } //if (Accel[i].flags&ACCEL_STATUS_GOT_INTERRUPT) {
        } //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
    } //for(i=0;i<NumActiveAccelerometers;i++) {


#endif //USE_FXOS8700CQ
    
    return(1);
} //uint8_t Process_Accelerometer_Interrupt(void) {


//Get_Accelerometer_Samples
//This function is called by the (10ms) timer 
//to initiate: 1) building the return UDP packet)
//and 2) sending the first sample I2C transmit request
//was:
//builds and sends a return UDP packet with all the active
//accelerometer X,Y 12-bit samples in it
//flag can = ACCEL_STATUS_POLLING or ACCEL_STATUS_SINGLE_SAMPLE
//uint8_t Get_Accelerometer_Samples(uint32_t flag) {
uint8_t Get_Accelerometer_Samples(void) {
    uint8_t i,RegAddr;
    uint8_t ReturnValue,result;
    //uint32_t TempSendLen;
    
    //SYS_CONSOLE_PRINT("S\n\r");
    
//#if 0 
    //TxSent=0;
    AccelTimerSendLen=5;
#if USE_FXOS8700CQ    
    RegAddr = FXOS8700CQ_STATUS;  //start reg address, STATUS
#endif
#if USE_MPU6050
    RegAddr= MPU6050_ACCEL_XOUT_H;  //start reg address for 14-byte bulk read
    //RegAddr= MPU6050_FIFO_R_W; //read from the FIFO
#endif     
    
    
    //determine which accel will be the last sample, in order for the 
    //I2C callback function to quickly and easily know to send the 
    //UDP packet (at each 10ms timer interrupt callback where there is
    //polling or an interrupt occurred)
    //with the change to bit-banged- polling stays set, but accel is not enabled
    //so added ACCEL_STATUS_ENABLED check for accels 1 and 2
    appData.LastAccelSample=2; //presume full 3 samples
    if (!((Accel[2].flags&ACCEL_STATUS_ENABLED) &&
           (Accel[2].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_CLEAR_INTERRUPT)))) {
            //accel[2] is not getting a sample]
        if ((Accel[1].flags&ACCEL_STATUS_ENABLED) && 
            (Accel[1].flags&(ACCEL_STATUS_POLLING|ACCEL_STATUS_CLEAR_INTERRUPT))) {
            appData.LastAccelSample=1;
        } else {
            appData.LastAccelSample=0;
        } // if (Accel[1]
    }// if (!(Accel[2]

    //freeze all interrupts here because an external interrupt
    //could occur while the for loop is running, and throw off the 
    //callback logic.
    for(i=0;i<NumAccelerometers;i++) {
        if (Accel[i].flags&ACCEL_STATUS_CLEAR_INTERRUPT) { 
            Accel[i].flags&=~ACCEL_STATUS_CLEAR_INTERRUPT;
            Accel[i].flags|=ACCEL_STATUS_GOT_INTERRUPT;
        } //if (Accel[i].flags
        
    } //for(i=0

    //TempSendLen=5;
    //while(!TxSent || i>NumAccelerometers) {
    //for(i=0;i<NumActiveAccelerometers;i++) {
    for(i=0;i<NumAccelerometers;i++) {
        //note that currently ACCEL_STATUS_GOT_INTERRUPT 
        //can be set, because external interrupts are always enabled
        if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
            if ((Accel[i].flags&ACCEL_STATUS_POLLING) || 
               ((Accel[i].flags&ACCEL_STATUS_INTERRUPT)&&(Accel[i].flags&ACCEL_STATUS_GOT_INTERRUPT))) { 
                //the app note AN4083 indicates that we should wait for the ZYXDR status bit to be set
                //but the FXOS8700 datasheet shows just reading all 13 bytes in one read
                //write the client address, the reg address
                //then read 13 bytes                
                //if (PLIB_I2C_BusIsIdle(Accel[i].I2CBus)) {
                //SYS_CONSOLE_PRINT("tx\r\n");
                
                //SYS_CONSOLE_PRINT("A%d LS %d\r\n",i,appData.LastAccelSample);
                //SYS_CONSOLE_PRINT("%x\r\n",Accel[i].flags);

                //clear processing flag
                //Accel[i].flags&=~ACCEL_STATUS_PROCESSING;

                Accel[i].I2CBufferHandle=DRV_I2C_TransmitThenReceive(Accel[i].handleI2C, 
                                                        Accel[i].I2CAddress,
                                                        &RegAddr, //byte to send: register addr
                                                        1,  
                                                        Accel[i].Buffer,//AccelTimerSend+TempSendLen,//AccelTimerSend+AccelTimerSendLen,//I2CData,
#if USE_FXOS8700CQ
                                                        13,
#endif                         
#if USE_MPU6050
                                                        14,
#endif                         
                                                        NULL); 
                //is is critical to only have one sample
                //going at a time. A single 14-byte read transaction 
                //completes is under 2ms, and there is currently 
                //only a 10ms window so 2ms is a good choice and works well.
                DelayMs(2); 
                //DelayUs(500); //delay500uS
                //} //if (PLIB_I2C_BusIsIdle(Accel[i].I2CBus)) {
            } //if (Accel[i].flags&ACCEL_STATUS_POLLING) { //skip any set to interrupt
        } //if (Accel[i].flags&ACCEL_STATUS_ENABLED) {
    } //for i
    //    i++; //if this accel was not enabled or is polling go to next accel 
    //} //while (!TxSent)

#if 0 
    if (AccelTimerSendLen>5) {
        //send the UDP packet
        while (TCPIP_UDP_PutIsReady(appData.socket)<AccelTimerSendLen) {};
        TCPIP_UDP_ArrayPut(appData.socket,AccelTimerSend,AccelTimerSendLen);  //little endian       
        TCPIP_UDP_Flush(appData.socket); //send the packet                                        
    } //if (AccelSendLen>5) {
#endif     
    
    return(1);
} //uint8_t Get_Accelerometer_Samples(void) {


uint8_t SendTimerUDPPacket(void) {

    if (AccelTimerSendLen>5) {
        //send the UDP packet
        while (TCPIP_UDP_PutIsReady(appData.socket)<AccelTimerSendLen) {};
        TCPIP_UDP_ArrayPut(appData.socket,AccelTimerSend,AccelTimerSendLen);  //little endian       
        TCPIP_UDP_Flush(appData.socket); //send the packet                                        
    } //if (AccelTimerSendLen>5) {
    return(1);
}

