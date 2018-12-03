//touchsensors.c - Touch Sensors functions

#include "touchsensors.h"
#include "app.h"

extern EthAccelStatus EAStatus;
extern uint8_t NumTouchSensors,NumActiveTouchSensors;
extern TouchSensorStatus TouchSensor[MAX_NUM_TOUCH_SENSORS]; //status of each touch sensor
extern uint8_t ActiveTouchSensor[MAX_NUM_TOUCH_SENSORS]; //list of all active touch sensors in order, for a quick reference
extern uint8_t TouchSensorSend[TOUCH_SENSOR_SEND_SIZE];  //touch sensor packet data to send back to requester

uint8_t Initialize_TouchSensors(void)
{
    uint8_t i;

    //clear the packet we send to whoever requests touch sensor data
    memset(TouchSensorSend,0,sizeof(TOUCH_SENSOR_SEND_SIZE));

    
    //one tricky change from pic32mx to pic32mz is that
    //AN# no longer maps to RB#
    //instead:
    //RB: 0,1,2,3,4, 5, 6, 7, 8, 9,10,11,12,13,14 = 
    //AN: 0,1,2,3,4,45,46,47,48,49, 5, 6, 7, 8, 9,10    
    //in addition only one or the other can be used/scanned at one time for:
    //5/45,6/46,7/47,8/48,9/49 
    //so the next revision will probably change to:
    //1) alternate ethernet configuration
    //2) 15 simulataneous AN channels
    //3) 3 I2C
    //4) 2 USART (Serial port monitor and GPS)- could GPS just use 3 or 4 USART pins?
    
    //as a result, until rev5, only AN0-9 can be used) currently.
    //and they map this way:
    //(TS0=n/a),TS1=AN4,TS2=AN3,TS3=AN2,TS4=AN1,TS5=AN0,(TS6-TS9=n/a),
    //TS10=AN5,TS11=AN6,TS12=AN7,TS13=AN8,TS14=AN9   (5 are n/a)
    //and since I only populate TS0-8, it means only T1,2,3,4,5 are available
    //on the unpopulated side T10,T11,T12,T13,T14 are available
    
    //Touch Sensors
    NumTouchSensors=15;
    NumActiveTouchSensors=0;
    TRISB=0xffff;//portb is all inputs for touch sensor
    AD1PCFG=0x0; //set PORTB pins (also digital pins) to all analog, (all AN# pins are analog by default)
    //pic32mz ANSELB=0xffff; //set PORTB pins (also digital pins) to all analog, (all AN# pins are analog by default)

    ODCB=0; //no pins on port B are open drain
    
    //touch sensors:
    //0=b5,1=b4,2=b3,3=b2,4=b1,5=b0,6=b6...

    memset(TouchSensor,0,sizeof(TouchSensorStatus)*NumTouchSensors);
    TouchSensor[0].flags|=TOUCH_SENSOR_STATUS_ACTIVE;
    TouchSensor[0].SensorBit=0x20; //RB5
    TouchSensor[1].SensorBit=0x10; //RB4
    TouchSensor[2].SensorBit=0x8; //RB3
    TouchSensor[3].SensorBit=0x4; //RB2
    TouchSensor[4].SensorBit=0x2; //RB1
    TouchSensor[5].SensorBit=0x1; //RB0
    for(i=6;i<NumTouchSensors;i++) {
        TouchSensor[i].SensorBit=1<<i;
    }

   
    for(i=0;i<NumTouchSensors;i++) {
        //TouchSensor[i].flags|=TOUCH_SENSOR_STATUS_ACTIVE; //for testing
        TouchSensor[i].Threshold=DEFAULT_TOUCH_THRESHOLD;
        //set min and max voltage for touch sensor to calibrate itself
        //note .Max is already set to 0
        //pic32mx is 10bit, pic32mz is 12bit = 0 to fff (/4096)
//        TouchSensor[i].Min=0x3ff;  //Set Min to highest value possible (10-bit ADC)
        //Currently I am going with 0.4 to 2.1v as min and max        
        //but these values can be set by the user
        //and are autotuned as the touch sensor is used (basically
        //if a lower or higher voltage is found, the min and max are changed,
        //which then changes the percent pressed, and percent change range)
        //conversion multiplier is: 
        //10-bit ADC sample, 0x3ff=1023.0  3.3/1023 = 0.003225806
        //12-bit ADC sample, 0xfff=4095.0  3.3/4096 = 0.000805
        TouchSensor[i].Min=0x1f0;//(496)  //0.4v pic23mz
        TouchSensor[i].Max=0xa2e;//(2606) //2.1v pic32mz
        

    } //for i
    //need SensorBitMask?


    
    //initialize Analog to Digital module
     // ***see 16 motors for how to do this***
    //AD1PCFG&=0xbfff;    //make the RB14 pin analog
    //by setting its corresponding bit in the AD1PCFG register to 0
    //Set the analog pin as the Positive Input Select bits for Sample A Multiplexer Setting
    //AD1CHSbits.CH0SA=5;//mask which determines which ADC pin to read- not used for scanning
    //Set VREFL as the Negative Input Select bit for Sample A Multiplexer Setting
    AD1CHSbits.CH0NA=0;
    AD1CON1bits.FORM=0; //integer 16-bit format (10-bits)
    AD1CON1bits.SSRC=7; //(autoconvert) internal counter ends sampling and starts conversion
    //tph: Note that ASAM=1 is not mentioned in ADC Module Configuration
    //but is critical for getting a constant supply of samples
    AD1CON1bits.ASAM=1; //sampling begins immediately after last conversion
    AD1CON2bits.VCFG=0; //VREFH=VDD VREFL=VSS
    AD1CON2bits.CSCNA=1; //1=scan inputs is enabled, 0=do not scan inputs
    AD1CON2bits.SMPI=NumTouchSensors; //interrupt after this many samples, 0=interrupt for each sample
    AD1CON2bits.BUFM=0; //buffer configured as 1 16-bit uint16_t buffer
    AD1CON2bits.ALTS=0; //always use Sample A input multiplexer settings
    AD1CON3bits.ADRC=0; //clock derived from the Peripheral Bus Clock (PBCLK)
    //a TAD is an ADC clock cycle- one for each bit plus 2 more
    AD1CON3bits.SAMC=25;//sample time= 25TAD (12 TAD is typical for a conversion)
    AD1CON3bits.ADCS=0xff; //clock prescaler= 256 256TPB=TAD was 2 8TPB=TAD
    //can be every 100ms (and sends 10 samples in a second)
    //so we want to mix sample time and TAD clock to finish each sample every 100ms
    //we need a TAD every 5ms (1s/10=0.1s /20TAD= .005s)
    //PB=80mhz 12.5ns * 20 TAD= 250ns 100,000,000/250=PB must be every 400,000 ns (400 us)
    //12.5ns * 256= 3.2us * 20 = 6.4us 15,625 interrupts in a second
    //slowest A2D using PB can be is 6.4us x 31TAD = 198.4us
    //currently 6.4us x 25 TAD = 625 interrupts every 100ms

    AD1CON1bits.ON=1; //turn on ADC module - touch sensors are enabled by default
    //the result will be in ADC1BUF4 (RB4)
    //AD1 - ADC1 Convert Done interrupt
    //LastSample=0; //set the initial sample to compare with

    IFS1bits.AD1IF=0; //clear AD1 interrupt flag
    IPC6bits.AD1IP=4;  //set interrupt priority
    IPC6bits.AD1IS=2;  //set interrupt subpriority

    //SYS_CONSOLE_PRINT("%x\r\n",ADCSYSCFG0);
    
    return(1);
} //uint8_t Initialize_TouchSensors(void)



//activate or deactivate (Activate=0) individual touch sensors
//touch sensors may be activate or inactive by using a 32-bit mask
//if active=1 any 1s in the mask activate the sensor, 0s remain in the same state
//if active=0 any 1s in the mask inactivate the sensor, 0s remain in the same state
uint8_t SetActiveTouchSensors(uint32_t mask,int Activate)
{
    uint8_t i,j,NumLower;

    //reset number of touch sensors
    NumActiveTouchSensors=0;
    AD1CSSLbits.CSSL=0; //which analog inputs are scanned
    for(i=0;i<NumTouchSensors;i++) {
        
        if (mask&(1<<i)) {
            if (Activate) {
                TouchSensor[i].flags|=TOUCH_SENSOR_STATUS_ACTIVE;
                //because touch sensor # doesn't=PORTB pin#, they have to be remapped
                AD1CSSLbits.CSSL|=TouchSensor[i].SensorBit; //which AD channels to scan
                ActiveTouchSensor[NumActiveTouchSensors]=i;
                TouchSensor[i].flags&=~TOUCH_SENSOR_STATUS_GOT_INITIAL_SAMPLE;
                NumActiveTouchSensors++;
            } else {
                TouchSensor[i].flags&=TOUCH_SENSOR_STATUS_ACTIVE;
                AD1CSSLbits.CSSL&=~TouchSensor[i].SensorBit; //clear this AD channels from scanning
            }
        } else { //if (mask&(1<<i)) {
            //no mask, these sensors stay in the same state
            if (TouchSensor[i].flags&TOUCH_SENSOR_STATUS_ACTIVE) {
                AD1CSSLbits.CSSL|=TouchSensor[i].SensorBit; //which AD channels to scan
                ActiveTouchSensor[NumActiveTouchSensors]=i;
                NumActiveTouchSensors++;
            }
        } //if (mask&(1<<i)) {
        //note that the sample buffer number does not correspond to the
        //analog pin number- for example, if only AN6 is enabled
        //the first sample in the scan will be in ADC1BUF0 (not ADC1BUF6)
        //and so, if the next motor, motor[1] uses AN3, the ADC buffer for motor[0]
        //will change to ADC1BUF1- so for this reason all pins are sampled
        //and then the ADC1BUF corresponds to the pin number,
        //alternatively, earlier motors could be adjusted to the correct ADC1BUF address
    } //for i


    //go back through sensors and
    //determine which ADC1BUF sample the CS sample for each motor will be in
    //because samples are in order of AN# channel, so an3 before an8, etc.

    for(i=0;i<NumActiveTouchSensors;i++) {
        NumLower=0; //number of sensors higher than the current sensor
        for(j=0;j<NumActiveTouchSensors;j++) {
            if (i!=j && TouchSensor[ActiveTouchSensor[j]].SensorBit<TouchSensor[ActiveTouchSensor[i]].SensorBit) {
                NumLower++;
            }
        } //for j
        TouchSensor[ActiveTouchSensor[i]].ADCBuf=&ADC1BUF0+NumLower*4; //add 4 bytes (+0x10) for each motor below this motor
    } //for i


    if (!NumActiveTouchSensors) {
        //no active accelerometers clear poll and interrupt flags
        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_INTERRUPT;
        EAStatus.flags&=~ETHACCEL_STATUS_TOUCH_SENSOR_POLLING;
//        if (!NumActiveAccelerometers) {
        //if (!(EAStatus.flags&ETHACCEL_STATUS_ACCEL_POLLING) &&
        //    !(EAStatus.flags&ETHACCEL_STATUS_ACCEL_INTERRUPT)) {
            //no interrupt or polling on touch sensors or accels, so stop timer
            //no active accelerometers
 //           T2CONbits.ON=0; //disable timer2+3 (for 32-bit)
 //       } //if (!NumActiveAccelerometers) {
    } //if (!NumActiveTouchSensors) {


} //uint8_t SetActiveTouchSensors(uint32_t mask)


