//#include <plib.h>
#include "i2c_master.h"
#include "app.h" 

//#include "system_services.h"

//#define SCCB_DELAY_US 10
//#define SCCB_WRITE_ADDR  0x42
//#define SCCB_READ_ADDR   0x43
//#define OV7670_I2C_BUS              I2C3
//#define I2C_BUS3              I2C3
#define I2C_TIMEOUT 1000 //100 millisecond
uint32_t  tI2CTimeout;
#define TRUE 1
#define FALSE 0


/*******************************************************************************
  Function:
    uint8_t StartTransfer( uint8_t restart, int I2CBus )

  Summary:
    Starts (or restarts) a transfer to/from the EEPROM.

  Description:
    This routine starts (or restarts) a transfer to/from the EEPROM, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition
    I2CBus - which I2C Bus

  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/

uint8_t StartTransfer(uint8_t restart,int I2CBus)
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(I2CBus);
    }
    else
    {
     
   /*
        // Wait for the bus to be idle, then start the transfer
        //SYS_TICK_ResolutionGet() returns the number of system ticks per second
        tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
        while(!I2CBusIsIdle(I2CBus)) {
            if (SYS_TICK_Get()>=tI2CTimeout) {
                return FALSE; //timed out
            }
        } //while
*/
        //if (PLIB_I2C_MasterStart(I2CBUS) != )
        if(I2CStart(I2CBus) != I2C_SUCCESS)        
        {
//            DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
    do
    {
        status = I2CGetStatus(I2CBus);
        if (ReadCoreTimer()>=tI2CTimeout) {
            return FALSE; //timed out
        }
    } while (!(status & I2C_START));

    return TRUE;
}


/*******************************************************************************
  Function:
    uint8_t TransmitOneByte( uint8_t data, int I2CBus )

  Summary:
    This transmits one byte to the EEPROM.

  Description:
    This transmits one byte to the EEPROM, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit
    I2CBus - which I2C bus

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/

uint8_t TransmitOneByte(uint8_t data,int I2CBus)
{
    // Wait for the transmitter to be ready
    tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
    while(!I2CTransmitterIsReady(I2CBus)) {
        if (SYS_TICK_Get()>=tI2CTimeout) {
            return FALSE; //timed out
        }
    } //while

    // Transmit the byte
    if(I2CSendByte(I2CBus, data) == I2C_MASTER_BUS_COLLISION)
    {
  //      DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
    while(!I2CTransmissionHasCompleted(I2CBus)) {
        if (SYS_TICK_Get()>=tI2CTimeout) {
            return FALSE; //timed out
        }
    } //while

    return TRUE;
}


/*******************************************************************************
  Function:
    uint8_t StopTransfer( void )

  Summary:
    Stops a transfer to/from the EEPROM.

  Description:
    This routine Stops a transfer to/from the EEPROM, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
 int I2CBus - which I2C bus

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/

uint8_t StopTransfer(int I2CBus)
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(I2CBus);

    // Wait for the signal to complete
    tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
    do
    {
        status = I2CGetStatus(I2CBus);
        if (SYS_TICK_Get()>=tI2CTimeout) {
            return(0); //timed out
        }
    } while ( !(status & I2C_STOP) );
    return(1);
}


uint8_t WriteI2CData(uint8_t *I2CData,int NumByte,int I2CBus)
{
    int                 Index;
    uint8_t                Acknowledged;
    uint8_t                Success = TRUE;
    uint8_t               i2cbyte;
    uint8_t               OrigSlaveAddress;
    I2C_7_BIT_ADDRESS   SlaveAddress;

    //note: possibly the I2CData should be copied
    //into a new array so we don't overwrite the original data
    //but that would put a limit on the NumByte to whatever we define
    //in this function, so currently I just rewrite the OrigSlaveAddress
    //before returning

    OrigSlaveAddress=I2CData[0];
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, OrigSlaveAddress, I2C_WRITE);
    I2CData[0]=SlaveAddress.byte;
//    I2CData[0] = SlaveAddress.byte;
 //   I2CData[1] = 0x05;              // EEPROM location to program (high address byte)
 //   I2CData[2] = 0x40;              // EEPROM location to program (low address byte)
 //   I2CData[3] = 0xAA;              // Data to write
 //   DataSz = 4;

    // Start the transfer to write data to the EEPROM
    if( !StartTransfer(FALSE,I2CBus) )
    {
//        while(1);
        I2CData[0]=OrigSlaveAddress;
        return(0);
    }

    // Transmit all data
    Index = 0;

    //note: transmitOneByte has a timeout
    while( Success && (Index < NumByte) )
    {
        // Transmit a byte
        if (TransmitOneByte(I2CData[Index],I2CBus))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2CBus))
            {
//                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }

    } //while

    // End the transfer (hang here if an error occured)-update: now there is a timeout in StopTransfer
    if (!StopTransfer(I2CBus)) {
        I2CData[0]=OrigSlaveAddress;
        return(0);
    }

    if(!Success)
    {
//        while(1);
        I2CData[0]=OrigSlaveAddress;
        return(0);
    }


    // Wait for EEPROM to complete write process, by polling the ack status.
    Acknowledged = FALSE;
    //do
    //{
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE,I2CBus) )
        {
            //while(1);
            I2CData[0]=OrigSlaveAddress;
            return(0);
        }

        // Transmit just the EEPROM's address
        if (TransmitOneByte(SlaveAddress.byte,I2CBus))
        {
            // Check to see if the byte was acknowledged
            Acknowledged = I2CByteWasAcknowledged(I2CBus);
        }
        else
        {
            Success = FALSE;
        }

        // End the transfer (stop here if an error occured)
        if (!StopTransfer(I2CBus)) {
            I2CData[0]=OrigSlaveAddress;
            return(0);
        }
        if(!Success)
        {
            //while(1);
            I2CData[0]=OrigSlaveAddress;
            return(0);
        }

    //} while (Acknowledged != TRUE);

    //put back original address
    I2CData[0]=OrigSlaveAddress;
    return(Acknowledged);
}//WriteI2CData


//sends 2 bytes, reads a single byte, and returns the byte read
//NumByte is 2, the number of bytes sent: the i2c address, and register
uint8_t ReadI2CData(uint8_t *I2CData,int NumByte,int I2CBus)
{
    int                 Index;
//    uint8_t                Acknowledged;
    uint8_t                Success = TRUE;
    uint8_t               I2CByte;
    uint8_t               OrigSlaveAddress;
    I2C_7_BIT_ADDRESS   SlaveAddress;


    I2CByte=0;
    OrigSlaveAddress=I2CData[0];
    //convert the I2C address to the correct address for writing
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, OrigSlaveAddress, I2C_WRITE);

    // Start the transfer to read the EEPROM.
    if( !StartTransfer(FALSE,I2CBus) )
    {
        //while(1);
        return(0);
    }

    // Address the I2C Device
    //sends I2C write address I2CData[0] and the register address I2CData[1]
    Index = 0;
    while( Success & (Index < NumByte) )
    {
        // Transmit a byte
        if (Index==0) {  //first byte is I2C address
            I2CByte=SlaveAddress.byte;
        } else {
            I2CByte=I2CData[Index]; //otherwise is register address
        }
        if (TransmitOneByte(I2CByte,I2CBus))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            Success = FALSE;
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2CBus))
        {
    //        DBPRINTF("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
        }
    }

    // Restart and send the I2C Device's internal address to switch to a read transfer
    if (Success)
    {
        // Send a Repeated Started condition
        if (!StartTransfer(TRUE,I2CBus))
        {
            //while(1);
            return(0);
        }

        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, OrigSlaveAddress, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte,I2CBus))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2CBus))
            {
  //              DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // Read the data from the desired address
    if(Success)
    {
        //PLIB_I2C_ReceiverOverflowHasOccurred(
        if(I2CReceiverEnable(I2CBus, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
//            DBPRINTF("Error: I2C Receive Overflow\n");
            Success = FALSE;
        }
        else
        {
            tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
            while(!I2CReceivedDataIsAvailable(I2CBus)) {
                if (SYS_TICK_Get()>=tI2CTimeout) {
                    return FALSE; //timed out
                }
            } //while
            I2CByte = I2CGetByte(I2CBus);
        }

    }

    // End the transfer (stop here if an error occured)
    StopTransfer(I2CBus);
    if(!Success)
    {
        //while(1);
        return(0);
    }

    return(I2CByte);
}//ReadI2CData


//read in multiple bytes
uint8_t ReadMultipleI2CData(uint8_t *I2CData,int NumByteToWrite,int NumByteToRead,int I2CBus)
{
    int                 Index;
//    uint8_t                Acknowledged;
    uint8_t                Success = TRUE;
    uint8_t               I2CByte;
    uint8_t               OrigSlaveAddress;
    I2C_7_BIT_ADDRESS   SlaveAddress;


    //I2CByte=0;
    OrigSlaveAddress=I2CData[0];
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress,OrigSlaveAddress,I2C_WRITE);
    //
    // Read the data back from the EEPROM.
    //

    // Initialize the data buffer
//    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, EEPROM_ADDRESS, I2C_WRITE);
//      I2CData[0] = SlaveAddress.byte;
//    I2CData[1] = 0x05;              // EEPROM location to read (high address byte)
//    I2CData[2] = 0x40;              // EEPROM location to read (low address byte)
 //   DataSz = 3;

    // Start the transfer to read the EEPROM.
    if( !StartTransfer(FALSE,I2CBus) )
    {
        //while(1);
        return(0);
    }

    // Address the I2C Device
    //sends I2C write address I2CData[0] and the register address I2CData[1]
    Index = 0;
    while( Success & (Index < NumByteToWrite) )
    {
        // Transmit a byte
        if (Index==0) {  //first byte is I2C address
            I2CByte=SlaveAddress.byte;
        } else {
            I2CByte=I2CData[Index]; //otherwise is register address
        }
        if (TransmitOneByte(I2CByte,I2CBus))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            Success = FALSE;
            return(0);
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(I2CBus))
        {
    //        DBPRINTF("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
            return(0);
        }
    }

    // Restart and send the I2C Device's internal address to switch to a read transfer
    if (Success)
    {
        // Send a Repeated Started condition
        if (!StartTransfer(TRUE,I2CBus))
        {
            //while(1);
            return(0);
        }

        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, OrigSlaveAddress, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte,I2CBus))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2CBus))
            {
  //              DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
                return(0);
            }
        }
        else
        {
            Success = FALSE;
            return(0);
        }
    }//    if (Success)

    Index = 0;
    while(Index<NumByteToRead) {
        // Read the data from the desired address
        if(Success)
        {
            if(I2CReceiverEnable(I2CBus,TRUE) == I2C_RECEIVE_OVERFLOW)
            {
    //            DBPRINTF("Error: I2C Receive Overflow\n");
                Success = FALSE;
                return FALSE;
            }
            else
            {
                tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
                while(!I2CReceivedDataIsAvailable(I2CBus)) {
                    if (SYS_TICK_Get()>=tI2CTimeout) {
                        return FALSE; //timed out
                    }
                } //while
                if (Index<NumByteToRead-1) {
                    //send ack for the next byte
                    //I2C3CONbits.ACKDT=0;
                    //I2C3CONbits.ACKEN=1;
                    I2CAcknowledgeByte(I2CBus,TRUE);
                    
                } else {
                    //send nak for done
                    //I2C3CONbits.ACKDT=1;
                    //I2C3CONbits.ACKEN=1;
                    I2CAcknowledgeByte(I2CBus,FALSE);
                }
                I2CData[Index] = I2CGetByte(I2CBus);
                //while(I2C3CON & 0x1F);
                tI2CTimeout=ReadCoreTimer()+I2C_TIMEOUT*ms_SCALE;
                while(!I2CAcknowledgeHasCompleted(I2CBus)) {
                    if (SYS_TICK_Get()>=tI2CTimeout) {
                        return FALSE; //timed out
                    }
                }  //while
                Index++;

                
            }//if(I2CReceiverEnable(I2CBus, TRUE) == I2C_RECEIVE_OVERFLOW)

        }  //if Success
    } //    while(Index<NumByteToRead) {


    //while (!I2CBusIsIdle(I2CBus)); //wait for bus to become idle
    // End the transfer (stop here if an error occured)
    StopTransfer(I2CBus);
    if(!Success)
    {
        //while(1);
        return(0);
    }

    return(Index);
}//ReadMultipleI2CData
