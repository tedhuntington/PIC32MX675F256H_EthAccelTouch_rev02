/* 
 * File:   FXOS8700CQ.h
 * Author: ted
 *
 * Created on December 12, 2015, 6:53 PM
 */

#ifndef FXOS8700CQ_H
#define	FXOS8700CQ_H

#ifdef	__cplusplus
extern "C" {
#endif


//FXOS8700CQ has accel ranges +-2g(used in this app), +-4g and +-8g
    // and magnetic range of +-1200 uT
    //see appnote: http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
    //for more explanation of determining orientation using accel and magnetic data
#define FXOS8700CQ_ADDRESS    0x3c//0x1e

#define FXOS8700CQ_STATUS  0x0 //Data Status
#define FXOS8700CQ_STATUS_ZYXDR  0x08 //XYZ axis new data is ready
#define FXOS8700CQ_OUT_X_MSB 0x01 //12-bit X Accel Data most significant byte
#define FXOS8700CQ_OUT_Y_MSB 0x03 //12-bit Y Accel Data most significant byte
#define FXOS8700CQ_OUT_Z_MSB 0x05 //12-bit Z Accel Data most significant byte
//#define FXOS8700CQ_STATUS_ZYXDR  0x8 //Data Status ZYX data ready
#define FXOS8700CQ_INT_SOURCE  0x0c //used to clear a_vecm interrupt if not set to auto clear
#define FXOS8700CQ_WHOAMI  0x0d //returns C7
#define FXOS8700CQ_XYZ_DATA_CFG  0x0e //Acceleration dynamic range and filter enable settings
#define FXOS8700CQ_HP_FILTER_CUTOFF 0x0f //high pass filter cutoff frequency
#define FXOS8700CQ_FF_MT_CFG  0x15 //Freefall/Motion Config
#define FXOS8700CQ_FF_MT_CFG_ELE 0x80
#define FXOS8700CQ_FF_MT_CFG_OAE 0x40
#define FXOS8700CQ_FF_MT_CFG_ZEFE 0x20
#define FXOS8700CQ_FF_MT_CFG_YEFE 0x10
#define FXOS8700CQ_FF_MT_CFG_XEFE 0x08
#define FXOS8700CQ_FF_MT_SRC  0x16 //Freefall/Motion Source
#define FXOS8700CQ_FF_MT_SRC_EA  0x80
#define FXOS8700CQ_FF_MT_SRC_ZHE  0x20
#define FXOS8700CQ_FF_MT_SRC_ZHP  0x10
#define FXOS8700CQ_FF_MT_SRC_YHE  0x08
#define FXOS8700CQ_FF_MT_SRC_YHP  0x04
#define FXOS8700CQ_FF_MT_SRC_XHE  0x02
#define FXOS8700CQ_FF_MT_SRC_XHP  0x01
#define FXOS8700CQ_FF_MT_THS  0x17 //Freefall/Motion Threshold- note can be set individually too with reg 0x73-0x78
#define FXOS8700CQ_FF_MT_COUNT  0x18 //Freefall/Motion Debounce
#define FXOS8700CQ_M_DR_STATUS  0x32 //Magnetic Data Ready Status
#define FXOS8700CQ_M_DR_STATUS_ZYXDR  0x08 //XYZ axis new magnetic data is ready
#define FXOS8700CQ_M_OUT_X_MSB 0x33 //12-bit X Magnetic Data most significant byte
#define FXOS8700CQ_M_OUT_Y_MSB 0x35 //12-bit Y Magnetic Data most significant byte
#define FXOS8700CQ_M_OUT_Z_MSB 0x37 //12-bit Z Magnetic Data most significant byte
#define FXOS8700CQ_M_OFF_X_MSB 0x3f //15-bit hard iron offset
#define FXOS8700CQ_M_OFF_X_LSB 0x40
#define FXOS8700CQ_M_OFF_Y_MSB 0x41 
#define FXOS8700CQ_M_OFF_Y_LSB 0x42
#define FXOS8700CQ_M_OFF_Z_MSB 0x43 
#define FXOS8700CQ_M_OFF_Z_LSB 0x44
#define FXOS8700CQ_A_VECM_CFG  0x5F //Acceleration vector magnitude configuration
#define FXOS8700CQ_A_VECM_CFG_ELE 0x40
#define FXOS8700CQ_A_VECM_CFG_INITM 0x20
#define FXOS8700CQ_A_VECM_CFG_UPDM 0x10
#define FXOS8700CQ_A_VECM_CFG_EN 0x08
#define FXOS8700CQ_A_VECM_THS_MSB  0x60 //Acceleration vector-magnitude threshold MSB
#define FXOS8700CQ_A_VECM_THS_LSB  0x61 //Acceleration vector-magnitude threshold LSB
#define FXOS8700CQ_A_VECM_CNT        0x62 //Acceleration vector-magnitude debounce count
#define FXOS8700CQ_A_VECM_INITX_MSB  0x63 //Acceleration vector-magnitude x-axis reference value MSB
#define FXOS8700CQ_A_VECM_INITX_LSB  0x64 //Acceleration vector-magnitude x-axis reference value LSB
#define FXOS8700CQ_A_VECM_INITY_MSB  0x65 //Acceleration vector-magnitude y-axis reference value MSB
#define FXOS8700CQ_A_VECM_INITY_LSB  0x66 //Acceleration vector-magnitude y-axis reference value LSB
#define FXOS8700CQ_A_VECM_INITZ_MSB  0x67 //Acceleration vector-magnitude z-axis reference value MSB
#define FXOS8700CQ_A_VECM_INITZ_LSB  0x68 //Acceleration vector-magnitude z-axis reference value LSB
#define FXOS8700CQ_CTRL_REG1   0x2a //Data rates and modes setting
#define FXOS8700CQ_CTRL_REG1_DR_2  0x20 //output data rate (ODR) bit2
#define FXOS8700CQ_CTRL_REG1_DR_1  0x10 //output data rate (ODR) bit1
#define FXOS8700CQ_CTRL_REG1_DR_0  0x08 //output data rate (ODR) bit0
#define FXOS8700CQ_CTRL_REG1_LNOISE 0x04 //reduced noise and full-scale range mode
#define FXOS8700CQ_CTRL_REG1_ACTIVE  0x01 //Standby/Active mode
#define FXOS8700CQ_CTRL_REG2   0x2b //Selt test, Reset, accelerometer OSR, and Sleep mode settings
#define FXOS8700CQ_CTRL_REG2_RST 0x40
#define FXOS8700CQ_CTRL_REG4   0x2d //Interrupt Enable Map
#define FXOS8700CQ_CTRL_REG4_INT_EN_FF_MT 0x04 //Frefall/Motion interrupt
#define FXOS8700CQ_CTRL_REG4_INT_EN_A_VECM 0x02 //Acceleration vector-magnitude interrupt
#define FXOS8700CQ_CTRL_REG5   0x2e //Interrupt Configuration
#define FXOS8700CQ_CTRL_REG5_INT_CFG_FF_MT   0x04 //Freefall/Motion int1 or int2
#define FXOS8700CQ_CTRL_REG5_INT_CFG_A_VECM   0x02 //Freefall/Motion int1 or int2
#define FXOS8700CQ_CTRL_REG5_INT_CFG_DRDY   0x01 //interrupt data is ready
#define FXOS8700CQ_M_CTRL_REG1 0x5B //magnetometer control register 1
#define FXOS8700CQ_M_CTRL_REG2 0x5C //magnetometer control register 2
#define FXOS8700CQ_M_CTRL_REG3 0x5D //magnetometer control register 3




#ifdef	__cplusplus
}
#endif

#endif	/* FXOS8700CQ_H */

