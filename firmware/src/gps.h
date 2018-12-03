/* 
 * File:   gps.h
 * Author: tedh
 *
 * Created on November 9, 2016, 8:15 PM
 */

#ifndef GPS_H
#define	GPS_H

#include <stdint.h>  //for uint8_t

#ifdef	__cplusplus
extern "C" {
#endif


uint8_t SendSerial(uint8_t *serstr,uint32_t num);

#ifdef	__cplusplus
}
#endif

#endif	/* GPS_H */

