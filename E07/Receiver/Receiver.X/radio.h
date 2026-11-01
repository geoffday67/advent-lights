/*
 * File:   radio.h
 * Author: geoff
 *
 * Created on March 31, 2025, 8:53 PM
 */

#ifndef RADIO_H
#define	RADIO_H

#define CS_PORT C
#define CS_BIT 0x04

#define CLOCK_PORT C
#define CLOCK_BIT 0x02

#define MOSI_PORT C
#define MOSI_BIT 0x01

#define MISO_PORT A
#define MISO_BIT 0x04

#ifdef	__cplusplus
extern "C" {
#endif

    void initialise();
    void initialiseAfterPowerDown();
    void enableRx();
    void enableWor();
    void reset();
    int receiveValid(void *ppacket);
    uint8_t getStatus();
    void powerDown();
    uint8_t radio_version();

#ifdef	__cplusplus
    }
#endif

#endif	/* RADIO_H */
