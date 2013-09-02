
#ifndef ANDROID_RFID_INTERFACE_H
#define ANDROID_RFID_INTERFACE_H
#include <hardware/hardware.h>

__BEGIN_DECLS

//#ifndef __RC522REG_H__
//#define __RC522REG_H__

/*! \name Register definitions of Page 0
 *  \ingroup reg
 *  Following all register defintions of the RC522 Page 0.
 */
/*@{*/
#define     JREG_RFU00           0x00      /*   Currently not used.                                     */
#define     JREG_COMMAND         0x01      /*!< Contains Command bits, PowerDown bit and bit to
                                                switch receiver off.                                    */
#define     JREG_COMMIEN         0x02      /*!< Contains Communication interrupt enable bits and
                                                bit for Interrupt inversion.                            */
#define     JREG_DIVIEN          0x03      /*!< Contains RfOn, RfOff, CRC and Mode Interrupt enable
                                                and bit to switch Interrupt pin to PushPull mode.       */
#define     JREG_COMMIRQ         0x04      /*!< Contains Communication interrupt request bits.          */
#define     JREG_DIVIRQ          0x05      /*!< Contains RfOn, RfOff, CRC and Mode Interrupt request.  */
#define     JREG_ERROR           0x06      /*!< Contains Protocol, Parity, CRC, Collision, Buffer
                                                overflow, Temperature and RF error flags.               */
#define     JREG_STATUS1         0x07      /*!< Contains status information about Lo- and HiAlert,
                                                RF-field on, Timer, Interrupt request and CRC status.   */
#define     JREG_STATUS2         0x08      /*!< Contains information about internal states (Modemstate),
                                                Mifare states and possibility to switch Temperature
                                                sensor off.                                             */
#define     JREG_FIFODATA        0x09      /*!< Gives access to FIFO. Writing to register increments the
                                                FIFO level (register 0x0A), reading decrements it.      */
#define     JREG_FIFOLEVEL       0x0A      /*!< Contains the actual level of the FIFO.                  */
#define     JREG_WATERLEVEL      0x0B      /*!< Contains the Waterlevel value for the FIFO              */
#define     JREG_CONTROL         0x0C      /*!< Contains information about last received bits and to
                                                Start and stop the Timer unit.                          */
#define     JREG_BITFRAMING      0x0D      /*!< Contains information of last bits to send, to align
                                                received bits in FIFO and activate sending in Transceive*/
#define     JREG_COLL            0x0E      /*!< Contains all necessary bits for Collission handling     */
#define     JREG_RFU0F           0x0F      /*   Currently not used.                                     */
/*@}*/

/*! \name Register definitions of Page 1
 *  \ingroup reg
 *  Following all register defintions of the RC522 Page 1.
 */
/*@{*/
#define     JREG_RFU10           0x10      /*   Currently not used.                                     */
#define     JREG_MODE            0x11      /*!< Contains bits for auto wait on Rf, to detect SYNC byte in
                                                NFC mode and MSB first for CRC calculation              */
#define     JREG_TXMODE          0x12      /*!< Contains Transmit Framing, Speed, CRC enable, bit for
                                                inverse mode and TXMix bit.                             */
#define     JREG_RXMODE          0x13      /*!< Contains Transmit Framing, Speed, CRC enable, bit for
                                                multiple receive and to filter errors.                  */
#define     JREG_TXCONTROL       0x14      /*!< Contains bits to activate and configure Tx1 and Tx2 and
                                                bit to activate 100% modulation.                        */
#define     JREG_TXASK           0x15      /* */
#define     JREG_TXSEL           0x16      /*!< Contains SigoutSel, DriverSel and LoadModSel bits.      */
#define     JREG_RXSEL           0x17      /*!< Contains UartSel and RxWait bits.                       */
#define     JREG_RXTRESHOLD      0x18      /*!< Contains MinLevel and CollLevel for detection.          */
#define     JREG_DEMOD           0x19      /*!< Contains bits for time constants, hysteresis and
                                                IQ demodulator settings.                                */
#define     JREG_RFU1A           0x1A      /* */
#define     JREG_RFU1B           0x1B      /* */
#define     JREG_MFTX            0x1C      /* */
#define     JREG_MFRX            0x1D      /* */
#define     JREG_TYPEB           0x1E      /*   Currently not used.                                     */
#define     JREG_SERIALSPEED     0x1F      /*!< Contains speed settings for serila interface.           */
/*@}*/

/*! \name Register definitions of Page 2
 *  \ingroup reg
 *  Following all register defintions of the RC522 Page 2.
 */
/*@{*/
#define     JREG_RFU20           0x20      /*   Currently not used.                                     */
#define     JREG_CRCRESULT1      0x21	   /*!< Contains MSByte of CRC Result.                          */
#define     JREG_CRCRESULT2      0x22	   /*!< Contains LSByte of CRC Result.                          */
#define     JREG_RFU23           0x23      /*   Currently not used.                                     */
#define     JREG_MODWIDTH        0x24      /*!< Contains modulation width setting.                      */
#define     JREG_RFU25           0x25      /*   Currently not used.                                     */
#define     JREG_RFCFG           0x26      /*!< Contains sensitivity of Rf Level detector, the receiver
                                                gain factor and the RfLevelAmp.                         */
#define     JREG_GSN             0x27      /*!< Contains the conductance and the modulation settings for
                                                the N-MOS transistor. */
#define     JREG_CWGSP           0x28      /*!< Contains the conductance for the P-Mos transistor.      */
#define     JREG_MODGSP          0x29      /*!< Contains the modulation index for the PMos transistor.  */
#define     JREG_TMODE           0x2A      /*!< Contains all settings for the timer and the highest 4
                                                bits of the prescaler.                                  */
#define     JREG_TPRESCALER      0x2B      /*!< Contais the lowest byte of the prescaler.               */
#define     JREG_TRELOADHI       0x2C      /*!< Contains the high byte of the reload value.             */
#define     JREG_TRELOADLO       0x2D      /*!< Contains the low byte of the reload value.              */
#define     JREG_TCOUNTERVALHI   0x2E      /*!< Contains the high byte of the counter value.            */
#define     JREG_TCOUNTERVALLO   0x2F      /*!< Contains the low byte of the counter value.             */
/*@}*/

/*! \name Register definitions of Page 3
 *  \ingroup reg
 *  Following all register defintions of the RC522 Page 3.
 */
/*@{*/
#define     JREG_RFU30           0x30      /*   Currently not used.                                     */
#define     JREG_TESTSEL1        0x31      /*   Test register                                           */
#define     JREG_TESTSEL2        0x32      /*   Test register                                           */
#define     JREG_TESTPINEN       0x33      /*   Test register                                           */
#define     JREG_TESTPINVALUE    0x34      /*   Test register                                           */
#define     JREG_TESTBUS         0x35      /*   Test register                                           */
#define     JREG_AUTOTEST        0x36      /*   Test register                                           */
#define     JREG_VERSION         0x37      /*!< Contains the product number and the version.            */
#define     JREG_ANALOGTEST      0x38      /*   Test register                                           */
#define     JREG_TESTDAC1        0x39      /*   Test register                                           */
#define     JREG_TESTDAC2        0x3A      /*   Test register                                           */
#define     JREG_TESTADC         0x3B      /*   Test register                                           */
#define     JREG_ANALOGUETEST1   0x3C      /*   Test register                                           */
#define     JREG_RFT3D           0x3D      /*   Test register                                           */
#define     JREG_RFT3E           0x3E      /*   Test register                                           */
#define     JREG_RFT3F           0x3F      /*   Test register                                           */
/*@}*/


/* /////////////////////////////////////////////////////////////////////////////
 * Possible commands
 * ////////////////////////////////////////////////////////////////////////// */
/*! \name RC522 Command definitions
 *  \ingroup reg
 *  Following all commands of the RC522.
 */
/*@{*/
#define     JCMD_IDLE          0x00 /*!< No action: cancel current command
                                     or home state. \n */
#define     JCMD_CALCCRC       0x03 /*!< Activate the CRC-Coprocessor \n<em><strong>
                                     Remark: </strong>The result of the CRC calculation can
                                     be read from the register CRCResultXXX </em>*/
#define     JCMD_TRANSMIT      0x04 /*!< Transmit data from FIFO to the card \n<em>
                                     <strong>Remark: </strong>If data is already in
                                     the FIFO when the command is activated, this data is
                                     transmitted immediately. It is possible to
                                     write data to the FIFO while the Transmit
                                     command is active. Thus it is possible to
                                     transmit an unlimited number of bytes in one
                                     stream by writting them to the FIFO in time.</em>*/
#define     JCMD_NOCMDCHANGE   0x07 /*!< This command does not change the actual commant of
                                     the RC522 and can only be written. \n<em><strong>
                                     Remark: </strong>This command is used for WakeUp procedure
                                     of RC522 to not change the current state. </em>*/
#define     JCMD_RECEIVE       0x08 /*!< Activate Receiver Circuitry. Before the
                                     receiver actually starts, the state machine
                                     waits until the time configured in the
                                     register RxWait has passed. \n<em><strong>
                                     Remark: </strong>It is possible to read any received
                                     data from the FIFO while the Receive command
                                     is active. Thus it is possible to receive an
                                     unlimited number of bytes by reading them
                                     from the FIFO in time. </em>*/
#define     JCMD_TRANSCEIVE    0x0C /*!< This Command has two modes:\n
                                     Transmits data from FIFO to the card and after
                                     that automatically activates
                                     the receiver. Before the receiver actually
                                     starts,the state machine waits until the
                                     time configured in the register RxWait has
                                     passed. \n <em><strong>
                                     Remark: </strong>This command is the combination of
                                     Transmit and Receive.</em> */
#define     JCMD_AUTHENT       0x0E /*!< Perform the card authentication using the
                                     Crypto1 algorithm.
                                     \n <em><strong>Remark: </strong></em>*/
#define     JCMD_SOFTRESET     0x0F /*!< Runs the Reset- and Initialisation Phase
                                     \n <em><strong>Remark:</strong> This command can
                                     be activated by software, but only by a Power-On
                                     or Hard Reset </em>*/
/*@}*/


/* /////////////////////////////////////////////////////////////////////////////
 * Bit Definitions
 * ////////////////////////////////////////////////////////////////////////// */
/*! \name RC522 Bit definitions of Page 0
 *  \ingroup reg
 *  Below there are useful bit definition of the RC522 register set of Page 0.
 */
/*@{*/
/* Command Register							(01) */
#define     JBIT_RCVOFF             0x20   /*!< Switches the receiver on/off. */
#define     JBIT_POWERDOWN          0x10   /*!< Switches RC522 to Power Down mode. */

/* CommIEn Register							(02) */
#define     JBIT_IRQINV             0x80   /*!< Inverts the output of IRQ Pin. */

/* DivIEn Register							(03) */
#define     JBIT_IRQPUSHPULL        0x80   /*!< Sets the IRQ pin to Push Pull mode. */

/* CommIEn and CommIrq Register         (02, 04) */
#define     JBIT_TXI                0x40   /*!< Bit position for Transmit Interrupt Enable/Request. */
#define     JBIT_RXI                0x20   /*!< Bit position for Receive Interrupt Enable/Request. */
#define     JBIT_IDLEI              0x10   /*!< Bit position for Idle Interrupt Enable/Request. */
#define     JBIT_HIALERTI           0x08   /*!< Bit position for HiAlert Interrupt Enable/Request. */
#define     JBIT_LOALERTI           0x04   /*!< Bit position for LoAlert Interrupt Enable/Request. */
#define     JBIT_ERRI               0x02   /*!< Bit position for Error Interrupt Enable/Request. */
#define     JBIT_TIMERI             0x01   /*!< Bit position for Timer Interrupt Enable/Request. */

/* DivIEn and DivIrq Register           (03, 05) */
#define     JBIT_SIGINACTI          0x10   /*!< Bit position for SiginAct Interrupt Enable/Request. */
#define     JBIT_CRCI               0x04   /*!< Bit position for CRC Interrupt Enable/Request. */

/* CommIrq and DivIrq Register          (04, 05) */
#define     JBIT_SET                0x80   /*!< Bit position to set/clear dedicated IRQ bits. */

/* Error Register 							(06) */
#define     JBIT_WRERR              0x80   /*!< Bit position for Write Access Error. */
#define     JBIT_TEMPERR            0x40   /*!< Bit position for Temerature Error. */
#define     JBIT_BUFFEROVFL         0x10   /*!< Bit position for Buffer Overflow Error. */
#define     JBIT_COLLERR            0x08   /*!< Bit position for Collision Error. */
#define     JBIT_CRCERR             0x04   /*!< Bit position for CRC Error. */
#define     JBIT_PARITYERR          0x02   /*!< Bit position for Parity Error. */
#define     JBIT_PROTERR            0x01   /*!< Bit position for Protocol Error. */

/* Status 1 Register 						(07) */
#define     JBIT_CRCOK              0x40   /*!< Bit position for status CRC OK. */
#define     JBIT_CRCREADY           0x20   /*!< Bit position for status CRC Ready. */
#define     JBIT_IRQ                0x10   /*!< Bit position for status IRQ is active. */
#define     JBIT_TRUNNUNG           0x08   /*!< Bit position for status Timer is running. */
#define     JBIT_HIALERT            0x02   /*!< Bit position for status HiAlert. */
#define     JBIT_LOALERT            0x01   /*!< Bit position for status LoAlert. */

/* Status 2 Register				    		(08) */
#define     JBIT_TEMPSENSOFF        0x80   /*!< Bit position to switch Temperture sensors on/off. */
#define     JBIT_I2CFORCEHS         0x40   /*!< Bit position to forece High speed mode for I2C Interface. */
#define     JBIT_CRYPTO1ON          0x08   /*!< Bit position for reader status Crypto is on. */

/* FIFOLevel Register				    		(0A) */
#define     JBIT_FLUSHBUFFER        0x80   /*!< Clears FIFO buffer if set to 1 */

/* Control Register					    		(0C) */
#define     JBIT_TSTOPNOW           0x80   /*!< Stops timer if set to 1. */
#define     JBIT_TSTARTNOW          0x40   /*!< Starts timer if set to 1. */

/* BitFraming Register					    (0D) */
#define     JBIT_STARTSEND          0x80   /*!< Starts transmission in transceive command if set to 1. */

/* BitFraming Register					    (0E) */
#define     JBIT_VALUESAFTERCOLL    0x80   /*!< Activates mode to keep data after collision. */
/*@}*/

/*! \name RC522 Bit definitions of Page 1
 *  \ingroup reg
 *  Below there are useful bit definition of the RC522 register set of Page 1.
 */
/*@{*/
/* Mode Register							(11) */
#define     JBIT_TXWAITRF           0x20   /*!< Tx waits until Rf is enabled until transmit is startet, else
                                                transmit is started immideately. */
#define     JBIT_POLSIGIN           0x08   /*!< Inverts polarity of SiginActIrq, if bit is set to 1 IRQ occures
                                                when Sigin line is 0. */

/* TxMode Register							(12) */
#define     JBIT_INVMOD             0x08   /*!< Activates inverted transmission mode. */

/* RxMode Register							(13) */
#define     JBIT_RXNOERR            0x08   /*!< If 1, receiver does not receive less than 4 bits. */

/* Definitions for Tx and Rx		    (12, 13) */
#define     JBIT_106KBPS            0x00   /*!< Activates speed of 106kbps. */
#define     JBIT_212KBPS            0x10   /*!< Activates speed of 212kbps. */
#define     JBIT_424KBPS            0x20   /*!< Activates speed of 424kbps. */

#define     JBIT_CRCEN              0x80   /*!< Activates transmit or receive CRC. */

/* TxControl Register						(14) */
#define     JBIT_INVTX2ON           0x80   /*!< Inverts the Tx2 output if drivers are switched on. */
#define     JBIT_INVTX1ON           0x40   /*!< Inverts the Tx1 output if drivers are switched on. */
#define     JBIT_INVTX2OFF          0x20   /*!< Inverts the Tx2 output if drivers are switched off. */
#define     JBIT_INVTX1OFF          0x10   /*!< Inverts the Tx1 output if drivers are switched off. */
#define     JBIT_TX2CW              0x08   /*!< Does not modulate the Tx2 output, only constant wave. */
#define     JBIT_TX2RFEN            0x02   /*!< Switches the driver for Tx2 pin on. */
#define     JBIT_TX1RFEN            0x01   /*!< Switches the driver for Tx1 pin on. */

/* Demod Register 							(19) */
#define     JBIT_FIXIQ              0x20   /*!< If set to 1 and the lower bit of AddIQ is set to 0, the receiving is fixed to I channel.
                                                If set to 1 and the lower bit of AddIQ is set to 1, the receiving is fixed to Q channel. */
/*@}*/


/*! \name RC522 Bit definitions of Page 2
 *  \ingroup reg
 *  Below there are useful bit definition of the RC522 register set.
 */
/*@{*/
/* TMode Register 							(2A) */
#define     JBIT_TAUTO              0x80   /*!< Sets the Timer start/stop conditions to Auto mode. */
#define     JBIT_TAUTORESTART       0x10   /*!< Restarts the timer automatically after finished
                                                counting down to 0. */
/*@}*/


/* /////////////////////////////////////////////////////////////////////////////
 * Bitmask Definitions
 * ////////////////////////////////////////////////////////////////////////// */
/*! \name RC522 Bitmask definitions
 *  \ingroup reg
 *  Below there are some useful mask defintions for the RC522. All specified
 *  bits are set to 1.
 */
/*@{*/

/* Command register                 (0x01)*/
#define     JMASK_COMMAND           0x0F   /*!< Bitmask for Command bits in Register JREG_COMMAND. */

/* Waterlevel register              (0x0B)*/
#define     JMASK_WATERLEVEL        0x3F   /*!< Bitmask for Waterlevel bits in register JREG_WATERLEVEL. */

/* Control register                 (0x0C)*/
#define     JMASK_RXBITS            0x07   /*!< Bitmask for RxLast bits in register JREG_CONTROL. */

/* Mode register                    (0x11)*/
#define     JMASK_CRCPRESET         0x03   /*!< Bitmask for CRCPreset bits in register JREG_MODE. */

/* TxMode register                  (0x12, 0x13)*/
#define     JMASK_SPEED             0x70   /*!< Bitmask for Tx/RxSpeed bits in register JREG_TXMODE and JREG_RXMODE. */

/* TxSel register                   (0x16)*/
#define     JMASK_DRIVERSEL         0x30   /*!< Bitmask for DriverSel bits in register JREG_TXSEL. */
#define     JMASK_SIGOUTSEL         0x0F   /*!< Bitmask for SigoutSel bits in register JREG_TXSEL. */

/* RxSel register                   (0x17)*/
#define     JMASK_UARTSEL           0xC0   /*!< Bitmask for UartSel bits in register JREG_RXSEL. */
#define     JMASK_RXWAIT            0x3F   /*!< Bitmask for RxWait bits in register JREG_RXSEL. */

/* RxThreshold register             (0x18)*/
#define     JMASK_MINLEVEL          0xF0   /*!< Bitmask for MinLevel bits in register JREG_RXTHRESHOLD. */
#define     JMASK_COLLEVEL          0x07   /*!< Bitmask for CollLevel bits in register JREG_RXTHRESHOLD. */

/* Demod register                   (0x19)*/
#define     JMASK_ADDIQ             0xC0   /*!< Bitmask for ADDIQ bits in register JREG_DEMOD. */
#define     JMASK_TAURCV            0x0C   /*!< Bitmask for TauRcv bits in register JREG_DEMOD. */
#define     JMASK_TAUSYNC           0x03   /*!< Bitmask for TauSync bits in register JREG_DEMOD. */

/* RFCfg register                   (0x26)*/
#define     JMASK_RXGAIN            0x70   /*!< Bitmask for RxGain bits in register JREG_RFCFG. */

/* GsN register                     (0x27)*/
#define     JMASK_CWGSN             0xF0   /*!< Bitmask for CWGsN bits in register JREG_GSN. */
#define     JMASK_MODGSN            0x0F   /*!< Bitmask for ModGsN bits in register JREG_GSN. */

/* CWGsP register                   (0x28)*/
#define     JMASK_CWGSP             0x3F   /*!< Bitmask for CWGsP bits in register JREG_CWGSP. */

/* ModGsP register                  (0x29)*/
#define     JMASK_MODGSP            0x3F   /*!< Bitmask for ModGsP bits in register JREG_MODGSP. */

/* TMode register                   (0x2A)*/
#define     JMASK_TGATED            0x60   /*!< Bitmask for TGated bits in register JREG_TMODE. */
#define     JMASK_TPRESCALER_HI     0x0F   /*!< Bitmask for TPrescalerHi bits in register JREG_TMODE. */

/*@}*/

//#endif /* __RC522REG_H__ */

/* ///////////////////////////////////////////////////////////////////////////
 * End of File
 * //////////////////////////////////////////////////////////////////////// */
/* /////////////////////////////////////////////////////////////////////////////////////////////////
//                     Copyright (c) Philips Semiconductors
//
//                       (C)PHILIPS Electronics
//         All rights are reserved. Reproduction in whole or in part is
//        prohibited without the written consent of the copyright owner.
//    Philips reserves the right to make changes without notice at any time.
//   Philips makes no warranty, expressed, implied or statutory, including but
//   not limited to any implied warranty of merchantability or fitness for any
//  particular purpose, or that the use will not infringe any third party patent,
//   copyright or trademark. Philips must not be liable for any loss or damage
//                            arising from its use.
///////////////////////////////////////////////////////////////////////////////////////////////// */

/*! \file ErrCode.h
 *
 * Project: Object Oriented Library Framework.
 *
 * Workfile: ErrCode.h
 * $Author: mha $
 * $Revision: 1.1 $
 * $Date: Fri Jun 30 14:33:59 2006 $
 *
 * $Author: mha $
 * $Revision: 1.1 $
 * $Date: Fri Jun 30 14:33:59 2006 $
 *
 * Comment:
 *  None.
 *
 *
 * History:
 *  GK: Generated 4. Sept. 2002
 *
*/
/*! \skipline STATUS_H__INCLUDED */

//#ifndef STATUS_H__INCLUDED                                                                          /*  */
//#define STATUS_H__INCLUDED                                                                          /*  */

/* S U C C E S S                                                                                            */
/*! \name Success
    \ingroup error */
/*@{*/
#define STATUS_SUCCESS                  (0x0000)            /*!< Returned in case of no error when there
                                                                    isn't any more appropriate code.        */
/*@}*/


/* C O M M U N I C A T I O N                                                                                */
/*! \name Communication Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_IO_TIMEOUT               (0x0001)  /*!< No reply received, e.g. PICC removal.    */
#define STATUS_CRC_ERROR                (0x0002)  /*!< Wrong CRC detected by RC or library.     */
#define STATUS_PARITY_ERROR             (0x0003)  /*!< Parity error detected by RC or library.  */
#define STATUS_BITCOUNT_ERROR           (0x0004)  /*!< Typically, the RC reports such an error. */
#define STATUS_FRAMING_ERROR            (0x0005)  /*!< Invalid frame format.                    */
#define STATUS_COLLISION_ERROR          (0x0006)  /*!< Typically, the RC repors such an error.  */
#define STATUS_BUFFER_TOO_SMALL         (0x0007)  /*!< Communication buffer size insufficient.  */
#define STATUS_ACCESS_DENIED            (0x0008)  /*!< Access has not been granted (readonly?). */
#define STATUS_BUFFER_OVERFLOW          (0x0009)  /*!< Attempt to write beyond the end of a
                                                          buffer.                                  */
#define STATUS_PROTOCOL_ERROR           (0x000B)  /*!< Mifare start bit wrong, buffer length
                                                          error.                                   */
#define STATUS_ERROR_NY_IMPLEMENTED     (0x000C)  /*!< Feature not yet implemented.             */
#define STATUS_FIFO_WRITE_ERROR         (0x000D)  /*!< Error caused because of interface conflict
                                                          during write access to FIFO.             */
#define STATUS_USERBUFFER_FULL          (0x000E)  /*!< The user buffer is full, the calling
                                                          application/routine gets the chance to
                                                          save user buffer data and start over.    */
/*@}*/


/* I N T E R F A C E (Device as well as function parameters) errors:                               */
/*! \name Interface Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_INVALID_PARAMETER        (0x0101)  /*!< Parameter is invalid (range, format).    */
#define STATUS_UNSUPPORTED_PARAMETER    (0x0102)  /*!< Parameter value/format is correct but not
                                                          supported in the current configuration.  */
#define STATUS_UNSUPPORTED_COMMAND      (0x0103)  /*!< The device does not support the command. */
#define STATUS_INTERFACE_ERROR          (0x0104)  /*!< Host-peripheral interface error.         */
#define STATUS_INVALID_FORMAT           (0x0105)  /*!< The data format does not match the spec. */
#define STATUS_INTERFACE_NOT_ENABLED    (0x0106)  /*!< This interface is currently(!) not
                                                          supported (e.g. function ptr. to NULL).  */
/*@}*/



/* M F  errors:                                                                                             */
/*! \name Mifare Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_AUTHENT_ERROR            (0x0201)  /*!< Authentication failure (e.g. key
                                                          mismatch).                                         */
#define STATUS_ACK_SUPPOSED             (0x0202)  /*!< Single byte or nibble received, CRC error
                                                          detected, possibly MF (N)ACK response.            */
/*@}*/


/* I S O 1 4 4 4 3 . 4 Level specific errors:                                                               */
/*! \name ISO 14443-4 Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_BLOCKNR_NOT_EQUAL        (0x0301)  /*!< Frame OK, but Blocknumber mismatch.               */
/*@}*/


/* N F C  Errors and Stati:                                                                                 */
/*! \name NFC Errors
    \ingroup error */
/*@{*/
#define STATUS_TARGET_DEADLOCKED        (0x0401)  /*!< Target has not sent any data, but RF
                                                          (generated by the Target) is still switched on.   */
#define STATUS_TARGET_SET_TOX           (0x0402)  /*!< Target has sent Timeout Extension Request.        */
#define STATUS_TARGET_RESET_TOX         (0x0403)  /*!< Reset timeout-value after Timeout Extension.      */
/*@}*/


/* I S O 1 4 4 4 3 . 3 Level specific errors:                                                     */
/*! \name ISO 14443-3 Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_WRONG_UID_CHECKBYTE      (0x0501)  /*!< UID check byte is wrong.                 */
#define STATUS_WRONG_HALT_FORMAT        (0x0502)  /*!< HALT Format error.                       */
/*@}*/


/*  I D  M A N A G E R  specific errors:                                                                    */
/*! \name ID-Manager Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_ID_ALREADY_IN_USE        (0x0601)  /*!< ID cannot be assigned because it is already used. */
#define STATUS_INSTANCE_ALREADY_IN_USE  (0x0602)  /*!< INSTANCE cannot be assigned because it is already used. */
#define STATUS_ID_NOT_IN_USE            (0x0603)  /*!< Specified ID is not in use.                       */
#define STATUS_NO_ID_AVAILABLE          (0x0604)  /*!< No ID is available, all are occupied.             */
/*@}*/


/* O T H E R   E R R O R S :                                                                       */
/*! \name Other Errors/Status Values
    \ingroup error */
/*@{*/
#define STATUS_OTHER_ERROR              (0x7E01)  /*!< Unspecified, error, non-categorised.     */
#define STATUS_INSUFFICIENT_RESOURCES   (0x7E02)  /*!< The system runs low of resources!        */
#define STATUS_INVALID_DEVICE_STATE     (0x7E03)  /*!< The (sub-)system is in a state which
                                                          does not allow the operation.            */
#define STATUS_RC522_TEMP_ERROR        (0x7E04)  /*!< Temperature error indicated by RC522 HW.*/
/*@}*/


//#endif                                               /* STATUS_H__INCLUDED                    */

/* EOF */
/* /////////////////////////////////////////////////////////////////////////////////////////////////
//                     Copyright (c) Philips Semiconductors
//
//         All rights are reserved. Reproduction in whole or in part is
//        prohibited without the written consent of the copyright owner.
//    Philips reserves the right to make changes without notice at any time.
//   Philips makes no warranty, expressed, implied or statutory, including but
//   not limited to any implied warranty of merchantability or fitness for any
//  particular purpose, or that the use will not infringe any third party patent,
//   copyright or trademark. Philips must not be liable for any loss or damage
//                            arising from its use.
///////////////////////////////////////////////////////////////////////////////////////////////// */

/*! \file OpCtrl.h
 *
 * Project: Project: Mifare reader with RC522
 *
 *  Source: OpCtrl.h
 * $Author: mha $
 * $Revision: 1.1 $
 * $Date: Fri Jun 30 14:33:59 2006 $
 *
 * Comment:
 *  This file defines ordinal numbers for configuration types and settings or values respectively.
 *
 * History:
 *  GK: Generated 11. June 2003
 *
 */


//#ifndef __OPCRTL_H__
//#define __OPCRTL_H__

/* Ordinals are all byte ! */

/*! \name Metrics
   \ingroup opctl
   Definitions for static settings, version and there more.
*/
/*@{*/
#define RCO_GROUP_RCMETRICS            (0x00)    /*!< Metrics group. */
#define RCO_ATTR_VERSION               (0x00)    /*!< RC version retrieval. */
/*@}*/

/*! \name Operation Modes
   \ingroup opctl
   Definitions for dedicated mode settings including Softreset, Reader's, Initiator and Target.
*/
/*@{*/
#define RCO_GROUP_MODE                 (0x01)    /*!< Operation mode group. */
#define RCO_ATTR_SOFTRESET             (0x00)    /*!< Resets Hardware to defaults */
#define RCO_ATTR_MFRD_A                (0x03)    /*!< Mode = Mifare Reader  \n
                                                     param_a: TxSpeed, param_b: RxSpeed (see group communication) */
/*@}*/

/*! \name Communication
   \ingroup opctl
   Definitions for serial interface and contactless interface.
*/
/*@{*/
#define RCO_GROUP_COMMUNICATION        (0x02)    /*!< Communications group. */
#define RCO_ATTR_RFBAUD                (0x00)    /*!< Baud rate for the CL RF UART, \n
                                                      TxSpeed: param_a and RxSpeed: param_b.   */
#define RCO_VAL_RF106K                 (0x00)    /* RF UART speed setting(s). */
#define RCO_VAL_RF212K                 (0x01)    /* */
#define RCO_VAL_RF424K                 (0x02)    /* */
#define RCO_ATTR_RS232BAUD             (0x01)    /*!< Serial interface baud setting. */
#define RCO_VAL_SER9600                (0x00)    /* Serial RS232 speed setting(s). */
#define RCO_VAL_SER19200               (0x01)    /* */
#define RCO_VAL_SER38400               (0x02)    /* */
#define RCO_VAL_SER57600               (0x03)    /* */
#define RCO_VAL_SER115200              (0x04)    /* */
#define RCO_VAL_SER230400              (0x05)    /* */
#define RCO_VAL_SER460800              (0x06)    /* */
#define RCO_VAL_SER921600              (0x07)    /* */
#define RCO_VAL_SER1288000             (0x08)    /* */
#define RCO_ATTR_CRC                   (0x03)    /*!< TxCRC: param_a, RxCRC: param_b */
#define RCO_VAL_CRC_OFF                (0x00)    /* */
#define RCO_VAL_CRC_ON                 (0x01)    /* */
#define RCO_ATTR_FLUSH_BUFFER          (0x04)    /*!< no parameter needed */
#define RCO_ATTR_DRIVER_SEL            (0x06)    /*!< tbd */
#define RCO_ATTR_SIGOUT_SEL            (0x07)    /*!< tbd */
#define RCO_ATTR_UART_SEL              (0x08)    /*!< tbd */
/*@}*/

/*! \name System
   \ingroup opctl
   Definitions for system settings like PuwerDown, Sensor activation, IRQ behaviour, Waterlevel settings.
*/
/*@{*/
#define RCO_GROUP_SYSTEM               (0x03)    /*!< System group/settings. */
#define RCO_ATTR_SOFT_POWER_DOWN       (0x00)    /*!< Soft Power down functional attribute. */
#define RCO_VAL_SOFT_POWER_UP          (0x00)    /* */
#define RCO_VAL_SOFT_POWER_DOWN        (0x01)    /* */
#define RCO_ATTR_TEMP_SENS             (0x01)    /* tbd */
#define RCO_VAL_TEMP_SENS_ON           (0x00)    /* */
#define RCO_VAL_TEMP_SENS_OFF          (0x01)    /* */
#define RCO_ATTR_IRQ_PUSH_PULL         (0x02)    /*!< IrqMode = PushPull or open drain output pad */
#define RCO_VAL_IRQ_PUSH_PULL_OFF      (0x00)    /* */
#define RCO_VAL_IRQ_PUSH_PULL_ON       (0x01)    /* */
#define RCO_ATTR_IRQ_INV               (0x03)    /*!< Inversion of Irq output pad */
#define RCO_VAL_IRQ_INV_OFF            (0x00)    /* */
#define RCO_VAL_IRQ_INV_ON             (0x01)    /* */
#define RCO_ATTR_WATERLEVEL            (0x04)    /* tbd */
#define RCO_ATTR_SELFTEST              (0x05)    /*!< Activetes selftest for external components. \n
                                                       param_a returns the status of the check.
                                                       0x00: pass, other: fail */
/*@}*/

/* \name RF driver settings
   \ingroup opctl
   Definitions for RF and driver settings.
*/
/*@{*/
#define RCO_GROUP_RF                   (0x04)    /*!< RF driver settings. */
#define RCO_ATTR_RCV_OFF               (0x00)    /*!< Switches Receiver either on or off. */
#define RCO_VAL_RCV_ON                 (0x00)    /* */
#define RCO_VAL_RCV_OFF                (0x01)    /* */
#define RCO_ATTR_RF_EN                 (0x01)    /*!< Switches the driver on or off.\n
                                                       param_a: Tx1, param_b: Tx2 */
#define RCO_VAL_RF_OFF                 (0x00)    /* */
#define RCO_VAL_RF_ON                  (0x01)    /* */
#define RCO_ATTR_RF_INV_TXON           (0x03)    /*!< Activates or deactivates the inverted driver output in case the drivers are switched on.\n
                                                      param_a: Tx1, param_b: Tx2 */
#define RCO_ATTR_RF_INV_TXOFF          (0x04)    /*!< Activates or deactivates the inverted driver output in case the drivers are switched off (loadmodulation).\n
                                                       param_a: Tx1, param_b: Tx2 */
#define RCO_VAL_RF_INV_TX_OFF          (0x00)    /* */
#define RCO_VAL_RF_INV_TX_ON           (0x01)    /* */
/*@}*/

/* \name Analogue signal settings
   \ingroup opctl
   Definitions for Analogue signal settings like Gain, Modulation index, ... .
*/
/*@{*/
#define RCO_GROUP_SIGNAL               (0x05)    /*!< Signal interface group. */
#define RCO_ATTR_GAIN                  (0x00)    /*!< Sets the Gain attribute (max. 0x07). */
#define RCO_ATTR_MINLEVEL              (0x01)    /*!< Sets the RX min. level (max. 0x0F). */
#define RCO_ATTR_COLLEVEL              (0x02)    /*!< Sets the Collision Min. level for the receiver (max. 0x07). */
#define RCO_ATTR_RXWAIT                (0x03)    /*!< Sets the number of RxWaitBits (max. 0x3F) [bits]      */
#define RCO_ATTR_CWGSN                 (0x04)    /*!< Sets the CW of the N-Mos driver (max. 0x0F) */
#define RCO_ATTR_MODGSN                (0x05)    /*!< Sets the MOD of the N-Mos driver (max. 0x0F) */
#define RCO_ATTR_CWGSP                 (0x06)    /*!< Sets the CW of the P-Mos driver (max. 0x3F) */
#define RCO_ATTR_MODGSP                (0x07)    /*!< Sets the MOD of the P-Mos driver (max. 0x3F) */
#define RCO_ATTR_MODWIDTH              (0x09)    /*!< Sets the Modwidth register */
#define RCO_ATTR_ADD_IQ                (0x0A)    /*!< Sets the ADDIQ mode (max. 0x03) */
#define RCO_ATTR_TAU_RCV               (0x0C)    /* */
#define RCO_ATTR_TAU_SYNC              (0x0D)    /* */
/*@}*/

/* \name Timer settings
   \ingroup opctl
   Definitions for timer relevant settings.
*/
/*@{*/
#define RCO_GROUP_TIMER                (0x06)    /*!< Timer relevant settings. */
#define RCO_ATTR_TIMER_START_STOP      (0x00)    /*!< tbd */
#define RCO_VAL_TIMER_START_NOW        (0x00)    /* */
#define RCO_VAL_TIMER_STOP_NOW         (0x01)    /* */
#define RCO_ATTR_TIMER_AUTO            (0x01)    /*!< tbd */
#define RCO_ATTR_TIMER_AUTO_RESTART    (0x02)    /*!< tbd */
#define RCO_VAL_TIMER_AUTO_OFF         (0x00)    /* */
#define RCO_VAL_TIMER_AUTO_ON          (0x01)    /* */
#define RCO_ATTR_TIMER_GATED           (0x03)    /*!< param_a: value of TGated (max. 3) */
#define RCO_ATTR_TIMER_PRESCALER       (0x04)    /*!< param_a: TPrescaler_LO, \n
                                                    param_b: TPrescaler_HI (max. 0x0F) */
#define RCO_ATTR_TIMER_RELOAD          (0x05)    /*!< param_a: TReloadVal_LO \n
                                                    param_b: TReloadVal_HI      */
#define RCO_ATTR_TIMER_COUNTER         (0x06)    /*!< param_a: TCounterVal_LO, \n param_b: TCounterVal_HI */
/*@}*/

/* \name CRC
   \ingroup opctl
   Definitions for CRC calculation.
*/
/*@{*/
#define RCO_GROUP_CRC                  (0x07)    /*!< CRC relevant settings. */
#define RCO_ATTR_CRC_PRESET            (0x00)    /*!< param_a: use only bits 0 and 1, \n
                                                      value 00 -> Preset: 0x00, 0x00 \n
                                                      value 01 -> Preset: 0x63, 0x63 \n
                                                      value 10 -> Preset: 0xA6, 0x71 \n
                                                      value 11 -> Preset: 0xFF, 0xFF \n */
#define RCO_ATTR_CRC_RESULT            (0x01)    /*!< param_a: Result_LSB, param_b: Result_MSB */
#define RCO_ATTR_CRC_CALCULATE         (0x03)    /*!< in: param_a: length of data (max 64), data: start of data buffer
                                                     out: param_a: result LSB, param_b: result MSB */
/*@}*/

/* \name Protocoll settings
   \ingroup opctl
   Definitions for register settings for protocol operation.
*/
/*@{*/
#define RCO_GROUP_PROTOCOL             (0x08)    /*!< Protocol (data exchange) settings. */
#define RCO_ATTR_CRYPTO1_ON            (0x08)    /*       */
/*@}*/

#define RC522_UARTSEL_SHL_VALUE        0x06 /* */
#define RC522_SPEED_SHL_VALUE          0x04 /* */
#define RC522_MINLEVEL_SHL_VALUE       0x04 /* */
#define RC522_GAIN_SHL_VALUE           0x04 /* */
#define RC522_TGATED_SHL_VALUE         0x05 /* */
#define RC522_DRIVERSEL_SHL_VALUE      0x04 /* */
#define RC522_CWGSN_SHL_VALUE          0x04 /* */
#define RC522_ADDIQ_SHL_VALUE          0x06 /* */
#define RC522_TAURCV_SHL_VALUE         0x02 /* */
#define RC522_CRYPTO1ON_SHL_VALUE      0x03 /* */


//#endif /* __OPCRTL_H__*/



/* \ingroup mifare
 *  \name Commands
 *  Command definitions for Mifare operation. These command bytes comply to the MIFARE
 *  specification and serve as one parameter for the MIFARE transaction commands defined
 *  within the scope of this implementation.
 */
/*@{*/
#define MIFARE_AUTHENT_A            ((byte)0x60)   /* AUTHENT A command. */
#define MIFARE_AUTHENT_B            ((byte)0x61)   /* AUTHENT B command. */
#define MIFARE_READ                 ((byte)0x30)   /* READ command. */
#define MIFARE_WRITE                ((byte)0xA0)   /* WRITE 16 bytes command. */
#define MIFARE_WRITE4               ((byte)0xA2)   /* WRITE 4 bytes command. */
#define MIFARE_INCREMENT            ((byte)0xC1)   /* INCREMENT command. */
#define MIFARE_DECREMENT            ((byte)0xC0)   /* DECREMENT command.  */
#define MIFARE_RESTORE              ((byte)0xC2)   /* RESTORE command. */
#define MIFARE_TRANSFER             ((byte)0xB0)   /* TRANSFER command. */
#define MIFARE_NOCOMMAND            ((byte)0x00)   /* VOID command (no MIFARE command). */


/*@{*/
#define MIFARE_EXPECT_TIMEOUT       ((byte)0x01)   /*!< Tells the library to expect a timeout. */
#define MIFARE_EXPECT_ACK           ((byte)0x02)   /*!< Let the library expect an Acknowledge response. */
#define MIFARE_EXPECT_DATA          ((byte)0x04)   /*!< The library shall expect data. */
/*!
 * (Not-) Acknowledge:
 * When a MF PICC returns ACK, a logical AND with the mask yields a value other than 0. Otherwise,
 * in case of NACK, the mask zeroes the byte (or nibble respectively).
 */
#define MIFARE_ACK_MASK             ((byte)0x0A)           /* */

/* Definitions for Request command. */
#define REQUEST_BITS                0x07
#define ATQA_LENGTH                 0x02

/* Request codes */
/*! \ingroup mifare
    \brief Request code for all devices. */
#define ISO14443_3_REQALL           0x52
/*! \ingroup mifare
    \brief Request code only for idle devices. */
#define ISO14443_3_REQIDL           0x26

#define ISO14443_3_REQB           0x00
#define ISO14443_3_WUPB           0x08

/* Definitions for lower Anticollision / Select functions. */
#define BITS_PER_BYTE               0x08
#define UPPER_NIBBLE_SHIFT          0x04
#define COMPLETE_UID_BITS           0x28
#define NVB_MIN_PARAMETER           0x20
#define NVB_MAX_PARAMETER           0x70

/*Command byte definitions for Anticollision/Select functions.*/
#define SELECT_CASCADE_LEVEL_1      0x93
#define SELECT_CASCADE_LEVEL_2      0x95
#define SELECT_CASCADE_LEVEL_3      0x97
#define MAX_CASCADE_LEVELS          0x03
#define SINGLE_UID_LENGTH           0x20
#define CASCADE_BIT                 0x04

/* Definitions for Select functions. */
#define SAK_LENGTH                  0x01

/* Command and Parameter byte definitions for HaltA function. */
#define HALTA_CMD                   0x50
#define HALTA_PARAM                 0x00
#define HALTA_CMD_LENGTH            0x02


#define  ISO1443_TYPEA   0x0A
#define  ISO1443_TYPEB   0x0B

#define byte unsigned char 
typedef struct
{
	byte cmd;                 //!< command code
	char           status;              //!< communication status
	byte  nBytesSent;          //!< how many bytes already sent
	byte  nBytesToSend;        //!< how many bytes to send
	byte  nBytesReceived;      //!< how many bytes received
	int nBitsReceived;       //!< how many bits received
	byte  collPos;             //collision position
} CmfInfo;

#define ResetInfo(info)    \
	info.cmd            = 0;\
	info.status         = STATUS_SUCCESS;\
	info.nBytesSent     = 0;\
	info.nBytesToSend   = 0;\
	info.nBytesReceived = 0;\
	info.nBitsReceived  = 0;\
	info.collPos        = 0;

__END_DECLS

#endif

