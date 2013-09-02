/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "rfid rftool.c"
#include <utils/Log.h>

#include <fcntl.h>
#include <errno.h>
#include <cutils/log.h>
#include <cutils/atomic.h>
#include <linux/delay.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include "rfid.h"
#include "jni.h"

#define DEVICE_NAME "/dev/ttySAC1"

CmfInfo Info;
CmfInfo *IsrInfo = {0};
static int fd = 0;
byte SerBuffer[20];


//=======================================================================
static int delay(int msec)
{
	int i,j;
	for (i=0;i<msec;i++)
	{
		for (j=0;j<50000;j++);;
	}
	return 0;
}

static int open(const char* port) {

	int err;
	struct stat    status;        // Returns the status of the baseband serial port.
	struct termios curr_term;     // The current serial port configuration descriptor.
	
	if (fd > 0) return 0;
		
	// Check for a valid PC COM Port Name
	if ( port == 0 )
	{
	  LOGD( " rfid_comm_open:  ERROR: Port Invalid Port Name\r\n" );
	  return( -1 );
	}
	// First check the port exists.
	// This avoids thread cancellation if the port doesn't exist.
	if ( ( err = stat( port, &status ) ) == -1 )
	{
	  LOGD( " rfid_comm_open: stat(%s,*) = %d,  errno %d\r\n", port, err, errno );
	  return( -1 );
	}
  // Open the serial port.
	if((fd = open(port, (O_RDWR | O_NOCTTY | O_NONBLOCK))) <= 0) {
		LOGE("Rfid Stub: failed to open %s -- %s.",port, strerror(errno));
		return -1;
	}

	tcflush(fd, TCIOFLUSH);
	// Get the current serial port terminal state.
	if ( ( err = tcgetattr( fd, &curr_term ) ) != 0 )
	{
	  LOGD( " rfid_comm_open: tcgetattr(%d) = %d,  errno %d\r\n", fd, err, errno );
	  close( fd );
	  fd = 0;
	  return( -1 );
	}

	// Set the terminal state to local (ie a local serial port, not a modem port),
	// raw (ie binary mode).
	curr_term.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	curr_term.c_oflag &= ~OPOST;
	curr_term.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	curr_term.c_cflag &= ~(CSIZE|PARENB);
	curr_term.c_cflag |= CS8;
  curr_term.c_cflag &= ~CRTSCTS;

	tcsetattr(fd, TCSANOW, &curr_term);
	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &curr_term);
	tcflush(fd, TCIOFLUSH);
	tcflush(fd, TCIOFLUSH);

  // Set the input baud rates in the termios.9600
	if ( cfsetispeed( &curr_term, B9600 ) )
	{
	  close( fd );
	  fd = 0;
	  return( -1 );
	}

	// Set the output baud rates in the termios.
	if ( cfsetospeed( &curr_term, B9600 ) )
	{
	  close( fd );
	  fd = 0;
	  return( -1 );
	}
	tcsetattr(fd, TCSANOW, &curr_term);
	return 0;
}

static int close() {

	if(fd > 0) {
		close(fd);
		fd=0;
	}
	
	return 0;
}

static int write_bytes(byte* buff, int length)
{
	int ret;
	if (fd <= 0) return -1;
	ret = write(fd, buff, length);
	return ret;
}

static int read_bytes(byte* buff, int length)
{
	int ret;
	if (fd <= 0) return -1;
	ret=read( fd, buff, length );
	return ret;
}

static void rc_setreg(byte addr, byte val)
{
	if (fd <= 0) return;
	addr &= 0x3f;
	byte buff[2] = {addr,val};
	write_bytes(buff,2);
	read_bytes(&val,1);
}

static byte rc_getreg(byte addr)
{
	byte val;
	if (fd <= 0) return -1;
	addr = (addr & 0x3f) | 0x80;
	write_bytes((byte*)&addr,1);
	if (read_bytes(&val,1) != 1)
	{
		return 0xff;
	}
	return val;
}

static void rc_modifyreg(byte addr, byte modify_val, byte mask_byte)
{
	byte val;
	if (fd <= 0) return;
	val = rc_getreg(addr);
	if(modify_val)
	{
		val |= mask_byte;
	}
	else
	{
		val &= (~mask_byte);
	}
	rc_setreg(addr, val);
}

/*************************************************
Function:       SetPowerDown
Description:
set the rc522 enter or exit power down mode
Parameter:
ucFlag     0   --  exit power down mode
!0  --  enter power down mode
Return:
int      status of implement
**************************************************/
static void set_powerdown(byte flag)
{
	byte val;
	/*
	Note: The bit Power Down can not be set when the SoftReset command has been activated.
	*/
	if (fd <= 0) return;
	if(flag)
	{
		val = rc_getreg(JREG_COMMAND);  //enter power down mode
		val |= 0x10;
		rc_setreg(JREG_COMMAND, val);
	}
	else
	{
		val = rc_getreg(JREG_COMMAND);  //disable power down mode
		val &= (~0x10);
		rc_setreg(JREG_COMMAND, val);
	}
}

static int set_timeoutus(unsigned int micro_seconds)
{
	unsigned int RegVal;
	byte TmpVal;
	RegVal = micro_seconds / 100;
	if (fd <= 0) return -1;
	/*
	NOTE: The supported hardware range is bigger, since the prescaler here
	is always set to 100 us.
	*/
	if(RegVal >= 0xfff)
	{
		return STATUS_INVALID_PARAMETER;
	}
	rc_modifyreg(JREG_TMODE, 1, JBIT_TAUTO);

	rc_setreg(JREG_TPRESCALER, 0xa6);

	TmpVal = rc_getreg(JREG_TMODE);
	TmpVal &= 0xf0;
	TmpVal |= 0x02;
	rc_setreg(JREG_TMODE, TmpVal);//82

	rc_setreg(JREG_TRELOADLO, ((byte)(RegVal&0xff)));
	rc_setreg(JREG_TRELOADHI, ((byte)((RegVal>>8)&0xff)));
	return STATUS_SUCCESS;
}

static int change_baudrate(int baudrate)
{
	int   status = STATUS_SUCCESS;
	byte setRegVal;
	if (fd <= 0) return -1;
	switch(baudrate)
	{
	case 9600:
		setRegVal = 0xEB;
		break;

	case 14400:
		setRegVal = 0xDA;
		break;

	case 19200:
		setRegVal = 0xCB;
		break;

	case 38400:
		setRegVal = 0xAB;
		break;

	case 57600:
		setRegVal = 0x9A;
		break;

	case 115200:
		setRegVal = 0x7A;
		break;

	case 128000:
		setRegVal = 0x74;
		break;

	default:
		status = STATUS_INVALID_PARAMETER;
		break;
	}

	/* Set the appropriate value */
	if (status == STATUS_SUCCESS)
		rc_setreg(JREG_SERIALSPEED,setRegVal);
	/* Now the RC522 is set to the new speed*/
	return status;
}

static void rc522_reset()
{
	/*RstL();*/
	/*delay(10);*/
	//RstH();
	if (fd <= 0) return;
	rc_setreg(JREG_COMMAND, 0x0f); /*reset the RC522*/
}

static int  pcdcmd(byte cmd,byte *ExchangeBuf,CmfInfo  *info)
{
	int          status    = STATUS_SUCCESS;
	if (fd <= 0) return -1;

	byte  commIrqEn   = 0;
	byte  divIrqEn    = 0;
	byte  waitForComm = JBIT_ERRI | JBIT_TXI;
	byte  waitForDiv  = 0;
	byte  doReceive   = 0;
	byte  i;
	byte  getRegVal,setRegVal;

	byte  nbytes, nbits;
	unsigned int counter;

	/*remove all Interrupt request flags that are used during function,
	keep all other like they are*/
	rc_setreg(JREG_COMMIRQ, waitForComm);
	rc_setreg(JREG_DIVIRQ, waitForDiv);
	rc_setreg(JREG_FIFOLEVEL, JBIT_FLUSHBUFFER);

	/*disable command or set to transceive*/
	getRegVal = rc_getreg(JREG_COMMAND);
	if(cmd == JCMD_TRANSCEIVE)
	{
		/*re-init the transceive command*/
		setRegVal = (getRegVal & ~JMASK_COMMAND) | JCMD_TRANSCEIVE;
		rc_setreg(JREG_COMMAND, setRegVal);//0c
	}
	else
	{
		/*clear current command*/
		setRegVal = (getRegVal & ~JMASK_COMMAND);
		rc_setreg(JREG_COMMAND, setRegVal);
	}
	IsrInfo = info;
	switch(cmd)
	{
	case JCMD_IDLE:         /* values are 00, so return immediately after all bytes written to FIFO */
		waitForComm = 0;
		waitForDiv  = 0;
		break;
	case JCMD_CALCCRC:      /* values are 00, so return immediately after all bytes written to FIFO */
		waitForComm = 0;
		waitForDiv  = 0;
		break;
	case JCMD_TRANSMIT:
		commIrqEn = JBIT_TXI | JBIT_TIMERI;
		waitForComm = JBIT_TXI;
		break;
	case JCMD_RECEIVE:
		commIrqEn = JBIT_RXI | JBIT_TIMERI | JBIT_ERRI;
		waitForComm = JBIT_RXI | JBIT_TIMERI | JBIT_ERRI;
		doReceive = 1;
		break;
	case JCMD_TRANSCEIVE:
		commIrqEn = JBIT_RXI | JBIT_TIMERI | JBIT_ERRI;
		waitForComm = JBIT_RXI | JBIT_TIMERI | JBIT_ERRI;
		doReceive = 1;
		break;
	case JCMD_AUTHENT:
		commIrqEn = JBIT_IDLEI | JBIT_TIMERI | JBIT_ERRI;
		waitForComm = JBIT_IDLEI | JBIT_TIMERI | JBIT_ERRI;
		break;
	case JCMD_SOFTRESET:    /* values are 0x00 for IrqEn and for waitFor, nothing to do */
		waitForComm = 0;
		waitForDiv  = 0;
		break;
	default:
		status = STATUS_UNSUPPORTED_COMMAND;
	}
	if(status == STATUS_SUCCESS)
	{
		/* activate necessary communication Irq's */
		getRegVal = rc_getreg(JREG_COMMIEN);
		rc_setreg(JREG_COMMIEN, getRegVal | commIrqEn);

		/* activate necessary other Irq's */
		getRegVal = rc_getreg(JREG_DIVIEN);
		rc_setreg(JREG_DIVIEN, getRegVal | divIrqEn);

		/*write data to FIFO*/
		for(i=0; i<IsrInfo->nBytesToSend; i++)
		{
			rc_setreg(JREG_FIFODATA, ExchangeBuf[i]);
		}

		/*do seperate action if command to be executed is transceive*/
		if(cmd == JCMD_TRANSCEIVE)
		{
			/*TRx is always an endless loop, Initiator and Target must set STARTSEND.*/
			rc_modifyreg(JREG_BITFRAMING, 1, JBIT_STARTSEND);
		}
		else
		{
			getRegVal = rc_getreg(JREG_COMMAND);
			rc_setreg(JREG_COMMAND, (getRegVal & ~JMASK_COMMAND) | cmd);
		}

		/*polling mode*/
		getRegVal = 0;
		setRegVal = 0;
		/*counter = 0; [>Just for debug<]*/
		while(!(waitForComm ? (waitForComm & setRegVal) : 1) ||
			!(waitForDiv ? (waitForDiv & getRegVal) :1))
		{
			setRegVal = rc_getreg(JREG_COMMIRQ);
			getRegVal = rc_getreg(JREG_DIVIRQ);
			/*counter ++;*/
			/*if(counter > 0x0100)*/
			/*break;*/
		}
		/*store IRQ bits for clearance afterwards*/
		waitForComm = (byte)(waitForComm & setRegVal);
		waitForDiv  = (byte)(waitForDiv & getRegVal);

		/*set status to Timer Interrupt occurence*/
		if (setRegVal & JBIT_TIMERI)
		{
			status = STATUS_IO_TIMEOUT;
		}
	}

	/*disable all interrupt sources*/
	rc_modifyreg(JREG_COMMIEN, 0, commIrqEn);

	rc_modifyreg(JREG_DIVIEN, 0, divIrqEn);

	if(doReceive && (status == STATUS_SUCCESS))
	{
		/*read number of bytes received (used for error check and correct transaction*/
		IsrInfo->nBytesReceived = rc_getreg(JREG_FIFOLEVEL);
		nbytes = IsrInfo->nBytesReceived;
		getRegVal = rc_getreg(JREG_CONTROL);
		IsrInfo->nBitsReceived = (byte)(getRegVal & 0x07);
		nbits = IsrInfo->nBitsReceived;

		getRegVal = rc_getreg(JREG_ERROR);
		/*set status information if error occured*/
		if(getRegVal)
		{
			if(getRegVal & JBIT_COLLERR)
				status = STATUS_COLLISION_ERROR;         /* Collision Error */
			else if(getRegVal & JBIT_PARITYERR)
				status = STATUS_PARITY_ERROR;            /* Parity Error */

			if(getRegVal & JBIT_PROTERR)
				status = STATUS_PROTOCOL_ERROR;          /* Protocoll Error */
			else if(getRegVal & JBIT_BUFFEROVFL)
				status = STATUS_BUFFER_OVERFLOW;         /* BufferOverflow Error */
			else if(getRegVal & JBIT_CRCERR)
			{   /* CRC Error */
				if(IsrInfo->nBytesReceived == 0x01 &&
					(IsrInfo->nBitsReceived == 0x04 ||
					IsrInfo->nBitsReceived == 0x00))
				{   /* CRC Error and only one byte received might be a Mifare (N)ACK */
					ExchangeBuf[0] = rc_getreg(JREG_FIFODATA);
					IsrInfo->nBytesReceived = 1;
					status = STATUS_ACK_SUPPOSED;        /* (N)ACK supposed */
				}
				else
					status = STATUS_CRC_ERROR;           /* CRC Error */
			}
			else if(getRegVal & JBIT_TEMPERR)
				status = STATUS_RC522_TEMP_ERROR;       /* Temperature Error */
			if(getRegVal & JBIT_WRERR)
				status = STATUS_FIFO_WRITE_ERROR;        /* Error Writing to FIFO */
			if(status == STATUS_SUCCESS)
				status = STATUS_ERROR_NY_IMPLEMENTED;    /* Error not yet implemented, shall never occur! */

			/* if an error occured, clear error register before IRQ register */
			rc_setreg(JREG_ERROR, 0);
		}

		/*read data from FIFO and set response parameter*/
		if(status != STATUS_ACK_SUPPOSED)
		{
			for(i=0; i<IsrInfo->nBytesReceived; i++)
			{
				ExchangeBuf[i] = rc_getreg(JREG_FIFODATA);
			}
			/*in case of incomplete last byte reduce number of complete bytes by 1*/
			if(IsrInfo->nBitsReceived && IsrInfo->nBytesReceived)
				IsrInfo->nBytesReceived --;
		}
	}
	rc_setreg(JREG_COMMIRQ, waitForComm);
	rc_setreg(JREG_DIVIRQ, waitForDiv);
	rc_setreg(JREG_FIFOLEVEL, JBIT_FLUSHBUFFER);
	rc_setreg(JREG_COMMIRQ, JBIT_TIMERI);
	rc_setreg(JREG_BITFRAMING, 0);
	return status;
}

static int casc_anticollA(byte sel_code,byte bitcount,byte snr[])
{
	int status = STATUS_SUCCESS;

	byte  i;
	byte  complete = 0; /* signs end of anticollission loop */
	byte  rbits    = 0; /* number of total received bits */
	byte  nbits    = 0; /* */
	byte  nbytes   = 0; /* */
	byte  byteOffset;   /* stores offset for ID copy if uncomplete last byte was sent */

	if (fd <= 0) return -1;
	/* initialise relvant bytes in internal buffer */
	for(i=2;i<7;i++)
		SerBuffer[i] = 0x00;

	/* disable TxCRC and RxCRC */
	rc_modifyreg(JREG_TXMODE, 0, JBIT_CRCEN);
	rc_modifyreg(JREG_RXMODE, 0, JBIT_CRCEN);

	/* activate deletion of bits after coll */
	rc_setreg(JREG_COLL, 0);

	/* init parameters for anticollision */
	while(!complete && (status == STATUS_SUCCESS))
	{
		/* if there is a communication problem on the RF interface, bcnt
		could be larger than 32 - folowing loops will be defective. */
		if(bitcount > SINGLE_UID_LENGTH)
		{
			status = STATUS_INVALID_PARAMETER;
			continue;
		}

		/* prepare data length */
		nbits = (byte)(bitcount % BITS_PER_BYTE);
		nbytes = (byte)(bitcount / BITS_PER_BYTE);
		if(nbits)
			nbytes++;

		/* prepare data buffer */
		SerBuffer[0] = sel_code;
		SerBuffer[1] = (byte)(NVB_MIN_PARAMETER + ((bitcount / BITS_PER_BYTE) << UPPER_NIBBLE_SHIFT) + nbits);
		for(i=0;i<nbytes;i++)
			SerBuffer[2+i] = snr[i];   /* copy serial number to tranmit buffer */

		/* set TxLastBits and RxAlign to number of bits sent */
		rc_setreg(JREG_BITFRAMING, (byte)((nbits << UPPER_NIBBLE_SHIFT) | nbits));

		/* prepare data for common transceive */
		ResetInfo(Info);
		Info.nBytesToSend   = (byte)(nbytes + 2);

		set_timeoutus(10000);
		status = pcdcmd(JCMD_TRANSCEIVE, SerBuffer, (CmfInfo*)&Info);

		if(status == STATUS_COLLISION_ERROR || status == STATUS_SUCCESS)
		{
			/* store number of received data bits and bytes internaly */
			rbits = (byte)(Info.nBitsReceived + (Info.nBytesReceived << 3) - nbits);

			if((rbits + bitcount) > COMPLETE_UID_BITS)
			{
				status = STATUS_BITCOUNT_ERROR;
				continue;
			}

			/* increment number of bytes received if also some bits received */
			if(Info.nBitsReceived)
				Info.nBytesReceived++;

			/* reset offset for data copying */
			byteOffset = 0;
			/* if number of bits sent are not 0, write first received byte in last of sent */
			if(nbits)
			{   /* last byte transmitted and first byte received are the same */
				snr[nbytes - 1] |= SerBuffer[0];
				byteOffset++;
			}

			for(i=0;i<(4-nbytes);i++)
				snr[nbytes + i] = SerBuffer[i + byteOffset];

			if(status == STATUS_COLLISION_ERROR)
			{
				/* calculate new bitcount value */
				bitcount = (byte)(bitcount + rbits);
				status = STATUS_SUCCESS;
			} else
			{
				if((snr[0] ^ snr[1] ^ snr[2] ^ snr[3]) != SerBuffer[i + byteOffset])
				{
					status = STATUS_WRONG_UID_CHECKBYTE;
					continue;
				}
				complete=1;
			}
		}
	}

	/* clear RxAlign and TxLastbits */
	rc_setreg(JREG_BITFRAMING, 0);

	/* activate values after coll */
	rc_setreg(JREG_COLL, JBIT_VALUESAFTERCOLL);
	return status;
}
static int selectA(byte sel_code, byte *snr, byte *sak)
{
	int status = STATUS_SUCCESS;
	/* define local variables */
	byte i;
	if (fd <= 0) return -1;
	/* activate CRC */
	rc_modifyreg(JREG_TXMODE, 1, JBIT_CRCEN);
	rc_modifyreg(JREG_RXMODE, 1, JBIT_CRCEN);

	/* prepare data stream */
	SerBuffer[0] = sel_code;   /* command code */
	SerBuffer[1] = NVB_MAX_PARAMETER;       /* parameter */
	for(i=0;i<4;i++)
		SerBuffer[2+i] = snr[i];   /* serial numbner bytes 1 to 4 */
	SerBuffer[6] = (byte)(snr[0] ^ snr[1] ^ snr[2] ^ snr[3]);   /* serial number check byte */

	/* prepare data for common transceive */
	ResetInfo(Info);
	Info.nBytesToSend   = 0x07;
	set_timeoutus(2000);
	status = pcdcmd(JCMD_TRANSCEIVE, SerBuffer, (CmfInfo*)&Info);

	if(status == STATUS_SUCCESS)
	{
		if(Info.nBytesReceived == SAK_LENGTH && Info.nBitsReceived == 0)
			*sak = SerBuffer[0];
		else
			status = STATUS_BITCOUNT_ERROR;
	}
	return status;
}


static int value_operA(byte OperMode,byte addr,byte *value,byte trans_addr)
{
	int status = STATUS_SUCCESS;
	if (fd <= 0) return -1;
	ResetInfo(Info);
	SerBuffer[0] = OperMode;
	SerBuffer[1] = addr;
	Info.nBytesToSend   = 2;
	set_timeoutus(20000);
	status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);

	if (status != STATUS_IO_TIMEOUT)
	{
		if (Info.nBitsReceived != 4)
		{
			status = STATUS_BITCOUNT_ERROR;
		}
		else
		{
			SerBuffer[0] &= 0x0f;
			switch(SerBuffer[0])
			{
			case 0x00:
				status = STATUS_AUTHENT_ERROR;
				break;
			case 0x0a:
				status = STATUS_SUCCESS;
				break;
			case 0x01:
				status = STATUS_INVALID_FORMAT;
				break;
			default:
				status = STATUS_OTHER_ERROR;
				break;
			}
		}
	}

	if ( status == STATUS_SUCCESS)
	{
		set_timeoutus(10000);
		ResetInfo(Info);
		memcpy(SerBuffer,value,4);
		Info.nBytesToSend   = 4;
		status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);

		if (status == STATUS_IO_TIMEOUT||(status == MIFARE_DECREMENT && OperMode == MIFARE_DECREMENT))
		{
			status = STATUS_SUCCESS;
		}
		else
		{
			status = STATUS_OTHER_ERROR;
		}
	}
	if ( status == STATUS_SUCCESS)
	{
		ResetInfo(Info);
		SerBuffer[0] = MIFARE_TRANSFER;
		SerBuffer[1] = trans_addr;
		Info.nBytesToSend   = 2;
		status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);

		if (status & MIFARE_ACK_MASK)
		{
			status = STATUS_SUCCESS;
		}
		else
		{
			status = STATUS_OTHER_ERROR;
		}
	}
	return status;
}



//****************************************************
//JNI interface
//****************************************************

static int rfid_open(JNIEnv *env, jobject thiz,jstring port)
{
	byte RegVal;
	int count=0;
	const char* szport = env->GetStringUTFChars(port,0);
	if (open(szport) < 0) return -1;
	env->ReleaseStringUTFChars(port,szport);
	delay(200);
	RegVal= rc_getreg(JREG_CONTROL);
	printf("RegVal is %#x",RegVal);

	rc522_reset();



	/* disable Crypto1 bit*/
	rc_modifyreg(JREG_STATUS2, 0, JBIT_CRYPTO1ON);

	/* ADDIQ = 01b; FixIQ = 1; RFU = 0; TauRcv = 11b; TauSync = 01b */
	rc_setreg(JREG_DEMOD, 0x6D);

	/*RxGain = 4*/
	rc_setreg(JREG_RFCFG, 0x48);

	/* do settings common for all functions */
	rc_setreg(JREG_RXTRESHOLD, 0x55);    /* MinLevel = 5; CollLevel = 5 */

	rc_setreg(JREG_MODWIDTH, 0x26);      /* Modwidth = 0x26 */
	rc_setreg(JREG_GSN, 0xF0 | 0x04);     /* CWGsN = 0xF; ModGsN = 0x4 */

	/* Set the timer to auto mode, 5ms using operation control commands before HF is switched on to
	* guarantee Iso14443-3 compliance of Polling procedure
	*/
	set_timeoutus(5000);

	/* Activate the field  */
	rc_modifyreg(JREG_TXCONTROL, 1, JBIT_TX2RFEN | JBIT_TX1RFEN);

	/* start timer manually to check the initial waiting time */
	rc_modifyreg(JREG_CONTROL, 1, JBIT_TSTARTNOW);

	/*
	* After switching on the timer wait until the timer interrupt occures, so that
	* the field is on and the 5ms delay have been passed.
	*/
	do {
		RegVal = rc_getreg(JREG_COMMIRQ);
		delay(5);
	}
	while((!(RegVal & JBIT_TIMERI)) && count < 1000);


	/* Clear the status flag afterwards */
	rc_setreg(JREG_COMMIRQ, JBIT_TIMERI);

	/*
	* Reset timer 1 ms using operation control commands (AutoMode and Prescaler are the same)
	* set reload value
	*/
	set_timeoutus(5000);

	rc_setreg(JREG_WATERLEVEL, 0x1A);
	rc_setreg(JREG_TXSEL, 0x10);
	rc_setreg(JREG_RXSEL, 0x84);



	/* Activate receiver for communication
	The RcvOff bit and the PowerDown bit are cleared, the command is not changed. */
	rc_setreg(JREG_COMMAND, JCMD_IDLE);

	/* Set timeout for REQA, ANTICOLL, SELECT to 200us */
	set_timeoutus(2000);

	return (count < 1000 ? 0 : -1);

}

static void rfid_close(JNIEnv *env, jobject thiz)
{
	close();

}

static int rfid_haltA(JNIEnv *env, jobject thiz)
{
	int  status = STATUS_SUCCESS;
	/* initialise data buffer */
	SerBuffer[0] = HALTA_CMD;
	SerBuffer[1] = HALTA_PARAM;

	if (fd <= 0) return -1;
	ResetInfo(Info);
	Info.nBytesToSend   = HALTA_CMD_LENGTH;
	set_timeoutus(1000);
	status = pcdcmd(JCMD_TRANSCEIVE, SerBuffer, (CmfInfo*)&Info);

	if(status == STATUS_IO_TIMEOUT)
		status = STATUS_SUCCESS;
	return status;
}

static int rfid_requestA(JNIEnv *env, jobject thiz, byte req_code, byte atq[])
{
	char  status = STATUS_SUCCESS;

	if (fd <= 0) return -1;
	/************* initialize *****************/
	rc_modifyreg(JREG_STATUS2, 0, JBIT_CRYPTO1ON);  /* disable Crypto if activated before */
	rc_setreg(JREG_COLL, JBIT_VALUESAFTERCOLL);  //active values after coll
	rc_modifyreg(JREG_TXMODE, 0, JBIT_CRCEN);  //disable TxCRC and RxCRC
	rc_modifyreg(JREG_RXMODE, 0, JBIT_CRCEN);
	rc_setreg(JREG_BITFRAMING, REQUEST_BITS);

	/* set necessary parameters for transmission */
	ResetInfo(Info);
	SerBuffer[0] = req_code;
	Info.nBytesToSend   = 1;

	/* Set timeout for REQA, ANTICOLL, SELECT*/
	set_timeoutus(400);

	status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);
	if(status == STATUS_SUCCESS || status == STATUS_COLLISION_ERROR)
	{
		if(Info.nBytesReceived != ATQA_LENGTH || Info.nBitsReceived != 0x00)
		{
			status = STATUS_PROTOCOL_ERROR;
		}
		else
		{
			memcpy(atq,SerBuffer,2);
		}
	}
	else
	{   /* reset atqa parameter */
		atq[0] = 0x00;
		atq[1] = 0x00;
	}
	//RcSetReg(JREG_BITFRAMING, 0);
	return status;
}

static int rfid_anticoll_selectA(JNIEnv *env, jobject thiz, byte bcnt, byte snr[], byte plen[])
{
	byte i;
	int status=STATUS_SUCCESS;
	byte length, casc_code, length_in,sak,tmpSnr[12];
	if (fd <= 0) return -1;
	memcpy(tmpSnr, snr, 12);
	length_in = bcnt;
	/* do loop for max. cascade level */
	for(i=0;i<MAX_CASCADE_LEVELS;i++)
	{
		if(length_in)
		{
			if(length_in > SINGLE_UID_LENGTH)
			{
				length = SINGLE_UID_LENGTH;
				length_in -= SINGLE_UID_LENGTH;
			}
			else
			{
				length = length_in;
				length_in = 0;
			}
		}
		else
		{
			length = 0;
		}

		switch(i)
		{
		case 0:  casc_code = SELECT_CASCADE_LEVEL_1;
			break;
		case 1:  casc_code = SELECT_CASCADE_LEVEL_2;
			break;
		case 2:  casc_code = SELECT_CASCADE_LEVEL_3;
			break;
		default:
			break;
		}

		if(length != SINGLE_UID_LENGTH && status == STATUS_SUCCESS)
			/* do anticollission with selected level */
			status = casc_anticollA(casc_code,
			length,
			tmpSnr + i * 4);
		if(status != STATUS_SUCCESS)
		{
			return status;
		}

		/* select 1st cascade level uid */
		status = selectA(casc_code, tmpSnr + i * 4, &sak);
		if(status != STATUS_SUCCESS)
		{
			return status;
		}

		/* increase number of received bits in parameter */
		bcnt = (byte)(SINGLE_UID_LENGTH * (i + 1)); //the actually length of the UID, you can return it.

		/* check if cascade bit is set */
		if(!(sak & CASCADE_BIT))
		{
			break;
		}
	}
	switch(i)
	{
	case 0:
		memcpy(snr, tmpSnr,4);
		*plen = 4;
		break;
	case 1:
		memcpy(snr, tmpSnr+1,3);
		memcpy(snr+3, tmpSnr+4,4);
		*plen = 7;
		break;
	case 2:
		memcpy(snr, tmpSnr+1,3);
		memcpy(snr+3, tmpSnr+5,3);
		memcpy(snr+6, tmpSnr+8,4);
		*plen = 8;
		break;
	default:
		break;
	}
	return status;
}

static int rfid_authenticationA(JNIEnv *env, jobject thiz, byte auth_mode,
							   byte key[],
							   byte snr[],
							   byte addr)
{
	int status;
	byte i = 0, RegVal;

	if (fd <= 0) return -1;
	ResetInfo(Info);

	SerBuffer[0] = auth_mode;      //key A or key B
	SerBuffer[1] = addr;           //address to authentication
	memcpy(SerBuffer+2,key,6);     //6 bytes key
	memcpy(SerBuffer+8,snr,4);     //4 bytes UID
	Info.nBytesToSend = 12;       //length
	set_timeoutus(2000);
	status = pcdcmd(JCMD_AUTHENT, SerBuffer, (CmfInfo*)&Info);
	if(status == STATUS_SUCCESS)
	{
		RegVal = rc_getreg(JREG_STATUS2);
		if((RegVal & 0x0f) != 0x08)
			status = STATUS_AUTHENT_ERROR;
	}
	return status;
}

static int rfid_readA(JNIEnv *env, jobject thiz, byte addr, byte dat[])
{
	int status = STATUS_SUCCESS;
	char tmp    = 0;

	if (fd <= 0) return -1;
	ResetInfo(Info);
	SerBuffer[0] = MIFARE_READ;
	SerBuffer[1] = addr;
	Info.nBytesToSend   = 2;
	set_timeoutus(10000);
	status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);

	if (status != STATUS_SUCCESS)
	{
		if (status != STATUS_IO_TIMEOUT )     // no timeout occured
		{
			if (Info.nBitsReceived == 4)
			{
				SerBuffer[0] &= 0x0f;
				if ((SerBuffer[0] & 0x0a) == 0)
				{
					status = STATUS_AUTHENT_ERROR;
				}
				else
				{
					status = STATUS_INVALID_FORMAT;
				}
			}
		}
		memset(dat,0,16);
	}
	else   // Response Processing
	{
		if (Info.nBytesReceived != 16)
		{
			status = STATUS_ACCESS_DENIED;
			memset(dat,0,16);
		}
		else
		{
			memcpy(dat,SerBuffer,16);
		}
	}
	return status;
}

static int rfid_writeA(JNIEnv *env, jobject thiz, byte addr, byte dat[])
{
	int status = STATUS_SUCCESS;
	if (fd <= 0) return -1;
	ResetInfo(Info);
	SerBuffer[0] = MIFARE_WRITE;
	SerBuffer[1] = addr;
	Info.nBytesToSend   = 2;
	set_timeoutus(20000);
	status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);

	if (status != STATUS_IO_TIMEOUT)
	{
		if (Info.nBitsReceived != 4)
		{
			status = STATUS_BITCOUNT_ERROR;
		}
		else
		{
			SerBuffer[0] &= 0x0f;
			if ((SerBuffer[0] & 0x0a) == 0)
			{
				status = STATUS_AUTHENT_ERROR;
			}
			else
			{
				if (SerBuffer[0] == 0x0a)
				{
					status = STATUS_SUCCESS;
				}
				else
				{
					status = STATUS_INVALID_FORMAT;
				}
			}
		}
	}

	if ( status == STATUS_SUCCESS)
	{

		set_timeoutus(8000);

		ResetInfo(Info);
		memcpy(SerBuffer,dat,16);
		Info.nBytesToSend   = 16;
		status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);

		if (status & 0x80)
		{
			status = STATUS_IO_TIMEOUT;
		}
		else
		{
			if (Info.nBitsReceived != 4)
			{
				status = STATUS_BITCOUNT_ERROR;
				printf("status is %#x",status);
			}
			else
			{
				SerBuffer[0] &= 0x0f;
				if ((SerBuffer[0] & 0x0a) == 0)
				{
					status = STATUS_ACCESS_DENIED;
				}
				else
				{
					if (SerBuffer[0] == 0x0a)
					{
						status = STATUS_SUCCESS;
					}
					else
					{
						status = STATUS_INVALID_FORMAT;
					}
				}
			}
		}
	}

	return status;
}

static int rfid_change_type(JNIEnv *env, jobject thiz, byte type)
{
	byte txAsk;
	byte txFraming;
	byte RegVal;
	byte crcPreset;

	if (fd <= 0) return -1;
	rc_setreg(JREG_AUTOTEST, 0x40);
	if (type == ISO1443_TYPEA) {
		txAsk = 0x40; /* 100% */
		txFraming = 0x00;
		crcPreset = 1; /* 6363h */
	} else {
		txAsk = 0x00;
		txFraming = 0x03;
		crcPreset = 3; /* FFFFh */
		rc_setreg(JREG_TYPEB, 0x02);
	}

	RegVal = rc_getreg(JREG_MODE);
	RegVal &= ~0x03;
	rc_setreg(JREG_MODE, RegVal|crcPreset);

	rc_setreg(JREG_TXASK, txAsk);
	/* do not touch bits: InvMod in register TxMode */
	RegVal = rc_getreg(JREG_TXMODE);
	RegVal = (byte)(RegVal & JBIT_INVMOD);
	RegVal = (byte)(RegVal | JBIT_CRCEN | (RCO_VAL_RF106K << RC522_SPEED_SHL_VALUE) | txFraming);
	/* TxCRCEn = 1; TxSpeed = x; InvMod, TXMix = 0; TxFraming = xx */
	rc_setreg(JREG_TXMODE, RegVal);

	/* do not touch bits: RxNoErr in register RxMode */
	RegVal = rc_getreg(JREG_RXMODE);
	RegVal = (byte)(RegVal & JBIT_RXNOERR);
	RegVal = (byte)(RegVal | JBIT_CRCEN | (RCO_VAL_RF106K << RC522_SPEED_SHL_VALUE) | txFraming);
	/* RxCRCEn = 1; RxSpeed = x; RxNoErr, RxMultiple = 0; TxFraming = xx */
	rc_setreg(JREG_RXMODE, RegVal);
	return 0;
}

static int rfid_init_blockA(JNIEnv *env, jobject thiz, byte addr,byte *value)
{
	byte tmp[16],i;
	int status = STATUS_SUCCESS;
	if (fd <= 0) return -1;
	for(i=0;i<4;i++)
	{
		tmp[i]=value[i];
		tmp[i+4]=255-value[i];
		tmp[i+8]=value[i];
	}
	tmp[12]=addr;
	tmp[13]=255-addr;
	tmp[14]=tmp[12];
	tmp[15]=tmp[13];
	status=rfid_writeA(env,thiz,addr,tmp);
	return status;
}

#define ISO14443_3_APF 0x05
#define ISO14443_3_AFIALL 0x00
static int rfid_anticoll_requestB(JNIEnv *env, jobject thiz, byte req_code, byte** atq, byte *ptags)
{
	char  status = STATUS_SUCCESS;
	byte slotExp = 0;
	byte slotNum = 1;
	byte maxTags = *ptags;
	byte colli = 0;
	byte maxColliTry = 3;
	byte i;
	*ptags = 0;


	if (fd <= 0) return -1;
	/************* initialize *****************/
	rc_modifyreg(JREG_STATUS2, 0, JBIT_CRYPTO1ON);  /* disable Crypto if activated before */
	rc_setreg(JREG_COLL, JBIT_VALUESAFTERCOLL);  //active values after coll
	rc_modifyreg(JREG_TXMODE, 1, JBIT_CRCEN);  //enable TxCRC and RxCRC
	rc_modifyreg(JREG_RXMODE, 1, JBIT_CRCEN);
	rc_setreg(JREG_BITFRAMING, 0);

	while (maxColliTry--) {
		/* set necessary parameters for transmission */
		ResetInfo(Info);
		SerBuffer[0] = ISO14443_3_APF;
		SerBuffer[1] = ISO14443_3_AFIALL;
		SerBuffer[2] = req_code | slotExp;
		Info.nBytesToSend   = 3;
		/* Set timeout for REQB*/
		set_timeoutus(40000);
		status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);
		for (i=0; i<slotNum; i++) {
			if (i != 0) {
				/* SlotMarker*/
				ResetInfo(Info);
				SerBuffer[0] = (i<<4)|0x05;
				Info.nBytesToSend   = 1;
				set_timeoutus(40000);
				status = pcdcmd(JCMD_TRANSCEIVE,SerBuffer,(CmfInfo*)&Info);
			}
			if(status==STATUS_SUCCESS && Info.nBytesReceived==12) {
				memcpy(atq[(*ptags)++], SerBuffer, 12);
				if (*ptags >= maxTags) {
					return status;
				}
			} else if (status == STATUS_COLLISION_ERROR) {
				colli++;
			}
		}
		if (colli==0) {
			break;
		}
		slotExp = 3;
		slotNum = 8;
	}
	return status;
}


//=======================================================================

static const char *classPathName = "android/Rfid/RfidJNI";

static JNINativeMethod methods[] = {
  {"rfid_open", "(Ljava/lang/String;)I", (void*)rfid_open },
  {"rfid_close", "()V", (void*)rfid_close },
  {"rfid_haltA", "()I", (void*)rfid_haltA },
  {"rfid_requestA", "(B[B)I", (void*)rfid_requestA },
  {"rfid_anticoll_selectA", "(B[B[B)I", (void*)rfid_anticoll_selectA },
  {"rfid_authenticationA", "(B[B[BB)I", (void*)rfid_authenticationA },
  {"rfid_readA", "(B[B)I", (void*)rfid_readA },
  {"rfid_writeA", "(B[B)I", (void*)rfid_writeA },
  {"rfid_change_type", "(B)I", (void*)rfid_change_type },
  {"rfid_init_blockA", "(B[B)I", (void*)rfid_init_blockA },
  {"rfid_anticoll_requestB", "(B[[B[B)I", (void*)rfid_anticoll_requestB },
};


// ----------------------------------------------------------------------------

/*
 * This is called by the VM when the shared library is first loaded.
 */
 
jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    jint result = -1;
    JNIEnv* env = NULL;
    jclass clazz;
    LOGI("JNI_OnLoad");

    if (vm->GetEnv((void**)&env, JNI_VERSION_1_4) != JNI_OK) {
        LOGE("ERROR: GetEnv failed");
        goto bail;
    }
    
    result = JNI_VERSION_1_4;
    clazz = env->FindClass(classPathName);
    env->RegisterNatives(clazz,methods, sizeof(methods) / sizeof(methods[0]));
    
bail:
    return result;
}
