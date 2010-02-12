/*
 * Copyright (c) 2003 Hiroki Mori. All rights reserved.
 * Copyright (c) 1998-2001 Apple Computer, Inc. All rights reserved.
 *
* IMPORTANT:  This Apple software is supplied to you by Apple Computer,
 * Inc. ("Apple") in consideration of your agreement to the following
 * terms, and your use, installation, modification or redistribution of
 * this Apple software constitutes acceptance of these terms.  If you do
 * not agree with these terms, please do not use, install, modify or
 * redistribute this Apple software.
 *
 * In consideration of your agreement to abide by the following terms, and
 * subject to these terms, Apple grants you a personal, non-exclusive
 * license, under Apple's copyrights in this original Apple software (the
 * "Apple Software"), to use, reproduce, modify and redistribute the Apple
 * Software, with or without modifications, in source and/or binary forms;
 * provided that if you redistribute the Apple Software in its entirety and
 * without modifications, you must retain this notice and the following
 * text and disclaimers in all such redistributions of the Apple Software.
 * Neither the name, trademarks, service marks or logos of Apple Computer,
 * Inc. may be used to endorse or promote products derived from the Apple
 * Software without specific prior written permission from Apple.  Except
 * as expressly stated in this notice, no other rights or licenses, express
 * or implied, are granted by Apple herein, including but not limited to
 * any patent rights that may be infringed by your derivative works or by
 * other works in which the Apple Software may be incorporated.
 *
 * The Apple Software is provided by Apple on an "AS IS" basis.  APPLE
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
 * THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND
 * OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.
 *
 * IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
 * MODIFICATION AND/OR DISTRIBUTION OF THE APPLE SOFTWARE, HOWEVER CAUSED
 * AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
 * STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
 
	/* BelkinF5U103Driver.cpp - MacOSX implementation of		*/
	/* USB Communication Device Class (CDC) Driver.			*/

#include <machine/limits.h>			/* UINT_MAX */
#include <libkern/OSByteOrder.h>

#include <IOKit/assert.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMessage.h>

#include <IOKit/usb/IOUSBBus.h>
#include <IOKit/usb/IOUSBNub.h>
#include <IOKit/usb/IOUSBDevice.h>
#include <IOKit/usb/IOUSBPipe.h>
#include <IOKit/usb/USB.h>
#include <IOKit/usb/IOUSBInterface.h>

#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/IOSerialDriverSync.h>
#include <IOKit/serial/IOModemSerialStreamSync.h>
#include <IOKit/serial/IORS232SerialStreamSync.h>

#include <UserNotification/KUNCUserNotifications.h>

#include "BelkinF5U103Driver.h"

#define MIN_BAUD (50 << 1)


	static globals		g;	/**** Instantiate the globals ****/

#define super IOSerialDriverSync

	OSDefineMetaClassAndStructors( BelkinF5U103Driver, IOSerialDriverSync )	;

/****************************************************************************************************/
//
//		Function:	Asciify
//
//		Inputs:		i - the nibble
//
//		Outputs:	return byte - ascii byte
//
//		Desc:		Converts to ascii. 
//
/****************************************************************************************************/
 
static UInt8 Asciify(UInt8 i)
{

	i &= 0xF;
	if ( i < 10 )
		 return( '0' + i );
	else return( 55  + i );
	
}/* end Asciify */

#if USE_ELG
/****************************************************************************************************/
//
//		Function:	AllocateEventLog
//
//		Inputs:		size - amount of memory to allocate
//
//		Outputs:	None
//
//		Desc:		Allocates the event log buffer
//
/****************************************************************************************************/

static void AllocateEventLog( UInt32 size )
{
	if ( g.evLogBuf )
		return;

	g.evLogFlag = 0;            /* assume insufficient memory   */
	g.evLogBuf = (UInt8*)IOMalloc( size );
	if ( !g.evLogBuf )
	{
		kprintf( "probe - BelkinF5U103Driver evLog allocation failed " );
		return;
	}

	bzero( g.evLogBuf, size );
	g.evLogBufp	= g.evLogBuf;
	g.evLogBufe	= g.evLogBufp + kEvLogSize - 0x20; // ??? overran buffer?
	g.evLogFlag  = 0xFEEDBEEF;	// continuous wraparound
//	g.evLogFlag  = 'step';		// stop at each ELG
//	g.evLogFlag  = 0x0333;		// any nonzero - don't wrap - stop logging at buffer end

	IOLog( "AllocateEventLog - &globals=%8x buffer=%8x", (unsigned int)&g, (unsigned int)g.evLogBuf );

    return;
	
}/* end AllocateEventLog */

/****************************************************************************************************/
//
//		Function:	EvLog
//
//		Inputs:		a - anything, b - anything, ascii - 4 charater tag, str - any info string			
//
//		Outputs:	None
//
//		Desc:		Writes the various inputs to the event log buffer
//
/****************************************************************************************************/

static void EvLog( UInt32 a, UInt32 b, UInt32 ascii, char* str )
{
	register UInt32		*lp;           /* Long pointer      */
	mach_timespec_t		time;

	if ( g.evLogFlag == 0 )
		return;

	IOGetTime( &time );

    lp = (UInt32*)g.evLogBufp;
    g.evLogBufp += 0x10;

    if ( g.evLogBufp >= g.evLogBufe )       /* handle buffer wrap around if any */
    {    g.evLogBufp  = g.evLogBuf;
        if ( g.evLogFlag != 0xFEEDBEEF )    // make 0xFEEDBEEF a symbolic ???
            g.evLogFlag = 0;                /* stop tracing if wrap undesired   */
    }

        /* compose interrupt level with 3 byte time stamp:  */

	*lp++ = (g.intLevel << 24) | ((time.tv_nsec >> 10) & 0x003FFFFF);   // ~ 1 microsec resolution
    *lp++ = a;
    *lp++ = b;
    *lp   = ascii;

    if( g.evLogFlag == 'step' )
	{	static char	code[ 5 ] = {0,0,0,0,0};
		*(UInt32*)&code = ascii;
		IOLog( "%8x BelkinF5U103Driver: %8x %8x %s\n", time.tv_nsec>>10, (unsigned int)a, (unsigned int)b, code );
	}

    return;
	
}/* end EvLog */
#endif // USE_ELG

#if LOG_DATA
#define dumplen		32		// Set this to the number of bytes to dump and the rest should work out correct

#define buflen		((dumplen*2)+dumplen)+3
#define Asciistart	(dumplen*2)+3

/****************************************************************************************************/
//
//		Function:	USBLogData
//
//		Inputs:		Dir - direction, Count - number of bytes, buf - the data
//
//		Outputs:	None
//
//		Desc:		Puts the data in the log. 
//
/****************************************************************************************************/

static void USBLogData(UInt8 Dir, UInt32 Count, char *buf)
{
	UInt8		wlen, i, Aspnt, Hxpnt;
	UInt8		wchr;
	char		LocBuf[buflen+1];

	for ( i=0; i<=buflen; i++ )
	{
		LocBuf[i] = 0x20;
	}
	LocBuf[i] = 0x00;
	
	if ( Dir == kUSBIn )
	{
		IOLog( "BelkinF5U103Driver: USBLogData - Read Complete, size = %8x\n", Count );
	} else {
		if ( Dir == kUSBOut )
		{
			IOLog( "BelkinF5U103Driver: USBLogData - Write, size = %8x\n", Count );
		} else {
			if ( Dir == kUSBAnyDirn )
			{
				IOLog( "BelkinF5U103Driver: USBLogData - Other, size = %8x\n", Count );
			}
		}			
	}

	if ( Count > dumplen )
	{
		wlen = dumplen;
	} else {
		wlen = Count;
	}
	
	if ( wlen > 0 )
	{
		Aspnt = Asciistart;
		Hxpnt = 0;
		for ( i=1; i<=wlen; i++ )
		{
			wchr = buf[i-1];
			LocBuf[Hxpnt++] = Asciify( wchr >> 4 );
			LocBuf[Hxpnt++] = Asciify( wchr );
			if (( wchr < 0x20) || (wchr > 0x7F )) 		// Non printable characters
			{
				LocBuf[Aspnt++] = 0x2E;					// Replace with a period
			} else {
				LocBuf[Aspnt++] = wchr;
			}
		}
		LocBuf[(wlen + Asciistart) + 1] = 0x00;
		IOLog( LocBuf );
		IOLog( "\n" );
		IOSleep(Sleep_Time);							// Try and keep the log from overflowing
	} else {
		IOLog( "BelkinF5U103Driver: USBLogData - No data, Count=0\n" );
	}
	
}/* end USBLogData */
#endif // LOG_DATA

/* QueuePrimatives	*/

/****************************************************************************************************/
//
//		Function:	AddBytetoQueue
//
//		Inputs:		Queue - the queue to be added to
//
//		Outputs:	Value - Byte to be added, Queue status - full or no error
//
//		Desc:		Add a byte to the circular queue.
//
/****************************************************************************************************/

QueueStatus	BelkinF5U103Driver::AddBytetoQueue( CirQueue *Queue, char Value )
{
	
		/* Check to see if there is space by comparing the next pointer,	*/
		/* with the last, If they match we are either Empty or full, so		*/
		/* check the InQueue of being zero.									*/

    if ( (Queue->NextChar == Queue->LastChar) && Queue->InQueue )
        return queueFull;

	*Queue->NextChar++ = Value;
	Queue->InQueue++;

		/* Check to see if we need to wrap the pointer.	*/
		
	if ( Queue->NextChar >= Queue->End )
		 Queue->NextChar =  Queue->Start;

    return queueNoError;
	
}/* end AddBytetoQueue */

/****************************************************************************************************/
//
//		Function:	GetBytetoQueue
//
//		Inputs:		Queue - the queue to be removed from
//
//		Outputs:	Value - where to put the byte, Queue status - empty or no error
//
//		Desc:		Remove a byte from the circular queue.
//
/****************************************************************************************************/

QueueStatus	BelkinF5U103Driver::GetBytetoQueue( CirQueue *Queue, UInt8 *Value )
{

		/* Check to see if the queue has something in it.	*/
		
    if ( (Queue->NextChar == Queue->LastChar) && !Queue->InQueue )
        return queueEmpty;

	*Value = *Queue->LastChar++;
	Queue->InQueue--;

		/* Check to see if we need to wrap the pointer.	*/
	if ( Queue->LastChar >= Queue->End )
		 Queue->LastChar =  Queue->Start;

    return queueNoError;
	
}/* end GetBytetoQueue */

/****************************************************************************************************/
//
//		Function:	InitQueue
//
//		Inputs:		Queue - the queue to be initialized, Buffer - the buffer, size - length of buffer
//
//		Outputs:	Queue status - queueNoError.
//
//		Desc:		Pass a buffer of memory and this routine will set up the internal data structures.
//
/****************************************************************************************************/

QueueStatus	BelkinF5U103Driver::InitQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size )
{
    Queue->Start	= Buffer;
    Queue->End		= (UInt8*)((size_t)Buffer + Size);
    Queue->Size		= Size;
    Queue->NextChar	= Buffer;
    Queue->LastChar	= Buffer;
    Queue->InQueue	= 0;

    IOSleep( 1 );
	
    return queueNoError ;
	
}/* end InitQueue */

/****************************************************************************************************/
//
//		Function:	CloseQueue
//
//		Inputs:		Queue - the queue to be closed
//
//		Outputs:	Queue status - queueNoError.
//
//		Desc:		Clear out all of the data structures.
//
/****************************************************************************************************/

QueueStatus	BelkinF5U103Driver::CloseQueue( CirQueue *Queue )
{

    Queue->Start	= 0;
    Queue->End		= 0;
    Queue->NextChar	= 0;
    Queue->LastChar	= 0;
    Queue->Size		= 0;

    return queueNoError;
	
}/* end CloseQueue */

/****************************************************************************************************/
//
//		Function:	AddtoQueue
//
//		Inputs:		Queue - the queue to be added to, Buffer - data to add, Size - length of data
//
//		Outputs:	BytesWritten - Number of bytes actually put in the queue.
//
//		Desc:		Add an entire buffer to the queue.
//
/****************************************************************************************************/

size_t BelkinF5U103Driver::AddtoQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size )
{
    size_t		BytesWritten = 0;

    while ( FreeSpaceinQueue( Queue ) && (Size > BytesWritten) )
	{
        AddBytetoQueue( Queue, *Buffer++ );
        BytesWritten++;
    }

    return BytesWritten;
	
}/* end AddtoQueue */

/****************************************************************************************************/
//
//		Function:	RemovefromQueue
//
//		Inputs:		Queue - the queue to be removed from, Size - size of buffer
//
//		Outputs:	Buffer - Where to put the data, BytesReceived - Number of bytes actually put in Buffer.
//
//		Desc:		Get a buffers worth of data from the queue.
//
/****************************************************************************************************/

size_t BelkinF5U103Driver::RemovefromQueue( CirQueue *Queue, UInt8 *Buffer, size_t MaxSize )
{
	size_t		BytesReceived = 0;
	UInt8		Value;

	while( (MaxSize > BytesReceived) && (GetBytetoQueue(Queue, &Value) == queueNoError) ) 

//  while( (GetBytetoQueue( Queue, &Value ) == queueNoError ) && (MaxSize >= BytesReceived) )
	{
        *Buffer++ = Value;
        BytesReceived++;
    }/* end while */

    return BytesReceived;
	
}/* end RemovefromQueue */

/****************************************************************************************************/
//
//		Function:	FreeSpaceinQueue
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	Return Value - Free space left
//
//		Desc:		Return the amount of free space left in this buffer.
//
/****************************************************************************************************/

size_t BelkinF5U103Driver::FreeSpaceinQueue( CirQueue *Queue )
{
    size_t				retVal;

    retVal = Queue->Size - Queue->InQueue;

    return retVal;
	
}/* end FreeSpaceinQueue */

/****************************************************************************************************/
//
//		Function:	UsedSpaceinQueue
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	UsedSpace - Amount of data in buffer
//
//		Desc:		Return the amount of data in this buffer.
//
/****************************************************************************************************/

size_t BelkinF5U103Driver::UsedSpaceinQueue( CirQueue *Queue )
{
    return Queue->InQueue;
	
}/* end UsedSpaceinQueue */

/****************************************************************************************************/
//
//		Function:	GetQueueSize
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	QueueSize - The size of the queue.
//
//		Desc:		Return the total size of the queue.
//
/****************************************************************************************************/

size_t BelkinF5U103Driver::GetQueueSize( CirQueue *Queue )
{
    return Queue->Size;
	
}/* end GetQueueSize */

/****************************************************************************************************/
//
//		Function:	GetQueueStatus
//
//		Inputs:		Queue - the queue to be queried
//
//		Outputs:	Queue status - full, empty or no error
//
//		Desc:		Returns the status of the circular queue.
//
/****************************************************************************************************/

QueueStatus BelkinF5U103Driver::GetQueueStatus( CirQueue *Queue )
{
    if ( (Queue->NextChar == Queue->LastChar) && Queue->InQueue )
        return queueFull;
    else if ( (Queue->NextChar == Queue->LastChar) && !Queue->InQueue )
        return queueEmpty;
		
    return queueNoError ;
	
}/* end GetQueueStatus */

/****************************************************************************************************/
//
//		Function:	CheckQueues
//
//		Inputs:		port - the port to check
//
//		Outputs:	None
//
//		Desc:		Checks the various queue's etc and manipulates the state(s) accordingly
//
/****************************************************************************************************/

void BelkinF5U103Driver::CheckQueues( PortInfo_t *port )
{
	unsigned long	Used;
	unsigned long	Free;
	unsigned long	QueuingState;
	unsigned long	DeltaState;

	// Initialise the QueueState with the current state.
	QueuingState = readPortState( port );

		/* Check to see if there is anything in the Transmit buffer. */
    Used = UsedSpaceinQueue( &port->TX );
    Free = FreeSpaceinQueue( &port->TX );
//	ELG( Free, Used, 'CkQs', "CheckQueues" );
    if ( Free == 0 )
	{
        QueuingState |=  PD_S_TXQ_FULL;
        QueuingState &= ~PD_S_TXQ_EMPTY;
    }
    else if ( Used == 0 )
	{
        QueuingState &= ~PD_S_TXQ_FULL;
        QueuingState |=  PD_S_TXQ_EMPTY;
    }
    else
	{
        QueuingState &= ~PD_S_TXQ_FULL;
        QueuingState &= ~PD_S_TXQ_EMPTY;
    }

    	/* Check to see if we are below the low water mark.	*/
    if ( Used < port->TXStats.LowWater )
         QueuingState |=  PD_S_TXQ_LOW_WATER;
    else QueuingState &= ~PD_S_TXQ_LOW_WATER;

    if ( Used > port->TXStats.HighWater )
         QueuingState |= PD_S_TXQ_HIGH_WATER;
    else QueuingState &= ~PD_S_TXQ_HIGH_WATER;


		/* Check to see if there is anything in the Receive buffer.	*/
    Used = UsedSpaceinQueue( &port->RX );
    Free = FreeSpaceinQueue( &port->RX );

    if ( Free == 0 )
	{
        QueuingState |= PD_S_RXQ_FULL;
        QueuingState &= ~PD_S_RXQ_EMPTY;
    }
    else if ( Used == 0 )
	{
        QueuingState &= ~PD_S_RXQ_FULL;
        QueuingState |= PD_S_RXQ_EMPTY;
    }
    else
	{
        QueuingState &= ~PD_S_RXQ_FULL;
        QueuingState &= ~PD_S_RXQ_EMPTY;
    }

    	/* Check to see if we are below the low water mark.	*/
    if ( Used < port->RXStats.LowWater )
         QueuingState |= PD_S_RXQ_LOW_WATER;
    else QueuingState &= ~PD_S_RXQ_LOW_WATER;

    if ( Used > port->RXStats.HighWater )
         QueuingState |= PD_S_RXQ_HIGH_WATER;
    else QueuingState &= ~PD_S_RXQ_HIGH_WATER;

		/* Figure out what has changed to get mask.*/
    DeltaState = QueuingState ^ readPortState( port );
    changeState( port, QueuingState, DeltaState );
	
	return;
	
}/* end CheckQueues */

/* end of QueuePrimatives */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::start
//
//		Inputs:		provider - my provider
//
//		Outputs:	Return code - true (it's me), false (sorry it probably was me, but I can't configure it)
//
//		Desc:		This is called once it has beed determined I'm probably the best 
//					driver for this device.
//
/****************************************************************************************************/

bool BelkinF5U103Driver::start( IOService *provider )
{
	UInt8	configs;	// number of device configurations
	UInt8		i;

	g.evLogBufp = NULL;
	
	for (i=0; i<numberofPorts; i++)
	{
		fPorts[i] = NULL;
	}

        fSessions = 0;
#if USE_ELG
    AllocateEventLog( kEvLogSize );
    ELG( &g, g.evLogBufp, 'USBM', "BelkinF5U103Driver::init - event logging set up." );

	waitForService( resourceMatching( "kdp" ) );
#endif /* USE_ELG */

    ELG( this, provider, 'strt', "BelkinF5U103Driver::start - this, provider." );
	if( !super::start( provider ) )
	{
		IOLogIt( 0, 0, 'SS--', "BelkinF5U103Driver::start - super failed" );
		return false;
	}

	/* Get my USB device provider - the device	*/

        fpDevice = OSDynamicCast( IOUSBDevice, provider );
        if( !fpDevice )
        {
		IOLogIt( 0, 0, 'Dev-', "BelkinF5U103Driver::start - provider invalid" );
		stop( provider );
		return false;
	}

	/* Let's see if we have any configurations to play with */
		
	configs = fpDevice->GetNumConfigurations();
	if ( configs < 1 )
	{
		IOLogIt( 0, 0, 'Cfg-', "BelkinF5U103Driver::start - no configurations" );
		stop( provider );
		return false;
	}
	
	/* Now take control of the device and configure it */
		
	if (!fpDevice->open(this))
	{
		IOLogIt( 0, 0, 'Opn-', "BelkinF5U103Driver::start - unable to open device" );
		stop( provider );
		return false;
	}
	
	if ( configureDevice(configs) )
	{
		ELG( 0, 0, 'Nub+', "BelkinF5U103Driver::start - successful and IOModemSerialStreamSync created" );
		return true;
	}
	
	IOLogIt( 0, 0, 'Nub-', "BelkinF5U103Driver::start - failed" );
	fpDevice->close(this);
	stop( provider );
	
	return false;
	
}/* end start */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::commReadComplete
//
//		Inputs:		obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//																					(whose idea was that?)
//
//		Outputs:	None
//
//		Desc:		Interrupt pipe (Comm interface) read completion routine
//
/****************************************************************************************************/

void BelkinF5U103Driver::commReadComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	BelkinF5U103Driver	*me = (BelkinF5U103Driver*)obj;
	PortInfo_t 			*port = (PortInfo_t*)param;
	IOReturn	ior;
	UInt32		dLen;
	UInt16		*tState;
	UInt32		tempS;

	if ( rc == kIOReturnSuccess )	/* If operation returned ok:	*/
	{
		dLen = COMM_BUFF_SIZE - remaining;
		ELG( rc, dLen, 'CRC+', "commReadComplete" );
		
			/* Now look at the state stuff */
		LogData( kUSBAnyDirn, dLen, me->fCommPipeBuffer );
		
#if 0
		if ((dLen > 7) && (me->fCommPipeBuffer[1] == kUSBSERIAL_STATE))
		{
			tState = (UInt16 *)&me->fCommPipeBuffer[8];
			me->fModemState = USBToHostWord(*tState);
			ELG( 0, me->fModemState, 'kUSS', "kUSBSERIAL_STATE" );
			
			tempS = me->fModemState & kUSBDCD;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_DCD, PD_RS232_S_DCD );
			} else {
				me->changeState( port, 0, PD_RS232_S_DCD );
			}
			
			tempS = me->fModemState & kUSBDSR;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_DSR, PD_RS232_S_DSR );
			} else {
				me->changeState( port, 0, PD_RS232_S_DSR );
			}
			
			tempS = me->fModemState & kUSBbBreak;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_BRK, PD_RS232_S_BRK );
			} else {
				me->changeState( port, 0, PD_RS232_S_BRK );
			}
			
			tempS = me->fModemState & kUSBbRingSignal;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_RNG, PD_RS232_S_RNG );
			} else {
				me->changeState( port, 0, PD_RS232_S_RNG );
			}
		}
#else
		if (dLen == 4)
                {
                        ELG( 0, me->fCommPipeBuffer[2], 'kUSS', "kUSBSERIAL_STATE LSR" );
                        ELG( 0, me->fCommPipeBuffer[3], 'kUSS', "kUSBSERIAL_STATE MSR" );

			tempS = me->fCommPipeBuffer[3] & UBSA_MSR_DCD;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_DCD, PD_RS232_S_DCD );
			} else {
				me->changeState( port, 0, PD_RS232_S_DCD );
			}

			tempS = me->fCommPipeBuffer[3] & UBSA_MSR_DSR;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_DSR, PD_RS232_S_DSR );
			} else {
				me->changeState( port, 0, PD_RS232_S_DSR );
			}

			tempS = me->fCommPipeBuffer[2] & UBSA_LSR_BI;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_BRK, PD_RS232_S_BRK );
			} else {
				me->changeState( port, 0, PD_RS232_S_BRK );
			}
			
			tempS = me->fCommPipeBuffer[3] & UBSA_MSR_RI;
			if (tempS)
			{
				me->changeState( port, PD_RS232_S_RNG, PD_RS232_S_RNG );
			} else {
				me->changeState( port, 0, PD_RS232_S_RNG );
			}
                }
#endif		
                /* Queue the next read:	*/
	
		ior = me->fpCommPipe->Read( me->fpCommPipeMDP, &me->fCommCompletionInfo, NULL );
		if ( ior == kIOReturnSuccess )
			return;
	}

        /* Read returned with error OR next interrupt read failed to be queued:	*/

        ELG( 0, rc, 'CRC-', "commReadComplete - error" );

//	if ( --me->fIOcount == 0 )
//		me->release();

	return;
	
}/* end commReadComplete */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::dataReadComplete
//
//		Inputs:		obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//
//		Outputs:	None
//
//		Desc:		BulkIn pipe (Data interface) read completion routine
//
/****************************************************************************************************/

void BelkinF5U103Driver::dataReadComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	BelkinF5U103Driver	*me = (BelkinF5U103Driver*)obj;
	PortInfo_t 			*port = (PortInfo_t*)param;
	IOReturn	ior;

	if ( rc == kIOReturnSuccess )	/* If operation returned ok:	*/
	{	
		ELG( port->State, 128 - remaining, 'DRC+', "dataReadComplete" );
		
		LogData( kUSBIn, (128 - remaining), me->fPipeInBuffer );
	
			/* Move the incoming bytes to the ring buffer:	*/
		AddtoQueue( &port->RX, me->fPipeInBuffer, 128 - remaining );
	
			/* Queue the next read:	*/
	
		ior = me->fpInPipe->Read( me->fpPipeInMDP, &me->fReadCompletionInfo, NULL );
		if ( ior == kIOReturnSuccess )
		{
			CheckQueues( port );
			return;
		}
	}

        /* Read returned with error OR next bulk read failed to be queued:	*/

        ELG( 0, rc, 'erR-', "BelkinF5U103Driver::dataReadComplete - io err" );

//	if ( --me->fIOcount == 0 )
//		me->release();

	return;
	
}/* end dataReadComplete */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::dataWriteComplete
//
//		Inputs:		obj - me, param - parameter block(the Port), rc - return code, remaining - what's left
//
//		Outputs:	None
//
//		Desc:		BulkOut pipe (Data interface) write completion routine
//
/****************************************************************************************************/

void BelkinF5U103Driver::dataWriteComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
	BelkinF5U103Driver	*me = (BelkinF5U103Driver*)obj;
	PortInfo_t 			*port = (PortInfo_t*)param;

//	if ( --me->fIOcount == 0 )
//		me->release();

	if ( rc == kIOReturnSuccess )	/* If operation returned ok:	*/
	{	
		ELG( rc, (me->fCount - remaining), 'DWC+', "dataWriteComplete" );

		changeState( port, 0, PD_S_TX_BUSY );	/// mlj
		port->AreTransmitting = false;
		CheckQueues( port );
		me->SetUpTransmit( port );					// just to keep it going??
		return;
	}

	ELG( 0, rc, 'erW-', "BelkinF5U103Driver::dataWriteComplete - io err" );

	return;
	
}/* end dataWriteComplete */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::merWriteComplete
//
//		Inputs:		obj - me, param - parameter block (may or may not be present depending on request), 
//																	rc - return code, remaining - what's left
//
//		Outputs:	None
//
//		Desc:		Management element request write completion routine
//
/****************************************************************************************************/

void BelkinF5U103Driver::merWriteComplete( void *obj, void *param, IOReturn rc, UInt32 remaining )
{
//	BelkinF5U103Driver	*me = (BelkinF5U103Driver*)obj;
	IOUSBDevRequest		*MER = (IOUSBDevRequest*)param;
	UInt16				dataLen;
	
//	if ( --me->fIOcount == 0 )
//		me->release();
	
	if (MER)
	{
		if ( rc == kIOReturnSuccess )
		{
			ELG( MER->bRequest, remaining, 'rWC+', "BelkinF5U103Driver::merWriteComplete" );
		} else {
			ELG( MER->bRequest, rc, 'rWC-', "BelkinF5U103Driver::merWriteComplete - io err" );
		}
		
		dataLen = MER->wLength;
		ELG( 0, dataLen, 'rWC ', "BelkinF5U103Driver::merWriteComplete - data length" );
		if ((dataLen != 0) && (MER->pData))
		{
			IOFree( MER->pData, dataLen );
		}
		IOFree( MER, sizeof(IOUSBDevRequest) );
		
	} else {
		if ( rc == kIOReturnSuccess )
		{
			ELG( 0, remaining, 'rWC+', "BelkinF5U103Driver::merWriteComplete (request unknown)" );
		} else {
			ELG( 0, rc, 'rWC-', "BelkinF5U103Driver::merWriteComplete (request unknown) - io err" );
		}
	}
	
	return;
	
}/* end merWriteComplete */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::free
//
//		Inputs:		None
//
//		Outputs:	None
//
//		Desc:		Clean up and free the log 
//
/****************************************************************************************************/

void BelkinF5U103Driver::free()
{

    ELG( 0, 0, 'free', "BelkinF5U103Driver::free" );
	
#if USE_ELG
    if ( g.evLogBuf )
    	IOFree( g.evLogBuf, kEvLogSize );
#endif /* USE_ELG */

    super::free();
    return;
	
}/* end free */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::stop
//
//		Inputs:		provider - my provider
//
//		Outputs:	None
//
//		Desc:		Stops
//
/****************************************************************************************************/

void BelkinF5U103Driver::stop( IOService *provider )
{
    UInt8	i;
	
    ELG( 0, 0, 'stop', "BelkinF5U103Driver::stop" );

    for (i=0; i<numberofPorts; i++)
    {
            if (fPorts[i] != NULL)
            {
                    IOFree( fPorts[i], sizeof(PortInfo_t) );
                    fPorts[i] = NULL;
            }
    }

    removeProperty( (const char *)propertyTag );

    if ( fpCommInterface )	
    {
        fpCommInterface->close( this );	
        fpCommInterface->release();
        fpCommInterface = NULL;	
    }
	
    fpDevice->close(this);
    super::stop( provider );
    return;
	
}/* end stop */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::allocateResources
//
//		Inputs:		port - the Port
//
//		Outputs:	return code - true (allocate was successful), false (it failed)
//
//		Desc:		Finishes up the rest of the configuration and gets all the endpoints open etc.
//
/****************************************************************************************************/

bool BelkinF5U103Driver::allocateResources( PortInfo_t *port )
{
	IOUSBFindEndpointRequest	epReq;		// endPoint request struct on stack
	bool						goodCall;	// return flag fm Interface call

        ELG( 0, 0, 'Allo', "BelkinF5U103Driver::allocateResources." );

        // Open all the end points:

        if(!fpCommInterface->isOpen(this)) {
	goodCall = fpCommInterface->open( this );
	if ( !goodCall )
	{
		ELG( 0, 0, 'epC-', "BelkinF5U103Driver::allocateResources - open comm interface failed." );
		fpCommInterface->release();
		fpCommInterface = NULL;
		return false;
	}
	fpCommInterfaceNumber = fpCommInterface->GetInterfaceNumber();
	ELG( 0, fpCommInterfaceNumber, 'CIn#', "BelkinF5U103Driver::allocateResources - Comm interface number." );

	epReq.type		= kUSBBulk;
	epReq.direction		= kUSBIn;
	epReq.maxPacketSize	= 0;
	epReq.interval		= 0;
	fpInPipe = fpCommInterface->FindNextPipe( 0, &epReq );
	if ( !fpInPipe )
	{
		ELG( 0, 0, 'i P-', "BelkinF5U103Driver::allocateResources - no bulk input pipe." );
		return false;
	}
	ELG( epReq.maxPacketSize << 16 |epReq.interval, fpInPipe, 'i P+', "BelkinF5U103Driver::allocateResources - bulk input pipe." );

	epReq.direction		= kUSBOut;
	fpOutPipe = fpCommInterface->FindNextPipe( 0, &epReq );
	if ( !fpOutPipe )
	{
		ELG( 0, 0, 'o P-', "BelkinF5U103Driver::allocateResources - no bulk output pipe." );
		return false;
	}
	ELG( epReq.maxPacketSize << 16 |epReq.interval, fpOutPipe, 'o P+', "BelkinF5U103Driver::allocateResources - bulk output pipe." );

        // Interrupt pipe - Comm Interface:

	epReq.type		= kUSBInterrupt;
	epReq.direction		= kUSBIn;
	fpCommPipe = fpCommInterface->FindNextPipe( 0, &epReq );
	if ( !fpCommPipe )
	{
		ELG( 0, 0, 'c P-', "BelkinF5U103Driver::allocateResources - no comm pipe." );
		releaseResources( port );
		return false;
	}
	ELG( epReq.maxPacketSize << 16 |epReq.interval, fpCommPipe, 'c P+', "BelkinF5U103Driver::allocateResources - comm pipe." );

        // Allocate Memory Descriptor Pointer with memory for the Comm pipe:

	fpCommPipeMDP = IOBufferMemoryDescriptor::withCapacity( COMM_BUFF_SIZE, kIODirectionIn );
	if ( !fpCommPipeMDP )
		return false;
		
	fpCommPipeMDP->setLength( COMM_BUFF_SIZE );
	fCommPipeBuffer = (UInt8*)fpCommPipeMDP->getBytesNoCopy();
        ELG( 0, fCommPipeBuffer, 'cBuf', "BelkinF5U103Driver::allocateResources - comm buffer" );

		// Allocate Memory Descriptor Pointer with memory for the data-in bulk pipe:

	fpPipeInMDP = IOBufferMemoryDescriptor::withCapacity( 128, kIODirectionIn );
	if ( !fpPipeInMDP )
		return false;
		
	fpPipeInMDP->setLength( 128 );
	fPipeInBuffer = (UInt8*)fpPipeInMDP->getBytesNoCopy();
        ELG( 0, fPipeInBuffer, 'iBuf', "BelkinF5U103Driver::allocateResources - input buffer" );

        // Allocate Memory Descriptor Pointer with memory for the data-out bulk pipe:

	fpPipeOutMDP = IOBufferMemoryDescriptor::withCapacity( MAX_BLOCK_SIZE, kIODirectionOut );
	if ( !fpPipeOutMDP )
		return false;
		
	fpPipeOutMDP->setLength( MAX_BLOCK_SIZE );
	fPipeOutBuffer = (UInt8*)fpPipeOutMDP->getBytesNoCopy();
        ELG( 0, fPipeOutBuffer, 'oBuf', "BelkinF5U103Driver::allocateResources - output buffer" );

        // Allocate some locks:

	port->serialRequestLock = IOLockAlloc();	// init lock used to protect code on MP
        if ( !port->serialRequestLock )
		return false;
            }
		
	return true;
	
}/* end allocateResources */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::releaseResources
//
//		Inputs:		port - the Port
//
//		Outputs:	None
//
//		Desc:		Frees up the resources allocated in allocateResources
//
/****************************************************************************************************/

void BelkinF5U103Driver::releaseResources( PortInfo_t *port )
{
	ELG( 0, 0, 'rlRs', "BelkinF5U103Driver::releaseResources" );

	if ( fpCommInterface )	
	{
		fpCommInterface->close( this );		
	}

	if ( port->serialRequestLock )
	{
		IOLockFree( port->serialRequestLock );	// free the Serial Request Lock
		port->serialRequestLock = 0;
	}

	if ( fpPipeOutMDP  )	
	{ 
		fpPipeOutMDP->release();	
		fpPipeOutMDP	= 0; 
	}
	
	if ( fpPipeInMDP   )	
	{ 
		fpPipeInMDP->release();	
		fpPipeInMDP		= 0; 
	}
	
	if ( fpCommPipeMDP )	
	{ 
		fpCommPipeMDP->release();	
		fpCommPipeMDP	= 0; 
	}

	return;
	
}/* end releaseResources */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::configureDevice
//
//		Inputs:		numConfigs - number of configurations present
//
//		Outputs:	return Code - true (device configured), false (device not configured)
//
//		Desc:		Finds the configurations and then the appropriate interfaces etc.
//
/****************************************************************************************************/

bool BelkinF5U103Driver::configureDevice( UInt8 numConfigs )
{
	IOUSBFindInterfaceRequest		req;			// device request Class on stack
	const IOUSBConfigurationDescriptor	*cd = NULL;		// configuration descriptor
	IOUSBInterfaceDescriptor 		*intf = NULL;		// interface descriptor
	IOReturn				ior;
	UInt8					cval;
	UInt8								config = 0;
	bool								gotDescriptors = false;
	const FunctionalDescriptorHeader 	*fDesc = NULL;
	CMFunctionalDescriptor			*CMFDesc;		// call management functional descriptor
	ACMFunctionalDescriptor			*ACMFDesc;		// abstract control management functional descriptor
       
        ELG( 0, numConfigs, 'cDev', "BelkinF5U103Driver::configureDevice" );
	
#if 1
        cd = fpDevice->GetFullConfigurationDescriptor(0);
        config = cd->bConfigurationValue;
#else
	for (cval=0; cval<numConfigs; cval++)
	{

            ELG( 0, cval, 'CkCn', "BelkinF5U103Driver::configureDevice - Checking Configuration" );

            cd = fpDevice->GetFullConfigurationDescriptor(cval);
            if ( !cd )
            {
                ELG( 0, 0, 'GFC-', "BelkinF5U103Driver::configureDevice - Error getting the full configuration descriptor" );
            } else {
                req.bInterfaceClass	= kUSBCommClass;
                req.bInterfaceSubClass	= kUSBAbstractControlModel;
                req.bInterfaceProtocol	= kUSBv25;
                req.bAlternateSetting	= kIOUSBFindInterfaceDontCare;
                ior = fpDevice->FindNextInterfaceDescriptor(cd, intf, &req, &intf);
                if ( ior == kIOReturnSuccess )
                {
                    if ( intf )
                    {
                        config = cd->bConfigurationValue;
                        ELG( 0, config, 'FNI+', "BelkinF5U103Driver::configureDevice - Interface descriptor found" );
                        break;
                    } else {
                        ELG( 0, config, 'FNI-', "BelkinF5U103Driver::configureDevice - That's weird the interface was null" );
                        cd = NULL;
                    }
                } else {
                    ELG( 0, cval, 'FNID', "BelkinF5U103Driver::configureDevice - No CDC interface found this configuration" );
                    cd = NULL;
                }
            }
	}
#endif
			
	if ( !cd )
	{
		return false;
	}
	
        // Now lets do it for real
		
	ior = fpDevice->SetConfiguration( this, config );
	if ( ior )
	{
		ELG( 0, ior, 'SCo-', "BelkinF5U103Driver::configureDevice - SetConfiguration error" );
		return false;			
	}
#if 0
	req.bInterfaceClass	= kUSBCommClass;
	req.bInterfaceSubClass	= kUSBAbstractControlModel;
	req.bInterfaceProtocol	= kUSBv25;
	req.bAlternateSetting	= kIOUSBFindInterfaceDontCare;

	fpCommInterface = fpDevice->FindNextInterface( NULL, &req );
#else
	req.bInterfaceClass	= kIOUSBFindInterfaceDontCare;
	req.bInterfaceSubClass	= kIOUSBFindInterfaceDontCare;
	req.bInterfaceProtocol	= kIOUSBFindInterfaceDontCare;
	req.bAlternateSetting	= kIOUSBFindInterfaceDontCare;
	fpCommInterface = fpDevice->FindNextInterface( NULL, &req );
#endif
	if ( !fpCommInterface )
	{
		ELG( 0, 0, 'FIC-', "BelkinF5U103Driver::configureDevice - Find next interface for the Comm Class failed" );
		return false;
	}
	fpCommInterface->retain();	
		// Set some defaults just in case and then get the associated functional descriptors
	
	fCMCapabilities = CM_ManagementData + CM_ManagementOnData;
	fACMCapabilities = ACM_DeviceSuppControl + ACM_DeviceSuppBreak;

	do
	{
		(const IOUSBDescriptorHeader*)fDesc = fpCommInterface->FindNextAssociatedDescriptor((void*)fDesc, CS_INTERFACE);
		if (!fDesc)
		{
			gotDescriptors = true;
		} else {
			ELG( fDesc->bDescriptorType, fDesc->bDescriptorSubtype, 'CMFD', "BelkinF5U103Driver::configureDevice - Got Functional Descriptor" );
			if (fDesc->bDescriptorSubtype == CM_FunctionalDescriptor)
			{
				(const FunctionalDescriptorHeader*)CMFDesc = fDesc;
				fCMCapabilities = CMFDesc->bmCapabilities;
				
				// Check the configuration supports data management on the data interface (that's all we support)
				
				if (!(fCMCapabilities & CM_ManagementData))
				{
					ELG( 0, 0, 'CMC-', "BelkinF5U103Driver::configureDevice - Device doesn't support Call Management" );
					return false;
				}
				if (!(fCMCapabilities & CM_ManagementOnData))
				{
					ELG( 0, 0, 'CMC-', "BelkinF5U103Driver::configureDevice - Device doesn't support Call Management on Data Interface" );
					return false;
				}
			} else {
				if (fDesc->bDescriptorSubtype == ACM_FunctionalDescriptor)
				{
					(const FunctionalDescriptorHeader*)ACMFDesc = fDesc;
					fACMCapabilities = ACMFDesc->bmCapabilities;
				}
			}
		}
	} while(!gotDescriptors);


	if ( createSerialStream() )	// Publish IOModemSerialStreamSync services
	{
		return true;
	}
	
	ELG( 0, 0, 'Nub-', "BelkinF5U103Driver::configureDevice - createSerialStream failed" );
	
	return false;

}/* end configureDevice */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::createSuffix
//
//		Inputs:		None
//
//		Outputs:	return Code - true (suffix created), false (suffix not create), sufKey - the key
//
//		Desc:		Creates the suffix key. It attempts to use the serial number string from the device
//					if it's reasonable i.e. less than 8 bytes ascii. Remember it's stored in unicode 
//					format. If it's not present or not reasonable it will generate the suffix based 
//					on the location property tag. At least this remains the same across boots if the
//					device is plugged into the same physical location. In the latter case trailing
//					zeros are removed.
//
/****************************************************************************************************/

bool BelkinF5U103Driver::createSuffix( unsigned char *sufKey )
{
    
	IOReturn				rc;
	UInt8					serBuf[10];		// arbitrary size > 8
	OSNumber				*location;
	UInt32					locVal;
	UInt8					*rlocVal;
	UInt16					offs, i, sig = 0;
	UInt8					indx;
	bool					keyOK = false;			
	
	ELG( 0, 0, 'cSuf', "BelkinF5U103Driver::createSuffix" );
	
        indx = fpDevice->GetSerialNumberStringIndex();	
	if ( indx != 0 )
	{	
		// Generate suffix key based on the serial number string (if reasonable <= 8 and > 0)	

		rc = fpDevice->GetStringDescriptor(indx, (char *)&serBuf, sizeof(serBuf));
		if ( !rc )
		{
			if ( (strlen((char *)&serBuf) < 9) && (strlen((char *)&serBuf) > 0) )
			{
				strcpy( (char *)sufKey, (const char *)&serBuf);
				keyOK = true;
			}			
		} else {
			ELG( 0, rc, 'Sdt-', "BelkinF5U103Driver::createSuffix error reading serial number string" );
		}
	}
	
	if ( !keyOK )
	{
		// Generate suffix key based on the location property tag
	
		location = (OSNumber *)fpDevice->getProperty(kUSBDevicePropertyLocationID);
		if ( location )
		{
			locVal = location->unsigned32BitValue();		
			offs = 0;
			rlocVal = (UInt8*)&locVal;
			for (i=0; i<4; i++)
			{
				sufKey[offs] = Asciify(rlocVal[i] >> 4);
				if ( sufKey[offs++] != '0')
					sig = offs;
				sufKey[offs] = Asciify(rlocVal[i]);
				if ( sufKey[offs++] != '0')
					sig = offs;
			}			
			sufKey[sig] = 0x00;
			keyOK = true;
		}
	}
	
	return keyOK;

}/* end createSuffix */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::createSerialStream
//
//		Inputs:		None
//
//		Outputs:	return Code - true (created and initialilzed ok), false (it failed)
//
//		Desc:		Creates and initializes the nub and port structure
//
/****************************************************************************************************/

bool BelkinF5U103Driver::createSerialStream()
{
    IOModemSerialStreamSync *pNub = new IOModemSerialStreamSync;
	bool					ret;
	UInt8					i,indx;
	IOReturn				rc;
	PortInfo_t 				*port = NULL;
	unsigned char			rname[10];
	const char				*suffix = (const char *)&rname;
	
/*----Allocate the port structure (multiple ports would be handled and registered here)-----*/
	
    ELG( 0, pNub, '=Nub', "BelkinF5U103Driver::createSerialStream - 0, nub" );
    if ( !pNub )
	{
        return false;
	}
		
	for (i=0; i<numberofPorts; i++)
	{
		if (fPorts[i] == NULL)
		{
			port = (PortInfo_t*)IOMalloc( sizeof(PortInfo_t) );
			fPorts[i] = port;
			ELG( port, i, 'Port', "BelkinF5U103Driver::createSerialStream - Port found" );
			break;
		}
	}
	
	if ( !port )
	{
		ELG( 0, i, 'Port', "BelkinF5U103Driver::createSerialStream - No ports available or IOMalloc failed" );
		return false;
	}

    	// Either we attached and should get rid of our reference
    	// or we failed in which case we should get rid our reference as well.
		// This just makes sure the reference count is correct.
	
    ret = (pNub->init(0, port) && pNub->attach( this ));
	
    pNub->release();
    if ( !ret )
	{
		ELG( ret, i, 'Nub-', "BelkinF5U103Driver::createSerialStream - Didn't attach to the nub properly" );
		IOFree( port, sizeof(PortInfo_t) );
		fPorts[i] = NULL;
        return false;
	}

    // Report the base name to be used for generating device nodes
	
    pNub->setProperty( kIOTTYBaseNameKey, baseName );
	
    // Create suffix key and set it
	
	if ( createSuffix( (unsigned char *)suffix ) )
	{		
		pNub->setProperty( kIOTTYSuffixKey, suffix );
	}

    pNub->registerService();
	
	SetStructureDefaults( port, true );			// init the Port structure

//    port->TXStats.BufferSize = BUFFER_SIZE_DEFAULT;
//    port->RXStats.BufferSize = BUFFER_SIZE_DEFAULT;
	
	// Save the Product String	(at least the first productNameLength's worth). This is done (same name) per stream for the moment.

	indx = fpDevice->GetProductStringIndex();	
	if ( indx != 0 )
	{	
		rc = fpDevice->GetStringDescriptor( indx, (char *)&fProductName, sizeof(fProductName) );
		if ( !rc )
		{
			if ( strlen((char *)fProductName) == 0 )		// believe it or not this sometimes happens (null string with an index defined???)
			{
				strcpy( (char *)fProductName, defaultName);
			}
			pNub->setProperty( (const char *)propertyTag, (const char *)fProductName );
		}
	}
	
/*---------------------------------------------------------------------------------------------*/

    return true;
	
}/* end createSerialStream */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::acquirePort
//
//		Inputs:		sleep - true (wait for it), false (don't), refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnExclusiveAccess, kIOReturnIOError and various others
//
//		Desc:		acquirePort tests and sets the state of the port object.  If the port was
// 					available, then the state is set to busy, and kIOReturnSuccess is returned.
// 					If the port was already busy and sleep is YES, then the thread will sleep
// 					until the port is freed, then re-attempts the acquire.  If the port was
// 					already busy and sleep is NO, then kIOReturnExclusiveAccess is returned.
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::acquirePort( bool sleep, void *refCon )
{
	PortInfo_t 			*port = (PortInfo_t *) refCon;
    UInt32 				busyState = 0;
    IOReturn 			rtn = kIOReturnSuccess;

    ELG( port, sleep, 'acqP', "BelkinF5U103Driver::acquirePort" );
	
    if ( !allocateResources( port ) ) 
    {
    	return kIOReturnNoMemory;
    }
	
    SetStructureDefaults( port, FALSE );	/* Initialize all the structures */

    for (;;)
	{
        busyState = readPortState( port ) & PD_S_ACQUIRED;
        if ( !busyState )
        {		
			// Set busy bit, and clear everything else
            changeState( port, (UInt32)PD_S_ACQUIRED | DEFAULT_STATE, (UInt32)STATE_ALL);
            break;
        } else {
            if ( !sleep )
            {
                ELG( 0, 0, 'busy', "BelkinF5U103Driver::acquirePort - Busy exclusive access" );
                releaseResources( port );
                return kIOReturnExclusiveAccess;
            } else {
                busyState = 0;
            	rtn = watchState( &busyState, PD_S_ACQUIRED, refCon );
            	if ( (rtn == kIOReturnIOError) || (rtn == kIOReturnSuccess) )
                {
                    continue;
            	} else {
                    ELG( 0, 0, 'int-', "BelkinF5U103Driver::acquirePort - Interrupted!" );
                    releaseResources( port );				// aquire should be called again
                    return rtn;
                }
            }
        }
    } /* end for */
	
    if (!allocateRingBuffer(&(port->TX), port->TXStats.BufferSize) || !allocateRingBuffer(&(port->RX), port->RXStats.BufferSize)) 
    {
        releaseResources( port );
        return kIOReturnNoMemory;
    }
	
    // Read the comm interrupt pipe for status:
		
    fCommCompletionInfo.target	= this;
    fCommCompletionInfo.action	= commReadComplete;
    fCommCompletionInfo.parameter	= port;
		
    rtn = fpCommPipe->Read(fpCommPipeMDP, &fCommCompletionInfo, NULL );
    if ( rtn == kIOReturnSuccess )
    {
//        fIOcount++;	// bump count of IO operations outstanding

        // Read the data-in bulk pipe:
			
        fReadCompletionInfo.target	= this;
        fReadCompletionInfo.action	= dataReadComplete;
        fReadCompletionInfo.parameter	= port;
		
        rtn = fpInPipe->Read(fpPipeInMDP, &fReadCompletionInfo, NULL );
			
        if ( rtn == kIOReturnSuccess )
        {
//        	fIOcount++;	// bump count of IO operations outstanding

        	// Set up the data-out bulk pipe:
			
        	fWriteCompletionInfo.target	= this;
        	fWriteCompletionInfo.action	= dataWriteComplete;
        	fWriteCompletionInfo.parameter	= port;
		
                // Set up the management element request completion routine:

                fMERCompletionInfo.target		= this;
                fMERCompletionInfo.action		= merWriteComplete;
                fMERCompletionInfo.parameter	= NULL;				// for now, filled in with parm block when allocated

//        	retain(); // IO operations outstanding - retain myself until complete

        	ELG( 0, 0, 'Erly', "BelkinF5U103Driver::acquirePort - Early" );
        }
    }

    if (rtn != kIOReturnSuccess)
    {
    	// We failed for some reason
	
    	freeRingBuffer(&(port->TX));
    	freeRingBuffer(&(port->RX));

    	releaseResources( port );
    	changeState(port, 0, STATE_ALL);	// Clear the entire state word
    } else {
        fSessions++;	//bump number of active sessions and turn on clear to send
        changeState( port, PD_RS232_S_CTS, PD_RS232_S_CTS);
    }

    return rtn;
	
}/* end acquirePort */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::releasePort
//
//		Inputs:		refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnNotOpen
//
//		Desc:		releasePort returns all the resources and does clean up.
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::releasePort( void *refCon )
{
    PortInfo_t 	*port = (PortInfo_t *) refCon;
    UInt32 	busyState;

    ELG( 0, port, 'relP', "BelkinF5U103Driver::releasePort" );

    //
    // Traditionally the PD_S_ACQUIRED test does not
    // have to be atomic.  One can usually assume that the code that
    // is opening and closing this driver knows how to single
    // thread the acquirePort/releasePort.
    //
    // However as I can't work the CDC driver out I'll have to make readPortState
    // slightly slower and test to see if it is safe to test to see if it is has
    // been acquired in readPortState???
    // 
    busyState = (readPortState( port ) & PD_S_ACQUIRED);
    if ( !busyState )
    {
        ELG( 0, 0, 'rlP-', "BelkinF5U103Driver::releasePort - NOT OPEN" );
        return kIOReturnNotOpen;
    }

    changeState( port, 0, (UInt32)PD_S_ACTIVE ); // deactivate the port
				
    USBSetControlLineState(false, false);			// clear RTS and clear DTR
	
    changeState( port, 0, (UInt32)STATE_ALL );	// Clear the entire state word

    // Remove all the buffers.

    freeRingBuffer( &port->TX );
    freeRingBuffer( &port->RX );
	
    // Release all resources

#if 0
    releaseResources( port );	
//	IOFree( port, sizeof(PortInfo_t) );

    ELG( 0, 0, 'RlP+', "BelkinF5U103Driver::releasePort - OK" );

    fSessions--;		// reduce number of active sessions
    ELG( fTerminate, fSessions, 'RlP+', "BelkinF5U103Driver::releasePort - close" );
    if ((!fTerminate) && (fSessions == 0))	// if it's the result of a terminate and session count is zero we also need to close the device
    {
    	fpDevice->close(this);
    }
#else
    if(fpCommPipe) {
        fpCommPipe->ClearStall();
    }
    if(fpInPipe) {
        fpInPipe->ClearStall();
    }
    if(fpOutPipe) {
        fpOutPipe->ClearStall();
    }

    changeState( port, 0, (UInt32)STATE_ALL );	// Clear the entire state word

    fSessions--;		// reduce number of active sessions
#endif
    return kIOReturnSuccess;
	
}/* end releasePort */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::setState
//
//		Inputs:		state - state to set, mask - state mask, refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess or kIOReturnBadArgument
//
//		Desc:		Set the state for the port device.  The lower 16 bits are used to set the
//					state of various flow control bits (this can also be done by enqueueing a
//					PD_E_FLOW_CONTROL event).  If any of the flow control bits have been set
//					for automatic control, then they can't be changed by setState.  For flow
//					control bits set to manual (that are implemented in hardware), the lines
//					will be changed before this method returns.  The one weird case is if RXO
//					is set for manual, then an XON or XOFF character may be placed at the end
//					of the TXQ and transmitted later.
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::setState( UInt32 state, UInt32 mask, void *refCon )
{
	PortInfo_t *port = (PortInfo_t *) refCon;
	
	ELG( state, mask, 'stSt', "BelkinF5U103Driver::setState" );
	
    if ( mask & (PD_S_ACQUIRED | PD_S_ACTIVE | (~EXTERNAL_MASK)) )
        return kIOReturnBadArgument;

    if ( readPortState( port ) & PD_S_ACQUIRED )
	{
	        // ignore any bits that are read-only
        mask &= (~port->FlowControl & PD_RS232_A_MASK) | PD_S_MASK;

        if ( mask)
            changeState( port, state, mask );

        return kIOReturnSuccess;
    }

    return kIOReturnNotOpen;
	
}/* end setState */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::getState
//
//		Inputs:		refCon - the Port
//
//		Outputs:	state - port state
//
//		Desc:		Get the state for the port.
//
/****************************************************************************************************/

UInt32 BelkinF5U103Driver::getState( void *refCon )
{
    PortInfo_t 	*port = (PortInfo_t *) refCon;
    UInt32 		state;
	
    ELG( 0, port, 'gtSt', "BelkinF5U103Driver::getState" );
	
    CheckQueues( port );
	
    state = readPortState( port ) & EXTERNAL_MASK;
	
    ELG( state, EXTERNAL_MASK, 'gtS-', "BelkinF5U103Driver::getState-->State" );

    return state;
	
}/* end getState */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::watchState
//
//		Inputs:		state - state to watch for, mask - state mask bits, refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from ::watchState
//
//		Desc:		Wait for the at least one of the state bits defined in mask to be equal
//					to the value defined in state. Check on entry then sleep until necessary,
//					see watchState for more details.
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::watchState( UInt32 *state, UInt32 mask, void *refCon)
{
    PortInfo_t 	*port = (PortInfo_t *) refCon;
    IOReturn 	ret = kIOReturnNotOpen;

    ELG( *state, mask, 'WatS', "BelkinF5U103Driver::watchState" );

    if ( readPortState( port ) & PD_S_ACQUIRED )
    {
        ret = kIOReturnSuccess;
        mask &= EXTERNAL_MASK;
        ret = privateWatchState( port, state, mask );
		*state &= EXTERNAL_MASK;
    }
	
    ELG( ret, 0, 'WatS', "BelkinF5U103Driver::watchState --> watchState" );
    return ret;
	
}/* end watchState */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::nextEvent
//
//		Inputs:		refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess
//
//		Desc:		Not used by this driver.
//
/****************************************************************************************************/

UInt32 BelkinF5U103Driver::nextEvent( void *refCon )
{
    UInt32 		ret = kIOReturnSuccess;

    ELG( 0, 0, 'NxtE', "BelkinF5U103Driver::nextEvent" );

    return ret;
	
}/* end nextEvent */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::executeEvent
//
//		Inputs:		event - The event, data - any data associated with the event, refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen or kIOReturnBadArgument
//
//		Desc:		executeEvent causes the specified event to be processed immediately.
//					This is primarily used for channel control commands like START & STOP
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::executeEvent( UInt32 event, UInt32 data, void *refCon )
{
    PortInfo_t 	*port = (PortInfo_t *) refCon;
    IOReturn	ret = kIOReturnSuccess;
    UInt32 		state, delta;
	
    delta = 0;
    state = readPortState( port );	
    ELG( port, state, 'ExIm', "BelkinF5U103Driver::executeEvent" );
	
    if ( (state & PD_S_ACQUIRED) == 0 )
        return kIOReturnNotOpen;

    switch ( event )
    {
	case PD_RS232_E_XON_BYTE:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_RS232_E_XON_BYTE" );
		port->XONchar = data;
		break;
	case PD_RS232_E_XOFF_BYTE:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_RS232_E_XOFF_BYTE" );
		port->XOFFchar = data;
		break;
	case PD_E_SPECIAL_BYTE:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_E_SPECIAL_BYTE" );
		port->SWspecial[ data >> SPECIAL_SHIFT ] |= (1 << (data & SPECIAL_MASK));
		break;

	case PD_E_VALID_DATA_BYTE:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_E_VALID_DATA_BYTE" );
		port->SWspecial[ data >> SPECIAL_SHIFT ] &= ~(1 << (data & SPECIAL_MASK));
		break;

	case PD_E_FLOW_CONTROL:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_E_FLOW_CONTROL" );
                USBRequest( UBSA_SET_FLOW_CTRL, UBSA_FLOW_NONE);
		break;

	case PD_E_ACTIVE:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_ACTIVE" );
		if ( (bool)data )
		{
			if ( !(state & PD_S_ACTIVE) )
			{
				SetStructureDefaults( port, FALSE );
                                changeState( port, (UInt32)PD_S_ACTIVE, (UInt32)PD_S_ACTIVE ); // activate port
				
				USBSetControlLineState(true, true);			// set RTS and set DTR
			}
		} else {
			if ( (state & PD_S_ACTIVE) )
			{
				changeState( port, 0, (UInt32)PD_S_ACTIVE );
				
				USBSetControlLineState(false, false);			// clear RTS and clear DTR
			}
		}
		break;

	case PD_E_DATA_LATENCY:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_E_DATA_LATENCY" );
		port->DataLatInterval = long2tval( data * 1000 );
		break;

	case PD_RS232_E_MIN_LATENCY:
		ELG( data, event, 'ExIm', "BelkinF5U103Driver::executeEvent - PD_RS232_E_MIN_LATENCY" );
		port->MinLatency = bool( data );
		break;

	case PD_E_DATA_INTEGRITY:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_DATA_INTEGRITY" );
		if ( (data < PD_RS232_PARITY_NONE) || (data > PD_RS232_PARITY_SPACE))
		{
			ret = kIOReturnBadArgument;
		}
		else
		{
			port->TX_Parity = data;
			port->RX_Parity = PD_RS232_PARITY_DEFAULT;
			
			USBSetLineCoding( port );			
		}
		break;

	case PD_E_DATA_RATE:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_DATA_RATE" );
			/* For API compatiblilty with Intel.	*/
		data >>= 1;
		ELG( 0, data, 'Exlm', "BelkinF5U103Driver::executeEvent - actual data rate" );
		if ( (data < MIN_BAUD) || (data > kMaxBaudRate) )
			ret = kIOReturnBadArgument;
		else
		{
			port->BaudRate = data;
			
			USBSetLineCoding( port );			
		}		
		break;

	case PD_E_DATA_SIZE:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_DATA_SIZE" );
			/* For API compatiblilty with Intel.	*/
		data >>= 1;
		ELG( 0, data, 'Exlm', "BelkinF5U103Driver::executeEvent - actual data size" );
		if ( (data < 5) || (data > 8) )
			ret = kIOReturnBadArgument;
		else
		{
			port->CharLength = data;
			
			USBSetLineCoding( port );			
		}
		break;

	case PD_RS232_E_STOP_BITS:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_RS232_E_STOP_BITS" );
		if ( (data < 0) || (data > 20) )
			ret = kIOReturnBadArgument;
		else
		{
			port->StopBits = data;
			
			USBSetLineCoding( port );
		}
		break;

	case PD_E_RXQ_FLUSH:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RXQ_FLUSH" );
		break;

	case PD_E_RX_DATA_INTEGRITY:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RX_DATA_INTEGRITY" );
		if ( (data != PD_RS232_PARITY_DEFAULT) &&  (data != PD_RS232_PARITY_ANY) )
			ret = kIOReturnBadArgument;
		else
			port->RX_Parity = data;
		break;

	case PD_E_RX_DATA_RATE:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RX_DATA_RATE" );
		if ( data )
			ret = kIOReturnBadArgument;
		break;

	case PD_E_RX_DATA_SIZE:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RX_DATA_SIZE" );
		if ( data )
			ret = kIOReturnBadArgument;
		break;

	case PD_RS232_E_RX_STOP_BITS:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_RS232_E_RX_STOP_BITS" );
		if ( data )
			ret = kIOReturnBadArgument;
		break;

	case PD_E_TXQ_FLUSH:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_TXQ_FLUSH" );
		break;

	case PD_RS232_E_LINE_BREAK:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_RS232_E_LINE_BREAK" );
		state &= ~PD_RS232_S_BRK;
		delta |= PD_RS232_S_BRK;
		break;

	case PD_E_DELAY:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_DELAY" );
		port->CharLatInterval = long2tval(data * 1000);
		break;
		
	case PD_E_RXQ_SIZE:
		ELG( 0, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RXQ_SIZE" );
		break;

	case PD_E_TXQ_SIZE:
		ELG( 0, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_TXQ_SIZE" );
		break;

	case PD_E_RXQ_HIGH_WATER:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RXQ_HIGH_WATER" );
		break;

	case PD_E_RXQ_LOW_WATER:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_RXQ_LOW_WATER" );
		break;

	case PD_E_TXQ_HIGH_WATER:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_TXQ_HIGH_WATER" );
		break;

	case PD_E_TXQ_LOW_WATER:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - PD_E_TXQ_LOW_WATER" );
		break;

	default:
		ELG( data, event, 'Exlm', "BelkinF5U103Driver::executeEvent - unrecognized event" );
		ret = kIOReturnBadArgument;
		break;
	}

    state |= state;/* ejk for compiler warnings. ?? */
    changeState( port, state, delta );
	
    return ret;
	
}/* end executeEvent */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::requestEvent
//
//		Inputs:		event - The event, refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnBadArgument, data - any data associated with the event
//
//		Desc:		requestEvent processes the specified event as an immediate request and
//					returns the results in data.  This is primarily used for getting link
//					status information and verifying baud rate and such.
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::requestEvent( UInt32 event, UInt32 *data, void *refCon )
{
    PortInfo_t	*port = (PortInfo_t *) refCon;
    IOReturn	returnValue = kIOReturnSuccess;

    ELG( 0, readPortState( port ), 'ReqE', "BelkinF5U103Driver::requestEvent" );

    if ( data == NULL ) {
		ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - data is null" );
        returnValue = kIOReturnBadArgument;
	}
    else
	{
		switch ( event )
		{
			case PD_E_ACTIVE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_ACTIVE" );
				*data = bool(readPortState( port ) & PD_S_ACTIVE);	
				break;
			
			case PD_E_FLOW_CONTROL:
				ELG( port->FlowControl, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_FLOW_CONTROL" );
				*data = port->FlowControl;							
				break;
			
			case PD_E_DELAY:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_DELAY" );
				*data = tval2long( port->CharLatInterval )/ 1000;	
				break;
			
			case PD_E_DATA_LATENCY:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_DATA_LATENCY" );
				*data = tval2long( port->DataLatInterval )/ 1000;	
				break;

			case PD_E_TXQ_SIZE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_TXQ_SIZE" );
				*data = GetQueueSize( &port->TX );	
				break;
			
			case PD_E_RXQ_SIZE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RXQ_SIZE" );
				*data = GetQueueSize( &port->RX );	
				break;

			case PD_E_TXQ_LOW_WATER:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_TXQ_LOW_WATER" );
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
		
			case PD_E_RXQ_LOW_WATER:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RXQ_LOW_WATER" );
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
		
			case PD_E_TXQ_HIGH_WATER:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_TXQ_HIGH_WATER" );
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
		
			case PD_E_RXQ_HIGH_WATER:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RXQ_HIGH_WATER" );
				*data = 0; 
				returnValue = kIOReturnBadArgument; 
				break;
		
			case PD_E_TXQ_AVAILABLE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_TXQ_AVAILABLE" );
				*data = FreeSpaceinQueue( &port->TX );	 
				break;
			
			case PD_E_RXQ_AVAILABLE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RXQ_AVAILABLE" );
				*data = UsedSpaceinQueue( &port->RX ); 	
				break;

			case PD_E_DATA_RATE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_DATA_RATE" );
				*data = port->BaudRate << 1;		
				break;
			
			case PD_E_RX_DATA_RATE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RX_DATA_RATE" );
				*data = 0x00;					
				break;
			
			case PD_E_DATA_SIZE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_DATA_SIZE" );
				*data = port->CharLength << 1;	
				break;
			
			case PD_E_RX_DATA_SIZE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RX_DATA_SIZE" );
				*data = 0x00;					
				break;
			
			case PD_E_DATA_INTEGRITY:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_DATA_INTEGRITY" );
				*data = port->TX_Parity;			
				break;
			
			case PD_E_RX_DATA_INTEGRITY:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_E_RX_DATA_INTEGRITY" );
				*data = port->RX_Parity;			
				break;

			case PD_RS232_E_STOP_BITS:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_RS232_E_STOP_BITS" );
				*data = port->StopBits << 1;		
				break;
			
			case PD_RS232_E_RX_STOP_BITS:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_RS232_E_RX_STOP_BITS" );
				*data = 0x00;					
				break;
			
			case PD_RS232_E_XON_BYTE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_RS232_E_XON_BYTE" );
				*data = port->XONchar;			
				break;
			
			case PD_RS232_E_XOFF_BYTE:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_RS232_E_XOFF_BYTE" );
				*data = port->XOFFchar;			
				break;
			
			case PD_RS232_E_LINE_BREAK:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_RS232_E_LINE_BREAK" );
				*data = bool(readPortState( port ) & PD_RS232_S_BRK);
				break;
			
			case PD_RS232_E_MIN_LATENCY:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - PD_RS232_E_MIN_LATENCY" );
				*data = bool( port->MinLatency );		
				break;

			default:
				ELG( 0, event, 'ReqE', "BelkinF5U103Driver::requestEvent - unrecognized event" );
				returnValue = kIOReturnBadArgument; 			
				break;
		}
    }

    return kIOReturnSuccess;
	
}/* end requestEvent */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::enqueueEvent
//
//		Inputs:		event - The event, data - any data associated with the event, 
//												sleep - true (wait for it), false (don't), refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//		Desc:		Not used by this driver.	
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::enqueueEvent( UInt32 event, UInt32 data, bool sleep, void *refCon)
{
    PortInfo_t *port = (PortInfo_t *) refCon;
	
    ELG( data, event, 'EnqE', "BelkinF5U103Driver::enqueueEvent" );

    if ( readPortState( port ) & PD_S_ACTIVE )
    {
        return kIOReturnSuccess;
    }

    return kIOReturnNotOpen;
	
}/* end enqueueEvent */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::dequeueEvent
//
//		Inputs:		sleep - true (wait for it), false (don't), refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess, kIOReturnNotOpen
//
//		Desc:		Not used by this driver.		
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::dequeueEvent( UInt32 *event, UInt32 *data, bool sleep, void *refCon )
{
    PortInfo_t *port = (PortInfo_t *) refCon;
	
    ELG( 0, 0, 'DeqE', "dequeueEvent" );

    if ( (event == NULL) || (data == NULL) )
        return kIOReturnBadArgument;

    if ( readPortState( port ) & PD_S_ACTIVE )
    {
        return kIOReturnSuccess;
    }

    return kIOReturnNotOpen;
	
}/* end dequeueEvent */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::enqueueData
//
//		Inputs:		buffer - the data, size - number of bytes, sleep - true (wait for it), false (don't),
//																						refCon - the Port
//
//		Outputs:	Return Code - kIOReturnSuccess or value returned from watchState, count - bytes transferred,  
//
//		Desc:		enqueueData will attempt to copy data from the specified buffer to
//					the TX queue as a sequence of VALID_DATA events.  The argument
//					bufferSize specifies the number of bytes to be sent.  The actual
//					number of bytes transferred is returned in count.
//					If sleep is true, then this method will sleep until all bytes can be
//					transferred.  If sleep is false, then as many bytes as possible
//					will be copied to the TX queue.
//					Note that the caller should ALWAYS check the transferCount unless
//					the return value was kIOReturnBadArgument, indicating one or more
//					arguments were not valid.  Other possible return values are
//					kIOReturnSuccess if all requirements were met.		
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::enqueueData( UInt8 *buffer, UInt32 size, UInt32 *count, bool sleep, void *refCon )
{
    PortInfo_t 	*port = (PortInfo_t *) refCon;
    UInt32 	state = PD_S_TXQ_LOW_WATER;
    IOReturn 	rtn = kIOReturnSuccess;

    ELG( 0, sleep, 'eqDt', "BelkinF5U103Driver::enqueData" );

    if ( fTerminate )
        return kIOReturnNotOpen;

    if ( count == NULL || buffer == NULL )
        return kIOReturnBadArgument;

    *count = 0;

    if ( !(readPortState( port ) & PD_S_ACTIVE) )
        return kIOReturnNotOpen;

    ELG( port->State, size, 'eqDt', "BelkinF5U103Driver::enqueData State" );	
    LogData( kUSBAnyDirn, size, buffer );

    /* OK, go ahead and try to add something to the buffer	*/
    *count = AddtoQueue( &port->TX, buffer, size );
    CheckQueues( port );

    /* Let the tranmitter know that we have something ready to go	*/
    SetUpTransmit( port );

    /* If we could not queue up all of the data on the first pass and	*/
    /* the user wants us to sleep until it's all out then sleep	*/

    while ( (*count < size) && sleep )
    {
        state = PD_S_TXQ_LOW_WATER;
        rtn = watchState( &state, PD_S_TXQ_LOW_WATER, refCon );
        if ( rtn != kIOReturnSuccess )
            {
        	ELG( 0, rtn, 'EqD-', "BelkinF5U103Driver::enqueueData - interrupted" );
                return rtn;
            }

        *count += AddtoQueue( &port->TX, buffer + *count, size - *count );
        CheckQueues( port );

        /* Let the tranmitter know that we have something ready to go.	*/

        SetUpTransmit( port );
    }/* end while */

    ELG( *count, size, 'enqd', "BelkinF5U103Driver::enqueueData - Enqueue" );

    return kIOReturnSuccess;
	
}/* end enqueueData */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::dequeueData
//
//		Inputs:		size - buffer size, min - minimum bytes required, refCon - the Port
//
//		Outputs:	buffer - data returned, min - number of bytes
//					Return Code - kIOReturnSuccess, kIOReturnBadArgument, kIOReturnNotOpen, or value returned from watchState
//
//		Desc:		dequeueData will attempt to copy data from the RX queue to the
//					specified buffer.  No more than bufferSize VALID_DATA events
//					will be transferred. In other words, copying will continue until
//					either a non-data event is encountered or the transfer buffer
//					is full.  The actual number of bytes transferred is returned
//					in count.
//					The sleep semantics of this method are slightly more complicated
//					than other methods in this API. Basically, this method will
//					continue to sleep until either min characters have been
//					received or a non data event is next in the RX queue.  If
//					min is zero, then this method never sleeps and will return
//					immediately if the queue is empty.
//					Note that the caller should ALWAYS check the transferCount
//					unless the return value was kIOReturnBadArgument, indicating one or
//					more arguments were not valid.
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::dequeueData( UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon )
{
    PortInfo_t 	*port = (PortInfo_t *) refCon;
    IOReturn 	rtn = kIOReturnSuccess;
    UInt32 	state = 0;

    ELG( size, min, 'dqDt', "BelkinF5U103Driver::dequeueData" );
	
    /* Check to make sure we have good arguments.	*/
    if ( (count == NULL) || (buffer == NULL) || (min > size) )
        return kIOReturnBadArgument;

    /* If the port is not active then there should not be any chars.	*/
    *count = 0;
    if ( !(readPortState( port ) & PD_S_ACTIVE) )
        return kIOReturnNotOpen;

    /* Get any data living in the queue.	*/
    *count = RemovefromQueue( &port->RX, buffer, size );
    CheckQueues( port );

    while ( (min > 0) && (*count < min) )
    {
        /* Figure out how many bytes we have left to queue up */
        state = 0;

        rtn = watchState( &state, PD_S_RXQ_EMPTY, refCon );

        if ( rtn != kIOReturnSuccess )
        {
            ELG( 0, rtn, 'DqD-', "BelkinF5U103Driver::dequeueData - Interrupted!" );
            return rtn;
        }
        /* Try and get more data starting from where we left off */
        *count += RemovefromQueue( &port->RX, buffer + *count, (size - *count) );
        CheckQueues( port );
		
    }/* end while */

    /* Now let's check our receive buffer to see if we need to stop	*/
    bool goXOIdle = (UsedSpaceinQueue( &port->RX ) < port->RXStats.LowWater) && (port->RXOstate == SENT_XOFF);

    if ( goXOIdle )
    {
        port->RXOstate = IDLE_XO;
        AddBytetoQueue( &port->TX, port->XOFFchar );
        SetUpTransmit( port );
    }

    ELG( *count, size, 'deqd', "dequeueData -->Out Dequeue" );

    return rtn;
	
}/* end dequeueData */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::SetUpTransmit
//
//		Inputs:		Channel - the channel to transmit on
//
//		Outputs:	return code - true (transmit started), false (transmission already in progress)
//
//		Desc:		Setup and then start transmisson on the channel specified
//
/****************************************************************************************************/

bool BelkinF5U103Driver::SetUpTransmit( PortInfo_t *Channel )
{

    ELG( Channel, Channel->AreTransmitting, 'upTx', "BelkinF5U103Driver::SetUpTransmit" );
	
    //  If we are already in the cycle of transmitting characters,
    //  then we do not need to do anything.
		
    if ( Channel->AreTransmitting == TRUE )
        return FALSE;

    if ( GetQueueStatus( &Channel->TX ) != queueEmpty )
    {
        StartTransmission( Channel );
    }

    return TRUE;
	
}/* end SetUpTransmit */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::StartTransmission
//
//		Inputs:		Channel - the channel to transmit on
//
//		Outputs:	None
//
//		Desc:		Start the transmisson on the channel specified
//
/****************************************************************************************************/

void BelkinF5U103Driver::StartTransmission( PortInfo_t *Channel )
{
    size_t	count;
    IOReturn	ior;

    // Sets up everything as we are running so as not to start this
    // channel twice if a call occurs twice to this function:
		
    Channel->AreTransmitting = TRUE;
    changeState( Channel, PD_S_TX_BUSY, PD_S_TX_BUSY );

    /* Fill up the buffer with characters from the queue */
		
    count = RemovefromQueue( &Channel->TX, fPipeOutBuffer, MAX_BLOCK_SIZE );
    ELG( Channel->State, count, ' Tx+', "BelkinF5U103Driver::StartTransmission" );
    LogData( kUSBOut, count, fPipeOutBuffer );	
    fCount = count;

    // If there are no bytes to send just exit:
		
    if ( count <= 0 )
    {
        // Updates all the status flags:
			
        CheckQueues( Channel );
        Channel->AreTransmitting = FALSE;
        changeState( Channel, 0, PD_S_TX_BUSY );
        return;
    }

    fpPipeOutMDP->setLength( count );
    ior = fpOutPipe->Write( fpPipeOutMDP, &fWriteCompletionInfo );
//	fIOcount++;				// bump count of IO operations outstanding

    // We just removed a bunch of stuff from the
    // queue, so see if we can free some thread(s)
    // to enqueue more stuff.
		
    CheckQueues( Channel );

    return;            // and return
	
}/* end StartTransmission */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::USBSetLineCoding
//
//		Inputs:		None
//
//		Outputs:	None
//
//		Desc:		Set up and send SetLineCoding Management Element Request(MER) for all settings.
//
/****************************************************************************************************/

void BelkinF5U103Driver::USBSetLineCoding( PortInfo_t *port )
{
    LineCoding		*lineParms;
    IOReturn		rc;
    IOUSBDevRequest	*MER;
    UInt16		lcLen = sizeof(LineCoding)-1;
	
    ELG( 0, port, 'USLC', "BelkinF5U103Driver::USBSetLineCoding" );
	
    // First check that Set Line Coding is supported
	
    if (!(fACMCapabilities & ACM_DeviceSuppControl))
    {
            return;
    }
	
    // Check for changes and only do it if something's changed
	
    if ( (port->BaudRate == port->LastBaudRate) && (port->StopBits == port->LastStopBits) && 
            (port->TX_Parity == port->LastTX_Parity) && (port->CharLength == port->LastCharLength) )
    {
        return;
    }
	
#if 0
    MER = (IOUSBDevRequest*)IOMalloc( sizeof(IOUSBDevRequest) );
    if ( !MER )
    {
        ELG( 0, 0, 'USL-', "BelkinF5U103Driver::USBSetLineCoding - allocate MER failed" );
        return;
    }
    bzero( MER, sizeof(IOUSBDevRequest) );
	
    lineParms = (LineCoding*)IOMalloc( lcLen );
    if ( !lineParms )
    {
        ELG( 0, 0, 'USL-', "BelkinF5U103Driver::USBSetLineCoding - allocate lineParms failed" );
        return;
    }
    bzero( lineParms, lcLen ); 
	
    // convert BaudRate - intel format doubleword (32 bits) 
		
    OSWriteLittleInt32( lineParms, dwDTERateOffset, port->BaudRate );
    port->LastBaudRate = port->BaudRate;
    lineParms->bCharFormat = port->StopBits - 2;
    port->LastStopBits = port->StopBits;
    lineParms->bParityType = port->TX_Parity - 1;
    port->LastTX_Parity = port->TX_Parity;
    lineParms->bDataBits = port->CharLength;
    port->LastCharLength = port->CharLength;
	
    LogData( kUSBAnyDirn, lcLen, lineParms );

    // now build the Management Element Request
		
    MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
    MER->bRequest = kUSBSET_LINE_CODING;
    MER->wValue = 0;
    MER->wIndex = fpCommInterfaceNumber;
    MER->wLength = lcLen;
    MER->pData = lineParms;
	
    fMERCompletionInfo.parameter = MER;
	
    rc = fpDevice->DeviceRequest(MER, &fMERCompletionInfo);
    if ( rc == kIOReturnSuccess )
    {
//	fIOcount++;				// bump count of IO operations outstanding
    } else {
        ELG( MER->bRequest, rc, 'SLER', "BelkinF5U103Driver::USBSetLineCoding - error issueing DeviceRequest" );
        IOFree( MER->pData, lcLen );
        IOFree( MER, sizeof(IOUSBDevRequest) );
    }
#else
    if (port->BaudRate != port->LastBaudRate)
    {
        port->LastBaudRate = port->BaudRate;
        USBRequest( UBSA_SET_BAUDRATE, 230400 / port->BaudRate);
    }

    if (port->LastStopBits != port->StopBits)
    {
        UInt16 value;
        port->LastStopBits = port->StopBits;
        switch(port->StopBits) {
            case 2:
                value = 0; break;
            case 4:
                value = 1; break;
            default:
                value = 0; break;
        }
        USBRequest( UBSA_SET_STOP_BITS, value);
    }

    if (port->LastTX_Parity != port->TX_Parity)
    {
        UInt16 value;
        port->LastTX_Parity = port->TX_Parity;
        switch(port->TX_Parity) {
            case PD_RS232_PARITY_NONE:
                value = UBSA_PARITY_NONE; break;
            case PD_RS232_PARITY_EVEN:
                value = UBSA_PARITY_EVEN; break;
            case PD_RS232_PARITY_ODD:
                value = UBSA_PARITY_ODD; break;
            default:
                value = UBSA_PARITY_NONE; break;            
        }
        USBRequest( UBSA_SET_PARITY, value);
    }

    if (port->LastCharLength != port->CharLength)
    {
        UInt16 value;
        port->LastCharLength = port->CharLength;
        switch(port->CharLength) {
            case 8:
                value = 3; break;
            case 7:
                value = 2; break;
            case 6:
                value = 1; break;
            case 5:
                value = 0; break;
            default:
                value = 3; break;            
        }
        USBRequest( UBSA_SET_DATA_BITS, value);
    }

//    USBRequest( UBSA_SET_FLOW_CTRL, UBSA_FLOW_OCTS | UBSA_FLOW_IRTS);

#endif
}/* end USBSetLineCoding */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::USBSetControlLineState
//
//		Inputs:		RTS - true(set RTS), false(clear RTS), DTR - true(set DTR), false(clear DTR)
//
//		Outputs:	None
//
//		Desc:		Set up and send SetControlLineState Management Element Request(MER).
//
/****************************************************************************************************/

void BelkinF5U103Driver::USBSetControlLineState( bool RTS, bool DTR)
{
#if 0
	IOReturn		rc;
	IOUSBDevRequest	*MER;
	UInt16			CSBitmap = 0;
	
	ELG( 0, 0, 'USLC', "BelkinF5U103Driver::USBSetControlLineState" );
	
	// First check that Set Control Line State is supported
	
	if (!(fACMCapabilities & ACM_DeviceSuppControl))
	{
		return;
	}
	
	MER = (IOUSBDevRequest*)IOMalloc( sizeof(IOUSBDevRequest) );
	if ( !MER )
	{
		ELG( 0, 0, 'USL-', "BelkinF5U103Driver::USBSetControlLineState - allocate MER failed" );
		return;
	}
	bzero( MER, sizeof(IOUSBDevRequest) );
	
		// now build the Management Element Request
		
        MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
	MER->bRequest = kUSBSET_CONTROL_LINE_STATE;
	if ( RTS )
		CSBitmap |= kRTSOn;
	if ( DTR )
	 	CSBitmap |= kDTROn;
        MER->wValue = CSBitmap;
        MER->wIndex = fpCommInterfaceNumber;
	MER->wLength = 0;
	MER->pData = NULL;
	
	fMERCompletionInfo.parameter = MER;
	
	rc = fpDevice->DeviceRequest(MER, &fMERCompletionInfo);
	if ( rc == kIOReturnSuccess )
	{
//		fIOcount++;				// bump count of IO operations outstanding
	} else {
		ELG( MER->bRequest, rc, 'SLER', "BelkinF5U103Driver::USBSetControlLineState - error issueing DeviceRequest" );
		IOFree( MER, sizeof(IOUSBDevRequest) );
	}
#else
	ELG( RTS, DTR, 'USLC', "BelkinF5U103Driver::USBSetControlLineState" );

        USBRequest( UBSA_SET_RTS, RTS);
        USBRequest( UBSA_SET_DTR, DTR);
#endif
}/* end USBSetControlLineState */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::USBRequest
//
//		Inputs:		RTS - true(set RTS), false(clear RTS), DTR - true(set DTR), false(clear DTR)
//
//		Outputs:	None
//
//		Desc:		Set up and send SetControlLineState Management Element Request(MER).
//
/****************************************************************************************************/

void BelkinF5U103Driver::USBRequest( UInt8 request, UInt16 value)
{
	IOReturn		rc;
	IOUSBDevRequest	*MER;
	UInt16			CSBitmap = 0;
	
	ELG( request, value, 'USLC', "BelkinF5U103Driver::USBRequest" );
	
	// First check that Set Control Line State is supported
	
	if (!(fACMCapabilities & ACM_DeviceSuppControl))
	{
		return;
	}
	
	MER = (IOUSBDevRequest*)IOMalloc( sizeof(IOUSBDevRequest) );
	if ( !MER )
	{
		ELG( 0, 0, 'USL-', "BelkinF5U103Driver::USBRequest - allocate MER failed" );
		return;
	}
	bzero( MER, sizeof(IOUSBDevRequest) );
	
		// now build the Management Element Request
		
        MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBVendor, kUSBDevice);
	MER->bRequest = request;
#if 0
        MER->wValue = HostToUSBWord(value);
#else
        MER->wValue = value;
#endif
        MER->wIndex = fpCommInterfaceNumber;
	MER->wLength = 0;
	MER->pData = NULL;
	
	fMERCompletionInfo.parameter = MER;
	
	rc = fpDevice->DeviceRequest(MER, &fMERCompletionInfo);
	if ( rc == kIOReturnSuccess )
	{
//		fIOcount++;				// bump count of IO operations outstanding
	} else {
		ELG( MER->bRequest, rc, 'SLER', "BelkinF5U103Driver::USBRequest - error issueing DeviceRequest" );
		IOFree( MER, sizeof(IOUSBDevRequest) );
	}

}/* end USBRequest */


/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::USBSendBreak
//
//		Inputs:		sBreak - true(set Break), false(clear Break) - This may change
//
//		Outputs:	None
//
//		Desc:		Set up and send SendBreak Management Element Request(MER).
//
/****************************************************************************************************/

void BelkinF5U103Driver::USBSendBreak( bool sBreak)
{
#if 0
	IOReturn		rc;
	IOUSBDevRequest	*MER;
	
	ELG( 0, 0, 'USLC', "BelkinF5U103Driver::USBSendBreak" );
	
	// First check that Send Break is supported
	
	if (!(fACMCapabilities & ACM_DeviceSuppBreak))
	{
		return;
	}
	
	MER = (IOUSBDevRequest*)IOMalloc( sizeof(IOUSBDevRequest) );
	if ( !MER )
	{
		ELG( 0, 0, 'USL-', "BelkinF5U103Driver::USBSendBreak - allocate MER failed" );
		return;
	}
	bzero( MER, sizeof(IOUSBDevRequest) );
	
		// now build the Management Element Request
		
        MER->bmRequestType = USBmakebmRequestType(kUSBOut, kUSBClass, kUSBInterface);
	MER->bRequest = kUSBSET_CONTROL_LINE_STATE;
	if (sBreak)
	{
		MER->wValue = 0xFFFF;
	} else {
		MER->wValue = 0;
	}
        MER->wIndex = fpCommInterfaceNumber;
	MER->wLength = 0;
	MER->pData = NULL;
	
	fMERCompletionInfo.parameter = MER;
	
	rc = fpDevice->DeviceRequest(MER, &fMERCompletionInfo);
	if ( rc == kIOReturnSuccess )
	{
//		fIOcount++;				// bump count of IO operations outstanding
	} else {
		ELG( MER->bRequest, rc, 'SLER', "BelkinF5U103Driver::USBSendBreak - error issueing DeviceRequest" );
		IOFree( MER, sizeof(IOUSBDevRequest) );
	}
#else
        USBRequest( UBSA_SET_BREAK, sBreak);
#endif
}/* end USBSendBreak */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::SetStructureDefaults
//
//		Inputs:		port - the port to set the defaults, Init - Probe time or not
//
//		Outputs:	None
//
//		Desc:		Sets the defaults for the specified port structure
//
/****************************************************************************************************/

void BelkinF5U103Driver::SetStructureDefaults( PortInfo_t *port, bool Init )
{
    UInt32	tmp;
	
    ELG( 0, 0, 'StSD', "BelkinF5U103Driver::SetStructureDefaults" );

		/* These are set up at start and cannot get reset during execution.	*/
    if ( Init )
    {
		port->FCRimage 			= 0x00;
		port->IERmask 			= 0x00;

		port->State 			= ( PD_S_TXQ_EMPTY | PD_S_TXQ_LOW_WATER | PD_S_RXQ_EMPTY | PD_S_RXQ_LOW_WATER );
		port->WatchStateMask	 	= 0x00000000;
                port->serialRequestLock 	= 0;
    }

	port->BaudRate 			= kDefaultBaudRate;			// 9600 bps
	port->LastBaudRate 		= 0;
	port->CharLength 		= 8;						// 8 Data bits
	port->LastCharLength	= 0;
	port->StopBits 			= 2;						// 1 Stop bit
	port->LastStopBits		= 0;
	port->TX_Parity 		= 1;						// No Parity
	port->LastTX_Parity		= 0;
	port->RX_Parity 		= 1;						// --ditto--
	port->MinLatency 		= false;
	port->XONchar 			= '\x11';
	port->XOFFchar 			= '\x13';
	port->FlowControl 		= 0x00000000;
	port->RXOstate 			= IDLE_XO;
	port->TXOstate 			= IDLE_XO;
	port->FrameTOEntry 		= NULL;

//	port->RXStats.BufferSize 	= BUFFER_SIZE_DEFAULT;
	port->RXStats.BufferSize 	= kMaxCirBufferSize;
	port->RXStats.HighWater 	= (port->RXStats.BufferSize << 1) / 3;
	port->RXStats.LowWater	 	= port->RXStats.HighWater >> 1;

//	port->TXStats.BufferSize 	= BUFFER_SIZE_DEFAULT;
	port->TXStats.BufferSize 	= kMaxCirBufferSize;
	port->TXStats.HighWater 	= (port->RXStats.BufferSize << 1) / 3;
	port->TXStats.LowWater 		= port->RXStats.HighWater >> 1;

	port->FlowControl			= (DEFAULT_AUTO | DEFAULT_NOTIFY);
//	port->FlowControl 			= DEFAULT_NOTIFY;

	port->AreTransmitting 	= FALSE;

	for ( tmp=0; tmp < (256 >> SPECIAL_SHIFT); tmp++ )
		port->SWspecial[ tmp ] = 0;

	return;
	
}/* end SetStructureDefaults */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::freeRingBuffer
//
//		Inputs:		Queue - the specified queue to free
//
//		Outputs:	None
//
//		Desc:		Frees all resources assocated with the queue, then sets all queue parameters 
//					to safe values.
//
/****************************************************************************************************/

void BelkinF5U103Driver::freeRingBuffer( CirQueue *Queue )
{
    ELG( 0, Queue, 'f rb', "BelkinF5U103Driver::freeRingBuffer" );

    IOFree( Queue->Start, Queue->Size );
    CloseQueue( Queue );
	return;
	
}/* end freeRingBuffer */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::allocateRingBuffer
//
//		Inputs:		Queue - the specified queue to allocate, BufferSize - size to allocate
//
//		Outputs:	return Code - true (buffer allocated), false (it failed)
//
//		Desc:		Allocates resources needed by the queue, then sets up all queue parameters. 
//
/****************************************************************************************************/

bool BelkinF5U103Driver::allocateRingBuffer( CirQueue *Queue, size_t BufferSize )
{
    UInt8		*Buffer;

		// Size is ignored and kMaxCirBufferSize, which is 4096, is used.
		
    ELG( 0, BufferSize, 'alrb', "BelkinF5U103Driver::allocateRingBuffer" );
    Buffer = (UInt8*)IOMalloc( kMaxCirBufferSize );

    InitQueue( Queue, Buffer, kMaxCirBufferSize );

    if ( Buffer )
        return true;

	return false;
	
}/* end allocateRingBuffer */

/****************************************************************************************************/
//
//		Function:	BelkinF5U103Driver::message
//
//		Inputs:		type - message type, provider - my provider, argument - additional parameters
//
//		Outputs:	return Code - kIOReturnSuccess
//
//		Desc:		Handles IOKit messages. 
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::message( UInt32 type, IOService *provider,  void *argument )
{
	
    UInt8	i;
	
    ELG( 0, type, 'mess', "BelkinF5U103Driver::message" );
	
    switch ( type )
    {
        case kIOMessageServiceIsTerminated:
			ELG( fSessions, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceIsTerminated" );
			
			if ( fSessions )
			{
                            for (i=0; i<numberofPorts; i++)
                            {
                                if ( (fPorts[i] != NULL) && (fPorts[i]->serialRequestLock != 0) )
                                {
//						changeState( fPorts[i], 0, (UInt32)PD_S_ACTIVE );
                                }
                            }
                            if ( !fTerminate )		// Check if we're already being terminated
                            { 
                                KUNCUserNotificationDisplayNotice(
                                0,		// Timeout in seconds
                                0,		// Flags (for later usage)
                                "",		// iconPath (not supported yet)
                                "",		// soundPath (not supported yet)
                                "",		// localizationPath (not supported  yet)
                                "USB Modem Unplug Notice",		// the header
                                "The USB Modem has been unplugged while an application was still active. This can result in loss of data.",
                                "OK");
                            }
			} else {
				if ( fpCommInterface )	
				{
					fpCommInterface->close( this );	
					fpCommInterface->release();
					fpCommInterface = NULL;	
				}
	
            	fpDevice->close(this); 	// need to close so we can get the free and stop calls only if no sessions active (see releasePort)
			}
			
			fTerminate = true;		// we're being terminated (unplugged)
			break;
			
        case kIOMessageServiceIsSuspended: 	
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceIsSuspended" );
			break;
			
        case kIOMessageServiceIsResumed: 	
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceIsResumed" );
			break;
			
        case kIOMessageServiceIsRequestingClose: 
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceIsRequestingClose" ); 
			break;
			
        case kIOMessageServiceWasClosed: 	
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceWasClosed" ); 
			break;
			
        case kIOMessageServiceBusyStateChange: 	
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceBusyStateChange" ); 
			break;

        case kIOMessageServiceIsAttemptingOpen: 	
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - kIOMessageServiceIsAttemptingOpen" ); 
			break;

        default:
			ELG( 0, type, 'mess', "BelkinF5U103Driver::message - unknown message ???" ); 
			break;
    }
    
    return kIOReturnSuccess;
}

/****************************************************************************************************/
//
//		Function:	readPortState
//
//		Inputs:		port - the specified port
//
//		Outputs:	returnState - current state of the port
//
//		Desc:		Reads the current Port->State. 
//
/****************************************************************************************************/

UInt32 BelkinF5U103Driver::readPortState( PortInfo_t *port )
{
    UInt32	returnState = 0;
	
//	ELG( 0, port, 'rPSt', "readPortState" );
    // @@@ gvdl: copy the same self-defense code from changeState to here
    if ( port &&  port->serialRequestLock )
    {
//	ELG( state, mask, 'chSt', "changeState" );
        IOLockLock( port->serialRequestLock );
        returnState = port->State;
        IOLockUnlock( port->serialRequestLock);
    }

//	ELG( returnState, 0, 'rPS-', "readPortState" );

    return returnState;
	
}/* end readPortState */

/****************************************************************************************************/
//
//		Function:	changeState
//
//		Inputs:		port - the specified port, state - new state, mask - state mask (the specific bits)
//
//		Outputs:	None
//
//		Desc:		Change the current Port->State to state using the mask bits.
//					if mask = 0 nothing is changed.
//					delta contains the difference between the new and old state taking the
//					mask into account and it's used to wake any waiting threads as appropriate. 
//
/****************************************************************************************************/

void BelkinF5U103Driver::changeState( PortInfo_t *port, UInt32 state, UInt32 mask )
{
    UInt32				delta;
	
//    ELG( state, mask, 'chSt', "changeState" );

    IOLockLock( port->serialRequestLock );
    state = (port->State & ~mask) | (state & mask); // compute the new state
    delta = state ^ port->State;		    		// keep a copy of the diffs
    port->State = state;

    // Wake up all threads asleep on WatchStateMask
		
    if ( delta & port->WatchStateMask )
        thread_wakeup_with_result( &port->WatchStateMask, THREAD_RESTART );

    IOLockUnlock( port->serialRequestLock );

    ELG( port->State, delta, 'chSt', "changeState --> changeState" );
    return;
	
}/* end changeState */

/****************************************************************************************************/
//
//		Function:	privateWatchState
//
//		Inputs:		port - the specified port, state - state watching for, mask - state mask (the specific bits)
//
//		Outputs:	IOReturn - kIOReturnSuccess, kIOReturnIOError or kIOReturnIPCError
//
//		Desc:		Wait for the at least one of the state bits defined in mask to be equal
//					to the value defined in state. Check on entry then sleep until necessary.
//					A return value of kIOReturnSuccess means that at least one of the port state
//					bits specified by mask is equal to the value passed in by state.  A return
//					value of kIOReturnIOError indicates that the port went inactive.  A return
//					value of kIOReturnIPCError indicates sleep was interrupted by a signal. 
//
/****************************************************************************************************/

IOReturn BelkinF5U103Driver::privateWatchState( PortInfo_t *port, UInt32 *state, UInt32 mask )
{
    unsigned 			watchState, foundStates;
    bool 				autoActiveBit 	= false;
    IOReturn 			rtn 			= kIOReturnSuccess;

//    ELG( mask, *state, 'wsta', "privateWatchState" );

    watchState				= *state;
	IOLockLock( port->serialRequestLock );

		// hack to get around problem with carrier detection
		
    if ( *state | 0x40 )	/// mlj ??? PD_S_RXQ_FULL?
	{
        port->State |= 0x40;
    }

    if ( !(mask & (PD_S_ACQUIRED | PD_S_ACTIVE)) )
	{
        watchState &= ~PD_S_ACTIVE;	// Check for low PD_S_ACTIVE
        mask       |=  PD_S_ACTIVE;	// Register interest in PD_S_ACTIVE bit
        autoActiveBit = true;
    }

    for (;;)
	{
			// Check port state for any interesting bits with watchState value
			// NB. the '^ ~' is a XNOR and tests for equality of bits.
			
        foundStates = (watchState ^ ~port->State) & mask;

        if ( foundStates )
		{
            *state = port->State;
            if ( autoActiveBit && (foundStates & PD_S_ACTIVE) )
			{
                rtn = kIOReturnIOError;
            } else {
				rtn = kIOReturnSuccess;
			}
//			ELG( rtn, foundStates, 'FndS', "privateWatchState - foundStates" );
            break;
        }

			// Everytime we go around the loop we have to reset the watch mask.
			// This means any event that could affect the WatchStateMask must
			// wakeup all watch state threads.  The two events are an interrupt
			// or one of the bits in the WatchStateMask changing.
			
        port->WatchStateMask |= mask;

			// note: Interrupts need to be locked out completely here,
			// since as assertwait is called other threads waiting on
			// &port->WatchStateMask will be woken up and spun through the loop.
			// If an interrupt occurs at this point then the current thread
			// will end up waiting with a different port state than assumed
			//  -- this problem was causing dequeueData to wait for a change in
			// PD_E_RXQ_EMPTY to 0 after an interrupt had already changed it to 0.

        assert_wait( &port->WatchStateMask, true );	/* assert event */

        IOLockUnlock( port->serialRequestLock );
        rtn = thread_block( THREAD_CONTINUE_NULL );			/* block ourselves */
        IOLockLock( port->serialRequestLock );

        if ( rtn == THREAD_RESTART )
		{
			continue;
		} else {
            rtn = kIOReturnIPCError;
            break;
        }
    }/* end for */

	    	// As it is impossible to undo the masking used by this
	    	// thread, we clear down the watch state mask and wakeup
	    	// every sleeping thread to reinitialize the mask before exiting.
		
    port->WatchStateMask = 0;

    thread_wakeup_with_result( &port->WatchStateMask, THREAD_RESTART );
    IOLockUnlock( port->serialRequestLock);
	
	//    ELG( rtn, *state, 'wEnd', "privateWatchState end" );
	
    return rtn;
	
}/* end privateWatchState */