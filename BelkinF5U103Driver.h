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
 
#define DEBUG				0			// for debugging
#define USE_ELG				0			// to event log - DEBUG must also be set
#define kEvLogSize			(4096*16)		// 16 pages = 64K = 4096 events
#define	LOG_DATA			0			// logs data to the IOLog - DEBUG must also be set

#define Sleep_Time			20

#if DEBUG
	#if USE_ELG
		#define ELG(A,B,ASCI,STRING)    EvLog( (UInt32)(A), (UInt32)(B), (UInt32)(ASCI), STRING )		
	#else /* not USE_ELG */
		#define ELG(A,B,ASCI,STRING)	{IOLog( "BelkinF5U103Driver: %8x %8x " STRING "\n", (unsigned int)(A), (unsigned int)(B) );IOSleep(Sleep_Time);}
	#endif /* USE_ELG */
	#if LOG_DATA
		#define LogData(D, C, b)	USBLogData((UInt8)D, (UInt32)C, (char *)b)
	#else /* not LOG_DATA */
		#define LogData(D, C, b)
	#endif /* LOG_DATA */
#else /* not DEBUG */
	#define ELG(A,B,ASCI,S)
	#define LogData(D, C, b)
	#undef USE_ELG
	#undef LOG_DATA
#endif /* DEBUG */

#define IOLogIt(A,B,ASCI,STRING)	IOLog( "BelkinF5U103Driver: %8x %8x " STRING "\n", (unsigned int)(A), (unsigned int)(B) )

#define baseName			"F5U103"
#define defaultName			"Belkin USB Serial"
#define numberofPorts			1			// Number of ports we'll support (it's really only 1 at the moment)
#define productNameLength		32			// Arbitrary length
#define propertyTag			"F5U103"

/* USB CDC Defintions	*/
#if 0		
#define kUSBAbstractControlModel	2
#define kUSBv25				1		
	
#define kUSBbRxCarrier			0x01			// Carrier Detect
#define kUSBDCD				kUSBbRxCarrier
#define kUSBbTxCarrier			0x02			// Data Set Ready
#define kUSBDSR				kUSBbTxCarrier
#define kUSBbBreak			0x04
#define kUSBbRingSignal			0x08
#define kUSBbFraming			0x10
#define kUSBbParity			0x20
#define kUSBbOverRun			0x40

#define kDTROff				0
#define kRTSOff				0
#define kDTROn				1
#define kRTSOn				2
#endif

/* UBSA Defintions	*/
#define	UBSA_SET_BAUDRATE  		0x00
#define	UBSA_SET_STOP_BITS		0x01
#define	UBSA_SET_DATA_BITS		0x02
#define	UBSA_SET_PARITY			0x03
#define	UBSA_SET_DTR			0x0A
#define	UBSA_SET_RTS			0x0B
#define	UBSA_SET_BREAK			0x0C
#define	UBSA_SET_FLOW_CTRL		0x10

#define	UBSA_PARITY_NONE		0x00
#define	UBSA_PARITY_EVEN		0x01
#define	UBSA_PARITY_ODD			0x02
#define	UBSA_PARITY_MARK		0x03
#define	UBSA_PARITY_SPACE		0x04

#define	UBSA_FLOW_NONE			0x0000
#define	UBSA_FLOW_OCTS			0x0001
#define	UBSA_FLOW_ODSR			0x0002
#define	UBSA_FLOW_IDSR			0x0004
#define	UBSA_FLOW_IDTR			0x0008
#define	UBSA_FLOW_IRTS			0x0010
#define	UBSA_FLOW_ORTS			0x0020
#define	UBSA_FLOW_UNKNOWN		0x0040
#define	UBSA_FLOW_OXON			0x0080
#define	UBSA_FLOW_IXON			0x0100

/* line status register */
#define	UBSA_LSR_TSRE			0x40	/* Transmitter empty: byte sent */
#define	UBSA_LSR_TXRDY			0x20	/* Transmitter buffer empty */
#define	UBSA_LSR_BI			0x10	/* Break detected */
#define	UBSA_LSR_FE			0x08	/* Framing error: bad stop bit */
#define	UBSA_LSR_PE			0x04	/* Parity error */
#define	UBSA_LSR_OE			0x02	/* Overrun, lost incoming byte */
#define	UBSA_LSR_RXRDY			0x01	/* Byte ready in Receive Buffer */
#define	UBSA_LSR_RCV_MASK		0x1f	/* Mask for incoming data or error */

/* modem status register */
/* All deltas are from the last read of the MSR. */
#define	UBSA_MSR_DCD			0x80	/* Current Data Carrier Detect */
#define	UBSA_MSR_RI			0x40	/* Current Ring Indicator */
#define	UBSA_MSR_DSR			0x20	/* Current Data Set Ready */
#define	UBSA_MSR_CTS			0x10	/* Current Clear to Send */
#define	UBSA_MSR_DDCD			0x08	/* DCD has changed state */
#define	UBSA_MSR_TERI			0x04	/* RI has toggled low to high */
#define	UBSA_MSR_DDSR			0x02	/* DSR has changed state */
#define	UBSA_MSR_DCTS			0x01	/* CTS has changed state */

	enum
	{
		kUSBSEND_ENCAPSULATED_COMMAND		= 0,			// Requests
		kUSBGET_ENCAPSULATED_RESPONSE		= 1,
		kUSBSET_COMM_FEATURE 			= 2,
		kUSBGET_COMM_FEATURE 			= 3,
		kUSBCLEAR_COMM_FEATURE 			= 4,
		kUSBSET_LINE_CODING 			= 0x20,
		kUSBGET_LINE_CODING 			= 0x21,
		kUSBSET_CONTROL_LINE_STATE 		= 0x22,
		kUSBSEND_BREAK 				= 0x23
	};
	
	enum
	{
		kUSBNETWORK_CONNECTION 			= 0,			// Notifications
		kUSBRESPONSE_AVAILABLE 			= 1,
		kUSBSERIAL_STATE 			= 0x20
	};
	
	typedef struct
	{	
		UInt32			dwDTERate;
		UInt8			bCharFormat;
		UInt8			bParityType;
		UInt8			bDataBits;
		
	} LineCoding;
	
#define dwDTERateOffset			0

#define wValueOffset			2
#define wIndexOffset			4
#define wLengthOffset			6

	enum
	{
		CS_INTERFACE				= 0x24,
		
		Header_FunctionalDescriptor		= 0x00,
		CM_FunctionalDescriptor			= 0x01,
		ACM_FunctionalDescriptor		= 0x02,
		Union_FunctionalDescriptor		= 0x06,
		CS_FunctionalDescriptor			= 0x07,
		
		CM_ManagementData			= 0x01,
		CM_ManagementOnData			= 0x02,
		
		ACM_DeviceSuppCommFeature		= 0x01,
		ACM_DeviceSuppControl			= 0x02,
		ACM_DeviceSuppBreak			= 0x04,
		ACM_DeviceSuppNetConnect		= 0x08
	};
	
	typedef struct 
	{
		UInt8			bFunctionLength;
		UInt8		 	bDescriptorType;
		UInt8		 	bDescriptorSubtype;
	} FunctionalDescriptorHeader;

	typedef struct 
	{
		UInt8			bFunctionLength;
		UInt8 			bDescriptorType;
		UInt8 			bDescriptorSubtype;
		UInt8 			bmCapabilities;
		UInt8 			bDataInterface;
	} CMFunctionalDescriptor;
	
	typedef struct 
	{
		UInt8			bFunctionLength;
		UInt8 			bDescriptorType;
		UInt8 			bDescriptorSubtype;
		UInt8 			bmCapabilities;
	} ACMFunctionalDescriptor;

		/* Globals	*/

        typedef struct globals      /* Globals for this module (not per instance)   */
        {
                UInt32			evLogFlag; // debugging only
                UInt8			*evLogBuf;
                UInt8			*evLogBufe;
                UInt8			*evLogBufp;
                UInt8			intLevel;
		class			BelkinF5U103Driver	*BelkinF5U103DriverInstance;
        } globals;

		/* SccTypes.h	*/

	enum InterruptAssignments
	{
		kIntChipSet				= 0,
		kIntTxDMA,
		kIntRxDMA
	};
		
	enum ParityType
	{
		NoParity 				= 0,
		OddParity,
		EvenParity
	};
	
#define kDefaultBaudRate		9600
#define kMaxBaudRate			230400		// experimenting with higher speeds hul
#define kMaxCirBufferSize		4096

		/* SccQueuePrimatives.h	*/

	typedef struct CirQueue
	{
		UInt8			*Start;
		UInt8			*End;
		UInt8			*NextChar;
		UInt8			*LastChar;
		size_t			Size;
		size_t			InQueue;
	} CirQueue;

	typedef enum QueueStatus
	{
		queueNoError				= 0,
		queueFull,
		queueEmpty,
		queueMaxStatus
	} QueueStatus;

	/* PPCSerialPort.h	*/

#define BIGGEST_EVENT			3
//#define BUFFER_SIZE_DEFAULT		1200

#define SPECIAL_SHIFT			(5)
#define SPECIAL_MASK			((1<<SPECIAL_SHIFT) - 1)
#define STATE_ALL			( PD_RS232_S_MASK | PD_S_MASK )
#define FLOW_RX_AUTO   	 		( PD_RS232_A_RFR | PD_RS232_A_DTR | PD_RS232_A_RXO )
#define FLOW_TX_AUTO    		( PD_RS232_A_CTS | PD_RS232_A_DSR | PD_RS232_A_TXO | PD_RS232_A_DCD )
#define CAN_BE_AUTO			( FLOW_RX_AUTO | FLOW_TX_AUTO )
#define CAN_NOTIFY			( PD_RS232_N_MASK )
#define EXTERNAL_MASK   		( PD_S_MASK | (PD_RS232_S_MASK & ~PD_RS232_S_LOOP) )
#define INTERNAL_DELAY  		( PD_RS232_S_LOOP )
#define DEFAULT_AUTO			( PD_RS232_A_DTR | PD_RS232_A_RFR | PD_RS232_A_CTS | PD_RS232_A_DSR )
#define DEFAULT_NOTIFY			0x00
#define DEFAULT_STATE			( PD_S_TX_ENABLE | PD_S_RX_ENABLE | PD_RS232_A_TXO | PD_RS232_A_RXO )

#define IDLE_XO	   			0
#define NEEDS_XOFF 			1
#define SENT_XOFF 			-1
#define NEEDS_XON  			2
#define SENT_XON  			-2

#define MAX_BLOCK_SIZE			PAGE_SIZE
#define COMM_BUFF_SIZE			16

	typedef struct
	{
		UInt32	ints;
		UInt32	txInts;
		UInt32	rxInts;
		UInt32	mdmInts;
		UInt32	txChars;
		UInt32	rxChars;
	} Stats_t;

	typedef struct BufferMarks
	{
		unsigned long		BufferSize;
		unsigned long		HighWater;
		unsigned long		LowWater;
		bool			OverRun;
	} BufferMarks;

	typedef struct
	{
		UInt32			State;
		UInt32			WatchStateMask;
		IOLock			*serialRequestLock;

			// queue control structures:
			
		CirQueue		RX;
		CirQueue		TX;

		BufferMarks		RXStats;
		BufferMarks		TXStats;
	
			// UART configuration info:
			
		UInt32			CharLength;
		UInt32			StopBits;
		UInt32			TX_Parity;
		UInt32			RX_Parity;
		UInt32			BaudRate;
		UInt8			FCRimage;
		UInt8			IERmask;
		bool			MinLatency;
	
			// flow control state & configuration:
			
		UInt8			XONchar;
		UInt8			XOFFchar;
		UInt32			SWspecial[ 0x100 >> SPECIAL_SHIFT ];
		UInt32			FlowControl;	// notify-on-delta & auto_control
		
		int			RXOstate;    /* Indicates our receive state.	*/
		int			TXOstate;	 /* Indicates our transmit state, if we have received any Flow Control.	*/
	
		IOThread		FrameTOEntry;
	
		mach_timespec		DataLatInterval;
		mach_timespec		CharLatInterval;
	
		bool			AreTransmitting;
	
			/* extensions to handle the Driver */
			
		bool 			isDriver;
		void			*DriverPowerRegister;
		UInt32 			DriverPowerMask;
		
		UInt32			LastCharLength;
		UInt32			LastStopBits;
		UInt32			LastTX_Parity;
		UInt32			LastBaudRate;
		
	} PortInfo_t;
	
	/* Inline time conversions */
	
static inline unsigned long tval2long( mach_timespec val )
{
   return (val.tv_sec * NSEC_PER_SEC) + val.tv_nsec;
   
}

static inline mach_timespec long2tval( unsigned long val )
{
	mach_timespec	tval;

	tval.tv_sec  = val / NSEC_PER_SEC;
	tval.tv_nsec = val % NSEC_PER_SEC;
	return tval;
	
}

	/* BelkinF5U103Driver.h - This file contains the class definition for the	*/
	/* USB Communication Device Class (CDC) sample driver.				*/

class BelkinF5U103Driver : public IOSerialDriverSync
{
    OSDeclareDefaultStructors( BelkinF5U103Driver );	/* Constructor & Destructor stuff	*/

private:
//	UInt32				fIOcount;		// number of pipe IO operations outstanding
	UInt32				fCount;
	UInt16				fModemState;	// last serial state recieived from the modem
	UInt8				fSessions;		// Active sessions
        bool				fTerminate;		// Are we being terminated (ie the device was unplugged)
	UInt8				fCMCapabilities;	// Call Management Capabilities
	UInt8				fACMCapabilities;	// Abstract Control Management Capabilities
	UInt8				fProductName[productNameLength];	// Actually the product String from the Device
	PortInfo_t 			*fPorts[numberofPorts];		// Port array

	IOBufferMemoryDescriptor	*fpCommPipeMDP;
	IOBufferMemoryDescriptor	*fpPipeInMDP;
	IOBufferMemoryDescriptor	*fpPipeOutMDP;

	UInt8				*fCommPipeBuffer;
	UInt8				*fPipeInBuffer;
	UInt8				*fPipeOutBuffer;

	IOUSBCompletion			fCommCompletionInfo;
	IOUSBCompletion			fReadCompletionInfo;
	IOUSBCompletion			fWriteCompletionInfo;
	IOUSBCompletion			fMERCompletionInfo;

	static void			commReadComplete(  void *obj, void *param, IOReturn ior, UInt32 remaining );
	static void			dataReadComplete(  void *obj, void *param, IOReturn ior, UInt32 remaining );
	static void			dataWriteComplete( void *obj, void *param, IOReturn ior, UInt32 remaining );
	static void			merWriteComplete( void *obj, void *param, IOReturn ior, UInt32 remaining );

public:

	IOUSBDevice			*fpDevice;

	IOUSBInterface			*fpCommInterface;
	UInt8				fpCommInterfaceNumber;

	IOUSBPipe			*fpInPipe;
	IOUSBPipe			*fpOutPipe;
	IOUSBPipe			*fpCommPipe;

		/* IOKit methods:	*/
		
	virtual void			free( void );
	virtual bool			start( IOService *provider );
	virtual void			stop( IOService *provider );
        virtual IOReturn 		message( UInt32 type, IOService *provider,  void *argument = 0 );

		/**** IOSerialDriverSync Abstract Method Implementation	****/

	virtual IOReturn		acquirePort( bool sleep, void *refCon );
	virtual IOReturn		releasePort( void *refCon );
	virtual IOReturn		setState( UInt32 state, UInt32 mask, void *refCon );
	virtual UInt32			getState( void *refCon );
	virtual IOReturn		watchState( UInt32 *state, UInt32 mask, void *refCon );
	virtual UInt32			nextEvent( void *refCon );
	virtual IOReturn		executeEvent( UInt32 event, UInt32 data, void *refCon );
	virtual IOReturn		requestEvent( UInt32 event, UInt32 *data, void *refCon );
	virtual IOReturn		enqueueEvent( UInt32 event, UInt32 data, bool sleep, void *refCon );
	virtual IOReturn		dequeueEvent( UInt32 *event, UInt32 *data, bool sleep, void *refCon );
	virtual IOReturn		enqueueData( UInt8 *buffer, UInt32 size, UInt32 * count, bool sleep, void *refCon );
	virtual IOReturn		dequeueData( UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon );
												
	    /**** IOSerialDriverSync Abstract Method Implementation	****/
	
	bool 				allocateResources( PortInfo_t *port );
	void				releaseResources( PortInfo_t *port );
	bool 				configureDevice( UInt8 numConfigs );
	bool 				createSuffix( unsigned char *sufKey );
	bool				createSerialStream();
	bool 				SetUpTransmit( PortInfo_t *Channel );
	void 				StartTransmission( PortInfo_t *Channel );
	void 				USBSetLineCoding( PortInfo_t *port );
	void 				USBSetControlLineState( bool RTS, bool DTR);
	void 				USBSendBreak( bool sBreak);
	void 				SetStructureDefaults( PortInfo_t *port, bool Init );
	void 				freeRingBuffer( CirQueue *Queue );
	bool 				allocateRingBuffer( CirQueue *Queue, size_t BufferSize );

private:

	// QueuePrimatives
	static QueueStatus		AddBytetoQueue( CirQueue *Queue, char Value );
	static QueueStatus		GetBytetoQueue( CirQueue *Queue, UInt8 *Value );
	static QueueStatus		InitQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size );
	static QueueStatus		CloseQueue( CirQueue *Queue );
	static size_t			AddtoQueue( CirQueue *Queue, UInt8 *Buffer, size_t Size );
	static size_t			RemovefromQueue( CirQueue *Queue, UInt8 *Buffer, size_t MaxSize );
	static size_t			FreeSpaceinQueue( CirQueue *Queue );
	static size_t			UsedSpaceinQueue( CirQueue *Queue );
	static size_t			GetQueueSize( CirQueue *Queue );
	static QueueStatus		GetQueueStatus( CirQueue *Queue );
	static void			CheckQueues( PortInfo_t *port );

	static IOReturn			privateWatchState( PortInfo_t *port, UInt32 *state, UInt32 mask );
	static UInt32			readPortState( PortInfo_t *port );
	static void			changeState( PortInfo_t *port, UInt32 state, UInt32 mask );

        void				USBRequest( UInt8 request, UInt16 value);
}; /* end class BelkinF5U103Driver */

