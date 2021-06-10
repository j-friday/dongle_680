/*************************************************************
 * @file        Mu_mcp.c
 * @brief       code of USB multifunctional composite peripheral of BK3435_v2
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par
 * @attention
 *
 * @history     2016-09-29 gwf    create this file
 */

/*
 * multifunctional composite peripheral
 */

#include "mu_arch.h"
#include "mu_cdi.h"
#include "mu_diag.h"
#include "mu_mem.h"
#include "sdp_service_task.h"
#include "reg_blecore.h"
#include "user_config.h"
#include "app.h"
#include "flash.h"

//int uart_printf(const char *fmt,...);
/******************************************************************
Defines
******************************************************************/
#define STATIC
//#define STATIC static
//#define __ISOC_16K__


//#define MUSIC_48KHZ_2CH_16BIT
//#define MUSIC_24KHZ_2CH_16BIT

#if USB_APP_CONFIG
#define INTERFACE_NUM           3
#else
#define INTERFACE_NUM           1
#endif

#define HIDS_KB_REPORT_ID       1
#define HIDS_MOUSE_REPORT_ID    5
#define RMC_VENDOR_REPORT_ID_1  0xFD
#define RMC_VENDOR_REPORT_ID_2  0x1E
#define ISOC_VENDOR_REPORT_ID_1 0x5A
#define ISOC_VENDOR_REPORT_ID_2 0x6B
#define HIDS_MM_KB_REPORT_ID    3
#define HIDS_PWR_KB_REPORT_ID   4

#define RMC_SENSORS_DATA_REPORT_ID  0x32
#define OUTPUT_REPORT   	    0xBA


/******************************************************************
Forwards
******************************************************************/
STATIC uint8_t MGC_McpDeviceRequest(void* hClient, MUSB_BusHandle hBus,
				    uint32_t dwSequenceNumber,
				    const uint8_t* pSetup,
				    uint16_t wRequestLength);
STATIC uint8_t MGC_McpDeviceConfigSelected(void* hClient,
					   MUSB_BusHandle hBus,
					   uint8_t bConfigurationValue,
					   MUSB_Pipe* ahPipe);
STATIC void MGC_McpNewUsbState(void* hClient, MUSB_BusHandle hBus,
			       MUSB_State State);

/******************************************************************
Globals
******************************************************************/

/* UCDI variables */
STATIC uint8_t MGC_bMcpSelfPowered = FALSE;
STATIC MUSB_State MGC_eMcpUsbState = MUSB_POWER_OFF;

STATIC uint8_t MGC_aControlData[256];

const uint8_t gHidKeyboardReportDescriptor[] =
{
	// MicoSoft Keyboard
	0x05, 0x01, 
	0x09, 0x06, 
	0xA1, 0x01, 
	0x85, HIDS_KB_REPORT_ID,	//0x02, 
	0x15, 0x00, 
	0x25, 0x01, 
	0x05, 0x07, 
	0x1A, 0xE0, 0x00,
	0x2A, 0xE7, 0x00, 
	0x75, 0x01, 
	0x95, 0x08, 
	0x81, 0x02, 
	0x05, 0x07, 
	0x19, 0x00, 
	0x2A, 0x91, 0x00, 
	0x16, 0x00, 0x00, 
	0x26, 0xFF, 0x00, 
	0x75, 0x08, 
	0x95, 0x0A, 
	0x81, 0x00, 
	0xC0,
};
unsigned long ulgHidKeyboardReportDescriptorLen = sizeof(gHidKeyboardReportDescriptor);

#if 0
const uint8_t gHidMouseReportDescriptor[] =
{
};
unsigned long ulgHidMouseReportDescriptorLen = sizeof(gHidMouseReportDescriptor);

const uint8_t gHidISOCVoiceReportDescriptor[] =
{
};

unsigned long ulgHidISOCVoiceReportDescriptorLen = sizeof(gHidISOCVoiceReportDescriptor);

const uint8_t gIsocCurrentDescriptor[] =
{
};

unsigned long ulgIsocCurrentDescriptorLen = sizeof(gIsocCurrentDescriptor);
#endif
STATIC CONST uint8_t MGC_aDescriptorData[] =
{
    /* Device Descriptor */
    0x12,                      /* bLength              */
    MUSB_DT_DEVICE,            /* DEVICE               */
    0x00,0x02,                 /* USB 1.0              */
    0x00,                      /* CLASS                */
    0x00,                      /* Subclass             */
    0x00,                      /* Protocol             */
    0x40,                      /* bMaxPktSize0         */    
    0x61,0x8F,                 /* idVendor             */
    0x25,0xF8,                 /* idProduct            */
    //0xc4,0x10,                 /* idVendor             */
   	//0x33,0x00,                 /* idProduct            */       
    //0x00,0x02, //by gwf        /* bcdDevice            */
    0x10,0x01,                 /* bcdDevice            */
    0x01,                      /* iManufacturer        */
    0x02,                      /* iProduct             */
    0x03,                      /* iSerial Number       */
    0x01,                      /* One configuration    */

	/*×Ö·û´®ÃèÊö·û*/
	// ÓïÑÔÃèÊö·û
    /* strings */
    0x04,
    MUSB_DT_STRING,
    0x09, 0x04,			/* English (U.S.) */

    /* TODO: make tool to generate strings and eventually whole descriptor! */
    /* English (U.S.) strings */
    0x24,			/* Manufacturer: Mentor Graphics */
    MUSB_DT_STRING,
    'b', 0, 'e', 0, 'k', 0, 'e', 0, 'n', 0, ' ', 0,
    'c', 0, 'o', 0, 'r', 0, 'p', 0, 'o', 0, 'r', 0, 
    'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0,

    0x12,			/* Product ID: Keyboard */
    MUSB_DT_STRING,
    'K', 0, 'e', 0, 'y', 0, 'b', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0,

    0x1a,			/* Serial #: 123412341234 */
    MUSB_DT_STRING,
    '1', 0, '2', 0, '3', 0, '4', 0,
    '5', 0, '6', 0, '7', 0, '8', 0,
    '9', 0, 'a', 0, 'b', 0, 'c', 0,


	/*ÅäÖÃÃèÊö·û*/
    0x09,                               // Length
    MUSB_DT_CONFIG,                               // Type
    //LEN_USB_CONFIGURATION_DESCRIPTOR&0xff, // Totallength = 9+(9+9+7)*3+214+25
    //LEN_USB_CONFIGURATION_DESCRIPTOR>>8,
    0x24,0x00,//    0x11,0x01,//0xF8,0x00
    0x01,                               // NumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0xa0,                               // bmAttributes
    0x32,                               // MaxPower (in 2mA units)

  

    // ------ Keyboard ---------------------
    0x09,                                   /* bLength              */
    0x04,                                   /* INTERFACE            */
    0x00,                                   /* bInterfaceNumber     */
    0x00,                                   /* bAlternateSetting    */
    0x01,                                   /* bNumEndpoints        */
    0x03,                                   /* bInterfaceClass      */
    0x01,                                   /* bInterfaceSubClass (1=RBC, 6=SCSI) */
    0x01,                               // bInterfaceProcotol // keyboard:mouse = 1:2
    0x00,                                   /* iInterface           */
	
    /*HID Descriptor*/
    0x09,                      // the size of this descriptor
    0x21,                      // hid descriptor type
    0x10,0x01,                          // bcdHID
    0x00,                      // the country code
    0x01,                      // the number of class descriptor
    0x22,                      // report descriptor
    0x2E,0x00,//HID_MEDIA_REPORT_DESCRIPTOR_SIZE>>8
    //0x00,
      
    /* Endpoint Descriptor  : In */
    0x07,                      /* bLength              */
    0x05,                      /* ENDPOINT             */
    0x82,                      /* bEndpointAddress     */
    0x03,                      /* bmAttributes         */
    0x40,0x00,                 // MaxPacketSize (LITTLE ENDIAN)    
    0x01,                      // bInterval
    
};

unsigned long ulMGC_aDescriptorDataLen = sizeof(MGC_aDescriptorData);
const uint8_t *pMGC_aDescriptorData = MGC_aDescriptorData;

STATIC CONST uint8_t MGC_aHighSpeedDescriptorData[] =
{
    /* device qualifier */
    0x0a,                      /* bLength              */
    MUSB_DT_DEVICE_QUALIFIER,  /* DEVICE Qualifier     */
    0x00,0x02,                 /* USB 2.0              */
    0,                         /* CLASS                */
    0,                         /* Subclass             */
    0x00,                      /* Protocol             */
    0x40,                      /* bMaxPacketSize0      */
    0x01,                      /* One configuration    */
    0x00,                      /* bReserved            */

    0x09,                               // Length
    MUSB_DT_OTHER_SPEED,                // Type
    //LEN_USB_CONFIGURATION_DESCRIPTOR&0xff, // Totallength = 9+(9+9+7)*3+214+25
    //LEN_USB_CONFIGURATION_DESCRIPTOR>>8,
    0x24,0x00,//    0x11,0x01,//0xF8,0x00
    0x01,                               // NumInterfaces
    0x01,                               // bConfigurationValue
    0x00,                               // iConfiguration
    0xa0,                               // bmAttributes
    0x32,                               // MaxPower (in 2mA units)

    // ------ Keyboard ---------------------
    0x09,                               // bLength
    0x04,                               // bDescriptorType
    0x00,                               // bInterfaceNumber
    0x00,                               // bAlternateSetting
    0x01,                               // bNumEndpoints
    0x03,                               // bInterfaceClass (3 = HID)
    0x01,                               // bInterfaceSubClass
    0x01,                               // bInterfaceProcotol // keyboard:mouse = 1:2
    0x00,                               // iInterface

    0x09,                               // bLength
    0x21,                               // bDescriptorType
    0x10,0x01,                          // bcdHID
    0x00,                               // bCountryCode
    0x01,                               // bNumDescriptors
    0x22,                               // bDescriptorType
    0x2E,0x00,//0x26,sizeof(gHidKeyboardReportDescriptor),      

    // IN endpoint (mandatory for HID)
    0x07,                               // bLength
    0x05,                               // bDescriptorType
    0x82,                               // bEndpointAddress
    0x03,                               // bmAttributes
    0x40,0x00,                          // MaxPacketSize (LITTLE ENDIAN)    
    0x01,                               // bInterval
};

unsigned long ulMGC_aHighSpeedDescriptorDataLen = sizeof(MGC_aHighSpeedDescriptorData);
const uint8_t *pMGC_aHighSpeedDescriptorData = MGC_aHighSpeedDescriptorData;


STATIC uint8_t MGC_bMcpInterface = 0;

// by gwf   STATIC uint8_t MGC_aJunk[1];//[512];
STATIC uint8_t MGC_aJunk[64];
uint8_t MGC_RX_Buffer[64];
uint8_t MGC_TX_Buffer[64];


/** IRP for tx isoc data transmission */
STATIC MUSB_IsochIrp MGC_McpIsocTxDataIrp =
{
    NULL,   /* hPipe */
    NULL,   /* pBuffer */
    0,   /* adwLength */
    0,   /* adwStatus */
    0,   /* adwActualLength */
    NULL,   /* pfIrpComplete */
    NULL,   /* pCompleteParam */
    0,      /* wFrameCount */
    0,      /* wCurrentFrame */
    0,      /* wCallbackInterval */
    TRUE,	/* bIsrCallback */
    FALSE	/* bAllowDma */
};

STATIC MUSB_Irp MGC_McpUsbRxDataIrp = 
{
    NULL,
    NULL,
    0,
    0,
    0,
    NULL,
    NULL,
    FALSE,	/* bAllowShortTransfer */
    TRUE,	/* bIsrCallback */
    FALSE	/* bAllowDma */
};

/** IRP for mouse data transmission */
STATIC MUSB_Irp MGC_McpMouseTxDataIrp =
{
    NULL,
    NULL,
    0,
    0,
    0,
    NULL,
    NULL,
    FALSE,	/* bAllowShortTransfer */
    TRUE,	/* bIsrCallback */
    FALSE	/* bAllowDma */
};

/** IRP for keyboard data transmission */
STATIC MUSB_Irp MGC_McpKeyboardTxDataIrp =
{
    NULL,
    NULL,
    0,
    0,
    0,
    NULL,
    NULL,
    FALSE,	/* bAllowShortTransfer */
    TRUE,	/* bIsrCallback */
    FALSE	/* bAllowDma */
};


/*
* registration
*/
#if 1
MUSB_FunctionClient MGC_McpFunctionClient =
{
    NULL,	/* no instance data; we are singleton */
    MGC_aDescriptorData,
    sizeof(MGC_aDescriptorData),
    3,		/* strings per language */
    MGC_aHighSpeedDescriptorData,//
    sizeof(MGC_aHighSpeedDescriptorData),
    sizeof(MGC_aControlData),
    MGC_aControlData,
    &MGC_bMcpSelfPowered,
    MGC_McpDeviceRequest,
    MGC_McpDeviceConfigSelected,
    NULL,
    MGC_McpNewUsbState
};
#endif


////////////// for user ///////////////////////
STATIC volatile uint8_t b_isConnected = FALSE;
STATIC volatile uint8_t b_isTRxing = FALSE;

void test_usb_device(void);
#if 0
STATIC void USBD_IsocTxDataComplete(void* pCompleteParam, MUSB_IsochIrp* pIrp)
{
	b_isTRxing = FALSE;
	
	test_usb_device();
	os_printf("USBD_IsocTxDataComplete\r\n");
}

STATIC uint32_t USBD_MouseTxDataComplete(void* pCompleteParam, MUSB_Irp* pIrp)
{
	b_isTRxing = FALSE;
	os_printf("USBD_MouseTxDataComplete\r\n");
    test_usb_device();  ///190404
	return 0;
}
#endif
STATIC uint32_t USBD_KeyboardTxDataComplete(void* pCompleteParam, MUSB_Irp* pIrp)
{
	b_isTRxing = FALSE;
	//bk_printf("USBD_KeyboardTxDataComplete\r\n");
    test_usb_device();  ///190404
    //UART_PRINTF("USBD_KeyboardTxDataComplete \r\n");
	return 0;
}
#if 0
void USBD_IsocStartTx(unsigned char *pBuf, unsigned long ulLen)
{
	unsigned char *pBufTemp = pBuf;
	unsigned long ulLenTemp;
	os_printf("USBD_IsocStartTx ---1\r\n");
	do
	{
		if (b_isTRxing == TRUE)
		{
			return;
		}
        os_printf("USBD_IsocStartTx ---2\r\n");
		ulLenTemp = MIN(ulLen, 64);
		b_isTRxing = TRUE;
		MGC_McpIsocTxDataIrp.pBuffer = pBufTemp;
		MGC_McpIsocTxDataIrp.adwLength = ulLenTemp;
		MGC_McpIsocTxDataIrp.adwActualLength = 0;
		MGC_McpIsocTxDataIrp.pfIrpComplete = USBD_IsocTxDataComplete;
		MUSB_ScheduleIsochTransfer(0, &MGC_McpIsocTxDataIrp);
		ulLen -= ulLenTemp;
		pBufTemp += ulLenTemp;
	} while (ulLen);
	os_printf("USBD_IsocStartTx ---3\r\n");
}

void USBD_MouseStartTx(unsigned char *pBuf, unsigned long ulLen)
{
	unsigned char *pBufTemp = pBuf;
	unsigned long ulLenTemp;
	do
	{
		if (b_isTRxing == TRUE)
		{
			return;
		}
		ulLenTemp = MIN(ulLen, 64);
		b_isTRxing = TRUE;
#if 0
		memcpy(MGC_aJunk, pBufTemp, ulLenTemp);
		MGC_McpMouseTxDataIrp.pBuffer = MGC_aJunk;
#else
		MGC_McpMouseTxDataIrp.pBuffer = pBufTemp;
#endif
		MGC_McpMouseTxDataIrp.dwLength = ulLenTemp;
		MGC_McpMouseTxDataIrp.dwActualLength = 0;
		MGC_McpMouseTxDataIrp.pfIrpComplete = USBD_MouseTxDataComplete;
		MUSB_StartTransfer(&MGC_McpMouseTxDataIrp);
		ulLen -= ulLenTemp;
		pBufTemp += ulLenTemp;
	} while (ulLen);
}
#endif
void USBD_KeyboardStartTx(unsigned char *pBuf, unsigned long ulLen)
{
	unsigned char *pBufTemp = pBuf;
	unsigned long ulLenTemp;
	do
	{
		if (b_isTRxing == TRUE)
		{
			return;
		}
		ulLenTemp = MIN(ulLen, 64);
		b_isTRxing = TRUE;
		MGC_McpKeyboardTxDataIrp.pBuffer = pBufTemp;
		MGC_McpKeyboardTxDataIrp.dwLength = ulLenTemp;
		MGC_McpKeyboardTxDataIrp.dwActualLength = 0;
		MGC_McpKeyboardTxDataIrp.pfIrpComplete = USBD_KeyboardTxDataComplete;
		MUSB_StartTransfer(&MGC_McpKeyboardTxDataIrp);
		ulLen -= ulLenTemp;
		pBufTemp += ulLenTemp;
	} while (ulLen);
}
#if 0
unsigned char tISOCTable[]=
						{
							0x3c,0x37,
							0x0e,0x27,
							0x01,0x00,
							0xf1,0xd8,
							0xc5,0xc8,
							0xf2,0xd8,
							0x01,0x00,
							0x0f,0x27
						};

extern unsigned char ucAudioRecordOnOff;

extern uint8_t  read_decode_data(uint8_t *buf);
uint8_t notify_voice_data[64];
#endif
extern	uint8_t check_b_isTRxing(void);				
#include "gpio.h"
void test_usb_device(void)
{
	extern struct rev_ntf_data notify_data;

    if (b_isConnected == FALSE)
    {
        UART_PRINTF("sk 12\r\n");
        return;
    }
		
    if (b_isTRxing == TRUE)
    {
        UART_PRINTF("sk 13\r\n");
        return;
    }
       
   
#if (USB_DRIVER)	
	if(notify_data.notify_standard_key_status==1)
    {
        notify_data.notify_standard_key_status=0;
        notify_data.notify_standard_key[0]=HIDS_KB_REPORT_ID;
        USBD_KeyboardStartTx(notify_data.notify_standard_key, 12);
    }
#endif
    
}
////////////// for user ///////////////////////

/******************************************************************
CDI callbacks
******************************************************************/
STATIC void MGC_McpNewUsbState(void* hClient, MUSB_BusHandle hBus,
			       MUSB_State State)
{
	//uart_printf("MGC_McpNewUsbState: state = %x\r\n",State);
    MGC_eMcpUsbState = State;

    /* TODO: anything? */

    if (State == MUSB_CONFIGURED)
    {
        b_isConnected = TRUE;
    }
    else
    {
        b_isConnected = FALSE;
    }
}

STATIC uint8_t MGC_McpDeviceRequest(void* hClient, MUSB_BusHandle hBus,
				    uint32_t dwSequence, const uint8_t* pSetup,
				    uint16_t wLength)
{
    MUSB_DeviceRequest* pRequest = (MUSB_DeviceRequest*)pSetup;
    uint8_t bOk = FALSE;

    os_printf("MGC_McpDeviceRequest\r\n");
    if (MUSB_TYPE_STANDARD == (pRequest->bmRequestType & MUSB_TYPE_MASK))
    {
        switch(pRequest->bRequest)
        {
        case MUSB_REQ_GET_INTERFACE:
            MUSB_DeviceResponse(hBus, dwSequence, &MGC_bMcpInterface, 1, FALSE);
            bOk = TRUE;
            break;
        case MUSB_REQ_SET_INTERFACE:
            MUSB_DeviceResponse(hBus, dwSequence, NULL, 0, FALSE);
            bOk = TRUE;
            break;
        }
    }
    else if ((pRequest->bmRequestType & MUSB_TYPE_CLASS)
         && (pRequest->bmRequestType & MUSB_RECIP_INTERFACE))
    {
        switch(pRequest->bRequest)
        {
        	default:
            break;
        }
    }
    return bOk;
}

void USBD_StartRx()
{	
	while(b_isTRxing == TRUE)
	{
	    UART_PRINTF("tran rx\r\n");
	}
    MGC_McpUsbRxDataIrp.dwActualLength = 0;
	MUSB_StartTransfer(&MGC_McpUsbRxDataIrp);
}


volatile uint8_t b_isDataing = FALSE;
uint8_t user_flash_data1[16] = {0x12,0x34,0xaa,0x00,0x00,0x20,0x00,0x00,
                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
STATIC uint32_t USBD_RxDataCallback(void* pCompleteParam, MUSB_Irp* pIrp)
{
        UART_PRINTF("USBD_RxData Complete\r\n");
        UART_PRINTF("dwLength: %d  dwActualLength: %d\r\n",MGC_McpUsbRxDataIrp.dwLength,MGC_McpUsbRxDataIrp.dwActualLength);
         if((MGC_McpUsbRxDataIrp.pBuffer[0] == 0x01) && 
              (MGC_McpUsbRxDataIrp.pBuffer[4] == 0x0e) 
              &&(MGC_McpUsbRxDataIrp.pBuffer[5] == 0xa5) 
              &&(MGC_McpUsbRxDataIrp.pBuffer[1] == 0xe0)
              &&(MGC_McpUsbRxDataIrp.pBuffer[2] == 0xfc)
              &&(MGC_McpUsbRxDataIrp.pBuffer[3] == 0x02) 
              )
         {
                flash_write(0,0x7ff00,16,user_flash_data1,NULL);         
                platform_reset(0x1515);
         }
        return 0;
}

STATIC uint8_t MGC_McpDeviceConfigSelected(void* hClient, MUSB_BusHandle hBus,
					   uint8_t bConfigurationValue,
					   MUSB_Pipe* ahPipe)
{
        MGC_McpKeyboardTxDataIrp.hPipe = ahPipe[0];
		MGC_McpKeyboardTxDataIrp.pBuffer = (uint8_t*)MGC_aJunk;
		MGC_McpKeyboardTxDataIrp.dwLength = sizeof(MGC_aJunk);
		MGC_McpKeyboardTxDataIrp.pfIrpComplete = USBD_KeyboardTxDataComplete;
		return TRUE;
}

void set_b_isTRxing(unsigned char val)
{
    b_isTRxing = val;
}

unsigned char check_b_isTRxing(void)
{
    return b_isTRxing;
}
