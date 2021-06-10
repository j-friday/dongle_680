/******************************************************************
*                                                                *
*        Copyright Mentor Graphics Corporation 2006              *
*                                                                *
*                All Rights Reserved.                            *
*                                                                *
*    THIS WORK CONTAINS TRADE SECRET AND PROPRIETARY INFORMATION *
*  WHICH IS THE PROPERTY OF MENTOR GRAPHICS CORPORATION OR ITS   *
*  LICENSORS AND IS SUBJECT TO LICENSE TERMS.                    *
*                                                                *
******************************************************************/

/*
 * Command-line (really menu-driven) simple disk read/write utility,
 * to test mass-storage class driver and everything below it
 * $Revision: 1.34 $
 */

#include "mu_impl.h"
#include "mu_cdi.h"
#include "mu_mem.h"
#include "mu_stdio.h"
#include "mu_strng.h"
#include "mu_hfi.h"
//#include "mu_ghi.h"
//#include "mu_kii.h"
#include "mu_spi.h"

#include "class/mu_msd.h"

#include "mu_mapi.h"
//#include "mu_hidif.h"
#include "uart.h"
extern MUSB_FunctionClient MGC_McpFunctionClient;

#define MGC_TEST_MSD_BUFFER_SIZE    1024

/**************************** FORWARDS ****************************/

//static void MGC_TestMsdMediumCheckComplete(MUSB_HfiVolumeHandle hVolume);

/**************************** GLOBALS *****************************/

//static uint8_t MGC_bTimingMode = FALSE;


/* UCDI variables */
static MUSB_Port* MGC_pCdiPort = NULL;
static MUSB_BusHandle MGC_hCdiBus = NULL;
static uint8_t MGC_bDesireHostRole = TRUE;
static uint8_t MGC_aMsdPeripheralList[256];

#ifdef MUSB_HUB
static MUSB_DeviceDriver MGC_aDeviceDriver[3];
#else
static MUSB_DeviceDriver MGC_aDeviceDriver[2];
#endif

#if defined(MUSB_OTG)
static void MGC_MsdNewOtgState(void* hClient, MUSB_BusHandle hBus, 
			       MUSB_OtgState State);
static void MGC_MsdOtgError(void* hClient, MUSB_BusHandle hBus, 
			    uint32_t dwStatus);
/* current OTG state */
static MUSB_OtgState MGC_eTestMsdOtgState = MUSB_AB_IDLE;

static MUSB_OtgClient MGC_MsdOtgClient = 
{
    NULL,	/* no instance data; we are singleton */
    &MGC_bDesireHostRole,
    MGC_MsdNewOtgState,
    MGC_MsdOtgError
};
#endif

#if defined(MUSB_HOST) || defined(MUSB_OTG)
static MUSB_HostClient MGC_MsdHostClient = 
{
    MGC_aMsdPeripheralList,		/* peripheral list */
    0,			    /* filled in main */
    /*sizeof(MGC_aMsdPeripheralList),*/	/* peripheral list length */
    MGC_aDeviceDriver,
    0					/* filled in main */
};
#endif

/*************************** FUNCTIONS ****************************/

#if defined(MUSB_OTG)
/* OTG client */
static void MGC_MsdNewOtgState(void* hClient, MUSB_BusHandle hBus, 
			       MUSB_OtgState State)
{
    char aAnswer[4];
    MGC_eTestMsdOtgState = State;
    (void)MGC_eTestMsdOtgState;

    switch(State)
    {
	case MUSB_AB_IDLE:
		#if	0
		MUSB_PrintLine("S - Start Session");
		MUSB_GetLine(aAnswer, 4);
		#endif
		if(('s' == aAnswer[0]) || ('S' == aAnswer[0]))
		{
			MUSB_RequestBus(MGC_hCdiBus);
		}
		break;
	case MUSB_A_SUSPEND:
		#if	0
		MUSB_PrintLine("R - Resume bus");
		MUSB_GetLine(aAnswer, 4);
		#endif
		if(('r' == aAnswer[0]) || ('R' == aAnswer[0]))
		{
			MUSB_ResumeBus(MGC_hCdiBus);
		}
		break;
	default:
		break;
    }
}

static void MGC_MsdOtgError(void* hClient, MUSB_BusHandle hBus, 
			    uint32_t dwStatus)
{
	#if	0
    switch(dwStatus)
    {
    case MUSB_STATUS_UNSUPPORTED_DEVICE:
	MUSB_PrintLine("Device not supported");
	break;
    case MUSB_STATUS_UNSUPPORTED_HUB:
	MUSB_PrintLine("Hubs not supported");
	break;
    case MUSB_STATUS_OTG_VBUS_INVALID:
	MUSB_PrintLine("Vbus error");
	break;
    case MUSB_STATUS_OTG_NO_RESPONSE:
	MUSB_PrintLine("Device not responding");
	break;
    case MUSB_STATUS_OTG_SRP_FAILED:
	MUSB_PrintLine("Device not responding (SRP failed)");
	break;
    }
	#endif
}
#endif

/* medium check completion callback */
/*
static void MGC_TestMsdMediumCheckComplete(MUSB_HfiVolumeHandle hVolume)
{
}
*/

static MUSB_HfiDevice* MGC_pHfiDevice = NULL;
static const MUSB_HfiMediumInfo* MGC_pHfiMediumInfo = NULL;
static uint8_t MediaIsOk = FALSE;
/*
static void MGC_CheckMedium()
{
    if(MGC_pHfiDevice && !MGC_pHfiMediumInfo)
    {
	MGC_pHfiDevice->pfCheckMediumNotify(MGC_pHfiDevice->pPrivateData, 
		MGC_TestMsdMediumCheckComplete);
    }
}
*/
/* Implementation */
MUSB_HfiStatus 
MUSB_HfiAddDevice(MUSB_HfiVolumeHandle* phVolume,
		  const MUSB_HfiDeviceInfo* pInfo, 
		  MUSB_HfiDevice* pDevice)
{
    os_printf("MUSB_HfiAddDevice\r\n");
		MGC_pHfiDevice = pDevice;
		MediaIsOk = TRUE;
	return MUSB_HFI_SUCCESS;

}

/* Implementation */
void 
MUSB_HfiMediumInserted(MUSB_HfiVolumeHandle 	 hVolume,
		       const MUSB_HfiMediumInfo* pMediumInfo)
{
   	MGC_pHfiMediumInfo = pMediumInfo;
}

/* Implementation */
void MUSB_HfiMediumRemoved(MUSB_HfiVolumeHandle hVolume)
{
	MGC_pHfiMediumInfo = NULL;
}

/* Implementation */
void MUSB_HfiDeviceRemoved(void)
{
    os_printf("MUSB_HfiDeviceRemoved\r\n");
	MGC_pHfiDevice = NULL;
	MediaIsOk = FALSE;
}

uint8_t MGC_MsdGetMediumstatus(void)
{
	uint8_t Ret = 0;
	if(MGC_pHfiDevice)// && (MGC_pHfiMediumInfo))
	{
		Ret = MediaIsOk;
	}
	return Ret;
}

static uint32_t MGC_MsdRdTransferComplete(MUSB_HfiVolumeHandle hVolume,
					uint16_t wActualBlocks)
{
	return 2;//read ok
}
uint32_t MUSB_HfiRead( uint32_t first_block, uint32_t block_num, uint8_t *dest)
{
//	os_printf("====read ==\r\n");
	uint32_t RetValue = 1;
	
	if (MGC_pHfiDevice && MGC_pHfiDevice->pfReadDevice)
	{
//	MGC_MsdBotReadDevice()
		RetValue = MGC_pHfiDevice->pfReadDevice(MGC_pHfiDevice->pPrivateData,
	    first_block, 0, block_num, dest,MGC_MsdRdTransferComplete, FALSE);
	}
	return RetValue;
}

static uint32_t MGC_MsdWrTransferComplete(MUSB_HfiVolumeHandle hVolume,
					uint16_t wActualBlocks)
{
	return 3;//write ok
}
uint32_t MUSB_HfiWrite(uint32_t first_block, uint32_t block_num, uint8_t *dest)
{
	uint32_t RetValue = 1;

	if (MGC_pHfiDevice && MGC_pHfiDevice->pfWriteDevice)
	{
//	MGC_MsdBotWriteDevice()
		RetValue = MGC_pHfiDevice->pfWriteDevice(MGC_pHfiDevice->pPrivateData,
                        first_block, 0, block_num, dest, FALSE, MGC_MsdWrTransferComplete, FALSE);
	}
	return RetValue;
}

uint32_t get_HfiMedium_size(void)
{
	if(MGC_pHfiMediumInfo)
		return MGC_pHfiMediumInfo->dwBlockCountLo;
	else
		return 0;
}
uint32_t get_HfiMedium_blksize(void)
{
	if(MGC_pHfiMediumInfo)
		return MGC_pHfiMediumInfo->dwBlockSize;
	else
		return 0;
}
/* Entrypoint */
uint8_t usb_sw_en=0;
int usb_sw_init(void)
{
    
    if(usb_sw_en)
        return 0;
#if 0//defined(MUSB_HOST) || defined(MUSB_OTG)
    MUSB_DeviceDriver* pDriver;
    uint8_t* pList;
    uint16_t wCount, wSize, wRemain;
    uint8_t bDriver;
    /* fill driver table */
    bDriver = 0;
    wSize = wCount = 0;
    
    wRemain = (uint16_t)sizeof(MGC_aMsdPeripheralList);
    pList = MGC_aMsdPeripheralList;
    wSize = MUSB_FillMsdPeripheralList(bDriver, pList, wRemain);
    if(wSize < wRemain)
    {
		pDriver = MUSB_GetStorageClassDriver();//��ΪHOST,��ö�U�̵���������
		if(pDriver)
		{
            MUSB_MemCopy(&(MGC_MsdHostClient.aDeviceDriverList[bDriver]),
            pDriver, sizeof(MUSB_DeviceDriver));
            pList += wSize;
            wCount += wSize;
            wRemain -= wSize;
            bDriver++;
		}
    }

    MGC_MsdHostClient.wPeripheralListLength = wCount;
    MGC_MsdHostClient.bDeviceDriverListLength = bDriver;
#endif

    if(!MUSB_InitSystem(5))
    {
		os_printf("error ,could not initialize MicroSW\r\n");
		return -1;
    }

    /* find first CDI port */
    MGC_pCdiPort = MUSB_GetPort(0);
    /*	
    if(!MGC_pCdiPort)
    {
    	os_printf("MUSB_GetPort fail\r\n");
    	MUSB_DestroySystem();
    	return -1;
    }*/
	bk_printf("usb_sw_init: phase 2\r\n");
     /* start session */
    MGC_hCdiBus = MUSB_RegisterOtgClient(MGC_pCdiPort, &MGC_McpFunctionClient, &MGC_MsdHostClient, &MGC_MsdOtgClient);

    if(!MGC_hCdiBus)
    {
        os_printf("usb_sw_init: MUSB_RegisterOtgClient fail\r\n");
        //MUSB_DestroySystem();
        return -1;
    }
    else
    {
        os_printf("usb_sw_init: MGC_hCdiBus = 0x%lx\r\n", (unsigned long)MGC_hCdiBus);
    }
    usb_sw_en = 1;
    return 0;
}

