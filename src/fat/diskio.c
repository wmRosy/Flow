/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* by grqd_xp                                                            */
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
#include <string.h>
#include <time.h>
#include "diskio.h"
#include "sdcard.h"
#include "../common/mcu.h"
#include "../common/gps.h"

/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */
/* Note that Tiny-FatFs supports only single drive and always            */
/* accesses drive number 0.                                              */

#define SECTOR_SIZE 512U
int read_count = 0;
int write_count = 0;

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                   */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	SD_CardInfo SDCardInfo;
	
	// SDIO Interrupt ENABLE
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	if (SD_Init() != SD_OK ||
		SD_GetCardInfo(&SDCardInfo) != SD_OK ||
		SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16)) != SD_OK ||
		SD_EnableWideBusOperation(SDIO_BusWide_4b) != SD_OK ||
		SD_SetDeviceMode(SD_DMA_MODE) != SD_OK)
	{
		return RES_ERROR;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{	
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
	if(count==1)
	{
		if (SD_ReadBlock(((int64_t)sector) << 9 , (uint32_t*)buff,SECTOR_SIZE) != SD_OK)
			return RES_ERROR;
	}
	else
	{
		if (SD_ReadMultiBlocks(((int64_t)sector) << 9 ,(uint32_t*)buff,SECTOR_SIZE,count) != SD_OK)
			return RES_ERROR;;
	}

	read_count++;

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	if(count==1)
	{
		if (SD_WriteBlock(((int64_t)sector) << 9 ,(uint32_t*)buff,SECTOR_SIZE) != SD_OK)
			return RES_ERROR;
	}
	else
	{
		if (SD_WriteMultiBlocks(((int64_t)sector) << 9 ,(uint32_t*)buff,SECTOR_SIZE,count) != SD_OK)
			return RES_ERROR;
	}
	write_count++;
	return RES_OK;
}
#endif /* _READONLY */


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{	
	SD_CardInfo SDCardInfo;
	switch(ctrl)
	{
	case GET_SECTOR_SIZE:
	case GET_BLOCK_SIZE:
		if (buff)
			*(DWORD*)buff = 512;
		break;
	case GET_SECTOR_COUNT:
		if (buff)
		{
			uint32_t DeviceSizeMul = (SDCardInfo.SD_csd.DeviceSizeMul + 2);

			if(SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
			{
				*(DWORD*)buff = (SDCardInfo.SD_csd.DeviceSize + 1) * 1024;
			}
			else
			{
				uint32_t NumberOfBlocks  = ((1 << (SDCardInfo.SD_csd.RdBlockLen)) / 512);
				*(DWORD*)buff = ((SDCardInfo.SD_csd.DeviceSize + 1) * (1 << DeviceSizeMul) << (NumberOfBlocks/2));
			}
		}
		break;
	case CTRL_SYNC:
		break;
	default:
		return RES_PARERR;
	}
	
	return RES_OK;
}

DWORD make_fattime(int sec, int min, int hour, int day, int mon, int year)
{
	return ((year+1900-1980) << 25)
		| ((mon+1) << 21)
		| ((day) << 16)
		| ((hour) << 11)
		| ((min) << 5)
		| ((sec) << 1)
		;

}

#ifndef NOGPS
DWORD get_fattime(void)
{
	time_t current_time;
	nmeaINFO *info = GPS_GetInfo();
	nmeaTIME *time =  info->utc2.year == 0 ? &info->utc : &info->utc2;
	struct tm _tm;

	_tm.tm_sec = time->sec;
	_tm.tm_min = time->min;
	_tm.tm_hour = time->hour;
	_tm.tm_mday = time->day;
	_tm.tm_mon = time->mon;
	_tm.tm_year = time->year;

	current_time = mktime(&_tm);
	if (current_time == -1)
		return 0;

	current_time += 8 * 3600;	// CHINA = UTC+8
	_tm = *localtime(&current_time);

	return make_fattime(_tm.tm_sec, _tm.tm_min, _tm.tm_hour, _tm.tm_mday, _tm.tm_mon, _tm.tm_year);
}
#else
DWORD get_fattime(void)
{
	return 0;
}
#endif






















