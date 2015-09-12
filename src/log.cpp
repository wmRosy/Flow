#include <stddef.h>
#include <string.h>
#include "log.h"
#include <stdio.h>

#include "ff.h"

extern "C"
{
	#include "diskio.h"
};

FRESULT res;
FATFS fs;
FIL *file = NULL;
int  log_ready;

int format_sdcard()
{
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_mkfs("", 0, 0);
	return 0;
}

int log_init()
{
	//format_sdcard();
	//printf("sdcard init...");
	FIL f;
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_open(&f, "flow.bin", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	log_ready = res == FR_OK;
	f_close(&f);
	printf("%s\r\n", log_ready ? "OK" : "FAIL");
	return 0;
}

int log_write(const void *data, int size)
{
	if (file == NULL && log_ready)
	{
		static FIL f;
		file = &f;
		char filename[20];
		int done  = 0;
		while(log_ready)
		{
			sprintf(filename, "im%04d.raw", done ++);
			FRESULT res = f_open(file, filename, FA_CREATE_NEW | FA_WRITE | FA_READ);
			if (res == FR_OK)
			{
				f_close(file);
				res = f_open(file, filename, FA_OPEN_EXISTING | FA_WRITE | FA_READ);
				//printf("opened %s for logging\n", filename);
				break;
			}
		}
	}

	if (log_ready && file)
	{
		unsigned int done;
		if (f_write(file, data, size, &done) != FR_OK || done !=size)
		{
			//printf("\r\nSDCARD ERROR\r\n");
			log_ready = false;
		}
		f_sync(file);
	}

	return 0;
}

int log_flush()
{
	if (log_ready && file)
		f_sync(file);

	return 0;
}
