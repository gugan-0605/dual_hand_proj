/******************************************************************************
* Copyright (c) 2013 - 2022 Xilinx, Inc.  All rights reserved.
* Copyright (c) 2022 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/*****************************************************************************/
/**
*
* @file xilffs_polled_example.c
*
*
* @note This example uses file system with SD to write to and read from
* an SD card using ADMA2 in polled mode.
* To test this example File System should not be in Read Only mode.
* To test this example USE_MKFS option should be true.
*
* This example was tested using SD2.0 card and eMMC (using eMMC to SD adaptor).
*
* To test with different logical drives, drive number should be mentioned in
* both FileName and Path variables. By default, it will take drive 0 if drive
* number is not mentioned in the FileName variable.
* For example, to test logical drive 1
* FileName =  "1:/<file_name>" and Path = "1:/"
* Similarly to test logical drive N, FileName = "N:/<file_name>" and
* Path = "N:/"
*
* None.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who Date     Changes
* ----- --- -------- -----------------------------------------------
* 1.00a hk  10/17/13 First release
* 2.2   hk  07/28/14 Make changes to enable use of data cache.
* 2.5   sk  07/15/15 Used File size as 8KB to test on emulation platform.
* 2.9   sk  06/09/16 Added support for mkfs.
* 3.10  mn  08/18/18 Change file size to 8MB from 8KB for ZynqMP platform
*
*</pre>
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xparameters.h"	/* SDK generated parameters */
#include "xsdps.h"		/* SD device driver */
#include "xil_printf.h"
#include "ff.h"
#include "xil_cache.h"
#include "xplatform_info.h"

/************************** Constant Definitions *****************************/


/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
int FfsSdPolledExample(uint8_t *data);

/************************** Variable Definitions *****************************/
static FIL fil;		/* File object */
static FATFS fatfs;
/*
 * To test logical drive 0, FileName should be "0:/<File name>" or
 * "<file_name>". For logical drive 1, FileName should be "1:/<file_name>"
 */
static char FileName[32] = "0:/Pointcloud.txt";
static char *SD_File;

#ifdef __ICCARM__
#pragma data_alignment = 32
u8 DestinationAddress[10*1024];
#pragma data_alignment = 32
u8 SourceAddress[10*1024];
#else
u8 DestinationAddress[10*1024] __attribute__ ((aligned(32)));
u8 SourceAddress[10*1024] __attribute__ ((aligned(32)));
#endif

#define TEST 7
MKFS_PARM mkfs_parm;
/*****************************************************************************/
/**
*
* Main function to call the SD example.
*
* @param	None
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*

/*****************************************************************************/
/**
*
* File system example using SD driver to write to and read from an SD card
* in polled mode. This example creates a new file on an
* SD card (which is previously formatted with FATFS), write data to the file
* and reads the same data back to verify.
*
* @param	None
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int FfsSdPolledExample(uint8_t *data)


{
	XSdPs_Config *Config;
	XSdPs SdCard;
	FRESULT res;
	FATFS fs;
	FIL fil;

	/* 1. Path has to define */
	TCHAR *Path = "0:/";
	// Initialize SD card interface
	Config = XSdPs_LookupConfig(XPAR_XSDPS_0_DEVICE_ID);
	XSdPs_CfgInitialize(&SdCard, Config, Config->BaseAddress);
	xil_printf("Configuration has Happened\r\n");

	// Mount the SD card file system
	res = f_mount(&fs, Path, 0);
	if (res != FR_OK) {
		xil_printf("SD Card mount failed!\r\n");
		return -1;
	}
	xil_printf("Mounted has Happened\r\n");
//	Write Data to the SD Card
	UINT bytesWritten;

	// Open a file for writing
	res = f_open(&fil, "point.txt", FA_WRITE|FA_OPEN_APPEND);
	if (res != FR_OK) {
		xil_printf("Failed to open file for writing.\r\n");
		return -1;
	}
	xil_printf("File Open has happened\r\n");

	// Write data to the file
	res = f_write(&fil,(const void*)data, 1362, &bytesWritten);
	if (res != FR_OK) {
		xil_printf("Failed to write to file.\r\n");
		return -1;
	}
	// Make sure the entire data is written
	if (bytesWritten != 1362)
	{
		xil_printf("Not all data was written to the file. Expected %d, but wrote %d bytes.\r\n", 1362, bytesWritten);
		return -1;
	}

	// Ensure data is written to the SD card (flush internal buffer)
	f_sync(&fil);

    // Unmount the file system
    f_mount(NULL, Path, 0);
    xil_printf("Unmounted the SD card\r\n");

	xil_printf("Data written to SD card successfully!\r\n");

	// Close the file
	f_close(&fil);
	xil_printf("File CLosing for SD Card\r\n");
	return XST_SUCCESS;
}

//
//{
//
//	XSdPs_Config *Config;
//	XSdPs SdCard;
//	FRESULT Res;			  /*Stores the result status of FAFTS things*/
//	UINT NumBytesRead;		  /*Stores the actual number of bytes read from the SD card*/
//	UINT NumBytesWritten;	  /*Stores the actual number of bytes written to the SD card*/
//	u32 BuffCnt;			  /*Used as a loop counter for data processing*/
//	BYTE work[FF_MAX_SS];	  /*Temporary working buffer for f_mkfs() (formatting the SD card)*/
//	u32 FileSize = (1362*16); /*Defines the number of bytes to read/write*/
//
//	/* 1. Path has to define */
//	TCHAR *Path = "0:/";
//
//	/*Initialize SD card interface*/
//	Config = XSdPs_LookupConfig(XPAR_XSDPS_0_DEVICE_ID);
//	XSdPs_CfgInitialize(&SdCard, Config, Config->BaseAddress);
//	xil_printf("Configuration has Happened\r\n");
//
//	/* 2. Mounting the Device */
//	Res = f_mount(&fatfs, Path, 0);
//	if(Res != FR_OK){ return XST_FAILURE;}
//	xil_printf("Mounted\r\n");
//
//	/* 3. Setting the FAT Format */
//	 mkfs_parm.fmt = FM_FAT32;
//	 xil_printf("FAT Format setted\r\n");
//
//	/* 4. Passing Args in mkfs */
//	 Res = f_mkfs(Path, &mkfs_parm , work, sizeof(work));
//	 if(Res != FR_OK){ return XST_FAILURE;}
//	 xil_printf("Arguments Passed\r\n");
//	 /* 5. Assigning the File name */
//	 SD_File = (char*)FileName;
//	 xil_printf("Filename Assigned\r\n");
//
//	 /* 6. Opening the File */
//	 Res = f_open(&fil,SD_File,FA_WRITE|FA_OPEN_APPEND);
//	 if(Res != FR_OK){ return XST_FAILURE;}
//	 xil_printf("File Opened\r\n");
//	 /* 7. Setting the Offset */
//	 Res = f_lseek(&fil,0);
//	 if (Res != FR_OK){return XST_FAILURE;}
//	 xil_printf("Fseek setted for Offset\r\n");
//
//	 /*This Source address is a Temperory space to store the Data as of now.*/
//	 /*f_write will take care for writing the Data into the SD Card*/
//	 Res = f_write(&fil, (const void*)data, FileSize, &NumBytesWritten);
//	 if (Res!= FR_OK || NumBytesWritten != FileSize){return XST_FAILURE;}
//	 xil_printf("Data written in File\r\n");
//
//	 return XST_SUCCESS;
//}












































//
//
//
//
//
//
//
//
//
//
//
//
//
//	/*
//	 * To test logical drive 0, Path should be "0:/"
//	 * For logical drive 1, Path should be "1:/"
//	 */
//	TCHAR *Path = "0:/";
//
////	for(int i=0;i<7;i++)
////		SourceAddress[i] = i;
//	for(BuffCnt = 0; BuffCnt < FileSize; BuffCnt++){
//		SourceAddress[BuffCnt] = BuffCnt;
//	}
//	xil_printf("Src Addrs: ");
//	for(int i=0;i<FileSize;i++)
//		xil_printf("%d ",SourceAddress[i]);
//	xil_printf("\r\n");
//	/*
//	 * Register volume work area, initialize device
//	 */
//	Res = f_mount(&fatfs, Path, 0);
//
//	if (Res != FR_OK) {
//		return XST_FAILURE;
//	}
//
//	 mkfs_parm.fmt = FM_FAT32;
//	/*
//	 * Path - Path to logical driver, 0 - FDISK format.
//	 * 0 - Cluster size is automatically determined based on Vol size.
//	 */
//	Res = f_mkfs(Path, &mkfs_parm , work, sizeof work);
//	if (Res != FR_OK) {
//		return XST_FAILURE;
//	}
//
//	/*
//	 * Open file with required permissions.
//	 * Here - Creating new file with read/write permissions. .
//	 * To open file with write permissions, file system should not
//	 * be in Read Only mode.
//	 */
//	SD_File = (char *)FileName;
//
//	Res = f_open(&fil, SD_File, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
//	if (Res) {
//		return XST_FAILURE;
//	}
//
//	/*
//	 * Pointer to beginning of file .
//	 */
//	Res = f_lseek(&fil, 0);
//	if (Res) {
//		return XST_FAILURE;
//	}
//
//	/*
//	 * Write data to file.
//	 */
////	Res = f_write(&fil, (const void*)SourceAddress, 7,
////			&NumBytesWritten);
//
//	/*This Source address is a Temperory space to store the Data as of now.*/
//	/*f_write will take care for writing the Data into the SD Card*/
//
//	Res = f_write(&fil, (const void*)SourceAddress, FileSize,
//			&NumBytesWritten);
//	if (Res) {
//		return XST_FAILURE;
//	}
//	xil_printf("Writing Happening\r\n");
//
//	/*
//	 * Pointer to beginning of file .
//	 */
//	Res = f_lseek(&fil, 0);
//	if (Res) {
//		return XST_FAILURE;
//	}
//	xil_printf("Fseek setted successfully\r\n");
//
//	/*
//	 * Read data from file.
//	 */
//	Res = f_read(&fil, (void*)DestinationAddress, FileSize,
//			&NumBytesRead);
////	Res = f_read(&fil, (void*)DestinationAddress, FileSize,
////			&NumBytesRead);
//	if (Res) {
//		return XST_FAILURE;
//	}
//	xil_printf("Reading done successfully\r\n");
//	xil_printf("Dest Addrs: ");
//	for(int i=0;i<FileSize;i++)
//			xil_printf("%d ",DestinationAddress[i]);
//	xil_printf("\r\n");
////	xil_printf("Dest Addrs: %s\r\n",DestinationAddress);
//	/*
//	 * Data verification
//	 */
//	for(BuffCnt = 0; BuffCnt < FileSize; BuffCnt++){
//		if(SourceAddress[BuffCnt] != DestinationAddress[BuffCnt]){
//			return XST_FAILURE;
//		}
//	}
//
//	/*
//	 * Close file.
//	 */
//	Res = f_close(&fil);
//	if (Res) {
//		return XST_FAILURE;
//	}
//
//	return XST_SUCCESS;
//}
