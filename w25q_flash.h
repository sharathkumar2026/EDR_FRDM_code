/*==================================================================================================
* Project : RTD AUTOSAR 4.9
* Platform : CORTEXM
* Peripheral : S32K3XX
* Dependencies : none
*
* Autosar Version : 4.9.0
* Autosar Revision : ASR_REL_4_9_REV_0000
* Autosar Conf.Variant :
* SW Version : 7.0.1
* Build Version : S32K3_RTD_7_0_1_D2602_ASR_REL_4_9_REV_0000_20260206
*
* Copyright 2020 - 2026 NXP
*
*   NXP Proprietary. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
==================================================================================================*/

#ifndef W25Q_FLASH_H
#define W25Q_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
 *                                        INCLUDE FILES
 ==================================================================================================*/
#include "Qspi_Ip.h"

/*==================================================================================================
 *                                      DEFINES AND MACROS
 ==================================================================================================*/
#define W25Q_SECTOR_SIZE        4096U
#define W25Q_PAGE_SIZE          256U
#define W25Q_QSPI_MEM_INST     0U
#define W25Q_TEST_SECTOR_COUNT  4U
#define W25Q_BUSY_POLL_TIMEOUT  10000000U

/* MEX virtual-LUT indices for status-register reads                          */
#define W25Q_LUT_IDX_READ_SR1  3U   /* cmd 0x05  -> SR1                      */
#define W25Q_LUT_IDX_READ_SR2  8U   /* cmd 0x35  -> SR2  (MEX: FastRead)     */
#define W25Q_LUT_IDX_READ_SR3  9U   /* cmd 0x15  -> SR3  (MEX: FastWrite)    */

/*==================================================================================================
 *                                      TYPE DEFINITIONS
 ==================================================================================================*/
typedef struct {
    uint8  manufacturerId;
    uint8  memoryType;
    uint8  capacityId;

    uint32 flashSizeBytes;
    uint32 totalSectors;
    uint32 totalPages;

    uint8  sr1;
    uint8  sr2;
    uint8  sr3;

    boolean jedecValid;
    boolean bpBitsCleared;
    boolean qeEnabled;

    Qspi_Ip_StatusType ctrlInitSt;
    Qspi_Ip_StatusType memInitSt;

    Qspi_Ip_StatusType jedecReadSt;
    Qspi_Ip_StatusType sr1ReadSt;
    Qspi_Ip_StatusType sr2ReadSt;
    Qspi_Ip_StatusType sr3ReadSt;

    uint32 passCount;
    uint32 failCount;
    uint32 totalCycles;
    uint32 lastErrorCode;
    uint32 lastErrorAddr;
} W25Q_DeviceInfo_t;

/*==================================================================================================
 *                                 GLOBAL VARIABLE DECLARATIONS
 ==================================================================================================*/
extern volatile W25Q_DeviceInfo_t g_devInfo;

/*==================================================================================================
 *                                      FUNCTION PROTOTYPES
 ==================================================================================================*/

/* Initialisation */
Qspi_Ip_StatusType W25Q_ControllerInit(void);
Qspi_Ip_StatusType W25Q_MemoryInit(void);
Qspi_Ip_StatusType W25Q_ReadId(uint8 *idBuf);

/* Device-info collection (Phase 1) */
void W25Q_CollectDeviceInfo(void);

/* Flash operations (Phase 2) */
Qspi_Ip_StatusType W25Q_EraseSector(uint32 addr);
Qspi_Ip_StatusType W25Q_WritePage(uint32 addr, const uint8 *data, uint32 len);
Qspi_Ip_StatusType W25Q_Read(uint32 addr, uint8 *buf, uint32 len);

/* Verify & pattern utilities */
boolean W25Q_VerifyErased(uint32 addr, uint32 len);
boolean W25Q_VerifyWritten(uint32 len);
void    W25Q_FillPattern(uint32 seed, uint32 len);
void    W25Q_ClearRxBuffer(uint32 len, uint8 fillValue);

/* Buffer access */
uint8 *W25Q_GetTxBuffer(void);
uint8 *W25Q_GetRxBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* W25Q_FLASH_H */
