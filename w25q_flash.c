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

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
 *                                        INCLUDE FILES
 ==================================================================================================*/
#include "w25q_flash.h"
#include "Qspi_Ip_Cfg.h"

/* IMU data read new code adding here remove anytime*/
#define IMU_FLASH_SECTOR   (g_devInfo.totalSectors - 1U) /* last sector */
#define IMU_FLASH_ADDR     (IMU_FLASH_SECTOR * W25Q_SECTOR_SIZE)

/*==================================================================================================
 *                                      GLOBAL VARIABLES
 ==================================================================================================*/
volatile W25Q_DeviceInfo_t g_devInfo;

/*==================================================================================================
 *                                      LOCAL VARIABLES
 ==================================================================================================*/
static uint8 s_txBuf[W25Q_PAGE_SIZE] __attribute__((aligned(4)));
static uint8 s_rxBuf[W25Q_PAGE_SIZE] __attribute__((aligned(4)));

/*==================================================================================================
 *                                   LOCAL FUNCTION PROTOTYPES
 ==================================================================================================*/
static Qspi_Ip_StatusType W25Q_WaitReady(void);

/*==================================================================================================
 *                                      PUBLIC FUNCTIONS
 ==================================================================================================*/

/**
 * @brief  Return pointer to the internal TX (pattern) buffer
 * @return Pointer to s_txBuf
 */
uint8 *W25Q_GetTxBuffer(void) {
    return s_txBuf;
}

/**
 * @brief  Return pointer to the internal RX (readback) buffer
 * @return Pointer to s_rxBuf
 */
uint8 *W25Q_GetRxBuffer(void) {
    return s_rxBuf;
}


/**
 * @brief  Initialise the QSPI controller via RTD API
 * @return QSPI status
 */
Qspi_Ip_StatusType W25Q_ControllerInit(void) {
    Qspi_Ip_StatusType st;
    st = Qspi_Ip_ControllerInit(W25Q_QSPI_MEM_INST, Qspi_Ip_paQspiUnitCfg);
    g_devInfo.ctrlInitSt = st;
    return st;
}

/**
 * @brief  Initialise the external flash memory via RTD API
 * @return QSPI status
 */
Qspi_Ip_StatusType W25Q_MemoryInit(void) {
    Qspi_Ip_StatusType st;
    st = Qspi_Ip_Init(W25Q_QSPI_MEM_INST, Qspi_Ip_paFlashCfg,
                       Qspi_Ip_paFlashConnectionCfg);
    g_devInfo.memInitSt = st;
    return st;
}

/**
 * @brief  Read the JEDEC ID from the external flash
 * @param  idBuf  Buffer to receive 3 ID bytes (manufacturer, type, capacity)
 * @return QSPI status
 */
Qspi_Ip_StatusType W25Q_ReadId(uint8 *idBuf) {
    return Qspi_Ip_ReadId(W25Q_QSPI_MEM_INST, idBuf);
}

/**
 * @brief  Collect JEDEC ID and status registers into g_devInfo (Phase 1)
 *
 *   Qspi_Ip_ReadId()         -> JEDEC ID via MEX LUT 10 (cmd 0x9F, 3 bytes)
 *   Qspi_Ip_RunReadCommand() -> status registers via MEX LUT 3 / 8 / 9
 */
void W25Q_CollectDeviceInfo(void) {
    uint8 idBuf[3] = { 0U, 0U, 0U };
    uint8 srBuf[1] = { 0U };
    Qspi_Ip_StatusType st;

    /* ── JEDEC ID (0x9F) ── */
    st = Qspi_Ip_ReadId(W25Q_QSPI_MEM_INST, idBuf);
    g_devInfo.jedecReadSt = st;
    if (STATUS_QSPI_IP_SUCCESS == st) {
        g_devInfo.manufacturerId = idBuf[0];
        g_devInfo.memoryType     = idBuf[1];
        g_devInfo.capacityId     = idBuf[2];
    } else {
        g_devInfo.manufacturerId = 0x00U;
        g_devInfo.memoryType     = 0x00U;
        g_devInfo.capacityId     = 0x00U;
    }

    g_devInfo.jedecValid = (boolean)((g_devInfo.manufacturerId == 0xEFU)
                                  && (g_devInfo.memoryType     == 0x40U)
                                  && (g_devInfo.capacityId     == 0x17U));

    if ((g_devInfo.capacityId >= 0x11U) && (g_devInfo.capacityId <= 0x19U)) {
        g_devInfo.flashSizeBytes = (1UL << g_devInfo.capacityId);
    } else {
        g_devInfo.flashSizeBytes = 0x800000U;
    }
    g_devInfo.totalSectors = g_devInfo.flashSizeBytes / W25Q_SECTOR_SIZE;
    g_devInfo.totalPages   = g_devInfo.flashSizeBytes / W25Q_PAGE_SIZE;

    /* ── Status Register 1 (0x05) ── */
    srBuf[0] = 0U;
    st = Qspi_Ip_RunReadCommand(W25Q_QSPI_MEM_INST, W25Q_LUT_IDX_READ_SR1,
                                 0U, srBuf, NULL_PTR, 1U);
    g_devInfo.sr1ReadSt = st;
    g_devInfo.sr1 = (STATUS_QSPI_IP_SUCCESS == st) ? srBuf[0] : 0xFFU;

    /* ── Status Register 2 (0x35) ── */
    srBuf[0] = 0U;
    st = Qspi_Ip_RunReadCommand(W25Q_QSPI_MEM_INST, W25Q_LUT_IDX_READ_SR2,
                                 0U, srBuf, NULL_PTR, 1U);
    g_devInfo.sr2ReadSt = st;
    g_devInfo.sr2 = (STATUS_QSPI_IP_SUCCESS == st) ? srBuf[0] : 0xFFU;

    /* ── Status Register 3 (0x15) ── */
    srBuf[0] = 0U;
    st = Qspi_Ip_RunReadCommand(W25Q_QSPI_MEM_INST, W25Q_LUT_IDX_READ_SR3,
                                 0U, srBuf, NULL_PTR, 1U);
    g_devInfo.sr3ReadSt = st;
    g_devInfo.sr3 = (STATUS_QSPI_IP_SUCCESS == st) ? srBuf[0] : 0xFFU;

    /* ── Decode key bits ── */
    g_devInfo.bpBitsCleared = (boolean)((g_devInfo.sr1 & 0x3CU) == 0x00U);
    g_devInfo.qeEnabled     = (boolean)((g_devInfo.sr2 & 0x02U) != 0x00U);

    g_devInfo.passCount     = 0U;
    g_devInfo.failCount     = 0U;
    g_devInfo.totalCycles   = 0U;
    g_devInfo.lastErrorCode = 0U;
    g_devInfo.lastErrorAddr = 0U;
}

/**
 * @brief  Erase one 4 KB sector and wait until the device is ready
 * @param  addr  Byte address of the sector to erase
 * @return QSPI status
 */
Qspi_Ip_StatusType W25Q_EraseSector(uint32 addr) {
    Qspi_Ip_StatusType st;
    st = Qspi_Ip_EraseBlock(W25Q_QSPI_MEM_INST, addr, W25Q_SECTOR_SIZE);
    if (STATUS_QSPI_IP_SUCCESS != st) {
        return st;
    }
    return W25Q_WaitReady();
}

/**
 * @brief  Program up to one page, splitting into QSPI_IP_MAX_WRITE_SIZE chunks
 * @param  addr  Byte address to start writing
 * @param  data  Source data buffer
 * @param  len   Number of bytes to write (<= W25Q_PAGE_SIZE)
 * @return QSPI status
 */
Qspi_Ip_StatusType W25Q_WritePage(uint32 addr, const uint8 *data, uint32 len) {
    Qspi_Ip_StatusType st;
    uint32 offset = 0U;
    uint32 chunk;

    while (offset < len) {
        chunk = len - offset;
        if (chunk > QSPI_IP_MAX_WRITE_SIZE) {
            chunk = QSPI_IP_MAX_WRITE_SIZE;
        }

        st = Qspi_Ip_Program(W25Q_QSPI_MEM_INST, addr + offset,
                              &data[offset], chunk);
        if (STATUS_QSPI_IP_SUCCESS != st) {
            return st;
        }

        st = W25Q_WaitReady();
        if (STATUS_QSPI_IP_SUCCESS != st) {
            return st;
        }

        offset += chunk;
    }

    return STATUS_QSPI_IP_SUCCESS;
}

/**
 * @brief  Read data from the external flash
 * @param  addr  Byte address to start reading
 * @param  buf   Destination buffer
 * @param  len   Number of bytes to read
 * @return QSPI status
 */
Qspi_Ip_StatusType W25Q_Read(uint32 addr, uint8 *buf, uint32 len) {
    return Qspi_Ip_Read(W25Q_QSPI_MEM_INST, addr, buf, len);
}

/**
 * @brief  Read len bytes from flash and verify all are 0xFF (erased)
 * @param  addr  Byte address to check
 * @param  len   Number of bytes to verify
 * @return TRUE if all bytes are 0xFF, FALSE otherwise
 */
boolean W25Q_VerifyErased(uint32 addr, uint32 len) {
    Qspi_Ip_StatusType st;
    uint32 i;

    st = W25Q_Read(addr, s_rxBuf, len);
    if (STATUS_QSPI_IP_SUCCESS != st) {
        return FALSE;
    }
    for (i = 0U; i < len; i++) {
        if (s_rxBuf[i] != 0xFFU) {
            return FALSE;
        }
    }
    return TRUE;
}

/**
 * @brief  Compare s_rxBuf against s_txBuf for len bytes
 * @param  len  Number of bytes to compare
 * @return TRUE if all bytes match, FALSE otherwise
 */
boolean W25Q_VerifyWritten(uint32 len) {
    uint32 i;
    for (i = 0U; i < len; i++) {
        if (s_rxBuf[i] != s_txBuf[i]) {
            return FALSE;
        }
    }
    return TRUE;
}

/**
 * @brief  Fill the TX buffer with a deterministic pattern
 * @param  seed  Seed value added to byte index
 * @param  len   Number of bytes to fill
 */
void W25Q_FillPattern(uint32 seed, uint32 len) {
    uint32 i;
    for (i = 0U; i < len; i++) {
        s_txBuf[i] = (uint8)((i + seed) & 0xFFU);
    }
}

/**
 * @brief  Fill the RX buffer with a constant value (pre-read poisoning)
 * @param  len        Number of bytes to fill
 * @param  fillValue  Value to write into every byte
 */
void W25Q_ClearRxBuffer(uint32 len, uint8 fillValue) {
    uint32 i;
    for (i = 0U; i < len; i++) {
        s_rxBuf[i] = fillValue;
    }
}

/*==================================================================================================
 *                                       LOCAL FUNCTIONS
 ==================================================================================================*/

/**
 * @brief  Poll the flash BUSY flag until the device is ready or timeout
 * @return STATUS_QSPI_IP_SUCCESS when ready, STATUS_QSPI_IP_TIMEOUT on timeout
 */
static Qspi_Ip_StatusType W25Q_WaitReady(void) {
    Qspi_Ip_StatusType st;
    uint32 timeout = W25Q_BUSY_POLL_TIMEOUT;

    do {
        st = Qspi_Ip_GetMemoryStatus(W25Q_QSPI_MEM_INST);
        timeout--;
    } while ((STATUS_QSPI_IP_BUSY == st) && (timeout > 0U));

    return (0U == timeout) ? STATUS_QSPI_IP_TIMEOUT : st;
}

#ifdef __cplusplus
}
#endif
