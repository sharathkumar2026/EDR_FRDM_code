/*==================================================================================================
*   Project              : RTD AUTOSAR 4.9
*   Platform             : CORTEXM
*   Peripheral           : C40
*   Dependencies         : None
*   Authors              : SHARATH Avinash kemsys
*   Autosar Version      : 4.9.0
*   Autosar Revision     : ASR_REL_4_9_REV_0000
*   Autosar Conf.Variant :
*   SW Version           : 7.0.1
*   Build Version        : S32K3_RTD_7_0_1_D2602_ASR_REL_4_9_REV_0000_20260206
*
*   Copyright 2020 - 2026 NXP
*
*   NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be
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

#include "Mcal.h"
#include "Clock_Ip.h"
#include "Clock_Ip_Cfg.h"
#include "S32K344.h"
#include "S32K344_SIUL2.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Port_Ip_Cfg.h"
#include "Siul2_Dio_Ip.h"
#include "Lpuart_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Sa_PBcfg.h"
#include "FlexCAN_Ip.h"
#include "FlexCAN_Ip_Sa_PBcfg.h"
#include "IntCtrl_Ip.h"
#include "Stm_Ip.h"
#include "Stm_Ip_SA_PBcfg.h"
#include "OsIf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "asm330lhh_app.h"
#include "w25q_flash.h"

#define RTD_VERSION "7.0.1"
#define RX_MB_IDX (11U)

static asm330lhh_sample_t imuSample;

#define SW_MAJOR_VERSION "1.0"
#define IMU_LOG_FIRST_SECTOR       (2U)
#define IMU_LOG_RESERVED_LAST      (1U)
#define IMU_LOG_RECORD_SIZE        (32U)
#define IMU_LOG_RECORDS_PER_PAGE   (W25Q_PAGE_SIZE / IMU_LOG_RECORD_SIZE)
#define IMU_LOG_FIRST_DATA_PAGE    (1U)
#define IMU_LOG_DATA_PAGE_COUNT    ((W25Q_SECTOR_SIZE / W25Q_PAGE_SIZE) - IMU_LOG_FIRST_DATA_PAGE)
#define IMU_LOG_RECORD_VERSION     (1U)
#define IMU_LOG_CRC_OFFSET         (23U)
#define IMU_LOG_HEADER_CRC_OFFSET  (16U)
#define IMU_LOG_HEADER_VERSION     (1U)

#define CAN0_STB_PIN            4u
#define CAN0_STB_PORT           PTC_H_HALF
#define CAN0_EN_PIN             5u
#define CAN0_EN_PORT            PTC_H_HALF

Clock_Ip_StatusType clockStatus;
static Flexcan_Ip_MsgBuffType rxData;
static const Flexcan_Ip_DataInfoType rx_info =
{
    .msg_id_type = FLEXCAN_MSG_ID_EXT,
    .data_length = 8U,
    .is_polling = FALSE,
    .is_remote = FALSE
};
char uartBuff[512];
static uint8 imuLogPage[W25Q_PAGE_SIZE] __attribute__((aligned(4)));
static uint32 imuLogSectorCount;
static uint32 imuLogActiveSlot;
static uint32 imuLogGeneration;
static uint32 imuLogNextSequence;
static uint32 imuLogNextPage;
static uint32 imuLogRecordsBuffered;
static boolean imuLogReady = FALSE;

void TestDelay(uint32 delay)
{
    static volatile uint32 DelayTimer = 0;
    while (DelayTimer < delay)
    {
        DelayTimer++;
    }
    DelayTimer = 0;
}

static void SetupCanTransceiver(void)
{
    Siul2_Dio_Ip_WritePin(CAN0_STB_PORT, CAN0_STB_PIN, 1U);
    Siul2_Dio_Ip_WritePin(CAN0_EN_PORT, CAN0_EN_PIN, 1U);
}

static void Mem_ExFls_ExampleAssert(boolean Condition)
{
    while (!Condition)
    {
        Siul2_Dio_Ip_TogglePins(PTA_H_HALF, 13U);
        TestDelay(450000);
    }
}

static void Mem_ExFls_PinSetup(void)
{
    IP_SIUL2->MSCR[67] = SIUL2_MSCR_OBE(1U) | SIUL2_MSCR_IBE(0U) | SIUL2_MSCR_SSS(6U) | SIUL2_MSCR_DSE(1U) | SIUL2_MSCR_SRC(1U);
    IP_SIUL2->MSCR[106] = SIUL2_MSCR_OBE(1U) | SIUL2_MSCR_IBE(1U) | SIUL2_MSCR_SSS(7U) | SIUL2_MSCR_DSE(1U) | SIUL2_MSCR_SRC(1U);
    IP_SIUL2->IMCR[821 - 512] = SIUL2_IMCR_SSS(1U);
    IP_SIUL2->MSCR[107] = SIUL2_MSCR_OBE(1U) | SIUL2_MSCR_IBE(1U) | SIUL2_MSCR_SSS(7U) | SIUL2_MSCR_DSE(1U) | SIUL2_MSCR_SRC(1U);
    IP_SIUL2->IMCR[817 - 512] = SIUL2_IMCR_SSS(1U);
    IP_SIUL2->MSCR[103] = SIUL2_MSCR_OBE(1U) | SIUL2_MSCR_IBE(1U) | SIUL2_MSCR_SSS(7U) | SIUL2_MSCR_DSE(1U) | SIUL2_MSCR_SRC(1U);
    IP_SIUL2->IMCR[818 - 512] = SIUL2_IMCR_SSS(1U);
    IP_SIUL2->MSCR[108] = SIUL2_MSCR_OBE(1U) | SIUL2_MSCR_IBE(1U) | SIUL2_MSCR_SSS(7U) | SIUL2_MSCR_DSE(1U) | SIUL2_MSCR_SRC(1U);
    IP_SIUL2->IMCR[819 - 512] = SIUL2_IMCR_SSS(1U);
    IP_SIUL2->MSCR[66] = SIUL2_MSCR_OBE(1U) | SIUL2_MSCR_IBE(1U) | SIUL2_MSCR_SSS(7U) | SIUL2_MSCR_DSE(1U) | SIUL2_MSCR_SRC(1U);
    IP_SIUL2->IMCR[820 - 512] = SIUL2_IMCR_SSS(1U);
}

static void Uart_Init(void)
{
    Lpuart_Uart_Ip_Init(LPUART_UART_IP_INSTANCE_USING_6,
                        &Lpuart_Uart_Ip_xHwConfigPB_6);
}

static void Can_Init(void)
{
    SetupCanTransceiver();
    IntCtrl_Ip_EnableIrq(FlexCAN0_1_IRQn);
    IntCtrl_Ip_InstallHandler(FlexCAN0_1_IRQn,
                              CAN0_ORED_0_31_MB_IRQHandler,
                              NULL_PTR);
    FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);
    FlexCAN_Ip_SetRxMaskType(INST_FLEXCAN_0, FLEXCAN_RX_MASK_GLOBAL);
    FlexCAN_Ip_SetRxMbGlobalMask(INST_FLEXCAN_0, 0x00000000U);
    FlexCAN_Ip_ConfigRxMb(INST_FLEXCAN_0, RX_MB_IDX, &rx_info, 0x000U);
    FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_IDX, &rxData, FALSE);
    FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);
}

static void GPIO_Init(void)
{
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
                       g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);
}

static void Stm_Timer_Init(void)
{
    Stm_Ip_Init(0U, &STM_0_InitConfig_PB);
    Stm_Ip_StartTimer(0U, 0U);
}

void CAN0_Callback(uint8 instance, Flexcan_Ip_EventType eventType, uint32 buffIdx, const Flexcan_Ip_StateType *flexcanState)
{
    (void)instance;
    (void)flexcanState;

    if ((eventType == FLEXCAN_EVENT_RX_COMPLETE) && (buffIdx == RX_MB_IDX))
    {
        FlexCAN_Ip_Receive(INST_FLEXCAN_0, RX_MB_IDX, &rxData, FALSE);
    }
}
static void IMU_FlashUartPrint(const char *message)
{
    Lpuart_Uart_Ip_SyncSend(LPUART_UART_IP_INSTANCE_USING_6,
                            (const uint8 *)message,
                            (uint32)strlen(message),
                            100000u);
}

#if 0
static boolean IMU_StoreAndVerifyAcceleration(const asm330lhh_sample_t *sample)
{
    uint8 writeData[IMU_ACCEL_RECORD_SIZE];
    uint8 readData[IMU_ACCEL_RECORD_SIZE];
    uint16 axisValue;
    uint32 index;
    Qspi_Ip_StatusType flashStatus;

    if (sample == NULL_PTR)
    {
        IMU_FlashUartPrint("FLASH TEST FAIL: null IMU sample\r\n");
        return FALSE;
    }

    axisValue = (uint16)sample->accel_raw[0];
    writeData[0] = (uint8)(axisValue & 0xFFU);
    writeData[1] = (uint8)((axisValue >> 8U) & 0xFFU);
    axisValue = (uint16)sample->accel_raw[1];
    writeData[2] = (uint8)(axisValue & 0xFFU);
    writeData[3] = (uint8)((axisValue >> 8U) & 0xFFU);
    axisValue = (uint16)sample->accel_raw[2];
    writeData[4] = (uint8)(axisValue & 0xFFU);
    writeData[5] = (uint8)((axisValue >> 8U) & 0xFFU);

    flashStatus = W25Q_EraseSector(IMU_FLASH_TEST_SECTOR_ADDR);
    if (flashStatus != STATUS_QSPI_IP_SUCCESS)
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "FLASH ERASE FAIL status=%d\r\n", (int)flashStatus);
        IMU_FlashUartPrint(uartBuff);
        return FALSE;
    }
    IMU_FlashUartPrint("FLASH ERASE PASS\r\n");

    if (W25Q_VerifyErased(IMU_FLASH_TEST_SECTOR_ADDR,
                          IMU_ACCEL_RECORD_SIZE) != TRUE)
    {
        IMU_FlashUartPrint("FLASH ERASE VERIFY FAIL\r\n");
        return FALSE;
    }
    IMU_FlashUartPrint("FLASH ERASE VERIFY PASS\r\n");

    flashStatus = W25Q_WritePage(IMU_FLASH_TEST_SECTOR_ADDR,
                                 writeData,
                                 IMU_ACCEL_RECORD_SIZE);
    if (flashStatus != STATUS_QSPI_IP_SUCCESS)
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "FLASH WRITE FAIL status=%d\r\n", (int)flashStatus);
        IMU_FlashUartPrint(uartBuff);
        return FALSE;
    }
    IMU_FlashUartPrint("FLASH WRITE PASS\r\n");

    flashStatus = W25Q_Read(IMU_FLASH_TEST_SECTOR_ADDR,
                            readData,
                            IMU_ACCEL_RECORD_SIZE);
    if (flashStatus != STATUS_QSPI_IP_SUCCESS)
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "FLASH READ FAIL status=%d\r\n", (int)flashStatus);
        IMU_FlashUartPrint(uartBuff);
        return FALSE;
    }

    for (index = 0U; index < IMU_ACCEL_RECORD_SIZE; index++)
    {
        if (writeData[index] != readData[index])
        {
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "FLASH DATA VERIFY FAIL byte=%lu TX=%02X RX=%02X\r\n",
                           (unsigned long)index,
                           (unsigned int)writeData[index],
                           (unsigned int)readData[index]);
            IMU_FlashUartPrint(uartBuff);
            return FALSE;
        }
    }

    (void)snprintf(uartBuff, sizeof(uartBuff),
                   "FLASH PASS addr=0x%08lX AX=%d AY=%d AZ=%d\r\n",
                   (unsigned long)IMU_FLASH_TEST_SECTOR_ADDR,
                   (int)sample->accel_raw[0],
                   (int)sample->accel_raw[1],
                   (int)sample->accel_raw[2]);
    IMU_FlashUartPrint(uartBuff);

    (void)snprintf(uartBuff, sizeof(uartBuff),
                   "FLASH DATA TX=%02X%02X%02X%02X%02X%02X "
                   "RX=%02X%02X%02X%02X%02X%02X MATCH\r\n",
                   (unsigned int)writeData[0], (unsigned int)writeData[1],
                   (unsigned int)writeData[2], (unsigned int)writeData[3],
                   (unsigned int)writeData[4], (unsigned int)writeData[5],
                   (unsigned int)readData[0], (unsigned int)readData[1],
                   (unsigned int)readData[2], (unsigned int)readData[3],
                   (unsigned int)readData[4], (unsigned int)readData[5]);
    IMU_FlashUartPrint(uartBuff);

    return TRUE;
}
#endif

static uint16 IMU_LogCrc16(const uint8 *data, uint32 length)
{
    uint16 crc = 0xFFFFU;
    uint32 index;
    uint8 bitIndex;

    for (index = 0U; index < length; index++)
    {
        crc = (uint16)(crc ^ ((uint16)data[index] << 8U));
        for (bitIndex = 0U; bitIndex < 8U; bitIndex++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16)(crc << 1U);
            }
        }
    }
    return crc;
}

static void IMU_LogPutU16(uint8 *data, uint32 offset, uint16 value)
{
    data[offset] = (uint8)(value & 0xFFU);
    data[offset + 1U] = (uint8)((value >> 8U) & 0xFFU);
}

static void IMU_LogPutU32(uint8 *data, uint32 offset, uint32 value)
{
    data[offset] = (uint8)(value & 0xFFU);
    data[offset + 1U] = (uint8)((value >> 8U) & 0xFFU);
    data[offset + 2U] = (uint8)((value >> 16U) & 0xFFU);
    data[offset + 3U] = (uint8)((value >> 24U) & 0xFFU);
}

static uint32 IMU_LogGetU32(const uint8 *data, uint32 offset)
{
    return ((uint32)data[offset]) |
           ((uint32)data[offset + 1U] << 8U) |
           ((uint32)data[offset + 2U] << 16U) |
           ((uint32)data[offset + 3U] << 24U);
}

static uint32 IMU_LogSectorAddress(uint32 slot)
{
    return (IMU_LOG_FIRST_SECTOR + slot) * W25Q_SECTOR_SIZE;
}

static boolean IMU_LogPageIsErased(const uint8 *data)
{
    uint32 index;

    for (index = 0U; index < W25Q_PAGE_SIZE; index++)
    {
        if (data[index] != 0xFFU)
        {
            return FALSE;
        }
    }
    return TRUE;
}

static boolean IMU_LogWriteAndVerifyPage(uint32 address, const uint8 *data)
{
    uint8 readback[W25Q_PAGE_SIZE] __attribute__((aligned(4)));
    uint32 index;
    Qspi_Ip_StatusType status;

    status = W25Q_WritePage(address, data, W25Q_PAGE_SIZE);
    if (status != STATUS_QSPI_IP_SUCCESS)
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "IMU LOG WRITE FAIL status=%d\r\n", (int)status);
        IMU_FlashUartPrint(uartBuff);
        return FALSE;
    }

    status = W25Q_Read(address, readback, W25Q_PAGE_SIZE);
    if (status != STATUS_QSPI_IP_SUCCESS)
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "IMU LOG READBACK FAIL status=%d\r\n", (int)status);
        IMU_FlashUartPrint(uartBuff);
        return FALSE;
    }

    for (index = 0U; index < W25Q_PAGE_SIZE; index++)
    {
        if (data[index] != readback[index])
        {
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "IMU LOG VERIFY FAIL addr=0x%08lX offset=%lu\r\n",
                           (unsigned long)address, (unsigned long)index);
            IMU_FlashUartPrint(uartBuff);
            return FALSE;
        }
    }
    return TRUE;
}

static boolean IMU_LogMakeSectorActive(uint32 slot, uint32 generation, uint32 firstSequence)
{
    uint8 header[W25Q_PAGE_SIZE] __attribute__((aligned(4)));
    uint32 address = IMU_LogSectorAddress(slot);
    Qspi_Ip_StatusType status;
    uint16 crc;

    status = W25Q_EraseSector(address);
    if (status != STATUS_QSPI_IP_SUCCESS)
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "IMU LOG ERASE FAIL addr=0x%08lX status=%d\r\n",
                       (unsigned long)address, (int)status);
        IMU_FlashUartPrint(uartBuff);
        return FALSE;
    }

    (void)memset(header, 0xFF, sizeof(header));
    header[0] = (uint8)'I';
    header[1] = (uint8)'M';
    header[2] = (uint8)'U';
    header[3] = (uint8)'L';
    header[4] = IMU_LOG_HEADER_VERSION;
    IMU_LogPutU32(header, 8U, generation);
    IMU_LogPutU32(header, 12U, firstSequence);
    crc = IMU_LogCrc16(header, IMU_LOG_HEADER_CRC_OFFSET);
    IMU_LogPutU16(header, IMU_LOG_HEADER_CRC_OFFSET, crc);

    if (IMU_LogWriteAndVerifyPage(address, header) != TRUE)
    {
        return FALSE;
    }

    imuLogActiveSlot = slot;
    imuLogGeneration = generation;
    imuLogNextSequence = firstSequence;
    imuLogNextPage = IMU_LOG_FIRST_DATA_PAGE;
    imuLogRecordsBuffered = 0U;
    (void)memset(imuLogPage, 0xFF, sizeof(imuLogPage));
    return TRUE;
}

static boolean IMU_LogRotateSector(void)
{
    uint32 nextSlot = imuLogActiveSlot + 1U;

    if (nextSlot >= imuLogSectorCount)
    {
        nextSlot = 0U;
    }
    return IMU_LogMakeSectorActive(nextSlot,
                                   imuLogGeneration + 1U,
                                   imuLogNextSequence);
}

static boolean IMU_LogInit(void)
{
    uint8 header[W25Q_PAGE_SIZE] __attribute__((aligned(4)));
    uint8 page[W25Q_PAGE_SIZE] __attribute__((aligned(4)));
    uint32 slot;
    uint32 newestSlot = 0U;
    uint32 newestGeneration = 0U;
    uint32 firstSequence = 0U;
    uint32 pageIndex;
    uint32 recordIndex;
    uint32 lastUsedPage;
    uint16 storedCrc;
    uint16 calculatedCrc;
    boolean foundHeader = FALSE;
    Qspi_Ip_StatusType status;

    if (g_devInfo.totalSectors <= (IMU_LOG_FIRST_SECTOR + IMU_LOG_RESERVED_LAST))
    {
        IMU_FlashUartPrint("IMU LOG INIT FAIL: flash too small\r\n");
        return FALSE;
    }

    imuLogSectorCount = g_devInfo.totalSectors -
                        IMU_LOG_FIRST_SECTOR -
                        IMU_LOG_RESERVED_LAST;

    for (slot = 0U; slot < imuLogSectorCount; slot++)
    {
        status = W25Q_Read(IMU_LogSectorAddress(slot), header, W25Q_PAGE_SIZE);
        if (status != STATUS_QSPI_IP_SUCCESS)
        {
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "IMU LOG SCAN FAIL slot=%lu status=%d\r\n",
                           (unsigned long)slot, (int)status);
            IMU_FlashUartPrint(uartBuff);
            return FALSE;
        }

        if ((header[0] == (uint8)'I') &&
            (header[1] == (uint8)'M') &&
            (header[2] == (uint8)'U') &&
            (header[3] == (uint8)'L') &&
            (header[4] == IMU_LOG_HEADER_VERSION))
        {
            storedCrc = (uint16)((uint16)header[IMU_LOG_HEADER_CRC_OFFSET] |
                                 ((uint16)header[IMU_LOG_HEADER_CRC_OFFSET + 1U] << 8U));
            calculatedCrc = IMU_LogCrc16(header, IMU_LOG_HEADER_CRC_OFFSET);
            if (storedCrc == calculatedCrc)
            {
                uint32 generation = IMU_LogGetU32(header, 8U);
                if ((foundHeader == FALSE) || (generation > newestGeneration))
                {
                    foundHeader = TRUE;
                    newestSlot = slot;
                    newestGeneration = generation;
                    firstSequence = IMU_LogGetU32(header, 12U);
                }
            }
        }
    }

    if (foundHeader == FALSE)
    {
        if (IMU_LogMakeSectorActive(0U, 0U, 0U) != TRUE)
        {
            return FALSE;
        }
        IMU_FlashUartPrint("IMU LOG NEW REGION INITIALIZED\r\n");
        return TRUE;
    }

    imuLogActiveSlot = newestSlot;
    imuLogGeneration = newestGeneration;
    imuLogNextSequence = firstSequence;
    lastUsedPage = 0U;

    for (pageIndex = IMU_LOG_FIRST_DATA_PAGE;
         pageIndex <= IMU_LOG_DATA_PAGE_COUNT;
         pageIndex++)
    {
        uint32 pageAddress = IMU_LogSectorAddress(newestSlot) +
                             (pageIndex * W25Q_PAGE_SIZE);
        status = W25Q_Read(pageAddress, page, W25Q_PAGE_SIZE);
        if (status != STATUS_QSPI_IP_SUCCESS)
        {
            IMU_FlashUartPrint("IMU LOG RECOVERY READ FAIL\r\n");
            return FALSE;
        }

        if (IMU_LogPageIsErased(page) == FALSE)
        {
            lastUsedPage = pageIndex;
            for (recordIndex = 0U; recordIndex < IMU_LOG_RECORDS_PER_PAGE; recordIndex++)
            {
                uint32 recordOffset = recordIndex * IMU_LOG_RECORD_SIZE;
                uint32 sequence;
                storedCrc = (uint16)((uint16)page[recordOffset + IMU_LOG_CRC_OFFSET] |
                                     ((uint16)page[recordOffset + IMU_LOG_CRC_OFFSET + 1U] << 8U));
                calculatedCrc = IMU_LogCrc16(&page[recordOffset], IMU_LOG_CRC_OFFSET);
                if ((page[recordOffset] == IMU_LOG_RECORD_VERSION) &&
                    (storedCrc == calculatedCrc))
                {
                    sequence = IMU_LogGetU32(page, recordOffset + 1U);
                    imuLogNextSequence = sequence + 1U;
                }
            }
        }
    }

    imuLogNextPage = lastUsedPage + 1U;
    imuLogRecordsBuffered = 0U;
    (void)memset(imuLogPage, 0xFF, sizeof(imuLogPage));
    (void)snprintf(uartBuff, sizeof(uartBuff),
                   "IMU LOG RESUMED slot=%lu nextSeq=%lu\r\n",
                   (unsigned long)imuLogActiveSlot,
                   (unsigned long)imuLogNextSequence);
    IMU_FlashUartPrint(uartBuff);
    return TRUE;
}

static boolean IMU_LogFlushPage(void)
{
    uint32 address;

    if (imuLogRecordsBuffered == 0U)
    {
        return TRUE;
    }

    if (imuLogNextPage > IMU_LOG_DATA_PAGE_COUNT)
    {
        if (IMU_LogRotateSector() != TRUE)
        {
            return FALSE;
        }
    }

    address = IMU_LogSectorAddress(imuLogActiveSlot) +
              (imuLogNextPage * W25Q_PAGE_SIZE);
    if (IMU_LogWriteAndVerifyPage(address, imuLogPage) != TRUE)
    {
        return FALSE;
    }

    (void)snprintf(uartBuff, sizeof(uartBuff),
                   "IMU LOG PAGE PASS addr=0x%08lX records=%lu\r\n",
                   (unsigned long)address,
                   (unsigned long)imuLogRecordsBuffered);
    IMU_FlashUartPrint(uartBuff);

    imuLogNextPage++;
    imuLogRecordsBuffered = 0U;
    (void)memset(imuLogPage, 0xFF, sizeof(imuLogPage));
    return TRUE;
}

static boolean IMU_LogAppendSample(const asm330lhh_sample_t *sample)
{
    uint32 recordOffset;
    uint32 axis;
    uint16 value;
    uint16 crc;
    uint8 *record;

    if ((sample == NULL_PTR) || (imuLogReady != TRUE))
    {
        return FALSE;
    }

    recordOffset = imuLogRecordsBuffered * IMU_LOG_RECORD_SIZE;
    record = &imuLogPage[recordOffset];
    (void)memset(record, 0xFF, IMU_LOG_RECORD_SIZE);
    record[0] = IMU_LOG_RECORD_VERSION;
    IMU_LogPutU32(record, 1U, imuLogNextSequence);
    IMU_LogPutU32(record, 5U, Stm_Ip_GetCounterValue(0U));

    for (axis = 0U; axis < 3U; axis++)
    {
        value = (uint16)sample->accel_raw[axis];
        IMU_LogPutU16(record, 9U + (axis * 2U), value);
        value = (uint16)sample->gyro_raw[axis];
        IMU_LogPutU16(record, 15U + (axis * 2U), value);
    }
    IMU_LogPutU16(record, 21U, (uint16)sample->temperature_raw);
    crc = IMU_LogCrc16(record, IMU_LOG_CRC_OFFSET);
    IMU_LogPutU16(record, IMU_LOG_CRC_OFFSET, crc);

    imuLogNextSequence++;
    imuLogRecordsBuffered++;
    if (imuLogRecordsBuffered >= IMU_LOG_RECORDS_PER_PAGE)
    {
        return IMU_LogFlushPage();
    }

    return TRUE;
}

int main(void)
{
    Qspi_Ip_StatusType flashStatus;
    int32_t imuStatus;
    boolean flashReady = FALSE;

    clockStatus = Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);
    Mem_ExFls_ExampleAssert(CLOCK_IP_SUCCESS == clockStatus);

    GPIO_Init();
    Mem_ExFls_PinSetup();
    Uart_Init();
    Can_Init();
    Stm_Timer_Init();
    OsIf_Init(NULL);

    flashStatus = W25Q_ControllerInit();
    if (flashStatus == STATUS_QSPI_IP_SUCCESS)
    {
        IMU_FlashUartPrint("QSPI CONTROLLER INIT PASS\r\n");
        flashStatus = W25Q_MemoryInit();
        if (flashStatus == STATUS_QSPI_IP_SUCCESS)
        {
            IMU_FlashUartPrint("W25Q MEMORY INIT PASS\r\n");
            W25Q_CollectDeviceInfo();
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "W25Q JEDEC=%02X %02X %02X\r\n",
                           (unsigned int)g_devInfo.manufacturerId,
                           (unsigned int)g_devInfo.memoryType,
                           (unsigned int)g_devInfo.capacityId);
            IMU_FlashUartPrint(uartBuff);
            flashReady = (boolean)((g_devInfo.jedecReadSt == STATUS_QSPI_IP_SUCCESS) &&
                                   (g_devInfo.jedecValid == TRUE));
            if (flashReady != TRUE)
            {
                IMU_FlashUartPrint("W25Q ID CHECK FAIL; LOGGER DISABLED\r\n");
            }
            else
            {
                imuLogReady = IMU_LogInit();
                if (imuLogReady == TRUE)
                {
                    IMU_FlashUartPrint("IMU FLASH LOGGER READY\r\n");
                }
                else
                {
                    IMU_FlashUartPrint("IMU FLASH LOGGER INIT FAIL\r\n");
                }
            }
        }
        else
        {
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "W25Q MEMORY INIT FAIL status=%d\r\n", (int)flashStatus);
            IMU_FlashUartPrint(uartBuff);
        }
    }
    else
    {
        (void)snprintf(uartBuff, sizeof(uartBuff),
                       "QSPI CONTROLLER INIT FAIL status=%d\r\n", (int)flashStatus);
        IMU_FlashUartPrint(uartBuff);
    }

    imuStatus = IMU_Init();
    if (imuStatus == 0)
    {
        IMU_FlashUartPrint("IMU INIT PASS\r\n");
    }
    else
    {
        if (imuStatus == IMU_APP_STATUS_ID_MISMATCH)
        {
            IMU_FlashUartPrint("IMU INIT FAIL: WHO_AM_I mismatch\r\n");
        }
        else
        {
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "IMU INIT FAIL status=%ld; sampling will be retried\r\n",
                           (long)imuStatus);
            IMU_FlashUartPrint(uartBuff);
        }
    }

    sprintf(uartBuff, "RTD VERSION = %s SW MAJOR VERSION = %s\r\n", RTD_VERSION, SW_MAJOR_VERSION);
    Lpuart_Uart_Ip_SyncSend(LPUART_UART_IP_INSTANCE_USING_6, (const uint8 *)(uartBuff), strlen(uartBuff), 100000u);

    while (1)
    {
        imuStatus = IMU_ReadSample(&imuSample);
        if (imuStatus == 0)
        {
            snprintf(uartBuff, sizeof(uartBuff),
                     "IMU Accel=(%d,%d,%d) Gyro=(%d,%d,%d) Temp=%d\r\n",
                     (int)imuSample.accel_raw[0], (int)imuSample.accel_raw[1], (int)imuSample.accel_raw[2],
                     (int)imuSample.gyro_raw[0], (int)imuSample.gyro_raw[1], (int)imuSample.gyro_raw[2],
                     (int)imuSample.temperature_raw);
            Lpuart_Uart_Ip_SyncSend(LPUART_UART_IP_INSTANCE_USING_6, (const uint8 *)(uartBuff), strlen(uartBuff), 100000u);

            if (imuLogReady == TRUE)
            {
                if (IMU_LogAppendSample(&imuSample) != TRUE)
                {
                    imuLogReady = FALSE;
                    IMU_FlashUartPrint("IMU FLASH LOGGER STOPPED AFTER ERROR\r\n");
                }
            }
        }
        else
        {
            (void)snprintf(uartBuff, sizeof(uartBuff),
                           "IMU READ FAIL status=%ld; no flash write\r\n",
                           (long)imuStatus);
            IMU_FlashUartPrint(uartBuff);
        }

        TestDelay(1000000U);
    }

    return 0;
}

#ifdef __cplusplus
}
#endif
