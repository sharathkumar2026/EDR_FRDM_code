#include "asm330lhh_app.h"
#include "Mcal.h"
#include "Lpi2c_Ip.h"
#include <string.h>

#define IMU_I2C_INSTANCE           0U
#define IMU_I2C_TIMEOUT_MS         100U
#define IMU_I2C_MAX_TX_BYTES       16U

static stmdev_ctx_t g_imu_ctx;
static uint8_t g_imu_whoami = 0U;
static uint8_t g_imu_i2c_handle = IMU_I2C_INSTANCE;
static Lpi2c_Ip_BaudRateType g_imu_i2c_baudrate =
{
    .Prescaler = LPI2C_MASTER_PRESC_DIV_1,
    .ClkHI = 0x20U,
    .ClkLO = 0x20U,
    .SetHold = 0x00U,
    .DataValid = 0x00U
};
static const Lpi2c_Ip_MasterConfigType g_imu_i2c_config =
{
    .SlaveAddress = (uint16)(ASM330LHH_I2C_ADD_L >> 1U),
    .DebugMode = FALSE,
    .Is10bitAddr = FALSE,
    .OperatingMode = LPI2C_STANDARD_MODE,
    .BaudrateParams = &g_imu_i2c_baudrate,
    .PinLowTimeout = 0xFFFFU,
    .BusIdleTimeout = 0xFFFFU,
    .GlitchFilterSDA = 0U,
    .GlitchFilterSCL = 0U,
    .MasterCode = 0U,
    .TransferType = LPI2C_USING_INTERRUPTS,
    .DmaTxChannel = 0U,
    .DmaRxChannel = 0U,
    .MasterCallback = NULL,
    .CallbackParam = 0U,
    .MasterStateIdx = 0U
};

static int32_t IMU_I2C_Read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t instance = (handle != NULL) ? *(const uint8_t *)handle : IMU_I2C_INSTANCE;   /* FIX: handle points to the instance number */
    uint8_t reg_addr = reg;
    Lpi2c_Ip_StatusType status;

    if ((data == NULL) || (len == 0U))
    {
        return -1;
    }

    status = Lpi2c_Ip_MasterSendDataBlocking(instance, &reg_addr, 1U, FALSE, IMU_I2C_TIMEOUT_MS);
    if (status != LPI2C_IP_SUCCESS_STATUS)
    {
        return (int32_t)status;
    }

    status = Lpi2c_Ip_MasterReceiveDataBlocking(instance, data, len, TRUE, IMU_I2C_TIMEOUT_MS);
    return (status == LPI2C_IP_SUCCESS_STATUS) ? 0 : (int32_t)status;
}

static int32_t IMU_I2C_Write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t instance = (handle != NULL) ? *(const uint8_t *)handle : IMU_I2C_INSTANCE;   /* FIX: handle points to the instance number */
    uint8_t tx_buf[IMU_I2C_MAX_TX_BYTES + 1U];
    Lpi2c_Ip_StatusType status;

    if ((data == NULL) && (len != 0U))
    {
        return -1;
    }

    if (len > IMU_I2C_MAX_TX_BYTES)
    {
        return -1;
    }

    tx_buf[0] = reg;
    if (len != 0U)
    {
        memcpy(&tx_buf[1], data, len);
    }

    status = Lpi2c_Ip_MasterSendDataBlocking(instance, tx_buf, (uint32_t)len + 1U, TRUE, IMU_I2C_TIMEOUT_MS);
    return (status == LPI2C_IP_SUCCESS_STATUS) ? 0 : (int32_t)status;
}

int32_t IMU_Init(void)
{
    int32_t status;

    Lpi2c_Ip_MasterInit(IMU_I2C_INSTANCE, &g_imu_i2c_config);

    g_imu_ctx.write_reg = IMU_I2C_Write;
    g_imu_ctx.read_reg  = IMU_I2C_Read;
    g_imu_ctx.mdelay    = NULL;
    g_imu_ctx.handle    = &g_imu_i2c_handle;

    status = asm330lhh_device_id_get(&g_imu_ctx, &g_imu_whoami);
    if (status != 0)
    {
        return status;
    }

    if (g_imu_whoami != ASM330LHH_ID)
    {
        return IMU_APP_STATUS_ID_MISMATCH;
    }

    status = asm330lhh_reset_set(&g_imu_ctx, PROPERTY_ENABLE);
    if (status != 0) { return status; }
    status = asm330lhh_device_conf_set(&g_imu_ctx, PROPERTY_ENABLE);
    if (status != 0) { return status; }
    status = asm330lhh_block_data_update_set(&g_imu_ctx, PROPERTY_ENABLE);
    if (status != 0) { return status; }
    status = asm330lhh_xl_data_rate_set(&g_imu_ctx, ASM330LHH_XL_ODR_104Hz);
    if (status != 0) { return status; }
    status = asm330lhh_xl_full_scale_set(&g_imu_ctx, ASM330LHH_2g);
    if (status != 0) { return status; }
    status = asm330lhh_gy_data_rate_set(&g_imu_ctx, ASM330LHH_GY_ODR_104Hz);
    if (status != 0) { return status; }
    status = asm330lhh_gy_full_scale_set(&g_imu_ctx, ASM330LHH_1000dps);
    if (status != 0) { return status; }
    status = asm330lhh_xl_filter_lp2_set(&g_imu_ctx, PROPERTY_ENABLE);
    if (status != 0) { return status; }
    status = asm330lhh_xl_hp_path_on_out_set(&g_imu_ctx, ASM330LHH_LP_ODR_DIV_100);

    return status;
}

int32_t IMU_ReadSample(asm330lhh_sample_t *sample)
{
    int16_t raw_accel[3] = {0};
    int16_t raw_gyro[3] = {0};
    int16_t raw_temp = 0;
    int32_t status = 0;

    if (sample == NULL)
    {
        return -1;
    }

    memset(sample, 0, sizeof(*sample));

    status = asm330lhh_acceleration_raw_get(&g_imu_ctx, raw_accel);
    if (status != 0)
    {
        return status;
    }

    status = asm330lhh_angular_rate_raw_get(&g_imu_ctx, raw_gyro);
    if (status != 0)
    {
        return status;
    }

    status = asm330lhh_temperature_raw_get(&g_imu_ctx, &raw_temp);
    if (status != 0)
    {
        return status;
    }

    sample->accel_raw[0] = raw_accel[0];
    sample->accel_raw[1] = raw_accel[1];
    sample->accel_raw[2] = raw_accel[2];
    sample->gyro_raw[0]  = raw_gyro[0];
    sample->gyro_raw[1]  = raw_gyro[1];
    sample->gyro_raw[2]  = raw_gyro[2];
    sample->temperature_raw = raw_temp;

    sample->accel_mg[0] = asm330lhh_from_fs2g_to_mg(raw_accel[0]);
    sample->accel_mg[1] = asm330lhh_from_fs2g_to_mg(raw_accel[1]);
    sample->accel_mg[2] = asm330lhh_from_fs2g_to_mg(raw_accel[2]);
    sample->gyro_mdps[0] = asm330lhh_from_fs1000dps_to_mdps(raw_gyro[0]);
    sample->gyro_mdps[1] = asm330lhh_from_fs1000dps_to_mdps(raw_gyro[1]);
    sample->gyro_mdps[2] = asm330lhh_from_fs1000dps_to_mdps(raw_gyro[2]);
    sample->temperature_c = (float_t)raw_temp / 16.0f;

    return status;
}
