    #ifndef ASM330LHH_APP_H
    #define ASM330LHH_APP_H

    #include <stdint.h>
    #include "asm330lhh_reg.h"

    #define IMU_APP_STATUS_ID_MISMATCH (-100)

    typedef struct
    {
        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        int16_t temperature_raw;
        float accel_mg[3];
        float gyro_mdps[3];
        float temperature_c;
    } asm330lhh_sample_t;

    int32_t IMU_Init(void);
    int32_t IMU_ReadSample(asm330lhh_sample_t *sample);

    #endif /* ASM330LHH_APP_H */
