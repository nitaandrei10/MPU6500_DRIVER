#ifndef MPU_6500_H_
#define MPU_6500_H_

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct accel_data
    {
        double x;
        double y;
        double z;
    } mpu6500_accel_data;

    esp_err_t mpu6500_write_reg(uint8_t reg, uint8_t val);
    esp_err_t mpu6500_init(void);
    esp_err_t mpu6500_accel_read(mpu6500_accel_data *data);
    esp_err_t mpu6500_read_reg(uint8_t reg, uint8_t *val);

#ifdef __cplusplus
}
#endif

#endif