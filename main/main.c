#include <stdio.h>
#include "esp_err.h"
#include "mpu6500.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
mpu6500_accel_data data;

void app_main(void)
{

    esp_err_t err;
    err = mpu6500_init();
    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Init failed: %s", esp_err_to_name(err));
        return;
    }
    while (1)
    {
        err = mpu6500_accel_read(&data);
        if (err != ESP_OK)
        {
            ESP_LOGI("MPU6500", "ACCEL READ FAIL: %s", esp_err_to_name(err));
            return;
        }
        printf("X acceleration: %.2lf g\n Y acceleration: %.2lf g\n Z acceleration: %.2lf g\n", data.x, data.y, data.z);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}