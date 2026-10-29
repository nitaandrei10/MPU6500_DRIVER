#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include "mpu6500.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define AUTO_I2C_PORT -1

// Declaring pins for I2C comms
static const uint8_t SDA = 21;
static const uint8_t SCL = 22;

// Decalring handles
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t device_handle;

// Decalring register addresses
static const uint8_t pwr_reg = 0x6B;
static const uint8_t acc_config_reg = 0x1C;

static const uint8_t XOUT_H_REG = 0x3B;
static const uint8_t XOUT_L_REG = 0x3C;

static const uint8_t YOUT_H_REG = 0x3D;
static const uint8_t YOUT_L_REG = 0x3E;

static const uint8_t ZOUT_H_REG = 0x3F;
static const uint8_t ZOUT_L_REG = 0x40;

esp_err_t mpu6500_write_reg(uint8_t reg, uint8_t val)
{
    esp_err_t err;
    uint8_t data[] = {reg, val};
    err = i2c_master_transmit(device_handle, data, 2, 1000);
    return err;
}

esp_err_t mpu6500_read_reg(uint8_t reg, uint8_t *val)
{
    if (val == NULL)
    {
        ESP_LOGI("MPU6500", "Value is null\n");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = i2c_master_transmit_receive(device_handle, &reg, 1, val, 1, 1000);
    // ESP_LOGI("MPU6500", "Read register 0x%02X: %s",
    //          reg, esp_err_to_name(err));
    return err;
}

esp_err_t mpu6500_init(void)
{
    uint8_t val;
    esp_err_t err;
    i2c_master_bus_config_t setup = {
        .i2c_port = AUTO_I2C_PORT,
        .scl_io_num = SCL,
        .sda_io_num = SDA,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&setup, &bus);
    if (err != ESP_OK)
    {
        return err;
    }
    err = i2c_master_probe(bus, 0x68, 1000);
    ESP_LOGI("MPU6500", "Probe result: %s", esp_err_to_name(err));
    const i2c_device_config_t device_setup = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = 100000,
    };
    err = i2c_master_bus_add_device(bus, &device_setup, &device_handle);
    if (err != ESP_OK)
    {
        return err;
    }
    // wake up the sensor module

    err = mpu6500_read_reg(pwr_reg, &val);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Power register read error: %s\n", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI("MPU_TEST", "Register before write: 0x%02X\n", val);

    val &= ~(1U << 6);
    err = mpu6500_write_reg(pwr_reg, val);
    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "WRITE FAILED : %s", esp_err_to_name(err));
    }
    return err;
}
esp_err_t mpu6500_accel_read(mpu6500_accel_data *data)
{
    // configuring the accelerometer
    if (data == NULL)
    {
        ESP_LOGI("MPU6500", "Data pointer is null\n");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t val;
    esp_err_t err = mpu6500_read_reg(acc_config_reg, &val);
    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Fail in accel init read %s\n", esp_err_to_name(err));
        return err;
    }

    val &= ~((1U << 3) | (1U << 4)); // enables the +- 2g range

    err = mpu6500_write_reg(acc_config_reg, val);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Fail in accel init write %s\n", esp_err_to_name(err));
        return err;
    }

    // reading measurements from the sensor
    uint8_t XOUT_L;
    uint8_t XOUT_H;

    err = mpu6500_read_reg(XOUT_H_REG, &XOUT_H);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Failed to read X HIGH BITS %s\n", esp_err_to_name(err));
        return err;
    }

    err = mpu6500_read_reg(XOUT_L_REG, &XOUT_L);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Failed to read X LOW BITS %s\n", esp_err_to_name(err));
        return err;
    }

    int16_t X_VAL = (XOUT_H << 8) | XOUT_L;

    data->x = (double)(X_VAL / 16384);
    // CONTINUE FOR THE Y AND Z VALUES

    uint8_t YOUT_L;
    uint8_t YOUT_H;

    err = mpu6500_read_reg(YOUT_H_REG, &YOUT_H);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Failed to read Y HIGH BITS %s\n", esp_err_to_name(err));
        return err;
    }

    err = mpu6500_read_reg(YOUT_L_REG, &YOUT_L);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Failed to read Y LOW BITS %s\n", esp_err_to_name(err));
        return err;
    }

    int16_t Y_VAL = (YOUT_H << 8) | YOUT_L;

    data->y = (double)(Y_VAL / 16384);

    // Z VALUES

    uint8_t ZOUT_L;
    uint8_t ZOUT_H;

    err = mpu6500_read_reg(ZOUT_H_REG, &ZOUT_H);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Failed to read Z HIGH BITS %s\n", esp_err_to_name(err));
        return err;
    }

    err = mpu6500_read_reg(ZOUT_L_REG, &ZOUT_L);

    if (err != ESP_OK)
    {
        ESP_LOGI("MPU6500", "Failed to read Z LOW BITS %s\n", esp_err_to_name(err));
        return err;
    }

    int16_t Z_VAL = (ZOUT_H << 8) | ZOUT_L;

    data->z = (double)(Z_VAL / 16384);

    return ESP_OK;
}