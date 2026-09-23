/*
 * bno055.c
 *
 *  BNO055 IMU Driver for STM32F407 (HAL I2C)
 *  Auto-detects I2C address: tries 0x28 (ADR=GND) then 0x29 (ADR=VCC)
 *
 *  Wiring:
 *    PB6  --> I2C1 SCL
 *    PB7  --> I2C1 SDA
 *    VCC_5V or 3.3V --> BNO055 VIN
 *    GND  --> BNO055 GND, ADR, PS0, PS1
 */

#include "bno055.h"
#include "stm32f4xx_hal.h"

/* Internal timeout for all I2C transactions */
#define BNO055_I2C_TIMEOUT_MS   100U

/* Detected working I2C address (set during BNO055_Init) */
static uint16_t bno055_addr = BNO055_I2C_ADDR_LOW;

/* ---------------------------------------------------------------------------
 * Low-level helpers — use bno055_addr (auto-detected)
 * --------------------------------------------------------------------------- */

static HAL_StatusTypeDef BNO055_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg,
                                          uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    return HAL_I2C_Master_Transmit(hi2c,
                                   bno055_addr,
                                   buf, 2,
                                   BNO055_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BNO055_ReadRegs(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg,
                                          uint8_t *buf,
                                          uint16_t len)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(hi2c,
                                   bno055_addr,
                                   &reg, 1,
                                   BNO055_I2C_TIMEOUT_MS);
    if (ret != HAL_OK)
        return ret;

    return HAL_I2C_Master_Receive(hi2c,
                                   bno055_addr,
                                   buf, len,
                                   BNO055_I2C_TIMEOUT_MS);
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */

HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t chip_id = 0;

    /* ---- 1. Wait for BNO055 boot (650ms after power-on) ---- */
    HAL_Delay(700);

    /* ---- 2. Auto-detect I2C address: try 0x28 then 0x29 ---- */
    uint16_t addrs[2] = { BNO055_I2C_ADDR_LOW, BNO055_I2C_ADDR_HIGH };
    uint8_t  found = 0;

    for (int a = 0; a < 2; a++)
    {
        bno055_addr = addrs[a];
        chip_id = 0;

        if (BNO055_ReadRegs(hi2c, BNO055_REG_CHIP_ID, &chip_id, 1) == HAL_OK
            && chip_id == BNO055_CHIP_ID_VALUE)
        {
            found = 1;
            break;
        }
        HAL_Delay(10);
    }

    if (!found)
        return HAL_ERROR;   /* Not found at 0x28 or 0x29 — check wiring */

    /* ---- 3. Switch to CONFIG mode ---- */
    if (BNO055_WriteReg(hi2c, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIG) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(25);

    /* ---- 4. System reset ---- */
    if (BNO055_WriteReg(hi2c, BNO055_REG_SYS_TRIGGER, 0x20) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(700);

    /* ---- 5. Re-verify chip ID after reset ---- */
    chip_id = 0;
    if (BNO055_ReadRegs(hi2c, BNO055_REG_CHIP_ID, &chip_id, 1) != HAL_OK)
        return HAL_ERROR;
    if (chip_id != BNO055_CHIP_ID_VALUE)
        return HAL_ERROR;

    /* ---- 6. Normal power mode ---- */
    if (BNO055_WriteReg(hi2c, BNO055_REG_PWR_MODE, 0x00) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    /* ---- 7. Degrees + m/s^2 units ---- */
    if (BNO055_WriteReg(hi2c, BNO055_REG_UNIT_SEL, 0x00) != HAL_OK)
        return HAL_ERROR;

    /* ---- 8. Page 0 ---- */
    if (BNO055_WriteReg(hi2c, BNO055_REG_PAGE_ID, 0x00) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    /* ---- 9. NDOF fusion mode ---- */
    if (BNO055_WriteReg(hi2c, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_NDOF) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(20);

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_Read_Euler(I2C_HandleTypeDef *hi2c, BNO055_Euler_t *euler)
{
    uint8_t raw[6];

    if (BNO055_ReadRegs(hi2c, BNO055_REG_EUL_DATA_X_LSB, raw, 6) != HAL_OK)
        return HAL_ERROR;

    int16_t heading_raw = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    int16_t roll_raw    = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    int16_t pitch_raw   = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);

    euler->x = (float)heading_raw / 16.0f;   /* Yaw   0..360 */
    euler->y = (float)roll_raw    / 16.0f;   /* Roll  -180..+180 */
    euler->z = (float)pitch_raw   / 16.0f;   /* Pitch -90..+90 */

    return HAL_OK;
}

HAL_StatusTypeDef BNO055_Read_CalibStatus(I2C_HandleTypeDef *hi2c, BNO055_CalibStatus_t *status)
{
    uint8_t calib_byte = 0;

    if (BNO055_ReadRegs(hi2c, BNO055_REG_CALIB_STAT, &calib_byte, 1) != HAL_OK)
        return HAL_ERROR;

    status->sys   = (calib_byte >> 6) & 0x03;
    status->gyro  = (calib_byte >> 4) & 0x03;
    status->accel = (calib_byte >> 2) & 0x03;
    status->mag   = (calib_byte >> 0) & 0x03;

    return HAL_OK;
}
