/*
 * bno055.h
 *
 *  BNO055 IMU Driver for STM32F407 (HAL I2C)
 *  Reads Euler angles: X (Yaw / Heading), Y (Roll), Z (Pitch)
 *  Units: degrees * 16 (raw), converted to float degrees in driver.
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stm32f4xx_hal.h"

/* -----------------------------------------------------------------------
 * I2C Address
 * BNO055 default 7-bit address when ADR pin = GND: 0x28
 * When ADR pin = VCC: 0x29
 * HAL uses 8-bit shifted address
 * ----------------------------------------------------------------------- */
#define BNO055_I2C_ADDR_LOW   (0x28 << 1)   /* ADR pin = GND */
#define BNO055_I2C_ADDR_HIGH  (0x29 << 1)   /* ADR pin = VCC */
#define BNO055_I2C_ADDR        BNO055_I2C_ADDR_LOW

/* -----------------------------------------------------------------------
 * Register Map (Page 0)
 * ----------------------------------------------------------------------- */
#define BNO055_REG_CHIP_ID        0x00
#define BNO055_REG_PAGE_ID        0x07
#define BNO055_REG_OPR_MODE       0x3D
#define BNO055_REG_PWR_MODE       0x3E
#define BNO055_REG_SYS_TRIGGER    0x3F
#define BNO055_REG_UNIT_SEL       0x3B

/* Euler angle output registers (6 bytes: H-LSB, H-MSB, R-LSB, R-MSB, P-LSB, P-MSB) */
#define BNO055_REG_EUL_HEADING_LSB  0x1A
#define BNO055_REG_EUL_ROLL_LSB     0x1C
#define BNO055_REG_EUL_PITCH_LSB    0x1E

/* Euler data block start - read 6 bytes from 0x1A for H, R, P */
#define BNO055_REG_EUL_DATA_X_LSB   0x1A   /* Heading (Yaw) */

/* Calibration status */
#define BNO055_REG_CALIB_STAT     0x35

/* -----------------------------------------------------------------------
 * Chip ID value
 * ----------------------------------------------------------------------- */
#define BNO055_CHIP_ID_VALUE      0xA0

/* -----------------------------------------------------------------------
 * Operation modes
 * ----------------------------------------------------------------------- */
#define BNO055_OPR_MODE_CONFIG    0x00
#define BNO055_OPR_MODE_NDOF      0x0C   /* 9-DOF fusion - all sensors on */
#define BNO055_OPR_MODE_IMU       0x08   /* Accel + Gyro fusion */

/* -----------------------------------------------------------------------
 * Data structure
 * ----------------------------------------------------------------------- */
typedef struct
{
    float x;   /* Heading (Yaw)  — 0 to 360.0 degrees */
    float y;   /* Roll           — -180.0 to +180.0 degrees */
    float z;   /* Pitch          — -90.0 to +90.0 degrees */
} BNO055_Euler_t;

/* -----------------------------------------------------------------------
 * Calibration status bits (0=uncalibrated, 3=fully calibrated)
 * ----------------------------------------------------------------------- */
typedef struct
{
    uint8_t sys;    /* System calibration (0-3) */
    uint8_t gyro;   /* Gyroscope (0-3) */
    uint8_t accel;  /* Accelerometer (0-3) */
    uint8_t mag;    /* Magnetometer (0-3) */
} BNO055_CalibStatus_t;

/* -----------------------------------------------------------------------
 * Function Prototypes
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialize BNO055 in NDOF (9-DOF) fusion mode.
 * @param  hi2c  Pointer to HAL I2C handle (e.g. &hi2c1)
 * @retval HAL_OK on success, HAL_ERROR if sensor not detected
 */
HAL_StatusTypeDef BNO055_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read Euler angles from BNO055.
 * @param  hi2c   Pointer to HAL I2C handle
 * @param  euler  Pointer to BNO055_Euler_t struct to fill
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef BNO055_Read_Euler(I2C_HandleTypeDef *hi2c, BNO055_Euler_t *euler);

/**
 * @brief  Read calibration status from BNO055.
 * @param  hi2c    Pointer to HAL I2C handle
 * @param  status  Pointer to BNO055_CalibStatus_t struct to fill
 * @retval HAL_OK on success
 */
HAL_StatusTypeDef BNO055_Read_CalibStatus(I2C_HandleTypeDef *hi2c, BNO055_CalibStatus_t *status);

#endif /* INC_BNO055_H_ */
