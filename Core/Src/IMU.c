/*
 * IMU.c
 *
 *  Created on: 11 May 2026
 *      Author: Prashan Anjanpalage
 */

#include "IMU.h"
#include "config.h"

/* LSM6DSOX power/config registers */
#define IMU_CTRL1_XL   0x10   /* accelerometer power/config */
#define IMU_CTRL2_G    0x11   /* gyroscope power/config */
#define IMU_CTRL3_C    0x12   /* common control 3: BDU, IF_INC, etc. */

#define CTRL3_C_BDU    (1 << 6)  /* Block Data Update: L/H output bytes latch together */
#define CTRL3_C_IF_INC (1 << 2)  /* register address auto-increment (already default-on, set explicitly) */

/* Register bits and LSB sensitivities for the ranges chosen in config.h
 * (LSM6DSOX datasheet Table 2, CTRL1_XL / CTRL2_G). ODR is bits [7:4],
 * full-scale is bits [3:2] for both registers. */
#if   IMU_ODR_HZ == 104
#define IMU_ODR_BITS (0x4 << 4)
#elif IMU_ODR_HZ == 208
#define IMU_ODR_BITS (0x5 << 4)
#elif IMU_ODR_HZ == 416
#define IMU_ODR_BITS (0x6 << 4)
#elif IMU_ODR_HZ == 833
#define IMU_ODR_BITS (0x7 << 4)
#else
#error "IMU_ODR_HZ must be 104, 208, 416 or 833"
#endif

#if   IMU_ACCEL_FS_G == 2
#define IMU_ACCEL_FS_BITS  (0x0 << 2)
#define IMU_ACCEL_MG_PER_LSB 0.061f
#elif IMU_ACCEL_FS_G == 4
#define IMU_ACCEL_FS_BITS  (0x2 << 2)
#define IMU_ACCEL_MG_PER_LSB 0.122f
#elif IMU_ACCEL_FS_G == 8
#define IMU_ACCEL_FS_BITS  (0x3 << 2)
#define IMU_ACCEL_MG_PER_LSB 0.244f
#elif IMU_ACCEL_FS_G == 16
#define IMU_ACCEL_FS_BITS  (0x1 << 2)
#define IMU_ACCEL_MG_PER_LSB 0.488f
#else
#error "IMU_ACCEL_FS_G must be 2, 4, 8 or 16"
#endif

#if   IMU_GYRO_FS_DPS == 250
#define IMU_GYRO_FS_BITS     (0x0 << 2)
#define IMU_GYRO_MDPS_PER_LSB 8.75f
#elif IMU_GYRO_FS_DPS == 500
#define IMU_GYRO_FS_BITS     (0x1 << 2)
#define IMU_GYRO_MDPS_PER_LSB 17.5f
#elif IMU_GYRO_FS_DPS == 1000
#define IMU_GYRO_FS_BITS     (0x2 << 2)
#define IMU_GYRO_MDPS_PER_LSB 35.0f
#elif IMU_GYRO_FS_DPS == 2000
#define IMU_GYRO_FS_BITS     (0x3 << 2)
#define IMU_GYRO_MDPS_PER_LSB 70.0f
#else
#error "IMU_GYRO_FS_DPS must be 250, 500, 1000 or 2000"
#endif

static void SPI_WriteRegister(IMU *dev, uint8_t address, uint8_t data)
{
    uint8_t transmit[2] = { (uint8_t)(address & 0x7F), data }; /* bit 7 = 0 for write */

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi_handle, transmit, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

uint8_t IMU_Initialise(IMU *dev, SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->spi_handle = spi_handle;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;
    dev->acceleration[0] = 0.0f;
    dev->acceleration[1] = 0.0f;
    dev->acceleration[2] = 0.0f;
    dev->gyro[0] = 0.0f;
    dev->gyro[1] = 0.0f;
    dev->gyro[2] = 0.0f;

    /* CS must idle high (deselected) before the first transaction, and the
     * device needs its full turn-on time (Ton = 35 ms max, datasheet Table 4)
     * after VDD is applied before it will respond correctly. If IMU_Initialise
     * runs immediately after HAL_Init()/SystemClock_Config(), VDD may only just
     * have stabilised, so this delay is what was causing WHO_AM_I reads to fail
     * intermittently on cold boot. */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(40);

    uint8_t whoami = SPI_ReadRegister(dev, IMU_WHO_AM_I_REG);
    if (whoami != IMU_WHOAMI_VAL) {
        return whoami;
    }

    /* BDU=1: OUT_x_L/OUT_x_H are frozen together until both bytes of a given
     * axis have been read, so a read landing across the sensor's own internal
     * ODR update boundary can't return a "torn" sample (high byte from one
     * conversion, low byte from the next). This matters here because
     * ReadAcceleration/ReadGyroscope issue the Hi and Lo byte reads as two
     * separate SPI transactions rather than one burst read.
     * IF_INC=1 is the power-on default, set explicitly for clarity. */
    SPI_WriteRegister(dev, IMU_CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC);

    uint8_t accelPower = IMU_ODR_BITS | IMU_ACCEL_FS_BITS; /* first-stage digital filtering output */
    uint8_t gyroPower  = IMU_ODR_BITS | IMU_GYRO_FS_BITS;

    SPI_WriteRegister(dev, IMU_CTRL1_XL, accelPower);
    SPI_WriteRegister(dev, IMU_CTRL2_G, gyroPower);

    return whoami;
}

/* Sentinel returned when the SPI transfer itself fails (HAL_ERROR/HAL_TIMEOUT/HAL_BUSY),
 * as opposed to a real byte read back from the device (which could legitimately be 0xFF).
 */
#define IMU_HAL_FAIL 0xFE

uint8_t SPI_ReadRegister(IMU *dev, uint8_t address)
{
    uint8_t transmit[2] = { (uint8_t)(READ | address), 0x00 };
    uint8_t receive[2]  = { 0 };

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->spi_handle, transmit, receive, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        return IMU_HAL_FAIL;
    }

    return receive[1];
}

void ReadGyroscope(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index)
{
    uint8_t dataHi = SPI_ReadRegister(dev, addressHi);
    uint8_t dataLo = SPI_ReadRegister(dev, addressLo);

    int16_t gyrodata = (int16_t)((dataHi << 8) | dataLo);

    dev->gyro[index] = IMU_GYRO_MDPS_PER_LSB * gyrodata; /* output in mdps */
}

void ReadAcceleration(IMU *dev, uint8_t addressHi, uint8_t addressLo, uint8_t index)
{
    uint8_t dataHi = SPI_ReadRegister(dev, addressHi);
    uint8_t dataLo = SPI_ReadRegister(dev, addressLo);

    int16_t acceldata = (int16_t)((dataHi << 8) | dataLo);

    dev->acceleration[index] = IMU_ACCEL_MG_PER_LSB * acceldata; /* output in mg */
}

/* Convenience wrapper: reads all 6 axes in one call so main doesn't have to
 * remember all 12 register addresses / (Hi, Lo, index) triples every loop. */
void ReadAllIMU(IMU *dev)
{
    ReadAcceleration(dev, accelXRegHi, accelXRegLow, 0);
    ReadAcceleration(dev, accelYRegHi, accelYRegLow, 1);
    ReadAcceleration(dev, accelZRegHi, accelZRegLow, 2);

    ReadGyroscope(dev, gyroXRegHi, gyroXRegLow, 0);
    ReadGyroscope(dev, gyroYRegHi, gyroYRegLow, 1);
    ReadGyroscope(dev, gyroZRegHi, gyroZRegLow, 2);
}
