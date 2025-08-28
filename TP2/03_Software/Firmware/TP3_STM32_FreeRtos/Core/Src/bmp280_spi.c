#include "bmp280_spi.h"
#include "gpio.h"

static SPI_HandleTypeDef *bmp_spi;
static GPIO_TypeDef *bmp_cs_port;
static uint16_t bmp_cs_pin;
static BMP280_CalibData bmp_calib;
static int32_t t_fine;

static void CS_Select(void) {
    HAL_GPIO_WritePin(bmp_cs_port, bmp_cs_pin, GPIO_PIN_RESET);
}

static void CS_Unselect(void) {
    HAL_GPIO_WritePin(bmp_cs_port, bmp_cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef BMP280_Write8(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg & 0x7F, value};
    CS_Select();
    HAL_StatusTypeDef res = HAL_SPI_Transmit(bmp_spi, data, 2, BMP280_SPI_TIMEOUT);
    CS_Unselect();
    return res;
}

static HAL_StatusTypeDef BMP280_ReadBytes(uint8_t reg, uint8_t *buffer, uint16_t length) {
    reg |= 0x80; // Read bit
    CS_Select();
    HAL_StatusTypeDef res1 = HAL_SPI_Transmit(bmp_spi, &reg, 1, BMP280_SPI_TIMEOUT);
    HAL_StatusTypeDef res2 = HAL_SPI_Receive(bmp_spi, buffer, length, BMP280_SPI_TIMEOUT);
    CS_Unselect();
    return (res1 == HAL_OK && res2 == HAL_OK) ? HAL_OK : HAL_ERROR;
}

static bool BMP280_ReadCalibration(void) {
    uint8_t calib[24];
    if (BMP280_ReadBytes(BMP280_REG_CALIB_START, calib, 24) != HAL_OK)
        return false;

    bmp_calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp_calib.dig_T2 = (int16_t)(calib[3]  << 8 | calib[2]);
    bmp_calib.dig_T3 = (int16_t)(calib[5]  << 8 | calib[4]);

    bmp_calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    bmp_calib.dig_P2 = (int16_t)(calib[9]  << 8 | calib[8]);
    bmp_calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    bmp_calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    bmp_calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    bmp_calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    bmp_calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    bmp_calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    bmp_calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);

    return true;
}

bool BMP280_SPI_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    bmp_spi = hspi;
    bmp_cs_port = cs_port;
    bmp_cs_pin = cs_pin;

    CS_Unselect();
    HAL_Delay(100);

    // Reset
    if (BMP280_Write8(BMP280_REG_RESET, 0xB6) != HAL_OK)
        return false;

    HAL_Delay(100);

    uint8_t id = 0;
    if (BMP280_ReadBytes(BMP280_REG_ID, &id, 1) != HAL_OK || id != 0x58)
        return false;

    if (!BMP280_ReadCalibration())
        return false;

    if (BMP280_Write8(BMP280_REG_CTRL_MEAS, 0x27) != HAL_OK)
        return false;

    if (BMP280_Write8(BMP280_REG_CONFIG, 0x00) != HAL_OK)
        return false;

    return true;
}

float BMP280_ReadTemperature(void) {
    uint8_t data[3];
    if (BMP280_ReadBytes(BMP280_REG_TEMP_MSB, data, 3) != HAL_OK)
        return -1000.0f;

    int32_t Temperatura_Reg_BMP280 = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);	//	Temperatura en BMP280 sin compensar

    int32_t var1 = ((((Temperatura_Reg_BMP280 >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2 = (((((Temperatura_Reg_BMP280 >> 4) - ((int32_t)bmp_calib.dig_T1)) *
                      ((Temperatura_Reg_BMP280 >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) *
                    ((int32_t)bmp_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float Temperatura_compensada = (t_fine * 5 + 128) >> 8;	// Temperatura ± 0.1°C
    return Temperatura_compensada / 100.0f;
}

float BMP280_ReadPressure(void) {
    uint8_t data[3];
    if (BMP280_ReadBytes(BMP280_REG_PRESS_MSB, data, 3) != HAL_OK)
        return -1.0f;

    int32_t Presion_BMP280 = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);		//	Presión en BMP280 sin compensar
    int64_t var1, var2, presion_compensada;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp_calib.dig_P6;
    var2 += (var1 * (int64_t)bmp_calib.dig_P5) << 17;
    var2 += ((int64_t)bmp_calib.dig_P4) << 35;
    var1 = ((var1 * var1 * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp_calib.dig_P1) >> 33;

    if (var1 == 0) return 0; // avoid division by zero

    presion_compensada = 1048576 - Presion_BMP280;
    presion_compensada = (((presion_compensada<< 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_calib.dig_P9) * (presion_compensada>> 13) * (presion_compensada>> 13)) >> 25;
    var2 = (((int64_t)bmp_calib.dig_P8) * presion_compensada) >> 19;

    presion_compensada= ((presion_compensada + var1 + var2) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    return (float)presion_compensada / 25600.0f;
}
