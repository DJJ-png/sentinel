/**
  ****************************(C)2025 ckyf****************************
  * @file      ina226.h/.c
  * @brief     ina226的封装。
  * @note       
  * @history
  *  Version    Date            Author         
  *  V1.1.0    30-12-2025        kuyu        
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C)2025 ckyf****************************
  */

#include "ina226.h"

ina226_data_t ina226_data;
uint16_t dect[10];

/**
  * @brief          INA226_init
  * @param[in]      DevAddress: ina226地址
  * @retval         none
  */
void INA226_init(uint16_t DevAddress)
{                                                      
	ina226_data.vol=0.0f;
	ina226_data.current=0.0f;
	ina226_data.power=0.0f;
    INA226_SetConfig(0x4727,DevAddress);
    INA226_SetCalibrationReg(0x0a01,DevAddress);
}
/**
  * @brief          获取电压
  * @param[in]      DevAddress: ina226地址
  * @retval         none
  */
float INA226_GetBusV(uint16_t DevAddress)
{
	uint16_t regData;
	float fVoltage;
    regData = INA226_GetBusVReg(DevAddress);
	dect[0]=regData;
	fVoltage = regData * 0.00125f;/*LSB = 1.25mV*/
	return fVoltage;
}

/**
  * @brief          获取电流
  * @param[in]      DevAddress: ina226地址
  * @retval         none
  */
float INA226_GetCurrent(uint16_t DevAddress)
{
	uint16_t regData;
	float fCurrent;
	regData = INA226_GetCurrentReg(DevAddress);
	if(regData >= 0x8000)	regData = 0;
	fCurrent = regData * 0.0002f;/*???LSB = 0.2mA,?????*/
	return fCurrent;
}
/**
  * @brief          获取功率
  * @param[in]      DevAddress: ina226地址
  * @retval         none
  */
float INA226_GetPower(uint16_t DevAddress)
{
	uint16_t regData;
	float fPower;
	regData = INA226_GetPowerReg(DevAddress);
	dect[2]=regData;
	fPower = regData * 0.005f;/*???LSB = ???LSB*25*/
	return fPower;
}
/**
  * @brief          ina设置配置寄存器
  * @param[in]      DevAddress: ina226地址
  * @param[in]      ConfigWord: 配置的十六进制值
  * @retval         none
  */
uint8_t INA226_SetConfig(uint16_t ConfigWord,uint16_t DevAddress)
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_CONFIG;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(&HI2CX, DevAddress, SentTable, 3, 0x100);
}

uint16_t INA226_GetBusVReg(uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_BUSV};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&HI2CX,DevAddress, SentTable, 1, 0x100);
    if (HAL_I2C_Master_Receive(&HI2CX,DevAddress, ReceivedTable, 2, 0x100) != HAL_OK){ 
			return 0xFF;
		}
    else {
			return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
		}

}

uint8_t INA226_SetCalibrationReg(uint16_t ConfigWord,uint16_t DevAddress)
{
    uint8_t SentTable[3];
    SentTable[0] = INA226_CALIB;
    SentTable[1] = (ConfigWord & 0xFF00) >> 8;
    SentTable[2] = (ConfigWord & 0x00FF);
    return HAL_I2C_Master_Transmit(&HI2CX, DevAddress, SentTable, 3, 0x100);
}

uint16_t INA226_GetPowerReg(uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_POWER};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&HI2CX,DevAddress, SentTable, 1, 0x100);
    if (HAL_I2C_Master_Receive(&HI2CX,DevAddress, ReceivedTable, 2, 0x100) != HAL_OK) {
			return 0xFF;
		}
    else {
			dect[3]=ReceivedTable[0];
			dect[3]=ReceivedTable[1];
			return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
		}
}

uint16_t INA226_GetCurrentReg(uint16_t DevAddress)
{
    uint8_t SentTable[1] = {INA226_CURRENT};
    uint8_t ReceivedTable[2];
    HAL_I2C_Master_Transmit(&HI2CX,DevAddress, SentTable, 1, 0x10);
    if (HAL_I2C_Master_Receive(&HI2CX,DevAddress, ReceivedTable, 2, 0x100) != HAL_OK) return 0xFF;
    else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}



