// Whats up suckaaaas
#include<PCA9685.h>

uint8_t PCA9685_PWM_init(hPCA9685 *hpca, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
	hpca->address = (addr<<1);
	hpca->hi2c = hi2c;

	// check device availability
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, hpca->address, 2, 10);
	if(status != HAL_OK)
	{
		return 0x01;
	}

	//set default mode
	uint8_t mode1 = PCA9685_MODE1_DEFAULT;
	status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, PCA9685_MODE1_REGADDR, 1, &mode1, 1, HAL_MAX_DELAY);
	if(status != HAL_OK)
	{
		return 0x02;
	}
	return 0x00;
}


uint8_t PCA9685_PWM_write(hPCA9685 *hpca, uint8_t pin, uint16_t val)
{
	if (pin > 15) return 0x01;               // only 0–15 valid
	if (val > 4095) val = 4095;         // clamp 12-bit val

	uint8_t reg_base = PCA9685_LED0_ON_L_REGADDR + 4 * pin;

	// ON = 0, OFF = val
	uint8_t pwm_data[4];
	pwm_data[0] = 0 & 0xFF;          // ON_L
	pwm_data[1] = 0 >> 8;            // ON_H
	pwm_data[2] = val & 0xFF;      // OFF_L
	pwm_data[3] = val >> 8;        // OFF_H
	HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, reg_base, I2C_MEMADD_SIZE_8BIT,
					  pwm_data, 4, HAL_MAX_DELAY);
	return 0;
}

uint8_t PCA9685_LED0_off(hPCA9685 *hpca)
{
	uint8_t data_on = 0x10;
	uint8_t data_off = 0x00;
	HAL_StatusTypeDef i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, PCA9685_LED0_ON_H_REGADDR, 1, &data_off, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, PCA9685_LED0_OFF_H_REGADDR, 1, &data_on, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	return 0x00;
}
uint8_t PCA9685_LED0_on(hPCA9685 *hpca)
{
	uint8_t data_on = 0x10;
	uint8_t data_off = 0x00;
	HAL_StatusTypeDef i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, PCA9685_LED0_ON_H_REGADDR, 1, &data_on, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, PCA9685_LED0_OFF_H_REGADDR, 1, &data_off, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	return 0x00;
}

uint8_t PCA9685_LEDX_off(hPCA9685 *hpca, uint8_t pin)
{
	if(pin >15)
	{
		return 0x01;
	}
	uint8_t data_on = 0x10;
	uint8_t data_off = 0x00;
	uint8_t led_on_h_addr = PCA9685_LED0_ON_H_REGADDR + pin*4;
	uint8_t led_off_h_addr = PCA9685_LED0_OFF_H_REGADDR + pin*4;
	HAL_StatusTypeDef i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, led_on_h_addr, 1, &data_off, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, led_off_h_addr, 1, &data_on, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	return 0x00;
}

uint8_t PCA9685_LEDX_on(hPCA9685 *hpca, uint8_t pin)
{
	if(pin >15)
	{
		return 0x01;
	}
	uint8_t data_on = 0x10;
	uint8_t data_off = 0x00;
	uint8_t led_on_h_addr = PCA9685_LED0_ON_H_REGADDR + pin*4;
	uint8_t led_off_h_addr = PCA9685_LED0_OFF_H_REGADDR + pin*4;
	HAL_StatusTypeDef i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, led_on_h_addr, 1, &data_on, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	i2c_status = HAL_I2C_Mem_Write(hpca->hi2c, hpca->address, led_off_h_addr, 1, &data_off, 1, HAL_MAX_DELAY);
	if(i2c_status != HAL_OK)
	{
		return 0x01;
	}
	return 0x00;
}

uint8_t PCA9685_digital_write(hPCA9685 *hpca, uint8_t pin, uint8_t val)
{
	if(val)
		return PCA9685_LEDX_on(hpca, pin);
	else
		return PCA9685_LEDX_off(hpca, pin);
}
