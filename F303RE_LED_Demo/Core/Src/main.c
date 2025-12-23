/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PCA9685.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCA9685_MODE1         0x00
#define PCA9685_ALL_LED_ON_L  0xFA
#define PCA9685_ALL_LED_ON_H  0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD

#define LED0_ON_H 0x07
#define LED0_OFF_H 0x09

#define SPI2_BUFFER_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scan(I2C_HandleTypeDef *hi2c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t SPI_data_out;
uint8_t SPI_data_in;
uint8_t SPI_RX_buf[SPI2_BUFFER_SIZE];
uint8_t SPI_RX_ptr = 0;
uint8_t SPI_RX_msg_len = 0;
uint8_t SPI_RX_msg_ready = 0;
uint8_t I2C_TX_Buffer[];
uint8_t I2C_PWM_chip_address = (0x40<<1);
uint8_t data_in = 0;

HAL_StatusTypeDef i2c_status;

hPCA9685 hpca;
LED_ConfigTypeDef LED_config;

// we consider max 5 RGB LEDs and max 15 LEDs overall
float LED_Hue[5] = {0,0,0,0,0};
float LED_Saturation[5] = {0,0,0,0,0};
uint16_t LED_Intensity[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

LED_RGBTypeDef RGB_from_ESP;
LED_HSVTypeDef HSV_from_RGB_from_ESP;
LED_RGBTypeDef RGB_from_HSV_from_RGB_from_ESP;

// HMI
#define NUM_BUTTONS 3
#define DEBOUNCE_SAMPLES 10
//#define BTN_SAMPLE_PERIOD_MS 10

GPIO_TypeDef* button_ports[NUM_BUTTONS] = {
		Button_1_GPIO_Port,
		Button_2_GPIO_Port,
		Button_3_GPIO_Port
};
uint16_t button_pins[NUM_BUTTONS] = {
		Button_1_Pin,
		Button_2_Pin,
		Button_3_Pin
};

// selected LED id - used in some button bindings
uint8_t selected_LED_id = 0;

/// button mode settings
/// 1 - toggle all LEDs
/// 2 - toggle LED {ARG}
/// 3 - toggle selected LED
/// 4 - select next LED
/// 5 - select prev LED
/// 6 - switch all LEDs mode
/// 7 - switch LED {ARG} mode
/// 8 - switch selected LED mode

uint8_t button_action[NUM_BUTTONS] = {
		3,
		4,
		5
};

/// arguments for button_action
uint8_t button_action_ARG[NUM_BUTTONS] = {
		0, 0, 0
};

GPIO_PinState btn_state[NUM_BUTTONS];  // stable state
uint8_t btn_counter[NUM_BUTTONS]; // debounce counter

uint8_t start_sequence_flag = 0;

LED_HSVTypeDef RGB_8bit_to_HSV(uint8_t r, uint8_t g, uint8_t b)
{
	LED_HSVTypeDef hsv;
	hsv.h = hsv.s = hsv.v = -0.1f;
	uint8_t max_rgb = r;
	uint8_t which_max = 1; // 1 - RED, 2 - GREEN, 3 - BLUE
	if(r >= g && r >= b)
	{
		max_rgb = r;
		which_max = 1;
	}
	if(g >= b && g >= r)
	{
		max_rgb = g;
		which_max = 2;
	}
	if(b >= r && b >= g)
	{
		max_rgb = b;
		which_max = 3;
	}

	uint8_t min_rgb = 0;
	if(r <= g && r <= b)
		min_rgb = r;
	if(g <= r && g <= b)
		min_rgb = g;
	if(b <= r && b <= g)
		min_rgb = b;

	// due to using 8 bits
	float v = max_rgb/255.0f;
	float s = ((float)(max_rgb-min_rgb))/max_rgb;
	float h = 0.0;

	uint8_t delta = max_rgb - min_rgb;

	if(delta != 0)
		switch (which_max)
		{
			case 1:
				// max is Red
				h = 60*(0+ ((float)(g-b))/delta);
				break;

			case 2:
				//max is Green
				h = 60*(2+ ((float)(b-r))/delta);
				break;

			case 3:
				//max is Blue
				h = 60*(4+ ((float)(r-g))/delta);
				break;

			default:
				h = 0.0;
				break;
		}
	else
		h = 0;
	while(h < 0)
		h += 360.0f;
	while(h > 360.0f)
		h -= 360.0f;
	hsv.h = h;
	hsv.s = s;
	hsv.v = v;
	return hsv;
}

LED_RGBTypeDef HSV_to_RGB_12bit(float h, float s, float v)
{
	LED_RGBTypeDef rgb;
	float c = v*s;
	h /= 60;
	float h_mod_2 = h;
	while (h_mod_2 > 2)
		h_mod_2 -= 2;
	h_mod_2 -= 1;
	if(h_mod_2 < 0)
		h_mod_2 *= -1.0f;
	float x = c*(1-h_mod_2);

	float r,g,b;
	if(h >= 0 && h < 1)
	{
		r = c;	g = x;	b = 0;
	}
	else if(h >= 1 && h < 2)
	{
		r = x;	g = c;	b = 0;
	}
	else if(h >= 2 && h < 3)
	{
		r = 0;	g = c;	b = x;
	}
	else if(h >= 3 && h < 4)
	{
		r = 0;	g = x;	b = c;
	}
	else if(h >= 4 && h < 5)
	{
		r = x;	g = 0;	b = c;
	}
	else if(h >= 5 && h < 6)
	{
		r = c;	g = 0;	b = x;
	}
	else
	{
		r = 0; g = 0; b = 0;
	}
	float m = v - c;
	rgb.r = (uint16_t)((r + m)*4095);
	rgb.g = (uint16_t)((g + m)*4095);
	rgb.b = (uint16_t)((b + m)*4095);

	return rgb;
}

void LED_Reset_All()
{
	for (uint8_t i = 0; i < 16; ++i)
	{
		LED_Intensity[i] = 0;
		PCA9685_LEDX_off(&hpca, i);
	}
	for (uint8_t i = 0; i < 5; i++)
	{
		LED_Hue[i] = 0;
		LED_Saturation[i] = 0;
	}
}

void LED_Set_All()
{
	uint8_t led_count = LED_config.Greyscale_LED_Count + LED_config.RGB_LED_Count;
	for (uint8_t i = 0; i < led_count; ++i)
		Set_LED_Intensity(i, 4095);
}

void LED_Toggle_All()
{
	// loop through all LEDs, if at least one is on, turn all off, if all are off, turn each on
	int num_leds = LED_config.Greyscale_LED_Count + LED_config.RGB_LED_Count;
	for (int i = 0; i < num_leds; ++i)
	{
		if(LED_Intensity[i] > 0)
		{
			LED_Reset_All();
			return;
		}
	}
	LED_Set_All();
}

void LED_Toggle(uint8_t id)
{
	if(LED_Intensity[id] == 0)
		Set_LED_Intensity(id, 4095);
	else
		Set_LED_Intensity(id, 0);
}

void Set_LED_Config(uint8_t rgb_count, uint8_t greyscale_count)
{
	// input verification and adjustment
	if(rgb_count > 5)
		rgb_count = 5;
	if(greyscale_count > 16)
		greyscale_count = 16;
	if(3*rgb_count  + greyscale_count > 16)
		greyscale_count = 16 - 3*rgb_count;

	// changing config might make some LEDs unadressable - we want those to be off by default.
	LED_Reset_All();

	LED_config.Greyscale_LED_Count = greyscale_count;
	LED_config.RGB_LED_Count = rgb_count;

	// loop through all RGB LEDs and assign their first pin
	for (uint8_t i = 0; i < rgb_count; ++i)
		LED_config.LED_Pins[i] = i*3;

	// loop through all Greyscale LEDs and assign their pin
	// first Greyscale ID will start after the last RGB LED and end after `greyscale_count` increments
	for (int i = 0; i < greyscale_count; ++i)
		LED_config.LED_Pins[rgb_count+i] = rgb_count*3+i;

	for (uint8_t i = rgb_count+greyscale_count; i <= 15; ++i)
		LED_config.LED_Pins[i] = -1;
}

LED_RGBTypeDef Get_LED_RGB(uint8_t id)
{
	// read Hues, Saturations and Intensity values, convert them to 12 byte RGB values for export
	LED_RGBTypeDef out;
	out.r =out.g = out.b = 0xffff;

	if(id >= LED_config.RGB_LED_Count)
		return out;

	out = HSV_to_RGB_12bit(LED_Hue[id], LED_Saturation[id], LED_Intensity[id]/4095.0);
	return out;
}

void Set_LED_Intensity(uint8_t id, uint16_t intensity)
{
	LED_Intensity[id] = intensity;
	if(LED_config.LED_Pins[id] == -1)
		return;
	if(id >= LED_config.RGB_LED_Count)
	{
		// greyscale LED
		PCA9685_PWM_write(&hpca, LED_config.LED_Pins[id], intensity);
		return;
	}
	// RGB LED
	LED_RGBTypeDef rgb = Get_LED_RGB(id);
	PCA9685_RGB_write(&hpca, LED_config.LED_Pins[id], rgb.r, rgb.g, rgb.b);
}

void Set_LED_Color(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
	if(id > 5)
		return;

	// Convert and set to HSV values
	LED_HSVTypeDef hsv = RGB_8bit_to_HSV(r, g, b);
	LED_Hue[id] = hsv.h;
	LED_Saturation[id] = hsv.s;
	LED_Intensity[id] = (uint16_t)(hsv.v*4095);

	// Write to pins
	PCA9685_RGB_write(&hpca, LED_config.LED_Pins[id], ((uint16_t)r)*4, ((uint16_t)g)*4, ((uint16_t)b)*4);
}

void ESP_SPI_Handle_Message(uint8_t* data, uint16_t len)
{
	// determine message code
	uint8_t header = data[0] & 0b11100000;
	switch (header) {
		case 0b00000000:
		{
			// LED on/off message
			uint8_t LED_id = data[0] & 0b00001111;
			uint8_t state = (data[0] & 0b00010000) >> 4;
			Set_LED_Intensity(LED_id, state? 4095:0);
			break;
		}
		case 0b00100000:
		{
			// LED intensity message
			// need 2 more bytes
//			HAL_StatusTypeDef SPI_status = HAL_SPI_Receive(&hspi2, data, 2, 200);
//			if(SPI_status != HAL_OK)
//				return;
			uint8_t LED_id = data[0] & 0b00001111;
			uint16_t PWM_val = (data[1] << 4) | (data[2] >> 4);
			Set_LED_Intensity(LED_id, PWM_val);
			break;
		}
		case 0b01000000:
		{
			// LED Color message
			// need 3 more bytes
//			uint8_t data[3];
//			HAL_StatusTypeDef SPI_status = HAL_SPI_Receive(&hspi2, data, 3, 200);
			uint8_t LED_id = data[0] & 0b00000111;
			uint8_t r = data[1];
			uint8_t g = data[2];
			uint8_t b = data[3];

			Set_LED_Color(LED_id, r, g, b);
			break;
		}
		case 0b01100000:
		{
			// LED configuration message
			// need 1 more byte
//			uint8_t data;
//			HAL_StatusTypeDef SPI_status = HAL_SPI_Receive(&hspi2, &data, 1, 200);
//			if(SPI_status != HAL_OK)
//				return;
			uint8_t RGB_count = (data[1] & 0b11100000)>>5;
			uint8_t Greyscale_count = (data[1] & 0b00011111);
			Set_LED_Config(RGB_count, Greyscale_count);
			// perform start sequence by setting the flag
			start_sequence_flag = 1;
			break;
		}
		default:
			break;
	}
	// indicate received message
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void HMI_Select_Next()
{
	if(++selected_LED_id >= LED_config.Greyscale_LED_Count + LED_config.RGB_LED_Count)
		selected_LED_id = 0;
}

void HMI_Select_Prev()
{
	uint8_t led_count = LED_config.Greyscale_LED_Count + LED_config.RGB_LED_Count;
	if(selected_LED_id-- == 0)
		selected_LED_id = led_count-1;
}

void On_Button_Pressed(uint8_t id)
{
	/// button mode settings
	/// 1 - toggle all LEDs
	/// 2 - toggle LED {ARG}
	/// 3 - toggle selected LED
	/// 4 - select next LED
	/// 5 - select prev LED
	/// 6 - switch all LEDs mode
	/// 7 - switch LED {ARG} mode
	/// 8 - switch selected LED mode
	switch (button_action[id]) {
		case 1:
			LED_Toggle_All();
			break;
		case 2:
			// toggle LED specified in button_action_ARG
			LED_Toggle(button_action_ARG[id]);
			break;
		case 3:
			LED_Toggle(selected_LED_id);
			break;
		case 4:
			HMI_Select_Next();
			break;
		case 5:
			HMI_Select_Prev();
			break;
		default:
			LED_Toggle_All();
			break;
	}
}

void On_Button_Changed(uint8_t id, GPIO_PinState state)
{
	if(state == GPIO_PIN_RESET)
		On_Button_Pressed(id);
}

void Handle_HMI()
{
	for(uint8_t i = 0; i < NUM_BUTTONS; i++)
	{
		// read inputs
		GPIO_PinState btn_i = HAL_GPIO_ReadPin(button_ports[i], button_pins[i]);

		// look for change
		if(btn_i == btn_state[i])
			btn_counter[i] = 0;
		else if(++btn_counter[i] >= DEBOUNCE_SAMPLES)
		{
			btn_state[i] = btn_i;
			btn_counter[i] = 0;

			// button is pressed, perform action:
			On_Button_Changed(i, btn_i);
		}
	}
}

void Start_Sequence()
{
	// flash all individual LED pins

	// first flash all RGB pins individually
	for (int i = 0; i < LED_config.RGB_LED_Count; ++i)
	{
		uint8_t rgb_pin_0 = LED_config.LED_Pins[i];
		for (int rgb_pin = 0; rgb_pin < 3; ++rgb_pin)
		{
			PCA9685_LEDX_on(&hpca, rgb_pin);
			HAL_Delay(300);
			PCA9685_LEDX_off(&hpca, rgb_pin);
		}
	}
	// then flash all greyscale LEDs
	for (int i = 0; i < LED_config.Greyscale_LED_Count; i++)
	{
		uint8_t grey_pin = LED_config.LED_Pins[LED_config.RGB_LED_Count+i];
		PCA9685_LEDX_on(&hpca, grey_pin);
		HAL_Delay(300);
		PCA9685_LEDX_off(&hpca, grey_pin);
	}

	// send SPI message to indicate finished start sequence
	SPI_data_out = 0b1;
	HAL_StatusTypeDef status = HAL_SPI_Transmit_IT(&hspi2, &SPI_data_out, 1);
//	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, &SPI_data_out, 1, 1000);
	if(status)
		return;
	return;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == SPI2_NSS_Pin)
//    {
//    	if (HAL_GPIO_ReadPin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin) == GPIO_PIN_RESET) {
//			// Prepare SPI/DMA to receive exactly from index 0
//			HAL_SPI_DMAStop(&hspi2);
//			HAL_SPI_Receive_DMA(&hspi2, SPI_RX_buf, SPI2_BUFFER_SIZE);
//		}
//		// RISING EDGE: Master finished the message
//		else {
//			HAL_SPI_DMAStop(&hspi2);
//			// Calculate how many bytes were actually transferred
//			uint16_t count = SPI2_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hspi2.hdmarx);
////			if (count > 0) {
//			ESP_SPI_Handle_Message(SPI_RX_buf, count);
////			}
//		}
//    }
//}

uint8_t Get_Message_Length(uint8_t header)
{
	switch (header)
	{
			case 0b00000000:
			{
				return 1;
			}
			case 0b00100000:
			{
				return 3;
			}
			case 0b01000000:
			{
				return 4;
			}
			case 0b01100000:
			{
				return 2;
			}
			default:
				return 0;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
	// find out if this is the first byte of the message
	// if it is, determine the length of incoming bytes
	if(SPI_RX_ptr == 0)
	{
		uint8_t header = SPI_data_in & 0b11100000;
		SPI_RX_msg_len = Get_Message_Length(header);
	}
	SPI_RX_buf[SPI_RX_ptr++] = SPI_data_in;
	if(SPI_RX_ptr >= SPI_RX_msg_len)
	{
		// message is complete and ready to parse
		// reset buffer pointer
		SPI_RX_ptr = 0;
		SPI_RX_msg_ready = 1;
	}
	HAL_SPI_Receive_IT(hspi, &SPI_data_in, 1);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

//  HAL_SPI_Receive_DMA(&hspi2, SPI_RX_buf, SPI2_BUFFER_SIZE);
  HAL_SPI_Receive_IT(&hspi2, &SPI_data_in, 1);
//  I2C_Scan(&hi2c1);
//
//  uint8_t mode1 = 0x01;  // Normal mode, all-call enabled by default
//  i2c_status = HAL_I2C_Mem_Write(&hi2c1, I2C_PWM_chip_address, PCA9685_MODE1, 1, &mode1, 1, HAL_MAX_DELAY);
//  HAL_Delay(10);
//
//  i2c_status = HAL_I2C_Mem_Read(&hi2c1, I2C_PWM_chip_address, 0x00, I2C_MEMADD_SIZE_8BIT, &data_in, 1, HAL_MAX_DELAY);
//
//  // read ALLCALLADR register
//  i2c_status = HAL_I2C_Mem_Read(&hi2c1, I2C_PWM_chip_address, 0x05, I2C_MEMADD_SIZE_8BIT, &data_in, 1, HAL_MAX_DELAY);
//
//  // read SUBADR1 register
//  i2c_status = HAL_I2C_Mem_Read(&hi2c1, I2C_PWM_chip_address, 0x02, I2C_MEMADD_SIZE_8BIT, &data_in, 1, HAL_MAX_DELAY);

  uint8_t pca_status = PCA9685_PWM_init(&hpca, &hi2c1, 0x40);
  Set_LED_Config(1, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	GPIO_PinState pinState = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, pinState);


	// I2C turn LED0 ON
//	pca_status = PCA9685_PWM_write(&hpca, 0, 2024);
//	pca_status = PCA9685_LED0_on(&hpca);
//    pca_status = PCA9685_LEDX_on(&hpca, 15);
//	uint8_t data_on = 0x10;   // bit 4 = full ON
//	uint8_t data_off = 0x00; //set bit 4 to 0;
//	i2c_status = HAL_I2C_Mem_Write(&hi2c1, I2C_PWM_chip_address, LED0_ON_H, 1, &data_on, 1, HAL_MAX_DELAY);
//	i2c_status = HAL_I2C_Mem_Write(&hi2c1, I2C_PWM_chip_address, LED0_OFF_H, 1, &data_off, 1, HAL_MAX_DELAY);

	// SPI
//	HAL_SPI_Receive_IT(&hspi2, &SPI_data_in, 1);

	if(start_sequence_flag)
	{
		Start_Sequence();
		start_sequence_flag = 0;
	}

	if(SPI_RX_msg_ready)
	{
		ESP_SPI_Handle_Message(SPI_RX_buf, SPI_RX_msg_len);
		SPI_RX_msg_ready = 0;
	}

	Handle_HMI();
//	if(SPI_data_in == 0b0001111)
//	    pca_status = PCA9685_LEDX_on(&hpca, 15);
//	else if(SPI_data_in == 0b11110000)
//	    pca_status = PCA9685_LEDX_off(&hpca, 15);

//	pca_status = PCA9685_LED0_on(&hpca);
//	HAL_Delay(500);
//	pca_status = PCA9685_LED0_off(&hpca);
//	HAL_Delay(500);

//	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi3, &SPI_data_out, 1, 1000);
//	if(status == HAL_OK)
//	{
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	}
//	HAL_Delay(500);

	// I2C turn LED0 OFF
//	pca_status = PCA9685_PWM_write(&hpca, 0, 4095);
//	pca_status = PCA9685_LED0_off(&hpca);
//  pca_status = PCA9685_LEDX_off(&hpca, 15);
//	i2c_status = HAL_I2C_Mem_Write(&hi2c1, I2C_PWM_chip_address, LED0_ON_H, 1, &data_off, 1, HAL_MAX_DELAY);
//	i2c_status = HAL_I2C_Mem_Write(&hi2c1, I2C_PWM_chip_address, LED0_OFF_H, 1, &data_on, 1, HAL_MAX_DELAY);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_3_Pin Button_2_Pin */
  GPIO_InitStruct.Pin = Button_3_Pin|Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    printf("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 10) == HAL_OK)
        {
            printf("  Found device at 0x%02X\r\n", addr);
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
