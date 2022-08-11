/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include <math.h>
#include "MadgwickAHRS.h"
#include "compress.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int16_t head;
	int16_t acc_X;
	int16_t acc_Y;
	int16_t acc_Z;
	int16_t gyro_X;
	int16_t gyro_Y;
	int16_t gyro_Z;
	int16_t mag_X;
	int16_t mag_Z;
	int16_t mag_Y;
} Sensor_Data;

Sensor_Data sensors_data;

typedef struct {
	float cal_acc_X;
	float cal_acc_Y;
	float cal_acc_Z;
	float cal_gyro_X;
	float cal_gyro_Y;
	float cal_gyro_Z;
	float cal_mag_X;
	float cal_mag_Z;
	float cal_mag_Y;
} Calibrated_Data;

Calibrated_Data calibrated_data;

typedef struct {
	uint8_t human_control;
	float roll;
	float pitch;
} IMU;

IMU imu;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI   3.14159265358979323846 /* pi */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
volatile uint8_t data_ready_gyro, data_ready_acc, data_ready_mag, data_ready_all;
extern volatile float q0, q1, q2, q3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t spi1_beriRegister(uint8_t);
void spi1_beriRegistre(uint8_t, uint8_t*, uint8_t);
void spi1_pisiRegister(uint8_t, uint8_t);
void initGyro(void);

uint8_t i2c1_pisiRegister(uint8_t, uint8_t, uint8_t);
void i2c1_beriRegistre(uint8_t, uint8_t, uint8_t*, uint8_t);
void initOrientation(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
float ChangeInterval(float value, float in_min, float in_max, float out_min, float out_max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Pavza, s katero omogocimo pravilno delovanje avtomatskega testa
void pavza(){
	uint32_t counter = 0;
	for(counter=0; counter<100; counter++){
		asm("nop");
	}
}

uint8_t spi1_beriRegister(uint8_t reg) {
	uint16_t buf_out, buf_in;
	reg |= 0x80; // najpomembnejsi bit na 1
	buf_out = reg; // little endian, se postavi na pravo mesto ....
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	pavza();
	//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&buf_out, (uint8_t*)&buf_in, 2, 2); // blocking posiljanje ....
	HAL_SPI_TransmitReceive(&hspi1, &((uint8_t*)&buf_out)[0], &((uint8_t*)&buf_in)[0], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
	pavza();
	HAL_SPI_TransmitReceive(&hspi1, &((uint8_t*)&buf_out)[1], &((uint8_t*)&buf_in)[1], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	pavza();
	return buf_in >> 8; // little endian...
}

void spi1_pisiRegister(uint8_t reg, uint8_t vrednost) {
	uint16_t buf_out;
	buf_out = reg | (vrednost<<8); // little endian, se postavi na pravo mesto ....
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	pavza();
	//HAL_SPI_Transmit(&hspi1, (uint8_t*)&buf_out, 2, 2); // blocking posiljanje ....
	HAL_SPI_Transmit(&hspi1, &((uint8_t*)&buf_out)[0], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
	pavza();
	HAL_SPI_Transmit(&hspi1, &((uint8_t*)&buf_out)[1], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	pavza();
}

void spi1_beriRegistre(uint8_t reg, uint8_t* buffer, uint8_t velikost) {
	reg |= 0xC0; // najpomembnejsa bita na 1
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	pavza();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 10); // blocking posiljanje....
	pavza();
	HAL_SPI_Receive(&hspi1,  buffer, velikost, velikost); // blocking posiljanje....
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	pavza();
}

void initGyro() {
	// preverimo ali smo "poklicali" pravi senzor
	uint8_t cip = spi1_beriRegister(0x0F);
	if (cip!=0xD4 && cip!=0xD3) {
		for (;;);
	}

	// CR1 DR & BW = 00 -> ODR = 95Hz, Cut-off = 12.5
	// CR1 PD & Zen & Xen & Yen = 0F -> enable normal mode and all axis
	spi1_pisiRegister(0x20, 0x0F);// zbudi ziroskop in omogoci osi

	// CR3 I2_DRDY = 3 -> enable interrupt INT2
	spi1_pisiRegister(0x22, 0x8);

	// CR4 FS = 01 (500dps)
	spi1_pisiRegister(0x23, 0x10);

	spi1_beriRegistre(0x28, (uint8_t*)&sensors_data.gyro_X, 6);
}

uint8_t i2c1_pisiRegister(uint8_t naprava, uint8_t reg, uint8_t podatek) {
	naprava <<= 1;
	return HAL_I2C_Mem_Write(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, &podatek, 1, 10);
}

void i2c1_beriRegistre(uint8_t naprava, uint8_t reg, uint8_t* podatek, uint8_t dolzina) {
	if ((dolzina>1)&&(naprava==0x19))  // ce je naprava 0x19 moramo postaviti ta bit, ce zelimo brati vec zlogov
		reg |= 0x80;
	naprava <<= 1;
	HAL_I2C_Mem_Read(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, podatek, dolzina, dolzina);
}

int16_t MSB_swap_LSB(int16_t value) {
	uint16_t svalue = (uint16_t) value;
	uint8_t t1_high = svalue >> 8;
	uint8_t t1_low = svalue & 0xFF;
	uint16_t LSB = (t1_low << 8) | t1_high;
	return (int16_t)LSB;
}

void initOrientation() {
	HAL_Delay(10);

	// Za potrebe testa, moramo testni napravi sporoviti kateri senzor imamo
#define OLD_SENSOR 0x73 // Odkomentiramo za LSM303DLHC / stari senzor
	//#define NEW_SENSOR 0x6E // Odkomentiramo za LSM303AGR / novi senzor

#if defined(OLD_SENSOR) && !defined(NEW_SENSOR)
	i2c1_pisiRegister(0x1e, 0x4F, OLD_SENSOR); // Povemo testni napravi, da imamo stari senzor
#elif !defined(OLD_SENSOR) && defined(NEW_SENSOR)
	i2c1_pisiRegister(0x1e, 0x4F, NEW_SENSOR); // Povemo testni napravi, da imamo novi senzor
#else
	for(;;); // V primeru napake, pocakamo tukaj
#endif
	HAL_Delay(100);


	// inicializiraj pospeskometer
	// odzivnost 100HZ - 0101
	// enable Z Y X - 111
	// 0101 0111 -> 0x57
	i2c1_pisiRegister(0x19, 0x20, 0x57);  // zbudi pospeskometer in omogoci osi
	// block data update - 1
	// LSB/MSB - 0
	// obcutljivost +/- 2g - 00
	// high resolution - 1
	// 1000 1000 -> 0x88
	i2c1_pisiRegister(0x19, 0x23, 0x88);  // nastavi posodobitev samo ko se prebere vrednost ter visoko locljivost
	// interupt INT1 I1_DRDY1 - 1
	i2c1_pisiRegister(0x19, 0x22, 0x10);
	i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&sensors_data.acc_X, 6);

	// inicializiraj magnetometer
	// odzivnost 75HZ - 110
	// 0001 1000 -> 0x18
	i2c1_pisiRegister(0x1E, 0x00, 0x18);
	// obcutljivost +/- 1.3 - 001
	// 0010 0000 -> 0x20
	i2c1_pisiRegister(0x1E, 0x01, 0x20);
	// mode select - continuous-conversion mode - 00
	i2c1_pisiRegister(0x1E, 0x02, 0x0);
	// interupt DRDY
	// 0001 000- -> 0x10
	i2c1_pisiRegister(0x1E, 0x09, 0x1);
	i2c1_beriRegistre(0x1E, 0x03,(uint8_t*)&sensors_data.mag_X, 6);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_1)
		data_ready_gyro = 1;
	else if (GPIO_Pin == GPIO_PIN_4)
		data_ready_acc = 1;
	else if (GPIO_Pin == GPIO_PIN_2)
	{
		data_ready_mag = 1;
		data_ready_gyro = 1;
		data_ready_acc = 1;
	}
}

float ChangeInterval(float value, float in_min, float in_max, float out_min, float out_max)
{
	return (out_max - out_min) * ((value - in_min) / (in_max - in_min)) + out_min;
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	__HAL_SPI_ENABLE(&hspi1);
	__HAL_I2C_ENABLE(&hi2c1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); // CS postavimo na 1

	data_ready_gyro = 0;
	data_ready_acc = 0;
	data_ready_mag = 0;
	data_ready_all = 0;
	sensors_data.head = 0xaaab;

	uint8_t pressed = 0;
	uint8_t hold = 0;
	uint8_t count = 0;

	uint8_t human_control = 1;

	// pristranskost
	float gyro_bias_X = -1.0277027780444998f;
	float gyro_bias_Y = -0.1140749769030433f;
	float gyro_bias_Z = 0.27546970587419833f;

	// offset
	float mag_hard_iron_X = -0.00827521645181321f;
	float mag_hard_iron_Y = 0.0035153249622327603f;
	float mag_hard_iron_Z = 0.002665074831656998f;

	// factor
	float mag_soft_iron_X = 0.016130681156147946f;
	float mag_soft_iron_Y = 0.016845809702668836f;
	float mag_soft_iron_Z = 0.019288506648354117f;


	uint8_t output[100];
	uint8_t len = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	initGyro();
	initOrientation();

	while (1)
	{
		if (data_ready_gyro == 1)
		{
			spi1_beriRegistre(0x28, (uint8_t*)&sensors_data.gyro_X, 6);
			sensors_data.gyro_Y = -sensors_data.gyro_Y;
			data_ready_gyro = 0;
		}
		if (data_ready_acc == 1)
		{
			i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&sensors_data.acc_X, 6);
			sensors_data.acc_X = -sensors_data.acc_X;
			data_ready_acc = 0;
		}
		if (data_ready_mag == 1)
		{
			i2c1_beriRegistre(0x1E, 0x03,(uint8_t*)&sensors_data.mag_X, 6);
			sensors_data.mag_X = -sensors_data.mag_X;
			sensors_data.mag_X = MSB_swap_LSB(sensors_data.mag_X);
			sensors_data.mag_Y = MSB_swap_LSB(sensors_data.mag_Y);
			sensors_data.mag_Z = MSB_swap_LSB(sensors_data.mag_Z);
			data_ready_mag = 0;
			data_ready_all = 1;
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
			pressed = 1;
		else
			pressed = hold = 0;

		if (pressed && !hold)
		{
			hold = 1;
			count = ++count % 2;
		}

		switch (count) {
		case 0:
		{
			if (human_control == 1)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				human_control = 0;
				imu.human_control = human_control;
			}
			memset(output, 0, sizeof output);
			len = sprintf(output, ";%d;%0.3f;%0.3f;\n", imu.human_control, 0.0f, 0.0f);
	        uint8_t *p = compress_data(output, &len);
			CDC_Transmit_FS(p, len);
		}
		break;
		case 1:
		{
			if (human_control == 0)
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				human_control = 1;
				imu.human_control = human_control;
			}
			if (data_ready_all == 1)
			{
				data_ready_all = 0;
				// Napredni ASCII
				// Get calibrated data from ACC
				calibrated_data.cal_acc_X = (float)sensors_data.acc_X / 32767 * 2;
				calibrated_data.cal_acc_Y = (float)sensors_data.acc_Y / 32767 * 2;
				calibrated_data.cal_acc_Z = (float)sensors_data.acc_Z / 32767 * 2;

				// Get calibrated data from GYRO
				calibrated_data.cal_gyro_X = ((float)sensors_data.gyro_X / 32767 * 500) - gyro_bias_X;
				calibrated_data.cal_gyro_Y = ((float)sensors_data.gyro_Y / 32767 * 500) - gyro_bias_Y;
				calibrated_data.cal_gyro_Z = ((float)sensors_data.gyro_Z / 32767 * 500) - gyro_bias_Z;

				// Get calibrated data from MAG
				calibrated_data.cal_mag_X = (((float)sensors_data.mag_X / 32767 * 1.3) + mag_hard_iron_X) / mag_soft_iron_X;
				calibrated_data.cal_mag_Y = (((float)sensors_data.mag_Y / 32767 * 1.3) + mag_hard_iron_Y) / mag_soft_iron_Y;
				calibrated_data.cal_mag_Z = (((float)sensors_data.mag_Z / 32767 * 1.3) + mag_hard_iron_Z) / mag_soft_iron_Z;

				// Change GYRO from DEG/s -> RAD/s
				calibrated_data.cal_gyro_X = calibrated_data.cal_gyro_X * M_PI / 180;
				calibrated_data.cal_gyro_Y = calibrated_data.cal_gyro_Y * M_PI / 180;
				calibrated_data.cal_gyro_Z = calibrated_data.cal_gyro_Z * M_PI / 180;

				// Normalize MAG from [-1.3, 1.3] to [-1, 1]
				calibrated_data.cal_mag_X = ChangeInterval(calibrated_data.cal_mag_X, -1.3, 1.3, -1, 1);
				calibrated_data.cal_mag_Y = ChangeInterval(calibrated_data.cal_mag_Y, -1.3, 1.3, -1, 1);
				calibrated_data.cal_mag_Z = ChangeInterval(calibrated_data.cal_mag_Z, -1.3, 1.3, -1, 1);

				MadgwickAHRSupdate(
						-calibrated_data.cal_gyro_X, -calibrated_data.cal_gyro_Y, -calibrated_data.cal_gyro_Z,
						calibrated_data.cal_acc_X, calibrated_data.cal_acc_Y, calibrated_data.cal_acc_Z,
						calibrated_data.cal_mag_X, calibrated_data.cal_mag_Y, calibrated_data.cal_mag_Z
				);

				imu.roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 180 / M_PI;
				imu.pitch = asin(2 * (q0 * q2 - q3 * q1)) * 180 / M_PI;

				// OUTPUT: HUMAN_CONTROL;STEERING;THROTTLE
				memset(output, 0, sizeof output);
				len = sprintf(output, ";%d;%0.3f;%0.3f;\n", imu.human_control, imu.roll, imu.pitch);
		        uint8_t *p = compress_data(output, &len);
				CDC_Transmit_FS(p, len);
			}
		}
		break;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT2_Pin */
  GPIO_InitStruct.Pin = INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
