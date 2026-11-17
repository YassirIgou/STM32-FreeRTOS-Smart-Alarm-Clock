/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "SGP40.h"
#include "../../Drivers/VL53L1_Nucleo/core/inc/vl53l1_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NB_LED				4
#define POT_SEUIL_MANUEL	50

#define DUREE_AUBE_MS       15000U
#define DUREE_CREPUSCULE_MS 15000U
#define DUREE_ALARME_MS     15000U

#define TOUCH_THRESHOLD		430
#define TOUCH_LONG_PRESS_MS 1500U

#define MAX_NOTES 			32
#define NB_SONNERIES 		3
#define SD_EST_PRESENTE()  	(HAL_GPIO_ReadPin(DETECT_GPIO_Port, DETECT_Pin) == GPIO_PIN_RESET)

#define DUREE_SNOOZE_MS       10000U

#define TOF_SEUIL_MAIN_MM     250U
#define TOF_FENETRE_GESTE_MS  600U
#define TOF_COOLDOWN_MS       500U
#define TOF_PERIODE_MS        10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim3_ch4_up;

TSC_HandleTypeDef htsc;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

// Horloge & Alarme

typedef struct
{
    uint8_t heure;
    uint8_t min;
    uint8_t sec;
} temps;

temps horloge = {0, 0, 0};
temps alarme  = {0, 0, 0};

volatile uint8_t alarmeActive = 0;
volatile uint8_t alarmeEnCours = 0;

// Affichage OLED

/*
 	 0 = Heure
 	 1 = Alarme
 	 2 = Température
 	 3 = Qualité de l'air
 */
uint8_t ecran = 0;

volatile uint8_t oledColor = 1;

// Bluetooth

uint8_t bt_char;
char bt_buffer[30];
uint8_t bt_index = 0;

// Seuils

volatile int16_t  seuilTemperature = 3000;
volatile uint16_t seuilVOC = 500;
volatile uint16_t seuilLDR = 1500;

// Température & VOC

uint8_t control_T[2] = {0x80, 0x15};
uint8_t Temp_Address = 0x02;
uint8_t Temp[3];

int16_t temperature = 0;

uint16_t voc = 0;

// ADC : Potentiomètre et LDR

volatile uint16_t adcPot = 0;
volatile uint16_t adcLdr = 0;

// Aube & Crépuscule

volatile uint8_t aubeActive = 0;
volatile uint8_t crepusculeActif = 0;

TickType_t debutAube = 0;
TickType_t debutCrepuscule = 0;

// LED RGB

uint16_t PWM_LED[NB_LED * 24 + 50];

// Buzzer & Sonnerie

volatile uint8_t demandeMelodie = 0;

typedef struct
{
    uint16_t frequence;
    uint16_t duree_ms;
    uint16_t pause_ms;
} Note;

const Note melodieDefaut[] =
{
    {1000, 300, 150},
    {1000, 300, 150},
    {1000, 300, 600},
};

const Note melodieDouce[] =
{
    {392, 500, 150},
    {440, 500, 150},
    {392, 700, 250},

    {330, 500, 150},
    {349, 500, 150},
    {330, 700, 300},

    {294, 600, 150},
    {330, 600, 150},
    {262, 900, 500}
};

typedef struct
{
    Note notes[MAX_NOTES];
    uint8_t taille;
} Sonnerie;

Sonnerie sonneries[NB_SONNERIES];

volatile uint8_t sonnerieSelectionnee = 0;

volatile uint8_t demandePreview = 0;
volatile uint8_t previewSonnerie = 0;
volatile uint8_t audioPreviewVersion = 0;

// ToF gauche et droit

VL53L1_Dev_t devLeft;
VL53L1_Dev_t devRight;

VL53L1_RangingMeasurementData_t RangingDataLeft;
VL53L1_RangingMeasurementData_t RangingDataRight;

volatile uint8_t tofLeftReady = 0;
volatile uint8_t tofRightReady = 0;

volatile uint32_t tofLeftIrqTick = 0;
volatile uint32_t tofRightIrqTick = 0;

// Gestion STOP / SNOOZE

volatile uint8_t demandeStop = 0;
volatile uint8_t demandeSnooze = 0;

volatile uint8_t snoozeActif = 0;
TickType_t debutSnooze = 0;

// FreeRTOS

SemaphoreHandle_t semaphBT;
SemaphoreHandle_t semaphSW1;

SemaphoreHandle_t mutexI2C;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TSC_Init(void);
static void MX_SDMMC1_SD_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

void TaskHorloge(void *pvParameters);
void TaskAlarme(void *pvParameters);

void TaskAffichage(void *pvParameters);
void TaskBluetooth(void *pvParameters);

void TaskCapteurs(void *pvParameters);

void TaskVentilo(void *pvParameters);
void Ventilo_Set(uint8_t direction, uint32_t vitesse);

void TaskLED(void *pvParameters);
void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue);

void TaskAudio(void *pvParameters);
void Buzzer_SetFrequency(uint16_t frequence);

void TaskSD(void *pvParameters);
uint8_t SD_ChargerSonnerie(const char *nom, Sonnerie *dest);
uint8_t SD_ChargerSonneries(void);

void TaskTOF(void *pvParameters);
void ResetAndInitializeTOF1(void);
void ResetAndInitializeTOF2(void);

void TaskInteraction(void *pvParameters);
uint16_t TSC_ReadChannel(uint32_t channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Ventilo_Set(uint8_t direction, uint32_t vitesse)
{
    static uint8_t dirPrec = 0xFF;

    if (vitesse > 80000) vitesse = 80000;

    if (vitesse == 0)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_SET);
        dirPrec = 0xFF;
        return;
    }

    if (direction != dirPrec)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
        dirPrec = direction;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, vitesse);
    HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t Couleur[12] = {
        green, red, blue,
        green, red, blue,
        green, red, blue,
        green, red, blue
    };

    uint16_t k = 0;

    for (int i = 0; i < 12; i++)
    {
        for (int j = 7; j >= 0; j--)
        {
            PWM_LED[k++] = (Couleur[i] & (1u << j)) ? 64 : 32;
        }
    }

    for (int i = 0; i < 50; i++)
    {
        PWM_LED[k++] = 0;
    }

    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)PWM_LED, k);
}

void Buzzer_SetFrequency(uint16_t frequence)
{
    if (frequence == 0)
    {
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
        return;
    }

    uint32_t periode = 1000000U / frequence;

    __HAL_TIM_SET_AUTORELOAD(&htim8, periode - 1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, periode / 2);

    __HAL_TIM_SET_COUNTER(&htim8, 0);
}

uint8_t SD_ChargerSonnerie(const char *nom, Sonnerie *dest)
{
    FIL fichier;
    char ligne[40];
    unsigned long fr, du, pa;

    dest->taille = 0;

    if (f_open(&fichier, nom, FA_READ) != FR_OK)
        return 0;

    while (dest->taille < MAX_NOTES && f_gets(ligne, sizeof(ligne), &fichier))
    {
        if (sscanf(ligne, "%lu,%lu,%lu", &fr, &du, &pa) == 3 && fr <= 20000 && du <= 65535 && pa <= 65535)
        {
            dest->notes[dest->taille++] = (Note){ fr, du, pa };
        }
    }

    f_close(&fichier);
    return dest->taille > 0;
}

uint8_t SD_ChargerSonneries(void)
{
    Sonnerie tmp;
    const char *noms[NB_SONNERIES] = { "SON1.txt", "SON2.txt", "SON3.txt" };
    uint8_t nb = 0;

    if (f_mount(&SDFatFS, SDPath, 1) != FR_OK)
    {
        f_mount(NULL, SDPath, 0);
        return 0;
    }

    for (uint8_t i = 0; i < NB_SONNERIES && SD_EST_PRESENTE(); i++)
    {
        if (SD_ChargerSonnerie(noms[i], &tmp))
        {
            taskENTER_CRITICAL();
            sonneries[i] = tmp;
            taskEXIT_CRITICAL();
            nb++;
        }
    }

    f_mount(NULL, SDPath, 0);
    return nb == NB_SONNERIES;
}

uint16_t TSC_ReadChannel(uint32_t channel)
{
    TSC_IOConfigTypeDef ioConfig = {0};

    ioConfig.ChannelIOs = channel;
    ioConfig.ShieldIOs = 0;
    ioConfig.SamplingIOs = TSC_GROUP1_IO4;

    HAL_TSC_IODischarge(&htsc, ENABLE);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_TSC_IODischarge(&htsc, DISABLE);

    if (HAL_TSC_IOConfig(&htsc, &ioConfig) != HAL_OK)
        return 8191;

    if (HAL_TSC_Start(&htsc) != HAL_OK)
        return 8191;

    while (HAL_TSC_GetState(&htsc) == HAL_TSC_STATE_BUSY)
    {
        taskYIELD();
    }

    if (HAL_TSC_GetState(&htsc) != HAL_TSC_STATE_READY)
        return 8191;

    return HAL_TSC_GroupGetValue(&htsc, TSC_GROUP1_IDX);
}

static void TOF_Config(VL53L1_DEV dev)
{
    VL53L1_DetectionConfig_t config = {0};
    VL53L1_UserRoi_t roi = {0};

    VL53L1_WaitDeviceBooted(dev);
    VL53L1_DataInit(dev);
    VL53L1_StaticInit(dev);

    VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_LONG);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 40000);
    VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 50);

    config.DetectionMode = VL53L1_DETECTION_DISTANCE_ONLY;
    config.IntrNoTarget = 1;
    config.Distance.CrossMode = VL53L1_THRESHOLD_IN_WINDOW;
    config.Distance.Low = 0;
    config.Distance.High = 3000;

    VL53L1_SetThresholdConfig(dev, &config);

    roi.TopLeftX = 0;
    roi.TopLeftY = 15;
    roi.BotRightX = 15;
    roi.BotRightY = 0;

    VL53L1_SetUserROI(dev, &roi);
    VL53L1_ClearInterruptAndStartMeasurement(dev);
}

void ResetAndInitializeTOF1(void)
{
    HAL_GPIO_WritePin(SHUT1_GPIO_Port, SHUT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHUT2_GPIO_Port, SHUT2_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    devLeft.comms_speed_khz = 400;
    devLeft.I2cHandle = &hi2c1;
    devLeft.comms_type = 1;
    devLeft.I2cDevAddr = 0x52;

    HAL_GPIO_WritePin(SHUT1_GPIO_Port, SHUT1_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    VL53L1_SetDeviceAddress(&devLeft, 0x54);
    devLeft.I2cDevAddr = 0x54;

    TOF_Config(&devLeft);
}

void ResetAndInitializeTOF2(void)
{
    devRight.comms_speed_khz = 400;
    devRight.I2cHandle = &hi2c1;
    devRight.comms_type = 1;
    devRight.I2cDevAddr = 0x52;

    HAL_GPIO_WritePin(SHUT2_GPIO_Port, SHUT2_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    TOF_Config(&devRight);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TSC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Init();

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  HAL_GPIO_WritePin(DIS_GPIO_Port, DIS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  HAL_UART_Receive_IT(&hlpuart1, &bt_char, 1);

  ResetAndInitializeTOF1();
  ResetAndInitializeTOF2();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  mutexI2C = xSemaphoreCreateMutex();

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  semaphBT = xSemaphoreCreateBinary();
  semaphSW1 = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  xTaskCreate(TaskHorloge, "Horloge",
              256, NULL, tskIDLE_PRIORITY + 3, NULL);

  xTaskCreate(TaskAlarme, "Alarme",
              256, NULL, tskIDLE_PRIORITY + 3, NULL);

  xTaskCreate(TaskAffichage, "Affichage",
              512, NULL, tskIDLE_PRIORITY + 1, NULL);

  xTaskCreate(TaskBluetooth, "Bluetooth",
              512, NULL, tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(TaskCapteurs, "Capteurs",
              512, NULL, tskIDLE_PRIORITY + 1, NULL);

  xTaskCreate(TaskVentilo, "Ventilo",
              256, NULL, tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(TaskLED, "LED",
              256, NULL, tskIDLE_PRIORITY + 1, NULL);

  xTaskCreate(TaskAudio, "Audio",
              384, NULL, tskIDLE_PRIORITY + 3, NULL);

  xTaskCreate(TaskTOF, "TOF",
              512, NULL, tskIDLE_PRIORITY + 2, NULL);

  xTaskCreate(TaskSD, "SD",
              512, NULL, tskIDLE_PRIORITY + 1, NULL);

  xTaskCreate(TaskInteraction, "Interaction",
              512, NULL, tskIDLE_PRIORITY + 2, NULL);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00F12981;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 80-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 500-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO1|TSC_GROUP1_IO2|TSC_GROUP1_IO3;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, SHUT1_Pin|SHUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED5_Pin|DIS_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin INTOF2_Pin INTOF1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|INTOF2_Pin|INTOF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUT1_Pin SHUT2_Pin */
  GPIO_InitStruct.Pin = SHUT1_Pin|SHUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin DIR_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin DIS_Pin CE_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|DIS_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DETECT_Pin */
  GPIO_InitStruct.Pin = DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void TaskHorloge(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
        horloge.sec++;

        if (horloge.sec >= 60)
        {
            horloge.sec = 0;
            horloge.min++;

            if (horloge.min >= 60)
            {
                horloge.min = 0;
                horloge.heure++;

                if (horloge.heure >= 24)
                {
                    horloge.heure = 0;
                }
            }
        }

    }
}

void TaskAlarme(void *pvParameters)
{
    uint8_t declenchee = 0;
    TickType_t debutAlarme = 0;

    while (1)
    {
        TickType_t maintenant = xTaskGetTickCount();

        uint8_t heureAtteinte = (horloge.heure == alarme.heure) && (horloge.min == alarme.min) && (horloge.sec == alarme.sec);

        if (!heureAtteinte)
        {
            declenchee = 0;
        }

        if (!alarmeActive)
        {
            alarmeEnCours = 0;
            snoozeActif = 0;
            demandeStop = 0;
            demandeSnooze = 0;
        }

        if (heureAtteinte && !declenchee && alarmeActive && !snoozeActif)
        {
            declenchee = 1;

            if (!crepusculeActif)
            {
                debutAlarme = maintenant;
                debutAube = maintenant;

                alarmeEnCours = 1;
                aubeActive = 1;
            }
        }

        if (demandeStop)
        {
            demandeStop = 0;

            if (alarmeEnCours)
            {
                alarmeEnCours = 0;
                aubeActive = 0;
                snoozeActif = 0;
            }
        }

        if (demandeSnooze)
        {
            demandeSnooze = 0;

            if (alarmeEnCours)
            {
                alarmeEnCours = 0;
                aubeActive = 0;

                snoozeActif = 1;
                debutSnooze = maintenant;
            }
        }

        if (snoozeActif && alarmeActive && (maintenant - debutSnooze) >= pdMS_TO_TICKS(DUREE_SNOOZE_MS))
        {
            snoozeActif = 0;

            if (!crepusculeActif)
            {
                debutAlarme = maintenant;
                debutAube = maintenant;

                alarmeEnCours = 1;
                aubeActive = 1;
            }
        }

        if (alarmeEnCours)
        {
            if (!alarmeActive || crepusculeActif || (maintenant - debutAlarme) >= pdMS_TO_TICKS(DUREE_ALARME_MS))
            {
                alarmeEnCours = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskAffichage(void *pvParameters)
{
    char texteOled[20];

    while (1)
    {
        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            SSD1306_Fill(0);
            SSD1306_GotoXY(0, 0);

            switch (ecran)
            {
                case 0:

                    SSD1306_Puts("HEURE", &Font_11x18, oledColor);

                    sprintf(texteOled, "%02d:%02d:%02d", horloge.heure, horloge.min, horloge.sec);

                    break;

                case 1:

                    if (alarmeActive)
                        SSD1306_Puts("ALARME ON", &Font_11x18, oledColor);

                    else
                        SSD1306_Puts("ALARME OFF", &Font_11x18, oledColor);

                    sprintf(texteOled, "%02d:%02d:%02d", alarme.heure, alarme.min, alarme.sec);

                    break;

                case 2:

                	SSD1306_Puts("TEMPERATURE", &Font_11x18, oledColor);

                    if (temperature < 0)
                    {
                        int16_t t = -temperature;
                        sprintf(texteOled, "-%d.%02d C", t / 100, t % 100);
                    }

                    else
                    {
                        sprintf(texteOled, "%d.%02d C", temperature / 100, temperature % 100);
                    }

                    break;

                case 3:

                    SSD1306_Puts("QUALITE AIR", &Font_11x18, oledColor);

                    sprintf(texteOled, "VOC : %d", voc);

                    break;

                default:

                    SSD1306_Puts("AFFICHAGE", &Font_11x18, oledColor);

                    sprintf(texteOled, "...");

                    break;
            }

            SSD1306_GotoXY(0, 30);
            SSD1306_Puts(texteOled, &Font_11x18, oledColor);
            SSD1306_UpdateScreen();

            xSemaphoreGive(mutexI2C);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void TaskBluetooth(void *pvParameters)
{
    int h, m, s;
    int valeur;
    char reponse[256];

    while (1)
    {
        xSemaphoreTake(semaphBT, portMAX_DELAY);

        if (bt_char == '\r' || bt_char == '\n')
        {
            if (bt_index > 0)
            {
                bt_buffer[bt_index] = '\0';

                if (sscanf(bt_buffer, "heure : %d:%d:%d", &h, &m, &s) == 3)
                {

                    if ((h >= 0 && h < 24) && (m >= 0 && m < 60) && (s >= 0 && s < 60))
                    {
                        horloge.heure = h;
                        horloge.min   = m;
                        horloge.sec   = s;

                        sprintf(reponse, "OK heure : %02d:%02d:%02d\r\n", h, m, s);
                    }

                    else
                    {
                        sprintf(reponse, "ERREUR heure invalide\r\n");
                    }

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (sscanf(bt_buffer, "alarme : %d:%d:%d", &h, &m, &s) == 3)
                {

                    if ((h >= 0 && h < 24) && (m >= 0 && m < 60) && (s >= 0 && s < 60))
                    {
                        alarme.heure = h;
                        alarme.min   = m;
                        alarme.sec   = s;

                        alarmeActive = 1;

                        sprintf(reponse, "OK alarme : %02d:%02d:%02d\r\n", h, m, s);
                    }

                    else
                    {
                        sprintf(reponse, "ERREUR alarme invalide\r\n");
                    }

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (strcmp(bt_buffer, "alarme off") == 0)
                {

                    alarmeActive = 0;

                    sprintf(reponse, "OK alarme desactivee\r\n");

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (strcmp(bt_buffer, "alarme on") == 0)
                {
                    alarmeActive = 1;

                    sprintf(reponse, "OK alarme activee : %02d:%02d:%02d\r\n", alarme.heure, alarme.min, alarme.sec);

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (sscanf(bt_buffer, "temperature : %d", &valeur) == 1)
                {
                    if (valeur >= -40 && valeur <= 125)
                    {
                        seuilTemperature = (int16_t)(valeur * 100);

                        sprintf(reponse, "OK temperature : %d C\r\n", valeur);
                    }

                    else
                    {
                        sprintf(reponse, "ERREUR temperature invalide\r\n");
                    }

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (sscanf(bt_buffer, "voc : %d", &valeur) == 1)
                {

                    if (valeur >= 0 && valeur <= 65535)
                    {
                        seuilVOC = (uint16_t)valeur;
                        sprintf(reponse, "OK voc : %d\r\n", valeur);
                    }

                    else
                    {
                        sprintf(reponse, "ERREUR voc invalide\r\n");
                    }

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (sscanf(bt_buffer, "ldr : %d", &valeur) == 1)
                {
                    if (valeur >= 0 && valeur <= 4095)
                    {
                        seuilLDR = (uint16_t)valeur;

                        sprintf(reponse, "OK ldr : %d\r\n", valeur);
                    }

                    else
                    {
                        sprintf(reponse, "ERREUR ldr invalide\r\n");
                    }

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }


                else if (strcmp(bt_buffer, "status") == 0)
                {
                    sprintf(reponse,

                            "Heure : %02d:%02d:%02d\r\n"
                            "Alarme : %02d:%02d:%02d [%s]\r\n"
                            "Temperature seuil : %d C\r\n"
                            "VOC seuil : %u\r\n"
                            "LDR seuil : %u\r\n",

                            horloge.heure, horloge.min, horloge.sec,
                            alarme.heure, alarme.min, alarme.sec,
                            alarmeActive ? "ON" : "OFF",
                            seuilTemperature / 100,
                            seuilVOC,
                            seuilLDR);

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }

                else
                {
                    sprintf(reponse, "ERREUR commande inconnue\r\n");

                    HAL_UART_Transmit(&hlpuart1, (uint8_t *)reponse, strlen(reponse), HAL_MAX_DELAY);
                }

                bt_index = 0;
            }
        }

        else
        {
            if (bt_index < sizeof(bt_buffer) - 1)
            {
                bt_buffer[bt_index++] = bt_char;
            }

            else
            {
                bt_index = 0;
            }

        }

        HAL_UART_Receive_IT(&hlpuart1, &bt_char, 1);
    }
}

void TaskCapteurs(void *pvParameters)
{
    TickType_t dernierADC = xTaskGetTickCount();
    TickType_t dernierTemp = xTaskGetTickCount();
    TickType_t dernierVOC = xTaskGetTickCount();

    TickType_t debutConversion = 0;
    uint8_t conversionEnCours = 0;

    const TickType_t periodeADC = pdMS_TO_TICKS(100);
    const TickType_t periodeTemp = pdMS_TO_TICKS(1000);
    const TickType_t periodeVOC = pdMS_TO_TICKS(1000);
    const TickType_t dureeConversion = pdMS_TO_TICKS(150);

    uint8_t msbvalue, lsbvalue, sign;
    uint16_t temp;

    while (1)
    {
        TickType_t maintenant = xTaskGetTickCount();

        // ADC : 100 ms
        if ((maintenant - dernierADC) >= periodeADC)
        {
            dernierADC += periodeADC;

            if (HAL_ADC_Start(&hadc1) == HAL_OK)
            {
                if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
                {
                    adcPot = HAL_ADC_GetValue(&hadc1);
                }

                if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
                {
                    adcLdr = HAL_ADC_GetValue(&hadc1);
                    oledColor = (adcLdr < seuilLDR) ? 1 : 0;
                }

                HAL_ADC_Stop(&hadc1);
            }
        }

        // Température : 1 s
        if (!conversionEnCours &&
            (maintenant - dernierTemp) >= periodeTemp)
        {
            dernierTemp = maintenant;

            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);

            HAL_StatusTypeDef etat = HAL_SPI_Transmit(
                &hspi2, control_T, sizeof(control_T), 20);

            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

            if (etat == HAL_OK)
            {
                debutConversion = xTaskGetTickCount();
                conversionEnCours = 1;
            }
        }

        // Température : 150 ms
        if (conversionEnCours &&
            (xTaskGetTickCount() - debutConversion) >= dureeConversion)
        {
            conversionEnCours = 0;

            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);

            HAL_StatusTypeDef etatTX =
                HAL_SPI_Transmit(&hspi2, &Temp_Address, 1, 20);

            HAL_StatusTypeDef etatRX = HAL_ERROR;

            if (etatTX == HAL_OK)
            {
                etatRX = HAL_SPI_Receive(&hspi2, Temp, 3, 20);
            }

            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

            if (etatTX == HAL_OK && etatRX == HAL_OK)
            {
                msbvalue = Temp[0];
                lsbvalue = Temp[1];

                temp = ((((uint16_t)(msbvalue & 0x7F) << 2)
                         | (lsbvalue >> 6)) * 25);

                sign = (msbvalue & 0x80) ? 1 : 0;

                temperature = sign ? -(int16_t)temp : (int16_t)temp;
            }
        }

        // VOC : toutes les secondes
        maintenant = xTaskGetTickCount();

        if ((maintenant - dernierVOC) >= periodeVOC)
        {
            dernierVOC = maintenant;

            float voc_temp = temperature / 100.0f;
            float voc_humi = 50.0f;

            if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                voc = SGP40_MeasureVOC(voc_temp, voc_humi);

                xSemaphoreGive(mutexI2C);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskVentilo(void *pvParameters)
{
    uint8_t direction = 0;
    uint8_t modePrecedent = 0xFF;
    TickType_t dernierChangement = 0;
    const TickType_t periode = pdMS_TO_TICKS(2000);

    Ventilo_Set(0, 0);

    while (1)
    {
        TickType_t maintenant = xTaskGetTickCount();
        uint32_t vitesse;

        if (adcPot > POT_SEUIL_MANUEL)
        {
        	vitesse = ((uint32_t)adcPot * 79999U) / 4095U;
            Ventilo_Set(0, vitesse);
            modePrecedent = 3;
        }

        else
        {
        	if (temperature > seuilTemperature)
        	{
        	    int32_t ecart = (int32_t)temperature - seuilTemperature;

        	    if (ecart >= 1000)
        	        vitesse = 79999U;
        	    else
        	        vitesse = ((uint32_t)ecart * 79999U) / 1000U;

        	    if (modePrecedent != 1)
        	    {
        	        Ventilo_Set(0, vitesse);
        	        modePrecedent = 1;
        	    }

        	    else
        	    {
        	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, vitesse);
        	    }
        	}

            else if (voc > seuilVOC)
            {
                if (modePrecedent != 2)
                {
                    direction = 0;
                    Ventilo_Set(direction, 60000);
                    dernierChangement = maintenant;
                    modePrecedent = 2;
                }

                if ((maintenant - dernierChangement) >= periode)
                {
                    direction = !direction;
                    Ventilo_Set(direction, 60000);
                    dernierChangement = maintenant;
                }
            }

            else
            {
                if (modePrecedent != 0)
                {
                    Ventilo_Set(0, 0);
                    modePrecedent = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void TaskLED(void *pvParameters)
{
    while (1)
    {
        uint8_t red = 0, green = 0, blue = 0;
        TickType_t maintenant = xTaskGetTickCount();

        if (crepusculeActif)
        {
            uint32_t ecoule = (maintenant - debutCrepuscule) * portTICK_PERIOD_MS;

            if (ecoule < DUREE_CREPUSCULE_MS)
            {
                uint32_t reste = 255U - (ecoule * 255U) / DUREE_CREPUSCULE_MS;

                red   = reste;
                green = (100U * reste) / 255U;
                blue  = (20U  * reste) / 255U;
            }

            else
                crepusculeActif = 0;
        }

        else if (aubeActive)
        {
            uint32_t ecoule = (maintenant - debutAube) * portTICK_PERIOD_MS;

            if (ecoule < DUREE_AUBE_MS)
            {
                uint32_t p = (ecoule * 255U) / DUREE_AUBE_MS;

                red   = p;
                green = (180U * p) / 255U;
                blue  = 40U + (100U * p) / 255U;
            }

            else
                aubeActive = 0;
        }

        LED_SetColor(red, green, blue);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TaskAudio(void *pvParameters)
{
    typedef enum
    {
        AUDIO_AUCUN,
        AUDIO_PREVIEW,
        AUDIO_ALARME,
        AUDIO_SOMMEIL,
        AUDIO_TIC
    } AudioMode;

    AudioMode mode = AUDIO_AUCUN;
    AudioMode demande;

    const Note *melodie = NULL;

    Sonnerie melodieAudio;

    uint8_t taille = 0;
    uint8_t index = 0;
    uint8_t phasePause = 0;

    uint8_t versionPreview = audioPreviewVersion;
    uint8_t sommeilJoue = 0;
    uint8_t choixSonnerie = 0;

    TickType_t debutPhase = 0;
    TickType_t dureePhase = 0;

    Buzzer_SetFrequency(0);

    while (1)
    {
        TickType_t maintenant = xTaskGetTickCount();

        demande = AUDIO_AUCUN;

        uint8_t nouvelleMelodie = 0;

        // PRIORITE 1 : CREPUSCULE
        if (crepusculeActif)
        {
            if (demandeMelodie)
            {
            	demandeMelodie = 0;
                sommeilJoue = 0;
            }

            if (!sommeilJoue)
            {
                demande = AUDIO_SOMMEIL;
            }
        }

        // PRIORITE 2 : ALARME
        else if (alarmeEnCours)
        {
            demande = AUDIO_ALARME;
        }

        // PRIORITE 3 : TOUCH
        else
        {
            if (versionPreview != audioPreviewVersion)
            {
                versionPreview = audioPreviewVersion;

                if (demandePreview)
                {
                    demande = AUDIO_PREVIEW;
                    nouvelleMelodie = 1;
                }

                else
                {
                    demande = AUDIO_TIC;
                    nouvelleMelodie = 1;
                }

            }

            else if (mode == AUDIO_PREVIEW)
            {
                demande = AUDIO_PREVIEW;
            }

            else if (mode == AUDIO_TIC)
            {
                demande = AUDIO_TIC;
            }

            else if (demandePreview)
            {
                demande = AUDIO_PREVIEW;
            }
        }

        if (demande != mode || nouvelleMelodie)
        {
            Buzzer_SetFrequency(0);

            mode = demande;
            melodie = NULL;
            taille = 0;
            index = 0;
            phasePause = 0;
            dureePhase = 0;

            if (mode == AUDIO_SOMMEIL)
            {
                melodie = melodieDouce;
                taille = sizeof(melodieDouce) / sizeof(Note);
            }

            else if (mode == AUDIO_ALARME)
            {
                choixSonnerie = sonnerieSelectionnee;
            }

            else if (mode == AUDIO_PREVIEW)
            {
                choixSonnerie = previewSonnerie;
                demandePreview = 0;
            }

            else if (mode == AUDIO_TIC)
            {
                Buzzer_SetFrequency(1200);
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

                debutPhase = maintenant;
                dureePhase = pdMS_TO_TICKS(60);
            }

            if (mode == AUDIO_ALARME || mode == AUDIO_PREVIEW)
            {
                if (choixSonnerie < NB_SONNERIES)
                {
                    taskENTER_CRITICAL();

                    melodieAudio = sonneries[choixSonnerie];

                    taskEXIT_CRITICAL();

                    if (melodieAudio.taille > 0)
                    {
                        melodie = melodieAudio.notes;
                        taille = melodieAudio.taille;
                    }
                    else
                    {
                        melodie = melodieDefaut;
                        taille = sizeof(melodieDefaut) / sizeof(melodieDefaut[0]);
                    }
                }

                else
                {
                    melodie = melodieDefaut;
                    taille = sizeof(melodieDefaut) / sizeof(melodieDefaut[0]);
                }
            }
        }

        if (mode == AUDIO_TIC)
        {
            if ((maintenant - debutPhase) >= dureePhase)
            {
                Buzzer_SetFrequency(0);
                mode = AUDIO_AUCUN;
            }
        }

        else if (mode != AUDIO_AUCUN && melodie != NULL)
        {
            if (dureePhase == 0)
            {
                Buzzer_SetFrequency(melodie[index].frequence);
                HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

                debutPhase = maintenant;
                dureePhase =
                    pdMS_TO_TICKS(melodie[index].duree_ms);

                if (dureePhase == 0)
                    dureePhase = 1;
            }

            else if ((maintenant - debutPhase) >= dureePhase)
            {
                if (!phasePause)
                {
                    Buzzer_SetFrequency(0);

                    phasePause = 1;
                    debutPhase = maintenant;

                    dureePhase =
                        pdMS_TO_TICKS(melodie[index].pause_ms);

                    if (dureePhase == 0)
                        dureePhase = 1;
                }
                else
                {
                    phasePause = 0;
                    dureePhase = 0;
                    index++;

                    if (index >= taille)
                    {
                        index = 0;

                        if (mode == AUDIO_SOMMEIL)
                        {
                            sommeilJoue = 1;
                            mode = AUDIO_AUCUN;
                        }

                        else if (mode == AUDIO_PREVIEW)
                        {
                            mode = AUDIO_AUCUN;
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void TaskSD(void *pvParameters)
{
    uint8_t presente = 0;

    while (1)
    {
        if (SD_EST_PRESENTE() != presente)
        {
            vTaskDelay(pdMS_TO_TICKS(50));

            if (SD_EST_PRESENTE() != presente)
            {
                presente = !presente;

                if (presente)
                {
                    HAL_SD_DeInit(&hsd1);

                    if (HAL_SD_Init(&hsd1) == HAL_OK)
                        SD_ChargerSonneries();
                }
                else
                {
                    f_mount(NULL, SDPath, 0);

                    for (uint8_t i = 0; i < NB_SONNERIES; i++)
                        sonneries[i].taille = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void TaskTOF(void *pvParameters)
{
    uint16_t distanceG = 0;
    uint16_t distanceD = 0;

    uint8_t precedentG = 0;
    uint8_t precedentD = 0;

    uint8_t vuG = 0;
    uint8_t vuD = 0;

    uint8_t attenteRetrait = 0;

    TickType_t tempsG = 0;
    TickType_t tempsD = 0;
    TickType_t finCooldown = 0;

    while (1)
    {
        TickType_t maintenant = xTaskGetTickCount();

        if (!alarmeEnCours)
        {
            precedentG = 0;
            precedentD = 0;
            vuG = 0;
            vuD = 0;
            attenteRetrait = 0;

            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            if (tofLeftReady)
            {
                tofLeftReady = 0;

                if (VL53L1_GetRangingMeasurementData(&devLeft, &RangingDataLeft) == VL53L1_ERROR_NONE)
                {
                    distanceG = RangingDataLeft.RangeMilliMeter;
                }

                VL53L1_ClearInterruptAndStartMeasurement(&devLeft);
            }

            if (tofRightReady)
            {
                tofRightReady = 0;

                if (VL53L1_GetRangingMeasurementData(&devRight, &RangingDataRight) == VL53L1_ERROR_NONE)
                {
                    distanceD = RangingDataRight.RangeMilliMeter;
                }

                VL53L1_ClearInterruptAndStartMeasurement(&devRight);
            }

            xSemaphoreGive(mutexI2C);
        }

        uint8_t mainG = (distanceG > 0 && distanceG < TOF_SEUIL_MAIN_MM);

        uint8_t mainD = (distanceD > 0 && distanceD < TOF_SEUIL_MAIN_MM);

        if (attenteRetrait)
        {
            if (!mainG && !mainD && (int32_t)(maintenant - finCooldown) >= 0)
            {
                attenteRetrait = 0;
            }
        }
        else
        {
            if (mainG && !precedentG && !vuG)
            {
                vuG = 1;
                tempsG = maintenant;
            }

            if (mainD && !precedentD && !vuD)
            {
                vuD = 1;
                tempsD = maintenant;
            }

            if (vuG && vuD)
            {
                if (tempsG < tempsD && (tempsD - tempsG) <= pdMS_TO_TICKS(TOF_FENETRE_GESTE_MS))
                {
                    demandeStop = 1;
                }
                else if (tempsD < tempsG && (tempsG - tempsD) <= pdMS_TO_TICKS(TOF_FENETRE_GESTE_MS))
                {
                    demandeSnooze = 1;
                }

                vuG = 0;
                vuD = 0;

                attenteRetrait = 1;
                finCooldown = maintenant + pdMS_TO_TICKS(TOF_COOLDOWN_MS);
            }

            if (vuG && (maintenant - tempsG) > pdMS_TO_TICKS(TOF_FENETRE_GESTE_MS))
            {
                vuG = 0;
            }

            if (vuD && (maintenant - tempsD) > pdMS_TO_TICKS(TOF_FENETRE_GESTE_MS))
            {
                vuD = 0;
            }
        }

        precedentG = mainG;
        precedentD = mainD;

        vTaskDelay(pdMS_TO_TICKS(TOF_PERIODE_MS));
    }
}

void TaskInteraction(void *pvParameters)
{
    const uint32_t canaux[3] = {TSC_GROUP1_IO1, TSC_GROUP1_IO2, TSC_GROUP1_IO3};

    uint8_t precedentTouch[3] = {0};
    uint8_t longValide[3] = {0};
    TickType_t debutAppui[3] = {0};

    GPIO_PinState precedentSW2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);

    TickType_t dernierTouch = xTaskGetTickCount();

    TickType_t debutSW2 = 0;
    uint8_t attenteSW2 = 0;

    while (1)
    {
        TickType_t maintenant = xTaskGetTickCount();

        // SW1
        if (xSemaphoreTake(semaphSW1, 0) == pdTRUE)
        {
            taskENTER_CRITICAL();

            if (!crepusculeActif)
            {
                alarmeEnCours = 0;
                aubeActive = 0;

                demandePreview = 0;

                debutCrepuscule = maintenant;
                crepusculeActif = 1;
                demandeMelodie = 1;
            }

            taskEXIT_CRITICAL();
        }

        // SW2
        GPIO_PinState actuelSW2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);

        if (actuelSW2 == GPIO_PIN_SET && precedentSW2 == GPIO_PIN_RESET)
        {
            debutSW2 = maintenant;
            attenteSW2 = 1;
        }

        if (attenteSW2 && (maintenant - debutSW2) >= pdMS_TO_TICKS(20))
        {
            if (actuelSW2 == GPIO_PIN_SET)
            {
                ecran = (ecran + 1) % 4;
            }

            attenteSW2 = 0;
        }

        precedentSW2 = actuelSW2;

        // Touch
        if ((maintenant - dernierTouch) >= pdMS_TO_TICKS(50))
        {
            dernierTouch = maintenant;

            uint8_t libre = !alarmeEnCours && !crepusculeActif;

            for (uint8_t i = 0; i < 3; i++)
            {
                uint8_t touche = TSC_ReadChannel(canaux[i]) < TOUCH_THRESHOLD;

                TickType_t tempsTouch = xTaskGetTickCount();

                if (touche && !precedentTouch[i])
                {
                    debutAppui[i] = tempsTouch;
                    longValide[i] = 0;

                    if (libre)
                    {
                        previewSonnerie = i;
                        demandePreview = 1;
                        audioPreviewVersion++;
                    }
                }
                else if (touche && !longValide[i] && (tempsTouch - debutAppui[i]) >= pdMS_TO_TICKS(TOUCH_LONG_PRESS_MS))
                {
                    longValide[i] = 1;

                    if (libre)
                    {
                        sonnerieSelectionnee = i;
                        demandePreview = 0;
                        audioPreviewVersion++;
                    }
                }

                precedentTouch[i] = touche;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint32_t dernierSW1 = 0;

    uint32_t maintenant = HAL_GetTick();

    if (GPIO_Pin == SW1_Pin)
    {
        if ((maintenant - dernierSW1) >= 200U)
        {
            dernierSW1 = maintenant;
            xSemaphoreGiveFromISR(semaphSW1, &xHigherPriorityTaskWoken);
        }
    }
    else if (GPIO_Pin == INTOF1_Pin)
    {
        tofLeftIrqTick = maintenant;
        tofLeftReady = 1;
    }
    else if (GPIO_Pin == INTOF2_Pin)
    {
        tofRightIrqTick = maintenant;
        tofRightReady = 1;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == LPUART1)
    {
        xSemaphoreGiveFromISR(semaphBT, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
    for (;;)
    {
        osDelay(1000);
    }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
