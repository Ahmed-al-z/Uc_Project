/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - REFACTORED ETAPE 1
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* LWIP includes */
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include <time.h>
#include "lwip/apps/sntp.h"

extern struct netif gnetif;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t id;
  char     text[128];
} LogMessage_t;


typedef struct {
    float rms_value;       // Valeur de la secousse
    uint32_t timestamp;    // Quand c'est arrivé
    char source_id[16];
} SeismicEvent_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACCEL_SAMPLES_PER_AXIS_WINDOW_MS 10 // Nombre d'échantillons X,Y,Z reçus tous les 100ms
#define WINDOW_MS_IN_S 10                  // 10 x 100ms = 1s
#define ACCEL_SAMPLES_1S (ACCEL_SAMPLES_PER_AXIS_WINDOW_MS * WINDOW_MS_IN_S) // 100 échantillons/axe pour 1s

#define V_REF 3.3f           // Tension de référence ADC (Vref)
#define ADC_MAX_VAL 4095.0f  // Valeur maximale 12 bits

#define NTP_SERVER_IP "162.159.200.1"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

#define RTC_ADDR 0xD0
char g_iso_timestamp[30] = "2026-01-02T00:00:00Z";
#define NTP_PORT 123
#define NTP_MSG_LEN 48
#define NTP_TIMESTAMP_DELTA 2208988800u

volatile uint8_t ntp_synced = 0;
struct udp_pcb *ntp_pcb = NULL;

/* --- Global System Flags --- */


/* Buffer circulaire 1s (100 échantillons/axe) */
float g_accel_x_buffer_1s[ACCEL_SAMPLES_1S];
float g_accel_y_buffer_1s[ACCEL_SAMPLES_1S];
float g_accel_z_buffer_1s[ACCEL_SAMPLES_1S];
volatile uint16_t g_buffer_index = 0;

/* Résultats Traitement (Moyenne Glissante et RMS sur 1s) */
volatile float g_accel_rms_1s = 0.0f;
volatile float g_accel_avg_1s = 0.0f;

/* Sémaphore de synchronisation Acquisition - Traitement */
osSemaphoreId h_AcqProcSemaphore;


/* --- Global System Flags --- */
volatile bool g_user_button_pressed = false;

/* --- Acquisition Variables (Accelerometre) --- */
volatile uint8_t g_adc_dma_complete = 0;       // Flag interruption DMA
uint16_t g_adc_dma_buffer[30];                 // Buffer Circulaire (3 channels * 10 samples)

/* Données traitées (Moyennes) */
float g_accel_x_samples[10], g_accel_y_samples[10], g_accel_z_samples[10];
float g_accel_mean_x = 0.0f;
float g_accel_mean_y = 0.0f;
float g_accel_mean_z = 0.0f;

/* --- RTOS Task Handles  --- */
/* 1. Tâches Principales */
osThreadId h_TaskMaster;        // Tâche Maître (Gère le bouton)
osThreadId h_TaskHeartbeat;     // Tâche Heartbeat (LED)
osThreadId h_TaskDebugUART;     // Tâche Debug (UART)
osMailQId  h_LogQueue;          // Queue pour les messages UART

/* 2. Tâches Acquisition & Métier */
osThreadId h_TaskAcquisition;   // Tâche Acquisition (ADC DMA)
osThreadId h_TaskProcessing;    // Tâche Traitement (Safety/Sismique - Placeholder Step 2)

/* 3. Tâches Stockage & Temps (Placeholder Step 3) */
osThreadId h_TaskTimeSync;
osThreadId h_TaskStorage;

/* 4. Tâches Réseau */
osThreadId h_TaskNetBroadcast;  // Tâche Broadcast (UDP)
osThreadId h_TaskNetServer;     // Serveur (Placeholder Step 2)
osThreadId h_TaskNetClient;     // Client (Placeholder Step 2)


osSemaphoreId h_StorageSemaphore; // Le réveil pour la tâche stockage


#define FRAM_STORAGE_ADDR 0x000000
#define MAX_STORED_EVENTS 10

// 1. Liste des 10 plus gros séismes
volatile SeismicEvent_t g_top_10_events[MAX_STORED_EVENTS];
volatile int g_events_count = 0;

// 2. Flags pour la décision collective (Consensus)
volatile bool g_local_shake_detected = false;
volatile bool g_peer_shake_detected = false;
volatile uint32_t g_last_local_shake_time = 0;
volatile uint32_t g_last_peer_shake_time = 0;

// 3. Outils de Synchro FreeRTOS
osMutexId h_Top10Mutex;          // Protège la liste quand on écrit dedans


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* --- Helper Functions --- */
extern void log_to_uart(const char *format, ...);

/* --- Task Entry Functions --- */
void Start_Task_Master(void const * argument);
void Start_Task_Heartbeat(void const * argument);
void Start_Task_DebugUART(void const * argument);

void Start_Task_Acquisition(void const * argument);
void Start_Task_Processing(void const * argument);

void Start_Task_TimeSync(void const * argument);
void Start_Task_Storage(void const * argument);

void Start_Task_Net_Broadcast(void const * argument);
void Start_Task_Net_Server(void const * argument);
void Start_Task_Net_Client(void const * argument);
void presence_broadcast(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  // Démarrage Hardware
  HAL_TIM_Base_Start(&htim2); // Lance le Timer trigger (100Hz)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adc_dma_buffer, 30); // Lance l'ADC en DMA Circulaire

  HAL_UART_Transmit(&huart3, (uint8_t*)"SYSTEM START... [OK]\r\n", 22, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */

  osMutexDef(Top10Mut);
  h_Top10Mutex = osMutexCreate(osMutex(Top10Mut));


  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* Creation manuelle des tâches avec NOUVEAUX NOMS */

  // 1. Queue de Log
  osMailQDef(logQ, 16, LogMessage_t);
  h_LogQueue = osMailCreate(osMailQ(logQ), NULL);

  // 2. Tâches Système
  osThreadDef(TaskMaster, Start_Task_Master, osPriorityHigh, 0, 128);
  h_TaskMaster = osThreadCreate(osThread(TaskMaster), NULL);

  osThreadDef(TaskHeart, Start_Task_Heartbeat, osPriorityNormal, 0, 128);
  h_TaskHeartbeat = osThreadCreate(osThread(TaskHeart), NULL);

  osThreadDef(TaskDebug, Start_Task_DebugUART, osPriorityBelowNormal, 0, 512);
  h_TaskDebugUART = osThreadCreate(osThread(TaskDebug), NULL);

  // 3. Tâches Acquisition
  osThreadDef(TaskAcq, Start_Task_Acquisition, osPriorityNormal, 0, 512);
  h_TaskAcquisition = osThreadCreate(osThread(TaskAcq), NULL);

  osThreadDef(TaskProc, Start_Task_Processing, osPriorityNormal, 0, 1024);
  h_TaskProcessing = osThreadCreate(osThread(TaskProc), NULL);

  // 4. Tâches Stockage/Temps (Placeholders)
  osThreadDef(TaskTime, Start_Task_TimeSync, osPriorityBelowNormal, 0, 2024);
  h_TaskTimeSync = osThreadCreate(osThread(TaskTime), NULL);

  osThreadDef(TaskStore, Start_Task_Storage, osPriorityLow, 0, 1024);
  h_TaskStorage = osThreadCreate(osThread(TaskStore), NULL);

  // 5. Tâches Réseau
  osThreadDef(TaskBcast, Start_Task_Net_Broadcast, osPriorityBelowNormal, 0, 2024);
  h_TaskNetBroadcast = osThreadCreate(osThread(TaskBcast), NULL);

  osThreadDef(TaskServ, Start_Task_Net_Server, osPriorityAboveNormal, 0, 2024);
  h_TaskNetServer = osThreadCreate(osThread(TaskServ), NULL);

  osThreadDef(TaskCli, Start_Task_Net_Client, osPriorityNormal, 0, 2024);
  h_TaskNetClient = osThreadCreate(osThread(TaskCli), NULL);





    // 6. Sémaphore Acquisition <-> Traitement (Synchronisation 100ms)
    osSemaphoreDef(AcqProcSem);
    h_AcqProcSemaphore = osSemaphoreCreate(osSemaphore(AcqProcSem), 1);



      osSemaphoreDef(StoreSem);
      h_StorageSemaphore = osSemaphoreCreate(osSemaphore(StoreSem), 1);

      // On prend le sémaphore tout de suite pour qu'il soit "vide" au départ (la tâche dormira)
      osSemaphoreWait(h_StorageSemaphore, 0);



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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
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
  hi2c1.Init.Timing = 0x20303E5D;
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
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Heart_Pin|alarme_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Heart_Pin alarme_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Heart_Pin|alarme_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FRAM_CS_Pin */
  GPIO_InitStruct.Pin = FRAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



// --- OUTILS RTC BQ32000 ---

/* USER CODE BEGIN 4 */

// --- 1. CALLBACK DE RECEPTION NTP  ---
void ntp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL && p->tot_len >= 48)
    {
        uint8_t *payload = (uint8_t *)p->payload;

        // Extraction du timestamp (octets 40 à 43)
        uint32_t ntp_seconds = (payload[40] << 24) | (payload[41] << 16) | (payload[42] << 8) | payload[43];

        // Conversion NTP (1900) vers UNIX (1970)
        uint32_t unix_time = ntp_seconds - NTP_TIMESTAMP_DELTA;

        // Conversion en structure de date complète (Année, Mois, Jour...)
        struct tm *time_info;
        time_t raw_time = (time_t)unix_time;
        time_info = gmtime(&raw_time);

        // Mise à jour de la RTC Physique BQ32000
        RTC_SetTime(time_info);

        ntp_synced = 1; // VICTOIRE !
        log_to_uart(" NTP SYNC OK! Date: %04d-%02d-%02d", time_info->tm_year + 1900, time_info->tm_mon+1, time_info->tm_mday);
    }

    pbuf_free(p);
    if (pcb != NULL) {
        udp_remove(pcb);
        ntp_pcb = NULL;
    }
}

// --- 2. FONCTION D'ENVOI NTP MANUEL ---
void Sync_Time_NTP_Manual(void)
{
    struct pbuf *p;
    ip_addr_t dest_ip;
    err_t err;

    // Nettoyage si un ancien essai traîne
    if (ntp_pcb != NULL) {
        udp_remove(ntp_pcb);
        ntp_pcb = NULL;
    }

    // Création Socket UDP
    ntp_pcb = udp_new();
    if (!ntp_pcb) return;

    // Configuration Callback
    udp_recv(ntp_pcb, ntp_recv_callback, NULL);

    ipaddr_aton("162.159.200.1", &dest_ip);

    // Préparation du paquet
    p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    if (!p) {
        udp_remove(ntp_pcb);
        return;
    }
    memset(p->payload, 0, NTP_MSG_LEN);
    ((uint8_t*)p->payload)[0] = 0x1B; // Mode Client

    // Envoi
    err = udp_sendto(ntp_pcb, p, &dest_ip, NTP_PORT);
    pbuf_free(p);

    if (err == ERR_OK) {
        log_to_uart("NTP: Request sent to 162.159.200.1...");
    } else {
        udp_remove(ntp_pcb);
        ntp_pcb = NULL;
    }
}

// Conversion BCD - Décimal [cite: 60, 62]
uint8_t BCD_to_Decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Conversion Décimal - BCD (Nécessaire pour écrire l'heure NTP dans la RTC)
uint8_t Decimal_to_BCD(uint8_t decimal) {
    return (((decimal / 10) << 4) | (decimal % 10));
}


// --- OUTILS RTC BQ32000 ---


// Lecture Sécurisée
bool RTC_GetTime(struct tm *time_info) {
    uint8_t buffer[7];
    memset(time_info, 0, sizeof(struct tm)); // Reset RAM

    if (HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 7, 100) == HAL_OK) {

        // Si on lit 0xFF partout, le bus I2C est planté -> on ignore
        if(buffer[0] == 0xFF && buffer[6] == 0xFF) return false;

        time_info->tm_sec  = BCD_to_Decimal(buffer[0] & 0x7F); // Important : Masque 0x7F
        time_info->tm_min  = BCD_to_Decimal(buffer[1]);
        time_info->tm_hour = BCD_to_Decimal(buffer[2]);
        // Reg 3 = Jour semaine
        time_info->tm_mday = BCD_to_Decimal(buffer[4]);
        time_info->tm_mon  = BCD_to_Decimal(buffer[5]) - 1;
        time_info->tm_year = BCD_to_Decimal(buffer[6]) + 100; // +100 car tm_year commence en 1900

        // Filtre anti-date absurde (2165 = tm_year 265)
        if (time_info->tm_year > 150) return false;

        return true;
    }
    return false;
}

// Ecriture
void RTC_SetTime(struct tm *time_info) {
    uint8_t buffer[7];
    buffer[0] = Decimal_to_BCD(time_info->tm_sec);
    buffer[1] = Decimal_to_BCD(time_info->tm_min);
    buffer[2] = Decimal_to_BCD(time_info->tm_hour);
    buffer[3] = Decimal_to_BCD(time_info->tm_wday == 0 ? 7 : time_info->tm_wday);
    buffer[4] = Decimal_to_BCD(time_info->tm_mday);
    buffer[5] = Decimal_to_BCD(time_info->tm_mon + 1);
    buffer[6] = Decimal_to_BCD(time_info->tm_year % 100); // On écrit juste "25" pour 2025

    HAL_I2C_Mem_Write(&hi2c1, RTC_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 7, 100);
}



/* ======================= USER HELPERS ======================= */
void log_to_uart(const char *format, ...)
{
    LogMessage_t *msg = osMailAlloc(h_LogQueue, 0);
    if (!msg) return;

    va_list ap;
    va_start(ap, format);
    vsnprintf(msg->text, sizeof(msg->text), format, ap);
    va_end(ap);

    osMailPut(h_LogQueue, msg);
}

/* ======================= TASK IMPLEMENTATIONS ======================= */

/* 6. Tâche Synchronisation Temps (NTP + RTC) */
/* 6. Tâche Synchronisation Temps (NTP + RTC) */
void Start_Task_TimeSync(void const * argument)
{
    // 1. Attente Réseau
    while (!netif_is_up(&gnetif) || ip_addr_isany(&gnetif.ip_addr)) {
        osDelay(1000);
    }

    // Initialisation valeur par défaut propre
    snprintf(g_iso_timestamp, sizeof(g_iso_timestamp), "2000-01-01T00:00:00Z");

    log_to_uart("TIME: Network OK. Starting Manual NTP...");

    struct tm time_struct;
    int retry_count = 0;

    // 2. BOUCLE DE SYNCHRO NTP ( Bloquante au démarrage)
    // On insiste tant qu'on n'a pas reçu l'heure (ntp_synced passe à 1 dans le callback)
    while (ntp_synced == 0) {

        Sync_Time_NTP_Manual(); // Envoi la demande UDP brute

        // On attend la réponse pendant 5 secondes (50 x 100ms)
        for(int i=0; i<50; i++) {
            osDelay(100);
            if(ntp_synced) break; // Si le callback a mis le flag à 1, on sort
        }

        if(ntp_synced == 0) {
            retry_count++;
            log_to_uart("TIME: No response, retrying (%d)...", retry_count);
        }
    }

    log_to_uart("TIME: Sync Complete! Entering main loop.");

    // 3. Boucle principale : Mise à jour de la variable globale g_iso_timestamp
    for(;;)
    {
        // On lit la RTC physique (qui a été mise à l'heure par le NTP)
        if (RTC_GetTime(&time_struct)) {
            snprintf(g_iso_timestamp, sizeof(g_iso_timestamp),
                     "%04d-%02d-%02d T %02d:%02d:%02d",
                     time_struct.tm_year + 1900,
                     time_struct.tm_mon + 1,
                     time_struct.tm_mday,
                     time_struct.tm_hour + 1,
                     time_struct.tm_min,
                     time_struct.tm_sec);
        }

        osDelay(1000);
    }
}

/* 1. Tâche Maître (Gère le bouton) */
void Start_Task_Master(void const * argument)
{
	static bool running = true;
	static bool lastButtonState = false;

    for(;;) {
    	bool currentButton = g_user_button_pressed;

    	if(currentButton && !lastButtonState){ // Toggle
    		if(!running){
				vTaskResume(h_TaskHeartbeat);
				log_to_uart("CMD: System RESUMED");
				running = true;
			}
    		else{
    			vTaskSuspend(h_TaskHeartbeat);
				log_to_uart("CMD: System SUSPENDED");
				running = false;
			}
    	}
    	lastButtonState = currentButton;
    	osDelay(20); // Debounce
    }
}

/* 2. Tâche Heartbeat */
void Start_Task_Heartbeat(void const * argument)
{
    for(;;) {
    	HAL_GPIO_TogglePin(GPIOB, Heart_Pin);
    	osDelay(400);
    }
}

/* 3. Tâche Debug UART */
void Start_Task_DebugUART(void const * argument)
{
    for (;;)
    {
        osEvent evt = osMailGet(h_LogQueue, osWaitForever);
        if (evt.status == osEventMail)
        {
            LogMessage_t *m = evt.value.p;
            HAL_UART_Transmit(&huart3, (uint8_t*)m->text, strlen(m->text), 100);
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 100);
            osMailFree(h_LogQueue, m);
        }
    }
}

/* 4. Tâche Acquisition (ADC) */
void Start_Task_Acquisition(void const * argument)
{
    for(;;) {
        // float sum_x = 0, sum_y = 0, sum_z = 0; // Anciens calculs de moyenne retirés

		if(g_adc_dma_complete){
            // Les 30 échantillons bruts (10x X,Y,Z) sont disponibles dans g_adc_dma_buffer

            // L'index de départ pour ce nouveau bloc de 10 échantillons dans le buffer 1s
            uint16_t start_idx = g_buffer_index;

			for(int i=0; i<30; i++){
				// Conversion 12 bits -> Volts (3.3V)
				float val = ((float)g_adc_dma_buffer[i] * V_REF) / ADC_MAX_VAL;

                // Calcul de l'index de stockage pour l'échantillon i dans le buffer 1s
                uint16_t sample_idx = start_idx + (i / 3);

                if(i % 3 == 0) { // X
					g_accel_x_buffer_1s[sample_idx] = val;
				}
				else if(i % 3 == 1) { // Y
					g_accel_y_buffer_1s[sample_idx] = val;
				}
				else if(i % 3 == 2) { // Z
					g_accel_z_buffer_1s[sample_idx] = val;
				}
			}

            // Mettre à jour l'index du prochain bloc de 10 échantillons (100ms)
            g_buffer_index += ACCEL_SAMPLES_PER_AXIS_WINDOW_MS;
            if (g_buffer_index >= ACCEL_SAMPLES_1S) {
                g_buffer_index = 0; // Retour au début du buffer circulaire
            }

			g_adc_dma_complete = 0; // Reset Flag DMA

            // Signalement à la tâche de traitement que 10 nouveaux échantillons sont là
            osSemaphoreRelease(h_AcqProcSemaphore);
		}
        osDelay(1); // La tâche ne doit pas monopoliser le CPU
    }
}


/* 5. Tâche Broadcast (UDP) */
void Start_Task_Net_Broadcast(void const * argument)
{
    /* 1. Attente du lien Ethernet */
    log_to_uart("NET: Waiting for Link...");
    while (!netif_is_up(&gnetif) || ip_addr_isany(&gnetif.ip_addr)) {
        osDelay(500);
    }

    // Affichage IP une seule fois
    char ip_txt[16];
    ipaddr_ntoa_r(&gnetif.ip_addr, ip_txt, 16);
    log_to_uart("NET: Ready! My IP is: %s", ip_txt);

    for(;;)
    {
        // Appel de la fonction séparée
        send_presence_broadcast();

        // Délai de 10s pour les tests
        osDelay(10000);
    }
}


/* USER CODE BEGIN 4 */

//le unicast fonctionnait mais pas le broadcast car lwip bloque le broadcast donc il fallait ajouter : ip_set_option(pcb, SOF_BROADCAST);
/* Fonction send_presence_broadcast    */
void send_presence_broadcast(void)
{
    struct udp_pcb *pcb;
    struct pbuf *p;
    err_t err;
    char msg[256];
    char my_ip[16];

    // 1. Création du PCB
    pcb = udp_new();
    if (pcb == NULL) return;

    //   Autoriser le Broadcast sur ce PCB
    // Sans ça, LwIP refuse d'envoyer le paquet et Wireshark ne voit rien.
    ip_set_option(pcb, SOF_BROADCAST);

    udp_bind(pcb, IP_ADDR_ANY, 0);

    ipaddr_ntoa_r(&gnetif.ip_addr, my_ip, 16);
    snprintf(msg, sizeof(msg),
        "{\n"
        " \"type\": \"presence\",\n"
        " \"id\": \"nucleo-Ahmed\",\n"
        " \"ip\": \"%s\",\n"
        " \"timestamp\": \"%s\"\n"
        "}",
        my_ip, g_iso_timestamp);

    // 4. Envoi
    p = pbuf_alloc(PBUF_TRANSPORT, strlen(msg), PBUF_RAM);
    if (p != NULL) {
        pbuf_take(p, (char*)msg, strlen(msg));

        udp_sendto_if(pcb, p, IP_ADDR_BROADCAST, 12345, &gnetif);


        pbuf_free(p);
    }

    udp_remove(pcb);
}

/* 7. Tâche Traitement (Seismic processing + Consensus) */
void Start_Task_Processing(void const * argument)
{
    const float SEISMIC_THRESHOLD_RMS = 0.050f;
    osDelay(1000);

    bool alarm_active_state = false;

    for(;;)
    {
        osSemaphoreWait(h_AcqProcSemaphore, osWaitForever);

        float sum_x = 0, sum_y = 0, sum_z = 0;
        float sum_sq_diff = 0;

        // --- CALCULS MATHS ---
        // 1. Moyenne
        for (int i = 0; i < ACCEL_SAMPLES_1S; i++) {
            sum_x += g_accel_x_buffer_1s[i];
            sum_y += g_accel_y_buffer_1s[i];
            sum_z += g_accel_z_buffer_1s[i];
        }
        g_accel_mean_x = sum_x / (float)ACCEL_SAMPLES_1S;
        g_accel_mean_y = sum_y / (float)ACCEL_SAMPLES_1S;
        g_accel_mean_z = sum_z / (float)ACCEL_SAMPLES_1S;

        // 2. RMS
        for (int i = 0; i < ACCEL_SAMPLES_1S; i++) {
            float dx = g_accel_x_buffer_1s[i] - g_accel_mean_x;
            float dy = g_accel_y_buffer_1s[i] - g_accel_mean_y;
            float dz = g_accel_z_buffer_1s[i] - g_accel_mean_z;
            sum_sq_diff += (dx*dx) + (dy*dy) + (dz*dz);
        }
        g_accel_rms_1s = sqrtf(sum_sq_diff / (float)(3 * ACCEL_SAMPLES_1S));

        // --- LOGIQUE DETECTION ---

        // A. Détection Locale
        if (g_accel_rms_1s > SEISMIC_THRESHOLD_RMS) {
            g_local_shake_detected = true;
            g_last_local_shake_time = HAL_GetTick();

            // SAUVEGARDE FRAM (Top 10)
            update_top_10_events(g_accel_rms_1s, "local");

            HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
        } else {
            // Reset après 2s
            if (HAL_GetTick() - g_last_local_shake_time > 2000) {
                g_local_shake_detected = false;
                 HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
            }
        }

        // B. Consensus (ALARME GENERALE)
        bool condition_consensus = g_local_shake_detected && g_peer_shake_detected;

        if (g_peer_shake_detected && (HAL_GetTick() - g_last_peer_shake_time > 2000)) {
            g_peer_shake_detected = false; // Info trop vieille
            condition_consensus = false;
        }

        if (condition_consensus) {
            // --- CAS D'ALARME ---
            HAL_GPIO_WritePin(GPIOB, alarme_Pin, GPIO_PIN_SET); // LED ROUGE ON

            if (!alarm_active_state) {
                vTaskSuspend(h_TaskHeartbeat);
                alarm_active_state = true;
                log_to_uart("!!! ALARME GENERALE - SYSTEME FIGE !!!");
            }

        } else {
            // --- CAS NORMAL ---
            HAL_GPIO_WritePin(GPIOB, alarme_Pin, GPIO_PIN_RESET); // LED ROUGE OFF

            if (alarm_active_state) {
                // L'alarme est finie, on relance le BEAT
                vTaskResume(h_TaskHeartbeat);
                alarm_active_state = false;
                log_to_uart("Systeme Stabilise - Reprise");
            }
        }
    }
}

/* 8. Tâche Serveur TCP (Répond aux requêtes) */
void Start_Task_Net_Server(void const * argument)
{
    int server_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len;
    int bytes_read;
    char rx_buffer[512];
    char tx_buffer[512];

    while (!netif_is_up(&gnetif) || ip_addr_isany(&gnetif.ip_addr)) {
        osDelay(1000);
    }
    log_to_uart("SRV: Server Ready on Port 12345");

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        log_to_uart("SRV: Error creating socket");
        vTaskDelete(NULL);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(12345);
    server_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        log_to_uart("SRV: Error binding");
        close(server_sock);
        vTaskDelete(NULL);
    }

    if (listen(server_sock, 5) < 0) {
        log_to_uart("SRV: Error listening");
        close(server_sock);
        vTaskDelete(NULL);
    }

    for(;;)
    {
        client_addr_len = sizeof(client_addr);
        // Accept est bloquant
        client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_addr_len);

        if (client_sock >= 0) {

            memset(rx_buffer, 0, sizeof(rx_buffer));
            // On laisse 100ms max pour recevoir la donnée (évite un blocage infini si le client n'envoie rien)
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100 ms
            setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

            bytes_read = recv(client_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (bytes_read > 0) {
                // Si c'est une requête valide
                if (strstr(rx_buffer, "data_request") != NULL)
                {
                    float my_x = g_accel_mean_x;
                    float my_y = g_accel_mean_y;
                    float my_z = g_accel_mean_z;

                    int len = snprintf(tx_buffer, sizeof(tx_buffer),
                        "{\n"
                        " \"type\": \"data_response\",\n"
                        " \"id\": \"nucleo-Ahmed\",\n"
                        " \"timestamp\": \"%s\",\n"
                        " \"acceleration\": {\n"
                        "   \"x\": %.2f,\n"
                        "   \"y\": %.2f,\n"
                        "   \"z\": %.2f\n"
                        " },\n"
                        " \"status\": \"%s\"\n"
                        "}",
						g_iso_timestamp,
                        my_x, my_y, my_z,
                        (g_accel_rms_1s > 0.05f) ? "alert" : "normal"
                    );

                    send(client_sock, tx_buffer, len, 0);
                }
            }


            osDelay(20);

            close(client_sock);
        }

        osDelay(10);
    }
}


typedef struct {
    char* name;
    char* id;
    char* ip;
} Peer_t;

/* 9. Tâche Client TCP (Interroge les autres cartes) */
void Start_Task_Net_Client(void const * argument)
{
    Peer_t peers[] = {
    		//{"Kate",     "nucleo-6",  "192.168.129.72"},
    		//  {"Arthur",   "nucleo-14", "192.168.1.185"},
    		 {"Ilya",     "nucleo-8",  "192.168.129.181"},
    		//  {"Maxime",   "nucleo-3",  "192.168.1.183"},
    		//   {"Charles",  "nucleo-20", "192.168.1.151"},
    		//   {"Marvin",   "nucleo-12", "192.168.1.191"},
    		  // {"Nico",    "nucleo-11", "192.168.129.190"},
       {"PC_Sim",   "pc-debug",  "192.168.129.1"}
    };

    int num_peers = sizeof(peers) / sizeof(peers[0]);
    int sock;
    struct sockaddr_in server_addr;
    char tx_buffer[256];
    char rx_buffer[512];
    int bytes_received;

    // Attente lien réseau
    while (!netif_is_up(&gnetif) || ip_addr_isany(&gnetif.ip_addr)) {
        osDelay(1000);
    }
    log_to_uart("CLI: Client Task Started. Scanning %d peers...", num_peers);

    for(;;)
    {
        // On parcourt la liste des collègues
        for (int i = 0; i < num_peers; i++)
        {
            // 1. Création Socket
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                log_to_uart("CLI: Error creating socket");
                continue;
            }

            // Configuration du timeout de réception
            struct timeval tv;
            tv.tv_sec = 2;   // 2 secondes max d'attente pour la réponse
            tv.tv_usec = 0;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

            // 2. Configuration Adresse Cible
            memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(12345); // Port standard du projet
            server_addr.sin_addr.s_addr = inet_addr(peers[i].ip);

            log_to_uart("CLI: Connecting to %s (%s)...", peers[i].name, peers[i].ip);

            // 3. Connexion (Connect)
            if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                // Echec connexion
                 log_to_uart("CLI: %s Unreachable", peers[i].name);
                close(sock);
                osDelay(100); // Petite pause avant le suivant
                continue;
            }

            // 4. Préparation de la requête JSON
            int len = snprintf(tx_buffer, sizeof(tx_buffer),
                "{\n"
                " \"type\": \"data_request\",\n"
                " \"from\": \"nucleo-Ahmed\",\n"
                " \"to\": \"%s\",\n"
                " \"timestamp\": \"%lu\"\n"
                "}",
                peers[i].id,
				g_iso_timestamp);

            // 5. Envoi
            if (send(sock, tx_buffer, len, 0) < 0) {
                log_to_uart("CLI: Send failed to %s", peers[i].name);
                close(sock);
                continue;
            }

            // 6. Réception de la réponse
            memset(rx_buffer, 0, sizeof(rx_buffer));
            bytes_received = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (bytes_received > 0) {
                            // log_to_uart("CLI: RECV from %s", peers[i].name);

                            // --- ETAPE 3 : Analyse simple du JSON ---

                            // 1. Chercher si le voisin est en alerte
                            if (strstr(rx_buffer, "\"status\": \"alert\"") != NULL || strstr(rx_buffer, "\"alert\"") != NULL) {
                                g_peer_shake_detected = true;
                                g_last_peer_shake_time = HAL_GetTick();
                                log_to_uart("CONSENSUS: Peer %s signals ALERT!", peers[i].name);
                            }

                            // 2. Chercher si le voisin envoie une forte vibration (pour le Top 10)
                            char* pX = strstr(rx_buffer, "\"x\":");
                            if(pX) {
                                float val_x = 0;
                                // Lecture un peu "brute" mais efficace
                                if(sscanf(pX + 4, "%f", &val_x) == 1) {
                                     // Si c'est fort (>0.1), on l'enregistre
                                     if(val_x > 0.1f || val_x < -0.1f) {
                                         update_top_10_events(fabs(val_x), peers[i].id);
                                     }
                                }
                            }
                        } else {
                log_to_uart("CLI: No response from %s", peers[i].name);
            }

            // 7. Fermeture propre
            close(sock);

            osDelay(500);
        }

        log_to_uart("CLI: Round finished. Waiting 5s...");
        osDelay(20000);
    }
}






/* 8. Tâche Stockage (Gère la FRAM ) */

// --- A. DRIVER FRAM (Mode Burst) ---
// Adresses et commandes
#define FRAM_WREN  0x06
#define FRAM_WRITE 0x02
#define FRAM_READ  0x03

void FRAM_CS_Select(void) {
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
}
void FRAM_CS_Deselect(void) {
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

// Ecriture rapide d'un bloc complet
void FRAM_WriteBurst(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *pData, uint16_t size) {
    uint8_t cmd[4];
    // 1. WREN
    FRAM_CS_Select();
    cmd[0] = FRAM_WREN;
    HAL_SPI_Transmit(hspi, &cmd[0], 1, 100);
    FRAM_CS_Deselect();
    for(volatile int i=0; i<50; i++); // Petit délai

    // 2. WRITE + Address + Data
    FRAM_CS_Select();
    cmd[0] = FRAM_WRITE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8)  & 0xFF;
    cmd[3] = (address)       & 0xFF;
    HAL_SPI_Transmit(hspi, cmd, 4, 100);       // En-tête
    HAL_SPI_Transmit(hspi, pData, size, 1000); // Données
    FRAM_CS_Deselect();
}

// Lecture rapide d'un bloc complet
void FRAM_ReadBurst(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *pData, uint16_t size) {
    uint8_t cmd[4];
    FRAM_CS_Select();
    cmd[0] = FRAM_READ;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8)  & 0xFF;
    cmd[3] = (address)       & 0xFF;
    HAL_SPI_Transmit(hspi, cmd, 4, 100);
    HAL_SPI_Receive(hspi, pData, size, 1000);
    FRAM_CS_Deselect();
}

// --- B. FONCTIONS HAUT NIVEAU ---
void Save_Top10_To_FRAM(void) {
    // Sauvegarde la tableau ET le compteur
    FRAM_WriteBurst(&hspi2, FRAM_STORAGE_ADDR, (uint8_t*)g_top_10_events, sizeof(g_top_10_events));
    FRAM_WriteBurst(&hspi2, FRAM_STORAGE_ADDR + sizeof(g_top_10_events), (uint8_t*)&g_events_count, sizeof(g_events_count));
}

void Load_Top10_From_FRAM(void) {
    FRAM_ReadBurst(&hspi2, FRAM_STORAGE_ADDR, (uint8_t*)g_top_10_events, sizeof(g_top_10_events));
    FRAM_ReadBurst(&hspi2, FRAM_STORAGE_ADDR + sizeof(g_top_10_events), (uint8_t*)&g_events_count, sizeof(g_events_count));
    // Reset si corrompu
    if(g_events_count < 0 || g_events_count > MAX_STORED_EVENTS) g_events_count = 0;
}

// --- C. LOGIQUE TOP 10 ---
// C'est cette fonction qui décide si on garde l'événement et qui réveille la tâche stockage
void update_top_10_events(float rms, const char* source_id) {
    if (rms < 0.05f) return; // Seuil minimum
    bool changed = false;

    osMutexWait(h_Top10Mutex, osWaitForever); // On protège la liste

    // 1. Ajout direct s'il y a de la place
    if (g_events_count < MAX_STORED_EVENTS) {
        g_top_10_events[g_events_count].rms_value = rms;
        g_top_10_events[g_events_count].timestamp = HAL_GetTick();
        strncpy(g_top_10_events[g_events_count].source_id, source_id, 15);
        g_events_count++;
        changed = true;
    }
    // 2. Sinon, on remplace le plus petit si le nouveau est plus fort
    else {
        int min_idx = 0;
        float min_val = g_top_10_events[0].rms_value;
        for(int i=1; i<MAX_STORED_EVENTS; i++){
            if(g_top_10_events[i].rms_value < min_val){
                min_val = g_top_10_events[i].rms_value;
                min_idx = i;
            }
        }
        if(rms > min_val){
            g_top_10_events[min_idx].rms_value = rms;
            g_top_10_events[min_idx].timestamp = HAL_GetTick();
            strncpy(g_top_10_events[min_idx].source_id, source_id, 15);
            changed = true;
        }
    }

    // SI CHANGEMENT - On réveille le Gatekeeper (Tâche Stockage)
    if (changed) {
        osSemaphoreRelease(h_StorageSemaphore);
    }
    osMutexRelease(h_Top10Mutex);
}



/* 8. Tâche Stockage (Gatekeeper pour la FRAM) */
void Start_Task_Storage(void const * argument)
{
    // 1. Initialisation : Charger les données au démarrage
    FRAM_CS_Deselect();
    osDelay(10); // Laisser la FRAM démarrer

    osMutexWait(h_Top10Mutex, osWaitForever);
    Load_Top10_From_FRAM();
    log_to_uart("STORAGE: Loaded %d events.", g_events_count);
    osMutexRelease(h_Top10Mutex);

    for(;;)
    {
        // 2. ATTENTE : On dort jusqu'à ce que update_top_10_events() nous réveille
        osSemaphoreWait(h_StorageSemaphore, osWaitForever);

        // 3. ACTION : Sauvegarde lente en SPI
        log_to_uart("STORAGE: Saving to FRAM...");

        osMutexWait(h_Top10Mutex, osWaitForever);
        Save_Top10_To_FRAM(); // Fonction définie plus haut
        osMutexRelease(h_Top10Mutex);
    }
}







// --- GESTION DU TEMPS SYSTEME ---
#include <sys/time.h>

// Variable globale pour stocker la différence entre l'heure NTP et le Tick système
static time_t g_time_offset = 0;

// Appelée par "time(NULL)"
int _gettimeofday(struct timeval *tv, void *tzvp)
{
    uint32_t t = HAL_GetTick();
    // On ajoute l'offset (qui sera 0 tant que NTP n'a pas répondu)
    tv->tv_sec = (t / 1000) + g_time_offset;
    tv->tv_usec = (t % 1000) * 1000;
    return 0;
}

// Appelée par SNTP (LwIP) quand il reçoit l'heure
int _settimeofday(const struct timeval *tv, const struct timezone *tz)
{
    uint32_t t = HAL_GetTick();
    // On calcule la différence entre l'heure reçue et le temps système actuel
    g_time_offset = tv->tv_sec - (t / 1000);
    return 0;
}




/* --- Placeholder Tasks (Boucles vides) --- */



/* ======================= CALLBACKS ======================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == USER_Btn_Pin) {
        g_user_button_pressed = !g_user_button_pressed;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	g_adc_dma_complete = 1;
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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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
