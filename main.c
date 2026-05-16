/* USER CODE BEGIN Header */
/**
This code represents the first draft for the VDR release (rel2) control with Vref control, including the LLC soft-start:
- TIM2 is used to trigger the duty cycle variation and the ADC conversions (selectable period from 100us to 1s)
- ADC1_1/7/6/2 are used to acquire Iin, Vout, Vb, Vin respectively
- HRTIM_CHA is used for the LLC signals
- HRTIM_CHE is used for the boost signals
- The input signal at PB8 is used as an external event (EEV8) to trigger the BCM negative current extension period

The code:
- initializes the peripherals and declares the constants / variables
- starts the LLC soft-startup phase (at each TIM2 counter period event, the equivalent duty cycle decreases)
- implements the Vref P&O control by updating Ton until the Vref is reached, taking care of eventual overvoltages

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

HRTIM_HandleTypeDef hhrtim1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

	/* Variables: generic */
	volatile int Regime_Startupn = 0;
	volatile int Boost_SDn = 0;
	volatile int CS_normaln = 0;
	volatile int count = 0;
	uint32_t ADC_Value[4];
	volatile uint32_t Vin_tot = 0;
	volatile uint32_t Iin_tot = 0;
	volatile uint32_t Vb_tot = 0;
	volatile uint32_t Vout_tot = 0;
	volatile float Vin_average = 0;
	volatile float Iin_average = 0;
	volatile float Vb_average = 0;
	volatile float Vout_average = 0;
	volatile float Pin = 0;
	volatile int OV_flag = 0;
	volatile int OVexit_flag = 0;
	volatile int Proximity_flag = 0;
	volatile int Sign_flag = 0;
	volatile int Vref_flag = 0;
	volatile int DTextmax_flag =0;
	volatile int BCM_LPn = 1;
	volatile int DeltaVth = 20;
	volatile float G2;
	volatile float DeltaV;

	/* Constants: prototype 1 */
	//const float Gain_Vin = 14.55;
	//const float Offset_Iin = 0.3287;
	//const float Gain_Iin = 0.099;
	//const float Gain_Vout = 137.1;
	//const float Gain_Vb = 81.36;

	/* Constants: prototype 2 */
	const float Gain_Vin = 14.31;
	const float Offset_Iin = 0.3327;
	const float Gain_Iin = 0.099;
	const float Gain_Vout = 140.8;
	const float Gain_Vb = 83.05;


	/* Constants: generic */
	const int count_tot = 10;
	const uint16_t Vout_max_int = 3427;			// Corresponding to 380V (modify in case of gain adjustment)
	const uint16_t Vout_min_hyst = 3066;		// Corresponding to 340V (modify in case of gain adjustment)
	const float L = 33e-06;
	const float Isat = 15;
	const int DeltaVth_min = 5;
	const int DeltaV_max = 20;


	/* Constants: LLC timing (Tsw could be modified in case the final fsw is different from 250kHz)  */
	const float fHRTIMA = 5.44e+09;
	const float Tsw = 3.846e-06;	// 260kHz
	const float Tsw_HCS = 4.167e-6;	// 240kHz
	const float dtmin = 50e-09;

	/* Variables: LLC timing */
	volatile float Deq = 0.01;
	volatile uint16_t Tsw_ticks;


	/* Constants: Boost timing  */
	const float fHRTIME = 6.8e+08;
	const float fswBmin = 20e+03;
	const float fswBmax = 200e+03;
	const float dtshort = 300e-09;
	const float dtlong = 600e-09;
	const float Tonmin = 200e-09;
	const float DeltaTon_min = 100e-09;
	const float Tswbmin = 4.444e-06;		// Corresponding to 225kHz

	/* Variables: Boost timing */
	volatile float DText = 200e-09;
	volatile float Ton = 6.522e-06;
	volatile uint16_t Ton_ticks;
	volatile uint16_t DText_ticks;
	volatile float DeltaTon_max = 2.0e-06;
	volatile float DeltaTon = 1.0e-06;
	volatile float Tonmax = 2e-05;
	volatile float Tonmin_BCM;
	volatile float IR = -0.05;
	volatile float DeltaIR = 0.02;


	/* Constants to be set for desired working point */
	const int Vref = 250;
	const float IRmin = -0.05;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Start the calibration of the ADC */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* Start the ADC in DMA mode */
  HAL_ADC_Start_DMA(&hadc1, ADC_Value, 4);

  /* Set the period of trigger timer TIM2 at a desired value: 9 (10kHz) -- 99999 (1Hz). 1Hz is good for debugging */
  __HAL_TIM_SET_AUTORELOAD(&htim2, 99999);

  /* Computation of default timing intervals */
  Tsw_ticks = (uint16_t)((Tsw*(1-CS_normaln)+Tsw_HCS*CS_normaln)*fHRTIMA);

  /* Assignment of period and compare values to HRTIM_TA (LLC modulation signals) */
  __HAL_HRTIM_SETPERIOD(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, (uint16_t)((1+CS_normaln)*Tsw_ticks));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, (uint16_t)(Deq*Tsw_ticks));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, (uint16_t)(Tsw_ticks/2));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (uint16_t)((1+2*Deq)*Tsw_ticks/2));

  // Half-cycle skipping
  //__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, (uint16_t)((1+CS_normaln)*Tsw_ticks/2));
  //__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (uint16_t)((1+CS_normaln+2*Deq)*Tsw_ticks/2));


  /* Assignment of compare values to HRTIM_TE (Boost modulation signals) */
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, (uint16_t)(DText*fHRTIME));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, (uint16_t)((DText+dtlong)*fHRTIME));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, (uint16_t)((DText+dtlong+Ton)*fHRTIME));
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_4, (uint16_t)((DText+dtlong+Ton+dtshort)*fHRTIME));


  /* Activate Boost signals (leave "Boost_SDn=0" for LLC only debug)*/
  Boost_SDn = 1;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, Boost_SDn);
  HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1);
  HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2);

  HAL_Delay(5000);


  /* Start LLC timers */
  HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1);
  HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2);


  /* Soft startup phase: boost inactive (SDn = 0) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, Boost_SDn);

  /* Start the timer with interrupt */
  HAL_TIM_Base_Start_IT(&htim2);

  /* Handle soft startup: Deq=49% corresponds to 40ns deadtime, Deq=48% corresponds to 80ns deadtime (with 250kHz)*/
  /* Use Deq<0.475 for 48% duty, and Deq<0.485 for 49% duty */
  while (Deq < 0.475)
	{

	}


	/* After that the equivalent duty cycle has reached around 50%, stop the TIM2 (no more callbacks), pause and then start with the timer again with the steady state */
	HAL_TIM_Base_Stop_IT(&htim2);


	/* Start the timer with interrupt, which now will trigger the ADC conversions */
	//Regime_Startupn = 1;
	//HAL_TIM_Base_Start_IT(&htim2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (OV_flag == 1) {

			/* Overvoltage state: reset count, stop the PWM outputs */
			count = 0;
			Vin_tot = 0;
			Iin_tot = 0;
			Vb_tot = 0;
			Vout_tot = 0;

			/* Stop HRTIM channels for the boost signals */
			HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_E);
			HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1);
			HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2);


		} else if (OVexit_flag == 1) {

			/* Exit the OV state: reset flag, reactivate PWM outputs with default timing */
			OVexit_flag = 0;
			Ton = 3e-06;
			DText = 500e-09;

			/* Update compare registers values */
			__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, (uint16_t)(DText*fHRTIME));
			__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, (uint16_t)((DText+dtlong)*fHRTIME));
			__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, (uint16_t)((DText+dtlong+Ton)*fHRTIME));
			__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_4, (uint16_t)((DText+dtlong+Ton+dtshort)*fHRTIME));

			/* Activate again the HRTIM channels */
			HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1);
			HAL_HRTIM_SimplePWMStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2);


		} else if (count == count_tot) {

			/* Reset count */
			count = 0;

			/* Compute average Vin, Iin, Pin, reset Vin and Iin summations, update Ton */
			Vin_average = (float) ((((float) Vin_tot/count_tot)*3.312/4095)*Gain_Vin);

			// IF THE INPUT CURRENT SENSOR IS NOT PRESENT, ASSUME FIXED Iin_average TO AVOID NEGATIVE DeltaTon_max
			//Iin_average = (float) (((3.312/4095)*((float) Iin_tot/count_tot)-Offset_Iin)/Gain_Iin);
			Iin_average = 10;

			Vb_average = (float) ((((float) Vb_tot/count_tot)*3.312/4095)*Gain_Vb);
			Vout_average = (float)((((float) Vout_tot/count_tot)*3.312/4095)*Gain_Vout);
			DeltaV = (float) (Vout_average-Vref);
			Vin_tot = 0;
			Iin_tot = 0;
			Vout_tot = 0;
			Vb_tot = 0;

			/* Compute boundaries of time intervals according to working point */
			DeltaTon_max = (float) (2*L*Iin_average*DeltaV_max/(Vin_average*Vout_average));
			Tonmax = (float) (L*(Isat-IRmin)/Vin_average);
			G2 = (float) (Vb_average/Vin_average);
			Tonmin_BCM = (float) (Tswbmin*(G2-1)/G2);


			/* Evaluation of flags */

			/* Polarity flag */
			if (DeltaV > 0) {
				Sign_flag = 1;
			} else {
				Sign_flag = 0;
			}

			/* Proximity flag */
			if (DeltaV > -DeltaVth && DeltaV < DeltaVth) {
				Proximity_flag = 1;

				/* Vref convergence flag */
				if (DeltaV > -DeltaVth_min && DeltaV < DeltaVth_min) {
					Vref_flag = 1;
				} else {
					Vref_flag = 0;
				}
			} else {
				Proximity_flag = 0;
				Vref_flag = 0;
			}


			/* Mode selector for the boost control according to Ton and IR values */
			if (BCM_LPn == 0 && IR>IRmin) {
				/* Exit the low power mode: come back to Ton modulation */
				BCM_LPn = 1;
				IR = IRmin;
			} else if (BCM_LPn == 1 && Ton<Tonmin_BCM) {
				/* Enter the low power mode: dt_ext modulation */
				BCM_LPn = 0;
			}
			/* If none of the two conditions is satisfied, it means that the controller remains in the previous state (no BCM_LPn change) */



			/* Ton update according to Vout magnitude and distance to Vref */
			if (Vref_flag == 0) {
				/* Not too close to Vref */

				if (BCM_LPn == 1) {
				/* Ton modulation, adaptive step according to closeness to Vref and polarity */

					if (Proximity_flag == 0) {

						/* Far from Vref: increase step and voltage threshold */
						DeltaTon = MIN(2*DeltaTon, DeltaTon_max);
						DeltaVth = MIN(DeltaVth+5, 20);

						if (Sign_flag == 0) {
							/* Far and below Vref: increase Ton */
							Ton = MIN(Tonmax, Ton+DeltaTon);
						} else {
							/* Far and above Vref: decrease Ton */
							Ton = MAX(Tonmin, Ton-DeltaTon);
						}

					} else {

						/* Close to Vref: decrease step and power threshold */
						DeltaTon = MAX(0.5*DeltaTon, DeltaTon_min);
						DeltaVth = MAX(DeltaVth-5, DeltaVth_min);

						if (Sign_flag == 0) {
							/* Close and below Vref: increase Ton */
							Ton = MIN(Tonmax, Ton+DeltaTon);
						} else {
							/* Close and above Vref: decrease Ton */
							Ton = MAX(Tonmin, Ton-DeltaTon);
						}
					}

					/* Update of DT_ext according to minimum IR value (default value) and instantaneous boost gain */
					DText = (float) MIN((L*(-IRmin)/(Vin_average*(G2-1))), Ton/(2*(G2-1)));

				} else {
				/* DText modulation: update IR value and compute the corresponding DText */

					if (Sign_flag == 0) {
						/* Below Vref: increase IR towards 0 (increase power) */
						IR = IR+DeltaIR;
					} else {
						/* Above Vref: decrease IR further from 0 (decrease power), but only if there is still the possibility to increase DText ensuring positive power (DText<Ton/2) */

						if (DTextmax_flag == 0) {
							/* There is still the possibility to decrease IR (increase DText) */
							IR = IR-DeltaIR;
						}
					}

					/* Update of DT_ext according to updated IR */
					DText = (float) MIN((L*(-IR)/(Vin_average*(G2-1))), Ton/(2*(G2-1)));

					/* If the maximum DText Ton/(2*(G2-1)) has been reached, set the corresponding flag to 0 and at the next cycles IR will no more be reduced */
					if (DText == Ton/(2*(G2-1))) {
						DTextmax_flag = 1;
					} else {
						DTextmax_flag =0;
					}
				}


				/* Update compare registers values */
				__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, (uint16_t)(DText*fHRTIME));
				__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, (uint16_t)((DText+dtlong)*fHRTIME));
				__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, (uint16_t)((DText+dtlong+Ton)*fHRTIME));
				__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_4, (uint16_t)((DText+dtlong+Ton+dtshort)*fHRTIME));



			}
			/* If the outer "if" is not satisfied, it means that P is already very close to Pref and nothing must be done, irrespectively of the boost mode */


		}
		/* Close Overvoltage or Vref P&O mode for Ton/DText update */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_EventCfgTypeDef pEventCfg = {0};
  HRTIM_FaultCfgTypeDef pFaultCfg = {0};
  HRTIM_FaultBlankingCfgTypeDef pFaultBlkCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_TimerEventFilteringCfgTypeDef pTimerEventFilteringCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_FLT1;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_EventPrescalerConfig(&hhrtim1, HRTIM_EVENTPRESCALER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  pEventCfg.Source = HRTIM_EEV8SRC_GPIO;
  pEventCfg.Polarity = HRTIM_EVENTPOLARITY_LOW;
  pEventCfg.Sensitivity = HRTIM_EVENTSENSITIVITY_FALLINGEDGE;
  pEventCfg.Filter = HRTIM_EVENTFILTER_NONE;
  if (HAL_HRTIM_EventConfig(&hhrtim1, HRTIM_EVENT_8, &pEventCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV1) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_DIGITALINPUT;
  pFaultCfg.Polarity = HRTIM_FAULTPOLARITY_LOW;
  pFaultCfg.Filter = HRTIM_FAULTFILTER_NONE;
  pFaultCfg.Lock = HRTIM_FAULTLOCK_READWRITE;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultBlkCfg.Threshold = 0;
  pFaultBlkCfg.ResetMode = HRTIM_FAULTCOUNTERRST_UNCONDITIONAL;
  pFaultBlkCfg.BlankingSource = HRTIM_FAULTBLANKINGMODE_RSTALIGNED;
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultBlankingConfigAndEnable(&hhrtim1, HRTIM_FAULT_1, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_1, HRTIM_FAULTMODECTL_ENABLED);
  pTimeBaseCfg.Period = 20923;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.TrigHalf = HRTIM_TIMERTRIGHALF_DISABLED;
  pTimerCtl.GreaterCMP3 = HRTIM_TIMERGTCMP3_EQUAL;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_UPDATE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 218;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 10880;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 11098;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 2652;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_4, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerEventFilteringCfg.Filter = HRTIM_TIMEEVFLT_BLANKINGCMP4;
  pTimerEventFilteringCfg.Latch = HRTIM_TIMEVENTLATCH_DISABLED;
  if (HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_EVENT_8, &pTimerEventFilteringCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP2;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP3;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP4;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 8138;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL4;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 340;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 544;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 2584;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

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
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1699;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ISR for the end of the TIM2 period */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{

		/* Toggle the user LED to indicate that the Callback is executed (only debug mode!) */
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);


		if (Regime_Startupn == 0)
		{

		    /* Startup phase: update equivalent duty cycle by 1% and update compare registers */
			Deq = Deq + 0.01;
		    __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, (uint16_t)(Deq*Tsw_ticks));
		    __HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (uint16_t)((1+2*Deq)*Tsw_ticks/2));

		    // Half-cycle skipping
		    //__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, (uint16_t)((1+CS_normaln)*Tsw_ticks/2));
		    //__HAL_HRTIM_SETCOMPARE(&hhrtim1,  HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, (uint16_t)((1+CS_normaln+2*Deq)*Tsw_ticks/2));

		} else {

				count = count+1;
				Vin_tot = Vin_tot+ADC_Value[0];
				Iin_tot = Iin_tot+ADC_Value[1];
				Vb_tot = Vb_tot+ADC_Value[3];
				Vout_tot = Vout_tot+ADC_Value[2];

				/* Check on Vout: correct Ton in case of overvoltage */
				if (ADC_Value[2] >= Vout_max_int) {
					/* Enter the OV state */
					OV_flag = 1;

				} else if (ADC_Value[2] < Vout_min_hyst && OV_flag == 1) {
					/* Exit the overvoltage state: reset the flag and reactivate timers with initial Ton value */
					OV_flag = 0;
					OVexit_flag = 1;
				}

		}
		/* Close Regime or startup state check */


	}	/* Close ISR of TIM2 */
}



/* Callback for external asynchronous reset triggered by the B1 pushbutton */
/* Comment: in a future release, implement again the soft startup to come back from the OFF state */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == B1_Pin)
	{
		/* When the B1 pushbutton is pressed, both the HRTIM channels must be reset and their outputs set to 0, and the TIM2 that triggers the ADC must be stopped as well */
		HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_A);
		HAL_HRTIM_SoftwareReset(&hhrtim1, HRTIM_TIMERRESET_TIMER_E);

		// Interrupt the PWM generation on all the PWM channels
		HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1);
		HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2);
		HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1);
		HAL_HRTIM_SimplePWMStop(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2);


		HAL_TIM_Base_Stop_IT(&htim2);
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
