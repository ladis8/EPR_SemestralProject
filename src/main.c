
#include "main.h"

#include <stdint.h>

#include "tiny_printf.h"

UART_HandleTypeDef 		huart;
DAC_HandleTypeDef  		hdac;
static DAC_ChannelConfTypeDef sConfig;

#define LED_PIN_INNER 	GPIO_PIN_5	//PA5

#define MOTA_DIR		GPIO_PIN_6 	//PA6 D12
#define MOTA_DIR_GPIO	GPIOA		//PA6 D12
#define MOTA_PWM 		GPIO_PIN_3	//PB3 D3
#define MOTA_PWM_GPIO	GPIOB
#define MOTA_BREAK 		GPIO_PIN_7	//PC7 D9
#define MOTA_BREAK_GPIO	GPIOC


#define MOTB_DIR		GPIO_PIN_5	//PA5 D13
#define MOTB_DIR_GPIO	GPIOA
#define MOTB_PWM		GPIO_PIN_7	//PA7 D11
#define MOTB_PWM_GPIO	GPIOA
#define MOTB_BREAK 		GPIO_PIN_9 	//PA9 D8
#define MOTB_BREAK_GPIO	GPIOA



void SystemClock_Config(void);
static void Error_Handler(void);


typedef enum {
	SET_DIR,
	SET_BREAK,
	SET_PWM,

} Commands_t;


void PWM_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode =GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate= GPIO_AF2_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//set TIM2 prescaler to 10kHz clock
	//1CNT in ARR == 0.05 ms
	TIM2->PSC = 99;

	//set TIM2 CNT overflow -- frequency 1/(0.00005 * ARR) = 200
	TIM2->ARR = 100;
	//PWM range 0 - 255
		//1 == 0.2 ms  == 5000Hz
		//255 == 51 ms == 19.6 Hz



	//enable mux pins TIM2_CH2 and TIM2_CH3 in output compare mode
	TIM2->CCER |= TIM_CCER_CC2E;
	//set the compare level
	TIM2-> CCR2 = 0;

	//set OC1M to 110 to PWM mode 1
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
	TIM2->CR1 |= TIM_CR1_CEN;

}




void GPIO_Init(){

	GPIO_InitTypeDef  GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = LED_PIN_INNER;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* Configure the MOTOR output pins */
	GPIO_InitStruct.Pin = MOTA_DIR;
	HAL_GPIO_Init(MOTA_DIR_GPIO, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = MOTA_BREAK;
	HAL_GPIO_Init(MOTA_BREAK_GPIO, &GPIO_InitStruct);

	//GPIO_InitStruct.Pin = MOTA_PWM;
	//HAL_GPIO_Init(MOTA_PWM_GPIO, &GPIO_InitStruct);

	/* Configure the MOTOR input pins */
	GPIO_InitStruct.Pin = MOTA_PWM;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(MOTA_PWM_GPIO, &GPIO_InitStruct);
}


void USART2_Init(){
	huart.Instance        = USART2;
	huart.Init.BaudRate   = 115200;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits   = UART_STOPBITS_1;
	huart.Init.Parity     = UART_PARITY_NONE;
	huart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	huart.Init.Mode       = UART_MODE_TX_RX;


	if(HAL_UART_DeInit(&huart) != HAL_OK)
		Error_Handler();

	if(HAL_UART_Init(&huart) != HAL_OK)
		Error_Handler();

}


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__GPIOA_CLK_ENABLE();
	__USART2_CLK_ENABLE();

	/**USART2 GPIO Configuration
	  PA2     ------> USART2_TX
	  PA3     ------> USART2_RX
	  */
	//GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void DAC_Init(){
	hdac.Instance = DAC;

	if (HAL_DAC_Init(&hdac) != HAL_OK)
	    Error_Handler();

	/*##-2- Configure DAC channel1 #############################################*/
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;

	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	    Error_Handler();



}
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* DAC Periph clock enable */
  __HAL_RCC_DAC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  //GPIO_InitStruct.Pin = MOTA_PWM;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


int main(void)
{
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	USART2_Init();
	DAC_Init();
	PWM_Init();

	//Establishes forward direction of Channel A
	HAL_GPIO_WritePin(GPIOA, MOTA_DIR , GPIO_PIN_SET);
	//Disengage the Brake for Channel A
	HAL_GPIO_WritePin(GPIOC, MOTA_BREAK , GPIO_PIN_RESET);

	//HAL_GPIO_WritePin(MOTA_PWM_GPIO, MOTA_PWM , GPIO_PIN_SET);

	//HAL_GPIO_WritePin(GPIOB, MOTA_PWM , GPIO_PIN_SET);
/*
	//##-3- Set DAC Channel1 DHR register ######################################
	if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0xFF) != HAL_OK)
	    Error_Handler();

	//##-4- Enable DAC Channel1 ################################################
	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
	    Error_Handler();

	while (0){
		//HAL_GPIO_WritePin(GPIOA, LED_PIN_INNER , GPIO_PIN_SET);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0xFF);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_Delay(1000);

		//HAL_GPIO_WritePin(GPIOA, LED_PIN_INNER , GPIO_PIN_RESET);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0x00);
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_Delay(1000);
	}*/

	uint8_t RxBuf[3];
	while (1){

    	if (HAL_UART_Receive(&huart, RxBuf, 3, 5000) != HAL_TIMEOUT){

    		switch(RxBuf[0]){
				case SET_DIR:{
					uint8_t motor 	= RxBuf[1];
					uint8_t dir 	= RxBuf[2];
					if (motor == 0x00)
						HAL_GPIO_WritePin(MOTA_DIR_GPIO, MOTA_DIR , dir);
					else if (motor == 0x01)
						HAL_GPIO_WritePin(MOTB_DIR_GPIO, MOTB_DIR , dir);
					else break;
					printf ("Direction was set to %u [0 forward/1 backward] for motor %u\r\n", dir, motor);
					break;

				}
				case SET_BREAK:{
					uint8_t motor 	= RxBuf[1];
					uint8_t br 	= RxBuf[2];
					if (motor == 0x00)
						HAL_GPIO_WritePin(MOTA_BREAK_GPIO, MOTA_BREAK , br);
					else if (motor == 0x01)
						HAL_GPIO_WritePin(MOTB_BREAK_GPIO, MOTB_BREAK , br);
					else break;
					printf ("Break was set to %u [0 turn off/1 turn on] for motor %u\r\n", br, motor);
					break;
				}
				case SET_PWM:{
					uint8_t motor 	= RxBuf[1];
					uint8_t val 	= RxBuf[2];
					if (motor == 0x00){
						//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, val);
						//HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
						TIM2-> CCR2 = val;
					}
					else if (motor == 0x01){
						HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, val);
						HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
					}
					else break;
					printf ("PWM value vas set to %u percent for motor %u\r\n", val, motor);
					break;
				}
				default:
					printf("Unrecognized command\r\n");
    		}
    	}
	}

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 2000000
  *            HCLK(Hz)                       = 2000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 0
  *            Main regulator output voltage  = Scale3 mode
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.MSICalibrationValue=0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
  
}

static void Error_Handler(void){
	while (1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	    HAL_Delay(100);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	    HAL_Delay(100);
	  }
}

int _write(int file, char *data, int len)
{

   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
      return -1;

   HAL_UART_Transmit(&huart, (uint8_t*)data, len, 5000);
   return len;
}


