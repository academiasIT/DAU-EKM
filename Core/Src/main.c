/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "uart.h"
#include <stdio.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);

//static void MX_USART1_UART_Init(void);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//#define HEADER_LENGHT (1<<4)
#define DATA_LENGHT 512
#define TOKEN_START 0xFE456688

enum RxState {
	Search, Get_Data
};

volatile uint8_t DataRx[DATA_LENGHT];
volatile uint32_t DataLenght;
volatile uint8_t DataRxRdy;
volatile uint8_t HeaderRx[HEADER_LENGHT];
volatile uint8_t iHeaderRx;
volatile uint8_t RxTokenRdy;
volatile uint32_t RxTokenSec;
volatile enum RxState rx_state;

//uint8_t DataTx[DATA_LENGHT];
//uint8_t HeaderTx[HEADER_LENGHT];
volatile uint8_t flagTx;
volatile uint8_t flagRx;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART1_UART_Init();
	MX_CRC_Init();
	MX_USB_DEVICE_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	printf("Start of main loop\n\r");
	/* USER CODE END 2 */

	while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
		;

	uint8_t result = CDC_Notify_NetworkConnection(0);
	if (result != USBD_OK)
		printf("failed to notify network disconnection\n\r");
	else
		printf("notification disconnection ok\n\r");

	HAL_Delay(5000);
	result = CDC_Notify_NetworkConnection(1);
	if (result != USBD_OK)
		printf("failed to notify network connection\n\r");
	else
		printf("notification connection ok \n\r");

	//CDC_EnableReceiveData_FS();
	//MX_LWIP_Init();
	//USBD_CDC_Notify(&hUsbDeviceFS);
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	HAL_UART_Receive_IT(&huart2, HeaderRx,
	HEADER_LENGHT);
	rx_state = Search;
	iHeaderRx = 0;
	DataRxRdy = 0;
	RxTokenRdy = 0;
	RxTokenSec = 0;
	uint32_t txHeaderSec = 0;

	HAL_StatusTypeDef UartResult;
	uint8_t UsbResult;
	while (1) {

		/* USER CODE END WHILE */
		if (RxTokenRdy == 1) {
			RxTokenRdy = 0;
			printf("Token n: %lu\n", RxTokenSec);
		}
		if (DataRxRdy == 1) {
			DataRxRdy = 0;
			printf("uart data recv\n");
		}
		if (flagTx == 1) {
			printf("-Tx ok\n");
			flagTx = 0;
		}
		if (flagRx == 1) {
			printf("-Rx occur\n");
			flagRx = 0;
		}
		if (flagRx == 2) {
			printf("-Rx header data len: %lu\n", DataLenght);
			flagRx = 0;
		}
		if (flagRx == 3) {
			printf("-Rx Token start\n");
			flagRx = 0;
		}
		if (flagRx == 4) {
			printf("Rx no token\n");
			flagRx = 0;
		}
		/* USER CODE BEGIN 3 */
		if (DataUsbRecv == 1) { /* se ha recibido un paquete por usb */
			/* forming Header*/
			*((uint32_t *) UserRxBufferFS) = TOKEN_START;
			((uint32_t *) UserRxBufferFS)[1] = txHeaderSec++; //numero corelativo del paquete de datos
			((uint32_t *) UserRxBufferFS)[2] = (uint32_t) LengthDataUsbRecv;
			((uint32_t *) UserRxBufferFS)[3] = HAL_CRC_Calculate(&hcrc,
					(uint32_t *) UserRxBufferFS, 3);
			//HAL_UART_Transmit_IT(&huart2, DataTx, HEADER_LENGHT + n);
			flagTx = 0;
			UartResult = HAL_UART_Transmit_IT(&huart2, UserRxBufferFS,
					LengthDataUsbRecv + HEADER_LENGHT);
			DataUsbRecv = 0;
		}
		if (DataUartRecv == 1) {
			//printf("tx usb data\n\r");
			UsbResult = CDC_Transmit_FS(UserTxBufferFS, DataLenght);
			DataUartRecv = 0;
		}

	}
	/* USER CODE END 3 */

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		//printf("uart tx completed\n\r");
		CDC_EnableReceiveData_FS();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t buf[HEADER_LENGHT];
	uint8_t token[4];
	uint32_t crc;
	uint8_t idx;

	if (huart == &huart2) {
		flagRx = 1;
		switch (rx_state) {
		case Search:
			iHeaderRx = 0x0F & ((huart->RxXferSize) + iHeaderRx); //indice del byte consecutivo al ultimo byte recibido

			idx = 0x0F & (iHeaderRx - HEADER_LENGHT); //indice al primer byte de la ventana de header

			for (int i = 0; i < 4; i++) { //obtencion del token en la ventana
				token[i] = HeaderRx[idx];
				idx = 0x0F & (idx + 1);
			}
			//printf("tkn %lx, idx: %u\n",*((uint32_t *) token), idx);

			if (*((uint32_t *) token) == TOKEN_START) {
				//printf("token ok\n");
				flagRx = 3;
				*((uint32_t *) buf) = *((uint32_t *) token);
				for (int i = 4; i < HEADER_LENGHT; i++) {
					buf[i] = HeaderRx[idx];
					idx = 0x0F & (idx + 1);
				}
				//printf("token sec: %lu\n",((uint32_t *) buf)[1]);
				//printf("len: %lu\n",((uint32_t *) buf)[2]);
				crc = HAL_CRC_Calculate(&hcrc, (uint32_t *) buf, 3);
				if (crc == ((uint32_t *) buf)[3]) { //efectivamente se ha recibido una cabecera
					//printf("CRC OK\n");
					flagRx = 2;
					DataLenght = ((uint32_t *) buf)[2];
					HAL_UART_Receive_IT(&huart2, UserTxBufferFS,
							(uint16_t) ((uint32_t *) buf)[2]); // recibe los datos
					RxTokenSec = ((uint32_t *) buf)[1];
					RxTokenRdy = 1;
					rx_state = Get_Data;
				} else {
					HAL_UART_Receive_IT(&huart2, &HeaderRx[iHeaderRx], 1); // recibe un byte mas en la ventana
				}

			} else {
				flagRx = 4;
				HAL_UART_Receive_IT(&huart2, &HeaderRx[iHeaderRx], 1); // recibe un byte mas en la ventana
			}
			//HAL_Delay(10000);
			break;
		case Get_Data:
			/*for(int i=0; i<HEADER_LENGHT; i++)
			 HeaderRx[i] = 0;*/
			iHeaderRx = 0;
			DataUartRecv = 1;
			DataLenght = huart->RxXferSize;
			HAL_UART_Receive_IT(&huart2, HeaderRx, HEADER_LENGHT);
			DataRxRdy = 1;
			rx_state = Search;
			break;
		}

	}
}

/* CRC init function */
static void MX_CRC_Init(void) {

	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	switch (huart->ErrorCode) {
	case HAL_UART_ERROR_NONE:
		printf("HAL_UART_ERROR_NONE\n\r");
		break;
	case HAL_UART_ERROR_PE:
		printf("error HAL_UART_ERROR_PE\n\r");
		break;
	case HAL_UART_ERROR_NE:
		printf("error HAL_UART_ERROR_NE\n\r");
		break;
	case HAL_UART_ERROR_FE:
		printf("error HAL_UART_ERROR_FE\n\r");
		break;
	case HAL_UART_ERROR_ORE:
		printf("error HAL_UART_ERROR_ORE\n\r");
		break;
	case HAL_UART_ERROR_DMA:
		printf("HAL_UART_ERROR_DMA\n\r");
	}
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
