/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v2.0_Cube
 * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"
#include "uart.h"
/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device library.
 * @{
 */

/** @addtogroup USBD_CDC_IF
 * @{
 */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
 * @brief Private defines.
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
 * @brief Private variables.
 * @{
 */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
//static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_ReceiveData_FS(uint8_t* pbuf, uint32_t *Len);
int8_t CDC_EnableReceiveData_FS(void);
/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = { CDC_Init_FS, CDC_DeInit_FS,
		CDC_Control_FS, CDC_ReceiveData_FS };

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the CDC media low layer over the FS USB IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void) {
	/* USER CODE BEGIN 3 */
	/* Set Application Buffers */
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS, APP_RX_DATA_SIZE);

	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef *) hUsbDeviceFS.pClassData;
	hcdc->NtLength = 0;
	//CDC_Notify_NetworkConnection(1); /*link disconnected*/
	//USBD_CDC_Notify(&hUsbDeviceFS); /*carga la notificacion*/
	UartTxHeader = 1;
	HAL_UART_Receive_IT(&huart2, UserTxBufferFS,2);
	return (USBD_OK);
	/* USER CODE END 3 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void) {
	/* USER CODE BEGIN 4 */
	return (USBD_OK);
	/* USER CODE END 4 */
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
	/* USER CODE BEGIN 5 */
	switch (cmd) {
	case CDC_SEND_ENCAPSULATED_COMMAND:
		printf("CDC_SEND_ENCAPSULATED_COMMAND ctrl recv\n\r");
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		printf("CDC_GET_ENCAPSULATED_RESPONSE ctrl recv\n\r");
		break;

	case SET_ETHERNET_MULTICAST_FILTERS:
		printf("SET_ETHERNET_MULTICAST_FILTERS ctrl recv\n\r");
		break;

	case SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		printf("SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER ctrl recv\n\r");
		break;

	case GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		printf("GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER ctrl recv\n\r");
		break;
	case SET_ETHERNET_PACKET_FILTER:
		printf("SET_ETHERNET_PACKET_FILTER ctrl recv\n\r");
		EtherPacketFilter = ((USBD_SetupReqTypedef *) pbuf)->wValue;
		break;

	case GET_ETHERNET_STATISTIC:
		printf("GET_ETHERNET_STATISTIC ctrl recv\n\r");
		break;

	default:
		break;
	}

	return (USBD_OK);
	/* USER CODE END 5 */
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
/*static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
 {
 // USER CODE BEGIN 6
 USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0], CDC_DATA_FS_OUT_PACKET_SIZE);
 USBD_CDC_ReceivePacket(&hUsbDeviceFS);
 return (USBD_OK);
 // USER CODE END 6
 }*/

/* added by user */
static int8_t CDC_ReceiveData_FS(uint8_t* Buf, uint32_t *Len) {
	/* USER CODE BEGIN 6 */

	DataUsbRecv = 1;
	LengthDataUsbRecv = *((uint16_t *)Len);
	*((uint16_t *)(Buf-2)) = (uint16_t)(*Len);
	//printf("recv data len %d\n\r", *Len);
	//USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0], *Len);
	//USBD_CDC_ReceiveData(&hUsbDeviceFS);
	return (USBD_OK);
	/* USER CODE END 6 */
}

int8_t CDC_EnableReceiveData_FS(void) {
	DataUsbRecv = 0;
	uint16_t pLenghtBufRx = APP_RX_DATA_SIZE;
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[HEADER_LENGHT], &pLenghtBufRx);
	USBD_CDC_ReceiveData(&hUsbDeviceFS);
	return (USBD_OK);
}
/**
 * @brief  CDC_Transmit_FS
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
	uint8_t result = USBD_OK;
	/* USER CODE BEGIN 7 */
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0) {
		return USBD_BUSY;
	}
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &Buf[0], Len);
	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	/* USER CODE END 7 */
	return result;
}

uint8_t CDC_Notify_FS(uint8_t* Buf, uint16_t Len) {
	uint8_t result = USBD_OK;
	/* USER CODE BEGIN 7 */
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef*) hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0) {
		return USBD_BUSY;
	}
	//USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
	result = USBD_CDC_Notify(&hUsbDeviceFS);
	/* USER CODE END 7 */
	return result;
}

uint8_t CDC_Notify_NetworkConnection(uint16_t state) {
	USBD_CDC_HandleTypeDef *hcdc =
			(USBD_CDC_HandleTypeDef *) hUsbDeviceFS.pClassData;
	uint8_t *pBuf = hcdc->NtBuffer;
	uint8_t result = USBD_OK;
	//USB_DisableGlobalInt(hUsbDeviceFS.pData);

	if (hcdc->NtState == 0) {
		((USBD_CDC_NoticationTypeDef *) pBuf)->bmRequestType = 0b10100001;
		((USBD_CDC_NoticationTypeDef *) pBuf)->bNotification =
				NETWORK_CONNECTION;
		((USBD_CDC_NoticationTypeDef *) pBuf)->wIndex = 0;
		((USBD_CDC_NoticationTypeDef *) pBuf)->wLength = 0;
		((USBD_CDC_NoticationTypeDef *) pBuf)->wValue = state;
		hcdc->NtLength = 8;
		USBD_CDC_Notify(&hUsbDeviceFS);
	} else {
		result = USBD_BUSY;
	}
	//USB_EnableGlobalInt(hUsbDeviceFS.pData);
	return result;
	/*
	 static uint8_t notification[12];
	 ((USBD_CDC_NoticationTypeDef *) notification)->bmRequestType = 0b10100001;
	 ((USBD_CDC_NoticationTypeDef *) notification)->bNotification =
	 NETWORK_CONNECTION;
	 ((USBD_CDC_NoticationTypeDef *) notification)->wIndex = 0;
	 ((USBD_CDC_NoticationTypeDef *) notification)->wLength = 0;
	 ((USBD_CDC_NoticationTypeDef *) notification)->wValue = state;
	 return CDC_Notify_FS(notification, sizeof(notification));
	 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
