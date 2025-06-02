/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define APP_RX_DATA_SIZE	2048
#define APP_TX_DATA_SIZE	2048

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA	0x01
#define TX_NEW_TRANSMITTED_DATA	0x02
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

/* Data to send over USB CDC are stored in this buffer   */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

extern TX_EVENT_FLAGS_GROUP EventFlag;

// rusikok for speed test
//#define BUF_LEN		(8192 / 32)
#define BUF_LEN		(4200)
uint8_t buf[BUF_LEN];
uint32_t tickstart;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
	cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*)cdc_acm_instance; // Save the CDC instance
	
	// rusikok for speed test
	for(uint32_t i = 0; i < BUF_LEN; i++)
		buf[i] = 'u';
	
	buf[0] = 'V';
	buf[1] = 'I';
	buf[2] = 'Y';
	buf[BUF_LEN - 2] = '\r';
	buf[BUF_LEN - 1] = '\n';
	//HAL_Delay(30000);
	LED_ON();
	tickstart = HAL_GetTick();
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);
	cdc_acm = UX_NULL; // Reset the cdc acm instance
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
	UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID usbx_cdc_acm_read_thread_entry(ULONG thread_input)
{
  ULONG actual_length;
  ULONG senddataflag = 0;
  UX_SLAVE_DEVICE *device;

  UX_PARAMETER_NOT_USED(thread_input);

  device = &_ux_system_slave->ux_system_slave_device;

  while (1)
  {
    /* Check if device is configured */
    if((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
    {
#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
      /* Set transmission_status to UX_FALSE for the first time */
      cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
#endif /* UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE */

      

      
      /* Чтение полученных данных в режиме блокировки */
      ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBufferFS, 64, &actual_length);
      if(0 != actual_length)
      {
        // обработка данных
	      
	      
	      
	      
      }
      else
      {
        /*Усыпить поток на 10 мс, если данные не получены */
        tx_thread_sleep(MS_TO_TICK(10));
      }
    }
    else
    {
      /* Sleep thread for 10ms */
      tx_thread_sleep(MS_TO_TICK(10));
    }
  }
}

/**
  * @brief  Function implementing usbx_cdc_acm_write_thread_entry.
  * @param  thread_input: Not used
  * @retval none
  */
VOID usbx_cdc_acm_write_thread_entry(ULONG thread_input)
{		
	uint32_t i = 0;
	while(1)
	{
		// rusikok
		ULONG actual_len;
		/*uint8_t buf[] = {'v','i','y','5','\0'};
		ux_device_class_cdc_acm_write(cdc_acm, buf, strlen(buf), &actual_len);
		tx_thread_sleep(MS_TO_TICK(500));*/
		
		sprintf(&buf[3], "_%u_", i++);
		ux_device_class_cdc_acm_write(cdc_acm, buf, BUF_LEN, &actual_len); // отправляем в USB полученные данные
		/*if((HAL_GetTick() - tickstart) > 60000)
		{
			LED_OFF();
			while(1);
		}*/
	}
		
//----------------------------------------------------------------------------------------------------
  ULONG receivedataflag = 0;
  ULONG actual_length, buffptr, buffsize;

  UX_PARAMETER_NOT_USED(thread_input);

  while (1)
  {
    /* Wait until the requested flag RX_NEW_RECEIVED_DATA is received */
    if(tx_event_flags_get(&EventFlag, RX_NEW_RECEIVED_DATA, TX_OR_CLEAR, &receivedataflag, TX_WAIT_FOREVER) != TX_SUCCESS)
      Error_Handler();

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
    /* Set transmission_status to UX_FALSE for the first time */
    cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
#endif

    /* Check if there is a new data to send */
    if (UserTxBufPtrOut != UserTxBufPtrIn)
    {
      /* Check buffer overflow and Rollback */
      if (UserTxBufPtrOut > UserTxBufPtrIn)
      {
        buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
      }
      else
      {
        /* Calculate data size */
        buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
      }

      /* Copy UserTxBufPtrOut in buffptr */
      buffptr = UserTxBufPtrOut;


	/* Передача данных по классу cdc_acm_write */
      if(ux_device_class_cdc_acm_write(cdc_acm, (UCHAR *)(&UserTxBufferFS[buffptr]), buffsize, &actual_length) == UX_SUCCESS)
      {
        /* Increment the UserTxBufPtrOut pointer */
        UserTxBufPtrOut += buffsize;

        /* Откат UserTxBufPtrOut, если он равен APP_TX_DATA_SIZE*/
        if (UserTxBufPtrOut == APP_TX_DATA_SIZE)
          UserTxBufPtrOut = 0;
      }
    }
  }
}
/* USER CODE END 1 */
