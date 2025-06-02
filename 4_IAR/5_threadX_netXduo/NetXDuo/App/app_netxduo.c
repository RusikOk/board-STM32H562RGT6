/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.c
  * @author  MCD Application Team
  * @brief   NetXDuo applicative file
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
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
#include "nxd_dhcp_client.h"
/* USER CODE BEGIN Includes */
//#include "ux_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Define Threadx global data structures. */
TX_THREAD AppTCPThread;
TX_THREAD AppLinkThread;
/* Define NetX global data structures. */
ULONG IpAddress;
ULONG NetMask;
NX_TCP_SOCKET TCPSocket;
/* App memory pointer. */
CHAR *pointer;

// тест скорости отправки
#define BUF_LEN		(4200)
uint8_t buf[BUF_LEN] = {0, };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
TX_SEMAPHORE   DHCPSemaphore;
NX_DHCP        DHCPClient;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry (ULONG thread_input);
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);
/* USER CODE BEGIN PFP */
static VOID viy5_tcp_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port);
static VOID viy5_tcp_server_thread_entry(ULONG thread_input); /* TCP thread entry */
static VOID viy5_link_thread_entry(ULONG thread_input); /* Link thread entry */
/* USER CODE END PFP */

/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_NetXDuo_MEM_POOL */
  (void)byte_pool;
  /* USER CODE END App_NetXDuo_MEM_POOL */
  /* USER CODE BEGIN 0 */
  printf("viy5_tcp_server application started..\r\n");
  /* USER CODE END 0 */

  /* Initialize the NetXDuo system. */
  CHAR *pointer;
  nx_system_initialize();

    /* Allocate the memory for packet_pool.  */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the Packet pool to be used for packet allocation,
   * If extra NX_PACKET are to be used the NX_APP_PACKET_POOL_SIZE should be increased
   */
  ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_POOL_ERROR;
  }

    /* Allocate the memory for Ip_Instance */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

   /* Create the main NX_IP instance */
  ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
                     pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

    /* Allocate the memory for ARP */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Enable the ARP protocol and provide the ARP cache size for the IP instance */

  /* USER CODE BEGIN ARP_Protocol_Initialization */

  /* USER CODE END ARP_Protocol_Initialization */

  ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the ICMP */

  /* USER CODE BEGIN ICMP_Protocol_Initialization */

  /* USER CODE END ICMP_Protocol_Initialization */

  ret = nx_icmp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable TCP Protocol */

  /* USER CODE BEGIN TCP_Protocol_Initialization */

  /* USER CODE END TCP_Protocol_Initialization */

  ret = nx_tcp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

  /* Enable the UDP protocol required for  DHCP communication */

  /* USER CODE BEGIN UDP_Protocol_Initialization */

  /* USER CODE END UDP_Protocol_Initialization */

  ret = nx_udp_enable(&NetXDuoEthIpInstance);

  if (ret != NX_SUCCESS)
  {
    return NX_NOT_SUCCESSFUL;
  }

   /* Allocate the memory for main thread   */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  /* Create the main thread */
  ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", nx_app_thread_entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
                         NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

  if (ret != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* Create the DHCP client */

  /* USER CODE BEGIN DHCP_Protocol_Initialization */

  /* USER CODE END DHCP_Protocol_Initialization */

  ret = nx_dhcp_create(&DHCPClient, &NetXDuoEthIpInstance, "DHCP Client");

  if (ret != NX_SUCCESS)
  {
    return NX_DHCP_ERROR;
  }

  /* set DHCP notification callback  */
  tx_semaphore_create(&DHCPSemaphore, "DHCP Semaphore", 0);

  /* USER CODE BEGIN MX_NetXDuo_Init */
	// Allocate the memory for TCP server thread
	if(tx_byte_allocate(byte_pool, (VOID **) &pointer,2 *  DEFAULT_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
		return TX_POOL_ERROR;

	// create the TCP server thread
	ret = tx_thread_create(&AppTCPThread, "viy5 tcp server thread", viy5_tcp_server_thread_entry, 0, pointer, 2 * DEFAULT_MEMORY_SIZE, DEFAULT_PRIORITY, DEFAULT_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);
	if(TX_SUCCESS != ret)
		return TX_THREAD_ERROR;

	// Allocate the memory for Link thread
	if(TX_SUCCESS != tx_byte_allocate(byte_pool, (VOID **) &pointer, 2 *  DEFAULT_MEMORY_SIZE, TX_NO_WAIT))
		return TX_POOL_ERROR;

	// create the Link thread
	ret = tx_thread_create(&AppLinkThread, "viy5 link thread", viy5_link_thread_entry, 0, pointer, 2 * DEFAULT_MEMORY_SIZE, LINK_PRIORITY, LINK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
	if(TX_SUCCESS != ret)
		return TX_THREAD_ERROR;
  /* USER CODE END MX_NetXDuo_Init */

  return ret;
}

/**
* @brief ip address change callback.
* @param ip_instance: NX_IP instance
* @param ptr: user data
* @retval none
*/
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr)
{
  /* USER CODE BEGIN ip_address_change_notify_callback */
	nx_ip_address_get(ip_instance, &IpAddress, &NetMask);
	PRINT_IP_ADDRESS(IpAddress);
	PRINT_IP_ADDRESS(NetMask);
	/*PRINT_IP_ADDRESS(ip_instance->nx_ip_interface[0].nx_interface_ip_address);
	PRINT_IP_ADDRESS(ip_instance->nx_ip_interface[0].nx_interface_ip_network_mask);
	PRINT_IP_ADDRESS(ip_instance->nx_ip_interface[0].nx_interface_ip_network);*/
  /* USER CODE END ip_address_change_notify_callback */

  /* release the semaphore as soon as an IP address is available */
  tx_semaphore_put(&DHCPSemaphore);
}

/**
* @brief  Main thread entry.
* @param thread_input: ULONG user argument used by the thread entry
* @retval none
*/
static VOID nx_app_thread_entry (ULONG thread_input)
{
  /* USER CODE BEGIN Nx_App_Thread_Entry 0 */

  /* USER CODE END Nx_App_Thread_Entry 0 */

  UINT ret = NX_SUCCESS;

  /* USER CODE BEGIN Nx_App_Thread_Entry 1 */

  /* USER CODE END Nx_App_Thread_Entry 1 */

  /* register the IP address change callback */
  ret = nx_ip_address_change_notify(&NetXDuoEthIpInstance, ip_address_change_notify_callback, NULL);
  if (ret != NX_SUCCESS)
  {
    /* USER CODE BEGIN IP address change callback error */

    /* USER CODE END IP address change callback error */
  }

  /* start the DHCP client */
  ret = nx_dhcp_start(&DHCPClient);
  if (ret != NX_SUCCESS)
  {
    /* USER CODE BEGIN DHCP client start error */

    /* USER CODE END DHCP client start error */
  }

  /* wait until an IP address is ready */
  if(tx_semaphore_get(&DHCPSemaphore, NX_APP_DEFAULT_TIMEOUT) != TX_SUCCESS)
  {
    /* USER CODE BEGIN DHCPSemaphore get error */

    /* USER CODE END DHCPSemaphore get error */
  }

  /* USER CODE BEGIN Nx_App_Thread_Entry 2 */
	// the network is correctly initialized, start the TCP server thread
	tx_thread_resume(&AppTCPThread);

	// this thread is not needed any more, we relinquish it
	tx_thread_relinquish();
  /* USER CODE END Nx_App_Thread_Entry 2 */

}
/* USER CODE BEGIN 1 */
/**
* @brief TCP socket depth call back
* @param socket_ptr: NX_TCP_SOCKET socket registered for the callback
* @retval none
*/
static VOID viy5_tcp_socket_queue_depth_callback(NX_TCP_SOCKET *socket_ptr)
{
	// This service sets the transmit queue depth update notify function specified by the application, which iscalled whenever the specified socket determines that it has released packets from the transmit queue such that the queue depth is no longer exceeding its limit. If an application would be blocked on transmit due to queue depth,the callback function serves as a notification to the application that itmay start transmitting again. This service is available only if the NetXDuo library is built with the option NX_ENABLE_TCP_QUEUE_DEPTH_UPDATE_NOTIFYdefined.
	// TODO: понять, что с этим делать
	printf("\r\n\r\n\t static VOID viy5_tcp_socket_queue_depth_callback(NX_TCP_SOCKET *socket_ptr) \r\n");
}

/**
* @brief TCP listen call back
* @param socket_ptr: NX_TCP_SOCKET socket registered for the callback
* @param port: UINT  the port on which the socket is listening
* @retval none
*/
static VOID viy5_tcp_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port)
{
	ULONG ClientIpAddress = socket_ptr->nx_tcp_socket_connect_ip.nxd_ip_address.v4;
	printf("client connected: \r\n\t");
	PRINT_IP_ADDRESS(ClientIpAddress);
	printf("\tClientPort: %u \r\n", socket_ptr->nx_tcp_socket_connect_port);
	
	// as soon as the IP address is ready, the semaphore is released // как только IP-адрес будет готов, семафор будет освобожден
	tx_semaphore_put(&DHCPSemaphore);
}

/**
* @brief TCP server thread entry
* @param thread_input: ULONG thread parameter
* @retval none
*/
static VOID viy5_tcp_server_thread_entry(ULONG thread_input)
{
	UINT ret;
	UCHAR data_buffer[512];
	NX_PACKET *data_packet;
	ULONG bytes_read;

	// create the TCP socket // создаем TCP-сокет
	ret = nx_tcp_socket_create(&NetXDuoEthIpInstance, &TCPSocket, "VIY5 TCP Server Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, WINDOW_SIZE, NX_NULL, NX_NULL);
	if(ret)	Error_Handler();

	// устанавливаем калбек, для отслеживания конца памяти tcp. но это не точно
	ret = nx_tcp_socket_queue_depth_notify_set(&TCPSocket, viy5_tcp_socket_queue_depth_callback);
	if(ret)	Error_Handler();

	// listen to new client connections // прослушивать новые клиентские соединения.
	// the TCP_listen_callback will release the 'Semaphore' when a new connection is available // обратный вызов TCP_listen_callback освободит "семафор", когда будет доступно новое соединение
	ret = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, DEFAULT_PORT, &TCPSocket, MAX_TCP_CLIENTS, viy5_tcp_listen_callback);
	if(ret)
	{
		Error_Handler();
	}
	else
	{
		printf("TCP Server listening on PORT %d..\r\n", DEFAULT_PORT);

/*		// формируем кадр
		for(uint32_t i = 0; i < BUF_LEN; i++)
				buf[i] = 's';
		buf[0] = 'V';
		buf[1] = 'I';
		buf[2] = 'Y';
		buf[BUF_LEN - 2] = '\r';
		buf[BUF_LEN - 1] = '\n';*/
	}

	printf("\t tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER); \r\n");
	tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER);

	while(1)
	{
		ULONG socket_state;

		TX_MEMSET(data_buffer, '\0', sizeof(data_buffer));

		// get the socket state	// получить состояние сокета
		nx_tcp_socket_info_get(&TCPSocket, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &socket_state, NULL, NULL, NULL);

		// if the connections is not established then accept new ones, otherwise start receiving data
		// если соединения не установлены, то принимаем новые, иначе начинаем получать данные
		if(NX_TCP_ESTABLISHED != socket_state)
		{
			// примите новое клиентское соединение перед началом обмена данными
			ret = nx_tcp_server_socket_accept(&TCPSocket, TX_WAIT_FOREVER);

			// отправляем hello string
			nx_packet_allocate(&NxAppPool, &data_packet, NX_TCP_PACKET, NX_WAIT_FOREVER);
			nx_packet_data_append(data_packet, "VIY5PTR", sizeof("VIY5PTR"), &NxAppPool, NX_WAIT_FOREVER);
			nx_tcp_socket_send(&TCPSocket, data_packet, NX_WAIT_FOREVER);
			nx_packet_release(data_packet);
			
			// лог в лог)
			printf("send hello string\r\n");
		}

		uint32_t i = 0;
		if(NX_SUCCESS == ret)
		{
			// receive the TCP packet send by the client // получаем TCP-пакет, отправленный клиентом
			ret = nx_tcp_socket_receive(&TCPSocket, &data_packet, NX_WAIT_FOREVER);
			//printf("\t ret = nx_tcp_socket_receive(&TCPSocket, &data_packet, NX_WAIT_FOREVER); \r\n");
			if(NX_SUCCESS == ret)
			{
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // моргалка после отправки пакета
				
				// получаем данные от клиента
				nx_packet_data_retrieve(data_packet, data_buffer, &bytes_read);

				// print the received data // выведите полученные данные
				printf("DATA: %s \r\n", data_buffer);

				// отправляем данные клиенту
				nx_tcp_socket_send(&TCPSocket, data_packet, NX_IP_PERIODIC_RATE);
				
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // моргалка после отправки пакета
			}
			else
			{
				nx_tcp_socket_disconnect(&TCPSocket, NX_WAIT_FOREVER);
				nx_tcp_server_socket_unaccept(&TCPSocket);
				nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance, DEFAULT_PORT, &TCPSocket);
				printf("\t nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance, DEFAULT_PORT, &TCPSocket); \r\n");
			}
			












/*			// тест скорости блока отправки данных в 4кБ составляет ~6МБт/сек
			// максимальный размер буфера для двухканального радара составляет 4185Б, что на 89Б бойльше 4096
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); // моргалка после отправки пакета
			uint32_t tickstart = HAL_GetTick();

			while((HAL_GetTick() - tickstart) < 60000)
			{
				UINT ret;
				NX_PACKET *send_packet;

				// отправляем данные
				ret = nx_packet_allocate(&NxAppPool, &send_packet, NX_TCP_PACKET, NX_WAIT_FOREVER);
				if(NX_SUCCESS == ret)
				{
					sprintf(&buf[3], "_%u_", i);

					ret = nx_packet_data_append(send_packet, &buf, BUF_LEN, &NxAppPool, NX_WAIT_FOREVER);
					if(NX_SUCCESS == ret)
					{
						ret = nx_tcp_socket_send(&TCPSocket, send_packet, NX_WAIT_FOREVER);
						if(ret) nx_packet_release(send_packet);

						//printf("send block # %u \r\n", i);
						i++;

						//tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND * 10) / 1000); // 10ms
					}
				}
			}

			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // моргалка после отправки пакета
			printf("END TRANSFER DATA \r\n");
			//while(1);

			nx_tcp_socket_disconnect(&TCPSocket, NX_WAIT_FOREVER);
			nx_tcp_server_socket_unaccept(&TCPSocket);
			nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance, DEFAULT_PORT, &TCPSocket);*/
		}
	}
}

/**
* @brief Link thread entry
* @param thread_input: ULONG thread parameter
* @retval none
*/
static VOID viy5_link_thread_entry(ULONG thread_input)
{
	ULONG actual_status;
	UINT linkdown = 0, status;

	while(1)
	{
		// Get Physical Link stackavailtus
		status = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, NX_IP_LINK_ENABLED, &actual_status, 10);

		if(NX_SUCCESS == status)
		{
			if(1 == linkdown)
			{
				linkdown = 0;
				status = nx_ip_interface_status_check(&NetXDuoEthIpInstance, 0, NX_IP_ADDRESS_RESOLVED, &actual_status, 10);

				if(NX_SUCCESS == status)
				{
					// The network cable is connected again
					printf("The network cable is connected again.\r\n");
					// Print TCP Echo Server is available again
					printf("TCP Echo Server is available again.\r\n");
				}
				else
				{
					// The network cable is connected
					printf("The network cable is connected.\r\n");
					// Send command to Enable Nx driver
					nx_ip_driver_direct_command(&NetXDuoEthIpInstance, NX_LINK_ENABLE, &actual_status);
					// Restart DHCP Client
					nx_dhcp_stop(&DHCPClient);
					nx_dhcp_start(&DHCPClient);
				}
			}
		}
		else
		{
			if(0 == linkdown)
			{
				linkdown = 1;
				// The network cable is not connected
				printf("The network cable is not connected.\r\n");
			}
		}

		tx_thread_sleep(NX_ETH_CABLE_CON_CHECK_PERIOD);
	}
}
/* USER CODE END 1 */
