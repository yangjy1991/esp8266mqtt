/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"

#include "uart.h"
#include "dmsg.h"
#include "xmit.h"

#include "../app/paho/MQTTESP8266.h"
#include "../app/paho/MQTTClient.h"
#include "cJSON.h"

xSemaphoreHandle wifi_alive;
xQueueHandle publish_queue;
xQueueHandle CmdRxHandlTaskQueueHandle;

#define PUB_MSG_LEN 16
uint8 Rxueibuf[1500];

LOCAL const char * ICACHE_FLASH_ATTR get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!wifi_get_macaddr(STATION_IF, my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

#if 0   // Some debug stuff

LOCAL void ICACHE_FLASH_ATTR dbgpin_init(void)
{
    // Debug on Pin 5
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | BIT5);
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, BIT5);
}

#define DBG1 GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, BIT5)
#define DBG0 GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, BIT5)

#endif


#define STA_SSID    "Wulian_101E1D"
#define STA_PASSWORD  "yjy123456"


LOCAL void ICACHE_FLASH_ATTR wifi_task(void *pvParameters)
{
    uint8_t status;

    if (wifi_get_opmode() != STATION_MODE)
    {
        wifi_set_opmode(STATION_MODE);
        vTaskDelay(1000 / portTICK_RATE_MS);
        system_restart();
    }

    while (1)
    {
        dmsg_puts("WiFi: Connecting to WiFi\n");
        wifi_station_connect();
        struct station_config *config = (struct station_config *)zalloc(sizeof(struct station_config));
        sprintf(config->ssid, STA_SSID);
        sprintf(config->password, STA_PASSWORD);
        wifi_station_set_config(config);
        free(config);
        status = wifi_station_get_connect_status();
        int8_t retries = 30;
        while ((status != STATION_GOT_IP) && (retries > 0))
        {
            status = wifi_station_get_connect_status();
            if (status == STATION_WRONG_PASSWORD)
            {
                dmsg_puts("WiFi: Wrong password\n");
                break;
            }
            else if (status == STATION_NO_AP_FOUND)
            {
                dmsg_puts("WiFi: AP not found\n");
                break;
            }
            else if (status == STATION_CONNECT_FAIL)
            {
                dmsg_puts("WiFi: Connection failed\n");
                break;
            }
            vTaskDelay(1000 / portTICK_RATE_MS);
            --retries;
        }
        if (status == STATION_GOT_IP)
        {
            dmsg_puts("WiFi: Connected\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        while ((status = wifi_station_get_connect_status()) == STATION_GOT_IP)
        {
            xSemaphoreGive(wifi_alive);
            // dmsg_puts("WiFi: Alive\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        dmsg_puts("WiFi: Disconnected\n");
        wifi_station_disconnect();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


/*
 * lpSrcStr  : 要转换的字符串，如 “123456AbCdEf”。
 * lpRetBytes: 要存放转换结果的字节数组。
 * lpRetSize : lpRetBytes 数组的元素个数
 * 返回：写入 lpRetBytes 的元素个数
 */
static void StrToHex(const char lpSrcStr[], unsigned char lpRetBytes[], size_t *lpRetSize)
{
    if (lpSrcStr != NULL && lpRetBytes != NULL && lpRetSize != NULL)
    {
        size_t uiLength = strlen(lpSrcStr);
        if (uiLength % 2 == 0)
        {
            size_t i = 0;
            size_t n = 0;

             while (*lpSrcStr != 0 && (n = ((i++) >> 1)) < *lpRetSize)
             {
                lpRetBytes[n] <<= 4;
               if (*lpSrcStr >= ('0') && *lpSrcStr <= ('9'))
                {
                    lpRetBytes[n] |= *lpSrcStr - '0';
                }
                else if (*lpSrcStr >= ('a') && *lpSrcStr <= ('f'))
                {
                    lpRetBytes[n] |= *lpSrcStr - 'a' + 10;
                }
                else if (*lpSrcStr >= ('A') && *lpSrcStr <= ('F'))
                {
                    lpRetBytes[n] |= *lpSrcStr - 'A' + 10;
                }
                lpSrcStr++;
            }
            *lpRetSize = n;
        }
    }
}

/*
// C prototype : void HexToStr(BYTE *pbDest, BYTE *pbSrc, int nLen)
// parameter(s): [OUT] pbDest - 存放目标字符串
//	[IN] pbSrc - 输入16进制数的起始地址
//	[IN] nLen - 16进制数的字节数
// return value:
// remarks : 将16进制数转化为字符串
*/
static void HexToStr(unsigned char *pbDest, unsigned char *pbSrc, int nLen)
{
	char    ddl,ddh;
	int i;

	for (i=0; i<nLen; i++)
	{
		ddh = 48 + pbSrc[i] / 16;
		ddl = 48 + pbSrc[i] % 16;
		if (ddh > 57) ddh = ddh + 7;
		if (ddl > 57) ddl = ddl + 7;
		pbDest[i*2] = ddh;
		pbDest[i*2+1] = ddl;
	}

	pbDest[nLen*2] = '\0';
}

/* Demonstrating sending something to MQTT broker
   In this task we simply queue up messages in publish_queue. The MQTT task will dequeue the
   message and sent.
 */
LOCAL void ICACHE_FLASH_ATTR beat_task(void * pvParameters)
{
    portTickType xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];
    int count = 0;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, 10000 / portTICK_RATE_MS); // This is executed every 10000ms
        snprintf(msg, PUB_MSG_LEN, "Beat %d", count++);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE)
        {
            dmsg_puts("Publish queue overflow.\r\n");
        }
    }
}


// Callback when receiving subscribed message
LOCAL void ICACHE_FLASH_ATTR topic_received(MessageData* md)
{
    int i;
	uint8 retr[1500];
	size_t size = sizeof(retr);
    MQTTMessage* message = md->message;
    char *dataBuf = (char*)os_zalloc((int)message->payloadlen+1);
	memcpy(dataBuf, (char*)message->payload, (int)message->payloadlen);
	dataBuf[(int)message->payloadlen] = 0;

	cJSON *root = cJSON_Parse( dataBuf );
	cJSON *child  = cJSON_GetObjectItem( root, "String" );
    if( child->type == cJSON_String )
    {
        StrToHex(child->valuestring, retr, &size);

		for (i = 0; i < size+1; i++)
		{
			uart_tx_one_char(UART0, retr[i]);
		}
    }
    // 释放内存空间
    os_free(dataBuf);
    cJSON_Delete(root);
}


// testing mosquitto server
#define MQTT_HOST "172.18.2.159"
#define MQTT_PORT 1883
#define MQTT_USER "admin"
#define MQTT_PASS "password"


LOCAL void ICACHE_FLASH_ATTR mqtt_task(void *pvParameters)
{
	dmsg_printf("mqtt_start\n");
    int ret;
    struct Network network;
    MQTTClient client = DefaultClient;
    char mqtt_client_id[20];
    char mqtt_client_topic[30];
    unsigned char mqtt_buf[1500];
    unsigned char mqtt_readbuf[1500];
    uint8 retrs[1500];
    //uint8 Rxbuf[14];
	long xTaskWokenByReceive=pdFALSE;
	long err;
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

    NewNetwork(&network);
    while (1)
    {
        // Wait until wifi is up
        xSemaphoreTake(wifi_alive, portMAX_DELAY);

        // Unique client ID
        strcpy(mqtt_client_id, "ESP-");
        strcat(mqtt_client_id, get_my_id());
        strcpy(mqtt_client_topic, "/wulian/");
        strcat(mqtt_client_topic, get_my_id());
        strcat(mqtt_client_topic, "/beatr");
        printf("mqtt_client_id:%s\r\n" , mqtt_client_id );
        dmsg_printf("(Re)connecting to MQTT server %s ... ", MQTT_HOST);
        ret = ConnectNetwork(&network, MQTT_HOST, MQTT_PORT);
        if (!ret)
        {
            dmsg_puts("ok.\r\n");
            NewMQTTClient(&client, &network, 5000, mqtt_buf, 1500, mqtt_readbuf, 1500);
            data.willFlag = 0;
            data.MQTTVersion = 3;
            data.clientID.cstring = mqtt_client_id;
            data.username.cstring = MQTT_USER;
            data.password.cstring = MQTT_PASS;
            data.keepAliveInterval = 10;
            data.cleansession = 0;
            dmsg_puts("Send MQTT connect ...");
            ret = MQTTConnect(&client, &data);
            if (!ret)
            {
                dmsg_puts("ok.\r\n");
                // Subscriptions
                MQTTSubscribe(&client, "/mytopic1", QOS1, topic_received);
                // Empty the publish queue
                xQueueReset(publish_queue);
                while (1)
                {
                    // Publish all pending messages
                    char msg[PUB_MSG_LEN];
                    while (xQueueReceive(publish_queue, (void *)msg, 0) == pdTRUE)
                    {
                    	cJSON * Result = cJSON_CreateObject( );                          // Create JSON Object.
                    	cJSON_AddNumberToObject( Result , "status" , 123456 );
                    	cJSON_AddStringToObject( Result , "msg" , msg);
                    	char *Body =  cJSON_Print( Result );
                    	cJSON_Delete( Result );

                      //  msg[PUB_MSG_LEN - 1] = '\0';
                        MQTTMessage message;
                        message.payload = Body;
                        message.payloadlen = strlen(Body);
                        message.dup = 0;
                        message.qos = QOS1;
                        message.retained = 0;
                        ret = MQTTPublish(&client, mqtt_client_topic, &message);
                        if (ret != SUCCESS)
                            break;
                        free( Body );
                    }
                	err=xQueueReceiveFromISR(CmdRxHandlTaskQueueHandle, Rxueibuf, &xTaskWokenByReceive);//请求消息Message_Queue
        			if(err==pdTRUE)			//接收到消息
        			{
                    	HexToStr(retrs, Rxueibuf, UartRxCnt);

                        //msg[strlen(fifo_tmp) - 1] = '\0';
                    	cJSON * Result = cJSON_CreateObject( );                          // Create JSON Object.
                    	cJSON_AddNumberToObject( Result , "status" , 12345678 );
                    	cJSON_AddStringToObject( Result , "msg" , retrs);
                    	//cJSON_AddStringToObject( Result , "String" , retrs);
                    	char *Body =  cJSON_Print( Result );
                    	cJSON_Delete( Result );

                        MQTTMessage message;
                        message.payload = Body;
                        message.payloadlen = strlen(Body);
                        message.dup = 0;
                        message.qos = QOS1;
                        message.retained = 0;
                        ret = MQTTPublish(&client, mqtt_client_topic, &message);
                        //ret = MQTTPublish(&client, "/mytopic", &message);
                        if (ret != SUCCESS)
                            break;
        				memset(Rxueibuf,0,1500);
                        free( Body );
                    }
                    // Receiving / Ping
                    ret = MQTTYield(&client, 2000);
                    if (ret == DISCONNECTED)
                    {
                        break;
                    }
                }
                dmsg_puts("Connection broken, request restart\r\n");
            }
            else
            {
                dmsg_puts("failed.\r\n");
            }
            DisconnectNetwork(&network);
        }
        else
        {
            dmsg_puts("failed.\r\n");
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    dmsg_printf("MQTT task ended\r\n", ret);
    vTaskDelete(NULL);
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR user_init(void)
{
    int ret;
    MQTTClient client = DefaultClient;
    dmsg_init();
    uart_init_new();
    dmsg_puts("\r\n\r\n");
    vSemaphoreCreateBinary(wifi_alive);
    CmdRxHandlTaskQueueHandle = xQueueCreate( 3 , 1280 );
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);
    xSemaphoreTake(wifi_alive, 0);  // take the default semaphore
    xTaskCreate(beat_task, "beat", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(mqtt_task, "mqtt", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(wifi_task, "wifi", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
}

