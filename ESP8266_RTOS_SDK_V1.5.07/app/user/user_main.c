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
#include "../app/include/user_json.h"
#include "../app/include/jsonparse.h"
#include "../app/include/jsontree.h"

xSemaphoreHandle wifi_alive;
xQueueHandle publish_queue;
#define PUB_MSG_LEN 16
#define jsonSize   2*1024


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


#define STA_SSID    "ILOVE"
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

LOCAL int ICACHE_FLASH_ATTR
json_get(struct jsontree_context *js_ctx)
{
    const char *path = jsontree_path_name(js_ctx, js_ctx->depth - 1);
    if (os_strncmp(path, "String", 6) == 0) {
        jsontree_write_string(js_ctx, fifo_tmp );
    }
    return 0;
}

LOCAL struct jsontree_callback switch_callback =
    JSONTREE_CALLBACK(json_get, 0);
JSONTREE_OBJECT(switch_tree,
                JSONTREE_PAIR("String", &switch_callback));
JSONTREE_OBJECT(device_tree,
				JSONTREE_PAIR("device", &switch_tree));

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
        dmsg_puts("beat\r\n");
        snprintf(msg, PUB_MSG_LEN, "Beat %d\r\n", count++);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE)
        {
            dmsg_puts("Publish queue overflow.\r\n");
        }
    }
}


// Callback when receiving subscribed message
LOCAL void ICACHE_FLASH_ATTR topic_received(MessageData* md)
{
	dmsg_puts("555555\r\n");
    int i;
    MQTTMessage* message = md->message;
    dmsg_puts("Received: ");
    for (i = 0; i < md->topic->lenstring.len; ++i)
        dmsg_putchar(md->topic->lenstring.data[i]);
    dmsg_puts(" = ");
    for (i = 0; i < (int)message->payloadlen; ++i)
        dmsg_putchar(((char*)message->payload)[i]);
    dmsg_puts("\r\n");
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
    unsigned char mqtt_buf[1500];
    unsigned char mqtt_readbuf[1500];
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

    NewNetwork(&network);
    while (1)
    {
        // Wait until wifi is up
        xSemaphoreTake(wifi_alive, portMAX_DELAY);

        // Unique client ID
        strcpy(mqtt_client_id, "ESP-");
        strcat(mqtt_client_id, get_my_id());

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
                MQTTSubscribe(&client, "/mytopic", QOS1, topic_received);
                // Empty the publish queue
                xQueueReset(publish_queue);
                while (1)
                {
                    // Publish all pending messages
                    char msg[PUB_MSG_LEN];
                    while (xQueueReceive(publish_queue, (void *)msg, 0) == pdTRUE)
                    {
                        msg[PUB_MSG_LEN - 1] = '\0';
                        MQTTMessage message;
                        message.payload = msg;
                        message.payloadlen = PUB_MSG_LEN;
                        message.dup = 0;
                        message.qos = QOS1;
                        message.retained = 0;
                        ret = MQTTPublish(&client, "beatr", &message);
                        if (ret != SUCCESS)
                            break;
                    }
                    if(x == 1)
                    {
                    	x = 0;
                       	// Send new status to the MQTT broker
						char *json_buf = NULL;
						json_buf = (char *)os_zalloc(jsonSize);
						json_ws_send((struct jsontree_value *)&device_tree, "device", json_buf);
					//	MQTT_Publish(&mqttClient, "/mytopic", json_buf, strlen(json_buf), 0, 0);
				      //msg[strlen(fifo_tmp) - 1] = '\0';
                        MQTTMessage message;
                        message.payload = json_buf;
                        message.payloadlen = strlen(json_buf);
                        message.dup = 0;
                        message.qos = QOS1;
                        message.retained = 0;
                        ret = MQTTPublish(&client, "/mytopic", &message);
                        if (ret != SUCCESS)
                            break;
                		os_free(json_buf);
                		json_buf = NULL;
                        memset(fifo_tmp,0,1280);
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
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);
    xSemaphoreTake(wifi_alive, 0);  // take the default semaphore
    xTaskCreate(beat_task, "beat", 256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(mqtt_task, "mqtt", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(wifi_task, "wifi", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
}

