/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "driver/gpio16.h"
#include "user_interface.h"
#include "mem.h"
#include "espconn.h"
#include "string.h"

#define LIGHT_SWITCH_TOPIC "/home/lightswitch"
#define LIGHT_STATEREQ_TOPIC "/home/lightstatereq"
#define LIGHT_STATEREPLY_TOPIC "/home/lightstatereply"

MQTT_Client mqttClient;
uint8 boardID = 0, toggle = 0;

typedef struct{
	uint8_t s1LastState;
	uint8_t s2LastState;
	uint8_t s3LastState;
	uint8_t checksum;
} SWSTATE;

uint32_t addressOffset = 0;

SWSTATE swState;


void ICACHE_FLASH_ATTR SwitchState_Save();
void ICACHE_FLASH_ATTR SwitchState_Load();


void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, LIGHT_SWITCH_TOPIC, 0);
	MQTT_Subscribe(client, LIGHT_STATEREQ_TOPIC, 1);

	GPIO_OUTPUT_SET(GPIO_ID_PIN(5), 1);

	//MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
	//MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
	//MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);
}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);
	char outputBuf[10];

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	// get board id via dip switch setting
	boardID = GPIO_INPUT_GET(GPIO_ID_PIN(4)) | GPIO_INPUT_GET(GPIO_ID_PIN(2)) << 1;
	if ((dataBuf[0] - 0x30) == boardID){
		INFO("board id: %d\r\n", boardID);
		// toggle led to indicate message received.
		if (toggle)
			toggle--;
		else
			toggle++;
		gpio16_output_set(toggle);

		// check if it is light switching topic
		if (strncmp(topicBuf, LIGHT_SWITCH_TOPIC, 17) == 0){
			// check if it is channel 1
			if (dataBuf[2] == '1'){
				// check switch state
				if (dataBuf[4] == '1'){
					GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 0);
					swState.s1LastState = 0;
				}
				else{
					GPIO_OUTPUT_SET(GPIO_ID_PIN(13), 1);
					swState.s1LastState = 1;
				}
			}
			// check if it is channel 2
			else if (dataBuf[2] == '2'){
				// check switch state
				if (dataBuf[4] == '1'){
					GPIO_OUTPUT_SET(GPIO_ID_PIN(12), 0);
					swState.s2LastState = 0;
				}
				else{
					GPIO_OUTPUT_SET(GPIO_ID_PIN(12), 1);
					swState.s2LastState = 1;
				}
			}
			// check if it is channel 3
			else if (dataBuf[2] == '3'){
				// check switch state
				if (dataBuf[4] == '1'){
					GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 0);
					swState.s3LastState = 0;
				}
				else{
					GPIO_OUTPUT_SET(GPIO_ID_PIN(14), 1);
					swState.s3LastState = 1;
				}
			}
			swState.checksum = swState.s3LastState * 4 + swState.s2LastState * 2 + swState.s1LastState;
			SwitchState_Save();
		}
		// check if it is light state topic
		else if (strncmp(topicBuf, LIGHT_STATEREQ_TOPIC, 19) == 0){
			os_sprintf(outputBuf, "%d,1,%s", boardID, GPIO_INPUT_GET(13) ? "OFF" : "ON");
			MQTT_Publish(client, LIGHT_STATEREPLY_TOPIC, outputBuf, strlen(outputBuf), 0, 0);
			os_sprintf(outputBuf, "%d,2,%s", boardID, GPIO_INPUT_GET(12) ? "OFF" : "ON");
			MQTT_Publish(client, LIGHT_STATEREPLY_TOPIC, outputBuf, strlen(outputBuf), 0, 0);
			os_sprintf(outputBuf, "%d,3,%s", boardID, GPIO_INPUT_GET(14) ? "OFF" : "ON");
			MQTT_Publish(client, LIGHT_STATEREPLY_TOPIC, outputBuf, strlen(outputBuf), 0, 0);
		}
	}
	os_free(topicBuf);
	os_free(dataBuf);
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
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
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


void user_init(void)
{
	uart_div_modify(0, UART_CLK_FREQ / 115200);
	//uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

	CFG_Load();
	SwitchState_Load();

	// led configuration
	gpio16_output_conf();
	gpio16_output_set(0);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(5), 0);

	// dip switch configuration
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(2));
	PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(4));
	PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO4_U);

	// trigger configuration
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);

	// set trigger high - default value
	GPIO_OUTPUT_SET(GPIO_ID_PIN(13), swState.s1LastState);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(12), swState.s2LastState);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(14), swState.s3LastState);

	// get board id via dip switch setting
	boardID = GPIO_INPUT_GET(GPIO_ID_PIN(4)) | GPIO_INPUT_GET(GPIO_ID_PIN(2))<<1;
	INFO("board id: %d\r\n", boardID);

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	//MQTT_InitConnection(&mqttClient, "192.168.11.122", 1880, 0);

	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	//MQTT_InitClient(&mqttClient, "client_id", "user", "pass", 120, 1);

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
}

void ICACHE_FLASH_ATTR
SwitchState_Save()
{
	//spi_flash_erase_sector(CFG_LOCATION + 4);
	if (addressOffset == 4092){
		spi_flash_erase_sector(CFG_LOCATION + 4);
		addressOffset = 0;
	}
	else{
		addressOffset += 4;
		spi_flash_write((CFG_LOCATION + 4) * SPI_FLASH_SEC_SIZE + addressOffset,
			(uint32 *)&swState, sizeof(SWSTATE));
	}
}

void ICACHE_FLASH_ATTR
SwitchState_Load()
{
	INFO("\r\nload switch state ...\r\n");
	while (true){
		spi_flash_read((CFG_LOCATION + 4) * SPI_FLASH_SEC_SIZE + addressOffset,
			(uint32 *)&swState, sizeof(SWSTATE));
		if (swState.checksum == 0xFF)
			break;
		addressOffset += 4;
	}
	addressOffset -= 4;
	INFO("\r\nlast address %d ...\r\n", addressOffset);
	spi_flash_read((CFG_LOCATION + 4) * SPI_FLASH_SEC_SIZE + addressOffset,
		(uint32 *)&swState, sizeof(SWSTATE));
}
