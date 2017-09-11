/*
 * client.h
 *
 *  Created on: 2016年11月12日
 *      Author: Administrator
 */

#ifndef APP_INCLUDE_CLIENT_H_
#define APP_INCLUDE_CLIENT_H_
#include "esp_common.h"
#include "../include/espressif/espconn.h"
struct espconn user_tcp_conn;
void my_station_init(struct ip_addr *remote_ip,struct ip_addr *local_ip,int remote_port);
//定义数据类型
void ICACHE_FLASH_ATTR Wifi_conned(void *arg);
void ICACHE_FLASH_ATTR scan_done(void *arg,STATUS status);
void to_scan(void);
#endif /* APP_INCLUDE_CLIENT_H_ */
