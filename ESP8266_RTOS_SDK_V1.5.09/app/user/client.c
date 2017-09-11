
#include "client.h"
#define os_memcpy memcpy
#define os_memset memset
#define os_strlen strlen
struct espconn user_tcp_conn;
LOCAL os_timer_t connect_timer;

void my_station_init(struct ip_addr *remote_ip,struct ip_addr *local_ip,int remote_port);
//定义数据类型

void ICACHE_FLASH_ATTR user_tcp_sent_cb(void *arg)//定义发送
{
	os_printf("发送数据成功！");
}

void ICACHE_FLASH_ATTR user_tcp_discon_cb(void *arg)//定义接收
{
	os_printf("断开连接成功！");
}

void ICACHE_FLASH_ATTR user_tcp_recv_cb(void *arg,
		char *pdata,
		unsigned short len){
	os_printf("收到数据：%s\r\n",pdata);
	os_delay_us(300);//延时
	espconn_disconnect((struct espconn *)arg);//状态连接

}

void ICACHE_FLASH_ATTR user_tcp_recon_cb(void *arg, sint8 err)//重新连接回调函数
{
	os_printf("连接错误，错误代码为%d\r\n",err);
	espconn_connect((struct espconn *)arg);
}

void ICACHE_FLASH_ATTR user_tcp_connect_cb(void *arg)
{
	struct espconn *pespconn=arg;
	espconn_regist_recvcb(pespconn,user_tcp_recv_cb);//接受成功回调
	 espconn_regist_sentcb(pespconn,user_tcp_sent_cb);//发送成功 回调
	 espconn_regist_disconcb(pespconn,user_tcp_discon_cb);//重连回调
	 espconn_sent(pespconn,"这是esp8266",strlen("这是esp8266"));
}

void ICACHE_FLASH_ATTR my_station_init(struct ip_addr *remote_ip,struct ip_addr *local_ip,int remote_port){
	//espconn参数配置 ，定义数据类型
	user_tcp_conn.type=ESPCONN_TCP;//类型tcp
	user_tcp_conn.state=ESPCONN_NONE;//状态
	user_tcp_conn.proto.tcp=(esp_tcp *)os_zalloc(sizeof(esp_tcp));//共同体分配内存空间，os_zalloc在mem.h里面
	os_memcpy(user_tcp_conn.proto.tcp->local_ip,local_ip,4);//本地ip
	os_memcpy(user_tcp_conn.proto.tcp->remote_ip,remote_ip,4);//远程ip
	user_tcp_conn.proto.tcp->local_port=espconn_port();//指定端口
	user_tcp_conn.proto.tcp->remote_port=remote_port;//远程端口
	//注册连接回调函数和重连回调函数
	espconn_regist_connectcb(&user_tcp_conn,user_tcp_connect_cb);
	espconn_regist_reconcb(&user_tcp_conn,user_tcp_recon_cb);
	//启用连接
	espconn_connect(&user_tcp_conn);
}


void ICACHE_FLASH_ATTR Wifi_conned(void *arg)
//ICACHE_FLASH_ATTR 是宏定义不要在gpio和uart中断处理中回调，容易异常。
//定义回调函数Wifi_conned
{
	static uint8 count=0;//定变量
	uint8 status;//定变量
	os_timer_disarm(&connect_timer);//关闭定时器,connect_timer使能定时器
	count++;
	status=wifi_station_get_connect_status();//查询连接的状态函数输出
	if(status==STATION_GOT_IP)//判断连接状态
	{
		os_printf("Wifi connect success!");//输出连接成功
		struct ip_info info;//定义结构体指针
		const char remote_ip[4]={192,168,188,187};//远程的ip地址。都是路由器给分配 的,自己修改
		wifi_get_ip_info(STATION_IF,&info);//获取本地的ip地址
		my_station_init((struct ip_addr *)remote_ip,&info.ip,8080);
		//初始化客户端连接  远程8080端口
		return;
		}else{
			if(count>=7)//判断连接次数
			{
				os_printf("Wifi connect fail！");//输出连接失败
				return;
			}
		}
	os_timer_arm(&connect_timer,2000,NULL);
}

void ICACHE_FLASH_ATTR scan_done(void *arg,STATUS status)
//添加宏定义回调函数scan_done包含指针和状态
{
	uint8 ssid[33];
	 ESP_DBG((" 333 "));
	  if (status == OK)
	  {
	    struct bss_info *bss_link = (struct bss_info *)arg;
	    bss_link = bss_link->next.stqe_next;//ignore first
	    ESP_DBG((" 111 "));
	    while (bss_link != NULL)
	    {
	    	ESP_DBG((" 222 "));
	      os_memset(ssid, 0, 33);
	      if (os_strlen(bss_link->ssid) <= 32)
	      {
	        os_memcpy(ssid, bss_link->ssid, os_strlen(bss_link->ssid));
	      }
	      else
	      {
	        os_memcpy(ssid, bss_link->ssid, 32);
	      }
	      os_printf("+CWLAP:(%d,\"%s\",%d,\""MACSTR"\",%d)\r\n",bss_link->authmode, ssid, bss_link->rssi,MAC2STR(bss_link->bssid),bss_link->channel);
	      //串口输出wifi信息。可以在AT的源码中进行移植。ssid路由器帐号 rssi信号强度
          //bssid AP的mac地址   channel  AP的通道 ，wifi有14个通道
	      bss_link = bss_link->next.stqe_next;
	    }//输出AP的信息
	    struct station_config stationConf;//定义结构体stationConf
	    os_memcpy(&stationConf.ssid, "Wulian_101E1D", 32);//连接的路由器
	    os_memcpy(&stationConf.password, "12345678", 64);//密码
	    wifi_station_set_config_current(&stationConf);//wifi来连接的接口配置。sdk有例子
	    wifi_station_connect();//wifi连接函数
	    os_timer_setfn(&connect_timer,Wifi_conned,NULL);//定时器回调函数
	    os_timer_arm(&connect_timer,2000,NULL);//定时器2s扫描一次
	  }
	  else
	  {

	  }

}

void to_scan(void)
{
    wifi_station_scan(NULL,scan_done);
}
//定义回调函数to_scan，获取所有的ap 和扫描完成后的回调scan_done
