/* BSD Socket API Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include <stdlib.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "time.h"

#include "string.h"
#include "freertos/queue.h"
#include "soc/rtc.h"

#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "freertos/queue.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64

#define BUF_SIZE (100)

#define ECHO_TEST_TXD_B  (GPIO_NUM_4)
#define ECHO_TEST_RXD_B  (GPIO_NUM_4)
#define ECHO_TEST_RTS_B  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS_B  (UART_PIN_NO_CHANGE)


/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID "Group_7"
#define EXAMPLE_WIFI_PASS "smart-systems"

//#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR "192.168.1.118"
//#else
//#define HOST_IP_ADDR "192.168.1.105"
//#endif

#define PORT 3000

char id_msg[5];

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}

uart_config_t uart_config_beacon = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

static void udp_client_task(void *pvParameters)
{
	bool go = true;
	//int output_beacon;
	
	while(1){
		//output_beacon = 0;
		printf("CLIENT_TASK...\n");
		go = true;
		uart_param_config(UART_NUM_1, &uart_config_beacon);
		uart_set_pin(UART_NUM_1, ECHO_TEST_TXD_B, ECHO_TEST_RXD_B, ECHO_TEST_RTS_B, ECHO_TEST_CTS_B);
		uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
		uint8_t *data_b = (uint8_t *) malloc(BUF_SIZE);
		
		uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
		vTaskDelay(100);
		while(go){
		printf("reading ir...\n");
		int output_beacon = uart_read_bytes(UART_NUM_1, data_b, BUF_SIZE, 20 / portTICK_RATE_MS);
		if (output_beacon > 0)
		{
			for(int i = 0; i < 20; i++){
				if(data_b[i] == 0x0A){
					id_msg[0] = ((int)data_b[i+1])+'0';
					id_msg[1] = ((int)data_b[i+2])+'0';
					id_msg[2] = ((int)data_b[i+3])+'0';
					id_msg[3] = ((int)data_b[i+4])+'0';
					id_msg[4] = ((int)data_b[i+5])+'0';
					printf("ID and Password: %c %c%c%c%c\n", id_msg[0], id_msg[1], id_msg[2], id_msg[3], id_msg[4]);
					go = false;
					/*printf("ID is %d", data_b[i+1]);
					printf("%x", data_b[i+2]);
					printf("%x", data_b[i+3]);
					printf("%x", data_b[i+4]);
					printf("%x", data_b[i+5]);
					printf("%x\n", data_b[i+6]);*/
					break;
				}	
			}
		}
		vTaskDelay(100);
	}
	
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
	bool rr = true;
    //while (rr) {

        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);


        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        //while (1) {

            int err = sendto(sock, id_msg, strlen(id_msg), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            /*struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(sourceAddr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }*/
			//rr = false;
           // vTaskDelay(1000 / portTICK_PERIOD_MS);
        //}
		
		printf("id_msg sent\n");
		vTaskDelay(100);
		
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
		break;
   }
    vTaskDelete(NULL);
}


static void udp_server_task(void *pvParameters)
{	
	uart_param_config(UART_NUM_1, &uart_config_beacon);
	uart_set_pin(UART_NUM_1, ECHO_TEST_TXD_B, ECHO_TEST_RXD_B, ECHO_TEST_RTS_B, ECHO_TEST_CTS_B);
	uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
	uint8_t *data_b = (uint8_t *) malloc(BUF_SIZE);
	
	uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);
	
	while(1){
		printf("reading ir...\n");
		int output_beacon = uart_read_bytes(UART_NUM_1, data_b, BUF_SIZE, 20 / portTICK_RATE_MS);
		if (output_beacon > 0)
		{
			for(int i = 0; i < 20; i++){
				if(data_b[i] == 0x0A){
					id_msg[0] = (int)data_b[i+1];
					id_msg[1] = (int)data_b[i+2];
					id_msg[2] = (int)data_b[i+3];
					id_msg[3] = (int)data_b[i+4];
					id_msg[4] = (int)data_b[i+5];
					printf("ID and Password: %d %d%d%d%d\n", id_msg[0], id_msg[1], id_msg[2], id_msg[3], id_msg[4]);
					/*printf("ID is %d", data_b[i+1]);
					printf("%x", data_b[i+2]);
					printf("%x", data_b[i+3]);
					printf("%x", data_b[i+4]);
					printf("%x", data_b[i+5]);
					printf("%x\n", data_b[i+6]);*/
					break;
				}	
			}
		}
		vTaskDelay(100);
	}
	
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

//#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
/*#else // IPV6
        struct sockaddr_in6 destAddr;
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif*/

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket binded");

		int ii = 0;
        while (ii < 10) {
			ii++;
			printf("%d\n", ii);
            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(sourceAddr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (sourceAddr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (sourceAddr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "MESSAGE RECEIVED = %s", rx_buffer);

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }
            }
			printf("owehfwoeufh\n");
			vTaskDelay(100);
        }
		
		printf("owehfwoeufh\n");

        //if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        //}
		break;
    }
    vTaskDelete(NULL);
}


void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();


	xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

	//xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
	//xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
}