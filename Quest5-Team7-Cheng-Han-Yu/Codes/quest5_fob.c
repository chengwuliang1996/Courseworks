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
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"

// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

// LED Output pins definitions
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15

// Default ID
#define ID 0
#define EXAMPLE_WIFI_SSID "Group_7"
#define EXAMPLE_WIFI_PASS "smart-systems"
#define HOST_IP_ADDR "192.168.1.118"
#define PORT 4000

// Variables for my ID, minVal and status plus received ID, minVal, and status
char start = 0x0A;
char myID = (char)ID;
char qinglang[] = "9999";
char wuliang[] = "7777";
char cong[] = "4444";
int ff = 6;
char rxID;
char code[];
int len_out = 6;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
static xQueueHandle gpio_evt_queue = NULL;

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void* arg) {
	uint32_t gpio_num = (uint32_t)arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// RMT tx init
static void rmt_tx_init() {
	rmt_config_t rmt_tx;
	rmt_tx.channel = RMT_TX_CHANNEL;
	rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
	rmt_tx.mem_block_num = 1;
	rmt_tx.clk_div = RMT_CLK_DIV;
	rmt_tx.tx_config.loop_en = false;
	rmt_tx.tx_config.carrier_duty_percent = 50;
	// Carrier Frequency of the IR receiver
	rmt_tx.tx_config.carrier_freq_hz = 38000;
	rmt_tx.tx_config.carrier_level = 1;
	rmt_tx.tx_config.carrier_en = 1;
	// Never idle -> aka ontinuous TX of 38kHz pulses
	rmt_tx.tx_config.idle_level = 1;
	rmt_tx.tx_config.idle_output_en = true;
	rmt_tx.rmt_mode = 0;
	rmt_config(&rmt_tx);
	rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
	// Basic configs
	uart_config_t uart_config = {
		.baud_rate = 1200, // Slow BAUD rate
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_1, &uart_config);

	// Set UART pins using UART0 default pins
	uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// Reverse receive logic line
	uart_set_line_inverse(UART_NUM_1, UART_INVERSE_RXD);

	// Install UART driver
	uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init() {
	gpio_pad_select_gpio(BLUEPIN);
	gpio_pad_select_gpio(GREENPIN);
	gpio_pad_select_gpio(REDPIN);
	gpio_pad_select_gpio(18);
	gpio_set_direction(18, GPIO_MODE_OUTPUT);
	gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
}

// Button interrupt init
static void hw_int_init() {
	gpio_config_t io_conf;
	//interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	//bit mask of the pins, use GPIO4 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
	gpio_intr_enable(GPIO_INPUT_IO_1);
	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*)GPIO_INPUT_IO_1);
	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
}

// Button task -- reset minVal and status
void button_task() {
	uint32_t io_num;
	while (1) {
		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			xSemaphoreTake(mux, portMAX_DELAY);
			if (myID == 3) {
				myID = 0;
			}
			else {
				myID++;
			}
			xSemaphoreGive(mux);
			printf("Button pressed.\n");
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

}

// Send task -- sends payload | Start | myID | Start | myID
void send_task() {
	while (1) {

		char *data_out = (char *)malloc(len_out);
		xSemaphoreTake(mux, portMAX_DELAY);
		data_out[0] = start;
		data_out[1] = myID;
		if (myID == 0) {
			data_out[2] = 'Nah';
			
		}
		else if (myID == 1) {
			//printf("langlanglang");
			data_out[2] = 0;
			data_out[3] = 0;
			data_out[4] = 1;
			data_out[5] = 0;
		}
		else if(myID == 2) {
			//printf("congcongcong");
			//data_out[2] = *cong;
			data_out[2] = 1;
			data_out[3] = 0;
			data_out[4] = 1;
			data_out[5] = 1;
		}
		else if (myID == 3) {
			//data_out[2] = *wuliang;
			data_out[2] = 4;
			data_out[3] = 3;
			data_out[4] = 7;
			data_out[5] = 9;
		}
		uart_write_bytes(UART_NUM_1, data_out, len_out + 1);
		xSemaphoreGive(mux);

		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

// Receives task -- looks for Start byte then stores received values
void recv_task() {
	// Buffer for input data
	uint8_t *data_in = (uint8_t *)malloc(BUF_SIZE);
	while (1) {
		int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
		if (len_in >0) {
			for (int i = 0; i < 24; i++) {
				if (data_in[i] == 0x0A) {

					rxID = data_in[i + 1];
					//code = data_in[i + 2];
					printf("Received comm from device ID 0x%02X\n", rxID);
					printf("%c", rxID);
					if (rxID == '0') {
						printf("Code is Nah\n");
					}
					else if (rxID == '1') {
						printf("Code is %s\n", qinglang);
					}
					else if (rxID == '2') {
						printf("Code is %s\n", cong);
					}
					else if (rxID == '3') {
						printf("Code is %s\n", wuliang);
					}

					break;
				}
			}
		}
		else {
			// printf("Nothing received.\n");
		}
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
	free(data_in);
}


// LED task to light LED based on status
void led_task() {
	while (1) {
		switch ((int)myID) {
		case 0:
			gpio_set_level(GREENPIN, 0);
			gpio_set_level(REDPIN, 0);
			gpio_set_level(BLUEPIN, 0);
			// printf("Current state: %c\n",status);
			break;
		case 1:
			gpio_set_level(GREENPIN, 0);
			gpio_set_level(REDPIN, 0);
			gpio_set_level(BLUEPIN, 1);
			// printf("Current state: %c\n",status);
			break;
		case 2:
			gpio_set_level(GREENPIN, 1);
			gpio_set_level(REDPIN, 0);
			gpio_set_level(BLUEPIN, 0);
			// printf("Current state: %c\n",status);
			break;
		case 3:
			gpio_set_level(GREENPIN, 0);
			gpio_set_level(REDPIN, 1);
			gpio_set_level(BLUEPIN, 0);
			// printf("Current state: %c\n",status);
			break;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

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
static void udp_server_task(void *pvParameters)
{	
	printf("UPD_SERVER_TASK is runnning\n");
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
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
			
			gpio_set_level(18, 1);
			printf("received sth\n");
			vTaskDelay(500);
			gpio_set_level(18, 0);
        }
			/*gpio_set_level(18, 1);
			printf("EEEEEEEEEEEEEE\n");
			vTaskDelay(500);
			gpio_set_level(18, 0);*/
		

        //if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        //}
		break;
    }
    vTaskDelete(NULL);
}


void app_main() {

	ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();

	// Mutex for current values when sending and during election
	mux = xSemaphoreCreateMutex();

	// Initialize transmitt and button interrupt
	rmt_tx_init();
	uart_init();
	led_init();
	hw_int_init();

	// Create tasks for receive, send, elect, set gpio, and button
	xTaskCreate(recv_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES, NULL);
	xTaskCreate(send_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
	xTaskCreate(led_task, "set_gpio_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
	xTaskCreate(button_task, "button_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
	xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

}