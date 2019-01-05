//ultrasonic adc test

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "time.h"

#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "string.h"
#include "freertos/queue.h"
#include "soc/rtc.h"

#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64

#define BUF_SIZE (1024)
#define ECHO_TEST_TXD  (GPIO_NUM_17)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)




static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel0 = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2 for ultrasonic
static const adc_channel_t channel1 = ADC_CHANNEL_0;     //GPIO36 A4 for wheel speed sensor
static const adc_channel_t channel2 = ADC_CHANNEL_3;     //GPIO39 A3 for IR sensor
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

double lidar_result;

// speed sensor param
clock_t speed_sensor_t;
int speed_sensor_read_time = 0;
double speed_sensor_period;
double distance_speed = 2 * 3.1415926 * 2.5;
int voltage_edge;
double wheel_speed;
double speed_result;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}



void app_main()
{
	//adc ultrasonic
    check_efuse();

    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel0, atten);//ultrasonic
		adc1_config_channel_atten(channel1, atten);//speed
		adc1_config_channel_atten(channel2, atten);//ir
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel0, atten);
		adc2_config_channel_atten((adc2_channel_t)channel1, atten);
		adc2_config_channel_atten((adc2_channel_t)channel2, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);



	//Lidar uart setting
		uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
		};
		uart_param_config(UART_NUM_0, &uart_config);
		uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
		uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
		uint8_t *data = (uint8_t *) malloc(BUF_SIZE);



	uint8_t header = 0x59;

    while (1) {
        uint32_t adc_ultra = 0;
		uint32_t adc_speed = 0;
		uint32_t adc_ir = 0;

        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_ultra += adc1_get_raw((adc1_channel_t)channel0);
				adc_speed += adc1_get_raw((adc1_channel_t)channel1);
				adc_ir += adc1_get_raw((adc1_channel_t)channel2);

            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel0, ADC_WIDTH_BIT_12, &raw);
                adc_ultra += raw;
            }
		}


		//ultra
		int output = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 100);

		bool loop = true;
		int j = 0;
		while(loop == true)
		{
			//printf("data: %02hhx\n", data[j]);
			if (data[j] == header && (data[j+1]) == header)
			{
				lidar_result = ((uint16_t)(data[j+3]) << 8) | (data[j+2]);
				loop = false;
			}
			else
			{
				j++;
			}
		}


        adc_ultra /= NO_OF_SAMPLES;
		adc_speed /= NO_OF_SAMPLES;
		adc_ir /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage_ultra = esp_adc_cal_raw_to_voltage(adc_ultra, adc_chars);
		float distance_ultra = voltage_ultra;
		distance_ultra = distance_ultra/1000;
		distance_ultra = distance_ultra * 5120;
		distance_ultra = distance_ultra / 10;

		//speed sensor
    	if(adc_speed < 4050 && speed_sensor_read_time == 0)
        {
    		voltage_edge = 0; //low
    		speed_sensor_t = clock();
    		speed_sensor_read_time++;
        speed_result = 0;
    	}
    	else if(adc_speed >= 4050){
    		voltage_edge = 1;         //set high
        //speed_result = 0;
    	}
    	else if(adc_speed < 4050 && voltage_edge == 1)
        {
    		voltage_edge = 0;
    		clock_t second_clk = clock();
    		speed_sensor_period = (double) ((second_clk - speed_sensor_t)/(double) CLOCKS_PER_SEC);
    		wheel_speed = distance_speed / speed_sensor_period;
    		//printf("Time period is : %fs, wheel_speed is: %fcm/s \n", speed_sensor_period, wheel_speed);
			speed_result = wheel_speed;
    		speed_sensor_read_time = 0;
    		speed_sensor_t = second_clk;
    	}


		//IR
        //Convert adc_reading to voltage in mV
        uint32_t voltage_ir = esp_adc_cal_raw_to_voltage(adc_ir, adc_chars);
		double distance_ir;
        //150cm-30cm
          double distance_1 = 58.61/(voltage_ir/1000.0 - 0.0073);

          //30cm-20cm
          double distance_2 = 34.375/(voltage_ir/1000.0 - 0.8312);

          if(voltage_ir >= 400 && voltage_ir < 2000)
		  {
			  distance_ir = distance_1;
		  }
          else if(voltage_ir >= 2000 && voltage_ir < 2500)
		  {
			  distance_ir = distance_2;
          }
		  else
		  {
			  distance_ir = 0;
		  }



    printf("L: %fcm ", lidar_result);
    printf("U: %fcm ", distance_ultra);
		printf("I: %fcm ", distance_ir);
		printf("W: %fcm/s\n", speed_result);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
