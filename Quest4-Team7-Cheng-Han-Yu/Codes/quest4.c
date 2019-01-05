#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 26   //Set GPIO 15 as PWM0A left
#define GPIO_PWM0B_OUT 19   //Set GPIO 16 as PWM0B	right

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
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define Kp 0.8
#define Ki 0
#define Kd 0.4

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider, 80 to get 1MHz clock to timer */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (0.1)   /*!< test interval for timer 0 */

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64

#define BUF_SIZE (100)
#define ECHO_TEST_TXD  (GPIO_NUM_17)
#define ECHO_TEST_RXD  (GPIO_NUM_16)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define ECHO_TEST_TXD_SIDE  (GPIO_NUM_33)
#define ECHO_TEST_RXD_SIDE  (GPIO_NUM_15)
#define ECHO_TEST_RTS_SIDE  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS_SIDE  (UART_PIN_NO_CHANGE)

#define ECHO_TEST_TXD_B  (GPIO_NUM_4)
#define ECHO_TEST_RXD_B  (GPIO_NUM_4)
#define ECHO_TEST_RTS_B  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS_B  (UART_PIN_NO_CHANGE)

//adc stuff for IR sensor
/*static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel0 = ADC_CHANNEL_6;     //GPIO34 
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;*/

int dt_complete = 1;
double dt = 0.1;
double previous_error ;		// Set up PID loop
double integral ;
double error;
double integral;
double derivative;
double output;
double setpoint = 40; //setpoint is 50 cm
double PID(int measured_value) {
  error = setpoint - measured_value;
  integral = integral + error * dt;
  derivative = (error - previous_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  return output;
}


void IRAM_ATTR timer_isr()
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    // Indicate timer has fired
    dt_complete = 1;
}

// Set up periodic timer for dt = 100ms
static void periodic_timer_init()
{
	int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_0;
    // Basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;

    // register timer interrupt
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm.
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, TIMER_INTERVAL0_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_isr,
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, timer_idx);
}




/*static void check_efuse()
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
}*/


/*static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}*/


static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

uint16_t lidar_result;
	
uint16_t lidar_side;
	
int speed_r = 80;
int speed_l = 80;

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
/*static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}*/

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
/*static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}*/

/**
 * @brief motor stop
 */
/*static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}*/

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
	
	/*check_efuse();
	
	
	if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel0, atten);//IR A2
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel0, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);*/
	
	
	periodic_timer_init();
	
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
		};
		
	uart_config_t uart_config_side = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
		};
		
	uart_config_t uart_config_beacon = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
		
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    //printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 2000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 100.0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 100.0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	
	//right wheel
	gpio_pad_select_gpio(25);
	gpio_pad_select_gpio(18);
	gpio_set_direction(25, GPIO_MODE_OUTPUT);
	gpio_set_direction(18, GPIO_MODE_OUTPUT);
	gpio_set_level(25, 1);
	gpio_set_level(18, 0);

	//left wheel
	gpio_pad_select_gpio(32);
	gpio_pad_select_gpio(14);
	gpio_set_direction(32, GPIO_MODE_OUTPUT);
	gpio_set_direction(14, GPIO_MODE_OUTPUT);
	gpio_set_level(32, 1);
	gpio_set_level(14, 0);
	
	mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 100);
		
	mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100);
	
	
	
	
	uart_param_config(UART_NUM_0, &uart_config);
	uart_set_pin(UART_NUM_0, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
	uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	uint8_t header = 0x59;
	
	
	uart_param_config(UART_NUM_1, &uart_config_side);
	uart_set_pin(UART_NUM_1, ECHO_TEST_TXD_SIDE, ECHO_TEST_RXD_SIDE, ECHO_TEST_RTS_SIDE, ECHO_TEST_CTS_SIDE);
	uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
	uint8_t *data_side = (uint8_t *) malloc(BUF_SIZE);
	
	uart_param_config(UART_NUM_2, &uart_config_beacon);
	uart_set_pin(UART_NUM_2, ECHO_TEST_TXD_B, ECHO_TEST_RXD_B, ECHO_TEST_RTS_B, ECHO_TEST_CTS_B);
	uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
	uint8_t *data_b = (uint8_t *) malloc(BUF_SIZE);
	
	uart_set_line_inverse(UART_NUM_2, UART_INVERSE_RXD);
	
	bool stop_state = false;
	bool beacon_state = false;
	
	
	while (1) {
		if (stop_state == false)
		{	
		
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
    		else{
    			j++;
    		}
    	}
		
		int output_side = uart_read_bytes(UART_NUM_1, data_side, BUF_SIZE, 100);
		bool loop_side = true;
    	int k = 0;
    	while(loop_side == true)
    	{
    		//printf("data: %02hhx\n", data[j]);
    		if (data_side[k] == header && (data_side[k+1]) == header)
    		{
    			lidar_side = ((uint16_t)(data_side[k+3]) << 8) | (data_side[k+2]);
    			loop_side = false;
    		}
    		else{
    			k++;
    		}
    	}
		
		beacon_state = false;
		int output_beacon = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_RATE_MS);
		if (output_beacon > 0)
		{
			for(int i = 0; i < 20; i++){
				if(data[i] == 0x0A){
					printf("ID is %x\n", data[i+1]);
					if (data[i+1] == 1 || data[i+1] == 2 || data[i+1] == 3 || data[i+1] == 0)
					{
						beacon_state = true;
					}
					if (data[i+1] == 3)
					{
						stop_state = true;
					}
					break;
				}	
			}
		}
		
		
		if (dt_complete == 1)
		{
			printf("LIDAR READING: %dcm\n", lidar_result);
			printf("LIDAR side: %dcm\n", lidar_side);
			output = PID(lidar_side);
			printf("PID out: %d\n", output);
			dt_complete = 0;
			TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
		}
		
		printf(beacon_state ? "true" : "false");
		printf("\n");
		
		if (lidar_result < 27)
		{
			//if (beacon_state == true)
			//{
			//gpio_set_level(32, 0);
			//gpio_set_level(14, 1);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 20);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 90);
			vTaskDelay(500 / portTICK_RATE_MS);
			//gpio_set_level(32, 1);
			//gpio_set_level(14, 0);
		//	}
			/*else 
			{
				mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
				mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
				vTaskDelay(100 / portTICK_RATE_MS);
			}*/
		}
		else if (output > 0)
		{
			speed_r = speed_r + output;
			speed_l = speed_l - output;
			if (speed_r > 80){speed_r = 80;}
			if (speed_l > 80){speed_l = 80;}
			if (speed_r < 50){speed_r = 50;}
			if (speed_l < 50){speed_l = 50;}
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed_l);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed_r);
		}
		else if (output < 0)
		{
			speed_r = speed_r + output;
			speed_l = speed_l - output;
			if (speed_r > 80){speed_r = 80;}
			if (speed_l > 80){speed_l = 80;}
			if (speed_r < 50){speed_r = 50;}
			if (speed_l < 50){speed_l = 50;}
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed_l);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed_r);
		}
		else if (output == 0)
		{
			speed_r = 80;
			speed_l = 80;
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed_l);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed_r);
		}
    }
	else
	{
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
		vTaskDelay(200 / portTICK_RATE_MS);
	}
	}
}

void app_main()
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}