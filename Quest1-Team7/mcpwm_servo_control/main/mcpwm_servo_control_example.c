//Group member Wuliang Cheng, Cong Han, Qinglang Yu

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "string.h"
#include "freertos/queue.h"
#include "soc/rtc.h"
#include "driver/uart.h"

#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 60  //Maximum angle in degree upto which servo can rotate

#define BUF_SIZE (1024)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (1) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

//////timer struct
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

//////timer functs
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
                                    (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}


void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = 
        ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG0.hw_timer[timer_idx].cnt_low;

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

static void example_tg0_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
	
	
}

timer_event_t evt;
timer_event_t start_time;
//timer_event_t now_time;

static void timer_example_evt_task(void *arg)
{
	printf("Starting evttask\n");
    while (1) {
        //timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
	
}


static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 26);    //Set GPIO 18 as PWM0A, to which servo is connected
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, 25);
	}


static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


void mcpwm_example_servo_control(void *arg)
{
	gpio_pad_select_gpio(18);
	gpio_set_direction(18, GPIO_MODE_OUTPUT);
	
    uint32_t anglesec, sec, anglemin, min ,hr;
	sec = 0;
	min = 0;
	hr = 0;
	
	uint32_t alarm_hr, alarm_min;
	alarm_hr = 0;
	alarm_min = 0;
	
	bool alarm_set = false;
	
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
	printf("Please set hour:");
	printf(" \n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	
	//uart setting
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	
	
	//setting the current time
	int starthr = 0;
	int startmin = 0;
	int startsec = 0;
	
	
	while(1){
		int l1 = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 2000 / portTICK_RATE_MS);
				if (l1 > 0)
				{
					starthr = 10*(*data - '0')+(*(data+1) - '0');
					if (starthr >= 0 && starthr <24){
						printf("%d\n", starthr);
						printf("Please set minute: \n");
						
						break;
					}
					else{
						printf("Wrong input! Please try again!\n");
					}
				}
	}
	
	while(1){
		int l2 = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 2000 / portTICK_RATE_MS);
				if (l2 > 0)
				{
					startmin = 10*(*data - '0')+(*(data+1) - '0');
					if (startmin >= 0 && startmin < 60){
						printf("%d\n", startmin);
						break;
					}
					else{
						printf("Wrong input! Please try again!\n");
					}	
				}	
	}				
	
	printf("Press 'E' to check the alarm, press 'Q' to toggle the alarm, press 'A' and 'S' to increase hour and minute, press 'Z' and 'X' to decrease. \n");
	
	double starttime = (3600 * starthr) + (60 * startmin) + startsec;
	start_time.timer_counter_value = starttime * TIMER_SCALE;
	printf("starting time is %llu\n", start_time.timer_counter_value);
	double now_time = (evt.timer_counter_value + start_time.timer_counter_value)/TIMER_SCALE;

	
    while (1) {
			double timenow = (evt.timer_counter_value + start_time.timer_counter_value)/TIMER_SCALE;
			int nowsec = floor(timenow);
			nowsec = nowsec % 60;
			int nowmin = floor(timenow);
			nowmin = (nowmin % 3600) / 60;
			int nowhr = floor(timenow);
			nowhr = (nowhr % 86400) / 3600;
			
			xQueueReceive(timer_queue, &evt, portMAX_DELAY);
			//printf("current time is %f\n", timenow);
			
			if (nowsec != sec)
			{
				//sec
				sec = nowsec;
				anglesec = servo_per_degree_init(sec);
				mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, anglesec);
				
				
				//min
				
				min = nowmin;
				anglemin = servo_per_degree_init(min);
				mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, anglemin);
				
				
				
				//hr
				hr = nowhr;
				
				
				printf("The time is: %d : %d : %d\n", hr, min, sec);
				
				
				//alarm led
				
				if (alarm_set && alarm_hr == hr && alarm_min == min)
				{
					gpio_set_level(18, 1);
				}
				else{
					gpio_set_level(18, 0);
				}
				
				
				
				//alarm settings
				int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
				if (len > 0)
				{
					printf("Please press 'E' to check the alarm, press 'Q' to toggle the alarm: %c \n", *data);
					if (*data == 'e')
					{
						printf("Now the alarm time is set at %d : %d\n", alarm_hr, alarm_min);
						printf("The alarm is ");
						printf("%s\n", alarm_set ? "ON" : "OFF");
						printf("Press 'A' and 'S' to increase hour and minute, press 'Z' and 'X' to decrease. \n");
					}
					else if (*data == 'a')
					{
						if (alarm_hr == 23)
						{alarm_hr = 0;}
						else
						{alarm_hr++;}
						printf("Now the alarm time is set at %d : %d\n", alarm_hr, alarm_min);
					}
					else if (*data == 'z')
					{
						if (alarm_hr == 0)
						{alarm_hr = 23;}
						else
						{alarm_hr--;}
						printf("Now the alarm time is set at %d : %d\n", alarm_hr, alarm_min);
					}
					else if (*data == 's')
					{
						if (alarm_min >= 55)
						{alarm_min = 0;}
						else
						{alarm_min = alarm_min + 5;}
						printf("Now the alarm time is set at %d : %d\n", alarm_hr, alarm_min);
					}
					else if (*data == 'x')
					{
						if (alarm_min ==0)
						{alarm_min = 55;}
						else
						{alarm_min = alarm_min - 5;}
						printf("Now the alarm time is set at %d : %d\n", alarm_hr, alarm_min);
					}
					else if (*data == 'q')
					{
						alarm_set = !alarm_set;
						printf("The alarm is ");
						printf("%s\n", alarm_set ? "ON" : "OFF");
					}
				}
			}	
			//if (nowsec != sec) continue;	
            //vTaskDelay(100);
			
    }
}

void app_main()
{
    
	timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
	xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
	//xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
	
	
}