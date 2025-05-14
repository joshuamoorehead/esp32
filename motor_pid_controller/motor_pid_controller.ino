// Copyright 2025 Joshua Moorehead
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include <math.h>
#include <stdint.h>


// motor connector
#define motor_pwm_pin 32
#define motor_direction_pin 4
#define brake_control_pin 26
#define encoder_A_pin 35
#define encoder_B_pin 33

#define DPORT_REG_WRITE(_r, _v) (*(volatile uint32_t *)(_r)) = (_v)
#define DPORT_REG_READ(reg) (*(volatile uint32_t *)(reg))
#define DPORT_REG_SET_BIT(_r, _b) DPORT_REG_WRITE((_r), (DPORT_REG_READ(_r)|(_b)))
#define DPORT_REG_CLR_BIT(_r, _b) DPORT_REG_WRITE((_r), (DPORT_REG_READ(_r) & (~(_b))))


#define LEDC_HSTIMER0_CONF_REG 0x3FF59140
#define LEDC_HSCH0_CONF0_REG 0x3FF59000 
#define LEDC_HSCH0_CONF1_REG 0x3FF5900C 
#define LEDC_HSCH0_DUTY_REG 0x3FF59008 
#define LEDC_INT_RAW_REG 0x3FF59180
#define LEDC_INT_CLR_REG 0x3FF5918C
#define DPORT_PERIP_CLK_EN_REG 0x3FF000C0
#define DPORT_PERIP_RST_EN_REG 0x3FF000C4
#define LEDC_CONF_REG 0x3FF59190

//GPIO
#define GPIO_FUNC32_OUT_SEL_CFG_REG 0x3FF445B0 //LEDC
#define GPIO_ENABLE_REG 0x3FF44020
#define IO_MUX_GPIO32_REG 0x3FF49040

#define TASK_PERIOD_IN__MS 100  // Task period in milliseconds


// Global variables for motor control
volatile int motor_speed = 0;        // Current motor speed in RPM
int setpoint = 500;                  // Target RPM (can be adjusted)
int pwm_value = 0;                   // Current PWM value
int error = 0;                       // Current error
int derivative_error = 0;            // Error derivative
int integral_error = 0;              // Error integral
int prev_error = 0;                  // Previous error
float Kp = 0.5;                      // Proportional gain
float Ki = 0.05;                      // Integral gain
float Kd = 0.1;                     // Derivative gain


// Variables for CPU utilization monitoring
uint64_t start_time = 0;
uint64_t monitorTaskTotal = 0;
uint64_t controlTaskTotal = 0;

pcnt_unit_handle_t mypcnt = 0;
pcnt_channel_handle_t mychannel = 0;

TaskHandle_t speedMonitorTask1;
TaskHandle_t motorControlTask1;


void speedMonitorTask (void *args) {
  uint64_t start_task;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xTimeIncrement = pdMS_TO_TICKS(TASK_PERIOD_IN__MS);

  for (;;) {
    start_task = esp_timer_get_time();

    int pulse_count = 0;
    pcnt_unit_get_count(mypcnt, &pulse_count);
    pcnt_unit_clear_count(mypcnt);


    float T = TASK_PERIOD_IN__MS / 1000.0;  // T = 0.1 seconds
    motor_speed = (60.0 * pulse_count) / (100.0 * T);  // RPM calculation

    monitorTaskTotal += esp_timer_get_time() - start_task;
    vTaskDelayUntil(&xLastWakeTime,xTimeIncrement);
  }
}

void motorControlTask (void *args) {
  uint64_t start_task;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xTimeIncrement = pdMS_TO_TICKS(TASK_PERIOD_IN__MS);

  for (;;) {
    start_task = esp_timer_get_time();

    error = setpoint - motor_speed;
    derivative_error = error - prev_error;
    integral_error += error;

    float pid_output = Kp * error + Ki * integral_error + Kd * derivative_error;
    //if (pid_output < 0) pid_output = 0;
    //if (pid_output > 255) pid_output = 255;
    pwm_value = 4095 - pid_output;

    if (pwm_value < 0) pwm_value = 0;


    update_PWM(0, pwm_value);
    prev_error = error;
    

    controlTaskTotal += esp_timer_get_time() - start_task;
    vTaskDelayUntil(&xLastWakeTime,xTimeIncrement);
  }
}

void setup_PCNT() {

      pcnt_unit_config_t myunitconfig = {
        .low_limit = -1,
        .high_limit = (1<<14),
        .intr_priority = 0,
        .flags = {
        .accum_count=0 }
      };

    pcnt_chan_config_t mychannelconfig = {
      .edge_gpio_num = encoder_A_pin,
      .level_gpio_num = -1,
      .flags = {
      .invert_edge_input = 0,
      .invert_level_input = 0,
      .virt_edge_io_level= 0,
      .io_loop_back = 0
      }
    };

    pcnt_new_unit(&myunitconfig,&mypcnt);
    pcnt_new_channel(mypcnt,&mychannelconfig,&mychannel);
    pcnt_channel_set_edge_action(mychannel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_unit_clear_count(mypcnt);
    pcnt_unit_enable(mypcnt);
    pcnt_unit_start(mypcnt);


}

void update_PWM (uint32_t initial,uint32_t sample) {
    if (!initial) {
      while (!(REG_READ(LEDC_INT_RAW_REG) & (1 << 0))) { //LEDC_HSTIMER0_OVF_INT_RAW. goes high if timer overflowed - meaning pwm cycle finished
           //not ready to update yet
    }
    REG_SET_BIT(LEDC_INT_CLR_REG, 1 << 0); //LEDC_HSTIMER0_OVF_INT_CLR clearing interrupt flag

    REG_WRITE(LEDC_HSCH0_DUTY_REG, sample << 4);  //shifts into the top 8 of 13 bits
    REG_SET_BIT(LEDC_HSCH0_CONF1_REG, 1 << 31); // confirming configuration change. bit #31 think you can do this 

}

}

void setup_LEDC() {

    //Turn on LEDC
    DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, 1<<11);
    //Take LEDC out of reset
    DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, 1<<11);
  
    DPORT_REG_SET_BIT(LEDC_CONF_REG, 1);

    REG_WRITE(LEDC_HSTIMER0_CONF_REG,(19 << 13) | (12 << 0) | (1 << 25)); //bits 5-22 
    // Set DUTY_RES = 8 (bits 0–3)
    //REG_WRITE(LEDC_HSTIMER0_CONF_REG, (REG_READ(LEDC_HSTIMER0_CONF_REG) & ~(0xF)) | (8));
    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1<<24); // we had it as 3 


    //GPIO Setup
    REG_CLR_BIT(GPIO_FUNC32_OUT_SEL_CFG_REG, 1<<11);
    REG_SET_BIT(GPIO_FUNC32_OUT_SEL_CFG_REG, 1<<10);
    REG_CLR_BIT(GPIO_FUNC32_OUT_SEL_CFG_REG, 1<<9);
    REG_SET_BIT(IO_MUX_GPIO32_REG, 1<<13); //MCU_sel
    //TODO GPIO Func out sel
    REG_WRITE(GPIO_FUNC32_OUT_SEL_CFG_REG, 71);
    REG_SET_BIT(GPIO_ENABLE_REG, 1<<27); //GPIOENable // or just 1

    //IOMUX Setup
    // REG_SET_BIT(IO_MUX_GPIO32_REG, 1<<13); //MCU_sel
    // REG_CLR_BIT(IO_MUX_GPIO32_REG, 1<<8); //FUN_WPU
    // REG_CLR_BIT(IO_MUX_GPIO32_REG, 1<<7); //FUN_WPD
    // REG_SET_BIT(GPIO_ENABLE_REG, 1<<27); //GPIOENable

    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1<<3); 
    //enables channel
    REG_SET_BIT(LEDC_HSCH0_CONF0_REG, 1<<2); 
    //select timer 0 (bits 0–1 = 0)
    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1 << 1);
    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1);

    REG_SET_BIT(LEDC_HSCH0_CONF1_REG, 1 << 31); //LEDC_DUTY_START_HSCH0 to 1 to commit changes to other configuration registers

    
    // REG_SET_BIT(LEDC_HSTIMER0_CONF_REG, 1<<25); //LEDC_TICK_SEL_HSTIMER0
    // //clear
    // REG_CLR_BIT(LEDC_HSTIMER0_CONF_REG, 1<<24); //LEDC_HSTIMER0_RST
    // //set divider in bits 5–22
    // float divider_val = (8e7)/(sampleRate * 256);
    // uint32_t divider_fixed = (uint32_t)(divider_val * 256.0f);
    // REG_WRITE(LEDC_HSTIMER0_CONF_REG,(REG_READ(LEDC_HSTIMER0_CONF_REG) & ~(0x3FFFF << 5)) | (divider_fixed << 5)); //bits 5-22 
    // // Set DUTY_RES = 8 (bits 0–3)
    // REG_WRITE(LEDC_HSTIMER0_CONF_REG, (REG_READ(LEDC_HSTIMER0_CONF_REG) & ~(0xF)) | (8));
  


}

void setup() {
  Serial.begin(115200); 
  start_time = esp_timer_get_time();
  pinMode(motor_direction_pin, OUTPUT);
  pinMode(brake_control_pin, OUTPUT);
  digitalWrite(motor_direction_pin, 0);
  digitalWrite(brake_control_pin, 1); //supposed to be high?
  setup_PCNT();
  setup_LEDC();
  xTaskCreate(speedMonitorTask,"speedMonitorTask",1<<16,0,configMAX_PRIORITIES - 2,&speedMonitorTask1);
  xTaskCreate(motorControlTask,"motorControlTask",1<<16,0,configMAX_PRIORITIES - 1,&motorControlTask1);
}

void loop() {
  // Monitor CPU utilization
  static unsigned long lastReport = 0;
 
  // Report CPU utilization every second
  if (millis() - lastReport > 1000) {
    uint64_t total_time = esp_timer_get_time() - start_time;
    float monitor_util = (float)monitorTaskTotal / total_time * 100.0;
    float control_util = (float)controlTaskTotal / total_time * 100.0;
   
    Serial.print("Monitor Task CPU: ");
    Serial.print(monitor_util, 2);
    Serial.println("%");
   
    Serial.print("Control Task CPU: ");
    Serial.print(control_util, 2);
    Serial.println("%");
   
    Serial.print("Motor Speed: ");
    Serial.print(motor_speed);
    Serial.println(" RPM");
   
    Serial.print("PWM Value: ");
    Serial.println(pwm_value);
   
    Serial.println("---------------------");
   
    lastReport = millis();
  }
 
  // Give other tasks time to run
  delay(10);

}
