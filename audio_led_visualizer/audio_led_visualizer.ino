// Copyright 2025 Joshua Moorehead
#include <math.h>
#include <stdint.h>
#include <array.h>

#define DPORT_REG_WRITE(_r, _v) (*(volatile uint32_t *)(_r)) = (_v)
#define DPORT_REG_READ(reg) (*(volatile uint32_t *)(reg))
#define DPORT_REG_SET_BIT(_r, _b) DPORT_REG_WRITE((_r), (DPORT_REG_READ(_r)|(_b)))
#define DPORT_REG_CLR_BIT(_r, _b) DPORT_REG_WRITE((_r), (DPORT_REG_READ(_r) & (~(_b))));
// LED registers
#define LEDC_HSTIMER0_CONF_REG 0x3FF59140
#define LEDC_HSCH0_CONF0_REG 0x3FF59000 
#define LEDC_HSCH0_CONF1_REG 0x3FF5900C 
#define LEDC_HSCH0_DUTY_REG 0x3FF59008 
#define LEDC_INT_RAW_REG 0x3FF59180
#define LEDC_INT_CLR_REG 0x3FF5918C

//RMT registers
#define DPORT_PERIP_CLK_EN_REG 0x3FF000C0
#define DPORT_PERIP_RST_EN_REG 0x3FF000C4
#define RMT_APB_CONF_REG 0x3FF560F0
#define RMT_CH0_CONF0_REG 0x3FF56020
#define RMT_CH0_CONF1_REG 0x3FF56024
#define RMT_INT_RAW_REG 0x3FF560A0
#define RMT_INT_CLR_REG 0x3FF560AC
#define RMT_RAM 0x3FF56800

//GPIO
#define GPIO_FUNC12_OUT_SEL_CFG_REG 0x3FF44560 //RMT
#define GPIO_FUNC27_OUT_SEL_CFG_REG 0x3FF4459C //LEDC
#define GPIO_ENABLE_REG 0x3FF44020

#define IO_MUX_GPIO12_REG 0x3FF49034
#define IO_MUX_GPIO27_REG 0x3FF4902C

#define RMT_ONE_BIT  ((36 << 16) | (1 << 15) | 64)
#define RMT_ZERO_BIT ((68 << 16) | (1 << 15) | 32)
#define RMT_TERMINATOR 0x00000000



void setup_RMT();
void setup_LEDC();
void update_PWM (uint32_t initial,uint32_t sample);
void transmit_led_signal(uint32_t *colors);
void populateColors(uint32_t *colors, uint32_t sample);
void transmit_led_signal(uint32_t *colors);


void setup_RMT() {
  //GPIO Setup
    REG_CLR_BIT(GPIO_FUNC12_OUT_SEL_CFG_REG, 1<<11);
    REG_CLR_BIT(GPIO_FUNC12_OUT_SEL_CFG_REG, 1<<10);
    //TODO GPIO Func out sel
    REG_WRITE(GPIO_FUNC12_OUT_SEL_CFG_REG, 87);

    //IOMUX Setup(
    REG_SET_BIT(IO_MUX_GPIO12_REG, 1<<13); //MCU_sel
    REG_CLR_BIT(IO_MUX_GPIO12_REG, 1<<8);
    REG_CLR_BIT(IO_MUX_GPIO12_REG, 1<<7);
    REG_SET_BIT(GPIO_ENABLE_REG, 1<<12);
    //Turn on RMT
    DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, 1<<9); // set  RMT bit DPORT_RMT_CLK_EN
    //Pull RMT out of reset
    DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, 1<<9); // DPORT_RMT_RST
    //enable ram
    REG_SET_BIT(RMT_APB_CONF_REG, 1);//check
    //set clock divider to 1
    REG_WRITE(RMT_CH0_CONF0_REG, (REG_READ(RMT_CH0_CONF0_REG) & ~0xFF) | 1); //technically 8 bits, using bit twiddling to ensure its 0b00000001 
    //clear rmt carrier bit
    REG_CLR_BIT(RMT_CH0_CONF0_REG, 1<<28); // RMT_CARRIER_EN_CH0
    //set rmt idle
    REG_SET_BIT(RMT_CH0_CONF1_REG, 1<<19); // RMT_IDLE_OUT_EN_CH0
    //set rmt ref always on to use  80mhz clk
    REG_SET_BIT(RMT_CH0_CONF1_REG, 1<<17); // RMT_REF_ALWAYS_ON_CH0

}

void setup_LEDC() {
    //GPIO Setup
    REG_CLR_BIT(GPIO_FUNC27_OUT_SEL_CFG_REG, 1<<11);
    REG_CLR_BIT(GPIO_FUNC27_OUT_SEL_CFG_REG, 1<<10);
    //TODO GPIO Func out sel
    REG_WRITE(GPIO_FUNC27_OUT_SEL_CFG_REG, 71);

    //IOMUX Setup
    REG_SET_BIT(IO_MUX_GPIO27_REG, 1<<13); //MCU_sel
    REG_CLR_BIT(IO_MUX_GPIO27_REG, 1<<8); //FUN_WPU
    REG_CLR_BIT(IO_MUX_GPIO27_REG, 1<<7); //FUN_WPD
    REG_SET_BIT(GPIO_ENABLE_REG, 1<<27); //GPIOENable

    //Turn on LEDC
    DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, 1<<11);
    //Take LEDC out of reset
    DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, 1<<11);
    //set
    REG_SET_BIT(LEDC_HSTIMER0_CONF_REG, 1<<25); //LEDC_TICK_SEL_HSTIMER0
    //clear
    REG_CLR_BIT(LEDC_HSTIMER0_CONF_REG, 1<<24); //LEDC_HSTIMER0_RST
    //set divider in bits 5–22
    float divider_val = (8e7)/(sampleRate * 256);
    uint32_t divider_fixed = (uint32_t)(divider_val * 256.0f);
    REG_WRITE(LEDC_HSTIMER0_CONF_REG,(REG_READ(LEDC_HSTIMER0_CONF_REG) & ~(0x3FFFF << 5)) | (divider_fixed << 5)); //bits 5-22 
    // Set DUTY_RES = 8 (bits 0–3)
    REG_WRITE(LEDC_HSTIMER0_CONF_REG, (REG_READ(LEDC_HSTIMER0_CONF_REG) & ~(0xF)) | (8));
    //sets idle value to 0
    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1<<3); 
    //enables channel
    REG_SET_BIT(LEDC_HSCH0_CONF0_REG, 1<<2); 
    //select timer 0 (bits 0–1 = 0)
    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1 << 1);
    REG_CLR_BIT(LEDC_HSCH0_CONF0_REG, 1);
    //duty update
    REG_SET_BIT(LEDC_HSCH0_CONF1_REG, 1<<31); //LEDC_DUTY_START_HSCH0 to 1 to commit changes to other configuration registers


}


void setup() {
  Serial.begin(115200);
  setup_LEDC();
  setup_RMT();

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


void populateColors(uint32_t *colors, uint32_t sample) {
    if (sample == 0) sample = 1;

    float db = log(sample / 256.0f); //convert 256 to log scale
    int ledsOn = (int)((db + 1.0f) * 7.0f); //map 0-5
    if (ledsOn < 1) ledsOn = 1;
    if (ledsOn > 5) ledsOn = 5; //incase
    Serial.printf("db: %0.4f\n", db);
    Serial.printf("Leds On: %d\n", ledsOn);


    for (int i = 0; i < 5; i++) {
        uint8_t r = 0, g = 0, b = 0;

        if (i < ledsOn) {
            // Assign color based on LED position (left to right)
            switch (i) {
                case 0: r = 255; g = 0;   b = 0;   break; // red
                case 1: r = 255; g = 128; b = 0;   break; // orange
                case 2: r = 255; g = 255; b = 0;   break; // yellow
                case 3: r = 128; g = 255; b = 0;   break; // lime green
                case 4: r = 0;   g = 255; b = 0;   break; // green
            }
        }// else {
            // Off (black)
           // r = g = b = 0;
        //}

        colors[i] = (r << 16) | (g << 8) | b;
    }
}

void transmit_led_signal(uint32_t *colors) {

      volatile uint32_t *rmt_ram = (uint32_t *)RMT_RAM;
      // for each color, perform a transmission
      for (int i = 0; i < 5; i++) {
        REG_SET_BIT(RMT_INT_CLR_REG, 1);
        int ram_index = 0;
        uint32_t color = colors[i];
        
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t b = color & 0xFF;

        for (int bit = 7; bit >= 0; bit--) rmt_ram[ram_index++] = (g >> bit) & 1 ? RMT_ONE_BIT : RMT_ZERO_BIT;
        for (int bit = 7; bit >= 0; bit--) rmt_ram[ram_index++] = (r >> bit) & 1 ? RMT_ONE_BIT : RMT_ZERO_BIT;
        for (int bit = 7; bit >= 0; bit--) rmt_ram[ram_index++] = (b >> bit) & 1 ? RMT_ONE_BIT : RMT_ZERO_BIT;

        rmt_ram[ram_index] = 0;



        //start transmission
        REG_SET_BIT(RMT_CH0_CONF1_REG, 1 << 3); //RMT_MEM_RD_RST
        REG_CLR_BIT(RMT_CH0_CONF1_REG, 1 << 3); //RMT_MEM_RD_RST
        REG_SET_BIT(RMT_CH0_CONF1_REG, 1); //RMT_TX_START 

        // if(REG_READ(RMT_INT_RAW_REG) & (1 << 0)) {
        //   REG_SET_BIT(RMT_INT_CLR_REG, 1);
        //   //delayMicroseconds(1);
        // }
        while(!(REG_READ(RMT_INT_RAW_REG))) {
           //delayMicroseconds(1);
        }
         REG_SET_BIT(RMT_INT_CLR_REG, 1);
      }

      delayMicroseconds(50);
}



void loop() {

  int initial = 1; 
  int totalSamples = sizeof(sampleArray) / sizeof(sampleArray[0]);
  uint32_t colors[5];

  for(int i = 0; i < totalSamples; i++) {
    int sample = sampleArray[i];
    //Serial.print("Sample = ");
    //Serial.println(sample);
    update_PWM(initial, sample);
    if(i % 15 == 0) {
      populateColors(colors, sample); // leds based on amplitude of signal
      transmit_led_signal(colors);
    }
    initial = 0;
  }
  
}
