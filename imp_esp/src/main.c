#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "api_ssd1306/font8x8_basic.h"
#include "api_ssd1306/ssd1306.h"
#include "driver/i2c.h"
#include "API/max30102.h"


#define OLED_MOSI  19
#define OLED_CLK   18
#define OLED_DC    13
#define OLED_CS    12
#define OLED_RESET 23

#define I2C_MASTER_SCL_IO 14              
#define I2C_MASTER_SDA_IO 27              
#define I2C_MASTER_FREQ_HZ 100000
static MAX30102_DEVICE senzor;
static int32_t vzorky[MAX30102_BPM_SAMPLES_SIZE];

void app_main() {
    //inicializace displej
    SSD1306_t dev;
    spi_master_init(&dev, OLED_MOSI, OLED_CLK, OLED_CS, OLED_DC, OLED_RESET);
    spi_init(&dev, 128, 64);
    char str[10] = "BPM: ";

    //inicializace MAX30102
    senzor.read = esp32c3_read_max30102;
    senzor.write = esp32c3_write_max30102;
    senzor.delay_us = esp32c3_delay_us_max30102;

    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    max30102_set_spo2(MAX30102_SPO2_RANGE_16384 | MAX30102_SPO2_50_SPS | MAX30102_SPO2_LED_PW_411, &senzor);
    senzor.delay_us(40000);
    max30102_set_fifo(MAX30102_SMP_AVE_NO, &senzor);
    senzor.delay_us(40000);
    //nastaveni LED - nefunkcni
    max30102_set_led_amplitude(0x0F, &senzor);
    senzor.delay_us(40000);

    uint8_t bpmVzorky[8];
    uint32_t bpmAvg = 0;
    const uint8_t bpmFilter = 4;
    max30102_set_sensor_mode(MAX30102_HR_MODE, &senzor);

    // hlavni smycka
    while (1) 
    {
        //sber vzorku
        MAX30102_DATA tmp;
        for (int i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            max30102_get_sensor_data(MAX30102_BPM, &tmp, &senzor);
            vzorky[i] = tmp.bpm32;
            senzor.delay_us(20000);
        }

        //vyhlazovani dat ze 4 vzorku
        bpmAvg = 0;
        for (int i = bpmFilter - 1; i > 0; i--){
            bpmVzorky[i] = bpmVzorky[i-1];
        }
        bpmVzorky[0] = max30102_get_bpm(vzorky);
        for (int i = 0; i < bpmFilter; i++){
            bpmAvg += bpmVzorky[i];
        }
        bpmAvg /= bpmFilter;
    
        //vypis
        printf("BPM: %d\n", bpmAvg);
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 1, str, 8, false);
        str[5] = (bpmAvg/100)+'0';
        str[6] = ((bpmAvg%100)/10)+'0';
        str[7] = (bpmAvg%10)+'0';
    }

}