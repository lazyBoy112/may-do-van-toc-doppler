#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/uart.h"
#include "sdkconfig.h"
#include "soc/pcnt_struct.h"
#include <math.h>
#include "driver/pcnt.h"
#include <stdio.h>
#include "meter.h"

#define TAG "app"
#define PCNT_INPUT_SIG_IO GPIO_NUM_34
#define PCNT_INPUT_CTRL_IO GPIO_NUM_35
#define PCNT_COUNT_UNIT PCNT_UNIT_0
#define PCNT_COUNT_CHANNEL PCNT_CHANNEL_0
#define PCNT_H_LIM_VAL overflow
#define PCNT_COUNT_INC PCNT_CHANNEL_EDGE_ACTION_INCREASE
#define PCNT_MODE_DISABLE PCNT_CHANNEL_LEVEL_ACTION_HOLD

#define IN_BOARD_LED GPIO_NUM_2 // ESP32 native LED - GPIO 2
#define false 0
#define true 1

#define PCNT_COUNT_UNIT PCNT_UNIT_0       // Set Pulse Counter Unit - 0
#define PCNT_COUNT_CHANNEL PCNT_CHANNEL_0 // Set Pulse Counter channel - 0

#define PCNT_INPUT_SIG_IO GPIO_NUM_34   // Set Pulse Counter input - Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO GPIO_NUM_33    // Saida do LEDC - gerador de pulsos - GPIO_33
#define PCNT_INPUT_CTRL_IO GPIO_NUM_35  // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down
#define OUTPUT_CONTROL_GPIO GPIO_NUM_32 // Timer output control port - GPIO_32
#define PCNT_H_LIM_VAL overflow         // Overflow of Pulse Counter

// LCD1602
#define LCD_NUM_ROWS 2
#define LCD_NUM_COLUMNS 32
#define LCD_NUM_VISIBLE_COLUMNS 16

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN 0 // disabled
#define I2C_MASTER_RX_BUF_LEN 0 // disabled
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL
#define SCL_GPIO GPIO_NUM_22
#define SDA_GPIO GPIO_NUM_21

// register var for meter
bool flag = true;               // Flag to enable print frequency reading
uint32_t overflow = 20000;      // Max Pulse Counter value
int16_t pulses = 0;             // Pulse Counter value
uint32_t multPulses = 0;        // Quantidade de overflows do contador PCNT
uint32_t sample_time = 1000000; // sample time of 1 second to count pulses
float frequency = 0;            // frequency value
char buf[16];                   // Buffer_CHANNEL_LEVEL_ACTION_KEEP

esp_timer_create_args_t create_args; // Create an esp_timer instance
esp_timer_handle_t timer_handle;     // Create an single timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR pcnt_intr_handler(void *arg) // Counting overflow pulses
{
  portENTER_CRITICAL_ISR(&timerMux);       // disabling the interrupts
  multPulses++;                            // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT); // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timerMux);        // enabling the interrupts
}

void read_PCNT(void *p) // Read Pulse Counter
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);           // Stop counter - output control LOW
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses); // Read Pulse Counter value
  flag = true;                                      // Change flag to enable print
}

void reverse(char *str, int len)
{
  int i = 0, j = len - 1, temp;
  while (i < j)
  {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++;
    j--;
  }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
  int i = 0;
  while (x)
  {
    str[i++] = (x % 10) + '0';
    x = x / 10;
  }

  // If number of digits required is more, then
  // add 0s at the beginning
  while (i < d)
    str[i++] = '0';

  reverse(str, i);
  str[i] = '\0';
  return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char *res, int afterpoint)
{
  // Extract integer part
  int ipart = (int)n;

  // Extract floating part
  float fpart = n - (float)ipart;

  // convert integer part to string
  int i = intToStr(ipart, res, 0);

  // check for display option after point
  if (afterpoint != 0)
  {
    res[i] = '.'; // add dot

    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter
    // is needed to handle cases like 233.007
    fpart = fpart * pow(10, afterpoint);

    intToStr((int)fpart, res + i + 1, afterpoint);
  }
}

void meter_init(void)
{
  ESP_LOGI(TAG, "initiazl....");

  pcnt_config_t pcnt_config = {}; // PCNT unit instance

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO; // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO; // Control signal input GPIO 35
  pcnt_config.unit = PCNT_COUNT_UNIT;             // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;       // PCNT unit number - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;     // Maximum counter value - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;          // PCNT positive edge count mode - inc
  pcnt_config.neg_mode = PCNT_COUNT_INC;          // PCNT negative edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;     // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;        // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                 // Initialize PCNT unit

  pcnt_counter_pause(PCNT_COUNT_UNIT); // Pause PCNT unit
  pcnt_counter_clear(PCNT_COUNT_UNIT); // Clear PCNT unit
  ESP_LOGI(TAG, "begin sampling");
  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);  // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL); // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT);                   // Enable interrupts for PCNT unit

  pcnt_counter_resume(PCNT_COUNT_UNIT);

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                 // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT); // Set GPIO 32 as output

  create_args.callback = read_PCNT;              // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle); // Create esp-timer instance

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT); // Set LED inboard as output

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);    // Set GPIO matrin IN - Freq Meter input
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false); // Set GPIO matrix OUT - to inboard LED
}

static void i2c_master_init(void)
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = GPIO_NUM_21;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
  conf.scl_io_num = GPIO_NUM_22;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE; // GY-2561 provides 10kΩ pullups
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_MASTER_RX_BUF_LEN,
                     I2C_MASTER_TX_BUF_LEN, 0);
}

void meter_task(void *pvParameter)
{
  meter_init(); // init frequency meter
  i2c_master_init(); //init i2c 
  // Set up the LCD1602 device with backlight off
  i2c_port_t i2c_num = I2C_MASTER_NUM;
  uint8_t address = 0x27;

  // Set up the SMBus
  smbus_info_t *smbus_info = smbus_malloc();
  ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
  ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

  // Set up the LCD1602 device with backlight off
  i2c_lcd1602_info_t *lcd_info = i2c_lcd1602_malloc();
  ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                   LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));
  ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));
  ESP_LOGI(TAG, "complete setup");
  i2c_lcd1602_home(lcd_info);
  i2c_lcd1602_write_string(lcd_info, "Started!");

  while (true)
  {
    if (flag == true) // If count has ended
    {
      flag = false; // change flag to disable print
      frequency = (pulses + (multPulses * overflow)) / 2;
      ESP_LOGI(TAG, "freq: %d", (int)frequency);
      for (int i = 0; i < 16; i++)
      {
        buf[i] = '\0';
      }
      itoa((int)(frequency), buf, 10);
      // Calculation of frequency
      i2c_lcd1602_clear(lcd_info); // Print unity Hz
      i2c_lcd1602_home(lcd_info);
      i2c_lcd1602_write_string(lcd_info, "Freq: ");
      i2c_lcd1602_move_cursor(lcd_info, 6, 0);
      i2c_lcd1602_write_string(lcd_info, buf);
      i2c_lcd1602_move_cursor(lcd_info, 12, 0);
      i2c_lcd1602_write_string(lcd_info, "Hz");
      ftoa((frequency / 44), buf, 2);
      i2c_lcd1602_move_cursor(lcd_info, 0, 1);
      i2c_lcd1602_write_string(lcd_info, "v: ");
      i2c_lcd1602_move_cursor(lcd_info, 6, 1);
      i2c_lcd1602_write_string(lcd_info, buf);
      i2c_lcd1602_move_cursor(lcd_info, 12, 1);
      i2c_lcd1602_write_string(lcd_info, "Km/h");
      multPulses = 0; // Clear overflow counter
      // Put your function here, if you want
      // vTaskDelay(100/portTICK_PERIOD_MS);                                                        // Delay 100 ms
      // Put your function here, if you want

      pcnt_counter_clear(PCNT_COUNT_UNIT);             // Clear Pulse Counter
      esp_timer_start_once(timer_handle, sample_time); // Initialize High resolution timer (1 sec)
      gpio_set_level(OUTPUT_CONTROL_GPIO, 1);          // Set enable PCNT count
    }
  }
  vTaskDelete(NULL);
}
void app_main()
{

  xTaskCreate(&meter_task, "meter_task", 4096, NULL, 5, NULL);
}
