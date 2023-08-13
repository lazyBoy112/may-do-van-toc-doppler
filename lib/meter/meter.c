#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "meter.h"
#include "driver/i2c.h"

#define SMBUS_DEFAULT_TIMEOUT (1000 / portTICK_RATE_MS) ///< Default transaction timeout in ticks
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK true
#define NO_ACK_CHECK false
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1
#define MAX_BLOCK_LEN 255 // SMBus v3.0 increases this from 32 to 255

// Control flags
#define FLAG_BACKLIGHT_ON 0b00001000  // backlight enabled (disabled if clear)
#define FLAG_BACKLIGHT_OFF 0b00000000 // backlight disabled
#define FLAG_ENABLE 0b00000100
#define FLAG_READ 0b00000010       // read (write if clear)
#define FLAG_WRITE 0b00000000      // write
#define FLAG_RS_DATA 0b00000001    // data (command if clear)
#define FLAG_RS_COMMAND 0b00000000 // command
// COMMAND_DISPLAY_CONTROL flags
#define FLAG_DISPLAY_CONTROL_DISPLAY_ON 0x04
#define FLAG_DISPLAY_CONTROL_DISPLAY_OFF 0x00
#define FLAG_DISPLAY_CONTROL_CURSOR_ON 0x02
#define FLAG_DISPLAY_CONTROL_CURSOR_OFF 0x00
#define FLAG_DISPLAY_CONTROL_BLINK_ON 0x01
#define FLAG_DISPLAY_CONTROL_BLINK_OFF 0x00
// COMMAND_ENTRY_MODE_SET flags
#define FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT 0x02
#define FLAG_ENTRY_MODE_SET_ENTRY_DECREMENT 0x00
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON 0x01
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF 0x00

// Delays (microseconds)
#define DELAY_POWER_ON 50000 // wait at least 40us after VCC rises to 2.7V
#define DELAY_INIT_1 4500    // wait at least 4.1ms (fig 24, page 46)
#define DELAY_INIT_2 4500    // wait at least 4.1ms (fig 24, page 46)
#define DELAY_INIT_3 120     // wait at least 100us (fig 24, page 46)

#define DELAY_CLEAR_DISPLAY 2000
#define DELAY_RETURN_HOME 2000

#define DELAY_ENABLE_PULSE_WIDTH 1   // enable pulse must be at least 450ns wide
#define DELAY_ENABLE_PULSE_SETTLE 50 // command requires > 37us to settle (table 6 in datasheet)
// Commands
#define COMMAND_CLEAR_DISPLAY 0x01
#define COMMAND_RETURN_HOME 0x02
#define COMMAND_ENTRY_MODE_SET 0x04
#define COMMAND_DISPLAY_CONTROL 0x08
#define COMMAND_SHIFT 0x10
#define COMMAND_FUNCTION_SET 0x20
#define COMMAND_SET_CGRAM_ADDR 0x40
#define COMMAND_SET_DDRAM_ADDR 0x80
// COMMAND_FUNCTION_SET flags
#define FLAG_FUNCTION_SET_MODE_8BIT 0x10
#define FLAG_FUNCTION_SET_MODE_4BIT 0x00
#define FLAG_FUNCTION_SET_LINES_2 0x08
#define FLAG_FUNCTION_SET_LINES_1 0x00
#define FLAG_FUNCTION_SET_DOTS_5X10 0x04
#define FLAG_FUNCTION_SET_DOTS_5X8 0x00

static const char *TAG = "bus";

static esp_err_t _check_i2c_error(esp_err_t err)
{
    switch (err)
    {
    case ESP_OK:  // Success
        break;
    case ESP_ERR_INVALID_ARG:  // Parameter error
        ESP_LOGE(TAG, "I2C parameter error");
        break;
    case ESP_FAIL: // Sending command error, slave doesn't ACK the transfer.
        ESP_LOGE(TAG, "I2C no slave ACK");
        break;
    case ESP_ERR_INVALID_STATE:  // I2C driver not installed or not in master mode.
        ESP_LOGE(TAG, "I2C driver not installed or not master");
        break;
    case ESP_ERR_TIMEOUT:  // Operation timeout because the bus is busy.
        ESP_LOGE(TAG, "I2C timeout");
        break;
    default:
        ESP_LOGE(TAG, "I2C error %d", err);
    }
    return err;
}

static bool _is_init_b(const smbus_info_t * smbus_info)
{
    bool ok = false;
    if (smbus_info != NULL)
    {
        if (smbus_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "smbus_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "smbus_info is NULL");
    }
    return ok;
}

static bool _is_init(const i2c_lcd1602_info_t * i2c_lcd1602_info)
{
    bool ok = false;
    if (i2c_lcd1602_info != NULL)
    {
        if (i2c_lcd1602_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "i2c_lcd1602_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "i2c_lcd1602_info is NULL");
    }
    return ok;
}

esp_err_t smbus_send_byte(const smbus_info_t * smbus_info, uint8_t data)
{
    // Protocol: [S | ADDR | Wr | As | DATA | As | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init_b(smbus_info))
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, smbus_info->address << 1 | WRITE_BIT, ACK_CHECK);
        i2c_master_write_byte(cmd, data, ACK_CHECK);
        i2c_master_stop(cmd);
        err = _check_i2c_error(i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout));
        i2c_cmd_link_delete(cmd);
    }
    return err;
}

static esp_err_t _write_to_expander(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    // backlight flag must be included with every write to maintain backlight state
    ESP_LOGD(TAG, "_write_to_expander 0x%02x", data | i2c_lcd1602_info->backlight_flag);
    return smbus_send_byte(i2c_lcd1602_info->smbus_info, data | i2c_lcd1602_info->backlight_flag);
}

static esp_err_t _strobe_enable(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data | FLAG_ENABLE);
    ets_delay_us(DELAY_ENABLE_PULSE_WIDTH);
    esp_err_t err2 = _write_to_expander(i2c_lcd1602_info, data & ~FLAG_ENABLE);
    ets_delay_us(DELAY_ENABLE_PULSE_SETTLE);
    return err1 ? err1 : err2;
}

static esp_err_t _write_top_nibble(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    ESP_LOGD(TAG, "_write_top_nibble 0x%02x", data);
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data);
    esp_err_t err2 = _strobe_enable(i2c_lcd1602_info, data);
    return err1 ? err1 : err2;
}

static esp_err_t _write(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t value, uint8_t register_select_flag)
{
    ESP_LOGD(TAG, "_write 0x%02x | 0x%02x", value, register_select_flag);
    esp_err_t err1 = _write_top_nibble(i2c_lcd1602_info, (value & 0xf0) | register_select_flag);
    esp_err_t err2 = _write_top_nibble(i2c_lcd1602_info, ((value & 0x0f) << 4) | register_select_flag);
    return err1 ? err1 : err2;
}

// send command to controller
static esp_err_t _write_command(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t command)
{
    ESP_LOGD(TAG, "_write_command 0x%02x", command);
    return _write(i2c_lcd1602_info, command, FLAG_RS_COMMAND);
}

// send data to controller
static esp_err_t _write_data(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    ESP_LOGD(TAG, "_write_data 0x%02x", data);
    return _write(i2c_lcd1602_info, data, FLAG_RS_DATA);
}

esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t * i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        err = _write_command(i2c_lcd1602_info, COMMAND_RETURN_HOME);
        if (err == ESP_OK)
        {
            ets_delay_us(DELAY_RETURN_HOME);
        }
    }
    return err;
}

esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t * i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        err = _write_command(i2c_lcd1602_info, COMMAND_CLEAR_DISPLAY);
        if (err == ESP_OK)
        {
            ets_delay_us(DELAY_CLEAR_DISPLAY);
        }
    }
    return err;
}

esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t * i2c_lcd1602_info)
{
    esp_err_t first_err = ESP_OK;
    esp_err_t last_err = ESP_FAIL;

    // put Expander into known state - Register Select and Read/Write both low
    if ((last_err = _write_to_expander(i2c_lcd1602_info, 0)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_to_expander 1 failed: %d", last_err);
    }

    ets_delay_us(1000);

    // select 4-bit mode on LCD controller - see datasheet page 46, figure 24.
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 1 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_1);

    // repeat
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 2 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_2);

    // repeat
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 3 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_3);

    // select 4-bit mode
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x02 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 4 failed: %d", last_err);
    }

    // now we can use the command()/write() functions
    if ((last_err = _write_command(i2c_lcd1602_info, COMMAND_FUNCTION_SET | FLAG_FUNCTION_SET_MODE_4BIT | FLAG_FUNCTION_SET_LINES_2 | FLAG_FUNCTION_SET_DOTS_5X8)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 1 failed: %d", last_err);
    }

    if ((last_err = _write_command(i2c_lcd1602_info, COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 2 failed: %d", last_err);
    }

    if ((last_err = i2c_lcd1602_clear(i2c_lcd1602_info)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_clear failed: %d", last_err);
    }

    if ((last_err = _write_command(i2c_lcd1602_info, COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 3 failed: %d", last_err);
    }

    if ((last_err = i2c_lcd1602_home(i2c_lcd1602_info)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_home failed: %d", last_err);
    }

    return first_err;
}

smbus_info_t * smbus_malloc(void)
{
    smbus_info_t * smbus_info = malloc(sizeof(*smbus_info));
    if (smbus_info != NULL)
    {
        memset(smbus_info, 0, sizeof(*smbus_info));
        ESP_LOGD(TAG, "malloc smbus_info_t %p", smbus_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc smbus_info_t failed");
    }
    return smbus_info;
}

esp_err_t smbus_init(smbus_info_t * smbus_info, i2c_port_t i2c_port, i2c_address_t address)
{
    if (smbus_info != NULL)
    {
        smbus_info->i2c_port = i2c_port;
        smbus_info->address = address;
        smbus_info->timeout = SMBUS_DEFAULT_TIMEOUT;
        smbus_info->init = true;
    }
    else
    {
        ESP_LOGE(TAG, "smbus_info is NULL");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t smbus_set_timeout(smbus_info_t * smbus_info, portBASE_TYPE timeout)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init_b(smbus_info))
    {
        smbus_info->timeout = timeout;
        err = ESP_OK;
    }
    return err;
}

i2c_lcd1602_info_t * i2c_lcd1602_malloc(void)
{
    i2c_lcd1602_info_t * i2c_lcd1602_info = malloc(sizeof(*i2c_lcd1602_info));
    if (i2c_lcd1602_info != NULL)
    {
        memset(i2c_lcd1602_info, 0, sizeof(*i2c_lcd1602_info));
        ESP_LOGD(TAG, "malloc i2c_lcd1602_info_t %p", i2c_lcd1602_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc i2c_lcd1602_info_t failed");
    }
    return i2c_lcd1602_info;
}

esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t * i2c_lcd1602_info, smbus_info_t * smbus_info,
                           bool backlight, uint8_t num_rows, uint8_t num_columns, uint8_t num_visible_columns)
{
    esp_err_t err = ESP_FAIL;
    if (i2c_lcd1602_info != NULL)
    {
        i2c_lcd1602_info->smbus_info = smbus_info;
        i2c_lcd1602_info->backlight_flag = backlight ? FLAG_BACKLIGHT_ON : FLAG_BACKLIGHT_OFF;
        i2c_lcd1602_info->num_rows = num_rows;
        i2c_lcd1602_info->num_columns = num_columns;
        i2c_lcd1602_info->num_visible_columns = num_visible_columns;

        // display on, no cursor, no blinking
        i2c_lcd1602_info->display_control_flags = FLAG_DISPLAY_CONTROL_DISPLAY_ON | FLAG_DISPLAY_CONTROL_CURSOR_OFF | FLAG_DISPLAY_CONTROL_BLINK_OFF;

        // left-justified left-to-right text
        i2c_lcd1602_info->entry_mode_flags = FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT | FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF;

        i2c_lcd1602_info->init = true;

        // See page 45/46 of HD44780 data sheet for the initialisation procedure.

        // Wait at least 40ms after power rises above 2.7V before sending commands.
        ets_delay_us(DELAY_POWER_ON);

        err = i2c_lcd1602_reset(i2c_lcd1602_info);
    }
    else
    {
        ESP_LOGE(TAG, "i2c_lcd1602_info is NULL");
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t * i2c_lcd1602_info, const char * string)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        //ESP_LOGI(TAG, "i2c_lcd1602_write_string: %s", string);
        err = ESP_OK;
        for (int i = 0; err == ESP_OK && string[i]; ++i)
        {
            err = _write_data(i2c_lcd1602_info, string[i]);
        }
    }
    return err;
}


esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t col, uint8_t row)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
        if (row > i2c_lcd1602_info->num_rows)
        {
            row = i2c_lcd1602_info->num_rows - 1;
        }
        if (col > i2c_lcd1602_info->num_columns)
        {
            col = i2c_lcd1602_info->num_columns - 1;
        }
        err = _write_command(i2c_lcd1602_info, COMMAND_SET_DDRAM_ADDR | (col + row_offsets[row]));
    }
    return err;
}
