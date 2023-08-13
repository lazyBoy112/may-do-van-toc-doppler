#include <stdbool.h>
#include <string.h>

#define portBASE_TYPE int
#define I2C_LCD1602_H

typedef int i2c_port_t;
typedef uint16_t i2c_address_t;

typedef struct
{
    bool init;             ///< True if struct has been initialised, otherwise false
    i2c_port_t i2c_port;   ///< ESP-IDF I2C port number
    i2c_address_t address; ///< I2C address of slave device
    portBASE_TYPE timeout; ///< Number of ticks until I2C operation timeout
} smbus_info_t;

typedef struct
{
    bool init;                     ///< True if struct has been initialised, otherwise false
    smbus_info_t *smbus_info;      ///< Pointer to associated SMBus info
    uint8_t backlight_flag;        ///< Non-zero if backlight is to be enabled, otherwise
                                   ///< zero
    uint8_t num_rows;              ///< Number of configured columns
    uint8_t num_columns;           ///< Number of configured columns, including offscreen
                                   ///< columns
    uint8_t num_visible_columns;   ///< Number of visible columns
    uint8_t display_control_flags; ///< Currently active display control flags
    uint8_t entry_mode_flags;      ///< Currently active entry mode flags
} i2c_lcd1602_info_t;

smbus_info_t *smbus_malloc(void); // cấp phát bộ nhớ 
esp_err_t smbus_init(smbus_info_t *smbus_info, i2c_port_t i2c_port, i2c_address_t address); // khởi tạo các giá trị của smbus
esp_err_t smbus_set_timeout(smbus_info_t *smbus_info, portBASE_TYPE timeout); // thiết lập thời gian 

i2c_lcd1602_info_t *i2c_lcd1602_malloc(void); // cấp phát bộ nhớ
esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t *i2c_lcd1602_info, smbus_info_t *smbus_info, bool backlight, uint8_t num_rows, uint8_t num_columns, uint8_t num_visible_columns); // khởi tạo các tham số
esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t *i2c_lcd1602_info); // đặt lại các giá trị mạc định cho lcd
esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t *i2c_lcd1602_info); 
esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t *i2c_lcd1602_info, const char *string); // viết một đoạn string lên màn hình lcd
esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t *i2c_lcd1602_info); // xóa các dữ liệu trên màn hình
esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t *i2c_lcd1602_info,uint8_t col, uint8_t row); // thiết lập vị trí con trở
