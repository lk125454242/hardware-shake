#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "sdkconfig.h"

static const char *TAG = "main";

/* 电机由 I2C 从机驱动；转速为模拟值 1~100，不计算脉冲，具体 RPM 由从机处理 */
#define SPEED_MIN   1
#define SPEED_MAX   100
#define SPEED_DEFAULT 50
#define COUNTDOWN_DEFAULT_SEC 300
#define BUTTON_DEBOUNCE_MS   80
#define DISPLAY_REFRESH_MS   100

#define I2C_MASTER_FREQ_HZ     400000
#define OLED_WIDTH   128

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_oled_handle = NULL;
static i2c_master_dev_handle_t s_motor_handle = NULL;
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

/* 电机与倒计时状态（多任务共享，仅简单读写） */
static volatile int s_motor_running = 0;
static volatile int s_speed = SPEED_DEFAULT;   /* 转速模拟值 1~100，发从机 0x02,speed，具体计算由从机处理 */
static volatile int s_countdown_sec = COUNTDOWN_DEFAULT_SEC;
static volatile uint32_t s_buzz_until_ms = 0;

/* 6x8 字体，0-9、A-S、常用符号 */
static const uint8_t font_6x8[][6] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, /* space 0x20 */
    { 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00 }, /* ! */
    { 0x00, 0x07, 0x00, 0x07, 0x00, 0x00 }, /* " */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00 }, /* # */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00 }, /* $ */
    { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00 }, /* % */
    { 0x36, 0x49, 0x56, 0x20, 0x50, 0x00 }, /* & */
    { 0x00, 0x08, 0x07, 0x03, 0x00, 0x00 }, /* ' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00 }, /* ( */
    { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00 }, /* ) */
    { 0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x00 }, /* * */
    { 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00 }, /* + */
    { 0x00, 0x80, 0x70, 0x30, 0x00, 0x00 }, /* , */
    { 0x08, 0x08, 0x08, 0x08, 0x08, 0x00 }, /* - */
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 }, /* . 0x2E */
    { 0x20, 0x10, 0x08, 0x04, 0x02, 0x00 }, /* / */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00 }, /* 0 0x30 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00 }, /* 1 */
    { 0x72, 0x49, 0x49, 0x49, 0x46, 0x00 }, /* 2 */
    { 0x22, 0x49, 0x49, 0x49, 0x36, 0x00 }, /* 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00 }, /* 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39, 0x00 }, /* 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x31, 0x00 }, /* 6 */
    { 0x41, 0x21, 0x11, 0x09, 0x07, 0x00 }, /* 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36, 0x00 }, /* 8 */
    { 0x46, 0x49, 0x49, 0x29, 0x1E, 0x00 }, /* 9 */
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 }, /* : 0x3A */
    { 0x00, 0x80, 0x76, 0x36, 0x00, 0x00 }, /* ; */
    { 0x08, 0x14, 0x22, 0x41, 0x00, 0x00 }, /* < */
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x00 }, /* = */
    { 0x00, 0x41, 0x22, 0x14, 0x08, 0x00 }, /* > */
    { 0x02, 0x01, 0x59, 0x09, 0x06, 0x00 }, /* ? */
    { 0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x00 }, /* @ */
    { 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00 }, /* A */
    { 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00 }, /* B */
    { 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00 }, /* C */
    { 0x7F, 0x41, 0x41, 0x41, 0x3E, 0x00 }, /* D */
    { 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00 }, /* E */
    { 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00 }, /* F */
    { 0x3E, 0x41, 0x41, 0x51, 0x73, 0x00 }, /* G */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00 }, /* H */
    { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00 }, /* I 0x49 */
    { 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00 }, /* J */
    { 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00 }, /* K */
    { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00 }, /* L */
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00 }, /* M */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00 }, /* N */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00 }, /* O */
    { 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00 }, /* P 0x50 */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00 }, /* Q 0x51 */
    { 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00 }, /* R 0x52 */
    { 0x46, 0x49, 0x49, 0x49, 0x31, 0x00 }, /* S 0x53 */
    { 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00 }, /* T 0x54 */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00 }, /* U 0x55 */
    { 0x07, 0x08, 0x10, 0x20, 0x40, 0x3F }, /* V 0x56 */
    { 0x7F, 0x40, 0x20, 0x10, 0x20, 0x40 }, /* W 0x57 */
};

#define FONT_FIRST 0x20
#define FONT_LAST  0x57
#define FONT_COLS  6

static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[] = { 0x00, cmd };  /* 0x00 = command */
    esp_err_t ret = i2c_master_transmit(s_oled_handle, buf, sizeof(buf), pdMS_TO_TICKS(100));
    ESP_LOGD(TAG, "I2C TX [OLED 0x%02X] cmd=0x%02X %s", (unsigned)CONFIG_OLED_I2C_ADDR, (unsigned)cmd,
             ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    return ret;
}

static esp_err_t oled_write_data(const uint8_t *data, size_t len)
{
    if (len == 0) return ESP_OK;
    uint8_t buf[1 + OLED_WIDTH];
    buf[0] = 0x40;  /* 0x40 = data */
    if (len > OLED_WIDTH) len = OLED_WIDTH;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(s_oled_handle, buf, 1 + len, pdMS_TO_TICKS(200));
    ESP_LOGD(TAG, "I2C TX [OLED 0x%02X] data len=%u %s", (unsigned)CONFIG_OLED_I2C_ADDR, (unsigned)len,
             ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    return ret;
}

static esp_err_t oled_init(void)
{
    int sda = CONFIG_OLED_I2C_SDA_GPIO;
    int scl = CONFIG_OLED_I2C_SCL_GPIO;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = 0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus 创建失败: %s (SDA=%d SCL=%d)", esp_err_to_name(ret), sda, scl);
        return ret;
    }

    i2c_device_config_t oled_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_OLED_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus_handle, &oled_cfg, &s_oled_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 从设备添加失败: OLED 0x%02X, %s", (unsigned)CONFIG_OLED_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C 从设备已挂载: OLED 0x%02X (SDA=%d SCL=%d)", (unsigned)CONFIG_OLED_I2C_ADDR, sda, scl);

    i2c_device_config_t motor_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_MOTOR_SLAVE_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus_handle, &motor_cfg, &s_motor_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 从设备添加失败: 电机从机 0x%02X, %s", (unsigned)CONFIG_MOTOR_SLAVE_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C 从设备已挂载: 电机从机 0x%02X", (unsigned)CONFIG_MOTOR_SLAVE_I2C_ADDR);

    vTaskDelay(pdMS_TO_TICKS(50));  /* 上电后等待 OLED 就绪 */

    /* SSD1306 128x64 初始化命令 */
    const uint8_t init[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* set osc div */
        0xA8, 0x3F, /* multiplex 64-1 */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line 0 */
        0x8D, 0x14, /* charge pump on */
        0x20, 0x00, /* horizontal addressing */
        0xA1,       /* segment remap */
        0xC8,       /* com scan dec */
        0xDA, 0x12, /* com pins */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* precharge */
        0xDB, 0x40, /* vcomh */
        0xAF        /* display on */
    };
    for (size_t i = 0; i < sizeof(init); i++) {
        ret = oled_write_cmd(init[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OLED 写命令 0x%02x 失败: %s (地址 0x%02X)", (unsigned)init[i], esp_err_to_name(ret), (unsigned)CONFIG_OLED_I2C_ADDR);
            return ret;
        }
    }
    ESP_LOGI(TAG, "I2C OLED (0x%02X) 检测/通信正常，初始化完成", (unsigned)CONFIG_OLED_I2C_ADDR);
    return ESP_OK;
}

static esp_err_t oled_clear_page(uint8_t page)
{
    oled_write_cmd(0x21); oled_write_cmd(0); oled_write_cmd(OLED_WIDTH - 1);
    oled_write_cmd(0x22); oled_write_cmd(page); oled_write_cmd(page);
    uint8_t buf[OLED_WIDTH];
    memset(buf, 0, sizeof(buf));
    return oled_write_data(buf, sizeof(buf));
}

static uint8_t oled_get_glyph(unsigned char c, int col)
{
    if (c < FONT_FIRST || c > FONT_LAST || col < 0 || col >= FONT_COLS)
        return 0;
    return font_6x8[c - FONT_FIRST][col];
}

static esp_err_t oled_draw_string_line(uint8_t page, const char *str)
{
    oled_write_cmd(0x21); oled_write_cmd(0); oled_write_cmd(OLED_WIDTH - 1);
    oled_write_cmd(0x22); oled_write_cmd(page); oled_write_cmd(page);

    uint8_t buf[OLED_WIDTH];
    memset(buf, 0, sizeof(buf));
    size_t x = 0;
    for (const char *p = str; *p && x + FONT_COLS <= OLED_WIDTH; p++) {
        for (int col = 0; col < FONT_COLS; col++)
            buf[x + col] = oled_get_glyph((unsigned char)*p, col);
        x += FONT_COLS;
    }
    return oled_write_data(buf, sizeof(buf));
}

/* 向电机从机写指令（与 OLED 共用 I2C 总线，新驱动 driver/i2c_master.h） */
static esp_err_t motor_slave_write(const uint8_t *data, size_t len)
{
    if (len == 0 || !s_motor_handle) return ESP_OK;
    esp_err_t ret = i2c_master_transmit(s_motor_handle, data, len, pdMS_TO_TICKS(50));
    if (len <= 8) {
        char hex[32];
        char *p = hex;
        for (size_t i = 0; i < len && p - hex < (int)sizeof(hex) - 4; i++)
            p += sprintf(p, "%02X ", (unsigned)data[i]);
        ESP_LOGI(TAG, "I2C TX [电机从机 0x%02X] len=%u data:[%s] %s", (unsigned)CONFIG_MOTOR_SLAVE_I2C_ADDR, (unsigned)len, hex, ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C TX [电机从机 0x%02X] len=%u first=0x%02X %s", (unsigned)CONFIG_MOTOR_SLAVE_I2C_ADDR, (unsigned)len, (unsigned)data[0], ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    }
    return ret;
}

/* 检查所有外设/按钮配置的 GPIO 是否重复，若有则打日志并中止 */
static void check_gpio_duplicates(void)
{
    struct { int gpio; const char *name; } pins[] = {
        { CONFIG_OLED_I2C_SDA_GPIO,    "OLED SDA" },
        { CONFIG_OLED_I2C_SCL_GPIO,    "OLED SCL" },
        { CONFIG_BTN_SPEED_UP_GPIO,    "BTN SPEED_UP" },
        { CONFIG_BTN_SPEED_DOWN_GPIO,  "BTN SPEED_DOWN" },
        { CONFIG_BTN_MOTOR_START_GPIO,"BTN START" },
        { CONFIG_BTN_MOTOR_STOP_GPIO,  "BTN STOP" },
        { CONFIG_BUZZER_GPIO,          "BUZZER" },
    };
    const int n = sizeof(pins) / sizeof(pins[0]);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (pins[i].gpio == pins[j].gpio) {
                ESP_LOGE(TAG, "配置错误：GPIO %d 被重复使用：\"%s\" 与 \"%s\"。请修改 menuconfig 中引脚配置。",
                         pins[i].gpio, pins[i].name, pins[j].name);
                abort();
            }
        }
    }
}

static void peripheral_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CONFIG_BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(CONFIG_BUZZER_GPIO, 0);

    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << CONFIG_BTN_SPEED_UP_GPIO) |
                        (1ULL << CONFIG_BTN_SPEED_DOWN_GPIO) |
                        (1ULL << CONFIG_BTN_MOTOR_START_GPIO) |
                        (1ULL << CONFIG_BTN_MOTOR_STOP_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
}

/* 倒计时任务：电机由从机驱动，此处只做计时；到点发停止指令并触发蜂鸣 */
static void countdown_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_motor_running) continue;
        s_countdown_sec--;
        if (s_countdown_sec <= 0) {
            s_countdown_sec = 0;
            s_motor_running = 0;
            uint8_t stop_cmd = 0x00;
            motor_slave_write(&stop_cmd, 1);
            s_buzz_until_ms = (uint32_t)(esp_timer_get_time() / 1000) + 2000;
        }
    }
}

static int button_pressed(int gpio)
{
    return gpio_get_level(gpio) == 0; /* 低电平 = 按下 */
}

static void button_task(void *arg)
{
    uint32_t last_up = 0, last_down = 0, last_start = 0, last_stop = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15));
        uint32_t t = (uint32_t)(esp_timer_get_time() / 1000);

        if (button_pressed(CONFIG_BTN_SPEED_UP_GPIO)) {
            if (t - last_up > BUTTON_DEBOUNCE_MS) {
                last_up = t;
                if (s_speed < SPEED_MAX) {
                    s_speed++;
                    uint8_t cmd[] = { 0x02, (uint8_t)s_speed };
                    motor_slave_write(cmd, sizeof(cmd));
                }
            }
        }
        if (button_pressed(CONFIG_BTN_SPEED_DOWN_GPIO)) {
            if (t - last_down > BUTTON_DEBOUNCE_MS) {
                last_down = t;
                if (s_speed > SPEED_MIN) {
                    s_speed--;
                    uint8_t cmd[] = { 0x02, (uint8_t)s_speed };
                    motor_slave_write(cmd, sizeof(cmd));
                }
            }
        }
        if (button_pressed(CONFIG_BTN_MOTOR_START_GPIO)) {
            if (t - last_start > BUTTON_DEBOUNCE_MS) {
                last_start = t;
                s_motor_running = 1;
                s_countdown_sec = COUNTDOWN_DEFAULT_SEC;
                uint8_t start_cmd = 0x01;
                motor_slave_write(&start_cmd, 1);
                uint8_t speed_cmd[] = { 0x02, (uint8_t)s_speed };
                motor_slave_write(speed_cmd, sizeof(speed_cmd));
            }
        }
        if (button_pressed(CONFIG_BTN_MOTOR_STOP_GPIO)) {
            if (t - last_stop > BUTTON_DEBOUNCE_MS) {
                last_stop = t;
                s_motor_running = 0;
                uint8_t stop_cmd = 0x00;
                motor_slave_write(&stop_cmd, 1);
            }
        }
    }
}

static void display_task(void *arg)
{
    char line0[24], line1[24], line2[24], line3[24];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int buzz_on = 0;
        if (s_buzz_until_ms != 0) {
            if (now_ms < s_buzz_until_ms) {
                gpio_set_level(CONFIG_BUZZER_GPIO, 1);
                buzz_on = 1;
            } else {
                gpio_set_level(CONFIG_BUZZER_GPIO, 0);
                s_buzz_until_ms = 0;
            }
        } else {
            gpio_set_level(CONFIG_BUZZER_GPIO, 0);
        }
        int c = s_countdown_sec;
        int m = c / 60, s = c % 60;
        snprintf(line0, sizeof(line0), "STATE %s", s_motor_running ? "RUN" : "STOP");
        snprintf(line1, sizeof(line1), "RPM %d", s_speed);  /* 1~100 模拟值，具体 RPM 由从机计算 */
        snprintf(line2, sizeof(line2), "TIME %02d:%02d", m, s);
        snprintf(line3, sizeof(line3), "BEEP %s", buzz_on ? "ON " : "OFF");
        oled_draw_string_line(0, line0);
        oled_draw_string_line(1, line1);
        oled_draw_string_line(2, line2);
        oled_draw_string_line(3, line3);
    }
}

void app_main(void)
{
    esp_err_t err = oled_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED 初始化失败: %s", esp_err_to_name(err));
        return;
    }

    check_gpio_duplicates();
    peripheral_gpio_init();

    /* OLED 四行：STATE / SPEED / TIME / BEEP（限位与转速由从机处理） */
    for (uint8_t p = 0; p < OLED_PAGES; p++)
        oled_clear_page(p);
    char line0[24];
    int c0 = s_countdown_sec;
    oled_draw_string_line(0, "STATE STOP");
    snprintf(line0, sizeof(line0), "RPM %d", s_speed);
    oled_draw_string_line(1, line0);
    snprintf(line0, sizeof(line0), "TIME %02d:%02d", c0 / 60, c0 % 60);
    oled_draw_string_line(2, line0);
    oled_draw_string_line(3, "BEEP OFF");

    xTaskCreate(countdown_task, "countdown", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 6, NULL);
    xTaskCreate(display_task, "display", 2048, NULL, 6, NULL);
    ESP_LOGI(TAG, "电机由 I2C 从机 0x%02X 驱动，OLED/按钮/倒计时已启动", (unsigned)CONFIG_MOTOR_SLAVE_I2C_ADDR);
}
