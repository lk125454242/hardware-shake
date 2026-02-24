#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "sdkconfig.h"

static const char *TAG = "main";

/* TMC2209 步进电机：速度与倒计时，转速 1~300 RPM（步/秒依 MOTOR_STEPS_PER_REV 换算） */
#define MOTOR_SPEED_MIN      4    /* 最小步/秒 ≈ 1 RPM @200步/圈 */
#define MOTOR_SPEED_MAX      1000 /* 最大步/秒 = 300 RPM @200步/圈 */
#define MOTOR_SPEED_DEFAULT  100
#define COUNTDOWN_DEFAULT_SEC 300 /* 默认 5 分钟 */
#define BUTTON_DEBOUNCE_MS   80
#define DISPLAY_REFRESH_MS   200

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
#define WIFI_MAX_RETRY 5
static esp_ip4_addr_t s_ip_addr;

/* 电机与倒计时状态（多任务共享，仅简单读写） */
static volatile int s_motor_running = 0;
static volatile int s_speed_steps_per_sec = MOTOR_SPEED_DEFAULT;
static volatile int s_countdown_sec = COUNTDOWN_DEFAULT_SEC;
static char s_ip_str[20] = "0.0.0.0";

/* 6x8 字体，仅包含 0-9 . : 空格 I P，用于 "IP: x.x.x.x" */
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
};

#define FONT_FIRST 0x20
#define FONT_LAST  0x53
#define FONT_COLS  6

static esp_err_t oled_write_cmd(uint8_t cmd)
{
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    if (!h) return ESP_FAIL;
    esp_err_t ret = i2c_master_start(h)
        || i2c_master_write_byte(h, (CONFIG_OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true)
        || i2c_master_write_byte(h, 0x00, true)  /* 0x00 = command */
        || i2c_master_write_byte(h, cmd, true)
        || i2c_master_stop(h);
    if (ret == ESP_OK) ret = i2c_master_cmd_begin(I2C_MASTER_NUM, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    return ret;
}

static esp_err_t oled_write_data(const uint8_t *data, size_t len)
{
    if (len == 0) return ESP_OK;
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    if (!h) return ESP_FAIL;
    esp_err_t ret = i2c_master_start(h)
        || i2c_master_write_byte(h, (CONFIG_OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true)
        || i2c_master_write_byte(h, 0x40, true);  /* 0x40 = data */
    if (ret != ESP_OK) { i2c_cmd_link_delete(h); return ret; }
    ret = i2c_master_write(h, data, len, true) || i2c_master_stop(h);
    if (ret == ESP_OK) ret = i2c_master_cmd_begin(I2C_MASTER_NUM, h, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(h);
    return ret;
}

static esp_err_t oled_init(void)
{
    int sda = CONFIG_OLED_I2C_SDA_GPIO;
    int scl = CONFIG_OLED_I2C_SCL_GPIO;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED I2C param_config 失败: %s (SDA=%d SCL=%d)", esp_err_to_name(ret), sda, scl);
        return ret;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED I2C driver_install 失败: %s", esp_err_to_name(ret));
        return ret;
    }
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
            ESP_LOGE(TAG, "OLED 写命令 0x%02x 失败: %s (地址 0x%02X, SDA=%d SCL=%d)",
                     (unsigned)init[i], esp_err_to_name(ret),
                     (unsigned)CONFIG_OLED_I2C_ADDR, sda, scl);
            return ret;
        }
    }
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

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "重连 WiFi，第 %d 次", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); /* 避免死等 */
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_ip_addr = event->ip_info.ip;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t any_id, got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, CONFIG_ESP_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, CONFIG_ESP_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi 启动，SSID: %s", CONFIG_ESP_WIFI_SSID);
}

/* 检查所有外设/按钮配置的 GPIO 是否重复，若有则打日志并中止 */
static void check_gpio_duplicates(void)
{
    struct { int gpio; const char *name; } pins[] = {
        { CONFIG_OLED_I2C_SDA_GPIO,    "OLED SDA" },
        { CONFIG_OLED_I2C_SCL_GPIO,    "OLED SCL" },
        { CONFIG_TMC2209_STEP_GPIO,    "TMC2209 STEP" },
        { CONFIG_TMC2209_DIR_GPIO,     "TMC2209 DIR" },
        { CONFIG_TMC2209_ENABLE_GPIO, "TMC2209 ENABLE" },
        { CONFIG_BTN_SPEED_UP_GPIO,    "BTN SPEED_UP" },
        { CONFIG_BTN_SPEED_DOWN_GPIO,  "BTN SPEED_DOWN" },
        { CONFIG_BTN_MOTOR_START_GPIO,"BTN MOTOR_START" },
        { CONFIG_BTN_MOTOR_STOP_GPIO,  "BTN MOTOR_STOP" },
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

static void tmc2209_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CONFIG_TMC2209_STEP_GPIO) |
                        (1ULL << CONFIG_TMC2209_DIR_GPIO) |
                        (1ULL << CONFIG_TMC2209_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(CONFIG_TMC2209_STEP_GPIO, 0);
    gpio_set_level(CONFIG_TMC2209_DIR_GPIO, 0);
    /* TMC2209 低电平使能 */
    gpio_set_level(CONFIG_TMC2209_ENABLE_GPIO, 0);

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

static void stepper_task(void *arg)
{
    int64_t last_sec_us = esp_timer_get_time();
    for (;;) {
        if (!s_motor_running) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        int speed = s_speed_steps_per_sec;
        if (speed < MOTOR_SPEED_MIN) speed = MOTOR_SPEED_MIN;
        /* 每步 = 高 + 低，半周期 500000/speed 微秒 */
        uint32_t half_us = 500000 / (uint32_t)speed;
        if (half_us < 100) half_us = 100;

        gpio_set_level(CONFIG_TMC2209_STEP_GPIO, 1);
        if (half_us >= 1000) {
            vTaskDelay(pdMS_TO_TICKS(half_us / 1000));
        } else {
            int64_t until = esp_timer_get_time() + half_us;
            while (esp_timer_get_time() < until) { }
        }
        gpio_set_level(CONFIG_TMC2209_STEP_GPIO, 0);
        if (half_us >= 1000) {
            vTaskDelay(pdMS_TO_TICKS(half_us / 1000));
        } else {
            int64_t until = esp_timer_get_time() + half_us;
            while (esp_timer_get_time() < until) { }
        }

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_sec_us) >= 1000000) {
            last_sec_us = now_us;
            s_countdown_sec--;
            if (s_countdown_sec <= 0) {
                s_countdown_sec = 0;
                s_motor_running = 0;
            }
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
                if (s_speed_steps_per_sec < MOTOR_SPEED_MAX)
                    s_speed_steps_per_sec += 10;
            }
        }
        if (button_pressed(CONFIG_BTN_SPEED_DOWN_GPIO)) {
            if (t - last_down > BUTTON_DEBOUNCE_MS) {
                last_down = t;
                if (s_speed_steps_per_sec > MOTOR_SPEED_MIN)
                    s_speed_steps_per_sec -= 10;
            }
        }
        if (button_pressed(CONFIG_BTN_MOTOR_START_GPIO)) {
            if (t - last_start > BUTTON_DEBOUNCE_MS) {
                last_start = t;
                s_motor_running = 1;
                if (s_countdown_sec <= 0)
                    s_countdown_sec = COUNTDOWN_DEFAULT_SEC;
            }
        }
        if (button_pressed(CONFIG_BTN_MOTOR_STOP_GPIO)) {
            if (t - last_stop > BUTTON_DEBOUNCE_MS) {
                last_stop = t;
                s_motor_running = 0;
            }
        }
    }
}

static void display_task(void *arg)
{
    char line1[24], line2[24], line3[24];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
        int rpm = s_speed_steps_per_sec * 60 / CONFIG_MOTOR_STEPS_PER_REV;
        int c = s_countdown_sec;
        int m = c / 60, s = c % 60;
        snprintf(line1, sizeof(line1), "RPM: %d", rpm);
        snprintf(line2, sizeof(line2), "%02d:%02d", m, s);
        snprintf(line3, sizeof(line3), "%s", s_motor_running ? "RUN " : "STOP");
        oled_draw_string_line(1, line1);
        oled_draw_string_line(2, line2);
        oled_draw_string_line(3, line3);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, false, portMAX_DELAY);

    uint32_t ip = s_ip_addr.addr;
    snprintf(s_ip_str, sizeof(s_ip_str), "%d.%d.%d.%d",
             (int)(ip >> 0) & 0xff,
             (int)(ip >> 8) & 0xff,
             (int)(ip >> 16) & 0xff,
             (int)(ip >> 24) & 0xff);
    ESP_LOGI(TAG, "已连接 WiFi，IP: %s", s_ip_str);

    esp_err_t err = oled_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED 初始化失败: %s", esp_err_to_name(err));
        return;
    }

    check_gpio_duplicates();
    tmc2209_gpio_init();

    /* OLED 四行：第一行 WiFi IP，第二行转速，第三行倒计时，第四行 RUN/STOP */
    for (uint8_t p = 0; p < OLED_PAGES; p++)
        oled_clear_page(p);
    char line0[24];
    snprintf(line0, sizeof(line0), "IP: %s", s_ip_str);
    oled_draw_string_line(0, line0);
    int rpm0 = s_speed_steps_per_sec * 60 / CONFIG_MOTOR_STEPS_PER_REV;
    int c0 = s_countdown_sec;
    snprintf(line0, sizeof(line0), "RPM: %d", rpm0);
    oled_draw_string_line(1, line0);
    snprintf(line0, sizeof(line0), "%02d:%02d", c0 / 60, c0 % 60);
    oled_draw_string_line(2, line0);
    oled_draw_string_line(3, "STOP");

    xTaskCreate(stepper_task, "stepper", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 6, NULL);
    xTaskCreate(display_task, "display", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "OLED 四行已显示，电机与按钮任务已启动");
}
