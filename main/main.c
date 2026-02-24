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
#include "esp_err.h"

#include "sdkconfig.h"

static const char *TAG = "main";

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
};

#define FONT_FIRST 0x20
#define FONT_LAST  0x50
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

static esp_err_t oled_draw_string_line0(const char *str)
{
    oled_write_cmd(0x21); oled_write_cmd(0); oled_write_cmd(OLED_WIDTH - 1);
    oled_write_cmd(0x22); oled_write_cmd(0); oled_write_cmd(0);

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
    char ip_str[20];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
             (int)(ip >> 0) & 0xff,
             (int)(ip >> 8) & 0xff,
             (int)(ip >> 16) & 0xff,
             (int)(ip >> 24) & 0xff);
    ESP_LOGI(TAG, "已连接 WiFi，IP: %s", ip_str);

    esp_err_t err = oled_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED 初始化失败: %s", esp_err_to_name(err));
        return;
    }
    /* 第一行显示 "IP: x.x.x.x" */
    char line1[24];
    snprintf(line1, sizeof(line1), "IP: %s", ip_str);
    for (uint8_t p = 0; p < OLED_PAGES; p++)
        oled_clear_page(p);
    oled_draw_string_line0(line1);
    ESP_LOGI(TAG, "已在 OLED 第一行显示: %s", line1);
}
