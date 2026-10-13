// 4WD Wi-Fi remote-control car — ESP32-S3-N16R8 (ESP-IDF)
// SoftAP + 网页虚拟摇杆 (WebSocket) + TB6612FNG 差速驱动 + HC-SR04 前向避障 + ST7789 状态屏
// 设计文档: docs/design.md

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

static const char *TAG = "car";

// ---------------- 引脚 ----------------
// 引脚按面包板接线顺序排布: 每个模块使用开发板上连续的一段引脚, 且与模块自身引脚顺序一致
// ST7789 (屏幕引脚顺序 SCL SDA RST DC CS BL) -> GPIO 9..14 (SPI 经 GPIO 矩阵, 任意脚均可)
#define PIN_TFT_SCL  GPIO_NUM_9
#define PIN_TFT_SDA  GPIO_NUM_10
#define PIN_TFT_RST  GPIO_NUM_11
#define PIN_TFT_DC   GPIO_NUM_12
#define PIN_TFT_CS   GPIO_NUM_13
#define PIN_TFT_BL   GPIO_NUM_14

// TB6612 控制脚 (PWMA AIN2 AIN1 STBY BIN1 BIN2 PWMB) -> GPIO 4 5 6 7 15 16 17
#define PIN_PWMA GPIO_NUM_4
#define PIN_AIN2 GPIO_NUM_5
#define PIN_AIN1 GPIO_NUM_6
#define PIN_STBY GPIO_NUM_7
#define PIN_BIN1 GPIO_NUM_15
#define PIN_BIN2 GPIO_NUM_16
#define PIN_PWMB GPIO_NUM_17

// HC-SR04 (TRIG ECHO); ECHO 需经 1k + 2k 分压到 3.3V
// 避开 strapping 脚 (0/3/45/46) 和 N16R8 的 PSRAM 脚 (35/36/37)
#define PIN_TRIG GPIO_NUM_42
#define PIN_ECHO GPIO_NUM_41

// ---------------- 参数 ----------------
#define AP_SSID          "4WD-Car"
#define AP_PASS          "12345678"
#define PWM_FREQ_HZ      20000
#define OBSTACLE_CM      20.0f
#define CMD_TIMEOUT_MS   500
#define DEADZONE         0.08f
// 若方向反了, 改这里 (左右电机装反很常见)
#define INVERT_LEFT      false
#define INVERT_RIGHT     false
// 屏幕: 颜色发白/反色就改 INVERT, 红蓝互换就改 BGR, 无 CS 的模块需把 SPI_MODE 改 3
#define LCD_INVERT       true
#define LCD_BGR          false
#define LCD_SPI_MODE     0
#define LCD_W            240
#define LCD_H            240
#define LCD_ROW_MAX_H    32     // 最大字号 4 -> 8*4 行高

#define MAX_WS_CLIENTS   4

// ---------------- 共享状态 ----------------
static volatile float    g_cmd_x = 0, g_cmd_y = 0;
static volatile int64_t  g_last_cmd_ms = 0;
static volatile float    g_dist_cm = -1;     // -1: 无回波
static volatile bool     g_blocked = false;
static volatile int      g_action = 0;

static const char *ACTION_CN[] = {"停止", "障碍/停止", "前进", "后退", "左转", "右转", "左前", "右前", "左后", "右后"};
static const char *ACTION_EN[] = {"STOP", "BLOCKED", "FWD", "BACK", "LEFT", "RIGHT", "FWD-L", "FWD-R", "BACK-L", "BACK-R"};
enum { A_STOP, A_BLOCK, A_FWD, A_BACK, A_LEFT, A_RIGHT, A_FL, A_FR, A_BL, A_BR };

static httpd_handle_t g_httpd = NULL;
static int g_ws_fds[MAX_WS_CLIENTS] = {-1, -1, -1, -1};
static portMUX_TYPE g_ws_lock = portMUX_INITIALIZER_UNLOCKED;

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

// ======================================================================
// 电机
// ======================================================================
static void motors_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) | (1ULL << PIN_BIN1) |
                        (1ULL << PIN_BIN2) | (1ULL << PIN_STBY),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_STBY, 0);
    gpio_set_level(PIN_AIN1, 0); gpio_set_level(PIN_AIN2, 0);
    gpio_set_level(PIN_BIN1, 0); gpio_set_level(PIN_BIN2, 0);

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ch.channel = LEDC_CHANNEL_0; ch.gpio_num = PIN_PWMA;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ch.channel = LEDC_CHANNEL_1; ch.gpio_num = PIN_PWMB;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    gpio_set_level(PIN_STBY, 1);
}

// speed: -255..255, 正 = 前进
static void drive_channel(ledc_channel_t ch, gpio_num_t in1, gpio_num_t in2, int speed, bool invert)
{
    if (invert) speed = -speed;
    gpio_set_level(in1, speed > 0);
    gpio_set_level(in2, speed < 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, abs(speed));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void motors_set(int left, int right)
{
    drive_channel(LEDC_CHANNEL_0, PIN_AIN1, PIN_AIN2, left, INVERT_LEFT);
    drive_channel(LEDC_CHANNEL_1, PIN_BIN1, PIN_BIN2, right, INVERT_RIGHT);
}

static int clamp255(float v)
{
    int i = (int)(v * 255.0f);
    return i > 255 ? 255 : (i < -255 ? -255 : i);
}

static int ws_client_count(void)
{
    int n = 0;
    portENTER_CRITICAL(&g_ws_lock);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) if (g_ws_fds[i] >= 0) n++;
    portEXIT_CRITICAL(&g_ws_lock);
    return n;
}

static void control_task(void *arg)
{
    while (1) {
        float x = g_cmd_x, y = g_cmd_y;

        // 失联保护
        if (ws_client_count() == 0 || now_ms() - g_last_cmd_ms > CMD_TIMEOUT_MS) { x = 0; y = 0; }

        if (fabsf(x) < DEADZONE) x = 0;
        if (fabsf(y) < DEADZONE) y = 0;

        // 前向避障联锁: 只禁止前进, 允许后退和原地转向
        float d = g_dist_cm;
        bool blocked = (d > 0 && d < OBSTACLE_CM);
        g_blocked = blocked;
        if (blocked && y > 0) y = 0;

        int left = clamp255(y + x);
        int right = clamp255(y - x);
        motors_set(left, right);

        int a;
        if (left == 0 && right == 0)       a = blocked ? A_BLOCK : A_STOP;
        else if (y > 0 && fabsf(x) < 0.3f) a = A_FWD;
        else if (y < 0 && fabsf(x) < 0.3f) a = A_BACK;
        else if (y == 0 && x > 0)          a = A_RIGHT;
        else if (y == 0 && x < 0)          a = A_LEFT;
        else if (x > 0)                    a = y > 0 ? A_FR : A_BR;
        else                               a = y > 0 ? A_FL : A_BL;
        g_action = a;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ======================================================================
// HC-SR04 (GPIO 边沿中断计时, 不阻塞)
// ======================================================================
static volatile int64_t g_echo_rise_us = 0;
static volatile int32_t g_echo_width_us = 0;
static volatile bool    g_echo_done = false;

static void IRAM_ATTR echo_isr(void *arg)
{
    int64_t t = esp_timer_get_time();
    if (gpio_get_level(PIN_ECHO)) {
        g_echo_rise_us = t;
    } else {
        g_echo_width_us = (int32_t)(t - g_echo_rise_us);
        g_echo_done = true;
    }
}

static void ultrasonic_task(void *arg)
{
    gpio_config_t trig = {.pin_bit_mask = 1ULL << PIN_TRIG, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&trig);
    gpio_set_level(PIN_TRIG, 0);

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << PIN_ECHO,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&echo);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ECHO, echo_isr, NULL);

    while (1) {
        g_echo_done = false;
        gpio_set_level(PIN_TRIG, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TRIG, 0);

        vTaskDelay(pdMS_TO_TICKS(40));   // 4m 往返约 23ms

        if (g_echo_done) {
            int32_t w = g_echo_width_us;
            g_dist_cm = (w > 0 && w < 25000) ? w * 0.0343f / 2.0f : -1;
        } else {
            g_dist_cm = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

// ======================================================================
// 显示 (ST7789, 按行带背景重绘, 无闪烁)
// ======================================================================
// 5x7 字体, ASCII 32..90 (小写自动转大写)
static const uint8_t FONT5X7[59][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x56,0x20,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
};

#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define C_BLACK  RGB565(0, 0, 0)
#define C_WHITE  RGB565(255, 255, 255)
#define C_CYAN   RGB565(0, 255, 255)
#define C_GREEN  RGB565(0, 255, 0)
#define C_RED    RGB565(255, 0, 0)
#define C_YELLOW RGB565(255, 255, 0)

static esp_lcd_panel_handle_t g_panel = NULL;
static SemaphoreHandle_t g_lcd_done = NULL;
static uint16_t *g_rowbuf = NULL;

static bool lcd_trans_done_cb(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ev, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(g_lcd_done, &woken);
    return woken == pdTRUE;
}

static void lcd_init(void)
{
    gpio_config_t bl = {.pin_bit_mask = 1ULL << PIN_TFT_BL, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&bl);
    gpio_set_level(PIN_TFT_BL, 1);

    g_lcd_done = xSemaphoreCreateBinary();
    g_rowbuf = heap_caps_malloc(LCD_W * LCD_ROW_MAX_H * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(g_rowbuf);

    spi_bus_config_t bus = {
        .sclk_io_num = PIN_TFT_SCL,
        .mosi_io_num = PIN_TFT_SDA,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * LCD_ROW_MAX_H * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_TFT_DC,
        .cs_gpio_num = PIN_TFT_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = LCD_SPI_MODE,
        .trans_queue_depth = 4,
        .on_color_trans_done = lcd_trans_done_cb,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_TFT_RST,
        .rgb_ele_order = LCD_BGR ? LCD_RGB_ELEMENT_ORDER_BGR : LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,   // 直接提供主机序 RGB565
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(g_panel, LCD_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(g_panel, true));
}

// 绘制一整行 (宽 240, 高 8*scale), 带背景色
static void lcd_draw_row(int y, int scale, uint16_t fg, uint16_t bg, const char *s)
{
    int h = 8 * scale;
    for (int i = 0; i < LCD_W * h; i++) g_rowbuf[i] = bg;

    int cx = 8;
    for (; *s; s++) {
        int c = *s;
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c < 32 || c > 90) c = '?';
        if (cx + 5 * scale > LCD_W) break;
        const uint8_t *g = FONT5X7[c - 32];
        for (int col = 0; col < 5; col++) {
            for (int row = 0; row < 7; row++) {
                if (!((g[col] >> row) & 1)) continue;
                for (int dy = 0; dy < scale; dy++)
                    for (int dx = 0; dx < scale; dx++)
                        g_rowbuf[(row * scale + dy) * LCD_W + cx + col * scale + dx] = fg;
            }
        }
        cx += 6 * scale;
    }
    esp_lcd_panel_draw_bitmap(g_panel, 0, y, LCD_W, y + h, g_rowbuf);
    xSemaphoreTake(g_lcd_done, pdMS_TO_TICKS(200));
}

typedef struct { char text[32]; uint16_t color; } row_cache_t;

static void lcd_row_cached(row_cache_t *rc, int y, int scale, uint16_t fg, const char *text)
{
    if (rc->color == fg && strcmp(rc->text, text) == 0) return;
    strlcpy(rc->text, text, sizeof(rc->text));
    rc->color = fg;
    lcd_draw_row(y, scale, fg, C_BLACK, text);
}

static void lcd_clear(void)
{
    for (int y = 0; y < LCD_H; y += 8 * 4) {
        int h = (LCD_H - y) < 32 ? (LCD_H - y) : 32;
        for (int i = 0; i < LCD_W * h; i++) g_rowbuf[i] = C_BLACK;
        esp_lcd_panel_draw_bitmap(g_panel, 0, y, LCD_W, y + h, g_rowbuf);
        xSemaphoreTake(g_lcd_done, pdMS_TO_TICKS(200));
    }
}

// ======================================================================
// WebSocket / HTTP
// ======================================================================
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static void ws_add(int fd)
{
    portENTER_CRITICAL(&g_ws_lock);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) if (g_ws_fds[i] == fd) { portEXIT_CRITICAL(&g_ws_lock); return; }
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (g_ws_fds[i] < 0) { g_ws_fds[i] = fd; break; }
    }
    portEXIT_CRITICAL(&g_ws_lock);
}

static void ws_remove(int fd)
{
    portENTER_CRITICAL(&g_ws_lock);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) if (g_ws_fds[i] == fd) g_ws_fds[i] = -1;
    portEXIT_CRITICAL(&g_ws_lock);
}

// 从 JSON 文本里取数字: {"x":0.5,"y":-0.3}
static bool json_num(const char *s, const char *key, float *out)
{
    char pat[8];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(s, pat);
    if (!p) return false;
    *out = strtof(p + strlen(pat), NULL);
    return true;
}

static float clampf(float v) { return v > 1 ? 1 : (v < -1 ? -1 : v); }

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {          // 握手
        ws_add(httpd_req_to_sockfd(req));
        ESP_LOGI(TAG, "ws client connected (fd=%d)", httpd_req_to_sockfd(req));
        g_cmd_x = 0; g_cmd_y = 0;
        return ESP_OK;
    }

    uint8_t buf[128];
    httpd_ws_frame_t f = {.payload = buf, .type = HTTPD_WS_TYPE_TEXT};
    esp_err_t ret = httpd_ws_recv_frame(req, &f, sizeof(buf) - 1);
    if (ret != ESP_OK) return ret;
    buf[f.len] = 0;

    if (f.type == HTTPD_WS_TYPE_TEXT) {
        float x = 0, y = 0;
        json_num((char *)buf, "x", &x);
        json_num((char *)buf, "y", &y);
        g_cmd_x = clampf(x);
        g_cmd_y = clampf(y);
        g_last_cmd_ms = now_ms();
    }
    return ESP_OK;
}

static void httpd_close_cb(httpd_handle_t hd, int fd)
{
    ws_remove(fd);
    ESP_LOGI(TAG, "client closed (fd=%d)", fd);
    close(fd);
}

static void start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    cfg.close_fn = httpd_close_cb;
    ESP_ERROR_CHECK(httpd_start(&g_httpd, &cfg));

    httpd_uri_t idx = {.uri = "/", .method = HTTP_GET, .handler = index_handler};
    httpd_uri_t ws = {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true};
    httpd_register_uri_handler(g_httpd, &idx);
    httpd_register_uri_handler(g_httpd, &ws);
}

// ======================================================================
// UI 任务: 状态上报 (WebSocket) + 屏幕刷新, 每 200ms
// ======================================================================
static void ui_task(void *arg)
{
    row_cache_t r_title = {0}, r_ssid = {0}, r_ip = {0}, r_cli = {0}, r_dist = {0}, r_act = {0}, r_stat = {0};
    esp_netif_ip_info_t ip_info;
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    lcd_clear();

    while (1) {
        int clients = ws_client_count();
        float d = g_dist_cm;
        bool blocked = g_blocked;
        int act = g_action;

        // --- 上报给网页 ---
        if (clients > 0 && g_httpd) {
            char msg[96];
            int n = snprintf(msg, sizeof(msg), "{\"action\":\"%s\",\"dist\":%d,\"blocked\":%s}",
                             ACTION_CN[act], d < 0 ? -1 : (int)d, blocked ? "true" : "false");
            httpd_ws_frame_t f = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)msg, .len = n, .final = true};
            int fds[MAX_WS_CLIENTS];
            portENTER_CRITICAL(&g_ws_lock);
            memcpy(fds, g_ws_fds, sizeof(fds));
            portEXIT_CRITICAL(&g_ws_lock);
            for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                if (fds[i] >= 0) httpd_ws_send_frame_async(g_httpd, fds[i], &f);
            }
        }

        // --- 屏幕 ---
        char line[32];
        lcd_row_cached(&r_title, 8, 3, C_CYAN, "4WD CAR");
        lcd_row_cached(&r_ssid, 56, 2, C_WHITE, "WIFI:" AP_SSID);
        if (ap && esp_netif_get_ip_info(ap, &ip_info) == ESP_OK) {
            snprintf(line, sizeof(line), "IP:" IPSTR, IP2STR(&ip_info.ip));
            lcd_row_cached(&r_ip, 80, 2, C_WHITE, line);
        }
        lcd_row_cached(&r_cli, 104, 2, clients ? C_GREEN : C_RED, clients ? "CLIENT:ON" : "CLIENT:OFF");
        if (d < 0) snprintf(line, sizeof(line), "-- CM");
        else       snprintf(line, sizeof(line), "%d CM", (int)d);
        lcd_row_cached(&r_dist, 140, 4, blocked ? C_RED : C_YELLOW, line);
        lcd_row_cached(&r_act, 184, 2, C_GREEN, ACTION_EN[act]);
        lcd_row_cached(&r_stat, 208, 2, blocked ? C_RED : C_GREEN, blocked ? "OBSTACLE!" : "OK");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ======================================================================
// Wi-Fi SoftAP
// ======================================================================
static void wifi_start_ap(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, AP_SSID);
    strcpy((char *)wc.ap.password, AP_PASS);
    wc.ap.ssid_len = strlen(AP_SSID);
    wc.ap.channel = 1;
    wc.ap.max_connection = 4;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP started: SSID=%s, IP=192.168.4.1", AP_SSID);
}

void app_main(void)
{
    motors_init();
    motors_set(0, 0);

    lcd_init();
    wifi_start_ap();
    start_webserver();

    xTaskCreate(ultrasonic_task, "ultrasonic", 3072, NULL, 5, NULL);
    xTaskCreate(control_task, "control", 3072, NULL, 6, NULL);
    xTaskCreate(ui_task, "ui", 6144, NULL, 4, NULL);
}
