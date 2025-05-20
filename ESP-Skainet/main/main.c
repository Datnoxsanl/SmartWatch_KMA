#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_board_init.h"
#include "esp_log.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "model_path.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led_strip.h"

static const char *TAG = "ESP_Skainet";

#define LED_STRIP_GPIO 19
#define NUM_LEDS 8
#define UART_PORT_NUM UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BAUD_RATE 115200

int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;
static volatile int task_flag = 0;
srmodel_list_t *models = NULL;
static int play_voice = -2;
static led_strip_handle_t led_strip;
static volatile int led_effect_active = 0;
static volatile int persistent_color_active = 0;
static SemaphoreHandle_t led_strip_semaphore = NULL;
static TaskHandle_t led_converge_task_handle = NULL;
static volatile bool led_waiting_active = true;

void led_clear_task(void *arg);
void set_all_leds_color(uint8_t r, uint8_t g, uint8_t b, bool persistent);
void set_leds_off(void);
void set_leds_red(void);
void set_leds_yellow(void);
void set_leds_blue(void);
void set_leds_white(void);
void set_leds_deep_sky_blue(void);
void set_leds_blue_violet(void);
void set_leds_silver(void);
void set_leds_light_green(void);
void set_leds_dark_slate_blue(void);

void led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = NUM_LEDS,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, 
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(50));
    led_strip_semaphore = xSemaphoreCreateBinary();
    if (led_strip_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create LED strip semaphore");
    }
    xSemaphoreGive(led_strip_semaphore);
    ESP_LOGI(TAG, "LED strip initialized");
}

void led_reverse_chase(void)
{
    if (xSemaphoreTake(led_strip_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        static int offset = 0;
        for (int i = NUM_LEDS - 1; i >= 0; i--)
        {
            int hue = (i * 360 / NUM_LEDS + offset) % 360;
            int r, g, b;
            int c = 255 * 1 / 100;
            int x = c * (1 - abs((hue / 60) % 2 - 1));
            if (hue < 60)
            {
                r = c;
                g = x;
                b = 0;
            }
            else if (hue < 120)
            {
                r = x;
                g = c;
                b = 0;
            }
            else if (hue < 180)
            {
                r = 0;
                g = c;
                b = x;
            }
            else if (hue < 240)
            {
                r = 0;
                g = x;
                b = c;
            }
            else if (hue < 300)
            {
                r = x;
                g = 0;
                b = c;
            }
            else
            {
                r = c;
                g = 0;
                b = x;
            }
            led_strip_set_pixel(led_strip, i, r, g, b);
        }
        offset = (offset + 10) % 360;
        led_strip_refresh(led_strip);
        xSemaphoreGive(led_strip_semaphore);
    }
}

void led_converge_effect(void *arg)
{
    ESP_LOGI(TAG, "Starting converge LED effect");
    led_effect_active = 1;
    TickType_t start_time = xTaskGetTickCount();
    const int duration_ms = 2000;
    const int cycle_time_ms = 300;

    const struct
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    } colors[] = {
        {255, 0, 0}, {255, 165, 0}, {255, 255, 0}, {0, 255, 0}, {0, 0, 255}, {128, 0, 128}, {255, 255, 255}};
    const int num_colors = 7;
    int offset = 0;

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(duration_ms))
    {
        if (xSemaphoreTake(led_strip_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            led_strip_clear(led_strip);
            int elapsed_cycle = (xTaskGetTickCount() - start_time) % pdMS_TO_TICKS(cycle_time_ms);
            float phase = (float)elapsed_cycle / pdMS_TO_TICKS(cycle_time_ms);
            int led_pos = (int)(phase * (NUM_LEDS / 2));
            for (int i = 0; i <= led_pos && i < NUM_LEDS / 2; i++)
            {
                int color_idx = (i + offset) % num_colors;
                uint8_t r = colors[color_idx].r * 1 / 100;
                uint8_t g = colors[color_idx].g * 1 / 100;
                uint8_t b = colors[color_idx].b * 1 / 100;
                led_strip_set_pixel(led_strip, i, r, g, b);
                led_strip_set_pixel(led_strip, NUM_LEDS - 1 - i, r, g, b);
            }
            led_strip_refresh(led_strip);
            xSemaphoreGive(led_strip_semaphore);
            offset = (offset + 1) % num_colors;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    xTaskCreatePinnedToCore(&led_clear_task, "led_clear", 2 * 1024, NULL, 3, NULL, 1);
    ESP_LOGI(TAG, "Ending converge LED effect");
    led_converge_task_handle = NULL;
    vTaskDelete(NULL);
}

void led_waiting_wakeword(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    while (true)
    {
        if (led_waiting_active && !led_effect_active && !persistent_color_active && detect_flag == 0)
        {
            led_reverse_chase();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void led_clear_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    if (persistent_color_active)
    {
        ESP_LOGI(TAG, "Skipping LED clear due to persistent color");
        led_effect_active = 0;
        vTaskDelete(NULL);
        return;
    }
    if (xSemaphoreTake(led_strip_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        ESP_LOGI(TAG, "Clearing LEDs");
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        xSemaphoreGive(led_strip_semaphore);
    }
    led_effect_active = 0;
    vTaskDelete(NULL);
}

void led_color_transition(uint8_t r, uint8_t g, uint8_t b)
{
    if (r == 255 && g == 255 && b == 255)
    {
        persistent_color_active = 0;
        led_waiting_active = false;
        if (led_converge_task_handle != NULL)
        {
            vTaskDelete(led_converge_task_handle);
            led_converge_task_handle = NULL;
            led_effect_active = 0;
            if (xSemaphoreTake(led_strip_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                led_strip_clear(led_strip);
                led_strip_refresh(led_strip);
                xSemaphoreGive(led_strip_semaphore);
            }
        }
        xTaskCreatePinnedToCore(&led_converge_effect, "led_converge", 4 * 1024, NULL, 7, &led_converge_task_handle, 1);
    }
}

void set_all_leds_color(uint8_t r, uint8_t g, uint8_t b, bool persistent)
{
    if (led_converge_task_handle != NULL)
    {
        vTaskDelete(led_converge_task_handle);
        led_converge_task_handle = NULL;
        led_effect_active = 0;
    }
    led_waiting_active = false;
    if (xSemaphoreTake(led_strip_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        if (persistent)
        {
            persistent_color_active = 1;
            led_effect_active = 0;
        }
        else
        {
            led_effect_active = 1;
            persistent_color_active = 0;
        }
        ESP_LOGI(TAG, "Setting all LEDs to RGB(%d, %d, %d), persistent: %d", r, g, b, persistent);
        led_strip_clear(led_strip);
        for (int i = 0; i < NUM_LEDS; i++)
        {
            uint8_t scaled_r = r * 1 / 100;
            uint8_t scaled_g = g * 1 / 100;
            uint8_t scaled_b = b * 1 / 100;
            led_strip_set_pixel(led_strip, i, scaled_r, scaled_g, scaled_b);
            ESP_LOGI(TAG, "Set pixel %d to RGB(%d, %d, %d)", i, scaled_r, scaled_g, scaled_b);
        }
        led_strip_refresh(led_strip);
        ESP_LOGI(TAG, "LED strip refreshed");
        xSemaphoreGive(led_strip_semaphore);
        if (!persistent)
        {
            xTaskCreatePinnedToCore(&led_clear_task, "led_clear", 2 * 1024, NULL, 3, NULL, 1);
        }
        if (!persistent && !led_effect_active)
        {
            led_waiting_active = true;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to take LED strip semaphore");
    }
}

void set_leds_off(void)
{
    set_all_leds_color(0, 0, 0, true);
}

void set_leds_red(void)
{
    set_all_leds_color(255, 0, 0, true);
}

void set_leds_yellow(void)
{
    set_all_leds_color(255, 255, 0, true);
}

void set_leds_blue(void)
{
    set_all_leds_color(0, 0, 255, true);
}

void set_leds_white(void)
{
    set_all_leds_color(255, 255, 255, true);
}

void set_leds_deep_sky_blue(void)
{
    set_all_leds_color(0, 191, 255, false);
}

void set_leds_blue_violet(void)
{
    set_all_leds_color(138, 43, 226, false);
}

void set_leds_silver(void)
{
    set_all_leds_color(192, 192, 192, false);
}

void set_leds_light_green(void)
{
    set_all_leds_color(144, 238, 144, false);
}

void set_leds_dark_slate_blue(void)
{
    set_all_leds_color(72, 61, 139, false);
}

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 256, 0, 0, NULL, 0));
}

void uart_receive_task(void *arg)
{
    uint8_t data[1];
    while (true)
    {
        int len = uart_read_bytes(UART_PORT_NUM, data, 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            char cmd = data[0];
            if (cmd >= 32 && cmd <= 126)
            {
                ESP_LOGI(TAG, "Received UART command: %c", cmd);
                switch (cmd)
                {
                case 'm': // Ask time
                    set_leds_deep_sky_blue();
                    break;
                case 'n': // Ask weather
                    set_leds_blue_violet();
                    break;
                case 'o': // Set timer
                    set_leds_silver();
                    break;
                case 'p': // Set alarm
                    set_leds_light_green();
                    break;
                case 'c': // Red
                    set_leds_red();
                    break;
                case 'd': // Yellow
                    set_leds_yellow();
                    break;
                case 'e': // Blue
                    set_leds_blue();
                    break;
                case 'f': // White
                    set_leds_white();
                    break;
                case 'b': // LED off
                    set_leds_off();
                    break;
                default:
                    ESP_LOGI(TAG, "Unknown UART command: %c", cmd);
                    break;
                }
            }
            else
            {
                ESP_LOGW(TAG, "Ignored invalid UART command: 0x%02X", cmd);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void feed_Task(void *arg)
{
    afe_task_into_t *afe_task_info = (afe_task_into_t *)arg;
    esp_afe_sr_iface_t *afe_handle = afe_task_info->afe_handle;
    esp_afe_sr_data_t *afe_data = afe_task_info->afe_data;

    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    assert(nch == feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel * 4);
    if (!i2s_buff)
    {
        ESP_LOGE(TAG, "Failed to allocate i2s_buff!");
        vTaskDelete(NULL);
    }

    while (task_flag)
    {
        esp_get_feed_data(true, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        afe_handle->feed(afe_data, i2s_buff);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    free(i2s_buff);
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    afe_task_into_t *afe_task_info = (afe_task_into_t *)arg;
    esp_afe_sr_iface_t *afe_handle = afe_task_info->afe_handle;
    esp_afe_sr_data_t *afe_data = afe_task_info->afe_data;

    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data);
    assert(mu_chunksize == afe_chunksize);
    multinet->print_active_speech_commands(model_data);

    xTaskCreatePinnedToCore(&led_waiting_wakeword, "led_waiting", 4 * 1024, NULL, 5, NULL, 1);

    while (task_flag)
    {
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res)
        {
            ESP_LOGE(TAG, "AFE fetch returned NULL!");
            continue;
        }
        if (res->ret_value == ESP_FAIL)
        {
            ESP_LOGE(TAG, "AFE fetch failed!");
            continue;
        }

        if (res->wakeup_state == WAKENET_DETECTED)
        {
            ESP_LOGI(TAG, "WAKEWORD DETECTED");
            uart_write_bytes(UART_PORT_NUM, "a", 1);
            ESP_LOGI(TAG, "Sent UART command: a");
            led_color_transition(255, 255, 255);
            multinet->clean(model_data);
        }
        else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED)
        {
            play_voice = -1;
            detect_flag = 1;
            ESP_LOGI(TAG, "AFE_FETCH_CHANNEL_VERIFIED, channel index: %d", res->trigger_channel_id);
        }

        if (detect_flag == 1)
        {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);
            if (mn_state == ESP_MN_STATE_DETECTING)
            {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++)
                {
                    ESP_LOGI(TAG, "TOP %d, command_id: %d, phrase_id: %d, string: %s, prob: %f",
                             i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }
                if (mn_result->num > 0)
                {
                    switch (mn_result->command_id[0])
                    {
                    case 0: // LED OFF
                        // uart_write_bytes(UART_PORT_NUM, "b", 1);
                        // ESP_LOGI(TAG, "Sent UART command: b");
                        set_leds_off();
                        break;
                    case 1: // RED
                        // uart_write_bytes(UART_PORT_NUM, "c", 1);
                        // ESP_LOGI(TAG, "Sent UART command: c");
                        set_leds_red();
                        break;
                    case 2: // YELLOW
                        // uart_write_bytes(UART_PORT_NUM, "d", 1);
                        // ESP_LOGI(TAG, "Sent UART command: d");
                        set_leds_yellow();
                        break;
                    case 3: // BLUE
                        // uart_write_bytes(UART_PORT_NUM, "e", 1);
                        // ESP_LOGI(TAG, "Sent UART command: e");
                        set_leds_blue();
                        break;
                    case 4: // WHITE
                        // uart_write_bytes(UART_PORT_NUM, "f", 1);
                        // ESP_LOGI(TAG, "Sent UART command: f");
                        set_leds_white();
                        break;
                    case 5: // ASK TIME
                        uart_write_bytes(UART_PORT_NUM, "g", 1);
                        ESP_LOGI(TAG, "Sent UART command: g");
                        set_leds_deep_sky_blue();
                        break;
                    case 6: // ASK WEATHER
                        uart_write_bytes(UART_PORT_NUM, "h", 1);
                        ESP_LOGI(TAG, "Sent UART command: h");
                        set_leds_blue_violet();
                        break;
                    case 7: // SET ALARM AT SIX
                        uart_write_bytes(UART_PORT_NUM, "i", 1);
                        ESP_LOGI(TAG, "Sent UART command: i");
                        set_leds_light_green();
                        break;
                    case 8: // TIMER FIVE MINUTES
                        uart_write_bytes(UART_PORT_NUM, "k", 1);
                        ESP_LOGI(TAG, "Sent UART command: k");
                        set_leds_silver();
                        break;
                    case 9: // TIMER TWO MINUTES
                        uart_write_bytes(UART_PORT_NUM, "l", 1);
                        ESP_LOGI(TAG, "Sent UART command: l");
                        set_leds_dark_slate_blue();
                        break;
                    default:
                        break;
                    }
                }
                ESP_LOGI(TAG, "-----------Awaiting Order-----------");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT)
            {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                ESP_LOGI(TAG, "timeout, string:%s", mn_result->string);
                afe_handle->enable_wakenet(afe_data);
                detect_flag = 0;
                led_waiting_active = true;
                ESP_LOGI(TAG, "-----------Awaiting Order-----------");
            }
        }
    }
    if (model_data)
    {
        multinet->destroy(model_data);
    }
    if (xSemaphoreTake(led_strip_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        xSemaphoreGive(led_strip_semaphore);
    }
    ESP_LOGI(TAG, "detect exit");
    vTaskDelete(NULL);
}

void app_main(void)
{
    led_init();
    set_leds_red();
    vTaskDelay(pdMS_TO_TICKS(2000));
    uart_init();
    xTaskCreatePinnedToCore(&uart_receive_task, "uart_receive", 4 * 1024, NULL, 5, NULL, 1);

    models = esp_srmodel_init("model");
    ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_16K_SAMPLES, 1, 16));

#if CONFIG_IDF_TARGET_ESP32
    ESP_LOGE(TAG, "This demo only supports ESP32S3");
    return;
#else
    afe_config_t *afe_config = afe_config_init("MMNR", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    afe_config_print(afe_config);
    afe_handle = esp_afe_handle_from_config(afe_config);
#endif

    afe_config->wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
#if defined CONFIG_ESP32_S3_BOX_BOARD || defined CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_S3_DEVKIT_C
    afe_config->aec_init = false;
#if defined CONFIG_ESP32_S3_EYE_BOARD || CONFIG_ESP32_S3_DEVKIT_C
    afe_config->pcm_config.total_ch_num = 2;
    afe_config->pcm_config.mic_num = 1;
    afe_config->pcm_config.ref_num = 1;
#endif
#endif

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(afe_config);
    afe_task_into_t task_info = {
        .afe_data = afe_data,
        .afe_handle = afe_handle,
        .feed_task = NULL,
        .fetch_task = NULL};
    task_flag = 1;

    xTaskCreatePinnedToCore(&detect_Task, "detect", 12 * 1024, &task_info, 8, NULL, 1);
    xTaskCreatePinnedToCore(&feed_Task, "feed", 12 * 1024, &task_info, 5, NULL, 0);
}