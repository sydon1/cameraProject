#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include <esp_system.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_mac.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

// includes for networking
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>

#define WIFI_SSID "GAME4_ESP"
#define WIFI_PASS "yes123456"
#define PORT 8080  // TCP Port

static const char *TAG = "ESP32_GAME4";

bool first = true;

// Define GPIO pin for the external interrupt
#define INTERRUPT_PIN 40

// Define the camera pins for ESP32S3
#define CAM_PIN_PWDN 38
#define CAM_PIN_RESET -1   //software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 2
#define CAM_PIN_D0 18
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 21
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16

#include "mirf.h"

#define CONFIG_RECEIVER 1
#define CONFIG_RADIO_CHANNEL 99
#define CONFIG_ADVANCED 1
#define CONFIG_RF_RATIO_1M 1
#define CONFIG_RETRANSMIT_DELAY 100
#define PAYLOAD_SIZE 32
#define ARRAY_SIZE 200

#define CONFIG_IRQ_GPIO GPIO_NUM_40

NRF24_t dev;

typedef struct {
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .xclk_freq_hz = 10000000, // use 10 MHz
    .pixel_format = PIXFORMAT_JPEG,  
    .frame_size = FRAMESIZE_VGA,   // vga max quality = 4
    .jpeg_quality = 4,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

// Initialize camera
static esp_err_t init_camera(void) {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    // Adjust camera settings
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 1);   // Adjust brightness (-2 to 2)
        s->set_contrast(s, 2);     // Adjust contrast (-2 to 2)
        s->set_saturation(s, -1);  // Adjust saturation (-2 to 2)
        s->set_gainceiling(s, (gainceiling_t)4);  // Adjust gain ceiling (0 to 6)

        // Exposure settings
        s->set_exposure_ctrl(s, 1); // Enable auto exposure
        s->set_aec2(s, 1);          // Enable AEC algorithm
        s->set_ae_level(s, 0);      // Exposure level (-2 to 2)
        s->set_aec_value(s, 300);   // Manual exposure value (0-1200, lower = brighter)

        // White Balance
        s->set_whitebal(s, 1);  // Enable Auto White Balance (AWB)
    }

    return ESP_OK;
}
#endif

// Initialize NRF24L01 module
void nrf_init() {
    Nrf24_init(&dev);
    uint8_t payload = 32;
    uint8_t channel = CONFIG_RADIO_CHANNEL;
    Nrf24_config(&dev, channel, payload);

    esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *) "GAME4");
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "nrf24l01 not installed");
        while(1) { vTaskDelay(1); }
    }
    ret = Nrf24_setRADDR(&dev, (uint8_t *)"GAME4");
    if (ret != ESP_OK) {
        ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
        while(1) { vTaskDelay(1); }
    }

    Nrf24_setRetransmitDelay(&dev, (15 << ARD) | (15 << ARC)); 
    Nrf24_SetSpeedDataRates(&dev, 0);
    Nrf24_printDetails(&dev);

    // Clear RX FiFo
    uint8_t buf[32];
    while(1) {
        if (Nrf24_dataReady(&dev) == false) break;
        Nrf24_getData(&dev, buf);   
    }
}

// Queue to handle interrupt events
static QueueHandle_t gpio_evt_queue = NULL;

// Interrupt Service Routine (ISR)
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int pin = (int)arg;
    xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
}

// Task to handle GPIO events
static void gpio_task(void* arg) {
    int io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            uint8_t buf[32];
            if (Nrf24_dataReady(&dev)) {
                Nrf24_getData(&dev, buf);
                ESP_EARLY_LOGI("NRF24", "Received Data: %s", buf);
            } else {
                ESP_EARLY_LOGW("NRF24", "IRQ Triggered but No Data Ready!");
            }
        }
    }
}

// Initialize GPIO interrupt
void init_interrupt() {
    // Create a queue to handle interrupt events
    gpio_evt_queue = xQueueCreate(10, sizeof(int));

    // Configure GPIO 40 as input with pull-up and interrupt on falling edge
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INTERRUPT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // Falling edge interrupt
    };
    gpio_config(&io_conf);

    // Create a task to handle interrupt events
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // Install ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTERRUPT_PIN, gpio_isr_handler, (void*)INTERRUPT_PIN);

    ESP_LOGI(TAG, "GPIO interrupt initialized on GPIO %d", INTERRUPT_PIN);
}

// Send LED grid data via NRF24L01
void sendGrid(uint8_t grid[ARRAY_SIZE]) {
    ESP_LOGI(pcTaskGetName(NULL), "Start Sending Grid via NRF24L01");

    ESP_LOGI("NRF24", "Grid content:");
    for (int i = 0; i < ARRAY_SIZE; i++) {
        printf("%d ", grid[i]);
        if ((i + 1) % PAYLOAD_SIZE == 0) {
            printf("\n");
        }
    }

    for (int i = 0; i < ARRAY_SIZE; i += PAYLOAD_SIZE) {
        // Send chunks of 32 bytes
        Nrf24_send(&dev, &grid[i]);

        // Wait for sending confirmation
        if (Nrf24_isSend(&dev, 1000)) {
            ESP_LOGI("NRF24", "Chunk %d sent successfully", i / PAYLOAD_SIZE);
        } else {
            ESP_LOGW("NRF24", "Chunk %d failed", i / PAYLOAD_SIZE);
        }

        vTaskDelay(3 / portTICK_PERIOD_MS); // Small delay to avoid packet loss
    }
    ESP_LOGI("NRF24", "Full array sent!");
    vTaskDelay(1 / portTICK_PERIOD_MS); // Wait before next transmission
}

// Initialize WiFi access point
void start_wifi_ap() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap(); // Remove the unused variable

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    
    ESP_LOGI(TAG, "Wi-Fi AP started. SSID: %s", WIFI_SSID);
}


void tcp_server_task(void *pvParameters) {
    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // 1) create & bind & listen exactly as before
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    // … error‐checks omitted for brevity …
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(PORT);
    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    listen(listen_sock, 1);

    char buffer[1024];

    while (1) {
        // 2) Accept one client
        client_sock = accept(listen_sock,
                             (struct sockaddr *)&client_addr,
                             &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "accept failed");
            continue;
        }
        ESP_LOGI(TAG, "Client connected");

        // 3) Peek the first few bytes (no blocking beyond this)
        int len = recv(client_sock, buffer, sizeof(buffer)-1, 0);
        if (len <= 0) {
            ESP_LOGE(TAG, "recv failed or closed");
            close(client_sock);
            continue;
        }
        buffer[len] = '\0';

        // 4a) If it’s exactly "CAPTURE", grab & send a frame right now:
        if (strcmp(buffer, "CAPTURE") == 0) {
            ESP_LOGI(TAG, "CAPTURE command received");
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "camera capture failed");
            } else {
                // Send size (network‐order) then data
                int32_t sz = htonl(fb->len);
                send(client_sock, &sz, sizeof(sz), 0);
                send(client_sock, fb->buf, fb->len, 0);
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Frame sent (%d bytes)", fb->len);
            }
        }
        // 4b) Otherwise assume JSON grid: accumulate until ‘}’ and sendGrid()
        else if (buffer[0] == '{') {
            size_t idx = 0;
            static char json_buf[2048];

            // copy what we got
            memcpy(json_buf, buffer, len);
            idx += len;

            // keep reading until closing brace
            while (idx < sizeof(json_buf) - 1
                   && !strchr(json_buf, '}')) {
                len = recv(client_sock,
                           json_buf + idx,
                           sizeof(json_buf)-1 - idx,
                           0);
                if (len <= 0) break;
                idx += len;
            }
            json_buf[idx] = '\0';

            // parse & send
            cJSON *j = cJSON_Parse(json_buf);
            if (j) {
                cJSON *arr = cJSON_GetObjectItem(j, "leds");
                if (cJSON_IsArray(arr)
                    && cJSON_GetArraySize(arr) == ARRAY_SIZE) {
                    uint8_t grid[ARRAY_SIZE];
                    for (int i = 0; i < ARRAY_SIZE; i++) {
                        grid[i] = (uint8_t)
                                  cJSON_GetArrayItem(arr,i)->valueint;
                    }
                    sendGrid(grid);
                } else {
                    ESP_LOGE(TAG, "bad JSON array");
                }
                cJSON_Delete(j);
            } else {
                ESP_LOGE(TAG, "JSON parse error");
            }
        }
        else {
            ESP_LOGW(TAG, "unknown command: %s", buffer);
        }

        // 5) Done – close and loop
        shutdown(client_sock, SHUT_RDWR);
        close(client_sock);
        ESP_LOGI(TAG, "Connection closed, waiting for next");
    }

    // unreachable
    close(listen_sock);
    vTaskDelete(NULL);
}


void sendTestGrid(){
    uint8_t grid[ARRAY_SIZE];
    for (int i = 0; i < ARRAY_SIZE; i++) {
        grid[i] = (rand() % 3) + 1;
    }
    sendGrid(grid);
}

void app_main(void) {
#if ESP_CAMERA_SUPPORTED
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Initialize WiFi Access Point
    start_wifi_ap();

    /*
    // Initialize camera
    if(ESP_OK != init_camera()) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return;
    }
    */

    init_camera();
    
    // Initialize NRF24L01
    nrf_init();
    
    // Initialize GPIO interrupt for NRF24L01
    init_interrupt();
    
    // Start TCP server task
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System initialized successfully");
    
    // Keep the main task alive
    while(1) {
        //sendTestGrid();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
#else
    ESP_LOGE(TAG, "Camera support is not enabled");
#endif
}