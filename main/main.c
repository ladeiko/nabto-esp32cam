#include <stdio.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include <driver/gpio.h>

#include <esp_http_server.h>

#include "demo_application.h"
#include <unabto_aes_cbc_test.h>
//#include <unabto_hmac_sha256_test.h>

#include <unabto/unabto_common_main.h>
#include <unabto/unabto_app.h>
#include <unabto_tunnel_select.h>

// Set in menuconfig .. or override here
#define WIFI_SSID CONFIG_SSID
#define WIFI_PASS CONFIG_SSID_PASSWORD
#define NABTO_ID CONFIG_NABTO_ID
#define NABTO_KEY CONFIG_NABTO_KEY


// Event group
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

#ifdef CONFIG_ESP_EYE

// ESP-EYE wiring
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    4
#define CAM_PIN_SIOD    18
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7      36
#define CAM_PIN_D6      37
#define CAM_PIN_D5      38
#define CAM_PIN_D4      39
#define CAM_PIN_D3      35
#define CAM_PIN_D2      14
#define CAM_PIN_D1      13
#define CAM_PIN_D0      34
#define CAM_PIN_VSYNC    5
#define CAM_PIN_HREF    27
#define CAM_PIN_PCLK    25

#elif defined CONFIG_ESP_TINKER

// Ai Thinker CAM 32 wiring 
#define CAM_PIN_PWDN    32 
#define CAM_PIN_RESET   -1 
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#elif CONFIG_M5CAMERA_A
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   15 //software reset will be performed
#define CAM_PIN_XCLK    27
#define CAM_PIN_SIOD    25
#define CAM_PIN_SIOC    23

#define CAM_PIN_D7      19
#define CAM_PIN_D6      36
#define CAM_PIN_D5      18
#define CAM_PIN_D4      39
#define CAM_PIN_D3      5
#define CAM_PIN_D2      34
#define CAM_PIN_D1      35
#define CAM_PIN_D0      32
#define CAM_PIN_VSYNC   22
#define CAM_PIN_HREF    26
#define CAM_PIN_PCLK    21

#else
#error "Not defined - this should not happed!"
#endif

#define CAM_XCLK_FREQ   20000000

// SCAN OPTIONS
#define DEFAULT_SCAN_LIST_SIZE      32
#define CANDIDATE_SEPARATOR         '|'
#define SCAN_INTERVAL               30 // in seconds
#define MAX_CANDIDATES              64

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

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

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = CAM_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 10, //0-63 lower number means higher quality
    .fb_count = 2 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

/*
 * WIFI event handler
 */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        NABTO_LOG_INFO(("SYSTEM_EVENT_STA_START\n"));
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // NABTO_LOG_INFO(("SYSTEM_EVENT_STA_DISCONNECTED\n"));
        // xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        esp_restart();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        NABTO_LOG_INFO(("SYSTEM_EVENT_STA_GOT_IP -> connected1!\n"));
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

/*
 * Convert hex to i (for converting the hex key to bytebuffer)
 */
int hctoi(const unsigned char h) {
  if (isdigit(h)) {
    return h - '0';
  } else {
    return toupper(h) - 'A' + 10;
  }
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/*
 * MJPEG stream handler on http://<host>/
 */
esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            NABTO_LOG_ERROR(("Camera capture failed"));
            res = ESP_FAIL;
        } else {
            if(fb->format != PIXFORMAT_JPEG){
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                if(!jpeg_converted){
                    NABTO_LOG_ERROR(("JPEG compression failed"));
                    esp_camera_fb_return(fb);
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        NABTO_LOG_INFO(("MJPG: %uKB %ums (%.1ffps)",
            (uint32_t)(_jpg_buf_len/1024),
                        (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time));
    }

    last_frame = 0;
    return res;
}



/*
 * Hello world on http://<host>/uri
 */
esp_err_t get_handler(httpd_req_t *req)
{
    /* Send a simple response */
    const char* resp = "Hello world test!";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

/*
 * 1m stream of bytes on http://<host>/1m for stream performance testing
 */
esp_err_t get_handler_1m(httpd_req_t *req)
{
    /* Send a simple response */
    const char* resp =
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n"           \
        "0123456789012345678901234567890123456789012345678\n";

    // Create a 1k byte buffer chuck
    char resp_1k[1000];
    int len = 1000;
    for(int i=0;i<10;i++) {
        memcpy(resp_1k+(i*100), resp, 100);
    }
    resp_1k[999]=0;

    NABTO_LOG_INFO(("strlen:%d 1000000/len:%d", len, 1000000/len));

    int64_t start_time = esp_timer_get_time();

    // Send 1m chunk
    for(int t=0; t< 1000000/len; t++) {
        int res = httpd_resp_send_chunk(req, (const char *) resp_1k, len);
        if(res != ESP_OK) {
            NABTO_LOG_INFO(("Could not send chunk %i", t));
            return ESP_FAIL;
        }
    }
    // send last 0 chunk to end the stream
    httpd_resp_send_chunk(req, (const char *) resp, 0);

    int64_t end_time = esp_timer_get_time();
    int64_t time = end_time - start_time;
    float kb = 1000000/1024;

    NABTO_LOG_INFO(("Finished in %lldms kb=%f kb/s=%f", time/1000, kb, kb/((float)time/1000000)));

    return ESP_OK;
}


/* URI handler structure for GET /uri */
httpd_uri_t uri_get = {
    .uri      = "/uri",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
};


/* URI handler structure for GET /1m */
httpd_uri_t uri_get_1m = {
    .uri      = "/1m",
    .method   = HTTP_GET,
    .handler  = get_handler_1m,
    .user_ctx = NULL
};

/* URI handler structure for GET / */
httpd_uri_t uri_get_mjpeg = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = jpg_stream_httpd_handler,
    .user_ctx = NULL
};


/*
 * Function for starting the webserver
 */
httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8081;

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_get_1m);
        httpd_register_uri_handler(server, &uri_get_mjpeg);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

/*
 * Function for stopping the webserver (not used)
 */
void stop_webserver(httpd_handle_t server)
{
    if (server) {
        /* Stop the httpd server */
        httpd_stop(server);
    }
}


/*
 * Main task - initialize WIFI and start Nabto tick
 */
void main_task(void *pvParameter)
{
  // device id and key from developer.nabto.com
  const char* nabtoId = NABTO_ID;
  const char* presharedKey = NABTO_KEY;
  NABTO_LOG_INFO(("NabtoId:%s", nabtoId));
  NABTO_LOG_INFO(("NabtoKey:%s", presharedKey));

  // wait for connection
  NABTO_LOG_INFO(("Main task: waiting for connection to the wifi network... "));
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
  NABTO_LOG_INFO(("connected!\n"));

  // print the local IP address
  tcpip_adapter_ip_info_t ip_info;
  ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));

  NABTO_LOG_INFO(("IP Address:  %s", ip4addr_ntoa(&ip_info.ip)));
  NABTO_LOG_INFO(("Subnet mask: %s", ip4addr_ntoa(&ip_info.netmask)));
  NABTO_LOG_INFO(("Gateway:     %s", ip4addr_ntoa(&ip_info.gw)));

  // Start the webserver thread
  start_webserver();

  // Configure Nabto with the right configuration
  nabto_main_setup* nms = unabto_init_context();
  nms->id = NABTO_ID;
  //nms->ipAddress = ip_info.ip.addr;
  nms->id = nabtoId;
  nms->secureAttach = 1;
  nms->secureData = 1;
  nms->cryptoSuite = CRYPT_W_AES_CBC_HMAC_SHA256;

  const char* p;
  unsigned char* up;
  for (p = presharedKey, up = nms->presharedKey; *p; p += 2, ++up) {
    *up = hctoi(p[0]) * 16 + hctoi(p[1]);  // hex string to byte array
  }

  // Init demo application
  demo_init();

  tunnel_loop_select();
  esp_restart();
}

static const char *TAG = "scan";

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }

    switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }
}

static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    default:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}

static void swap(wifi_ap_record_t *xp, wifi_ap_record_t *yp)
{
    const wifi_ap_record_t temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Move points with higher RSSI to the beginning
static void sortApRecordsByRSSI(wifi_ap_record_t arr[], size_t n)
{
   for (size_t i = 0; i < n - 1; i++) {
       for (size_t j = 0; j < n - i - 1; j++) {
           if (arr[j].rssi < arr[j + 1].rssi) {
              swap(&arr[j], &arr[j + 1]);
           }
       }
   }
}

static size_t numberOfWifiCandidates = 0;
static wifi_config_t wifiCandidates[MAX_CANDIDATES] = {0};
static size_t targetWifiCandidate = -1;

static void load_wifi_candidates()
{
    ESP_LOGI(TAG, "setup_wifi_candidates enter");

    numberOfWifiCandidates = 0;
    memset(wifiCandidates, 0, sizeof(wifiCandidates));

    const char* ssid = WIFI_SSID;
    const char* pass = WIFI_PASS;

    while (numberOfWifiCandidates < (sizeof(wifiCandidates) / sizeof(wifiCandidates[0]))) {

        const char* const nextSsid = strchr(ssid, CANDIDATE_SEPARATOR);
        const char* const nextPass = strchr(pass, CANDIDATE_SEPARATOR);

        if (nextSsid) {
            ESP_LOGI(TAG, "setup_wifi_candidates nextSsid = %s", nextSsid);
        }

        if (nextPass) {
            ESP_LOGI(TAG, "setup_wifi_candidates nextPass = %s", nextPass);
        }

        const size_t ssidLength = nextSsid ? nextSsid - ssid : strlen(ssid);
        const size_t passLength = nextPass ? nextPass - pass : strlen(pass);

        if (ssidLength > 0 
                && ssidLength <= sizeof(wifiCandidates[numberOfWifiCandidates].sta.ssid)
                && passLength <= sizeof(wifiCandidates[numberOfWifiCandidates].sta.password)
            ) {
            memcpy(wifiCandidates[numberOfWifiCandidates].sta.ssid, ssid, ssidLength);
            memcpy(wifiCandidates[numberOfWifiCandidates].sta.password, pass, passLength);
            numberOfWifiCandidates += 1;
        }

        if (!nextSsid) {
            break;
        }

        ssid = nextSsid + 1;

        if (nextPass) {
            pass = nextPass + 1;
        }
    }

    ESP_LOGI(TAG, "setup_wifi_candidates numberOfWifiCandidates = %d", (int)numberOfWifiCandidates);

    for (size_t i = 0; i < numberOfWifiCandidates; ++i) {
        char ssid[33] = {0};
        char password[65] = {0};
        memcpy(ssid, wifiCandidates[i].sta.ssid, sizeof(wifiCandidates[i].sta.ssid));
        memcpy(password, wifiCandidates[i].sta.password, sizeof(wifiCandidates[i].sta.password));
        ESP_LOGI(TAG, "setup_wifi_candidates candidate = '%s' / '%s'", ssid, password);
    }

    ESP_LOGI(TAG, "setup_wifi_candidates leave");
}

/* Sleep current thread for specified number of seconds */
static void cross_sleep_sec(int sec) {
    const TickType_t xDelay = (sec * 1000) / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);
}

/* Initialize Wi-Fi as sta and set scan method */
static void find_known_wifi(void)
{
    ESP_LOGI(TAG, "wifi_scan enter\n");

    const uint16_t maxNumber = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t* const ap_info = malloc(sizeof(wifi_ap_record_t) * maxNumber);

    ESP_ERROR_CHECK(esp_wifi_start());

    targetWifiCandidate = -1;

    while (true) {

        uint16_t ap_count = 0;
        uint16_t number = maxNumber;
        memset(ap_info, 0, sizeof(wifi_ap_record_t) * maxNumber);

        esp_wifi_scan_start(NULL, true);
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

        ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);

        sortApRecordsByRSSI(ap_info, ap_count);

        for (int i = 0; (i < maxNumber) && (i < ap_count); i++) {
            ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
            ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
            print_auth_mode(ap_info[i].authmode);
            if (ap_info[i].authmode != WIFI_AUTH_WEP) {
                print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
            }
            ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
        }

        esp_wifi_scan_stop();

        for (int i = 0; (targetWifiCandidate == -1) && (i < maxNumber) && (i < ap_count); i++) {
            char ssid1[33]             = {0};
            memcpy(ssid1, ap_info[i].ssid, sizeof(ap_info[i].ssid));
            for (size_t j = 0; j < numberOfWifiCandidates; ++j) {
                char ssid2[33]             = {0};
                memcpy(ssid2, wifiCandidates[j].sta.ssid, sizeof(wifiCandidates[j].sta.ssid));
                if (strcmp(ssid1, ssid2) == 0) {
                    targetWifiCandidate = j;
                    break;
                }
            }
        }

        if (targetWifiCandidate != -1) {
            ESP_LOGI(TAG, "targetWifiCandidate => %d", (int)targetWifiCandidate);
            break;
        }

        ESP_LOGI(TAG, "Waiting for the next scan %d second(s)\n", SCAN_INTERVAL);
        cross_sleep_sec(SCAN_INTERVAL);
    }

    free(ap_info);

    ESP_ERROR_CHECK(esp_wifi_stop());

    ESP_LOGI(TAG, "wifi_scan leave\n");
}

/*
 * Main application entry
 */
void app_main()
{
  NABTO_LOG_INFO(("Nabto ESP32 demo starting up!!!"));

  NABTO_LOG_INFO(("Initializing nvs flash"));

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      NABTO_LOG_INFO(("Erasing flash"));
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  //initialize the camera
  NABTO_LOG_INFO(("Initializing camera"));
 
#if CAMERA_WIRING==ESP_EYE
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

  ret = esp_camera_init(&camera_config);
  if (ret != ESP_OK) {
      NABTO_LOG_ERROR(("Camera Init Failed"));
  }

  // disable stdout buffering
  setvbuf(stdout, NULL, _IONBF, 0);

  // Parse WiFi networks descriptions
  load_wifi_candidates();

  // Initialize network stack
  ESP_ERROR_CHECK(esp_netif_init());

  // create the event group to handle wifi events
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  find_known_wifi();

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  // configure the wifi connection and start the interface
  wifi_config_t wifi_config = { 0 };
  strlcpy((char*)wifi_config.sta.ssid, (const char*)wifiCandidates[targetWifiCandidate].sta.ssid, sizeof(wifi_config.sta.ssid));
  strlcpy((char*)wifi_config.sta.password, (const char*)wifiCandidates[targetWifiCandidate].sta.password, sizeof(wifi_config.sta.password));

  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  NABTO_LOG_INFO(("Connecting to %s\n", wifiCandidates[targetWifiCandidate].sta.ssid));

  // start the main task
  xTaskCreate(&main_task, "main_task", 8192, NULL, 5, NULL);
}
