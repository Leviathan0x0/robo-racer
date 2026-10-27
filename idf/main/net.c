#include "net.h"

#include <errno.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "mailbox.h"

static const char *TAG = "net";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char styles_css_start[] asm("_binary_styles_css_start");
extern const char app_js_start[] asm("_binary_app_js_start");

static httpd_handle_t s_server;
static uint32_t s_token;
static uint32_t s_link_timeout_ms = 2000;
static rr_decoder_t s_decoder;
static atomic_int s_fd = -1;
static atomic_uint s_last_rx_ms;
static esp_timer_handle_t s_watchdog;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void drop_controller(int fd) {
    int expected = fd;
    if (atomic_compare_exchange_strong(&s_fd, &expected, -1)) {
        mailbox_set_link(false);
    }
}

static void link_watchdog(void *arg) {
    (void)arg;
    const int fd = atomic_load(&s_fd);
    if (fd < 0 || s_server == NULL) {
        return;
    }
    const uint32_t age = now_ms() - atomic_load(&s_last_rx_ms);
    if (age <= s_link_timeout_ms) {
        return;
    }
    int expected = fd;
    if (atomic_compare_exchange_strong(&s_fd, &expected, -1)) {
        mailbox_set_link(false);
        httpd_sess_trigger_close(s_server, fd);
        ESP_LOGW(TAG, "control link silent for %" PRIu32 " ms", age);
    }
}

static void tune_socket(int fd) {
    const int nodelay = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)) != 0) {
        ESP_LOGW(TAG, "TCP_NODELAY was not set on fd %d", fd);
    }
}

static esp_err_t ws_send(httpd_req_t *req, const uint8_t *data, size_t len) {
    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = (uint8_t *)data,
        .len = len,
    };
    return httpd_ws_send_frame(req, &frame);
}

static void fill_status(uint8_t frame[RR_STATUS_FRAME_LEN]) {
    rr_published_t published;
    mailbox_read(&published);
    uint8_t flags = RR_STATUS_SESSION_OK;
    if (published.state == RR_STATE_ARMED || published.state == RR_STATE_DRIVING) {
        flags |= RR_STATUS_ARMED;
    }
    if (published.link_up) {
        flags |= RR_STATUS_LINK_UP;
    }
    if (published.battery_ok) {
        flags |= RR_STATUS_BATTERY_OK;
    }
    rr_encode_status(published.state, published.has_fault, published.fault, flags, published.battery_mv, published.left,
                     published.right, published.client_seq, published.mode, frame);
}

static esp_err_t send_status(httpd_req_t *req) {
    uint8_t frame[RR_STATUS_FRAME_LEN];
    fill_status(frame);
    return ws_send(req, frame, sizeof(frame));
}

typedef struct {
    httpd_handle_t handle;
    int fd;
} rr_hello_job_t;

// One unmasked binary frame, payload under 126 bytes, appended to `out`.
static size_t append_ws(uint8_t *out, const uint8_t *payload, size_t len) {
    out[0] = 0x82;
    out[1] = (uint8_t)len;
    memcpy(out + 2, payload, len);
    return 2 + len;
}

// Runs on the HTTP task after the handshake handler has returned. A raw send
// is safe here: this task is the only owner of the socket, and the client
// does not transmit until HELLO arrives. One write carries both frames so
// they leave in a single TCP segment.
static void send_opening(void *arg) {
    rr_hello_job_t *job = arg;
    if (atomic_load(&s_fd) != job->fd) {
        free(job);
        return;
    }
    uint8_t hello[RR_HELLO_FRAME_LEN];
    uint8_t status[RR_STATUS_FRAME_LEN];
    rr_encode_hello(s_token, now_ms(), hello);
    fill_status(status);

    uint8_t packet[2 + RR_HELLO_FRAME_LEN + 2 + RR_STATUS_FRAME_LEN];
    size_t used = append_ws(packet, hello, sizeof(hello));
    used += append_ws(packet + used, status, sizeof(status));

    size_t sent = 0;
    bool ok = true;
    while (sent < used) {
        const int n = send(job->fd, packet + sent, used - sent, 0);
        if (n > 0) {
            sent += (size_t)n;
            continue;
        }
        // The socket blocks up to SO_SNDTIMEO, so a failed send must not be retried.
        ok = false;
        break;
    }
    if (!ok) {
        drop_controller(job->fd);
        httpd_sess_trigger_close(job->handle, job->fd);
    }
    free(job);
}

static void handle_payload(httpd_req_t *req, const uint8_t *data, size_t len) {
    rr_client_frame_t frame;
    const int error = rr_decoder_decode(&s_decoder, data, (uint32_t)len, &frame);
    if (error != RR_PROTO_OK) {
        uint8_t reply[RR_ERROR_FRAME_LEN];
        rr_encode_error((uint8_t)error, reply);
        ws_send(req, reply, sizeof(reply));
        send_status(req);
        return;
    }
    mailbox_post(&frame);
    if (frame.kind == RR_KIND_PING) {
        uint8_t pong[RR_PONG_FRAME_LEN];
        // The Rust firmware echoes the client sequence, not a timestamp.
        rr_encode_pong(frame.seq, now_ms(), pong);
        ws_send(req, pong, sizeof(pong));
    }
    send_status(req);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    const int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        int expected = -1;
        if (!atomic_compare_exchange_strong(&s_fd, &expected, fd)) {
            // One controller. This new session is deleted by the error return.
            return ESP_FAIL;
        }
        tune_socket(fd);
        atomic_store(&s_last_rx_ms, now_ms());
        s_decoder = rr_decoder_new(s_token);
        mailbox_set_link(true);

        rr_hello_job_t *job = malloc(sizeof(*job));
        if (job == NULL) {
            drop_controller(fd);
            return ESP_FAIL;
        }
        job->handle = req->handle;
        job->fd = fd;
        if (httpd_queue_work(req->handle, send_opening, job) != ESP_OK) {
            free(job);
            drop_controller(fd);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    if (fd != atomic_load(&s_fd)) {
        return ESP_FAIL;
    }

    httpd_ws_frame_t ws;
    memset(&ws, 0, sizeof(ws));
    if (httpd_ws_recv_frame(req, &ws, 0) != ESP_OK) {
        drop_controller(fd);
        // CLOSE already queued trigger_close. ESP_FAIL would free that session.
        if (ws.type == HTTPD_WS_TYPE_CLOSE) {
            return ESP_OK;
        }
        return ESP_FAIL;
    }
    atomic_store(&s_last_rx_ms, now_ms());

    if (ws.type == HTTPD_WS_TYPE_CLOSE) {
        drop_controller(fd);
        // httpd_req_delete queues the only close. ESP_FAIL would free it first.
        return ESP_OK;
    }
    if (ws.type == HTTPD_WS_TYPE_PING) {
        if (ws.len > 125) {
            drop_controller(fd);
            return ESP_FAIL;
        }
        uint8_t buf[125];
        if (ws.len > 0) {
            ws.payload = buf;
            if (httpd_ws_recv_frame(req, &ws, ws.len) != ESP_OK) {
                drop_controller(fd);
                return ESP_FAIL;
            }
        }
        ws.final = true;
        ws.type = HTTPD_WS_TYPE_PONG;
        if (httpd_ws_send_frame(req, &ws) != ESP_OK) {
            drop_controller(fd);
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    if (ws.type == HTTPD_WS_TYPE_PONG) {
        if (ws.len > 125) {
            drop_controller(fd);
            return ESP_FAIL;
        }
        if (ws.len > 0) {
            uint8_t buf[125];
            ws.payload = buf;
            if (httpd_ws_recv_frame(req, &ws, ws.len) != ESP_OK) {
                drop_controller(fd);
                return ESP_FAIL;
            }
        }
        return ESP_OK;
    }
    // Same fail-closed rule as the Rust codec: no fragments, no large frames.
    // `fragmented` is the driver's flag. Don't also require `final`: a zeroed
    // header field would reject every command.
    if (ws.fragmented || ws.len > 64) {
        drop_controller(fd);
        return ESP_FAIL;
    }
    if (ws.len == 0) {
        handle_payload(req, (const uint8_t *)"", 0);
        return ESP_OK;
    }

    uint8_t buffer[64];
    ws.payload = buffer;
    if (httpd_ws_recv_frame(req, &ws, ws.len) != ESP_OK) {
        drop_controller(fd);
        return ESP_FAIL;
    }
    atomic_store(&s_last_rx_ms, now_ms());
    handle_payload(req, buffer, ws.len);
    return ESP_OK;
}

static esp_err_t send_asset(httpd_req_t *req, const char *type, const char *body) {
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t index_get(httpd_req_t *req) {
    return send_asset(req, "text/html; charset=utf-8", index_html_start);
}

static esp_err_t css_get(httpd_req_t *req) {
    return send_asset(req, "text/css; charset=utf-8", styles_css_start);
}

static esp_err_t js_get(httpd_req_t *req) {
    return send_asset(req, "application/javascript; charset=utf-8", app_js_start);
}

static void make_password(char out[13]) {
    static const char alphabet[] = "abcdefghijkmnopqrstuvwxyz23456789";
    uint8_t random_bytes[12];
    esp_fill_random(random_bytes, sizeof(random_bytes));
    for (int i = 0; i < 12; i++) {
        out[i] = alphabet[random_bytes[i] % (sizeof(alphabet) - 1u)];
    }
    out[12] = '\0';
}

static bool nvs_ready(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) {
            return false;
        }
        err = nvs_flash_init();
    }
    return err == ESP_OK;
}

bool net_start(const rr_config_t *cfg) {
    s_link_timeout_ms = cfg->safety.link_timeout_ms;

    const char *ssid = CONFIG_ROBO_WIFI_SSID;
    const size_t ssid_len = strlen(ssid);
    if (ssid_len == 0 || ssid_len > 32) {
        ESP_LOGE(TAG, "SSID must be 1..32 characters");
        return false;
    }

    char generated[13];
    const char *password = CONFIG_ROBO_WIFI_PASS;
    const size_t password_len = strlen(password);
    if (password_len == 0) {
        make_password(generated);
        password = generated;
    } else if (password_len < 8 || password_len > 63) {
        ESP_LOGE(TAG, "Wi-Fi password must be empty (random) or 8..63 characters");
        return false;
    }

    if (!nvs_ready()) {
        ESP_LOGE(TAG, "NVS init failed");
        return false;
    }
    if (esp_netif_init() != ESP_OK || esp_event_loop_create_default() != ESP_OK) {
        ESP_LOGE(TAG, "network stack init failed");
        return false;
    }
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "softAP netif failed");
        return false;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    // 12-byte command frames must hit the air immediately. Aggregation waits
    // to fill a burst, which is the wrong trade for a drive command.
    wifi_init.ampdu_rx_enable = 0;
    wifi_init.ampdu_tx_enable = 0;
    wifi_init.amsdu_tx_enable = 0;
    if (esp_wifi_init(&wifi_init) != ESP_OK) {
        ESP_LOGE(TAG, "wifi init failed");
        return false;
    }

    wifi_config_t wifi = {0};
    memcpy(wifi.ap.ssid, ssid, ssid_len);
    wifi.ap.ssid_len = (uint8_t)ssid_len;
    memcpy(wifi.ap.password, password, strlen(password));
    wifi.ap.channel = (uint8_t)CONFIG_ROBO_WIFI_CHANNEL;
    wifi.ap.max_connection = 4;
    wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi.ap.beacon_interval = 100;
    wifi.ap.pmf_cfg.required = false;

    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK || esp_wifi_set_config(WIFI_IF_AP, &wifi) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "access point failed to start");
        return false;
    }

    // Modem sleep adds tens of milliseconds. The robot is on a pack, not a coin cell.
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    esp_wifi_set_max_tx_power(80);

    uint32_t token = 0;
    while (token == 0) {
        token = esp_random();
    }
    s_token = token;
    s_decoder = rr_decoder_new(token);

    printf("robo-racer: access point '%s' up, password '%s', open http://192.168.4.1\n", ssid, password);
    printf("robo-racer: session token %08" PRIx32 "\n", token);
    fflush(stdout);

    httpd_config_t http = HTTPD_DEFAULT_CONFIG();
    http.server_port = 80;
    http.core_id = 0;
    http.stack_size = 8192;
    http.max_open_sockets = 6;
    http.max_uri_handlers = 8;
    http.lru_purge_enable = true;
    http.recv_wait_timeout = 3;
    http.send_wait_timeout = 2;
    if (httpd_start(&s_server, &http) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed");
        return false;
    }

    const httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_get};
    const httpd_uri_t index_html = {.uri = "/index.html", .method = HTTP_GET, .handler = index_get};
    const httpd_uri_t css_uri = {.uri = "/styles.css", .method = HTTP_GET, .handler = css_get};
    const httpd_uri_t js_uri = {.uri = "/app.js", .method = HTTP_GET, .handler = js_get};
    const httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
        .handle_ws_control_frames = true,
    };
    if (httpd_register_uri_handler(s_server, &index_uri) != ESP_OK ||
        httpd_register_uri_handler(s_server, &index_html) != ESP_OK ||
        httpd_register_uri_handler(s_server, &css_uri) != ESP_OK ||
        httpd_register_uri_handler(s_server, &js_uri) != ESP_OK ||
        httpd_register_uri_handler(s_server, &ws_uri) != ESP_OK) {
        ESP_LOGE(TAG, "URI registration failed");
        return false;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = link_watchdog,
        .name = "link",
    };
    if (esp_timer_create(&timer_args, &s_watchdog) != ESP_OK ||
        esp_timer_start_periodic(s_watchdog, 200 * 1000) != ESP_OK) {
        ESP_LOGE(TAG, "link watchdog failed");
        return false;
    }
    return true;
}
