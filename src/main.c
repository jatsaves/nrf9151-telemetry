/*
todo
-> obtain meter resets
-> full sleep shutdown, use rtc to wake up/turn on again for daily
-> de power the rs232 transceiver, like power with digital output maybe, and set to no voltage to turn off
-> psm mode, leave modem on and maybe connected, investigate for 15 min
-> pulse input to wake from deep sleep, change to 15minute rate, change back to daily when stop watering
-> compress/zip the payload maybe
-> receive config updates over the air COTA
-> use async lte_lc_connect and wait, as current is blocking if antenna not connected lte_lc_connect_async(lte_handler); with semaphore
-> mqtt publish raw modbus response packet/bytes too
-> system off, will lose memeory data, if not RETAIN config, or write to flahs, or eeprom
-> avoid writing to flash, to reduce wear. there is system off retain ram, if want to use when system off [deepest sleep], ai suggests not too SYSTEM OFF
-> only timesync daily, not hourly
-> preventative reboot....reboot daily or weekly, network issues, memory leaks
-> watchdog reboot if error
-> remove excessive print
-> dump error log
-> random sleep, say 0-60 seconds, to avoid collisions, then do modbus and mqtt publish
-> mqtts, i understand now, should not need to change the blob
-> modbus retries
-> different timer for modbus reading, versus publishing
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <date_time.h>

#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem_at.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>

#define MB_UART_NODE DT_NODELABEL(uart1)
#define SLEEP_INTERVAL_S (60 * 60)

#define FW_VERSION "0.6.1"
#define DEVICE_NAME "nrf1"
#define MQTT_TOPIC "devices/" DEVICE_NAME
#define EVENT_COLD_BOOT 1000
#define MODBUS_RESPONSE_LEN 19
#define MODBUS_RETRIES      1

static const struct device *mb_uart = DEVICE_DT_GET(MB_UART_NODE);

static struct mqtt_client client;
static struct sockaddr_storage broker;

static uint8_t rx_buffer[1024];
static uint8_t tx_buffer[1024];

#define MEASUREMENT_HISTORY_SIZE 10
#define ERROR_HISTORY_SIZE 10
#define ERROR_MSG_LEN      64

typedef int  (*build_payload_fn_t)(char *buf, size_t len);
typedef void (*clear_queue_fn_t)(void);
typedef uint8_t (*count_queue_fn_t)(void);

static bool mqtt_connected;

typedef struct
{
    bool     utc_valid;
    int64_t  timestamp;

    uint16_t flow;
    uint16_t battery;
    uint16_t solar;

    uint32_t peak;
    uint32_t offpeak;

    int16_t  rsrp_dbm;

} measurement_t;

typedef struct
{
    bool     utc_valid;
    int64_t  timestamp;    // UTC ms if utc_valid, otherwise oh yeah,uptime ms
    int32_t  error_code;
    char     message[ERROR_MSG_LEN];
} error_entry_t;

static measurement_t measurements[MEASUREMENT_HISTORY_SIZE];
static error_entry_t error_log[ERROR_HISTORY_SIZE];
static uint8_t measurement_head;
static uint8_t measurement_count;
static uint8_t error_head;
static uint8_t error_count;    

static void measurement_log_clear(void)
{
    measurement_head = 0;
    measurement_count = 0;
}

static void error_log_clear(void)
{
    error_head = 0;
    error_count = 0;
}

static uint8_t measurement_log_count(void)
{
    return measurement_count;
}

static uint8_t error_log_count(void)
{
    return error_count;
}

static void measurement_log_add(const measurement_t *m)
{
    measurements[measurement_head] = *m;

    measurement_head =
        (measurement_head + 1) % MEASUREMENT_HISTORY_SIZE;

    if (measurement_count < MEASUREMENT_HISTORY_SIZE) {
        measurement_count++;
    }
}

static void error_log_add(int32_t error_code,
                   const char *fmt, ...)
{
    error_entry_t *entry = &error_log[error_head];

    entry->error_code = error_code;

    if (date_time_is_valid() &&
        date_time_now(&entry->timestamp) == 0) {
        entry->utc_valid = true;
    } else {
        entry->utc_valid = false;
        entry->timestamp = k_uptime_get();
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry->message,
              sizeof(entry->message),
              fmt,
              args);
    va_end(args);

    error_head = (error_head + 1) % ERROR_HISTORY_SIZE;

    if (error_count < ERROR_HISTORY_SIZE)
        error_count++;
}

static const measurement_t *measurement_log_get(uint8_t i)
{
    if (i >= measurement_count) {
        return NULL;
    }

    uint8_t index =
        (measurement_head + MEASUREMENT_HISTORY_SIZE - measurement_count + i) %
        MEASUREMENT_HISTORY_SIZE;

    return &measurements[index];
}

static const error_entry_t *error_log_get(uint8_t i)
{
    if (i >= error_count) {
        return NULL;
    }

    uint8_t index =
        (error_head  + ERROR_HISTORY_SIZE - error_count + i) %
        ERROR_HISTORY_SIZE;

    return &error_log[index];
}

static int build_measurement_payload(char *buf, size_t len)
{
    int written;

    written = snprintf(buf, len,
        "{"
        "\"d\":\"" DEVICE_NAME "\","
        "\"v\":\"" FW_VERSION "\","
        "\"measurements\":[");
    if (written < 0 || (size_t)written >= len)
        return -ENOMEM;

    size_t pos = (size_t)written;

    uint8_t count = measurement_log_count();
    for (uint8_t i = 0; i < count; i++) {

        const measurement_t *m = measurement_log_get(i);
        if (m == NULL) {
            return -EINVAL;
        }

        written = snprintf(
            buf + pos,
            len - pos,
            "%s{"
            "\"t\":%lld,"
            "\"utc\":%s,"
            "\"p\":%u,"
            "\"o\":%u,"
            "\"f\":%u,"
            "\"b\":%u,"
            "\"s\":%u,"
            "\"q\":%d"
            "}",
            (i == 0) ? "" : ",",
            (long long)m->timestamp,
            m->utc_valid ? "true" : "false",
            m->peak,
            m->offpeak,
            m->flow,
            m->battery,
            m->solar,
            m->rsrp_dbm);

        if (written < 0 || (size_t)written >= (len - pos))
            return -ENOMEM;

        pos += (size_t)written;
    }

    written = snprintf(buf + pos, len - pos, "]}");

    if (written < 0 || (size_t)written >= (len - pos))
        return -ENOMEM;

    return 0;
}

static int build_error_payload(char *buf, size_t len)
{
    int written = 0;

    written = snprintf(buf, len,
        "{"
        "\"d\":\"" DEVICE_NAME "\","
        "\"v\":\"" FW_VERSION "\","
        "\"errors\":[");
    if (written < 0 || (size_t)written >= len)
        return -ENOMEM;

    size_t pos = (size_t)written;

    uint8_t count = error_log_count();
    for (uint8_t i = 0; i < count; i++) {

        const error_entry_t *e = error_log_get(i);
        if (e == NULL) {
            return -EINVAL;
        }

        written = snprintf(buf + pos,
                   len - pos,
                   "%s{\"t\":%lld,\"utc\":%s,\"c\":%d,\"m\":\"%s\"}",
                   (i == 0) ? "" : ",",
                   (long long)e->timestamp,
                   e->utc_valid ? "true" : "false",
                   (int)e->error_code,
                   e->message);

        if (written < 0 || (size_t)written >= (len - pos))
            return -ENOMEM;

        pos += (size_t)written;
    }

    written = snprintf(buf + pos, len - pos, "]}");

    if (written < 0 || (size_t)written >= (len - pos))
        return -ENOMEM;

    return 0;
}

static void sleep_until_next_wakeup(void)
{
    int64_t now_ms;

    if (date_time_is_valid() &&
        date_time_now(&now_ms) == 0) {

        int64_t now_sec = now_ms / 1000;

        int64_t next_wakeup =
            ((now_sec / SLEEP_INTERVAL_S) + 1) * SLEEP_INTERVAL_S;

        int64_t sleep_sec = next_wakeup - now_sec;

        printk("Sleeping %lld seconds until next wakeup\n",
               sleep_sec);

        k_sleep(K_SECONDS(sleep_sec));

    } else {

        printk("UTC time unavailable, sleeping %d seconds\n",
               SLEEP_INTERVAL_S);

        k_sleep(K_SECONDS(SLEEP_INTERVAL_S));
    }
}

static int uart_send_bytes(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(mb_uart, data[i]);
    }
    return 0;
}

static int uart_read_bytes(uint8_t *buf,
                           size_t max_len,
                           int first_byte_timeout_ms)
{
    const int inter_byte_timeout_ms = 10;

    int count = 0;
    int64_t timeout;

    /* Wait for the first byte */
    timeout = k_uptime_get() + first_byte_timeout_ms;

    while (k_uptime_get() < timeout) {

        if (uart_poll_in(mb_uart, &buf[0]) == 0) {
            count = 1;
            break;
        }

        k_sleep(K_MSEC(1));
    }

    if (count == 0) {
        return 0;   /* No response */
    }

    /* Read until no more bytes arrive */
    timeout = k_uptime_get() + inter_byte_timeout_ms;

    while (count < max_len) {

        if (uart_poll_in(mb_uart, &buf[count]) == 0) {
            count++;

            /* Another byte arrived, extend timeout */
            timeout = k_uptime_get() + inter_byte_timeout_ms;
        } else {
            if (k_uptime_get() >= timeout) {
                break;
            }

            k_sleep(K_MSEC(1));
        }
    }

    return count;
}

#if 0
static void run_at(const char *cmd)
{
    int err;
    char resp[256] = {0};

    printk("\n=== %s ===\n", cmd);

    err = nrf_modem_at_cmd(resp,
                           sizeof(resp),
                           "%s",
                           cmd);

    printk("err=%d\n", err);

    if (err == 0) {
        printk("%s\n", resp);
    }
}
#endif

static void mqtt_evt_handler(struct mqtt_client *const client,
                             const struct mqtt_evt *evt)
{
    ARG_UNUSED(client);
    printk("MQTT event: %d\n", evt->type);

    switch (evt->type) {

    case MQTT_EVT_CONNACK:
        printk("MQTT connected\n");
        mqtt_connected = true;
        break;
    case MQTT_EVT_DISCONNECT:
        mqtt_connected = false;
        printk("MQTT disconnected\n");
        break;
    case MQTT_EVT_PUBLISH:
        printk("MQTT publish received\n");
        break;

    default:
        break;
    }
}

static int mqtt_publish_method(const char *payload)
{
    int err;

    mqtt_client_init(&client);

    client.broker = &broker;
    client.evt_cb = mqtt_evt_handler;
    client.client_id.utf8 = (uint8_t *)DEVICE_NAME;
    client.client_id.size = strlen(DEVICE_NAME);

    static uint8_t mqtt_user[] = "iotuser";
    static uint8_t mqtt_password[] = "rit2026!";

    client.user_name = &(struct mqtt_utf8){
        .utf8 = mqtt_user,
        .size = strlen((char *)mqtt_user)
    };

    client.password = &(struct mqtt_utf8){
        .utf8 = mqtt_password,
        .size = strlen((char *)mqtt_password)
    };

    client.protocol_version = MQTT_VERSION_3_1_1;

    client.rx_buf = rx_buffer;
    client.rx_buf_size = sizeof(rx_buffer);

    client.tx_buf = tx_buffer;
    client.tx_buf_size = sizeof(tx_buffer);

    struct sockaddr_in *addr =
        (struct sockaddr_in *)&broker;

    addr->sin_family = AF_INET;
    addr->sin_port = htons(1883);

    zsock_inet_pton(AF_INET,
                    "202.90.240.102",
                    &addr->sin_addr);

    mqtt_connected = false;

    err = mqtt_connect(&client);
    if (err) {
        printk("mqtt_connect failed: %d\n", err);
        return err;
    }

    printk("Waiting for CONNACK...\n");

    for (int i = 0; i < 50 && !mqtt_connected; i++) {

        err = mqtt_input(&client);
        if (err && err != -EAGAIN) {
            (void)mqtt_disconnect(&client, NULL);
            return err;
        }

        err = mqtt_live(&client);
        if (err && err != -EAGAIN) {
            (void)mqtt_disconnect(&client, NULL);
            return err;
        }

        k_sleep(K_MSEC(100));
    }

    if (!mqtt_connected) {
        printk("MQTT connection timed out\n");
        (void)mqtt_disconnect(&client, NULL);
        return -ETIMEDOUT;
    }

    /* Service MQTT once more before publishing */
    err = mqtt_input(&client);
    if (err && err != -EAGAIN) {
        (void)mqtt_disconnect(&client, NULL);
        return err;
    }

    static const char topic[] = MQTT_TOPIC;

    struct mqtt_publish_param param = {
        .message.topic.topic = {
            .utf8 = (uint8_t *)topic,
            .size = strlen(topic)
        },
        .message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE,
        .message.payload.data = (uint8_t *)payload,
        .message.payload.len = strlen(payload),
        .message_id = 1,
        .dup_flag = 0,
        .retain_flag = 0
    };

    err = mqtt_publish(&client, &param);
    if (err) {
        printk("mqtt_publish failed: %d\n", err);
        (void)mqtt_disconnect(&client, NULL);
        return err;
    }

    printk("mqtt_publish = %d\n", err);

    int64_t end = k_uptime_get() + 500;
    while (k_uptime_get() < end) {
        (void)mqtt_input(&client);
        (void)mqtt_live(&client);
        k_sleep(K_MSEC(20));
    }

    (void)mqtt_disconnect(&client, NULL);

    return 0;
}

static uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t pos = 0; pos < len; pos++) {
        crc ^= buf[pos];

        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static int modbus_read(const uint8_t *request,
                       size_t request_len,
                       uint8_t *response,
                       size_t response_len)
{
    int last_err = -EIO;

    for (int attempt = 1; attempt <= MODBUS_RETRIES; attempt++) {

        memset(response, 0, response_len);

        /* Flush any stale UART bytes */
        uint8_t dummy;
        while (uart_poll_in(mb_uart, &dummy) == 0) {
        }

        uart_send_bytes(request, request_len);

        int n = uart_read_bytes(response, response_len, 500);

        if (n < MODBUS_RESPONSE_LEN) {
            last_err = -EIO;

            if (attempt == MODBUS_RETRIES) {
                error_log_add(last_err,
                              "Modbus timeout after %d retries (%d bytes)",
                              MODBUS_RETRIES,
                              n);
            }
            continue;
        }

        if (response[0] != 0x01 ||
            response[1] != 0x03 ||
            response[2] != 0x0E) {

            last_err = -EBADMSG;

            if (attempt == MODBUS_RETRIES) {
                error_log_add(last_err,
                              "Invalid Modbus header");
            }
            continue;
        }

        uint16_t crc_received =
            ((uint16_t)response[18] << 8) | response[17];

        uint16_t crc_calculated =
            modbus_crc16(response, 17);

        if (crc_received != crc_calculated) {
            last_err = -EBADMSG;

            if (attempt == MODBUS_RETRIES) {
                error_log_add(last_err,
                              "CRC mismatch: rx=%04X calc=%04X",
                              crc_received,
                              crc_calculated);
            }
            continue;
        }

        return 0;
    }

    return last_err;
}

static int getMeasurement(measurement_t *m)
{
    static const uint8_t request[8] =
        {0x01,0x03,0x00,0x1D,0x00,0x07,0x94,0x0E};

    uint8_t response[32];

    int err = modbus_read(request,
                          sizeof(request),
                          response,
                          sizeof(response));
    if (err) {
        return err;
    }

    m->flow =
        ((uint16_t)response[3] << 8) | response[4];

    m->battery =
        ((uint16_t)response[5] << 8) | response[6];

    m->solar =
        ((uint16_t)response[7] << 8) | response[8];

    m->peak =
        ((uint32_t)response[9] << 24) |
        ((uint32_t)response[10] << 16) |
        ((uint32_t)response[11] << 8) |
        response[12];

    m->offpeak =
        ((uint32_t)response[13] << 24) |
        ((uint32_t)response[14] << 16) |
        ((uint32_t)response[15] << 8) |
        response[16];

    if (date_time_is_valid() &&
        date_time_now(&m->timestamp) == 0) {
        m->utc_valid = true;
    } else {
        m->utc_valid = false;
        m->timestamp = k_uptime_get();
    }

    return 0;
}

int update_signal(measurement_t *m)
{
    short rsrp_idx = 0;
    m->rsrp_dbm = -127;
    int ret = modem_info_short_get(MODEM_INFO_RSRP, &rsrp_idx);
    if (ret < 0) {
        printk("Failed to get RSRP (err=%d)\n", ret);
        return ret;
    }
    m->rsrp_dbm = RSRP_IDX_TO_DBM(rsrp_idx);
    printk("RSRP = %d dBm (idx=%d)\n",
           m->rsrp_dbm,
           rsrp_idx);
    return 0;
}

static int publish_log(const char *name,
                         build_payload_fn_t build,
                         count_queue_fn_t count,
                         clear_queue_fn_t clear,
                         bool log_errors)
{
    if (count() == 0) {
        return 0;
    }

    char payload[1024];

    int err = build(payload, sizeof(payload));
    if (err) {
        printk("%s payload build failed: %d\n", name, err);

        if (log_errors) {
            error_log_add(err,
                          "%s payload build failed: %d",
                          name, err);
        }

        return err;
    }

    err = mqtt_publish_method(payload);
    if (err) {
        printk("%s publish failed: %d\n", name, err);

        if (log_errors) {
            error_log_add(err,
                          "%s publish failed: %d",
                          name, err);
        }

        return err;
    }

    clear();

    return 0;
}

static int publish_measurements(void)
{
    return publish_log(
        "Measurement",
        build_measurement_payload,
        measurement_log_count,
        measurement_log_clear,
        true);
}

static int publish_errors(void)
{
    return publish_log(
        "Error",
        build_error_payload,
        error_log_count,
        error_log_clear,
        false);
}

static bool first_boot = true;

int main(void)
{    
    printk("Initialising modem...\n");

    int err = nrf_modem_lib_init();
    printk("Modem init = %d\n", err);
    if (err) {
        printk("Modem init failed\n");
        return 0;
    }
    
    err = modem_info_init();
    if (err) {
        printk("modem_info_init failed: %d\n", err);
    }

    while (1)
    {
        measurement_t m = {0};
        int rc = 0;
        if (!first_boot) {
            rc = getMeasurement(&m);
            if (rc != 0) {
                error_log_add(rc, "Measurement failed: %d", rc);
            }
        }
        
        //run_at("AT");
        err = lte_lc_normal(); //int err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL); //run_at("AT+CFUN=1");
        if (err) {
            printk("Failed to set normal mode: %d\n", err);
            error_log_add(err, "Failed to set normal mode: %d", err);
        }
        printk("Before LTE connect\n");
        err = lte_lc_connect();
        printk("After LTE connect, err=%d\n", err);
        if (err) {
            printk("Failed to connect network: %d\n", err);
            error_log_add(err, "Failed to connect network: %d", err);
            if (!first_boot && rc == 0) {
                measurement_log_add(&m);
            }
            err = lte_lc_power_off();
            if (err) {
                error_log_add(err,
                    "Failed to power off modem: %d",
                    err);
            }
            sleep_until_next_wakeup();
            continue;
        }

        if (!date_time_is_valid()) {
            err = date_time_update_async(NULL);
            if (err) {
                printk("Failed to request network time: %d\n", err);
                error_log_add(err, "Failed to request network time: %d", err);
            }

            for (int i = 0; i < 60 && !date_time_is_valid(); i++) {
                k_sleep(K_SECONDS(1));
            }
        }

        if (first_boot) {
            printk("Cold boot complete\n");
            error_log_add(EVENT_COLD_BOOT, "Cold boot");
        } else if (rc == 0) {
            int sig_rc = update_signal(&m);
            if (sig_rc) {
                printk("Failed to read RSRP: %d\n", sig_rc);
                m.rsrp_dbm = -127;
            }
            measurement_log_add(&m);
        }
        first_boot = false;

        //k_sleep(K_SECONDS(20));

        //run_at("AT+CEREG?");
        //run_at("AT+COPS?");
        //run_at("AT+CGPADDR");

        err = publish_measurements();
        if (err) {
            printk("Measurement upload failed: %d\n", err);
        }

        err = publish_errors();
        if (err) {
            printk("Error upload failed: %d\n", err);
        }

        err = lte_lc_power_off(); //err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_POWER_OFF);
        if (err) {
            printk("Failed to power off modem: %d\n", err);
            error_log_add(err, "Failed to power off modem: %d", err);
        }

        /*err = nrf_modem_lib_shutdown();
        if (err) {
            printk("Modem shutdown failed\n");
            return 0;
        }*/

        sleep_until_next_wakeup(); 
    }
}