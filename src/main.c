/*
todo
->remove print, or actually log, so we can read errors later, obtain log file, like when timesteps are missing, can read historical logs
->obtain meter resets
->full sleep shutdown, use rtc to wake up/turn on again for daily
->de power the rs232 transceiver, like power with digital output maybe, and set to no voltage to turn off
->psm mode, leave modem on and maybe connected, investigate for 15 min
->pulse input to wake from deep sleep, change to 15minute rate, change back to daily when stop watering
->ring buffer, flash memory, non volatilve or persistent memory
->compress/zip the payload maybe
->receive config updates over the air COTA
->use async lte_lc_connect and wait, as current is blocking if antenna not connected lte_lc_connect_async(lte_handler); with semaphore
 */

#include <stdio.h>
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
#define INTERVAL_MS 60000
#define SLEEP_INTERVAL_S (60 * 60)

static const struct device *mb_uart = DEVICE_DT_GET(MB_UART_NODE);

static struct mqtt_client client;
static struct sockaddr_storage broker;

static uint8_t rx_buffer[1024];
static uint8_t tx_buffer[1024];

static void sleep_until_next_wakeup(void)
{
    int64_t now_ms;
    int err;

    err = date_time_now(&now_ms);
    if (err) {
        printk("Failed to get time\n");

        /* Fallback */
        k_sleep(K_SECONDS(SLEEP_INTERVAL_S));
        return;
    }

    int64_t now_sec = now_ms / 1000;

    int64_t next_wakeup = ((now_sec / SLEEP_INTERVAL_S) + 1) * SLEEP_INTERVAL_S;

    int64_t sleep_sec = next_wakeup - now_sec;

    printk("Sleeping %lld seconds until next wakeup\n",
           sleep_sec);

    k_sleep(K_SECONDS(sleep_sec));
}

static int uart_send_bytes(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(mb_uart, data[i]);
    }
    return 0;
}

static int uart_read_bytes(uint8_t *buf, size_t max_len, int timeout_ms)
{
    int count = 0;
    int64_t start = k_uptime_get();

    while ((k_uptime_get() - start) < timeout_ms && count < max_len) {
        uint8_t c;

        if (uart_poll_in(mb_uart, &c) == 0) {
            buf[count++] = c;
        } else {
            k_sleep(K_MSEC(1));
        }
    }

    return count;
}

//printk("[%" PRIu32 "] Hello World! %s\n", k_uptime_seconds(), CONFIG_BOARD_TARGET);

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

static void mqtt_evt_handler(struct mqtt_client *const client,
                             const struct mqtt_evt *evt)
{
    printk("MQTT event: %d\n", evt->type);

    switch (evt->type) {

    case MQTT_EVT_CONNACK:
        printk("MQTT connected, result=%d\n",
           evt->result);
        break;

    case MQTT_EVT_DISCONNECT:
        printk("MQTT disconnected\n");
        break;

    case MQTT_EVT_PUBLISH:
        printk("MQTT publish received\n");
        break;

    default:
        break;
    }
}

int mqtt_publish_method(const char *payload)
{
    int err;

    mqtt_client_init(&client);

    client.broker = &broker;
    client.evt_cb = mqtt_evt_handler;
    client.client_id.utf8 = (uint8_t *)"nrf9151";
    client.client_id.size = strlen("nrf9151");

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

    err = mqtt_connect(&client);
    if (err) {
        printk("mqtt_connect failed: %d\n", err);
        return err;
    }

    printk("Waiting for CONNACK...\n");

    for (int i = 0; i < 50; i++) {
        mqtt_input(&client);
        mqtt_live(&client);
        k_sleep(K_MSEC(100));
    }

    printk("MQTT connect sent\n");

    k_sleep(K_SECONDS(2));
    
    // Fixed the missing semicolon here
    static const char topic[] = "test/topic"; 

    struct mqtt_publish_param param = {
        .message.topic.topic = {
            .utf8 = (uint8_t *)topic,
            .size = strlen(topic)
        },
        .message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE,
        // Map the payload argument into the MQTT message structure
        .message.payload.data = (uint8_t *)payload, 
        .message.payload.len = strlen(payload),
        .message_id = 1,
        .dup_flag = 0,
        .retain_flag = 0
    };

    err = mqtt_publish(&client, &param);

    printk("mqtt_publish = %d\n", err);

    k_sleep(K_SECONDS(5));

    mqtt_disconnect(&client, NULL);

    return err;
}

int getPayload(char *payload, size_t payload_len)
{
    //const uint8_t request[8] = { 0x01, 0x03, 0x00, 0x20, 0x00, 0x04, 0x45, 0xc3 };
    const uint8_t request[8] = { 0x1, 0x3, 0x0, 0x1d, 0x0, 0x7, 0x94, 0xe };
    uint8_t response[32];

    printk("Sending Modbus request...\n");
    uart_send_bytes(request, sizeof(request));

    memset(response, 0, sizeof(response));
    int n = uart_read_bytes(response, sizeof(response), 500);

    printk("Got %d bytes\n", n);
    for (int i = 0; i < n; i++) {
        printk("%02X ", response[i]);
    }
    printk("\n");

    if (n < 17) {
        return -1;
    }

    uint16_t flow = ((uint16_t)response[3] << 8) | response[4];
    uint16_t battery = ((uint16_t)response[5] << 8) | response[6];
    uint16_t solar = ((uint16_t)response[7] << 8) | response[8];

    printk("%u\n", flow);
    printk("%u\n", battery);
    printk("%u\n", solar);

    uint32_t peak =
        ((uint32_t)response[9] << 24) |
        ((uint32_t)response[10] << 16) |
        ((uint32_t)response[11] << 8) |
        response[12];

    uint32_t offpeak =
        ((uint32_t)response[13] << 24) |
        ((uint32_t)response[14] << 16) |
        ((uint32_t)response[15] << 8) |
        response[16];

    short rsrp = -127;   // sensible default
    int err = modem_info_short_get(MODEM_INFO_RSRP, &rsrp);
    if (err) {
        printk("Failed to get RSRP: %d\n", err);
    } else {
        printk("RSRP = %d dBm\n", rsrp);
    }

    int64_t now_ms;
    if (date_time_is_valid() && date_time_now(&now_ms) == 0) {
        // use UTC timestamp
    } else {
        now_ms = k_uptime_get();
        // use uptime as fallback
    }

    int len = snprintf(payload, payload_len,
        "{"
        "\"d\":\"nrf1\","
        "\"p\":%u,"
        "\"o\":%u,"
        "\"f\":%u,"
        "\"b\":%u,"
        "\"s\":%u,"
        "\"q\":%d,"
        "\"t\":%lld"
        "}",
        peak,
        offpeak,
        flow,
        battery,
        solar,
        rsrp,
        (long long)now_ms);

    printk("%s\n", payload);

    if (len < 0 || len >= payload_len) {
        printk("Payload truncated\n");
        return -ENOMEM;
    }

    if (len < 0 || len >= (int)payload_len) {
        return -2;
    }

    return 0;
}

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
        /*char payload2[256];
        getPayload(payload2, sizeof(payload2));
        sleep_until_next_wakeup();
        continue;*/

        //run_at("AT");
        err = lte_lc_normal(); //int err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL); //run_at("AT+CFUN=1");
        if (err) {
            printk("Failed to set normal mode: %d\n", err);
        }
        printk("Before LTE connect\n");
        err = lte_lc_connect();
        printk("After LTE connect, err=%d\n", err);
        if (err) {
            printk("Failed to connect network: %d\n", err);
            lte_lc_power_off();
            sleep_until_next_wakeup();
            continue;
        }
      
        err = date_time_update_async(NULL);
        if (err) {
            printk("Failed to request network time: %d\n", err);
        }

        for (int i = 0; i < 60 && !date_time_is_valid(); i++) {
            k_sleep(K_SECONDS(1));
        }

        int64_t now_ms;
        if (date_time_now(&now_ms) == 0) {
            printk("Current UTC time: %lld ms\n", now_ms);
        } else {
            printk("No valid time available\n");
        }

        //k_sleep(K_SECONDS(20));

        //run_at("AT+CEREG?");
        //run_at("AT+COPS?");
        //run_at("AT+CGPADDR");

        char payload[256];
        if (getPayload(payload, sizeof(payload)) == 0) {
            err = mqtt_publish_method(payload);
            if (err) {
                printk("Publish failed: %d\n", err);
            }
        }        

        err = lte_lc_power_off(); //err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_POWER_OFF);
        if (err) {
            printk("Failed to power off modem: %d\n", err);
        }

        /*err = nrf_modem_lib_shutdown();
        if (err) {
            printk("Modem shutdown failed\n");
            return 0;
        }*/

        sleep_until_next_wakeup(); 
    }
}