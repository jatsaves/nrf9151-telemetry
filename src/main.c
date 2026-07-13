/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2016-2026 Makerdiary <https://makerdiary.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>

#define MB_UART_NODE DT_NODELABEL(uart1)
#define INTERVAL_MS 60000

static const struct device *mb_uart = DEVICE_DT_GET(MB_UART_NODE);

static struct mqtt_client client;
static struct sockaddr_storage broker;

static uint8_t rx_buffer[1024];
static uint8_t tx_buffer[1024];

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

int mqtt_publish_test(void)
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

    printk("Waiting for CONNACK...\n");

    for (int i = 0; i < 50; i++) {
        mqtt_input(&client);
        mqtt_live(&client);
        k_sleep(K_MSEC(100));
    }

    if (err) {
        printk("mqtt_connect failed: %d\n", err);
        return err;
    }

    printk("MQTT connect sent\n");

    k_sleep(K_SECONDS(2));

    struct mqtt_publish_param param;

    static uint8_t payload[] =
        "{\"hello\":\"nrf9151\"}";

    param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
    param.message.topic.topic.utf8 =
        (uint8_t *)"pool/test";

    param.message.topic.topic.size =
        strlen("pool/test");

    param.message.payload.data = payload;
    param.message.payload.len = sizeof(payload) - 1;

    param.message_id = 1;
    param.dup_flag = 0;
    param.retain_flag = 0;

    err = mqtt_publish(&client, &param);

    printk("mqtt_publish = %d\n", err);

    k_sleep(K_SECONDS(5));

    mqtt_disconnect(&client, NULL);

    return err;
}

int main(void)
{   
    printk("Initialising modem...\n");

    int err = nrf_modem_lib_init();

    printk("Modem init = %d\n", err);

    if (err) {
        printk("Modem init failed\n");

        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    if (err) {
        return 0;
    }

    run_at("AT");
    run_at("AT+CFUN=1");

    k_sleep(K_SECONDS(20));

    run_at("AT+CEREG?");
    run_at("AT+COPS?");
    run_at("AT+CGPADDR");

    mqtt_publish_test();

    while (1) {
        k_sleep(K_SECONDS(60));
    }

    const uint8_t request[8] = { 0x01, 0x03, 0x00, 0x20, 0x00, 0x04, 0x45, 0xc3 };
    uint8_t response[32];
    int64_t next_ms = k_uptime_get() + INTERVAL_MS;

    while (1) {
        printk("Sending Modbus request...\n");
        uart_send_bytes(request, sizeof(request));

        memset(response, 0, sizeof(response));
        int n = uart_read_bytes(response, sizeof(response), 500);

        printk("Got %d bytes\n", n);
        for (int i = 0; i < n; i++) {
            printk("%02X ", response[i]);
        }
        printk("\n");

        uint32_t peak =
            ((uint32_t)response[3] << 24) |
            ((uint32_t)response[4] << 16) |
            ((uint32_t)response[5] << 8)  |
            response[6];

        uint32_t offpeak =
            ((uint32_t)response[7] << 24) |
            ((uint32_t)response[8] << 16) |
            ((uint32_t)response[9] << 8)  |
            response[10];

        printk("Peak totaliser = %u\n", peak);
        printk("Off-peak totaliser = %u\n", offpeak);

        next_ms += 60000;
        k_sleep(K_TIMEOUT_ABS_MS(next_ms));
    }
}