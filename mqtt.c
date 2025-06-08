#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "MQTTClient.h"

#define ADDRESS     "tcp://172.20.10.3:1883"
#define CLIENTID    "ExampleClient"
#define TOPIC_PUB   "sensor"
#define TOPIC_SUB   "led"
#define QOS         1
#define TIMEOUT     10000L
#define BH1750_DEV  "/dev/bh1750"
#define LED_DEV     "/dev/led_gpio60"

float read_light_sensor() {
    int fd = open(BH1750_DEV, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open /dev/bh1750");
        return -1;
    }

    char buf[16] = {0};
    int len = read(fd, buf, sizeof(buf) - 1);
    close(fd);

    if (len <= 0) {
        perror("Failed to read from /dev/bh1750");
        return -1;
    }

    return atof(buf);
}

void write_led_state(const char *state) {
    int fd = open(LED_DEV, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open /dev/led1");
        return;
    }

    ssize_t ret = write(fd, state, 1);
    if (ret < 0) {
        perror("Failed to write to /dev/led1");
    }

    close(fd);
}

// Callback khi nhận message trên topic subscribed
int messageArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    char *payload = (char *)message->payload;

    printf("Received message on topic %s: %.*s\n", topicName, message->payloadlen, payload);

    // Kiểm tra topic có đúng không
    if (strncmp(topicName, TOPIC_SUB, strlen(TOPIC_SUB)) == 0) {
        if (message->payloadlen == 1 && (payload[0] == '1' || payload[0] == '0')) {
            write_led_state(payload);
        } else {
            printf("Invalid payload for LED control: %.*s\n", message->payloadlen, payload);
        }
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

int main() {
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;

    rc = MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    if (rc != MQTTCLIENT_SUCCESS) {
        printf("Failed to create client, code: %d\n", rc);
        exit(EXIT_FAILURE);
    }

    // Đăng ký callback nhận tin nhắn
    MQTTClient_setCallbacks(client, NULL, NULL, messageArrived, NULL);

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    rc = MQTTClient_connect(client, &conn_opts);
    if (rc != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, code: %d\n", rc);
        MQTTClient_destroy(&client);
        exit(EXIT_FAILURE);
    }

    printf("Connected to MQTT broker\n");

    // Subscribe topic nhận lệnh bật/tắt led
    rc = MQTTClient_subscribe(client, TOPIC_SUB, QOS);
    if (rc != MQTTCLIENT_SUCCESS) {
        printf("Failed to subscribe, code: %d\n", rc);
        MQTTClient_disconnect(client, 1000);
        MQTTClient_destroy(&client);
        exit(EXIT_FAILURE);
    }

    while (1) {
        float lux = read_light_sensor();
        if (lux < 0) {
            sleep(5);
            continue;
        }

        char payload[32];
        snprintf(payload, sizeof(payload), "%.2f", lux);

        pubmsg.payload = payload;
        pubmsg.payloadlen = strlen(payload);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;

        rc = MQTTClient_publishMessage(client, TOPIC_PUB, &pubmsg, &token);
        if (rc != MQTTCLIENT_SUCCESS) {
            printf("Failed to publish message, code: %d\n", rc);
        } else {
            MQTTClient_waitForCompletion(client, token, TIMEOUT);
            printf("Sent lux: %s\n", payload);
        }

        sleep(10);
    }

    MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);

    return 0;
}

