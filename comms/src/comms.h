#include <WiFi.h>
#include <esp_now.h>
#include "comms_helpers.h"

void onSent(const uint8_t *macAddr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
        return;
    Serial.print("Failed to send to: ");
    printMac(macAddr);
    Serial.println();
}

void onReceive(const uint8_t *macAddr, const uint8_t *incomingData, int len)
{
    struct_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    for (int i = 0; i < 6; i++)
    {
        Serial.printf("%02X", receivedData.senderMAC[i]);
        if (i < 5)
            Serial.print(":");
    }
    Serial.print(" -> ");
    Serial.println(receivedData.msg);
}

void findMac()
{
    Serial.println(WiFi.macAddress());
    delay(500);
}

void setupEspNow()
{
    WiFi.mode(WIFI_STA);
    WiFi.macAddress(data.senderMAC);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    for (int i = 0; i < MAX_PEERS; i++)
    {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerAddresses[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            if (matchingMac(peerAddresses[i]))
                continue;
            Serial.println("Failed to add peer");
        }
    }
}

void broadcast(String msg)
{
    char msgChar[32];
    msg.toCharArray(msgChar, 32);

    Serial.print("Broadcasting: ");
    Serial.println(msg);

    strcpy(data.msg, msgChar);

    for (int i = 0; i < MAX_PEERS; i++)
    {
        if (matchingMac(peerAddresses[i]))
            continue;

        esp_err_t result = esp_now_send(peerAddresses[i], (uint8_t *)&data, sizeof(data));
    }
}