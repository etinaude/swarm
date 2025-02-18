#include <WiFi.h>
#include <esp_now.h>

#define MAX_PEERS 2

typedef struct struct_message
{
    char msg[32];
    uint8_t senderMAC[6];
} struct_message;

uint8_t peerAddresses[MAX_PEERS][6] = {
    {0xCC, 0x8D, 0xA2, 0x94, 0x74, 0x48},
    {0xCC, 0x8D, 0xA2, 0x94, 0xA0, 0x00}};

struct_message data;

void printMac(const uint8_t *macAddr)
{
    for (int i = 0; i < 6; i++)
    {
        Serial.printf("%02X", macAddr[i]);
        if (i < 5)
            Serial.print(":");
    }
}

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

bool matchingMac(uint8_t *mac)
{
    for (int i = 0; i < 6; i++)
    {
        if (mac[i] != data.senderMAC[i])
            return false;
    }
    return true;
}

void setupEspNow()
{
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());
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

void testConnections()
{
    esp_now_peer_num_t peer_num;
    esp_now_get_peer_num(&peer_num);
    Serial.print("Number of peers: ");
    Serial.println(peer_num.total_num);

    for (int i = 0; i < peer_num.total_num; i++)
    {
        esp_now_peer_info_t peer;
        esp_now_get_peer(peerAddresses[i], &peer);
        Serial.print("Peer ");
        Serial.print(i);
        Serial.print(" MAC: ");
        printMac(peer.peer_addr);
    }
}