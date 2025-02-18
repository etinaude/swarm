#include <WiFi.h>
#define MAX_PEERS 5

typedef struct struct_message
{
    char msg[32];
    uint8_t senderMAC[6];
} struct_message;

uint8_t peerAddresses[MAX_PEERS][6] = {
    {0xCC, 0x8D, 0xA2, 0x94, 0x74, 0x48},
    {0xCC, 0x8D, 0xA2, 0x94, 0xA0, 0x00},
    {0xCC, 0x8D, 0xA2, 0x8B, 0xAF, 0xA0},
    {0xCC, 0x8D, 0xA2, 0x94, 0x71, 0xAA},
    {0xCC, 0x8D, 0xA2, 0x94, 0x6A, 0x14}};

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

bool matchingMac(uint8_t *mac)
{
    for (int i = 0; i < 6; i++)
    {
        if (mac[i] != data.senderMAC[i])
            return false;
    }
    return true;
}