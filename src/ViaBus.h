#ifndef VIABUS_H_
#define VIABUS_H_

#include <Arduino.h>
#include <stdlib.h>
#include "utils/cobs.h"
#include "utils/crc8.h"


#define VIABUS_DATA_MAXLEN 253
#define VIABUS_ENCODED_DATA_MAXLEN 255
#define VIABUS_ENCODED_DATA_EMPTY 0x01
#define VIABUS_HEADER_LEN 8

#define VIABUS_MAXLEN (VIABUS_HEADER_LEN + VIABUS_ENCODED_DATA_MAXLEN)

#define VIABUS_VERSION 1


// message structure:
// 0 | DST | SRC | HOPS | MSG_TYPE | VERSION | ENC_DATA_LEN | HDR_CRC | ENC_DATA


typedef struct ViaBusMessage {
    uint8_t dst;
    uint8_t src;
    uint8_t hops;
    uint8_t msgType;
    uint8_t msgVersion;
    uint8_t encodedDataLength;
    uint8_t hdrCRC;
    uint8_t encodedData[VIABUS_ENCODED_DATA_MAXLEN];
} ViaBusMessage_t;


bool encodeViaBusData(ViaBusMessage_t* msg, const char* data, uint8_t dataLength);

uint8_t computeHeaderCRC(ViaBusMessage_t* msg);

/*
void decodeMessage(ViaBusMessage_t* msg, const char* dataRaw, uint8_t dataRawLen) {
    cobsEncode(dataRaw, dataRawLen, msg->encodedData);
}
*/

class ViaBus {
public:
    static const uint8_t MSG_TYPE_DATA      = 1;
    static const uint8_t MSG_TYPE_READDRESS = 2;
    static const uint8_t MSG_TYPE_HEARTBEAT = 3;

    static const uint8_t ADDR_CONTROLLER = 255;
    static const uint8_t ADDR_UNDEFINED  = 254;
    static const uint8_t ADDR_BASE       = 1;

    static const int BAUDRATE_DEFAULT     = 500;
    static const int HEARTBEAT_MS         = 800;
    static const int HEARTBEAT_TIMEOUT_MS = 2000;

    //ViaBus();
    void onMessage(void (*callback)(uint8_t srcAddress, const char* data, uint8_t dataLength));
    void onForward(void (*callback)(uint8_t srcAddress, uint8_t dstAddress, const char* data, uint8_t dataLength));
    void begin(long int baudrate=BAUDRATE_DEFAULT);
    void loop();
    bool send(uint8_t dstAddress, const char* data, uint8_t dataLength=0);

    uint8_t getAddress();

private:
    void transmit(ViaBusMessage_t* msg, Stream* stream);

    uint8_t address = ADDR_UNDEFINED;
    long int baudrate = 0;
    //void serializeMessage(ViaBusMessage_t* msg, char* byteStream);

    int lastLoopTime     = -1;
    int lastMessageTimer =  0;

    void (*onMessageCallback)(uint8_t srcAddress, const uint8_t* msg, uint8_t msgLength) = nullptr;
    void (*onForwardCallback)(uint8_t srcAddress, uint8_t dstAddress, const uint8_t* msg, uint8_t msgLength) = nullptr;
};


class ViaBusController: public ViaBus {
public:
    ViaBusController(Stream* streamBroadcast);
    uint8_t getNumberPeripherals();

private:
    Stream* streamBroadcast = nullptr;

    ViaBusMessage_t msgIn;
    ViaBusMessage_t msgOut;
}


class ViaBusPeripheral: public ViaBus {
public:
    ViaBusPeripheral(Stream * serialBroadcast, Stream * serialChain);

private:
    void sendHeartbeat();
    void sendReaddress();

    Stream* streamBroadcast = nullptr;
    Stream* streamChain = nullptr;

    ViaBusMessage_t msgInBroadcast;
    ViaBusMessage_t msgInChain;
    ViaBusMessage_t msgOut;
}


#endif // VIABUS_H_
