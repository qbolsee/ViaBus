#ifndef VIABUS_H_
#define VIABUS_H_

#include <Arduino.h>
#include <stdlib.h>
#include "utils/cobs.h"
#include "utils/crc8.h"


#define VIABUS_DATA_LEN 253
#define VIABUS_ENCODED_DATA_LEN 255


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
    uint8_t encodedData[VIABUS_ENCODED_DATA_LEN];
} ViaBusMessage_t;


bool encodeViaBusMessage(ViaBusMessage_t* msg, const char* data, uint8_t dataLength);

uint8_t computeHeaderCRC(ViaBusMessage_t* msg);

/*
void decodeMessage(ViaBusMessage_t* msg, const char* dataRaw, uint8_t dataRawLen) {
    cobsEncode(dataRaw, dataRawLen, msg->encodedData);
}
*/

class ViaBus {
public:
    typedef enum {
        MSG_TYPE_REGULAR   = 1,
        MSG_TYPE_HEARTBEAT = 2
    } MessageType;

    typedef enum {
        REPLY_ERROR     = 0,
        REPLY_AVAILABLE = 1,
        REPLY_TIMEOUT   = 2
    } ReplyStatus;

    static const uint8_t ADDR_CONTROLLER = 255;
    static const uint8_t ADDR_UNDEFINED = 254;
    static const uint8_t ADDR_BASE = 1;

    static const int BAUDRATE_DEFAULT     = 500;
    static const int HEARTBEAT_MS         = 800;
    static const int HEARTBEAT_TIMEOUT_MS = 2000;
    //const int 

    ViaBus(HardwareSerial* serialBroadcast);

    void onMessage(void (*callback)(uint8_t address, const char* msg));
    void begin(long int baudrate=BAUDRATE_DEFAULT);
    void loop();
    int send(uint8_t address, const char* msg);
    //void onMessageForward(void (*callback)());
    //void onAddressChange(void (*callback)());

private:
    //sendMessage(uint8_t address, uint8_t messageType)

    uint8_t address = ADDR_UNDEFINED;
    long int baudrate = 0;

    //void serializeMessage(ViaBusMessage_t* msg, char* byteStream);

    int lastLoopTime     = -1;
    int lastMessageTimer =  0;

    void (*onMessageCallback)(uint8_t address, const char* msg) = nullptr;

    // state machine
    char header[VIABUS_PAYLOAD_LEN];
    char payload[VIABUS_PAYLOAD_LEN];
    char crc = 0;
    //ViaBusMessage_t msgIn;
    //ViaBusMessage_t msgOut;
    HardwareSerial* serialBroadcast = nullptr;
};

/*
class ViaBusController: public ViaBus {
public:
    ViaBusController(Stream * serial_broadcast);

    int nPeripherals();
private:
    Stream* serial_broadcast = nullptr;
}


class ViaBusPeripheral: public ViaBus {
public:
    ViaBusPeripheral(Stream * serialBroadcast, Stream * serialChain);
private:
    Stream* serialBroadcast = nullptr;
    Stream* serial_chain = nullptr;

    ViaBusMessage_t msgInBroadcast;
}
*/

#endif // VIABUS_H_
