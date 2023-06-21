#include "ViaBus.h"


bool encodeMessageData(ViaBusMessage_t* msg, const char* data, uint8_t dataLength) {
    if (dataLength > VIABUS_DATA_MAXLEN) {
        return false;
    }
    msg->encodedDataLength = cobsEncode(data, dataLength, msg->encodedData);
    return true;
}


uint8_t computeHeaderCRC(ViaBusMessage_t* msg) {
    uint8_t crc = 0;
    crc = CRC8_NEXT(crc, msg->dst);
    crc = CRC8_NEXT(crc, msg->src);
    crc = CRC8_NEXT(crc, msg->hops);
    crc = CRC8_NEXT(crc, msg->msgType);
    crc = CRC8_NEXT(crc, msg->msgVersion);
    crc = CRC8_NEXT(crc, msg->encodedDataLength);
    return crc;
}


void ViaBus::getAddress() {
    return this->address;
}


void ViaBus::onMessage(void (*callback)(uint8_t srcAddress, const char* data)) {
    this->onMessageCallback = callback;
}


void ViaBus::onForward(void (*callback)(uint8_t srcAddress, uint8_t dstAddress, const uint8_t* data, uint8_t dataLength)) {
    this->onForwardCallback = callback;
}


void ViaBus::transmit(ViaBusMessage_t* msg, Stream* stream) {
    stream->write(0);
    stream->write(msgOut.dst);
    stream->write(msgOut.src);
    stream->write(msgOut.hops);
    stream->write(msgOut.msgType);
    stream->write(msgOut.msgVersion);
    stream->write(msgOut.encodedDataLength);
    stream->write(msgOut.hdrCRC);
    stream->write(msgOut.encodedData);
}


bool ViaBusController::send(uint8_t dstAddress, const char* data, uint8_t dataLength) {
    if (dataLength == 0) {
        dataLength = strlen(data);
    }
    msgOut.dst = dstAddress;
    msgOut.src = ADDR_CONTROLLER;
    msgOut.hops = 1;
    msgOut.msgType = MSG_TYPE_DATA;
    msgOut.msgVersion = VIABUS_VERSION;
    bool success = encodeMessageData(&msgOut, data, dataLength);
    if (!success) {
        return false;
    }
    msgOut.hdrCRC = computeHeaderCRC(&msgOut);
    transmit(&msgOut, streamBroadcast);
    return true;
}


bool ViaBusPeripheral::send(uint8_t dstAddress, const char* data, uint8_t dataLength) {
    if (dataLength == 0) {
        dataLength = strlen(data);
    }
    msgOut.dst = dstAddress;
    msgOut.src = this->address;
    msgOut.hops = 1;
    msgOut.msgType = MSG_TYPE_DATA;
    msgOut.msgVersion = VIABUS_VERSION;
    bool success = encodeMessageData(&msgOut, data, dataLength);
    if (!success) {
        return false;
    }
    msgOut.hdrCRC = computeHeaderCRC(&msgOut);
    transmit(&msgOut, streamChain);
    return true;
}


bool ViaBusPeripheral::sendHeartbeat() {
    msgOut.dst = ADDR_CONTROLLER;
    msgOut.src = ADDR_BASE;
    msgOut.hops = 1;
    msgOut.msgType = MSG_TYPE_HEARTBEAT;
    msgOut.msgVersion = VIABUS_VERSION;
    msgOut.encodedDataLength = 1;
    msgOut.encodedData[0] = VIABUS_ENCODED_DATA_EMPTY;
    msgOut.hdrCRC = computeHeaderCRC(&msgOut);
    transmit(&msgOut, streamChain);
    return true;
}


bool ViaBusPeripheral::sendReaddress() {
    msgOut.dst = ADDR_CONTROLLER;
    msgOut.src = ADDR_BASE;
    msgOut.hops = 1;
    msgOut.msgType = MSG_TYPE_READDRESS;
    msgOut.msgVersion = VIABUS_VERSION;
    msgOut.encodedDataLength = 1;
    msgOut.encodedData[0] = VIABUS_ENCODED_DATA_EMPTY;
    msgOut.hdrCRC = computeHeaderCRC(&msgOut);
    transmit(&msgOut, streamChain);
    return true;
}


ViaBusController::ViaBusController(Stream* streamBroadcast) {
    this->streamBroadcast = streamBroadcast;
    this->address = ADDR_CONTROLLER;
}


void ViaBusController::begin(long int baudrate) {
    //serial_broadcast.begin(baudrate);
    this->baudrate = baudrate;
}


ViaBusPeripheral::ViaBusPeripheral(Stream* streamBroadcast, Stream* streamChain) {
    this->streamBroadcast = streamBroadcast;
    this->streamChain = streamChain;
}


void ViaBusPeripheral::begin(long int baudrate) {
    //serial_broadcast.begin(baudrate);
    //serial_chain.begin(baudrate);
    this->baudrate = baudrate;
}


void ViaBusPeripheral::loop(long int baudrate) {
    unsigned long t = millis();
    unsigned long dt = 0;
    if (lastLoopTime >= 0) {
        dt = t - lastLoopTime;
    }
    lastLoopTime = t;
    lastMessageTimer += dt;

    if (lastMessageTimer > HEARTBEAT_TIMEOUT_MS) {
        // assume we are now the first node
        this->address == ADDR_BASE;
        sendReaddress();
        lastMessageTimer = 0;
    } else if (lastMessageTimer > HEARTBEAT_MS && this->address == ADDR_BASE) {
        // we are the first node, send heartbeat
        sendHeartbeat();
        lastMessageTimer = 0;
    }

    int k = 0;
    
    while (streamBroadcast.available() && k < VIABUS_MAXLEN) {
        char c = streamBroadcast.read();
        k++;
        if (c == 0) {
            // reset
            msgInBroadcast.encodedDataLength = 0;
        }
        msgInBroadcast[msgInBroadcast.]
        buffer_broadcast[buffer_broadcast_i++] = c;
        if (c == '\0') {
            // parse message
            if (on_msg_callback != nullptr) {

            }

            buffer_broadcast_i = 0;
        }

        if (buffer_broadcast_i >= MSG_BUFFER_SIZE) {
            // error, start from scratch
            buffer_broadcast_i = 0;
        }
    }

    
    while (serial_chain.available()) {
        char c = serial_chain.read()
        buffer_chain[buffer_chain_i++] = c;
        if (c == '\0') {
            // parse message
            buffer_chain_i = 0;
        }

        if (buffer_chain_i >= MSG_BUFFER_SIZE) {
            // error, start from scratch
            buffer_chain_i = 0;
        }
    }
}
