#include "ViaBus.h"


bool encodeViaBusMessage(ViaBusMessage_t* msg, const char* data, uint8_t dataLength) {
    if (dataLength > VIABUS_DATA_LEN) {
        return false;
    }
    msg->encodedDataLength = cobsEncode(data, dataLength, msg->encodedData);
    return true;
}


void computeHeaderCRC(ViaBusMessage_t* msg) {
    msg
}


void checkHeaderCRC(ViaBusMessage_t* msg) {
    
}


ViaBus::ViaBus(HardwareSerial* serialBroadcast) {
    this->serialBroadcast = serialBroadcast;
}


void ViaBus::begin(long int baudrate) {
    serialBroadcast->begin(baudrate);
}


void ViaBus::loop() {
    unsigned long t = millis();
    unsigned long dt = 0;
    if (lastLoopTime >= 0) {
        dt = t - lastLoopTime;
    }
    lastLoopTime = t;
    lastMessageTimer += dt;
    if (lastMessageTimer > HEARTBEAT_MS) {
        if (this->onMessageCallback != nullptr) {
            this->onMessageCallback(0, "hello");
        }
        lastMessageTimer = 0;
    }
}


void ViaBus::onMessage(void (*callback)(uint8_t address, const char* message)) {
    this->onMessageCallback = callback;
}


int ViaBus::send(uint8_t address, const char* message) {
    //
    return 0;
}


/*
ViaBusController::ViaBusController(Stream* serial_broadcast) {
    this->serial_broadcast = serial_broadcast;
}


void ViaBusController::begin(long int baudrate) {
    serial_broadcast.begin(baudrate);
    this->baudrate = baudrate;
}


ViaBusPeripheral::ViaBusPeripheral(Stream* serial_broadcast, Stream* serial_chain) {
    this->serial_broadcast = serial_broadcast;
    this->serial_chain = serial_chain;
}


void ViaBusPeripheral::begin(long int baudrate) {
    serial_broadcast.begin(baudrate);
    serial_chain.begin(baudrate);
    this->baudrate = baudrate;
}


void ViaBusPeripheral::on_message(void (*callback)) {
    this->on_msg_callback = callback;
}


void ViaBusPeripheral::loop(long int baudrate) {
    unsigned long t = millis();
    unsigned long dt = 0;
    if (last_call_ms >= 0) {
        dt = t - last_call_ms;
    }
    last_call_ms = t;
    timer_heartbeat_ms += dt;

    while (serial_broadcast.available()) {
        char c = serial_broadcast.read()
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
*/


