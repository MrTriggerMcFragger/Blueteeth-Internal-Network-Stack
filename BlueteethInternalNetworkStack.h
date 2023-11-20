
#ifndef BlueteethInternalNetworkStack_h
#define BlueteethInternalNetworkStack_h

#include <stdint.h>
#include <functional>
#include "HardwareSerial.h"
#include <string>
#include <set>
#include <deque>

#define TIME_STREAMING (1)

using namespace std;

#define MAX_PAYLOAD_SIZE (10)
#define TOKEN_HOLD_TIME_MS (10) //Used to create a fixed token passing speed
#define PACKET_DELAY_TIME_MS (5) //Used to create a fixed data packet transmit speed
#define RING_TOKEN_GENERATION_DELAY_MS (1000)
#define MAX_DATA_BUFFER_SIZE (40000)
#define DATA_PLANE_BAUD (441600)

enum PacketType {NONE, INITIALIZAITON, CLAIM_ADDRESS, PING, STREAM_RESULTS, SCAN, CONNECT, SELECT, DISCONNECT, STREAM, FLUSH, TEST};

/* Control plane callback function
*
*/
void uartFrameReceived();

/* Data plane callback function
*
*/
void dataStreamReceived();

/* Flushes all bytes in the serial data buffer -> When there's too much data in the buffer, no new data is added, and the onReceive callback won't get called.
*
*/
void inline flushSerialBuffer(HardwareSerial * serial);

class BlueteethPacket {

public:

    /*  Default Constructor - Sets packet to an initializaiton packet for requesting an address
    * 
    */
    BlueteethPacket(){ 
        this -> tokenFlag = 0;
        this -> srcAddr = 255;
        this -> dstAddr = 0;
        this -> type = NONE;
        for (int i = 0; i < MAX_PAYLOAD_SIZE; i++){
            payload[i] = NULL;
        }

    }


    /* Overloaded Constructor - Takes in inputs of the most commonly needed parameters during initialization
    *
    *   @tokenPacket - true = is a token packet, false = not a token packet
    *   @srcAddr - Packet source address
    *   @dstAddr - Packet destination address
    */
    BlueteethPacket(bool tokenPacket, uint8_t srcAddr, uint8_t dstAddr){
        if (tokenPacket){
            this -> tokenFlag = 1;
        }
        else this -> tokenFlag = 0;
        this -> srcAddr = srcAddr;
        this -> dstAddr = dstAddr;
        this -> type = NONE;
        for (int i = 0; i < MAX_PAYLOAD_SIZE; i++){
            payload[i] = NULL;
        }
    }

    uint8_t tokenFlag;  
    uint8_t srcAddr;
    uint8_t dstAddr;
    uint8_t type; 
    uint8_t payload[MAX_PAYLOAD_SIZE]; 

};

class BlueteethBaseStack
{

public:

    friend BlueteethPacket;

    /* Workaround for not being able to set a member function as a callback to serial interrupts through HardwareSerial::onRecieve
    *
    */
    friend void uartFrameReceived();

    /* Workaround for not being able to set a member function as a callback to serial interrupts through HardwareSerial::onRecieve
    *
    */
    friend void dataStreamReceived();

    /* Constructor
    *
    *  @bufferSize - The size of the received packet buffer
    *  @recieveTaskCallback - Task handle for the task to be resumed when a new packet has been received.
    *  @controlPlane - Serial port for the control plane
    *  @dataPlane - Serial port for the data plane
    */
    BlueteethBaseStack(uint8_t bufferSize, TaskHandle_t * receiveTaskCallback, HardwareSerial * controlPlane, HardwareSerial * dataPlane){
                
        receivedPacketBuffer = xQueueCreate(bufferSize, sizeof(BlueteethPacket));
        transmitPacketBuffer = xQueueCreate(bufferSize, sizeof(BlueteethPacket));

        this -> receiveTaskCallback = receiveTaskCallback;
        this -> controlPlane = controlPlane;
        this -> dataPlane = dataPlane;

        this -> address = 255; //Max value (indicates that the node has not been assigned an address)
    
    }

    void begin(){

        this -> controlPlane -> begin(115200);
        this -> controlPlane -> onReceive(uartFrameReceived);
        //441600
        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19); //Need to use pins 18 & 19 as Serial1 defaults literally cannot be used (they're involved in flashing and will crash your program). Baud chosen to have 441600 byte/s rate (> 40k & a multiple of 9600).
        this -> dataPlane -> onReceive(dataStreamReceived);
    }
    
    /* Queues a packet to be sent to the rest of the network upon receiving a token
    *
    *  @overwrite - Whether the packet buffer should be overwritten (if it is currently full)   
    *  @packet - The Blueteeth packet being queued
    *  @return - Indication of whether the buffer was written to  (0 = Did not write to packet buffer, 1 = Wrote to packet buffer)
    */
    bool queuePacket(bool overwrite, BlueteethPacket packet){
        
        // Serial.print("Adding packet to buffer\n\r"); //DEBUG STATEMENT
        xQueueSend( this -> transmitPacketBuffer, &packet, 0 );
        // Serial.print("Added packet to buffer\n\r"); //DEBUG STATEMENT
        return true;

    }
    
    /*  Retrieves the packet at the top of the receive buffer.
    *
    *   @return - The packet at the top of the receive buffer.
    */
    BlueteethPacket getPacket(){
        BlueteethPacket retrievedPacket;
        xQueueReceive(this -> receivedPacketBuffer, &retrievedPacket, 0);
        return retrievedPacket;

    }

    void flushDataPlaneSerialBuffer(){
        flushSerialBuffer(this -> dataPlane);
    }

    /*  Retrieves the device's address.
    *
    *   @return - This device's address.
    */
    uint8_t getAddress(){
        return this -> address;
    }

    /*  Stream data over the data plane -> This verison streams a single byte
    *
    *   @byte - The byte being transmitted;
    */
    void streamData(uint8_t byte){
        this -> dataPlane -> write(byte);
    };

    /*  Stream data over the data plane -> This version will stream more bytes
    *
    *   @byte - The byte being transmitted;
    */
    void streamData(const uint8_t * bytes, int length){
        // Serial.printf("Sending %d bytes data...\n\r", length);
        this -> dataPlane -> write(bytes, length);
    };
    
    /*  Callback to perform appropriate actions upon token packet reception
    *
    */
    virtual void tokenReceived(){

        while(uxQueueMessagesWaiting(this -> transmitPacketBuffer) > 0){
                // Serial.print("Transmitted a queued packet\n\r"); //DEBUG STATEMENT
                BlueteethPacket transmitPacket;
                xQueueReceive(this -> transmitPacketBuffer, &transmitPacket, 0);
                this -> transmitPacket(transmitPacket);
                // Serial.printf("[%lu - Tx Packet] t = %u, src = %u, dst = %u, type = %u, payload = %s\n\r", millis(), transmitPacket.tokenFlag, transmitPacket.srcAddr, transmitPacket.dstAddr, transmitPacket.type, (char*) transmitPacket.payload); //DEBUG STATEMENT
                delay(PACKET_DELAY_TIME_MS);
        }

        delay(TOKEN_HOLD_TIME_MS);

        BlueteethPacket tokenPacket(true, this -> address, 254);
        this -> transmitPacket(tokenPacket);
        // Serial.print("Token passed\n\r"); //DEBUG STATEMENT
    }

    /*  Callback to perform appropriate actions upon initialization packet reception
    *  
    *   @receivedPacket - The initialization packet recevied.
    */
    virtual void initializationReceived(BlueteethPacket receivedPacket){
        if (this -> address == 255){
            this -> address = receivedPacket.payload[0];
            
            BlueteethPacket addressClaimPacket(false, this -> address, 0);
            addressClaimPacket.type = CLAIM_ADDRESS;
            addressClaimPacket.payload[0] = this -> address;
            this -> queuePacket(1, addressClaimPacket);

            receivedPacket.payload[0]++; //increment the address value and pass it on
        }

        this -> queuePacket(1, receivedPacket);
    }

    deque<uint8_t> dataBuffer;


protected:
    
    void transmitPacket(BlueteethPacket packet){
        this -> controlPlane -> write(packet.tokenFlag);
        this -> controlPlane -> write(packet.srcAddr);
        this -> controlPlane -> write(packet.dstAddr);
        this -> controlPlane -> write(packet.type);
        for (int i = 0; i < MAX_PAYLOAD_SIZE; i++){
            this -> controlPlane -> write(packet.payload[i]);
        }
    }

    uint8_t address;
    string deviceType = "Base";
    QueueHandle_t transmitPacketBuffer;
    QueueHandle_t receivedPacketBuffer; 
    HardwareSerial * controlPlane;
    HardwareSerial * dataPlane;
    TaskHandle_t  * receiveTaskCallback;

};

class BlueteethMasterStack: public BlueteethBaseStack {

public:
    
    /* Constructor
    *
    *  @bufferSize - The size of the received packet buffer
    */
    BlueteethMasterStack(uint8_t bufferSize, TaskHandle_t * receiveTaskCallback, HardwareSerial * controlPlane, HardwareSerial * dataPlane) : BlueteethBaseStack(bufferSize, receiveTaskCallback, controlPlane, dataPlane){   
        
        this -> address = 0;
        this -> deviceType = "Master";

        BlueteethPacket initializationPacket(false, this -> address, 255);
        initializationPacket.type = INITIALIZAITON;
        initializationPacket.payload[0] = 1;
        this -> queuePacket(1, initializationPacket);
    }

    /* To be called from a watchdog timer task to generate a new token if the token is lost.
    *
    */
    void generateNewToken(){
        BlueteethPacket newTokenPacket;
        newTokenPacket.tokenFlag = 1;
        this -> transmitPacket(newTokenPacket);
    }

    /* Callback to perform appropriate actions upon token packet reception
    *
    */
    void tokenReceived() override {
        this -> tokenRxFlag = true;
        this -> BlueteethBaseStack::tokenReceived();
    }

    /* Callback to perform appropriate actions upon initialization packet reception
    *  
    *  @receivedPacket - The initialization packet recevied.
    */
    void initializationReceived(BlueteethPacket receivedPacket) override{
        
        if (receivedPacket.type != CLAIM_ADDRESS){
            return; //Don't need to handle anything if the initialization pacekt is not an address claim
        }

        uint8_t newAddress = receivedPacket.payload[0];
        if (knownAddresses.count(newAddress) == 0){
            this -> knownAddresses.insert(receivedPacket.payload[0]);
        }
        else {
            Serial.printf("Device attempted to use address %d, but it was already registered to another device\n\r", newAddress);
        }
    }

    bool getTokenRxFlag(){
        return this -> tokenRxFlag;
    }

    void resetTokenRxFlag(){
        tokenRxFlag = false;
    }

private:

    bool tokenRxFlag;
    set<uint8_t> knownAddresses;
    
};

#endif