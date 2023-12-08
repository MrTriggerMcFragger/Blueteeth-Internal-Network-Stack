
#ifndef BlueteethInternalNetworkStack_h
#define BlueteethInternalNetworkStack_h

#include <stdint.h>
#include <functional>
#include "HardwareSerial.h"
#include <string>
#include <set>
#include <deque>
#include <vector>

#define TIME_STREAMING //If defined, time is measured for test streams (will decrease performance)

using namespace std;

#define DATA_STREAM_TEST_SIZE (10080) // 5040

#define MAX_PAYLOAD_SIZE (10)
#define TOKEN_HOLD_TIME_MS (10) //Used to create a fixed token passing speed
#define PACKET_DELAY_TIME_MS (5) //Used to create a fixed data packet transmit speed
#define RING_TOKEN_GENERATION_DELAY_MS (1000)
#define MAX_DATA_BUFFER_SIZE (10080)
#define DATA_PLANE_BAUD (3360000) 


//Macros for defining serial buffer sizes
#define DATA_PLANE_SERIAL_TX_BUFFER_SIZE (1024)
#define DATA_PLANE_SERIAL_RX_BUFFER_SIZE (2112) 

#define MAX_DATA_PLANE_PAYLOAD_SIZE (420)
#define DATA_STREAM_TIMEOUT (1000) //How much time (ms) can elapse after the data buffer is accessed before it is reset

//Macros for data stream framing
#define FRAME_START_SENTINEL (0b11111111)
#define FRAME_PADDING_SENTINEL (0b10000000)
#define FRAME_SIZE (33) //Size of a data stream frame
#define BYTES_PER_ROTATION (8) //How many bytes there are per a data stream frame "rotation"
#define ROTATIONS_PER_FRAME (4) //The number of "rotations" in a data stream frame
#define PAYLOAD_SIZE (28) //The number of bytes in the payload of a data stream frame

enum PacketType {NONE, INITIALIZAITON, CLAIM_ADDRESS, PING, STREAM_RESULTS, SCAN, CONNECT, SELECT, DISCONNECT, STREAM, FLUSH, DROP, TEST};

/* Control plane callback function
*
*/
void uartFrameReceived();

/* Data plane callback function
*
*/
void dataStreamReceived();


/* Packages data into BlueTeeth Data Stream frames
*
*  @framedData - A pointer to a buffer where the frame data will be stored.
*  @dataLength - The length of the frame payload data.
*  @dataBuffer - A double-ended queue containing the frame payload data.
*/
void packDataStream(uint8_t * framedData, int dataLength, deque<uint8_t> & payloadBuffer);

/* Unpacks data from BlueTeeth data stream frames into buffer
*
*  @dataStream - A pointer to a buffer where the framed data is stored.
*  @dataStreamLength - The number of bytes in the data streram.
*  @dataBuffer - A double-ended queue where the frame payload data can be stored.
*/
void unpackDataStream(const uint8_t * dataStream, const int dataStreamLength, deque<uint8_t> & dataBuffer, HardwareSerial * dataPlane);


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
        for (int i = 0; i < MAX_PAYLOAD_SIZE; ++i){ //pre-increment is technically faster as there isn't a copy of the var made (so doing ++i rather than i++)
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
        for (int i = 0; i < MAX_PAYLOAD_SIZE; ++i){ //pre-increment is technically faster as there isn't a copy of the var made (so doing ++i rather than i++)
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

    //Give necessary functions and classes access to private member variables and functions
    friend BlueteethPacket;
    friend int32_t a2dpDirectTransfer(uint8_t * data, int32_t len);
    friend void uartFrameReceived();
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

    virtual void begin(){

        this -> controlPlane -> begin(115200);
        this -> controlPlane -> onReceive(uartFrameReceived);
        
        this -> dataPlane -> setRxBufferSize(DATA_PLANE_SERIAL_RX_BUFFER_SIZE);
        this -> dataPlane -> setRxFIFOFull(DATA_PLANE_SERIAL_RX_BUFFER_SIZE);

        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19); //Need to use pins 18 & 19 as Serial1 defaults literally cannot be used (they're involved in flashing and will crash your program). Baud chosen to have 441600 byte/s rate (> 40k & a multiple of 9600).
        this -> dataPlane -> onReceive(dataStreamReceived);
        this -> dataBufferMutex = xSemaphoreCreateMutex();
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
    
    /*  Flushes the data plane serial buffer
    *
    */
    void flushDataPlaneSerialBuffer(){
        //Don't have memory for this even if its faster
        // uint8_t tmp[this -> dataPlane -> available()];
        // this -> dataPlane -> readBytes(tmp, this -> dataPlane -> available());
        for (int i = 0; i < this -> getDataPlaneBytesAvailable(); i++){
            this -> dataPlane -> read();
        }
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
    *   @bytes - The bytes being transmitted.
    *   @length - The number of bytes being transmitted.
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

    /*  Method to get the number of bytes vailable to read from the data plane's RX serial buffer.
    *  
    *   @return - The number of bytes available to read from the data plane's RX serial buffer.
    */
    int getDataPlaneBytesAvailable(){

        return this -> dataPlane -> available();
    
    }

    /*  Method to get the number of bytes vailable to read from the data plane's TX serial buffer.
    *  
    *   @return - The number of bytes available to read from the data plane's TX serial buffer.
    */
    int getDataPlaneBytesAvailableToWrite(){

        return this -> dataPlane -> availableForWrite();
    
    }

    /*  Records the time at which data was added to the data buffer (used to perform a flush of the data buffer/data plane serial
    *   buffer if stagnant for too long)
    *  
    */
    void recordDataReceptionTime(){
        this -> lastDataReceptionTime = millis();
    }

    /*  Records the time at which data was taken out of the data buffer (used to perform a flush of the data buffer/data plane
    *   serial buffer if stagnant for too long)
    *  
    */
    void recordDataBufferAccessTime(){
        this -> lastDataReceptionTime = millis();
    }

    /*  Records the time at which a data buffer/data plane serial buffer flush occured (used to perform a flush of the data buffer/data
    *   plane serial buffer if stagnant for too long)
    *  
    */
    void resetDataBufferTimeout(){
        this -> lastDataReceptionTime = millis(); 
    }

    /*  Returns the time that has elapsed since the last time the data buffer was altered (used to perform a flush of the data 
    *   buffer/data plane serial buffer if stagnant for too long)
    *  
    *   @return - The amount of time (ms) that has elapsed since the last time the data buffer was altered
    */
    uint32_t getTimeElapsedSinceLastDataBufferAccess(){
        return millis() - this -> lastDataReceptionTime;
    }

    deque<uint8_t> dataBuffer;
    SemaphoreHandle_t dataBufferMutex;

protected:
    
    /* Sends a Blueteeth packet on the control plane (should only be called when the node holds the token)
    *  
    *   @packet - The Blueteeth packet to be transmitted
    */
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
    uint32_t lastDataReceptionTime;
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

    void begin(){
        this -> controlPlane -> begin(115200);
        this -> controlPlane -> onReceive(uartFrameReceived);
        
        this -> dataPlane-> setTxBufferSize(DATA_PLANE_SERIAL_TX_BUFFER_SIZE);
        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19);
        
        this -> dataBufferMutex = xSemaphoreCreateMutex();
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
            try {
                this -> knownAddresses.insert(receivedPacket.payload[0]);
            }
            catch (std::exception e) {
                Serial.printf("Issue occured while trying to insert the payload into the known addresses set. e.what() = %s", e.what());
            }
            catch (...){
                Serial.print("Not a standard exception.");
            }
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