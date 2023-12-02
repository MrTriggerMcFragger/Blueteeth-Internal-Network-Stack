
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
// #define DIRECT_TRANSFER //If defined, BT data streaming is direct from the UART buffer

using namespace std;

#define DATA_STREAM_TEST_SIZE (40320) // 5040

#define MAX_PAYLOAD_SIZE (10)
#define TOKEN_HOLD_TIME_MS (10) //Used to create a fixed token passing speed
#define PACKET_DELAY_TIME_MS (5) //Used to create a fixed data packet transmit speed
#define RING_TOKEN_GENERATION_DELAY_MS (1000)
#define MAX_DATA_BUFFER_SIZE (40320)

#define DATA_PLANE_BAUD (6720000)//6720000 (WORKING) 3840000 (STABLE)

#define MAX_DATA_PLANE_PAYLOAD_SIZE (420)
#define DATA_PLANE_SERIAL_TX_BUFFER_SIZE (1056) //Make an integer multiple of the frame size
#define DATA_PLANE_SERIAL_RX_BUFFER_SIZE (4125) //Make an integer multiple of the frame size

//Macros for framing
#define FRAME_START_SENTINEL (0b11111111)
#define FRAME_PADDING_SENTINEL (0b10000000)
#define FRAME_SIZE (33)
#define BYTES_PER_ROTATION (8)
#define ROTATIONS_PER_FRAME (4)
#define PAYLOAD_SIZE (28)

enum PacketType {NONE, INITIALIZAITON, CLAIM_ADDRESS, PING, STREAM_RESULTS, SCAN, CONNECT, SELECT, DISCONNECT, STREAM, FLUSH, DROP, TEST, RESERVED};
//RESERVED used for error checking (if > RESERVED) the packet is a fabrication

/* Control plane callback function
*
*/
void uartFrameReceived();

/* Data plane callback function
*
*/
void dataStreamReceived();

void dataStreamErrorEncountered(hardwareSerial_error_t error);

/* Packages data into BlueTeeth Data Stream frames
*
*  @PackedData - A pointer to a buffer where the packed data will be stored.
*  @len - The length of the unpacked data.
*  @dataBuffer - A double-ended queue containing the unpacked data.
*  @requestPadding - If the length of the data is not an integer multiple of the frame size, you can insert the last frame with real data and pad out the rest with a padding character.
*/
void inline packDataStream(uint8_t * packedData, int dataLength, deque<uint8_t> & payloadBuffer, bool requestPadding = true){

    uint8_t select_lower;
    //Converting dataLength to an integer multiple of the frame size
  
    size_t finalFrameEnd = ceil( (double) dataLength / PAYLOAD_SIZE ) * FRAME_SIZE;
    size_t finalPayloadBytePosition = dataLength/PAYLOAD_SIZE * FRAME_SIZE + 1 + (dataLength % PAYLOAD_SIZE)/BYTES_PER_ROTATION + dataLength % PAYLOAD_SIZE; //Size of frames prior to the final frame + Frame start sentinel + Bytes from complete rotations of final frame + Bytes from incomplete rotation of the final frame.

    for(volatile int frame = 0; frame < finalFrameEnd; frame += FRAME_SIZE){
        packedData[frame] = FRAME_START_SENTINEL;
        for (volatile int rotation = 0; rotation < (ROTATIONS_PER_FRAME * BYTES_PER_ROTATION) ; rotation += BYTES_PER_ROTATION){
          select_lower = 0b00000001; //used to select the lower portion of the unpacked byte;
          packedData[frame + rotation + 1] = 0; //Necessary as the first byte is some random number prior to starting algorithm 
          for(volatile int byte = 1; byte < BYTES_PER_ROTATION; ++byte){ //pre-increment is technically faster as there isn't a copy of the var made (so doing ++byte rather than byte++)

              if ((frame + rotation + byte) < finalPayloadBytePosition) {
                packedData[frame + rotation + byte] += payloadBuffer.front() >> byte;
                packedData[frame + rotation + byte + 1] = (select_lower & payloadBuffer.front()) << (7 - byte);           
                payloadBuffer.pop_front();
                select_lower = (select_lower << 1) + 1;
              }
              else {         
                packedData[frame + rotation + byte + 1] = FRAME_PADDING_SENTINEL;
                // cout << "Placing pading...." << endl;
                if (frame + rotation + byte + 1 >= finalFrameEnd){
                    Serial.println("Something went wrong trying to package a frame...");
                    return;
                }
              }
          }
        }
    }
}


/* Unpacks data from BlueTeeth data stream frames into buffer
*
*  @PackedData - A pointer to a buffer where the packed data is stored.
*  @len - The length of the packed data.
*  @dataBuffer - A double-ended queue containing the unpacked data.
*/
void inline unpackDataStream(uint8_t * packedData, int totalFrameLength, deque<uint8_t> & dataBuffer, SemaphoreHandle_t dataBufferMutex){
    uint8_t select_lower;
    uint8_t select_upper;
    vector<uint8_t> payload;

    int cnt = 0;


    loop_start:
    // const auto isDataCorrupted = [dataBuffer](int cnt, int byte) -> bool {
    //     return dataBuffer.size() < (cnt + byte + 1) ? true : false;
    // }; //Check to make sure the size of the buffer didn't change between operations.

    while (cnt < totalFrameLength){

        if (packedData[cnt++] == FRAME_START_SENTINEL){ //Don't begin unpacking until the sentinal character is found 
            for (int rotation = 0; rotation < ROTATIONS_PER_FRAME; ++rotation){ //pre-increment is technically faster as there isn't a copy of the var made (so doing ++byte rather than byte++)
                select_upper = 0b01111111; //Used to select the upper portion of the unpacked byte 
                select_lower = 0b01000000; //Used to select the lower portion of the unpacked byte
                for(int byte = 0; byte < (BYTES_PER_ROTATION - 1); ++byte){
                    switch(packedData[cnt + byte + 1]){
                        case FRAME_START_SENTINEL:
                          if (payload.size() > PAYLOAD_SIZE){
                            payload.erase(payload.end() - payload.size()% PAYLOAD_SIZE , payload.end());
                          }
                          else{
                            payload.erase(payload.begin(), payload.end());
                          }
                          cnt += byte + 1;; 
                          goto loop_start;
                      
                        case FRAME_PADDING_SENTINEL: //If you see a start sentinel before the end of a frame, the frame was corrupted.    
                          cnt += (ROTATIONS_PER_FRAME - rotation) * BYTES_PER_ROTATION; 
                          goto loop_start;

                        default:
                            payload.push_back(
                                ((packedData[cnt + byte] & select_upper) << (byte + 1)) + 
                                ((packedData[cnt + byte + 1] & select_lower) >> (6 - byte))
                            ); 
                            select_upper = select_upper >> 1;
                            select_lower += 1 << (5 - byte);
                    }
                }
                cnt += BYTES_PER_ROTATION;
            }

            if (payload.size() != PAYLOAD_SIZE){
                // Serial.print("Something went wrong with the unpack algorithm\n\r");
            }

        }

  }
  while (xSemaphoreTakeFromISR(dataBufferMutex, NULL) == pdFALSE){
    // Serial.printf("Data plane was blocked...\n\r");
    vPortYield();
  }
  Serial.printf("%s took the mutex\n\r", "DATA PLANE");
  dataBuffer.insert(dataBuffer.end(), payload.begin(), payload.end());
  Serial.printf("%s released the mutex\n\r", "DATA PLANE");
  xSemaphoreGiveFromISR(dataBufferMutex, NULL);
//   xSemaphoreGive(dataBufferMutex);

    //   if (droppedBytes > 0) Serial.printf(" Threw away %d bytes out of %d...\n\r", droppedBytes, len);
}

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
        this -> dataBufferWriteInProgress = false;
        this -> dataBufferAccessor = "UNINITIALIZED";
    }

    virtual void begin(){

        this -> controlPlane -> begin(115200);
        this -> controlPlane -> onReceive(uartFrameReceived);
        //441600
        this -> dataPlane -> setRxBufferSize(DATA_PLANE_SERIAL_RX_BUFFER_SIZE);
        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19); //Need to use pins 18 & 19 as Serial1 defaults literally cannot be used (they're involved in flashing and will crash your program). Baud chosen to have 441600 byte/s rate (> 40k & a multiple of 9600).
        this -> dataPlane -> onReceive(dataStreamReceived);
        this -> dataPlane -> onReceiveError(dataStreamErrorEncountered); //Ran into an error
        this -> dataPlaneMutex = xSemaphoreCreateMutex();
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

    void  flushDataPlaneSerialBuffer(){
        static uint8_t tmp[DATA_PLANE_SERIAL_RX_BUFFER_SIZE];
        this -> dataPlane -> readBytes(tmp, this -> dataPlane -> available()); //Faster than individual reads
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

    int getDataPlaneBytesAvailable(){

        return this -> dataPlane -> available();
    
    }
    int getDataPlaneBytesAvailableToWrite(){

        return this -> dataPlane -> availableForWrite();
    
    }

    void recordDataBufferAccessTime(){
        this -> lastDataReceptionTime = millis();
    }

    uint32_t getLastDataBufferAccessTime(){
        return this -> lastDataReceptionTime;
    }

    uint32_t timeElapsedSinceLastDataBufferAccess(){
        return millis() - this -> lastDataReceptionTime;
    }

    deque<uint8_t> dataBuffer;

    /*  Attempt to declare a write in progress.
    *
    *   @return - Whether the declaration was successful or not (true = success, false = failure) 
    */
    bool declareActiveDataBufferReadWrite(std::string accessIdentifier){

        static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
        // portENTER_CRITICAL(&mutex);

        if (dataBufferWriteInProgress == true){
            portEXIT_CRITICAL(&mutex);
            Serial.printf("<%s> tried access the data buffer, but <%s> is utilizing the resource.\n\r", accessIdentifier.c_str(), dataBufferAccessor.c_str());
            return false;
        }

        dataBufferWriteInProgress = true;
        dataBufferAccessor.assign(accessIdentifier);
        // portEXIT_CRITICAL(&mutex);
        return true;
    }

    /*  Attempt to declare that it's safe to read/write from the data buffer
    *
    *   @return - Whether the access state was changed or not (true = went from unsafe to safe, false = was already safe) 
    */
    bool declareDataBufferSafeToAccess(std::string accessIdentifier){

        static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mutex);

        if (dataBufferWriteInProgress == false){
            portEXIT_CRITICAL(&mutex);
            return false;
        }
        if ((dataBufferAccessor != "NOBODY") && (accessIdentifier != dataBufferAccessor)){ //Something weird is happening here.. It should be impossible to get the NOBODY identifier in this check
            portEXIT_CRITICAL(&mutex);
            Serial.printf("<%s> tried to declare the databuffer safe to access, but <%s> is utilizing the resource.\n\r", accessIdentifier.c_str() , dataBufferAccessor.c_str());
            return false;
        }
        dataBufferWriteInProgress = false;
        dataBufferAccessor = "NOBODY";
        portEXIT_CRITICAL(&mutex);
        return true;
    }

    bool checkForActiveDataBufferWrite(){
        return dataBufferWriteInProgress;
    }

    std::string getOriginalAccessor(){
        return this -> dataBufferAccessor;
    }

    void dataBufferTimeoutReset(){
        this -> lastDataReceptionTime = millis();
    }

    bool isNetworkAccessingResources(){
        return networkAccessingResources;
    }


    xSemaphoreHandle dataPlaneMutex;
    
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
    
    bool networkAccessingResources;
    bool dataBufferWriteInProgress;
    std::string dataBufferAccessor;
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
        
        this -> dataPlane->setTxBufferSize(DATA_PLANE_SERIAL_TX_BUFFER_SIZE);
        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19);
        this -> dataPlaneMutex = xSemaphoreCreateMutex();
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