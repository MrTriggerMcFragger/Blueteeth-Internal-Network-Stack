
#ifndef BlueteethInternalNetworkStack_h
#define BlueteethInternalNetworkStack_h

#include <stdint.h>
#include <functional>
#include "HardwareSerial.h"
#include <string>
#include <set>
#include <deque>

#define TIME_STREAMING //If defined, time is measured for test streams (will decrease performance)
// #define DIRECT_TRANSFER //If defined, BT data streaming is direct from the UART buffer

using namespace std;

#define DATA_STREAM_TEST_SIZE (5016)

#define MAX_PAYLOAD_SIZE (10)
#define TOKEN_HOLD_TIME_MS (10) //Used to create a fixed token passing speed
#define PACKET_DELAY_TIME_MS (5) //Used to create a fixed data packet transmit speed
#define RING_TOKEN_GENERATION_DELAY_MS (1000)
#define MAX_DATA_BUFFER_SIZE (40000)
#define DATA_PLANE_BAUD (6720000)//6720000 (WORKING)

#define MAX_DATA_PLANE_PAYLOAD_SIZE (112)
#define DATA_PLANE_SERIAL_TX_BUFFER_SIZE (1024)

#ifdef DIRECT_TRANSFER
#define DATA_PLANE_SERIAL_RX_BUFFER_SIZE (40000)
#else
#define DATA_PLANE_SERIAL_RX_BUFFER_SIZE (4096) //1024
#endif

//Macros for framing
#define FRAME_START_SENTINEL (0b11111111)
#define FRAME_PADDING_SENTINEL (0b10000000)
#define FRAME_SIZE (33)
#define BYTES_PER_ROTATION (8)
#define ROTATIONS_PER_FRAME (4)
#define PAYLOAD_SIZE (28)

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
*  @PackedData - A pointer to a buffer where the packed data will be stored.
*  @len - The length of the unpacked data.
*  @dataBuffer - A double-ended queue containing the unpacked data.
*  @requestPadding - If the length of the data is not an integer multiple of the frame size, you can insert the last frame with real data and pad out the rest with a padding character.
*/
void inline packDataStream(uint8_t * packedData, int len, deque<uint8_t> & payloadBuffer, bool requestPadding = true){
  
    uint8_t select_lower;
    size_t packagedDataEnd = ((len % PAYLOAD_SIZE == 0) ? len :  len + ((len + 1) % PAYLOAD_SIZE) ) * FRAME_SIZE/PAYLOAD_SIZE; //Converting len to an integer multiple of the frame size

    //dataEnd is where the actual data is finished. After this value, all values are padding.
    //It is the number of frames in addition to the remaining length (accounting for the extra byte added per rotation)
    size_t dataEnd = len / PAYLOAD_SIZE * FRAME_SIZE + (len % PAYLOAD_SIZE) / 7 * BYTES_PER_ROTATION + len % 7; 

    // Serial.printf("packageDataEnd is %d, dataEnd is %d, and payloadBuffer size is %d\n\r", packagedDataEnd, dataEnd, payloadBuffer.size());

    for(volatile int frame = 0; frame < packagedDataEnd; frame += FRAME_SIZE){
        packedData[frame] = FRAME_START_SENTINEL;
        for (volatile int rotation = 0; rotation < (ROTATIONS_PER_FRAME * BYTES_PER_ROTATION) ; rotation += BYTES_PER_ROTATION){
          select_lower = 0b00000001; //used to select the lower portion of the unpacked byte;
          packedData[frame + rotation + 1] = 0; //Necessary as the first byte is some random number prior to starting algorithm 
          for(volatile int byte = 1; byte < BYTES_PER_ROTATION; byte++){
              
              if ((frame + rotation + byte) <= dataEnd) {
                packedData[frame + rotation + byte] += payloadBuffer.front() >> byte;
                packedData[frame + rotation + byte + 1] = (select_lower & payloadBuffer.front()) << (7 - byte);           
                payloadBuffer.pop_front();
                select_lower = (select_lower << 1) + 1;
              }
              else {         
                packedData[frame + rotation + byte + 1] = FRAME_PADDING_SENTINEL;
                if (frame + rotation + byte + 1 >= packagedDataEnd){
                    // Serial.print("Went out of bounds... Exitting now.\n\r");
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
void inline unpackDataStream(uint8_t * packedData, int len, deque<uint8_t> & dataBuffer){
  uint8_t select_lower;
  uint8_t select_upper;

  // cout << "Length is " << len << endl;
  int droppedBytes = 0;
  int cnt = 0;
  //Circle through all of the data in the stream
  loop_start:
  while (cnt < len){

    if (packedData[cnt++] == FRAME_START_SENTINEL){ //Don't begin unpacking until the sentinal character is found 
      for (int rotation = 0; rotation < ROTATIONS_PER_FRAME; rotation++){
        select_upper = 0b01111111; //Used to select the upper portion of the unpacked byte 
        select_lower = 0b01000000; //Used to select the lower portion of the unpacked byte
        for(int byte = 0; byte < (BYTES_PER_ROTATION - 1); byte++){
          if (packedData[cnt + byte + 1] == FRAME_PADDING_SENTINEL){
            cnt += (ROTATIONS_PER_FRAME - rotation) * BYTES_PER_ROTATION; //bytes in unused rotations   
            goto loop_start; //Can't break out of nested loop.
          }
          dataBuffer.push_back(
            ((packedData[cnt + byte] & select_upper) << (byte + 1)) + 
            ((packedData[cnt + byte + 1] & select_lower) >> (6 - byte))
          ); 
          select_upper = select_upper >> 1;
          select_lower += 1 << (5 - byte);
        }
        cnt += BYTES_PER_ROTATION;
      }
    }
    else {
        // Serial.printf("[%d] ", cnt);
        droppedBytes++;
    }
  }
//   if (droppedBytes > 0) Serial.printf(" Threw away %d bytes out of %d...\n\r", droppedBytes, len);
}
int32_t a2dpDirectTransfer(uint8_t * data, int32_t len);

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
        //441600
        this -> dataPlane -> setRxBufferSize(DATA_PLANE_SERIAL_RX_BUFFER_SIZE);
        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19); //Need to use pins 18 & 19 as Serial1 defaults literally cannot be used (they're involved in flashing and will crash your program). Baud chosen to have 441600 byte/s rate (> 40k & a multiple of 9600).
        #ifndef DIRECT_TRANSFER
        this -> dataPlane -> onReceive(dataStreamReceived);
        #endif
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

    void begin(){
        this -> controlPlane -> begin(115200);
        this -> controlPlane -> onReceive(uartFrameReceived);
        
        this -> dataPlane->setTxBufferSize(DATA_PLANE_SERIAL_TX_BUFFER_SIZE);
        this -> dataPlane -> begin(DATA_PLANE_BAUD, SERIAL_8N1, 18, 19);
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