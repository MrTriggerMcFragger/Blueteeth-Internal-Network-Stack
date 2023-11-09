
#ifndef BlueteethInternalNetworkStack_h
#define BlueteethInternalNetworkStack_h

#include <stdint.h>
#include <functional>
#include "HardwareSerial.h"

#define MAX_PAYLOAD_SIZE (10)

typedef struct {

    bool tokenFlag;  
    uint8_t srcAddr;
    uint8_t dstAddr;
    uint8_t type; 
    uint8_t payload[MAX_PAYLOAD_SIZE]; 

} blueTeethPacket;

class BlueteethBaseStack
{

public:

    /* Constructor
    *
    *  @bufferSize - The size of the received packet buffer
    */
    BlueteethBaseStack(uint8_t bufferSize, HardwareSerial * controlPlane, HardwareSerial * dataPlane){
        
        receivedPacketBuffer = xQueueCreate(bufferSize, sizeof(blueTeethPacket));
        transmitPacketBuffer = xQueueCreate(bufferSize, sizeof(blueTeethPacket));

        this -> controlPlane = controlPlane;
        this -> dataPlane = dataPlane;

        this -> address = 255; //Max value (indicates that the node has not been assigned an address)
    
    }
    
    /* Queues a packet to be sent to the rest of the network upon receiving a token
    *
    *  @overwrite - Whether the packet buffer should be overwritten (if it is currently full)   
    *  @return - indication of whether the buffer was written to  (0 = Did not write to packet buffer, 1 = Wrote to packet buffer)
    */
    bool queuePacket(bool overwrite);

    /* Checks to see if a packet is available.
    *
    *  @return - indication of whether the buffer was overwritten.  
    */
    bool checkForPacket(){

        if (uxQueueMessagesWaiting(internalNetworkStack.transmitPacketBuffer) > 0){
            return 1;
        }
        else {
            return 0;
        }
    }
    
    /* Retrieves the packet at the top of the receive buffer.
    *
    *  @return - the packet at the top of the receive buffer.
    */
    blueTeethPacket getPacket(){
        blueTeethPacket retrievedPacket;
        xQueueReceive(internalNetworkStack.receivedPacketBuffer, &retrievedPacket, 0);
        return retrievedPacket;

    }

    /* Workaround for not being able to set a member function as a callback to serial interrupts through HardwareSerial::onRecieve
    *
    */
    friend void packetReceived();

    /* Sets the ISR callbacks for the controlPlane and dataPlane interrupts
    *
    */
    friend void blueTeethNetworkSetup();

protected:
    
    void transmitPacket(blueTeethPacket packet){

        this -> controlPlane -> write(packet.tokenFlag);
        this -> controlPlane -> write(packet.srcAddr);
        this -> controlPlane -> write(packet.dstAddr);
        for (int i = 0; i < MAX_PAYLOAD_SIZE; i++)
        this -> controlPlane -> write(packet.payload[i]);
    }

    uint8_t address;
    QueueHandle_t transmitPacketBuffer;
    QueueHandle_t receivedPacketBuffer; 
    HardwareSerial * controlPlane;
    HardwareSerial * dataPlane;

};

class BlueteethMasterStack: public BlueteethBaseStack {

public:
    
    /* Constructor
    *
    *  @bufferSize - The size of the received packet buffer
    */
    BlueteethMasterStack(uint8_t bufferSize, HardwareSerial * controlPlane, HardwareSerial * dataPlane) : BlueteethBaseStack(bufferSize, controlPlane, dataPlane){   
        
        this -> address = 0;

        blueTeethPacket initializationPacket;
        initializationPacket.dstAddr = 255;
        initializationPacket.payload[0] = 1;
        this -> transmitPacket(initializationPacket);
    }

    /* To be called from a watchdog timer task to generate a new token if one is lost.
    *
    */
    void generateNewToken(){

        blueTeethPacket newTokenPacket;
        newTokenPacket.tokenFlag = 1;
        this -> transmitPacket(newTokenPacket);   
    }


private:
    
};

#endif