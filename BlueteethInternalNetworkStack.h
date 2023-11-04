
#ifndef BlueteethInternalNetworkStack_h
#define BlueteethInternalNetworkStack_h

#define MAX_PAYLOAD_SIZE (50) //needs to accomdate up to 

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
    BlueteethBaseStack(uint8_t bufferSize){

        receivedPacketBuffer = xQueueCreate(bufferSize, sizeof(blueTeethPacket));
    
    }
    
    /* Queues a packet to be sent to the rest of the network upon receiving a token
    *
    *  @overwrite - Whether the packet buffer should be overwritten (if it is currently full)   
    *  @return - indication of whether the buffer was written to  (0 = Did not write to packet buffer, 1 = Wrote to packet buffer)
    */
    bool sendPacket(bool overwrite);

    /* Checks to see if a packet is available.
    *
    *  @return - indication of whether the buffer was overwritten.  
    */
    bool checkForPacket();
    
    /* Retrieves the packet at the top of the receive buffer.
    *
    *  @return - the packet at the top of the receive buffer.
    */
    blueTeethPacket getPacket();


    /* Schedules RTOS tasks related to the communications stack
    *
    *  @return - A pointer to the different tasks added to the scheduler.
    */
    TaskHandle_t * scheduleTasks();

protected:
    
    blueTeethPacket transmitPacketBuffer;
    QueueHandle_t receivedPacketBuffer; 

};

class BlueteethMasterStack: public BlueteethBaseStack {

public:
    
    /* Constructor
    *
    *  @bufferSize - The size of the received packet buffer
    */
    BlueteethMasterStack(uint8_t bufferSize):BlueteethBaseStack(bufferSize){   

    }


private:

};

#endif