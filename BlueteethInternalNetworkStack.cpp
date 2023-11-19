#include <BlueteethInternalNetworkStack.h>

extern BlueteethBaseStack * internalNetworkStackPtr;

extern uint32_t streamTime; //DEBUG (REMOVE LATER)

void inline flushSerialBuffer(HardwareSerial * serial){
    while (serial -> available() > 0){
        serial -> read();
    }
}


/* Grab contents from serial buffer and package them into a blueteeth packet
*
*  @serial - The serial port being read from
*  @return - The blueteeth packet received from the buffer
*/
BlueteethPacket retrievePacketFromBuffer(HardwareSerial * serial){

    //Have to temporarily grab the data form the UART buffer into a byte buffer (so it can be packaged in the packet struct)
    // uint8_t buffer [sizeof(blueTeethPacket)];
    // serial -> readBytes(buffer, sizeof(buffer));
    
    //Package the packet struct with data from buffer
    BlueteethPacket retrievedPacket;
    retrievedPacket.tokenFlag = serial -> read();
    retrievedPacket.srcAddr = serial -> read();
    retrievedPacket.dstAddr = serial -> read();
    retrievedPacket.type = serial -> read();
    for (int i = 0; i < MAX_PAYLOAD_SIZE; i++){
        retrievedPacket.payload[i] = serial -> read();
    }
    return retrievedPacket;

}

void uartFrameReceived(){
    
    int bytesInBuffer = internalNetworkStackPtr -> controlPlane -> available();

    //Don't try to package packet unless there's enough data on the buffer to do so
    if(bytesInBuffer < sizeof(BlueteethPacket)){
        // Serial.print("Not enough bits for a packet\n\r"); //DEBUG STATEMENT 
        return;
    }  
    //If there are extra bytes in the buffer, something is wrong, and the buffer has been corrupted.
    /*  
        This seems to prevent corruption as the serial callback doesn't get called EVERY time 
        a new frame is received, but after a burst of frames ends. Maybe move into the prior conditional. 
    */
    else if ((bytesInBuffer % sizeof(BlueteethPacket)) != 0 ) 
    {
        // Serial.print("Buffer corrupted\n\r");
        flushSerialBuffer(internalNetworkStackPtr -> controlPlane);
        return;
    }      
    
    BlueteethPacket receivedPacket = retrievePacketFromBuffer(internalNetworkStackPtr -> controlPlane);

    // Serial.printf("[%lu - Rx Packet] t = %u, src = %u, dst = %u, type = %u, payload = %s\n\r", millis(), receivedPacket.tokenFlag, receivedPacket.srcAddr, receivedPacket.dstAddr, receivedPacket.type, (char*) receivedPacket.payload); //DEBUG STATEMENT

    //Assess the packet
    //If the token was received, transmit items in queue.
    if (receivedPacket.tokenFlag == 1){
        // Serial.print("Token packet received\n\r"); //DEBUG STATEMENT 
        internalNetworkStackPtr -> tokenReceived();
    }
    else if (receivedPacket.dstAddr == 255){ //Packets with destination address 255 are intended to intialize node addresses
        Serial.print("Received initializaiton packet\n\r"); //DEBUG STATEMENT
        internalNetworkStackPtr -> initializationReceived(receivedPacket);
    }
    //If the packet was sent from this device, throw it away.
    else if (receivedPacket.srcAddr == internalNetworkStackPtr -> address){
        // Serial.print("Received my own packet\n\r"); //DEBUG STATEMENT 
        //Do nothing
    }
    else if (receivedPacket.dstAddr == 254){ //Packets with destination address 254 are intended for broadcasts
        // Serial.print("Received broadcast packet\n\r"); //DEBUG STATEMENT 
        internalNetworkStackPtr -> queuePacket(false, receivedPacket);
        xQueueSend(internalNetworkStackPtr -> receivedPacketBuffer, &receivedPacket, 0);
        vTaskResume(*(internalNetworkStackPtr -> receiveTaskCallback));
    }
    //If instead the packet was meant for this device, add it to the buffer
    else if (receivedPacket.dstAddr == internalNetworkStackPtr -> address){
        // Serial.print("Received packet for myself\n\r"); //DEBUG STATEMENT 
        xQueueSend(internalNetworkStackPtr -> receivedPacketBuffer, &receivedPacket, 0);
        vTaskResume(*(internalNetworkStackPtr -> receiveTaskCallback));
    }
    //If the packet was not meant for this device at all, send it on to the next device in the ring
    else {
        // Serial.print("Received packet for someone else\n\r"); //DEBUG STATEMENT
        internalNetworkStackPtr -> transmitPacket(receivedPacket);
    }
}

void dataStreamReceived(){

    static int totalBytes = 0;
    static uint32_t time = 0;
    int newBytes = internalNetworkStackPtr -> dataPlane -> available();
    
    if (totalBytes == 0) {
        time = millis();
    }

    if (totalBytes + newBytes > MAX_DATA_BUFFER_SIZE){
        newBytes = MAX_DATA_BUFFER_SIZE - totalBytes;
    }

    internalNetworkStackPtr -> dataPlane -> readBytes(internalNetworkStackPtr -> dataBuffer + totalBytes, newBytes);

    totalBytes += newBytes;

    if (totalBytes == 40000){ //DEBUG STATEMENT
        streamTime = millis() - time;
        totalBytes = 0;
    } 

    // Serial.printf("Data received: %d\n\r", totalBytes); //DEBUG STATEMENT
}