#include <BlueteethInternalNetworkStack.h>

extern BlueteethBaseStack * internalNetworkStackPtr;

#ifdef TIME_STREAMING
uint32_t streamTime; //DEBUG (REMOVE LATER)
#endif

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
    //Making variables static to try and save time on allocating memory on each function call
    static int newBytes;
    static int currentSize;
    static bool flushToken;
    static uint8_t tmp [DATA_PLANE_SERIAL_RX_BUFFER_SIZE]; 

    if (internalNetworkStackPtr -> dataPlane -> available() >= (DATA_PLANE_SERIAL_RX_BUFFER_SIZE - 1) ){
        Serial.println("The serial buffer is full");
    }

    newBytes = internalNetworkStackPtr -> dataPlane -> available();
    currentSize = internalNetworkStackPtr -> dataBuffer.size();
    
    #ifdef TIME_STREAMING
    if (internalNetworkStackPtr -> dataBuffer.size() == 0) { 
        streamTime = millis();
    }
    #endif
    
    flushToken = false;
    if ((currentSize + newBytes) > MAX_DATA_BUFFER_SIZE){
        newBytes = MAX_DATA_BUFFER_SIZE - currentSize;
        flushToken = true;
        // Serial.printf("Buffer Full (%d bytes in buffer and adding %d bytes)\n\r", internalNetworkStackPtr->dataBuffer.size(), newBytes);
    }
    
    static int bytesReady;
    static int bytesProcessed; //DEBUG VARIABLE
    bytesReady = newBytes - (newBytes % FRAME_SIZE);
    internalNetworkStackPtr -> dataPlane -> readBytes(tmp, bytesReady);
    // for (int pos = 0; pos < bytesReady; pos += FRAME_SIZE){
    //     if (tmp[pos] != FRAME_START_SENTINEL){
    //         // Serial.printf("One byte dropped between %d bytes received and %d bytes received\n\r", bytesProcessed + pos - FRAME_SIZE, bytesProcessed + pos);
    //     }
    //     // Serial.printf("%u ", tmp[pos]);
    // }
    bytesProcessed += bytesReady;
    // Serial.println();
    unpackDataStream(tmp, bytesReady, internalNetworkStackPtr -> dataBuffer);

    if(flushToken){
        Serial.printf("Flushing the serial buffer...\n\r");
        flushSerialBuffer(internalNetworkStackPtr -> dataPlane);
    }

    #ifdef TIME_STREAMING
    if (internalNetworkStackPtr -> dataBuffer.size() >= DATA_STREAM_TEST_SIZE){ //DEBUG STATEMENT
        streamTime = millis() - streamTime;
    } 
    #endif

    internalNetworkStackPtr -> recordDataReceptionTime();

    // flushSerialBuffer(internalNetworkStackPtr -> dataPlane);
    // Serial.printf("Received %d bytes\n\r", newBytes);
    // Serial.printf("Received %d bytes (attempted to read %d vs. expectation of %d). There were %d bytes and now there's %d bytes in the queue (delta = %d). There are %d bytes left in the buffer.\n\r", newBytes, bytesReady,(internalNetworkStackPtr -> dataBuffer.size() - currentSize)/7*9, currentSize, internalNetworkStackPtr -> dataBuffer.size(), internalNetworkStackPtr -> dataBuffer.size() - currentSize, internalNetworkStackPtr -> dataPlane -> available()); //DEBUG STATEMENT
}