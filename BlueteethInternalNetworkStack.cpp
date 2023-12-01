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
    //Picked up noise, discard.
    if (receivedPacket.type >= RESERVED){ //RESERVED and above will never be used, so you can guarantee noise was picked up if this case is reached.
    }
    //If the token was received, transmit items in queue.
    else if (receivedPacket.tokenFlag == 1){
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
    static int bytesReady;
    static uint8_t tmp [DATA_PLANE_SERIAL_RX_BUFFER_SIZE]; 
    static const std::string accessIdentifier = "DATA PLANE";
    static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

    internalNetworkStackPtr -> networkAccessingResources = true;

    // if (internalNetworkStackPtr -> dataPlane -> available() >= DATA_PLANE_SERIAL_RX_BUFFER_SIZE ){
    //     Serial.println("The serial buffer is full");
    // }
    
    #ifdef TIME_STREAMING
    if (internalNetworkStackPtr -> dataBuffer.size() == 0) { 
        streamTime = millis();
    }
    #endif

    newBytes = internalNetworkStackPtr -> dataPlane -> available();
    currentSize = internalNetworkStackPtr -> dataBuffer.size();

    if (currentSize + (newBytes / FRAME_SIZE * PAYLOAD_SIZE) > MAX_DATA_BUFFER_SIZE){
        // Serial.printf("Reducing size. I'm expecting %d bytes and I have %d bytes.", newBytes, currentSize);
        newBytes = (MAX_DATA_BUFFER_SIZE - currentSize) / PAYLOAD_SIZE * FRAME_SIZE;
        // Serial.printf(" Now my expected size is %d bytes.", newBytes);
    }
    
    bytesReady = newBytes - (newBytes % FRAME_SIZE);

    // portEXIT_CRITICAL(&mutex); //exit critical to read data

    // if ((currentSize + newBytes) > MAX_DATA_BUFFER_SIZE){
    //     static bool token = false;
    //     if (token == false){
    //         Serial.printf("Forcing fast recovery.\n\r");
    //         if ((bytesReady - FRAME_SIZE) > 0){
    //             bytesReady -= FRAME_SIZE;
    //         }
    //     }
    //     token = true;
    // }

    internalNetworkStackPtr -> dataPlane -> readBytes(tmp, bytesReady);
    
    unpackDataStream(tmp, bytesReady, internalNetworkStackPtr -> dataBuffer, internalNetworkStackPtr -> dataPlaneMutex);

    // portEXIT_CRITICAL(&mutex); //exit critical to read data


    #ifdef TIME_STREAMING
    if (internalNetworkStackPtr -> dataBuffer.size() >= DATA_STREAM_TEST_SIZE){ //DEBUG STATEMENT
        streamTime = millis() - streamTime;
    } 
    #endif
    
    internalNetworkStackPtr -> recordDataBufferAccessTime();

    internalNetworkStackPtr -> networkAccessingResources = false;

}

void dataStreamErrorEncountered(hardwareSerial_error_t error){
    switch (error)
    {
        case UART_BUFFER_FULL_ERROR:
            // Serial.println("The data plane's serial buffer overflowed. Ressetting now...");
            internalNetworkStackPtr -> flushDataPlaneSerialBuffer();
            break;

        case UART_PARITY_ERROR:
            // Serial.println("Data stream corruption detected at the serial level...");
            break;

        case UART_NO_ERROR:
            break;

        case UART_BREAK_ERROR:
            break;

        case UART_FIFO_OVF_ERROR:
            break;

        case UART_FRAME_ERROR:
            break;


        // default:
        //     Serial.println("The data plane experienced an error that was unhandled.");
    }
}