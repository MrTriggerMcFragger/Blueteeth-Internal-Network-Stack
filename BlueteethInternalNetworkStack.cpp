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


#define SENTINEL_CHAR (0b11111111) 

void inline packDataStream(uint8_t * packedData, int len, deque<uint8_t> & dataBuffer){

    uint8_t select_lower;

    size_t packagedDataEnd = len + (len/7)*2; //For each 7 bytes, 1 sentinal character byte and 8 0 bits are added.
    for(int frame = 0; frame < packagedDataEnd; frame += 9){
        select_lower = 0b00000001; //used to select the lower portion of the unpacked byte;
        
        packedData[frame] = SENTINEL_CHAR;
        packedData[frame + 1] = 0; //Need to set the first actual packaged byte to 0 for loop to work
        for(int byte = 1; byte < 8; byte++){
            packedData[frame + byte] += dataBuffer.front() >> byte;
            packedData[frame + byte + 1] = (select_lower & dataBuffer.front()) << (7 - byte); 
            dataBuffer.pop_front();

            select_lower = (select_lower << 1) + 1;
        }
    }

}

void inline unpackDataStream(uint8_t * packedData, int len, deque<uint8_t> & dataBuffer){
  uint8_t select_lower;
  uint8_t select_upper;

  int cnt = 0;
  //Circle through all of the data in the stream
  while (cnt < len){
    if (packedData[cnt++] == SENTINEL_CHAR){ //Don't begin unpacking until the sentinal character is found 
        
      select_upper = 0b01111111; //Used to select the upper portion of the unpacked byte  
      select_lower = 0b01000000; //used to select the lower portion of the unpacked byte;
        
      for(int i = 0; i < 7; i++){
          dataBuffer.push_back(
            ((packedData[cnt + i] & select_upper) << (i + 1)) + 
            ((packedData[cnt + i + 1] & select_lower) >> (6 - i))
          ); 
          select_upper = select_upper >> 1;
          select_lower += 1 << (5 - i);
        }
        cnt += 7;
    }
  }
}

void dataStreamReceived(){

    //Static to save time on allocation
    static uint8_t tmp [DATA_PLANE_SERIAL_RX_BUFFER_SIZE]; //Faster to temporarily read in bits with one readBytes function call than repeatedly read. Need to adapt readBytes for an iterator.


    int newBytes = internalNetworkStackPtr -> dataPlane -> available();
    int currentSize = internalNetworkStackPtr -> dataBuffer.size();
    
    Serial.print("Data received!");
    // Serial.printf("Buffer size is %d\n\r", internalNetworkStackPtr->dataBuffer.size());

    #ifdef TIME_STREAMING
    if (internalNetworkStackPtr -> dataBuffer.size() == 0) { //sometimes noise will be registered as a packet
        streamTime = millis();
    }
    #endif
    
    bool flushToken = false;
    if ((currentSize + newBytes) > MAX_DATA_BUFFER_SIZE){
        newBytes = MAX_DATA_BUFFER_SIZE - currentSize;
        flushToken = true;
        // Serial.printf("Buffer Full (%d bytes in buffer and adding %d bytes)\n\r", internalNetworkStackPtr->dataBuffer.size(), newBytes);
    }

    internalNetworkStackPtr -> dataPlane -> readBytes(tmp, newBytes);
    unpackDataStream(tmp, newBytes, internalNetworkStackPtr -> dataBuffer);

    if(flushToken){
        flushSerialBuffer(internalNetworkStackPtr -> dataPlane);
    }

    #ifdef TIME_STREAMING
    if (internalNetworkStackPtr -> dataBuffer.size() >= 40000){ //DEBUG STATEMENT
        streamTime = millis() - streamTime;
    } 
    #endif

    Serial.printf("Attempted to add %d bytes and now there are %d bytes in the deque (previously %d)\n\r", newBytes, internalNetworkStackPtr -> dataBuffer.size(), currentSize); //DEBUG STATEMENT
}