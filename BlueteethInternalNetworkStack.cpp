#include <BlueteethInternalNetworkStack.h>

extern BlueteethBaseStack * internalNetworkStack;

/* Grab contents from serial buffer and package them into a blueteeth packet
*
*  @serial - The serial port being read from
*  @return - The blueteeth packet received from the buffer
*/
blueTeethPacket retrievePacketFromBuffer(HardwareSerial * serial){

    //Have to temporarily grab the data form the UART buffer into a byte buffer (so it can be packaged in the packet struct)
    uint8_t buffer [sizeof(blueTeethPacket)];
    serial -> readBytes(buffer, sizeof(buffer));
    
    //Package the packet struct with data from buffer
    blueTeethPacket retrievedPacket;
    retrievedPacket.tokenFlag = buffer[0];
    retrievedPacket.srcAddr = buffer[1];
    retrievedPacket.dstAddr = buffer[2];
    retrievedPacket.type = buffer[3];
    for (int i = 4; i < sizeof(blueTeethPacket); i++){
        retrievedPacket.payload[i-4] = buffer[i];
    }
    return retrievedPacket;
}

void packetReceived(){
    
    //Don't try to package packet unless there's enough data on the buffer to do so
    if(internalNetworkStack -> controlPlane -> available() > sizeof(blueTeethPacket)){  
        
        blueTeethPacket receivedPacket = retrievePacketFromBuffer(internalNetworkStack -> controlPlane);

        //Assess the packet
        //If the token was received, transmit items in queue.
        if (receivedPacket.tokenFlag == 1){
            while(uxQueueMessagesWaiting(internalNetworkStack -> transmitPacketBuffer) > 0){
                blueTeethPacket transmitPacket;
                xQueueReceive(internalNetworkStack -> transmitPacketBuffer, &transmitPacket, 0);
                internalNetworkStack -> transmitPacket(transmitPacket);
            }

            blueTeethPacket tokenPacket;
            tokenPacket.tokenFlag = 1;
            internalNetworkStack -> transmitPacket(tokenPacket);
        }
        //If instead the packet was meant for this device, add it to the buffer
        else if (receivedPacket.dstAddr == internalNetworkStack -> address){
            xQueueSend(internalNetworkStack -> receivedPacketBuffer, &receivedPacket, 0);
        }
        else if (receivedPacket.dstAddr == 255){ //Packets with destination address 255 are intended to intialize node addresses
            switch(internalNetworkStack -> address){ 
                case 0: //0 is the master (who sends the intialization packet). Once the packet makes it all the way back to the sender, the initilaization phase is done (so add the packet to the number of slaves can be inferred)
                    xQueueSend(internalNetworkStack -> receivedPacketBuffer, &receivedPacket, 0);
                    break; 
                case 255: 
                    internalNetworkStack -> address = receivedPacket.payload[0];
                    receivedPacket.payload[0]++; //increment the address value and pass it on
                default: //Default response if a node has been assigned an address is to pass on the intializaton packet
                    internalNetworkStack -> transmitPacket(receivedPacket);
                    break;

            }
        }
        //If the packet was not meant for this device at all, send it on to the next device in the ring
        else if (receivedPacket.dstAddr != internalNetworkStack -> address){
            internalNetworkStack -> transmitPacket(receivedPacket);
        }
    }

}

void blueTeethNetworkSetup(){
    
    internalNetworkStack -> controlPlane -> onReceive(packetReceived);

}
