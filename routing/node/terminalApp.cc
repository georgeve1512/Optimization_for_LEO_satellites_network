//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "terminalApp.h"
#include "Routing.h" // For enums

Define_Module(TerminalApp);

TerminalApp::~TerminalApp(){
    /*cancelAndDelete(appMsg);
    cancelAndDelete(updateMyPositionMsg);
    cancelAndDelete(updateSatellitePositionMsg);*/
    delete indexList;
}

void TerminalApp::initialize()
{
    // Initialize parameters
    myAddress = par("address").intValue();
    numOfTerminals = par("numOfTerminals").intValue();

    /* Create an index array of possible targets.
     * The array is {0,1,...,numOfTerminals} without [myAddress].
     * When choosing a destination we take a random integer (name it
     *      'r') in range [0,numOfTerminals). That's the index
     *      in the above array. The destination itself is indexList[r]
     * In code: setDestAddr(indexList[intuniform(0, numOfTerminals-1)])
     * */
    indexList = new int[numOfTerminals-1];
    for(int i = 0, j = 0; i < numOfTerminals; i++){
        if (i != myAddress){
            indexList[j] = i;
            j++;
        }
    }

    //// Regular App
    sendIATime = &par("sendIaTime");
    packetLengthBytes = &par("packetLength");

    //// Burst App - not in use 03.09.2020
    burstNextInterval = &par("burst_next_interval");
    burstNextEvent = &par("burst_next_event");
    burstSize = &par("burst_size");

    // Initialize statistics
    numSent = 0;                                           // Record with recordScalar()
    numReceived = 0;                                       // Record with recordScalar()
    endToEndDelaySignal = registerSignal("endToEndDelay"); // Record with emit()

    // Initialize helpers
    radius = getParentModule()->par("radius").doubleValue();
    myDispStr = &getParentModule()->getSubmodule("mobility")->getThisPtr()->getDisplayString();
    updateInterval = getParentModule()->getSubmodule("mobility")->par("updateInterval").doubleValue();
    updatePosition(myDispStr, myPosX, myPosY);

    // Initialize satellite connection
    // TODO: initial connection, move to function
    isConnected = false;


    appMsg = new cMessage("Inter Arrival Time Self Message");
    scheduleAt(simTime() + sendIATime->doubleValue(), appMsg);
}

void TerminalApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()){
        // Self Message - Handle inner functionality
        switch (msg->getKind()){
            case interArrivalTime:{
                //// Send data out

                if (isConnected){
                    // Terminal is connected to a satellite - create new message
                    TerminalMsg * terMsg = new TerminalMsg("Regular Message");
                    terMsg->setKind(terminal);
                    terMsg->setSrcAddr(myAddress);
                    terMsg->setDestAddr(indexList[intuniform(0, numOfTerminals-1)]);
                    terMsg->setPacketType(terminal_message);
                    terMsg->setByteLength(packetLengthBytes->intValue());
                    send(terMsg, "localOut");
                    numSent++;
                }
                else{
                    EV << "Terminal " << myAddress << " is not connected to any satellite, retrying to send app data later" << endl;
                }

                cancelEvent(appMsg);
                scheduleAt(simTime() + sendIATime->doubleValue(), appMsg);
                break;
            }
            case selfPositionUpdateTime:{
                //// Update self position & check satellite connection

                updatePosition(myDispStr, myPosX, myPosY);
                // TODO - Check connection

                cancelEvent(updateMyPositionMsg);
                scheduleAt(simTime() + updateInterval, updateMyPositionMsg);
                break;
            }
            case satellitePositionUpdateTime:{
                //// Update satellite position & check satellite connection

                updatePosition(satDispStr, satPosX, satPosY);
                // TODO - Check connection

                cancelEvent(updateSatellitePositionMsg);
                scheduleAt(simTime() + satUpdateInterval, updateMyPositionMsg);
                break;
            }
        }
    }
    else{
        // Message from outside - Receive data

        if (!msg->getKind()){
            // Received legacy packet - error
            delete msg;
            std::cout << "Error! Received legacy packet!" << endl;
            endSimulation();
        }

        TerminalMsg *terMsg = check_and_cast<TerminalMsg*>(msg);
        switch (terMsg->getPacketType()){
            case terminal_message:{
                //// Received message from other terminal

                // Record end-to-end delay
                simtime_t eed = simTime() - terMsg->getCreationTime();
                emit(endToEndDelaySignal, eed);
                numReceived++;

                delete msg;
                break;
            }
            case terminal_list:
            case terminal_connect:
            case terminal_disconnect:
            default:
                // Received terminal message, that supposed to be packet (for satellites only)- error
                delete msg;
                std::cout << "Error! Received satellite's packet in TerminalMsg Format!" << endl;
                endSimulation();
        }
    }
}

void TerminalApp::finish(){
    recordScalar("numSent",numSent);
    recordScalar("numReceived",numReceived);
}

void TerminalApp::updatePosition(cDisplayString *cDispStr, double &posX, double &posY){
    /* Find the (X,Y) position of the terminal (self). The position is extracted from the mobility module's display string (the one that
     *      changes during runtime).
     * */

    std::string dispStr(cDispStr->str());                   // Read display string as string, string format is t=p: (posX\, posY\, 0) m\n ...
    std::size_t pos = dispStr.find_first_of("0123456789");  // find position of the first number

    dispStr = dispStr.substr(pos);                          // Cut string until the first number
    posX = std::stod(dispStr, &pos);                        // Extract number as double. This is the X position. Also, update [pos] the the
                                                            //      location AFTER the first double

    dispStr = dispStr.substr(pos);                          // Remove the X position from string

    pos = dispStr.find_first_of("0123456789");              // find position of the first number
    dispStr = dispStr.substr(pos);                          // Cut string until the first number
    posY = std::stod(dispStr);                              // Extract number as double. This is the Y position
}

bool TerminalApp::canConnectToSatellite(int satAddress){
    /* Test whether terminal can connect to [satAddress] satellite.
     *      This function reads [satAddress] satellite's display string, parsing its position from it
     *      and calculates if the connection is up and returns the result.
     * Note: it changes the following variables:
     *       1. [satDispStr]
     *       2. [satPosX]
     *       3. [satPosY]
     *       If the terminal can connect [satAddress] and [satUpdateInterval] are changed too.
     * This function assumes that the terminal's position is up-to-date
     * */
    // TODO: Might change return type to double, and return the distance (To pick minimum distance)

    // Read display string
    satDispStr = &getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->getThisPtr()->getDisplayString();

    // Parse the position to [satPosX] & [satPosY]
    updatePosition(satDispStr, satPosX, satPosY);

    // Calculate result, update variables if can connect
    isConnected = checkConnection();
    if (isConnected){
        this->satAddress = satAddress;
        satUpdateInterval = getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->par("updateInterval").doubleValue();
    }

    return isConnected;
}

bool TerminalApp::checkConnection(){
    /* Assuming the positions are up-to-date. Return whether the satellite is within disk or not
     * */
    return radius >= sqrt(pow(myPosX-satPosX,2)+pow(myPosY-satPosY,2));
}

void TerminalApp::connectToSatellite(int satAddress){

}

void TerminalApp::disconnectFromSatellite(){

}
