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

Define_Module(TerminalApp);
/************************************************************************************************************************************/
/*---------------------------------------PUBLIC FUNCTIONS---------------------------------------------------------------------------*/
/************************************************************************************************************************************/
TerminalApp::~TerminalApp(){
    /*cancelAndDelete(appMsg);
    cancelAndDelete(updateMyPositionMsg);
    cancelAndDelete(updateSatellitePositionMsg);*/
    delete indexList;
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

int TerminalApp::getConnectedSatelliteAddress(){
    /* Returns the address of the satellite connected to the terminal. If disconnected return -1.
     * */

    switch (isConnected){
        case 1:{
            // Only one is connected, that is the main
            return mainSatAddress;
        }
        case 2:{
            // Two are connected, return the sub satellite (More likely to be around when the packet is sent)
            return subSatAddress;
        }
        default:
            return -1;
    }
}

double TerminalApp::getDistanceFromSatellite(int satAddress){
    /* Get the distance from the terminal to [satAddress] satellite.
     *      This function reads [satAddress] satellite's display string, parsing its position from it
     *      and returns the distance from that satellite.
     * This function assumes that the terminal's position is up-to-date
     * */

    // Read display string
    cDisplayString *satDispStr = &getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->getThisPtr()->getDisplayString();
    double satPosX, satPosY;

    // Parse the position to [satPosX] & [satPosY]
    updatePosition(satDispStr, satPosX, satPosY);

    // Return distance from satellite
    return sqrt(pow(myPosX-satPosX,2)+pow(myPosY-satPosY,2));
}

/************************************************************************************************************************************/
/*---------------------------------------PRIVATE FUNCTIONS--------------------------------------------------------------------------*/
/************************************************************************************************************************************/
void TerminalApp::initialize()
{
    // Initialize message pointers
    appMsg = nullptr;
    updateMyPositionMsg = nullptr;
    updateMainSatellitePositionMsg = nullptr;
    updateSubSatellitePositionMsg = nullptr;

    // Initialize parameters
    myAddress = par("address").intValue();
    numOfTerminals = par("numOfTerminals").intValue();
    numOfSatellites = getParentModule()->getParentModule()->par("num_of_hosts").intValue();
    radius = getParentModule()->par("radius").doubleValue();

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

    // Initialize position
    myDispStr = &getParentModule()->getSubmodule("mobility")->getThisPtr()->getDisplayString();
    //updateInterval = getParentModule()->getSubmodule("mobility")->par("updateInterval").doubleValue();
    updatePosition(myDispStr, myPosX, myPosY);
    std::cout << "Terminal " << myAddress << " pos: (" << myPosX << ", " << myPosY << ")" << endl;

    // Initialize satellite database
    isConnected = 0;
    resetSatelliteData(mainSatAddress, mainSatDispStr, mainSatPosX, mainSatPosY, mainSatUpdateInterval, mainConnectionIndex);
    resetSatelliteData(subSatAddress, subSatDispStr, subSatPosX, subSatPosY, subSatUpdateInterval, subConnectionIndex);

    // Initialize satellite connection - connect to closest satellite as main
    // TODO: move after initialize...
    double minDistance = DBL_MAX;
    int satAddress = -1;
    bool foundMin;
    for(int i = 0; i < numOfSatellites; i++){
        double dist = getDistanceFromSatellite(i);
        if (dist <= radius && dist < minDistance){
            minDistance = dist;
            satAddress = i;
            foundMin = true;
        }
    }

    if (foundMin){
        std::cout << "Terminal " << myAddress << " can connect to " << satAddress << " dist=" << minDistance << endl;

        /*
        // Complete connection
        // TODO: Might want to combine everything here to [connectToSatellite]. Remember to check is message exists or not
        connectToSatellite(satAddress, main);
        updateMainSatellitePositionMsg = new cMessage("Main Satellite Position Update Time Message", mainSatellitePositionUpdateTime);
        scheduleAt(simTime() + mainSatUpdateInterval, updateMainSatellitePositionMsg);
        */
    }

    appMsg = new cMessage("Inter Arrival Time Self Message", interArrivalTime);
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

                    // Choose target satellite - if there are two give priority to sub
                    /* Note - sendDirect(cMessage *msg, simtime_t propagationDelay, simtime_t duration, cModule *mod, const char *gateName, int index=-1)
                     *        is being used here.
                     * TODO: set propagation/transmission duration. For transmission time R is required
                     * */
                    if (isConnected == 1)
                        sendDirect(terMsg, 0, 0,getParentModule()->getParentModule()->getSubmodule("rte", mainSatAddress),"terminalIn", mainConnectionIndex);
                    else
                        sendDirect(terMsg, 0, 0,getParentModule()->getParentModule()->getSubmodule("rte", subSatAddress),"terminalIn", subConnectionIndex);

                    // Update statistics
                    numSent++;
                }
                else{
                    EV << "Terminal " << myAddress << " is not connected to any satellite, retrying to send app data later" << endl;
                    if (hasGUI())
                        getParentModule()->bubble("Not connected!");
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
            case mainSatellitePositionUpdateTime:{
                //// Update main satellite position & check connection

                updatePosition(mainSatDispStr, mainSatPosX, mainSatPosY);
                // TODO - Check connection

                cancelEvent(updateMainSatellitePositionMsg);
                scheduleAt(simTime() + mainSatUpdateInterval, updateMainSatellitePositionMsg);
                break;
            }
            case subSatellitePositionUpdateTime:{
                //// Update sub satellite position & check connection

                updatePosition(subSatDispStr, subSatPosX, subSatPosY);
                // TODO - Check connection

                cancelEvent(updateSubSatellitePositionMsg);
                scheduleAt(simTime() + subSatUpdateInterval, updateSubSatellitePositionMsg);
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
            case terminal_index_assign:{
                //// Assign index from a satellite

                // TODO: Complete handshake
                if (terMsg->getSrcAddr() != -1){
                    //Satellite found an open gate for me

                    switch (terMsg->getDestAddr()){
                        case main:{
                            isConnected = 1;

                            if (!updateMainSatellitePositionMsg){
                                // First time connected to any satellite
                                updateMainSatellitePositionMsg = new cMessage("Main Satellite Position Update Self Message",mainSatellitePositionUpdateTime);
                            }

                            // TODO: get exact time for position update + 0.01 and schedule position update
                            // TODO: scheduleAt()

                            break;
                        }
                        case sub:{
                            isConnected = 2;

                            if (!updateSubSatellitePositionMsg){
                                // First time connected to any satellite
                                updateSubSatellitePositionMsg = new cMessage("Sub Satellite Position Update Self Message",subSatellitePositionUpdateTime);
                            }

                            // TODO: get exact time for position update + 0.01 and schedule position update
                            // TODO: scheduleAt()

                            break;
                        }
                    }
                }
                else{
                    // Satellite didn't have an open slot

                    // Delete satellite data
                    switch (terMsg->getDestAddr()){
                        case main:{
                            resetSatelliteData(mainSatAddress, mainSatDispStr, mainSatPosX, mainSatPosY, mainSatUpdateInterval, mainConnectionIndex);
                            break;
                        }
                        case sub:{
                            resetSatelliteData(subSatAddress, subSatDispStr, subSatPosX, subSatPosY, subSatUpdateInterval, subConnectionIndex);
                            break;
                        }
                    }
                }
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

bool TerminalApp::checkConnection(int mode){
    /* Assuming the positions are up-to-date. Return whether the [mode] satellite is within disk or not
     * */

    switch (mode){
        case main:{
            return radius >= sqrt(pow(myPosX-mainSatPosX,2)+pow(myPosY-mainSatPosY,2));
        }
        case sub:{
            return radius >= sqrt(pow(myPosX-subSatPosX,2)+pow(myPosY-subSatPosY,2));
        }
        default:
            // Removes warning
            return false;
    }
}

void TerminalApp::resetSatelliteData(int &satAddress, cDisplayString *satDispStr, double &satPosX, double &satPosY, double &satUpdateInterval, int &connectionIndex){
    satAddress = -1;
    satDispStr = nullptr;
    satPosX = -1;
    satPosY = -1;
    satUpdateInterval = -1;
    connectionIndex = -1;
}

void TerminalApp::connectToSatellite(int satAddress, int mode){
    /* Updates local database and send connection message to the satellite.
     * */

    switch (mode){
        case main:{
            mainSatAddress = satAddress;
            mainSatDispStr = &getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->getThisPtr()->getDisplayString();
            updatePosition(mainSatDispStr, mainSatPosX, mainSatPosY);
            mainSatUpdateInterval = getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->par("updateInterval").doubleValue();
            break;
        }
        case sub:{
            subSatAddress = satAddress;
            subSatDispStr = &getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->getThisPtr()->getDisplayString();
            updatePosition(subSatDispStr, subSatPosX, subSatPosY);
            subSatUpdateInterval = getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility")->par("updateInterval").doubleValue();
            break;
        }
    }

    // TODO: Connection handshake start - send terminal_connection to satellite
    // Kind = 1, destination = {main, sub} [mode],
}

void TerminalApp::disconnectFromSatellite(){
    /* Disconnect from a satellite, and clear related database. Also, if the disconnected satellite is main, copy sub satellite's
     *      data to main (if sub satellite exists)
     * Reduces [isConnected] by 1.
     * */

    // TODO: Disconnection process - send terminal_disconnection to satellite
    isConnected--;
}
