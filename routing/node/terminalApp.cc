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
/*---------------------------------------CODE FLOW IDEA-----------------------------------------------------------------------------*/
/************************************************************************************************************************************/
/* TerminalApp generates traffic (via self messages), and sends it to a satellite the terminal is connected to.
 * There are 2 different kinds of satellite - [main] and [sub]. The [main] satellite is the first satellite the terminal has connected
 *      to, and [sub] satellite is the second. When connected to sub satellite, the terminal would prioritize sending traffic to [sub]
 *      as it is more probable to stay around (Although it is possible for the [main] satellite to be the better option).
 * The self messages are of 3 different types:
 *      1. Create traffic (normal traffic for example)
 *      2. Update position - Keep track of the positions (=XY coordinates) of the 2 satellites and the terminal itself. Each update
 *                      leads to connection check (if satellites are still in disk).
 *      3. Initialize - INET's mobility module finishes initialization after this module, so self messages to finish
 *                      location initialization are necessary.
 *
 * The connection process (referred as 'handshake' in code) is as following:
 *      1. Connected satellites are chosen by minimum distance from terminal. [sub] satellites are chosen when [main] satellites
 *              are at distance of [thresholdRadius] from terminal.
 *      2. Send a [terminal_connect] message to a satellite. The satellite then begins looking for an open port to assign to terminal.
 *      3. The satellite returns [terminal_assign_index] message, which contains port index. If no port was found the assigned index is
*               -1, and every self message of traffic creation is ignored until a satellite is found.
 * The disconnection method is different - send [terminal_disconnect] message and the satellite simply removes terminal from registry.
 * */

/************************************************************************************************************************************/
/*---------------------------------------PUBLIC FUNCTIONS---------------------------------------------------------------------------*/
/************************************************************************************************************************************/
//TerminalApp::~TerminalApp(){
//
//}

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

int TerminalApp::getClosestSatellite(int ignoreIndex){
    /* Iterate over all satellites while ignoring the satellite index given in [ignoreIndex].
     * Returns the index of the closest satellite, or -1 if none are inside radius
     * */

    // Find closest satellite
    double minDistance = DBL_MAX;
    int satAddress = -1;
    for(int i = 0; i < numOfSatellites; i++){           // Iterate over all satellites
        if (i != ignoreIndex){                          // Ignore [ignoreIndex] satellite
            double dist = getDistanceFromSatellite(i);  // Calculate distance to satellite i

            // Save shortest distance (of a satellite inside the disk) and satellite index
            if (dist <= radius && dist < minDistance){
                minDistance = dist;
                satAddress = i;
            }
        }
    }

    return satAddress;
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
    rate =  check_and_cast<cDatarateChannel*>(getParentModule()->getParentModule()->getSubmodule("rte", 0)->getSubmodule("queue", 0)->gate("line$o")->getTransmissionChannel())->getDatarate();
    thresholdRadius = radius * getParentModule()->par("frac").doubleValue();

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
    updateInterval = 0; //getParentModule()->getSubmodule("mobility")->par("updateInterval").doubleValue();
    scheduleAt(simTime(), new cMessage("Initialize self position", initializeMyPosition));

    // Initialize satellite database
    isConnected = 0;
    resetSatelliteData(mainSatAddress, mainSatDispStr, mainSatPosX, mainSatPosY, mainSatUpdateInterval, mainConnectionIndex, updateMainSatellitePositionMsg);
    resetSatelliteData(subSatAddress, subSatDispStr, subSatPosX, subSatPosY, subSatUpdateInterval, subConnectionIndex, updateSubSatellitePositionMsg);

    // Initialize satellite connection - connect to closest satellite as main
    scheduleAt(simTime(), new cMessage("Initialize connection", initializeConnection));

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
                    TerminalMsg *terMsg = new TerminalMsg("Regular Message", terminal);
                    terMsg->setSrcAddr(myAddress);

                    int dst = intuniform(0, numOfTerminals-1);
                    while (dst == myAddress){
                        dst = intuniform(0, numOfTerminals-1);
                    }
                    terMsg->setDestAddr(dst);
                    terMsg->setPacketType(terminal_message);
                    terMsg->setByteLength(packetLengthBytes->intValue());

                    // Choose target satellite - if there are two give priority to sub
                    /* Note - sendDirect(cMessage *msg, simtime_t propagationDelay, simtime_t duration, cModule *mod, const char *gateName, int index=-1)
                     *        is being used here.
                     * */
                    if (isConnected == 1)
                        sendDirect(terMsg, 0, terMsg->getByteLength()/rate,getParentModule()->getParentModule()->getSubmodule("rte", mainSatAddress),"terminalIn", mainConnectionIndex);
                    else
                        sendDirect(terMsg, 0, terMsg->getByteLength()/rate,getParentModule()->getParentModule()->getSubmodule("rte", subSatAddress),"terminalIn", subConnectionIndex);

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
                // TODO - Check connection, implement after finishing static terminal simulation

                cancelEvent(updateMyPositionMsg);
                scheduleAt(simTime() + updateInterval, updateMyPositionMsg);
                break;
            }
            case mainSatellitePositionUpdateTime:{
                //// Update main satellite position & check connection

                updatePosition(mainSatDispStr, mainSatPosX, mainSatPosY);
                if (checkConnection(main)){
                    // Satellite is still around the terminal

                    if (isConnected == 1){ // If no sub exists
                        // Check if distance is greater than threshold
                        if (getDistanceFromSatellite(mainSatAddress) >= thresholdRadius){

                            // The main satellite is outside of threshold - look for a new satellite to connect to
                            EV << "Terminal " << myAddress << " main satellite is outside threshold radius, looking for sub satellite" << endl;
                            findSatelliteToConnect(mainSatAddress, sub);
                        }
                    }
                }
                else{
                    // Terminal is too far away and is no longer connected

                    if (isConnected == 1){
                        // No sub - just disconnect
                        EV << "Main is out of range" << endl;
                        disconnectFromSatellite(main);
                    }
                    else{
                        // There is a sub satellite - it promotes to main
                        EV << "Main is out of range, upgrade sub to main" << endl;
                        disconnectFromSatellite(main);
                        upgradeSubToMain();

                        // Calculate exact time for position update + 0.01 and schedule position update
                        double remainingTime = getRemainingTime(mainSatUpdateInterval);
                        scheduleAt(simTime() + remainingTime, updateMainSatellitePositionMsg);

                        EV << "Terminal " << myAddress << " upgrade complete for satellite " << mainSatAddress << endl;
                        return;
                    }
                }

                // Send self message only if still connected
                if (updateMainSatellitePositionMsg){
                    cancelEvent(updateMainSatellitePositionMsg);
                    scheduleAt(simTime() + mainSatUpdateInterval, updateMainSatellitePositionMsg);
                }
                break;
            }
            case subSatellitePositionUpdateTime:{
                //// Update sub satellite position & check connection

                updatePosition(subSatDispStr, subSatPosX, subSatPosY);
                if (!checkConnection(sub)){
                    // Sub satellite is too far away - disconnect
                    disconnectFromSatellite(sub);
                }

                if (updateSubSatellitePositionMsg){
                    cancelEvent(updateSubSatellitePositionMsg);
                    scheduleAt(simTime() + subSatUpdateInterval, updateSubSatellitePositionMsg);
                }
                break;
            }
            case initializeMyPosition:{
                // Message to initialize original position of the terminal
                if (updateInterval){
                    updateMyPositionMsg = new cMessage("Update my position message", selfPositionUpdateTime);
                    scheduleAt(simTime() + updateInterval, updateMyPositionMsg);
                }
                updatePosition(myDispStr, myPosX, myPosY);

                delete msg;
                break;
            }
            case initializeConnection:{
                // Try to connect to a satellite after initialization

                findSatelliteToConnect(-1, main);

                delete msg;
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
                if (hasGUI()){
                    getParentModule()->bubble("Received message!");
                }

                // Record end-to-end delay
                simtime_t eed = simTime() - terMsg->getCreationTime();
                emit(endToEndDelaySignal, eed);
                numReceived++;

                break;
            }
            case terminal_index_assign:{
                //// Assign index from a satellite

                if (terMsg->getReplyType() != -1){
                    //Satellite found an open gate for me

                    EV << "Satellite " << terMsg->getSrcAddr() << " accepted terminal " << myAddress << endl;

                    switch (terMsg->getMode()){
                        case main:{
                            isConnected = 1;

                            mainConnectionIndex = terMsg->getReplyType();
                            if (!updateMainSatellitePositionMsg){
                                // First time connected to any satellite
                                updateMainSatellitePositionMsg = new cMessage("Main Satellite Position Update Self Message",mainSatellitePositionUpdateTime);
                            }

                            // Calculate exact time for position update + 0.01 and schedule position update
                            double remainingTime = getRemainingTime(mainSatUpdateInterval);
                            scheduleAt(simTime() + remainingTime, updateMainSatellitePositionMsg);

                            break;
                        }
                        case sub:{
                            isConnected = 2;

                            subConnectionIndex = terMsg->getReplyType();
                            if (!updateSubSatellitePositionMsg){
                                // First time connected to any satellite
                                updateSubSatellitePositionMsg = new cMessage("Sub Satellite Position Update Self Message",subSatellitePositionUpdateTime);
                            }

                            // Calculate exact time for position update + 0.01 and schedule position update
                            double remainingTime = getRemainingTime(subSatUpdateInterval);
                            scheduleAt(simTime() + remainingTime, updateSubSatellitePositionMsg);

                            break;
                        }
                    }
                }
                else{
                    // Satellite didn't have an open slot

                    EV << "Satellite " << terMsg->getSrcAddr() << " rejected terminal " << myAddress << endl;

                    // Delete satellite data
                    switch (terMsg->getDestAddr()){
                        case main:{
                            resetSatelliteData(mainSatAddress, mainSatDispStr, mainSatPosX, mainSatPosY, mainSatUpdateInterval, mainConnectionIndex, updateMainSatellitePositionMsg);
                            break;
                        }
                        case sub:{
                            resetSatelliteData(subSatAddress, subSatDispStr, subSatPosX, subSatPosY, subSatUpdateInterval, subConnectionIndex, updateSubSatellitePositionMsg);
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

        delete msg;
    }
}

void TerminalApp::finish(){
    recordScalar("numSent",numSent);
    recordScalar("numReceived",numReceived);
    EV_INFO << "Terminal " << myAddress << ":" << endl;
    EV_INFO << "Received " << numReceived << " messages" << endl;
    EV_INFO << "Sent " << numSent << " messages" << endl;
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

void TerminalApp::resetSatelliteData(int &satAddress, cDisplayString *&satDispStr, double &satPosX, double &satPosY, double &satUpdateInterval, int &connectionIndex, cMessage *&updateMsg){
    /* Places default values inside given satellite database
     * */

    satAddress = -1;
    satDispStr = nullptr;
    satPosX = -1;
    satPosY = -1;
    satUpdateInterval = -1;
    connectionIndex = -1;
    if (updateMsg)
        delete updateMsg;
    updateMsg = nullptr;
}

void TerminalApp::findSatelliteToConnect(int ignoreIndex, int mode){
    /* This function tries to connect the closest satellite to terminal, ignoring [ignoreIndex]
     * */

    int satAddress = getClosestSatellite(ignoreIndex);

    if (satAddress != -1){
        // Start handshake with satellite
        connectToSatellite(satAddress, mode);
    }
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

    // Ask to connect
    TerminalMsg *terMsg = new TerminalMsg("Connection Request", terminal);
    terMsg->setSrcAddr(myAddress);
    terMsg->setDestAddr(satAddress);
    terMsg->setPacketType(terminal_connect);
    terMsg->setMode(mode);
    terMsg->setByteLength(64); // Ethernet minimum

    cModule *satellite = getParentModule()->getParentModule()->getSubmodule("rte", satAddress);
    sendDirect(terMsg, 0, terMsg->getByteLength()/rate, satellite->gate("terminalIn", 0));

    EV << "Terminal " << myAddress << " attempted connection with satellite " << satAddress << endl;
}

void TerminalApp::disconnectFromSatellite(int mode){
    /* Clears local database and send disconnection message to the satellite.
     * */

    int satAddress, gateIndex;
    switch (mode){
        case main:{
            satAddress = mainSatAddress;
            gateIndex = mainConnectionIndex;
            cancelEvent(updateMainSatellitePositionMsg);
            resetSatelliteData(mainSatAddress, mainSatDispStr, mainSatPosX, mainSatPosY, mainSatUpdateInterval, mainConnectionIndex, updateMainSatellitePositionMsg);
            break;
        }
        case sub:{
            satAddress = subSatAddress;
            gateIndex = subConnectionIndex;
            cancelEvent(updateSubSatellitePositionMsg);
            resetSatelliteData(subSatAddress, subSatDispStr, subSatPosX, subSatPosY, subSatUpdateInterval, subConnectionIndex, updateSubSatellitePositionMsg);
            break;
        }
    }

    // Send disconnection message
    TerminalMsg *terMsg = new TerminalMsg("Disconnection Request", terminal);
    terMsg->setSrcAddr(myAddress);
    terMsg->setDestAddr(satAddress);
    terMsg->setPacketType(terminal_disconnect);
    terMsg->setMode(mode);
    terMsg->setByteLength(0);

    cModule *satellite = getParentModule()->getParentModule()->getSubmodule("rte", satAddress);
    sendDirect(terMsg, satellite->gate("terminalIn", gateIndex));

    EV << "Terminal " << myAddress << " attempted disconnection from satellite " << satAddress << endl;

    isConnected--;
}

double TerminalApp::getRemainingTime(double updateInterval, double epsilon){
    /* Calculate how much time is needed to wait until next update, and add [epsilon] to it.
     * Calculation is taking the fraction part of (current simulation time) / [updateInterval], and subtract it from [updateInterval]:
     *      remaining time = updateInterval - frac(simTime()/updateInterval)
     * To this time we add [epsilon] in order to get small delay
     *  */

    double currSimTime = simTime().dbl();
    return updateInterval + epsilon - updateInterval * (currSimTime/updateInterval - int(currSimTime/updateInterval));
}

void TerminalApp::upgradeSubToMain(void){
    /* Replace main satellite's database + message with sub satellite's database
     * */

    // Copy data from sub satellite to main
    mainSatAddress = subSatAddress;
    mainSatDispStr = subSatDispStr;
    mainSatPosX = subSatPosX;
    mainSatPosY = subSatPosY;
    mainSatUpdateInterval = subSatUpdateInterval;
    mainConnectionIndex = subConnectionIndex;

    // Move message from sub to main by canceling old and replacing it
    if (updateMainSatellitePositionMsg){
        cancelEvent(updateMainSatellitePositionMsg);
        updateMainSatellitePositionMsg = nullptr;
    }
    cancelEvent(updateSubSatellitePositionMsg);
    updateMainSatellitePositionMsg = updateSubSatellitePositionMsg;
    updateSubSatellitePositionMsg = nullptr;

    // Clear sub satellite data
    resetSatelliteData(subSatAddress, subSatDispStr, subSatPosX, subSatPosY, subSatUpdateInterval, subConnectionIndex, updateSubSatellitePositionMsg);
}
