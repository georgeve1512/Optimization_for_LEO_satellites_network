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

#ifndef __ROUTING_TERMINALAPP_H_
#define __ROUTING_TERMINALAPP_H_

#include <omnetpp.h>
#include "Routing.h" // For enums

using namespace omnetpp;
enum selfMessageTypes{
    interArrivalTime,
    selfPositionUpdateTime,
    mainSatellitePositionUpdateTime,
    subSatellitePositionUpdateTime
};


/**
 * Terminal's basic application. 07.09.2020 - Only sends UDP data to a random terminal
 */

class TerminalApp : public cSimpleModule
{
    public:
        ~TerminalApp();

        // Auxiliary functions
        int getConnectedSatelliteAddress();
        void updatePosition(cDisplayString *cDispStr, double &posX, double &posY);
        double getDistanceFromSatellite(int satAddress);
    protected:
        // Simulation helpers
        cMessage *appMsg;                         // Reduce memory allocations
        cMessage *updateMyPositionMsg;            // Reduce memory allocations
        cMessage *updateMainSatellitePositionMsg; // Reduce memory allocations
        cMessage *updateSubSatellitePositionMsg;  // Reduce memory allocations

        // Simulation parameters
        int myAddress;              // UID
        int numOfTerminals;         // How many terminals are
        double radius;              // Coverage radius
        double thresholdRadius;     // Threshold to look for a sub satellite.
                                    // TODO - is [radius] - 2*(12000/R)*[c=3*10^8 m/s] a good threshold?
        int *indexList;             // For actual uniform choice between terminals (And not Geometric)
        int numOfSatellites;        // Number of satellites in simulation

        // Volatile Random Numbers - Expression is re-evaluated when reading value
        cPar *sendIATime;
        cPar *packetLengthBytes;
        cPar *burstNextInterval;
        cPar *burstNextEvent;
        cPar *burstSize;

        // Variables that read self's position (For future use)
        cDisplayString *myDispStr;  // Self's display string to extract (X,Y) position
        double myPosX;              // Self's X coordinate, extracted in meters ([m])
        double myPosY;              // Self's Y coordinate, extracted in meters ([m])
        double updateInterval;      // Time to update position

        //// Connected satellite information
        int isConnected;            // Connection status, determines how many active satellite connections the terminal has (max 2)
                                    // Note that if [isConnected]==1, then the connection is main

        // Main Satellite
        int mainSatAddress;             // Connected main satellite's address
        cDisplayString *mainSatDispStr; // Main satellite's display string to extract (X,Y) position
        double mainSatPosX;             // Main satellite's X coordinate
        double mainSatPosY;             // Main satellite's Y coordinate
        double mainSatUpdateInterval;   // Time between main satellite's position updates
        int mainConnectionIndex;        // Index inside main satellite's connection array, send in 0sec (simulation helper)

        // Sub Satellite
        int subSatAddress;             // Connected sub ssatellite's address
        cDisplayString *subSatDispStr; // Sub satellite's display string to extract (X,Y) position
        double subSatPosX;             // Sub satellite's X coordinate
        double subSatPosY;             // Sub satellite's Y coordinate
        double subSatUpdateInterval;   // Time between sub satellite's position updates
        int subConnectionIndex;        // Index inside sub satellite's connection array, send in 0sec (simulation helper)

        // For statistics
        int numSent;
        int numReceived;
        simsignal_t endToEndDelaySignal;

        // Simulation basic functions
        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
        virtual void finish() override;

        // Satellite connectivity
        bool checkConnection(int mode);
        void resetSatelliteData(int &satAddress, cDisplayString *satDispStr, double &satPosX, double &satPoxY, double &satUpdateInterval, int &connectionIndex);

        // Sending connection messages
        void connectToSatellite(int satAddress, int mode);
        void disconnectFromSatellite();
};

#endif
