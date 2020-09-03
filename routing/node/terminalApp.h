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

using namespace omnetpp;
enum selfMessageTypes{
    interArrivalTime,
    selfPositionUpdateTime,
    satellitePositionUpdateTime
};


/**
 * TODO - Generated class
 */

class TerminalApp : public cSimpleModule
{
    public:
        ~TerminalApp();
        int getConnectedSatelliteAddress() {return isConnected ? satAddress : -1;};
    protected:
        // Simulation helpers
        double updateInterval;      // Time to update position
        cMessage *appMsg;           // Reduce memory allocations
        cMessage *updateMyPositionMsg; // Reduce memory allocations
        cMessage *updateSatellitePositionMsg; // Reduce memory allocations

        // Simulation parameters
        int myAddress;              // UID
        int numOfTerminals;         // How many terminals are
        double radius;              // Coverage radius
        int *indexList;             // For actual uniform choice between terminals (And not Geometric)

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

        // Connected satellite information
        bool isConnected;           // Connection status
        int satAddress;             // Connected satellite's address
        cDisplayString *satDispStr; // Satellite's display string to extract (X,Y) position
        double satPosX;             // Satellite's X coordinate
        double satPosY;             // Satellite's X coordinate
        double satUpdateInterval;   // Time between satellite's position updates

        // For statistics
        int numSent;
        int numReceived;
        simsignal_t endToEndDelaySignal;

        // Simulation basic functions
        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
        virtual void finish() override;

        // Auxiliary functions
        void updatePosition(cDisplayString *cDispStr, double &posX, double &posY);

        // Satellite connectivity
        bool canConnectToSatellite(int satAddress);
        bool checkConnection();

        // Sending connection messages
        void connectToSatellite(int satAddress);
        void disconnectFromSatellite();
};

#endif
