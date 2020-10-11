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
#include "enums.h"
#include "TerminalMsg_m.h"

#include "inet/mobility/contract/IMobility.h"

using namespace omnetpp;


/**
 * Terminal's basic application.
 */

class TerminalApp : public cSimpleModule
{
    public:
         ~TerminalApp();

        // Auxiliary functions
        int getConnectedSatelliteAddress();
        void updatePosition(inet::IMobility* &mob, double &posX, double &posY);
        double getDistanceFromSatellite(int satAddress);
        int getClosestSatellite(int ignoreIndex);
        void getPos(double &posX, double &posY) {posX = myPosX; posY = myPosY;};
        double getRadius() const {return radius;};
    protected:
        // Simulation helpers
        cMessage *appMsg;                         // Self message for regular app
        cMessage *burstAppMsg;                    // Self message for burst app
        cMessage *pingAppMsg;                     // Self message for ping app
        cMessage *updateMyPositionMsg;            // Self message for annual position update (self)
        cMessage *updateMainSatellitePositionMsg; // Self message for annual position update (main satellite)
        cMessage *updateSubSatellitePositionMsg;  // Self message for annual position update (sub satellite)
        double rate;

        // Regular App helpers
        int *indexList;             // Index list for uniform choice of target terminal

        // Burst App helpers
        int *numOfBurstMsg;         // How many messages left from self to all other terminals

        // Simulation parameters
        int myAddress;              // UID
        int numOfTerminals;         // How many terminals are
        double radius;              // Coverage radius
        double thresholdRadius;     // Threshold to look for a sub satellite. Calculated by radius * frac
        int numOfSatellites;        // Number of satellites in simulation

        // Volatile Random Numbers - Expression is re-evaluated when reading value
        cPar *sendIATime;           // Next time a regular app message is sent
        cPar *packetLengthBytes;    // Length of a terminal message
        cPar *burstNextInterval;    // Next time the burst app is activated (it can send more than one message)
        cPar *burstSize;            // Number of messages in the flow
        cPar *pingTime;             // Next time a ping is sent

        // Variables that read self's position (For future use)
        inet::IMobility *myMob;     // Self's mobility module to extract (X,Y) position from
        double myPosX;              // Self's X coordinate, extracted in meters ([m])
        double myPosY;              // Self's Y coordinate, extracted in meters ([m])
        double updateInterval;      // Time to update position

        //// Connected satellite information
        int isConnected;            // Connection status, determines how many active satellite connections the terminal has (max 2)
                                    // Note that if [isConnected]==1, then the connection is main

        // Main Satellite
        int mainSatAddress;             // Connected main satellite's address
        inet::IMobility *mainSatMob;    // Main satellite's mobility module to extract (X,Y) position from
        double mainSatPosX;             // Main satellite's X coordinate
        double mainSatPosY;             // Main satellite's Y coordinate
        double mainSatUpdateInterval;   // Time between main satellite's position updates
        int mainConnectionIndex;        // Index inside main satellite's connection array, send in 0sec (simulation helper)

        // Sub Satellite
        int subSatAddress;             // Connected sub ssatellite's address
        inet::IMobility *subSatMob;    // Sub satellite's mobility module to extract (X,Y) position from
        double subSatPosX;             // Sub satellite's X coordinate
        double subSatPosY;             // Sub satellite's Y coordinate
        double subSatUpdateInterval;   // Time between sub satellite's position updates
        int subConnectionIndex;        // Index inside sub satellite's connection array, send in 0sec (simulation helper)

        // For statistics
        int numSent;                    // Number of sent messages to other terminals
        int numReceived;                // Number of received messages from other terminals. Destination increments this in source and records.
        cOutVector *latencyVector;      // End-to-end delay between each terminal and self
        cOutVector hopCountVector;      // Record hop count with time stamps
        cHistogram hopCountHistogram;   // Histogram of hops, allows to find hops PDF
        int64_t totalBytesSent;         // Helper for efficiency calculation.
        int64_t bytesSentSuccessfully;  // Helper for efficiency calculation. Destination adds number of received bytes in source.
        cOutVector efficiency;          // Calculation: bytesSentSuccessfully/totalBytesSent.
                                        // NOTE: numReceived/numSent is good when all messages are of the same size

        // Simulation basic functions
        virtual void initialize() override;
        virtual void handleMessage(cMessage *msg) override;
        virtual void finish() override;

        // Satellite connectivity
        bool checkConnection(int mode);
        void resetSatelliteData(int &satAddress, double &satPosX, double &satPoxY, double &satUpdateInterval, int &connectionIndex, cMessage* &updateMsg, inet::IMobility* &mob);

        // Sending connection messages
        int findSatelliteToConnect(int ignoreIndex, int mode);
        void connectToSatellite(int satAddress, int mode);
        void disconnectFromSatellite(int mode);

        // Auxiliary functions
        double getRemainingTime(double updateInterval, double epsilon=0.01);
        void upgradeSubToMain();
};

#endif
