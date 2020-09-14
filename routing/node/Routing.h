#include <map>
#include <omnetpp.h>
#include <string>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <iostream>
#include "L2Queue.h"
#include "TerminalMsg_m.h"
#include "terminalApp.h"
#include "enums.h"

#define MAXAMOUNTOFLINKS 6          //lll: max number of links availble

//lll: weight range. not in use
#define MAX_VALUE 10000
#define MIN_VALUE 0

#define PARAM 2                     //lll: does not appear in the code at all
#define ROOTNODE 0                  //lll: define the root node always with id 0
#define BROADCAST 255               //lll: Broadcast address : 255
#define DEBUG 1                     //lll: debug mode (printing logs on the fly)
#define MTUMESSAGE 1048576          //MTU defined as 1MBytes
#define HISTORY_LENGTH 876          // 35s per topology, 5 topologies, 5 samples per second => 5*5*35=875 + 1
#define ALPHA_P 300                 //alpha predict
#define BETA_P 7000                 //total errors predict
#define PATTERN_LENGTH 35           //7 seconds. 5 samples per second
//#define NUMOFNODES 25
#define LOAD_PREDICTION_INTERVAL 14  //search and predict the load every 5sec
#define BASELINE_MODE 0
using namespace omnetpp;
using namespace std;


class Routing: public cSimpleModule , public cListener
{
public:
    int leader = -1;
    int rootID = -1;
    int neighbors[MAXAMOUNTOFLINKS];
    ~Routing();
private:
    int mySatAddress;
    /*Flag to determine if a node has the updated topology. otherwise, it might send messages.
     *  through links that should not be exist.*/
    int hasInitTopo;
    int packetType;
    int currTopoID = 0;
    int rootID_current_topo_ID = 0;
    bool saveTempLinkFlag = false;
    bool keepAliveTempLinksFlag = false;
    int topocounter = 0;
    bool fiftLink = true;
    int packetCounter = 0;
    //lll: LAST variables predefined variable ( getParentModule()->par("variable"); )
    int alpha = 0;
    int m_var = 0;
    //lll: changing topology by predefined-time interval(getParentModule()->par("variable");
    int changeRate = 0;
    int firstNodeInfo = 0;
    int NodesInTopo = 0;
    int updateCounter = 0;
    bool fullTopoRun = true;
    //int num_of_hosts = getParentModule()->par("numofsatellite");
    int num_of_hosts = NUMOFNODES;

    int phase1Flag;
    int leader_table[NUMOFNODES];
    double leader_list[NUMOFNODES];

    double n_table[NUMOFNODES][NUMOFNODES];
    int rootCounter=0;
    int rootNodesArr[NUMOFNODES];
    int loadHistory[MAXAMOUNTOFLINKS][HISTORY_LENGTH];
    int loadTimerIndex;
    bool loadPredictionIsOn;
    bool loadPredictionFlag[MAXAMOUNTOFLINKS];
    int path[NUMOFNODES];
    int pathID;
    int pathWeight;
    int pathcounter;
    int tempActiveLinks [MAXAMOUNTOFLINKS];
    long queueSize[MAXAMOUNTOFLINKS];
    long frameCapacity;
    int packetLength;
    double rootNodesPathWeight[4];
    cPar *_sendTime;  // volatile parameter

    int load_threshold;// = (600*12000);//600 out of 666.66667
    double loadCounter[MAXAMOUNTOFLINKS] = {0};
    int active_links_array[MAXAMOUNTOFLINKS] = {0};

    bool load_balance_mode;
    bool load_balance_link_prediction;
    bool second_update = false;
    bool second_update_leader = false;
    bool app_is_on = false;
    double datarate;
    int active_links = 0;
//    // packetType 2 => Normal mode. These are regular packets generated from App.cc that will ride on the updated topology.
    Packet *_pk = NULL;
    int linkdatacounter = 0;
    typedef std::map<int, int> RoutingTable;  // destaddr -> gateindex
    RoutingTable rtable;
    int* linkCounter;
    
    typedef std::list<cTopology::LinkIn*> LinkIns;
    LinkIns edges;
    typedef std::list<cTopology::Node*> NodesList;
    NodesList Nodes;

    cTopology::Node *thisNode;
    cTopology *topo;
    cTopology *baseSPT;
    cTopology *newTopo;

    simsignal_t dropSignal;
    simsignal_t outputIfSignal;
    simsignal_t TopoLinkCounter;
    simsignal_t hopCountSignal;
    simsignal_t LBS;//load balance signal

    simtime_t delayTime;

    cModule *links_control[MAXAMOUNTOFLINKS];
    simtime_t queueschannel;

    cOutVector loadPrediction[4];

    //// Terminals
    RoutingTable myTerminalMap;       // Terminal address -> Index in terminal array [For terminals that are connected to self]
    RoutingTable neighborTerminalMap; // Terminal address -> Satellite address       [For terminals that are connected to a neighbor]
    int *isDirectPortTaken;           // Dynamic array of {0,1,2} to indicate if the port is taken or not (0 = not in use, 1 = main, 2 = sub).
                                      // For message collision avoidance in this module.
                                      // Size is calculated automatically from gateSize("terminalIn").
    int packetDropCounter = 0;        // Number of terminal packets lost (No match in any of the above tables)
    int maxHopCountForTerminalList;   // Decides when to drop terminal_list packets
    double estimatedLinkDelay = 1.2;  // Estimated average link delay, used for predictions
                                      // Estimated time = # of hops to target satellite * [estimatedLinkDelay]
                                      // TODO: implement estimatedTime calculation via ACKs. Right now 1.2 is used
                                      // [(radius/speed)/# of satellites=(1500/50)/25=1.2sec]

    // Helpers
    bool positionAtGrid = true;       // Whether the satellite starts at its position as calculated by grid sub module
                                      // Will become true after parsing the position from grid

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    //// Init. aux. functions
    cTopology* CreateTopology(cTopology *Origtopo, const char* topoName);
    void copyTopology(cTopology* OrigTopo, cTopology* NewTopo,const char* colour);
    void disableAllInLinks(cTopology *topo, bool debugging);
    void insertLinksToList(cTopology *topo);
    void setLinksWeight(cTopology *topo, double wgt);
    void LoadPacket(Packet *pk, cTopology *topo);
    void initRtable(cTopology* thisTopo, cTopology::Node *thisNode);
    void setDualSideWeights(cTopology *MyTopo, int Node1ID, double wgt);
    void setDualSideWeightsByQueueSize(cTopology *MyTopo, int Node1ID);

    //// Visualize/Debug
    int printEdgesList();
    void printNodesList();
    void PaintTopology(cTopology* OrigTopo, const char* colour);

    // Distributed Leader Selection
    void scheduleChoosingLeaderPhaseOne();
    void scheduleSendLinkInfo();
    void schduleNewTopology();
    void schduleTopologyUpdate();
    void send_phase1_update();
    void initial_leader_table();
    void update_leader_table(Packet *pk);
    void initial_leader_list();
    void setNeighborsIndex();
    int findDest(int direction,int index);
    void saveTempLinks();
    void keepAliveTempLinks();

    //// Interacting with leader
    void update_weight_table();
    void send_link_info_to_leader();
    int getNumOfLinks();
    void setPathToRoot(Packet *pk);

    // Queue functions
    void flushQueue(int type);

    //// LAST_Topology
    // MST:
    cTopology * primMST(cTopology *OrigTopo, int RootNodeInd);
    int minKey(int key[], bool mstSet[], int NodeCount);
    // Choosing roots:
    int getDiameter(cTopology *OrigTopo, int *srcIndex, int *dstIndex);
    int getInnerNodeFromDiameter(cTopology *OrigTopo, int dist, int src, int dst);
    void getRootsArray(cTopology *OrigTopo, int m);
    // LAST creation algorithm ("Balancing Minimum Spanning Trees and Shortest-Path Trees" by Khuller, Raghavachari & Young)
    cTopology* BuildLASTtopo(cTopology* origTopo,int alpha,int m);
    int* FindLASTForRoot(cTopology* origTopo,int alpha, int RootIndex, int* parent);
    cTopology* BuildSPT(cTopology* origTopo, int RootIndex);
    void DFS(int NodeIndex, int RootIndex, cTopology* origTopo, cTopology* MSTTopo, cTopology* SPTTopo, int alpha, int parent[],int distance[],int color[]);
    void RelaxTopo(int ChildNodeIndex, int ParentNodeIndex, cTopology* origTopo,int parent[], int distance[]);
    int GetW(cTopology::Node *nodeU, cTopology::Node *nodeV);
    void changeConnectingLink(cTopology::Node* currNode, int otherNodeInd,bool state);
    void AddPathToLAST(int NodeIndex, cTopology* origTopo, cTopology* SPTTopo,int parent[], int distance[]);
    cTopology* BuildTopoFromArray(int parent[], int nodesCount,cTopology* OrigTopo);
    cTopology* BuildTopoFromMultiArray(int** parentArrays, int ArrayCount,cTopology* OrigTopo);

    //// Load Balance
    int getLinkIndex(int req);
    int getNeighborIndex(int req);
    void setActiveLinks();
    void sendMessage(Packet *pk,int outGateIndex);
    int loadBalanceLinkFinder( int reqLink ,int incominglink,Packet *pk );
    int loadBalanceLinkPrediction(int reqLink,int incominglink);
    int loadBalance(int reqLink,int incominglink);
    void signalStringParser(const char *s,int* queueid,uint32_t* load,int* connectedto,int *queuesize);
    virtual void receiveSignal(cComponent *src, simsignal_t id, cObject *value,cObject *details)override;
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, const char *s, cObject *details) override;
    void updateLoadHistory();
    void link_weight_update();
    void reset_ntable();
    void LinkLoadPrediction();
    void updateLoadBalancePrediction(int index,int loadPredicted);
    void schduleBaseLine();
    int totalErrors(int *pattern,int *history,int offset,int size_P);
    void pi_func(int pattern[], int alpha,int size,int pi[]);
    double search(int *T,int *P,int alpha,int beta, int pattern_size, int History_size);
    int* getNeighborsArr(int index,int* arr);

    //// Terminals
    int calculateFutureSatellite(int destTerminal);
    Packet *createPacketForDestinationSatellite(TerminalMsg *terminalMessageToEncapsulate, bool usePrediction=true, int destSat = -1);
    void broadcastTerminalStatus(int terminalAddress, int status); // status = {connected/disconnected}
};

Define_Module(Routing);
