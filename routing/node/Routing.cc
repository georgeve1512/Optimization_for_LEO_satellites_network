//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Copyright (C) 1992-2015 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include "Routing.h"

void Routing::finish(){
    if (packetCounter){
        recordScalar("Prediction miss ratio", packetDropCounter/(double)packetCounter);
        recordScalar("Load Balancing loss ratio", packetDropCounterFromHops/(double)packetCounter);
    }
    if (totalMsgNum){
        recordScalar("Average delay on all links", getAverageLinkDelay());
    }
}

Routing::~Routing(void){
    delete isDirectPortTaken;
    delete topo;
    if (mySatAddress)
        delete baseSPT;
    delete linkCounter;
}

//This function is not working correctly. The problem seems to be with the method getNode(int i)
void Routing::insertLinksToList(cTopology *topo) {
    int numInLinks = 0;

    for (int i = 0; i < topo->getNumNodes(); i++) {
        numInLinks = 0;
        numInLinks = topo->getNode(i)->getNumInLinks();
        for (int j = 0; j < numInLinks; j++) {
            edges.push_back(topo->getNode(i)->getLinkIn(j)); //lll: what with links out ???
        }
    }
}

/*
 * Args: None
 * Return: Void
 * Description: Prints out the source and destination of each LinkIn and its weight.
 */
int Routing::printEdgesList() {
    std::list<cTopology::LinkIn*>::iterator it;
    int source_node = 0;
    int dest_node = 0;
    int enabledCounter = 0;
    double wgt = 0;

    for (it = edges.begin(); it != edges.end(); it++) {
        source_node = (*it)->getRemoteNode()->getModule()->par("address");
        dest_node = (*it)->getLocalNode()->getModule()->par("address");
        wgt = (*it)->getWeight();
        if ((*it)->isEnabled()) {
            enabledCounter += 1;
        }
        if (DEBUG) {
            EV << "[+] printEdgesList:: " << "Source Node: " << source_node
                      << " Dest Node: " << dest_node << " Weight: " << wgt
                      << " Enabled:" << (*it)->isEnabled() << endl;
        }
    }
    return enabledCounter;
}

/*
 * Pass it pk = new cPacket();
 */
void Routing::LoadPacket(Packet *pk, cTopology *topo) {
    // Create a new topology, extracted from Ned types (e.g. Net60)
    cTopology *tempTopo = new cTopology("LoadPacket topo");
    std::vector<std::string> nedTypes;
    nedTypes.push_back(getParentModule()->getNedTypeName());
    tempTopo->extractByNedTypeName(nedTypes);

    // Copy topology to send via packet (topo) to a new instance (tempTopo)
    copyTopology(tempTopo, topo, "green");

    // Packets for dispatching the topology will be identified as INIT packets
    pk->setTopologyVar(tempTopo);
}

void Routing::printNodesList() {
    std::list<cTopology::Node*>::iterator it;
    int temp;

    for (it = Nodes.begin(); it != Nodes.end(); it++) {
        //temp = (*it)->getModule()->par("address");
        temp = (*it)->getModule()->par("address").intValue();

        EV << "[+] printNodesList:: " << temp << endl;
    }
}

/*
 * Args: cTopology variable
 * Return: Void
 * Description: Iterates through each LinkIn in each Node and disable it. For 'calculateWeightedSingleShortestPathsTo', when
 * a LinkIn is disabled, it is "not connected". By doing that, we achieve a graph G'={}, although, all vertices and all
 * edges are still there. Later on, each edge that will be added, will be simply turned back with "enable()" method.
 */
void Routing::disableAllInLinks(cTopology *topo, bool debugging) {
    int numInLinks = 0,numInOut = 0;

    for (int i = 0; i < topo->getNumNodes(); i++) {
        numInLinks = 0;
        numInLinks = topo->getNode(i)->getNumInLinks();
        for (int j = 0; j < numInLinks; j++)
        {
            if (debugging)topo->getNode(i)->getLinkIn(j)->setWeight(-1);
            else
            {
                topo->getNode(i)->getLinkIn(j)->disable();
                cDisplayString& dispStr =
                        topo->getNode(i)->getLinkIn(j)->getRemoteGate()->getDisplayString();
                dispStr.setTagArg("ls", 0, "red");
                dispStr.setTagArg("ls", 1, "0");
            }
        }
        numInOut = 0;
        numInOut = topo->getNode(i)->getNumOutLinks();
        for (int j = 0; j < numInOut; j++)
        {
            if (debugging)topo->getNode(i)->getLinkOut(j)->setWeight(-1);
            else
            {
                topo->getNode(i)->getLinkOut(j)->disable();
                cDisplayString& dispStr =
                        topo->getNode(i)->getLinkOut(j)->getRemoteGate()->getDisplayString();
                dispStr.setTagArg("ls", 0, "red");
                dispStr.setTagArg("ls", 1, "0");
            }
        }
    }
}

void Routing::setDualSideWeights(cTopology *MyTopo, int Node1ID, double wgt)
{
    int Node2ID = 0;
    int numOfLinks, numInLinks, N2numInLinks,N2numOutLinks;
    numInLinks = MyTopo->getNode(Node1ID)->getNumInLinks();
    cTopology::Node* tempNode = MyTopo->getNode(Node1ID);
    cTopology::Node* N2tempNode;

    for (int j = 0; j < numInLinks; j++)
    {
        Node2ID = MyTopo->getNode(Node1ID)->getLinkIn(j)->getRemoteNode()->getModule()->getIndex();
        if (Node2ID >= 0)
        {
            if (MyTopo->getNode(Node1ID)->getLinkIn(j)->getWeight() == -1)
                MyTopo->getNode(Node1ID)->getLinkIn(j)->setWeight(wgt);
        }
        N2tempNode = MyTopo->getNode(Node2ID);
        N2numInLinks = N2tempNode->getNumInLinks();
        for (int v = 0; v < N2numInLinks; v++) {
            if (N2tempNode->getLinkIn(v)->getRemoteNode()->getModule()->getIndex() == Node1ID
                    && MyTopo->getNode(Node2ID)->getLinkIn(v)->getWeight() == -1)
                MyTopo->getNode(Node2ID)->getLinkIn(v)->setWeight(wgt);
        }
    }
    int numOutLinks = MyTopo->getNode(Node1ID)->getNumOutLinks();
    for (int j = 0; j < numOutLinks; j++)
    {
        Node2ID = MyTopo->getNode(Node1ID)->getLinkOut(j)->getRemoteNode()->getModule()->getIndex();
        if (Node2ID >= 0)
        {
            if (MyTopo->getNode(Node1ID)->getLinkOut(j)->getWeight() == -1)
                MyTopo->getNode(Node1ID)->getLinkOut(j)->setWeight(wgt);
        }
        N2tempNode = MyTopo->getNode(Node2ID);
        N2numOutLinks = N2tempNode->getNumOutLinks();
        for (int v = 0; v < N2numOutLinks; v++) {
            if (N2tempNode->getLinkOut(v)->getRemoteNode()->getModule()->getIndex() == Node1ID
                    && MyTopo->getNode(Node2ID)->getLinkOut(v)->getWeight() == -1)
                MyTopo->getNode(Node2ID)->getLinkOut(v)->setWeight(wgt);
        }
    }
}

void Routing::setDualSideWeightsByQueueSize(cTopology *MyTopo, int Node1ID) {
      int Node2ID = 0;
      int numOfLinks, numInLinks, N2numInLinks,N2numOutLinks;
      numInLinks = MyTopo->getNode(Node1ID)->getNumInLinks();
      cTopology::Node* tempNode = MyTopo->getNode(Node1ID);
      cTopology::Node* N2tempNode;
      double maxwgt;
      for (int j = 0; j < numInLinks; j++)
      {
          //Node2ID the Node1ID connected to with link j
          Node2ID = MyTopo->getNode(Node1ID)->getLinkIn(j)->getRemoteNode()->getModule()->getIndex();

          maxwgt = (n_table[Node1ID][Node2ID] >= n_table[Node2ID][Node1ID]) ?  n_table[Node1ID][Node2ID] : n_table[Node2ID][Node1ID];
          MyTopo->getNode(Node1ID)->getLinkIn(j) -> setWeight(maxwgt);
          int tempNeighbor[6];
          int linkindex = -1;
          int *arr = getNeighborsArr(Node2ID,tempNeighbor);
          int Neighbor = tempNeighbor[getNeighborIndex(j)];
          for(int k=0;k<6;k++)
              if(tempNeighbor[k] == Node1ID)
                  linkindex = getLinkIndex(k);
          if(linkindex == -1)
              exit(1321);
          MyTopo->getNode(Node2ID)->getLinkIn(linkindex) -> setWeight(maxwgt);
      }
}

/*
 * Args: cTopology variable, weight
 * Return: Void
 * Description: Set weights to LinkIn objects (edges), since this is the actual object that counts when computing
 * 'calculateWeightedSingleShortestPathsTo' (Disjktra Algorithm)
 */
void Routing::setLinksWeight(cTopology *MyTopo, double wgt) {
    int numInLinks = 0, Node2ID, Node1ID, numOfLinks;
    cTopology::Node* tempNode;
    cTopology::Node* temp;
    srand(time(NULL));
    int index=0;

    switch ((int) wgt) {
    case 0: // random

        for (int i = 0; i < MyTopo->getNumNodes(); i++) {
            wgt = truncnormal(50,20);//intuniform(80, 100, 0);//((int)simTime().dbl()*getSimulation()->getEventNumber())%117);

            Node1ID = i;
            setDualSideWeights(MyTopo, Node1ID, wgt);

        }
        break;

        //lll we change here to received link info from all topos.

    case 10:
        //lll: set the received values of each link
        for(int i=0;i<MyTopo->getNumNodes();i++)
        {
            setDualSideWeightsByQueueSize(MyTopo, i);
//            wgt = truncnormal(50,20);
//            setDualSideWeights(MyTopo, i, wgt);
        }

       break;
    case 1: //all ones - distributed
        for (int i = 0; i < MyTopo->getNumNodes(); i++) {
            numInLinks = 0;
            numInLinks = MyTopo->getNode(i)->getNumInLinks();
            for (int j = 0; j < numInLinks; j++) {
                MyTopo->getNode(i)->getLinkIn(j)->setWeight(1);
            }
        }break;

    case 2: //predefined const values - for debugging

        for (int i = 0; i < MyTopo->getNumNodes(); i++) {
            numInLinks = 0;
            numInLinks = MyTopo->getNode(i)->getNumInLinks();
            Node1ID = i;
            for (int j = 0; j < numInLinks; j++) {
                Node2ID =
                        MyTopo->getNode(i)->getLinkIn(j)->getRemoteNode()->getModule()->getIndex();
                if (Node2ID >= 0) {
                    tempNode = MyTopo->getNode(Node1ID);
                }
                wgt = i * j;
                if (wgt == 0)
                    wgt = 1;
                MyTopo->getNode(i)->getLinkIn(j)->setWeight(wgt);
            }
        }
        break;

    case 8:

        for (int i = 0; i < topo->getNumNodes(); i++) {
            numInLinks = 0;
            numInLinks = topo->getNode(i)->getNumInLinks();

            Node1ID = i;
            for (int j = 0; j < numInLinks; j++) {
                Node2ID =
                        topo->getNode(i)->getLinkIn(j)->getRemoteNode()->getModule()->getIndex();
                switch (Node1ID) {
                case 0:
                    switch (Node2ID) {
                    case 1:
                        wgt = 10;
                        break;
                    case 3:
                        wgt = 15;
                        break;
                    case 4:
                        wgt = 15;
                        break;
                    case 5:
                        wgt = 15;
                        break;
                    case 7:
                        wgt = 5;
                        break;
                    }
                    break;
                case 1:
                    if (Node2ID == 2)
                        wgt = 10;
                    break;
                case 2:
                    if (Node2ID == 3)
                        wgt = 10;
                    break;
                case 3:
                    if (Node2ID == 4)
                        wgt = 10;
                    else if (Node2ID == 5)
                        wgt = 5;
                    break;
                case 5:
                    if (Node2ID == 6)
                        wgt = 10;
                    break;
                case 6:
                    if (Node2ID == 7)
                        wgt = 10;
                    break;
                }
                if (Node2ID >= 0) {

                    tempNode = topo->getNode(Node1ID);
                    numOfLinks = tempNode->getNumInLinks();
                    for (int v = 0; v < numOfLinks; v++) {
                        if (tempNode->getLinkIn(v)->getRemoteNode()->getModule()->getIndex()
                                == Node2ID
                                && topo->getNode(Node1ID)->getLinkIn(v)->getWeight()
                                        == -1)
                            topo->getNode(Node1ID)->getLinkIn(v)->setWeight(
                                    wgt);
                    }
                    tempNode = topo->getNode(Node2ID);
                    numOfLinks = tempNode->getNumInLinks();

                    for (int v = 0; v < numOfLinks; v++) {
                        if (tempNode->getLinkIn(v)->getRemoteNode()->getModule()->getIndex()
                                == Node1ID
                                && topo->getNode(Node2ID)->getLinkIn(v)->getWeight()
                                        == -1)
                            topo->getNode(Node2ID)->getLinkIn(v)->setWeight(
                                    wgt);
                    }
                }
            }
        }
        break;

    default:

        for (int i = 0; i < topo->getNumNodes(); i++) {
            numInLinks = 0;
            numInLinks = topo->getNode(i)->getNumInLinks();
            for (int j = 0; j < numInLinks; j++) {
                topo->getNode(i)->getLinkIn(j)->setWeight(wgt);
            }
        }
        break;
    }

}

// A utility function to find the vertex with
// minimum key value, from the set of vertices
// not yet included in MST
int Routing::minKey(int key[], bool mstSet[], int NodeCount) {
    // Initialize min value
    int min = INT_MAX, min_index;

    for (int v = 0; v < NodeCount; v++)
        if (mstSet[v] == false && key[v] < min){
            min = key[v];
            min_index = v;
        }

    return min_index;
}

// Function to construct and print MST for
// a graph represented using adjacency
// matrix representation
cTopology * Routing::primMST(cTopology *OrigTopo, int RootNodeInd) {
    // Array to store constructed MST

    int nodesCount = OrigTopo->getNumNodes();

    int parent[nodesCount];

    // Key values used to pick minimum weight edge in cut
    int key[nodesCount];
    // To represent set of vertices not yet included in MST
    bool MstSet[nodesCount];
    int tempNodeNieghbor;
    cTopology::Node* tempNode;

    int numOfLinks, edgeWeight;

    // Initialize all keys as INFINITE
    for (int i = 0; i < nodesCount; i++) {
        MstSet[i] = false;
        key[i] = INT_MAX;
        parent[i] = -2;
    }
    // Always include first 1st vertex in MST.
    // Make key 0 so that this vertex is picked as first vertex.

    //lll: key[RootNodeInd] = 0 - need to change when going distributed
    key[RootNodeInd] = RootNodeInd;//0;
    parent[RootNodeInd] = -1; // First node is always root of MST
    int u;
    for (int count = 0; count < nodesCount; count++) {

        u = minKey(key, MstSet, nodesCount);

        tempNode = OrigTopo->getNode(u);
        MstSet[u] = true;
        numOfLinks = tempNode->getNumInLinks();

        for (int v = 0; v < numOfLinks; v++) {
            tempNodeNieghbor =
                    tempNode->getLinkIn(v)->getRemoteNode()->getModule()->getIndex();
            edgeWeight = (int) tempNode->getLinkIn(v)->getWeight();
            if (MstSet[tempNodeNieghbor] == false
                    && edgeWeight < key[tempNodeNieghbor]) {
                parent[tempNodeNieghbor] = u;
                key[tempNodeNieghbor] = edgeWeight;
            }
        }
    }
    cTopology *MSTtopology = BuildTopoFromArray(parent, nodesCount, OrigTopo);

    MSTtopology->calculateWeightedSingleShortestPathsTo(
            MSTtopology->getNode(RootNodeInd));

    return MSTtopology;
}

/*
 * void Routing::copyTopology(cTopology* OrigTopo,cTopology* NewTopo)
 * a function to copy 1 topology to the other
 * the function receives 2 Topologies - The Original topology and the New topology.
 * the function changes disable all of the links in the original topology and afterwards
 * copy the links enabled in the new topology to the Original, in order to display the topology
 */
void Routing::copyTopology(cTopology* OrigTopo, cTopology* NewTopo,
        const char* colour) {
    //disable all the links in the original topology
    disableAllInLinks(OrigTopo, false);

    int nodesCount = OrigTopo->getNumNodes();
    cTopology::Node *tempOrigNode, *tempNewNode;
    int edgeCount;
    double edgeWeight;
    //run over all of the nodes in the topology
    for (int i = 0; i < nodesCount; i++) {
        //get the nodes from both topologies
        tempOrigNode = OrigTopo->getNode(i);
        tempNewNode = NewTopo->getNode(i);
        edgeCount = tempOrigNode->getNumInLinks();

        //run over all of the links of the node
        for (int u = 0; u < edgeCount; u++)
        {
            //if the link is enabled in the new topology, update the original topology and paint the connection
            if (tempNewNode->getLinkIn(u)->isEnabled())
            {
                tempOrigNode->getLinkIn(u)->enable();
                edgeWeight = tempNewNode->getLinkIn(u)->getWeight();
                tempOrigNode->getLinkIn(u)->setWeight(edgeWeight);
             //   if (DEBUG) {EV << "[+] copyTopology - Adding link From Node "  << tempOrigNode->getModule()->getIndex()<< " To Node "<< tempOrigNode->getLinkIn(u)->getRemoteNode()->getModule()->getIndex()<< endl;}
            }
        }
        edgeCount = tempOrigNode->getNumOutLinks();
        for (int u = 0; u < edgeCount; u++)
        {
            //if the link is enabled in the new topology, update the original topology and paint the connection
            if (tempNewNode->getLinkOut(u)->isEnabled())
            {
                tempOrigNode->getLinkOut(u)->enable();
                edgeWeight = tempNewNode->getLinkOut(u)->getWeight();
                tempOrigNode->getLinkOut(u)->setWeight(edgeWeight);
              //  if (DEBUG) {EV << "[+] copyTopology - Adding link From Node "  << tempOrigNode->getModule()->getIndex()<< " To Node "<< tempOrigNode->getLinkIn(u)->getRemoteNode()->getModule()->getIndex()<< endl;}
            }
        }
    }
//    if (DEBUG) {
//        EV << "[+] copyTopology - Done!" << endl;
//    }

    PaintTopology(topo, colour);
//    if (hasGUI())
//        getParentModule()->bubble("Topology Updated!");
}

void Routing::setActiveLinks()
{
    int countActiveLinks = 0;
    for(int i=0;i< topo->getNode(mySatAddress)->getNumInLinks();i++)
    {
        active_links_array[i] = -1;
        if(topo->getNode(mySatAddress)->getLinkIn(i)->isEnabled())
        {
            countActiveLinks++;
            active_links_array[i] = neighbors[getNeighborIndex(i)];
        }
    }
    active_links = countActiveLinks;
}

cTopology* Routing::CreateTopology(cTopology *Origtopo, const char* topoName) {
    cTopology *NewTopology = new cTopology(topoName);
    std::vector<std::string> nedTypes;

    nedTypes.push_back(getParentModule()->getNedTypeName());
    NewTopology->extractByNedTypeName(nedTypes);

    copyTopology(NewTopology, Origtopo, "green");
    return NewTopology;
}

void Routing::PaintTopology(cTopology* OrigTopo, const char* colour) {
    int nodesCount = OrigTopo->getNumNodes();
    cTopology::Node *tempOrigNode;
    int edgeCount;
    //run over all of the nodes in the topology
    for (int i = 0; i < nodesCount; i++) {
        //get the nodes from both topologies
        tempOrigNode = OrigTopo->getNode(i);
        edgeCount = tempOrigNode->getNumInLinks();

        //run over all of the links of the node
        for (int u = 0; u < edgeCount; u++) {
            //if the link is enabled in the new topology, update the original topology and paint the connection
            if (tempOrigNode->getLinkIn(u)->isEnabled()) {
                cDisplayString& dspString =(cDisplayString&) tempOrigNode->getLinkIn(u)->getRemoteGate()->getDisplayString();
                dspString.setTagArg("ls", 0, colour);
                dspString.setTagArg("ls", 1, 4);
            }
        }
    }
}

void Routing::initRtable(cTopology* thisTopo, cTopology::Node *thisNode) {
    // find and store next hops
    for (int i = 0; i < thisTopo->getNumNodes(); i++) {
        if (thisTopo->getNode(i) == thisNode)
            continue;  // skip ourselves
        thisTopo->calculateWeightedSingleShortestPathsTo(thisTopo->getNode(i));

        if (thisNode->getNumPaths() == 0)
            continue;  // not connected

        cGate *parentModuleGate = thisNode->getPath(leader)->getLocalGate();
        int gateIndex = parentModuleGate->getIndex();
        int address = thisTopo->getNode(i)->getModule()->par("address");
        rtable[address] = gateIndex;
        if (DEBUG) {
            EV << "  towards address " << address << " gateIndex is "
                      << gateIndex << endl;
        }
    }
}

int Routing::GetW(cTopology::Node *nodeU, cTopology::Node *nodeV) {

    int numOfLinks = nodeV->getNumInLinks();
    cTopology::LinkIn* checkedLink;
    for (int i = 0; i < numOfLinks; i++) {
        checkedLink = nodeV->getLinkIn(i);
        if (checkedLink->getRemoteNode()->getModule()->getIndex()
                == nodeU->getModule()->getIndex() && checkedLink->isEnabled()) {
            return checkedLink->getWeight();
        }
    }
    return 0;
}

cTopology* Routing::BuildSPT(cTopology* origTopo, int RootIndex) {
    if (DEBUG) {
        EV << "[+] BuildSPT- calling CreateTopology " << endl;
    }
    cTopology* SPTtopo = CreateTopology(origTopo,"SPT topology");
    SPTtopo->calculateWeightedSingleShortestPathsTo(SPTtopo->getNode(RootIndex));
    return SPTtopo;

}

void Routing::RelaxTopo(int ChildNodeIndex, int ParentNodeIndex,
        cTopology* origTopo, int parent[], int distance[])
{
    int LinkWeight = GetW(origTopo->getNode(ChildNodeIndex),origTopo->getNode(ParentNodeIndex));
    int compareWeight = distance[ParentNodeIndex] + LinkWeight;

    if (distance[ChildNodeIndex] > compareWeight)
    {
        distance[ChildNodeIndex] = compareWeight;
        parent[ChildNodeIndex] = ParentNodeIndex;
    }
}

void Routing::AddPathToLAST(int NodeIndex, cTopology* origTopo,
        cTopology* SPTTopo, int parent[], int distance[]) {
    if (distance[NodeIndex] > (int) SPTTopo->getNode(NodeIndex)->getDistanceToTarget()) {
        cTopology::LinkOut* tempLink = SPTTopo->getNode(NodeIndex)->cTopology::Node::getPath(leader);
        if (tempLink != NULL) {
            int ParentNodeIndex = tempLink->getRemoteNode()->getModule()->getIndex();
            AddPathToLAST(ParentNodeIndex, origTopo, SPTTopo, parent, distance);
            RelaxTopo(ParentNodeIndex, NodeIndex ,origTopo, parent, distance);
        }
    }
}

void Routing::DFS(int NodeIndex, int RootIndex, cTopology* origTopo,
    cTopology* MSTTopo, cTopology* SPTTopo, int alpha, int parent[],
    int distance[],int color[])
{
    if(color[NodeIndex] == 2)//2 - black
        return;


    int ParentNodeIndex, ChildNodeIndex;
    int multiplySPT = alpha * SPTTopo->getNode(NodeIndex)->getDistanceToTarget();
    if (distance[NodeIndex] > multiplySPT) {
        AddPathToLAST(NodeIndex, origTopo, SPTTopo, parent, distance);
    }
    cTopology::Node* MSTNode = MSTTopo->getNode(NodeIndex);
    int NumOfLinks = MSTNode->getNumInLinks();
    if (NodeIndex == RootIndex)
        ParentNodeIndex = -1;
    else
        ParentNodeIndex = MSTNode->cTopology::Node::getPath(leader)->getRemoteNode()->getModule()->getIndex();
    color[NodeIndex] = 1;//1 - grey
    for (int i = 0; i < NumOfLinks; i++)
    {
        if (MSTNode->getLinkIn(i)->isEnabled())
        {
            ChildNodeIndex = MSTNode->getLinkIn(i)->getRemoteNode()->getModule()->getIndex();
            if(color[ChildNodeIndex] != 0)
                continue;
            if (ChildNodeIndex != ParentNodeIndex)
            {
                RelaxTopo(ChildNodeIndex, NodeIndex, origTopo, parent,distance);
                DFS(ChildNodeIndex, RootIndex, origTopo, MSTTopo, SPTTopo, alpha, parent, distance,color);
                RelaxTopo(NodeIndex, ChildNodeIndex, origTopo, parent, distance);
            }
        }
    }
    color[NodeIndex] = 2;//2 black
}

cTopology* Routing::BuildTopoFromArray(int parent[], int nodesCount,
        cTopology* OrigTopo) {
    if (DEBUG) {
        EV << "[+] BuildTopoFromArray- calling CreateTopology " << endl;
    }
    cTopology* newTopo = CreateTopology(OrigTopo,"BuildTopoFromArray" );
    cTopology::Node* tempNode;
    disableAllInLinks(newTopo, false);

    int Node1ID, Node2ID;
    for (int count = 0; count < nodesCount; count++) {
        Node1ID = count;
        Node2ID = parent[count];
        if (Node2ID >= 0) {
            tempNode = newTopo->getNode(Node1ID);
            changeConnectingLink(tempNode, Node2ID, true);

            tempNode = newTopo->getNode(Node2ID);
            changeConnectingLink(tempNode, Node1ID, true);
        }
    }
    return newTopo;
}

cTopology* Routing::BuildTopoFromMultiArray(int** parentArrays, int ArrayCount,cTopology* OrigTopo) {
    if (DEBUG){
         EV << "[+] BuildTopoFromMultiArray- calling CreateTopology " << endl;
     }
     cTopology* newTopoFromArr = CreateTopology(OrigTopo, "LAST topology");
     cTopology::Node* tempNode;
     disableAllInLinks(newTopoFromArr, false);
     int *parent;
     int Node1ID, Node2ID, nodesCount = OrigTopo->getNumNodes();
     for (int arrCtr= 0; arrCtr <ArrayCount; arrCtr++)
     {
         parent = parentArrays[arrCtr];
         for (int count = 0; count < nodesCount; count++)
         {
             Node1ID = count;
             Node2ID = parent[count];
             if (Node2ID >= 0){
                 tempNode = newTopoFromArr->getNode(Node1ID);
                 changeConnectingLink(tempNode, Node2ID, true);

                 tempNode = newTopoFromArr->getNode(Node2ID);
                 changeConnectingLink(tempNode, Node1ID, true);

             }
         }
     }
     return newTopoFromArr;
}

int* Routing::FindLASTForRoot(cTopology* origTopo,int alpha, int RootIndex,int* parent)
{
    cTopology* MSTTopo = primMST(origTopo, RootIndex);
    cTopology* SPTTopo = BuildSPT(origTopo, RootIndex);

    int nodesCount = origTopo->getNumNodes();
    int color[num_of_hosts];
        for(int j=0;j<num_of_hosts;j++)color[j]=0;//while.1-grey.2-black

    // Key values used to pick minimum weight edge in cut
    int distance[nodesCount];

    // Initialize all keys as INFINITE
    for (int i = 0; i < nodesCount; i++){
        distance[i] = INT_MAX;
        parent[i] = -2;
    }
    parent[RootIndex] = 0;
    distance[RootIndex] = 0;

    DFS(RootIndex,RootIndex, origTopo, MSTTopo, SPTTopo, alpha, parent, distance,color);

    delete SPTTopo;
    delete MSTTopo;

    return parent;
}

void Routing::changeConnectingLink(cTopology::Node* currNode, int otherNodeInd,
        bool state) {
    int numOfLinks = currNode->getNumInLinks();

    for (int v = 0; v < numOfLinks; v++)
    {
        if (currNode->getLinkIn(v)->getRemoteNode()->getModule()->getIndex() == otherNodeInd)
        {
            if (state == true) {
                currNode->getLinkIn(v)->enable();
                currNode->getLinkOut(v)->enable();
            } else {
                currNode->getLinkIn(v)->disable();
                currNode->getLinkOut(v)->disable();
            }
        }
    }
}

//llllast
cTopology* Routing::BuildLASTtopo(cTopology* origTopo,int alpha,int m)
{
   cTopology* MSTTopo = primMST(origTopo, mySatAddress);

   getRootsArray(MSTTopo, m);

   int* LASTparentArr[num_of_hosts/m];

   for(int i=0;i<rootCounter;i++)
   {
       int* parent = new int[num_of_hosts];
       LASTparentArr[i] = FindLASTForRoot(origTopo, alpha, rootNodesArr[i], parent);
   }
   cTopology* completeTopo = BuildTopoFromMultiArray(LASTparentArr, rootCounter, origTopo);

   delete MSTTopo;
   return completeTopo;
}

//for future use (link 5)
int Routing:: getNumOfLinks(){
    return 4;
}

//lll: triggering distributed leader selection packet
void Routing::scheduleChoosingLeaderPhaseOne() {
    Packet *phase1_initiate = new Packet("Selecting Leader Phase 1 update!");
    phase1_initiate->setSrcAddr(mySatAddress);
    phase1_initiate->setDestAddr(mySatAddress);
    phase1_initiate->setPacketType(phase1_update);
    phase1_initiate->setTopologyID(currTopoID);
    phase1_initiate -> setByteLength(1);
    scheduleAt(simTime(), phase1_initiate);
}

//lll:Send choosing leader phase 1 update message
void Routing::send_phase1_update(){
    int rootGateIndex;

 //   flushQueue(phase1_update);//delete previous updates
        for (int i = 0; i < topo->getNode(mySatAddress)->getNumInLinks(); i++) {
            if (neighbors[i] != -1)
            {

                //creating update packet
                Packet *phase1_update_message = new Packet( "Selecting Leader Phase 1 update!");
                phase1_update_message->setSrcAddr(mySatAddress);
                phase1_update_message->setPacketType(phase1_update);
                phase1_update_message->setDestAddr(neighbors[i]);
                phase1_update_message->setTopologyID(currTopoID);
                phase1_update_message -> setByteLength(64);

                // leader_table is updated, the update message is the leader_table data
                for (int j = 0; j < num_of_hosts; j++)
                    phase1_update_message->data[j] = leader_table[j];

                //find the gate to node Neighbors
                RoutingTable::iterator iter = rtable.find(neighbors[i]);
                rootGateIndex = (*iter).second;
               //delayTime = exponential( 1, intrand(6));
            //    delayTime = (simtime_t) _sendTime->doubleValue(); // one time read
              //  delayTime = (double)_sendTime->read();
                if(phase1Flag)

                delayTime = 0;//exponential(0.01);
                sendDelayed(phase1_update_message,delayTime,"out",rootGateIndex);
            }// end if neighbors[i] != -1
        }//end for
        if(hasGUI())
            getParentModule()->bubble("sending phase1 updates");
}// end send_phase1_update()

//initialize leader table
void Routing::initial_leader_table() {
    //learn from the last update
    for (int i = 0; i < num_of_hosts; i++)
            leader_table[i] = -1;
}

//lll: Update the Node leader_table and check's if leader can be chosen. given id's are non negative numbers. the tables holds num_of_hosts id's
void Routing::update_leader_table(Packet *pk) {
    //lll: for net100 :  run from 0 to 99
     int min = 101;

    //learn from the last update
    for (int i = 0; i < num_of_hosts; i++)
    {
        if (pk->data[i] != -1)
            leader_table[i] = pk->data[i];

        //check min - leader
        if ( leader_table[i] <= min )
                  min = leader_table[i];
    }
    //leader has been found and he will initiate the next phase
    if (min != -1 && min == mySatAddress){
        phase1Flag = 0;
        leader = min;
        initial_leader_list();
        scheduleSendLinkInfo();
    } else if( min != -1 ){
        phase1Flag = 0;
        //send_phase1_update();
    }
}

void Routing::reset_ntable()
{
    for(int i = 0; i < num_of_hosts; i++)
        for(int j= 0; j<num_of_hosts;j++)
            n_table[i][j]=-0.9;
}

//Only the leader run this function. asks for links information to buid LAST topology
void Routing::scheduleSendLinkInfo() {

    flushQueue(phase1_update);
    int rootGateIndex;
    update_weight_table();
    for( int i = 0; i < MAXAMOUNTOFLINKS; i++ )
        if(neighbors[i] != -1)
            n_table[leader][neighbors[i]] = (double)queueSize[getLinkIndex(i)] / frameCapacity * 100;//(datarate/12000) * 100;

    for(int i = 0; i < num_of_hosts; i++){
        if(i != mySatAddress){
            Packet *phase2_initiate = new Packet("Send link info to Leader!");
             phase2_initiate->setSrcAddr(mySatAddress);
             phase2_initiate->setDestAddr(i);
             phase2_initiate->setPacketType(phase2_link_info);
             phase2_initiate->setTopologyID(currTopoID);
             phase2_initiate -> setByteLength(1500);

             //find the gate to node Neighbors
             RoutingTable::iterator iter = rtable.find(i);
             rootGateIndex = (*iter).second;
             delayTime = 0;//exponential(0.01);
             sendDelayed(phase2_initiate,delayTime,"out",rootGateIndex);
        }
    }

}

//initialize leader list
void Routing::initial_leader_list() {
    //learn from the last update
    for (int i = 0; i < num_of_hosts; i++)
    {
            leader_list[i]  =  -1 ;
    }
}

//only chosen leader use this function to initiate the third phase - distribute the topology
void Routing::schduleNewTopology() {
    int rootGateIndex;
    flushQueue(phase1_update);
    Routing::disableAllInLinks(topo, true); // set G'= {}
    //we random the weights on the first run and then the link weight is determinant in  update_topology routine
    if (!second_update_leader )
    {
        setLinksWeight(topo,0); // Set random weights to edges in the graph
    }
    else
    {
        setLinksWeight(topo, 10);
    }
    cTopology* LASTtopo = BuildLASTtopo(topo,alpha,m_var);
//    EV<<endl<<endl<<"rootNodesArr[0] = "<< rootNodesArr[0] <<endl<<endl;
//    EV<<endl<<endl<<"rootNodesArr[1] = "<< rootNodesArr[1] <<endl<<endl;
//    EV<<endl<<endl<<"rootNodesArr[2] = "<< rootNodesArr[2] <<endl<<endl;
//    EV<<endl<<endl<<"rootNodesArr[3] = "<< rootNodesArr[3] <<endl<<endl;

// first topology
    if(!second_update_leader )
    {
        copyTopology(topo, LASTtopo, "yellow");
        PaintTopology(topo, "green");

    }
    else
    {
        copyTopology(topo, LASTtopo, "green");
        PaintTopology(topo, "green");
        second_update_leader = true;
    }

    LASTtopo->clear();
    delete LASTtopo;

    currTopoID = rand()% 100000 + 1;
    thisNode = topo->getNodeFor(getParentModule());
    initRtable(topo, thisNode);
    insertLinksToList(topo);
    int enabledCtr = printEdgesList();
    emit(TopoLinkCounter,enabledCtr);
    edges.clear();

    for(int i = 0; i < rootCounter; i++)
    {
        if(rootNodesArr[i] != -1)
        {
            Packet *phase4_initiate = new Packet("Roots will broadcast paths!!");
            phase4_initiate -> setSrcAddr(mySatAddress);
            phase4_initiate -> setDestAddr(rootNodesArr[i]);
            phase4_initiate -> setPacketType(leader_to_roots);
            phase4_initiate -> setTopologyID(currTopoID);
            phase4_initiate -> setHopCount(0);
            phase4_initiate -> setByteLength(1500);
            //set how many roots in data[0]
            phase4_initiate -> data[0] = rootCounter;
            //set the roots id's in data[1 to amountOfLeaders + 1] given searching less than num_of_hosts - 1 roots
            for( int j = 0; j < rootCounter + 1 ; j++ )
                phase4_initiate->data[ j + 1 ] = rootNodesArr[j];
            for( int j = rootCounter + 1; j < num_of_hosts; j++ )
                phase4_initiate->data[j] = -1;
            Routing::LoadPacket(phase4_initiate, topo);


            //find the gate to node Neighbors
           RoutingTable::iterator iter = rtable.find(rootNodesArr[i]);
           rootGateIndex = (*iter).second;
           send(phase4_initiate, "out",rootGateIndex);
        }
    }
    schduleTopologyUpdate();

}

void Routing::schduleTopologyUpdate()
{
    Packet *topoUpdate = new Packet("topo update!!");
    topoUpdate->setSrcAddr(mySatAddress);
    topoUpdate->setDestAddr(mySatAddress);
    topoUpdate  -> setPacketType(update_topology);
    topoUpdate  -> setTopologyID(currTopoID);
    scheduleAt( simTime() + changeRate, topoUpdate );
}

void Routing::setNeighborsIndex() {
    for ( int i = 0; i < topo -> getNode(mySatAddress)->getNumInLinks(); i++ )
        neighbors[i]=findDest(i,mySatAddress);
}

int Routing::findDest(int direction,int index){
    int row = 0;
    int base = getParentModule()->getParentModule()->par("colNum").intValue();
    int col = index%base;
    int temp = index;
    bool torus = true;

    //set col
    while( temp >= base )
    {
        row += 1;
        temp = temp - base;
    }

    switch ( direction ) {
    case left_d:
    {
        if ( base*row < index && index <= row*base + ( base - 1 ) )
            return index - 1;
        else
            return torus ? ( row*base + (base - 1) ): -1;
    }break;
    case up_d:
    {
        if ( row > 0 )
            return ((row-1)*base+col);

        else
            return torus ? ( num_of_hosts - base + col ): -1;
    }break;
    case right_d:
    {
        if( base*row <= index && index < row*base + ( base - 1 ) )
            return (index+1);

        else
            return torus ? ( base * row ): -1;

    }break;
    case down_d:
    {
        if ( row != base-1 )
            return ( ( row + 1 ) * base + col );

        else
            return torus ? col : -1;

    }break;
    case diagonal_up:
    {
        if( row > 0  && col > 0)
        {
            return (row-1)*base + (col -1);
        }
        else
        {
            int x;
            if( col > 0 )
                x = col -1;
            else
                x = base -1;
            int y;
            if( row == 0 )
                y = base -1;
            else
                y = row -1;
            return x + (y*base);
        }

    }break;
    case diagonal_down:
    {
        if( row < base - 1 && col < base -1)
        {
            return (row+1)*base + col + 1;
        }
        else
        {
            int x,y;
            if(row == base - 1)
                y = 0;
            else
                y = row + 1;
            if(col == base -1 )
                x = 0;
            else
                x = col + 1;

            return x + y*base;
        }

    }break;
    default :
    {
        return -1;
    }
    }//end switch direction

}

//update weight of neighbors links to send to leader. index i stands for satellite i. the weight is the value
void Routing::update_weight_table(){

    if(second_update)
    {
        return;
    }
    else
    {
        int linkID=0;

        for(int i = 0; i < topo->getNode(mySatAddress)->getNumInLinks();i++)
        {
            if(neighbors[i] != -1)
            {
                int j=0;
                EV<<"node["<<mySatAddress<<"] "
                        "link ["<<linkID<<"] "
                                "= "<<thisNode->getLinkIn(linkID)->getWeight()<<
                        ". old method ="<<this->topo->getNode(mySatAddress)->getLinkIn(linkID)->getWeight()<<endl<<endl;
               // thisNode->getLinkIn(linkID)->setWeight(8.5222 );
                EV<<"link updated to : "<<thisNode->getLinkIn(linkID)->getWeight()<<endl;
                leader_list[neighbors[i]] = thisNode->getLinkIn(linkID)->getWeight();
                linkID++;
            }
        }
    }
}

void Routing::send_link_info_to_leader(){
    int rootGateIndex;
    Packet *phase2_send_link_info_to_leader= new Packet("Send link info to Leader!");
    phase2_send_link_info_to_leader->setSrcAddr(mySatAddress);
    phase2_send_link_info_to_leader->setDestAddr(leader);
    phase2_send_link_info_to_leader->setPacketType(phase2_link_info);
    //phase2_send_link_info_to_leader->setTopologyID(currTopoID);
    phase2_send_link_info_to_leader -> setByteLength(1500);
    int i=0;
    for(i=0; i<topo->getNode(mySatAddress)->getNumInLinks(); i++)
//    for(int i = 0; i<4; i++)
    {
        phase2_send_link_info_to_leader->data[i]=neighbors[i];
        phase2_send_link_info_to_leader->datadouble[ i ] = queueSize[getLinkIndex(i)] / frameCapacity * 100;//(datarate/12000) * 100;
    }
    for(i = 0; i<num_of_hosts; i++)
    {
        phase2_send_link_info_to_leader -> datadouble[i] = -1;
        phase2_send_link_info_to_leader -> data[i] = -1;
    }
    //find the gate to reach leader
    RoutingTable::iterator iter = rtable.find(leader);
    rootGateIndex = (*iter).second;
  //  delayTime = (simTime(), intrand(num_of_hosts));
    sendDelayed(phase2_send_link_info_to_leader,0, "out",rootGateIndex);
}

void Routing::flushQueue(int type){
    for( int i = 0; i < topo->getNode(mySatAddress)->getNumInLinks() ; i++ ){
       cQueue *qptr= (cQueue *)(getParentModule()->getSubmodule("queue",i)); // check_and_cast gives runtime error
       if (!qptr)
           error("Missing queueModule");
       qptr->clear();
    }
}

/*
 * Args: Packet * pk (holds the path information from root)
 * Return: void
 * Description: set the better option (existing known route or incoming route
 */
void Routing::setPathToRoot(Packet *pk)
{
    int i=0,j=0,alternativePathWeight=0;
    //first
    if( pathID == -1 )
    {
        pathID = pk->getSrcAddr();
        i = 1;
        while( pk->data[i] != -1 )
        {
            path[i-1] = pk->data[i];
            alternativePathWeight += pk->datadouble[i-1];
            i++;
        }
        pathWeight = alternativePathWeight;

    }
    //new path arrived. will compare with old path.
    else
    {
        int contenderRoot= pk->getSrcAddr();
      //  while(pk->data[i] != -1)i++;
        while( pk->datadouble[j] !=-1 )
        {
            //path[j] = pk->data[i];
            alternativePathWeight += pk->datadouble[j];
            //i--;
            j++;
        }
        if( alternativePathWeight < pathWeight )
        {
            i=1;j=0;
            pathID = contenderRoot;
            pathWeight = alternativePathWeight;
            while(pk->data[i] != -1)i++;
            while( i > 1 )
            {
                path[j] = pk->data[i-1];
                //alternativePathWeight += pk->datadouble[j];
                i--;j++;
            }
            //clear history(old paths)
            while(j<num_of_hosts)
            {
                path[j] = -1;
                j++;
            }
        }
    }
}

//input : neighbor index
//output: queue index ("queue",index)
int Routing::getLinkIndex(int req){
    for(int queueIndex = 0; queueIndex< topo->getNode(mySatAddress)->getNumInLinks();queueIndex++)
        if(topo->getNode(mySatAddress)->getLinkIn(queueIndex)->getRemoteNode() == topo->getNode(neighbors[req]))
            return queueIndex;
    return -1; //if reaches here the link requested does not exist. exit the program
}

//input : queue index ("queue",index)
//output: neighbor index
int Routing::getNeighborIndex(int req){
    for( int NeihboorIndex = 0; NeihboorIndex < MAXAMOUNTOFLINKS ;NeihboorIndex++ )
        if( neighbors[NeihboorIndex] != -1 )
            if(topo->getNode(mySatAddress)->getLinkIn(req)->getRemoteNode() == topo->getNode(neighbors[NeihboorIndex]))
                return NeihboorIndex;
    return -1; //if reaches here the link requested does not exist. exit the program
}

int Routing::loadBalance(int reqLink,int incominglink)
{
    double temp_load_threshold = load_threshold;
    int bestmatch = 110;

    for(int pos = 0; pos < MAXAMOUNTOFLINKS; pos++)
        if(active_links_array[pos] != -1){
            int currentgate_id_local;
            currentgate_id_local = topo->getNode(mySatAddress)->getLinkIn(pos)->getLocalGateId();
            if(pos != reqLink && loadCounter[pos] < temp_load_threshold){
                if(incominglink != currentgate_id_local){
                    bestmatch = pos;
                    temp_load_threshold = loadCounter[pos];
                }
            }
        }

    if(bestmatch != 110)
       return bestmatch;
    else
       return reqLink;
}

int Routing::loadBalanceLinkPrediction(int reqLink,int incominglink)
{
    double temp_load_threshold = load_threshold;
    int bestmatch = 110;

    for(int pos = 0; pos < 6; pos++)
        if(active_links_array[pos] != -1 && !loadPredictionFlag[pos]){
           int currentgate_id_local;
           currentgate_id_local = topo->getNode(mySatAddress)->getLinkIn(pos)->getLocalGateId();
           if(pos != reqLink && loadCounter[pos] < temp_load_threshold){
               if(incominglink != currentgate_id_local){
                   bestmatch = pos;
                   temp_load_threshold = loadCounter[pos];
               }
           }
       }

    if(bestmatch != 110)
       return bestmatch;
    else
       return reqLink;
}

int Routing:: loadBalanceLinkFinder( int reqLink ,int incominglink,Packet *pk )
{
  //  int queuegate = -1;
    if (pk -> getPacketType() == leader_to_roots ||  pk -> getPacketType() == route_from_root ||pk -> getPacketType() == phase2_link_info){
        return reqLink;
    }

    if(pk->datadouble[0] == -5){
        int loop = 0,i=0;
        while( i < num_of_hosts ){
            if( pk->datadouble[i] == -1)
                break;
            if( pk->datadouble[i] == mySatAddress ){
                loop++;
            }
            if( loop > 1 )
                return reqLink;
            i++;
        }
        pk -> datadouble[i] = mySatAddress;
    }

    double temp_load_threshold;
    //load balance - can't reroute to the origin link. LINK A - 20% with lp TRUE; LINK B - 40% with lp FALSE
    if (load_balance_link_prediction){
        if( loadCounter[reqLink] > load_threshold )
            return loadBalance(reqLink,incominglink);
        else if( loadPredictionFlag[reqLink] )
            return loadBalanceLinkPrediction(reqLink,incominglink);//find min link with loadPredictionFlag FALSE
        else
            return reqLink;

    //without link prediction
    }else{
        if(loadCounter[reqLink] < load_threshold)
            return reqLink;
        return loadBalance(reqLink,incominglink);
    }
}

void Routing::receiveSignal(cComponent *src, simsignal_t id, cObject *value,cObject *details){
    EV<<endl<<"signal has recieved";
    controlSignals *signalMessage;
    int counter;
    signalMessage = (controlSignals *)value;
    if (hasGUI())
           getParentModule()->bubble("signal received");
}

/*
 * queueid = 1 char (0 to 3)
 * load = 00 - 99
 * connectedto = until end of char (\0)
 * queue size
 */
void Routing::signalStringParser(const char *s,int* queueid,uint32_t * load,int* connectedto,int *queuesize)
{
    int index = 0;
    //retrieve which queue is signaling the router (us)
    *queueid = s[index] - '0';
    index++;//space;
    index++;//start of next argument

    //load argument: with up to 20 digits (10^20)
    *queuesize = s[index] - '0';
    index++;
    while( s[index] != ' ' && s[index] != '\0')
    {
        *queuesize = *queuesize * 10 + (s[index] - '0');
        index++;
    }
}

void Routing::receiveSignal(cComponent *source, simsignal_t signalID, const char *s, cObject *details)
{
    int queueid,connectedto,queuesize;
    uint32_t load;
    signalStringParser(s,&queueid,&load,&connectedto,&queuesize);


    loadCounter[queueid] = (double)queuesize / (double)frameCapacity * (double)100;//(datarate/12000) * 100;
    queueSize[queueid] = queuesize;
    if(loadCounter[queueid]>=50)
            int a= 1;
//    if (hasGUI())
//       getParentModule()->bubble(s);

}

void Routing::sendMessage(Packet * pk, int outGateIndex, bool transmitAck)
{
    pk->setHopCount(pk->getHopCount()+1);

    // NOTE: New code for ACK
    if (transmitAck){
        sendAck(pk);
    }
    pk->setLastHopTime(simTime().dbl());        // Set time-stamp (that is unchanged by queue)
    if (pk->getPacketType() == message){
        msgSet.insert(pk->getId());             // ACK is only for messages
    }

    if( load_balance_mode && active_links > 2 )
    {
      int hotpotato;
      hotpotato = loadBalanceLinkFinder(outGateIndex,pk->getArrivalGateId(),pk);
      if( hotpotato != outGateIndex )
      {
          if(hasGUI())
            getParentModule()->bubble("LOAD BALANCING");
          pk -> setPacketType(hot_potato);
      }

        delayTime = 0;
        sendDelayed(pk,delayTime, "out",hotpotato);

    }
    else if( pk -> getHopCount() > 2*sqrt(num_of_hosts) )
    {
        int i=0;
        //find the gate. send to the gate to activate the func
        while(i<topo->getNode(mySatAddress)->getNumInLinks())
        {
            if( thisNode->getLinkIn(i)->getLocalGateId() == pk->getArrivalGateId())
                break;
            i++;
        }
      check_and_cast<L2Queue*>(getParentModule()->getSubmodule("queue", i)) -> deleteMessageAndRecord(pk,false);
      if (pk->getTopologyVarForUpdate())
          pk->deleteTopologyVar();
      Routing *rt = check_and_cast<Routing*>(getParentModule()->getParentModule()->getSubmodule("rte", pk->getSrcAddr())->getSubmodule("routing"));
      rt->packetDropCounterFromHops++;
      delete pk;
    }
    else
    {
      delayTime = 0;
      sendDelayed(pk,delayTime, "out",outGateIndex);
    }
}

//record last 10 topology's queue(or link) usage.uses overlap every 10 cycles
void Routing::updateLoadHistory()
{
    int load;
    int queuesize;
    L2Queue * temp;
    for( int i = 0; i < MAXAMOUNTOFLINKS; i++ )
    {
        if( loadTimerIndex % HISTORY_LENGTH != 0 )
        {
          //  Packet *pk = check_and_cast<Packet *>(msg);
            temp = (L2Queue*)getParentModule()->getSubmodule("queue", i);
            if(temp !=NULL)
            {
                queuesize = temp -> getQueueSize();
                if( queuesize  == 0 )
                {
                    load = 0;
                }
                else
                {
                    load = queuesize;
                }
                loadHistory[i][loadTimerIndex] = load;
            }
        }
        else
        {
            if(!loadPredictionIsOn)
            {
                LinkLoadPrediction();
                loadPredictionIsOn = true;
            }
            loadTimerIndex = 1;
        }
    }
    loadTimerIndex++;
}

void Routing::LinkLoadPrediction()
{
    int pattern[PATTERN_LENGTH];
    double loadPredicted;
    int history_temp[HISTORY_LENGTH];
    //Link prediction
    for ( int i = 0; i < MAXAMOUNTOFLINKS; i++ )
    {
        //its an inactive link at the moment
        if(active_links_array[i] == -1 || loadCounter[i] < 10 )
        {
            loadPredictionFlag[i] = false;
            continue;
        }
        for( int j = 0; j < HISTORY_LENGTH; j++ )
        {
            history_temp[j] = loadHistory[i][j];
        }
        int pivot=loadTimerIndex;
        if( pivot < PATTERN_LENGTH )
            pivot = HISTORY_LENGTH;
        for( int j = 0; j < PATTERN_LENGTH; j++ )
        {
           pattern[j] = history_temp[pivot - (PATTERN_LENGTH - j)];
        }
        loadPredicted = search(history_temp,pattern, ALPHA_P,BETA_P,PATTERN_LENGTH, loadTimerIndex);
        updateLoadBalancePrediction(i,loadPredicted);
        //cout<<"Node: "<<mySatAddress<<", link number: "<<i<<"load Predicted: "<<loadPredicted<<endl;
       // loadPrediction[i].record(loadPredicted);
    }
    Packet *loadpmRoutine = new Packet("load prediction routine");
    loadpmRoutine -> setPacketType(load_prediction_update);
    loadpmRoutine -> setSrcAddr(mySatAddress);
    loadpmRoutine -> setDestAddr(mySatAddress);
    scheduleAt( simTime() + LOAD_PREDICTION_INTERVAL, loadpmRoutine );
}

void Routing::updateLoadBalancePrediction(int index,int loadPredicted)
{
    double lp=(double)loadPredicted/frameCapacity*100;

    if(lp > 0.80*load_threshold)
        loadPredictionFlag[index]=true;
    else
        loadPredictionFlag[index]=false;
}

int Routing::totalErrors(int *pattern,int *history,int offset,int size_P){
    int total = 0;
    for(int i = 0; i < size_P; i++){
        total+=abs(pattern[i]-history[offset+i]);
    }
    return total;
}

void Routing::pi_func(int pattern[], int alpha,int size,int pi[]){
    int j,append=1;

    for (int i = 1; i < size; i++) {
        j = pi[i - 1];
        while (j > 0 && (abs(pattern[j] - pattern[i]) > alpha)) {
            j = pi[j - 1];
        } //while
        if (abs(pattern[j] - pattern[i]) <= alpha) {
            pi[append] = j + 1;
        } else {
            pi[append] = j;
        }
        append++;
    }
}

double Routing::search(int *T,int *P,int alpha,int beta, int pattern_size, int History_size)
{
    int i=0, j=0;
    int temp_j;
    int offset;
    int pi[pattern_size];
    for(int i = 0; i < pattern_size; i++)
        pi[i] = 0;
    int malloc_flag =1;
    int overlapIndexesSize = 1;
    int *overlapIndexes;
    int error;
    double load_predict=0;

    pi_func(P, alpha,pattern_size,pi);

    overlapIndexes = (int*) malloc(sizeof(int) * (overlapIndexesSize));
    overlapIndexes[0] = -1;

    for (i = 1; i < History_size; i++) {
        if(i == History_size - pattern_size){
            break;
        }

        while ((j > 0) && (abs(T[i] - P[j]) > alpha) && j<pattern_size) {
            j = pi[j - 1];
            //          cout <<"j inside while: " << j << endl;
            //          break;
        }    //while
        if ((abs(T[i] - P[j])) <= alpha) {
            j++;
        }    //if

        if (j == pattern_size) {
            temp_j = j;
            j = pi[j - 1];
            offset = i - pattern_size + 1;
            error = totalErrors(P, T, offset, pattern_size);
        //    cout << "num of errors: " << error << endl;
            if (error <= beta) {
                if (malloc_flag == 1) {
                    malloc_flag = 0;
                    overlapIndexes[overlapIndexesSize - 1] = i - (temp_j - 1);
                } else {
                    overlapIndexesSize++;
                    overlapIndexes = (int*) realloc(overlapIndexes,
                            sizeof(int) * (overlapIndexesSize));
                    overlapIndexes[overlapIndexesSize - 1] = i - (temp_j - 1);
                }
//                cout << "index of good match: "
//                        << overlapIndexes[overlapIndexesSize - 1] << endl;
                j = 0;
            }
        }    //if
    }    //for

    //no pattern has found
    if(malloc_flag == 1){
        free(overlapIndexes);
        return -1;
    //one pattern has been found
    }else if(overlapIndexesSize==1){
            //cout << T[overlapIndexes[0]+pattern_size] << endl;
            int history_index = overlapIndexes[0]+pattern_size;
            free(overlapIndexes);
            return T[history_index];
    //multiple patterns has been found. we will take the mean.
    }else{
        for(i=0; i<overlapIndexesSize;i++){
            load_predict += T[overlapIndexes[i]+pattern_size];
        }
        //cout << load_predict/overlapIndexesSize << endl;
        free(overlapIndexes);
        return load_predict/overlapIndexesSize;
    }
}

void Routing::schduleBaseLine()
{
    if (getParentModule()->getSubmodule("app")->par("hasApp").boolValue()){
        //start app
        Packet *startSend = new Packet("initiate App");
        startSend -> setPacketType(app_initial);
        startSend -> setTopologyID(currTopoID);
        //delayTime = 0;
        send( startSend , "localOut" );
    }
}

void Routing::saveTempLinks()
{
    for(int i = 0; i<MAXAMOUNTOFLINKS;i++)
    {
        tempActiveLinks[i] = -1;
        if(topo->getNode(mySatAddress)->getLinkIn(i)->isEnabled())
                 tempActiveLinks[i] = i;
    }
}

void Routing::keepAliveTempLinks()
{
    for(int i = 0; i<MAXAMOUNTOFLINKS;i++)
        if( tempActiveLinks[i] != -1 )
            topo->getNode(mySatAddress)->getLinkIn(tempActiveLinks[i])->enable();
    for(int i = 0; i<MAXAMOUNTOFLINKS;i++)
        if( tempActiveLinks[i] != -1 )
            topo->getNode(mySatAddress)->getLinkOut(tempActiveLinks[i])->enable();

    Packet *templink = new Packet("turn off link");
    templink -> setPacketType(turn_off_link);
    templink -> setSrcAddr(mySatAddress);
    templink -> setDestAddr(mySatAddress);
    scheduleAt( simTime() + 5, templink );
}

int* Routing::getNeighborsArr(int index,int* arr)
{
    for(int i = 0; i< MAXAMOUNTOFLINKS ; i++)
        arr[i]=check_and_cast<Routing *>(topo->getNode(index)->getModule()->getSubmodule("routing"))->neighbors[i];
    return arr;
}

void Routing::getRootsArray(cTopology *OrigTopo, int m){
    /* Create an array of roots for LAST. Traverse diameter (iteratively) ceil((nodeNum - m)/m)
     *      hops, and each node at that distance becomes a root for LAST topology.
     * Input:
     *      [m] - is a parameter of the algorithm
     *      [OrigTopo] - an MST, calculated by primMST
     * */

    int nodeNum = OrigTopo->getNumNodes();
    int dist = ceil((nodeNum - m)/m);
    int src, dst, diameter = INT_MAX;
    std::list<int> lst;
    cTopology *tmpTopo = CreateTopology(OrigTopo, "MST for root selection");

    // Push roots to list
    while(diameter > m){
        diameter = getDiameter(tmpTopo, &src, &dst);
        int res = getInnerNodeFromDiameter(tmpTopo, dist, src, dst);
        lst.push_back(res);
    }

    // Convert List to array
    lst.sort();
    lst.unique();
    rootCounter = lst.size();
    std::copy(lst.begin(), lst.end(), rootNodesArr);

    // Clean up
    delete tmpTopo;
}

int Routing::getInnerNodeFromDiameter(cTopology *OrigTopo, int dist, int src, int dst){
    /* Traverse Diameter (from given [src] to [dst]) [dist] steps, and delete all nodes in path.
     * Return the address of the last node visited (in distance [dist] or end of path)
     * NOTE: Never send the original topology to this function - it changes [OrigTopo]
     * */

    cTopology::LinkOut *tempLinkOut;

    // Switch topology information to refer to [dst]
     cTopology::Node *other = OrigTopo->getNode(dst);
     OrigTopo->calculateUnweightedSingleShortestPathsTo(other);

    // Traverse diameter
    cTopology::Node *curr = OrigTopo->getNode(src);
    while(dist){
        // Get the edge to the next node in path to [dst] (if exists)
        tempLinkOut = curr->getPath(dst);
        if (tempLinkOut){
            // Move to next node
            OrigTopo->deleteNode(curr);
            curr = tempLinkOut->getRemoteNode();
            dist--;
        }
        else
            break;
    }

    return curr->getModule()->par("address").intValue();
}

int Routing::getDiameter(cTopology *OrigTopo, int *src, int *dst){
    /* Find [OrigTopo]'s diameter by checking all combinations and looking for the longest
     *      shortest path.
     * Returns diameter length, and the indexes of the 2 nodes the diameter starts and finishes:
     *      [src] - Start of diameter
     *      [dst] - End of diameter
     * NOTE: [OrigTopo] should be MST calculated by primMST
     * */

    int numOfNodes = OrigTopo->getNumNodes();
    int pathWeight;
    int diameter = -1;
    cTopology::Node *curr = NULL;
    cTopology::Node *other = NULL;

    for(int i = 0; i < numOfNodes; i++){
        curr = OrigTopo->getNode(i);
        for(int j = 0; j < numOfNodes; j++){
            other = OrigTopo->getNode(j);
            OrigTopo->calculateUnweightedSingleShortestPathsTo(other);
            pathWeight = curr->getDistanceToTarget();
            if (pathWeight > 0 && pathWeight > diameter){
                diameter = pathWeight;
                *src = i;
                *dst = j;
            }
        }
    }

    return diameter;
}

int Routing::calculateFutureSatellite(int destTerminal){
    /*
     * */

    // Calculate number of hops
    int numOfHops;
    getClosestSatellite(destTerminal, &numOfHops);

    if (numOfHops != -1)
        return getClosestSatellite(destTerminal, nullptr, numOfHops * getAverageLinkDelay());
    else
        return -1;

    cModule *ter = getParentModule()->getParentModule()->getSubmodule("terminal", destTerminal)->getSubmodule("app");
    TerminalApp *terApp = check_and_cast<TerminalApp*>(ter);
    return terApp->getConnectedSatelliteAddress();
}

Packet* Routing::createPacketForDestinationSatellite(TerminalMsg *terminalMessageToEncapsulate, bool usePrediction, int destSat){
    /*  Creates an "Ethernet Frame" from a given terminal message ([terminalMessageToEncapsulate]) by encapsulating
     *      it as payload.
     *  This function calls [calculateFutureSatellite()] function to predict the destination satellite if [usePrediction] is true.
     *      In this case [destSat] is ignored.
     *  If [usePrediction] is false, destSat is used as destination satellite
     *  Returns the resulting packet.
     * */

    int dst = usePrediction? calculateFutureSatellite(terminalMessageToEncapsulate->getDestAddr()) : destSat;
    if (dst == -1){
        // Either no satellite will be connected OR given satellite is invalid
        EV << "Satellite " << mySatAddress << " didn't find any connected satellites to terminal " << terminalMessageToEncapsulate->getDestAddr() << endl;
        return nullptr;
    }

    // Create Ethernet frame
    Packet * terminalMessage = new Packet("Encapsulated terminal message");
    terminalMessage -> setSrcAddr(mySatAddress);
    terminalMessage -> setDestAddr(dst);
    terminalMessage -> setPacketType(message);
    terminalMessage -> setByteLength(22);     // Ethernet protocol header/trailer
    terminalMessage -> setHopCount(0);
    terminalMessage-> setTopologyID(currTopoID);
    int i=0;
    while( i <  num_of_hosts )
    {
      terminalMessage->data[i] = -1;
      terminalMessage->datadouble[i] = -1;
      i++;
    }
    terminalMessage -> datadouble[0] = -2;

    // Encapsulate payload
    terminalMessage->encapsulate(terminalMessageToEncapsulate);     // frame size now is 1500 + 22 = 1522, which is Ethernet MTU

    return terminalMessage;
}

void Routing::broadcastTerminalStatus(int terminalAddress, int status, bool loadAllConnections){
    /* Send terminal_list packet to all neighbors on all active links about a terminal's connection [status]
     *          (terminal is referred by [terminalAddress])
     * If [status] = 1 then the said terminal is connected, else it's disconnected
     * If [loadAllConnections] is true, load all known terminals in this message and ignores [terminalAddress]/[status]
     * Main idea: 1. Create packet
     *            2. Iterate over routing table to create a list of active links
     *            3. Send a duplication of the message across all active links, last gets the original
     * */

    // Create informative packet name
    std::string packetName = "Terminal List update - ";
    if (!loadAllConnections){
        // Local update - either a terminal joined or moved
        if (status){
            packetName += "add a new terminal";
        }
        else{
            packetName += "remove a terminal";
        }
    }
    else{
        // Send all connected terminals

        if (myTerminalMap.empty()){
            // No terminals are connected
            EV << "Satellite " << mySatAddress << " has no terminals to re-send" << endl;
            return;
        }

        // At least 1 terminal is connected
        packetName += "all known connections (satellite " + std::to_string(mySatAddress) + ")";
    }

    // Create information packet - same base as in old App::sendMessage()
    Packet * terminalInfoPacket = new Packet(packetName.c_str());
    terminalInfoPacket -> setSrcAddr(mySatAddress);
    terminalInfoPacket -> setDestAddr(-1);
    terminalInfoPacket -> setPacketType(terminal_list);
    terminalInfoPacket -> setByteLength(64);    // Ethernet protocol minimum frame size
    terminalInfoPacket -> setHopCount(0);

    if (loadAllConnections){
        // Send all known connections to neighbors
        terminalInfoPacket->terminalListLength = myTerminalMap.size();

        int i = 0;
        for(RoutingTable::iterator it = myTerminalMap.begin(); it != myTerminalMap.end(); it++, i++){
            terminalInfoPacket->terminalConnectionStatus[i] = connected;
            terminalInfoPacket->terminalList[i] = it->first;
        }
    }
    else{
        // Only local update
        terminalInfoPacket->terminalListLength = 1;
        terminalInfoPacket->terminalConnectionStatus[0] = status;
        terminalInfoPacket->terminalList[0] = terminalAddress;
    }

    // Get a list of all active links
    std::list<int> lst;
    for(int i = 0; i < rtable.size(); i++){
        lst.push_back(rtable[i]);
    }
    lst.sort(); // Necessary for lst.unique() to work properly
    lst.unique();

    // Broadcast terminal_list to all active links
    std::list<int>::iterator last = --lst.end(); // Last element in list

    for(std::list<int>::iterator it = lst.begin(); it != lst.end(); it++){
        if (it != last)
            sendMessage(terminalInfoPacket->dup(), *it, false);
        else
            sendMessage(terminalInfoPacket, *it, false);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                                                                // lllinitialize

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Routing::initialize() {

    //load balance workzone
    dropSignal = registerSignal("drop");
    outputIfSignal = registerSignal("outputIf");
    TopoLinkCounter = registerSignal("TopoLinkCounter");
    hopCountSignal = registerSignal("hopCount");
    LBS = registerSignal("load_balance_channel");

    links_control[0] = getParentModule()->getSubmodule("queue", 0);
    if( links_control[0] != NULL)
        links_control[0] -> subscribe(LBS, this);
    links_control[1] = getParentModule()->getSubmodule("queue", 1);
    if( links_control[1] != NULL)
        links_control[1] -> subscribe(LBS, this);
    links_control[2] = getParentModule()->getSubmodule("queue", 2);
    if( links_control[2] != NULL)
        links_control[2] -> subscribe(LBS, this);
    links_control[3] = getParentModule()->getSubmodule("queue", 3);
    if( links_control[3] != NULL)
        links_control[3] -> subscribe(LBS, this);
    loadPrediction[0].setName("link0");
    loadPrediction[1].setName("link1");
    loadPrediction[2].setName("link2");
    loadPrediction[3].setName("link3");

    //load balance workzone
    mySatAddress = getParentModule()->par("address");
    hasInitTopo = 0;
    packetType = 1;
    loadTimerIndex = 1;
    for(int i = 0; i < MAXAMOUNTOFLINKS; i++)
        for(int j = 0; j < HISTORY_LENGTH; j++)
            loadHistory[i][j] = 0;

    frameCapacity = getParentModule()->par("frameCapacity");
    packetLength = getParentModule()->getSubmodule("app")->par("packetLength");

    m_var = getParentModule()->par("m_var"); // How many 'hops' on diameter
    alpha = getParentModule()->par("alpha"); // Maximal ratio

    datarate =  check_and_cast<cDatarateChannel*>(getParentModule()->getSubmodule("queue", 0)->gate("line$o")->getTransmissionChannel())->getDatarate();
    load_balance_mode = getParentModule()->par("load_balance_mode");
    load_balance_link_prediction = getParentModule()->par("load_balance_link_prediction");
    load_threshold = getParentModule() -> par("load_threshold");

    fullTopoRun = getParentModule()->par("fullTopoRun");

    changeRate = getParentModule()->par("changeRate");
    topo = new cTopology("topo");
    std::vector<std::string> nedTypes;
    num_of_hosts = getParentModule()->par("num_of_satellite").intValue();
    nedTypes.push_back(getParentModule()->getNedTypeName());
    topo->extractByNedTypeName(nedTypes);
    NodesInTopo = topo->getNumNodes();
    EV << "cTopology found " << NodesInTopo << " nodes\n";
    thisNode = topo->getNodeFor(getParentModule());
    linkCounter = new int[thisNode->getNumInLinks()];
    //delayTime = exponential(1);
    _sendTime = &par("sendTime");
    for (int j = 0; j < thisNode->getNumInLinks(); j++) {
        linkCounter[j] = 0;
        loadPredictionFlag[j] = false;
    }
    loadPredictionIsOn = false;
    phase1Flag = 1;
    for (int j = 0; j < num_of_hosts; j++)
            path[j] = -1;

    pathID = -1;
    pathWeight = -1;
    pathcounter = 0;

    for(int i = 0; i < num_of_hosts; i++)
        rootNodesArr[i] = -1;

    for(int i = 0; i < MAXAMOUNTOFLINKS; i++)
        queueSize[i] = -1;

    for(int i = 0; i < 4; i++)
        rootNodesPathWeight[i] = -1;

    //// Terminal handling
    // Taken gates array
    isDirectPortTaken = new int[gateSize("terminalIn")];
    for(int i=0; i < gateSize("terminalIn"); i++)
        isDirectPortTaken[i] = 0;
    // Max hop count for terminal_list
    maxHopCountForTerminalList = getParentModule()->par("maxHopCountForTerminalList").intValue();

    //// Link delay prediction
    for(int i = 0; i < MAXAMOUNTOFLINKS; i++)
        linkDelay[i] = DEFAULT_DELAY;

    //set initial topo weight's (1on each link)
    setLinksWeight(topo, 0);
    //init Routing table from topology
    initRtable(topo, thisNode);
    //set neighbors for sending updates
    initial_leader_table() ;

    //set who is my neighbor
    setNeighborsIndex();
    update_weight_table();

    leader = 0;
    // start simulation without choosing leader
    if(mySatAddress == 0 && !BASELINE_MODE )
    {
        schduleNewTopology();
    }

    // start simulation without choosing leader - baseline
    else
    {
        setLinksWeight(topo,0);
        baseSPT = BuildSPT(topo,mySatAddress);
        baseSPT -> calculateWeightedSingleShortestPathsTo(thisNode);
        initRtable(baseSPT,thisNode);
        schduleBaseLine();
    }

//    // start simulation with choosing leader
//
//        scheduleChoosingLeaderPhaseOne();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                                                                // lllhandlemessage

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Routing::handleMessage(cMessage *msg) {

    switch (msg->getKind()){
        case legacy:{

            //// Kind = 0 -> Legacy code - Lavi's old code. [Packet]  messages go here
            Packet *pk = check_and_cast<Packet *>(msg);
            int destAddr = pk->getDestAddr();
            int srcAddr = pk->getSrcAddr();
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // lll   destAddr == mySatAddress
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if(destAddr == mySatAddress){
                // Current satellite is the target satellite
                    switch(pk->getPacketType()){
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll phase1_update  destAddr == mySatAddress
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case phase1_update:{
                            // Start Leader Election
                            if (srcAddr == mySatAddress){
                                // initial message (initialize or re-election)
                                if (pk->getTopologyVarForUpdate())
                                    pk->deleteTopologyVar();
                                delete pk;
                                leader_table[mySatAddress] = mySatAddress; // first update its only me
                                if (hasGUI())
                                    getParentModule()->bubble("Schedule choosing leader phase 1!");
                                send_phase1_update();// update message
                            }
                            else{
                                //wait to get everyone's ids
                                if(phase1Flag){
                                    update_leader_table(pk);
                                    if (pk->getTopologyVarForUpdate())
                                        pk->deleteTopologyVar();
                                    delete pk;
                                    send_phase1_update();
                                }
                                else{
                                    if (pk->getTopologyVarForUpdate())
                                        pk->deleteTopologyVar();
                                    delete pk;
                                }
                            }
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll phase2_link_info  destAddr == mySatAddress
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case phase2_link_info:{
                            // Leader gathers information about links
                            if(mySatAddress != leader){
                                // I'm not leader
                                leader = pk->getSrcAddr(); //only leader can send this type of message
                                if (pk->getTopologyVarForUpdate())
                                    pk->deleteTopologyVar();
                                delete pk;

                                //we have leader so we can raise the flag(if we didn't already)
                                phase1Flag = 0;

                                // send info to leader
                                send_link_info_to_leader();
                            }
                            else {
                                // I'm the leader - save data
                                for( int i = 0; i < 4; i++ )
                                    n_table[pk->getSrcAddr()][pk->data[i]] = pk->datadouble[i];

                                linkdatacounter++;
                                if(linkdatacounter == num_of_hosts-1 ){
                                    // gathered all data possible
                                    linkdatacounter = 0;
                                    schduleNewTopology();
                                }
                                if (pk->getTopologyVarForUpdate())
                                    pk->deleteTopologyVar();
                                delete pk;
                            }
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll   leader_to_roots destAddr == mySatAddress. IAM ROOT . send route_from root
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case leader_to_roots :{
                            if(currTopoID != 0 && currTopoID != pk -> getTopologyID()){
                                second_update = true;
                            }

                            if(second_update && currTopoID != 0 && !saveTempLinkFlag){
                                saveTempLinkFlag = true;
                                saveTempLinks();
                            }

                            copyTopology(topo, pk->getTopologyVarForUpdate(), "green");
                            rootID = mySatAddress;
                            currTopoID = pk -> getTopologyID();
                            rootID_current_topo_ID = currTopoID;
                            topocounter++;
                            thisNode = topo -> getNodeFor(getParentModule());
                            initRtable(topo, thisNode);
                            if(second_update && currTopoID != 0 && saveTempLinkFlag && !keepAliveTempLinksFlag){
                                keepAliveTempLinksFlag = true;
                                keepAliveTempLinks();
                            }
                            rootCounter = pk->data[0];
                            for( int i = 1; i < rootCounter + 1; i++ ){
                                rootNodesArr[i-1]=pk->data[i];
                            }
                            for(int i=0;i<num_of_hosts;i++){
                                if( i == mySatAddress )
                                    continue;

                                RoutingTable::iterator iter = rtable.find(i);
                                int rootGateIndex = (*iter).second;

                                Packet *source_route = new Packet("path packet");
                                source_route -> setPacketType(route_from_root);
                                source_route -> setTopologyID(currTopoID);
                                source_route -> setSrcAddr(mySatAddress);
                                source_route -> setDestAddr(i);
                                source_route -> setByteLength(1500);
                                source_route -> datadouble[0] = thisNode->getLinkOut(rootGateIndex)->getWeight();
                                source_route -> datadouble[1] = -1;
                                source_route -> data[0] = rootCounter;
                                source_route -> data[1] = mySatAddress;
                                Routing::LoadPacket(source_route, topo);
                                for( int j = 2; j < num_of_hosts; j++ ){
                                    source_route->data[j] = -1;
                                    source_route->datadouble[j] = -1;
                                }

                                send(source_route, "out",rootGateIndex);
                            }
                            //for all nodes create and send route_from_root;
                            //rx : route_from_root save in table; activate app; send by table;
                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            if (hasGUI())
                                getParentModule()->bubble("LAST tree created!");
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll   route_from_root destAddr == mySatAddress. node decide to which root to communicate.
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case route_from_root :{
                            if( currTopoID != 0 && currTopoID != pk -> getTopologyID() ){
                                 second_update = true;
                            }
                            if(  currTopoID != pk -> getTopologyID() ){
                                 topocounter++;
                            }
                            //New Topology. reset root info
                            if( currTopoID != 0 && currTopoID != pk -> getTopologyID() ){
                                //if i was a root in the old topology and now iam not.
                                if(second_update)
                                    rootID = pk -> getTopologyID() == rootID_current_topo_ID ? rootID : -1;
                                for(int i= 0;i<num_of_hosts;i++){
                                    rootNodesArr[i] = -1;
                                    path[i] = -1;
                                }
                                pathcounter = 0;
                                pathID = -1;
                                pathWeight = -1;

                                if( second_update && currTopoID != 0 && !saveTempLinkFlag ){
                                    saveTempLinkFlag = true;
                                    saveTempLinks();
                               }
                            }

                            currTopoID = pk -> getTopologyID();
                            pathcounter ++;
                            rootCounter = pk->data[0];
                            //iam not root. so i need to learn about the path u
                            double weightOfPath = 0;
                            int j=0;
                            while(pk->datadouble[j]!=-1){
                               weightOfPath += pk->datadouble[j];
                               j++;
                            }

                            j=0;
                            //learn about routes
                            while(rootNodesArr[j] != -1 && rootNodesArr[j] != pk->getSrcAddr())
                                j++;
                            rootNodesArr[j]=pk->getSrcAddr();
                            setPathToRoot(pk);
                            copyTopology(topo, pk->getTopologyVarForUpdate(), "green");
                            setActiveLinks();
                            if( second_update && currTopoID != 0 && !keepAliveTempLinksFlag  && saveTempLinkFlag  ){
                                keepAliveTempLinksFlag = true;
                                keepAliveTempLinks();
                            }

                            thisNode = topo->getNodeFor(getParentModule());
                            initRtable(topo, thisNode);

                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                           if (pathcounter == rootCounter && !app_is_on){
                               app_is_on = true;
                               if (getParentModule()->getSubmodule("app")->par("hasApp").boolValue()){
                                   //start app
                                   Packet *startSend = new Packet("initiate App");
                                   startSend -> setPacketType(app_initial);
                                   startSend -> setTopologyID(currTopoID);
                                   //delayTime = 0;
                                   send( startSend , "localOut" );
                               }

                               //start load prediction recording
                               if(load_balance_link_prediction){
                                    Packet *loadpm = new Packet("load prediction initial packet");
                                    loadpm -> setPacketType(load_prediction);
                                    loadpm -> setSrcAddr(mySatAddress);
                                    loadpm -> setDestAddr(mySatAddress);
                                    //we will start record in the 3rd cycle. when the system is stable
                                    scheduleAt( simTime() + 3*changeRate, loadpm );
                               }
                           }
                           //we learned about all the roots
                           //how to get the total value of the path

                           if (pathcounter == rootCounter){
                               // After learning all paths, clear all neighbor terminal knowledge, and broadcast my terminal database
                               neighborTerminalMap.clear();
                               Packet *resendTerminals = new Packet("Re-send terminals packet (self message)");
                               resendTerminals->setSrcAddr(mySatAddress);
                               resendTerminals->setDestAddr(mySatAddress);
                               resendTerminals->setPacketType(terminal_list_resend);
                               scheduleAt(simTime(), resendTerminals);
                           }

                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // lll   message == mySatAddress.
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case message :{
                            if (!getParentModule()->getSubmodule("app")->par("hasApp").boolValue()){
                                //// New message code
                                // Packet received is encapsulated
                                TerminalMsg *terMsg = check_and_cast<TerminalMsg*>(pk->decapsulate());

                                // Check if source exists in any table
                                RoutingTable::iterator it = myTerminalMap.find(terMsg->getDestAddr());

                                if (it != myTerminalMap.end()){
                                    //// Found terminal in my map

                                    EV << "Terminal " << terMsg->getDestAddr() << " is connected to me - satellite " << mySatAddress << endl;

                                    terMsg->setHopCount(pk->getHopCount());

                                    if (isDirectPortTaken[it->second] == 1){
                                        // Main connection
                                        sendDirect(terMsg, getParentModule()->getParentModule()->getSubmodule("terminal", it->first),"mainIn");
                                    }
                                    else{
                                        // Sub connection
                                        sendDirect(terMsg, getParentModule()->getParentModule()->getSubmodule("terminal", it->first),"subIn");
                                    }

                                }
                                else{
                                    // Terminal is not in my map - check neighbors

                                    it = neighborTerminalMap.find(terMsg->getDestAddr());
                                    if (it != neighborTerminalMap.end()){
                                        // Neighbor has that terminal connected - Re-encapsulate and sent

                                        EV << "Terminal " << terMsg->getDestAddr() << " is connected to my neighbor - satellite " << it->second << endl;

                                        int outGateIndex = rtable.find(it->second)->second;
                                        Packet* newPacket = createPacketForDestinationSatellite(terMsg, false, it->second);
                                        sendMessage(newPacket, outGateIndex, false);
                                        packetCounter++;
                                    }
                                    else{
                                        // No known neighbor has the terminal connected
                                        EV << "Terminal " << terMsg->getDestAddr() << " is not around, dropping message" <<endl;
                                        Routing *rt = check_and_cast<Routing*>(getParentModule()->getParentModule()->getSubmodule("rte", pk->getSrcAddr())->getSubmodule("routing"));
                                        rt->packetDropCounter++;
                                        delete terMsg;
                                    }
                                }

                                // Send ACK if the message is from other module
                                if (!pk->isSelfMessage())
                                    sendAck(pk);

                                delete pk;
                            }
                            else{
                                //// Legacy code
                                //recive message. send to app
                                send( (cMessage*)pk , "localOut" );
                            }
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // lll   load_prediction message == mySatAddress.
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case load_prediction :{
                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            updateLoadHistory();
                            Packet *loadpm = new Packet("load prediction initial packet");
                            loadpm -> setPacketType(load_prediction);
                            loadpm -> setSrcAddr(mySatAddress);
                            loadpm -> setDestAddr(mySatAddress);
                            loadpm -> setHopCount(0);
                            scheduleAt( simTime() + 0.2, loadpm );
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // lll   load_prediction_update message == mySatAddress.
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case load_prediction_update :{
                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            LinkLoadPrediction();
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // lll  update_topology self message for leader
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case update_topology:{
                            fiftLink = (topocounter % 2 == 1) ? true : false;
                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            double wgttemp;
                            reset_ntable();

                            for(int i=0;i<num_of_hosts;i++){
                                cTopology::Node * currNode = topo->getNode(i);
                                //negiboors of node i
                                int tempNeighbor[6];
                                int *arr = getNeighborsArr(i,tempNeighbor);
                                int Neighbor;
                                //learn and update wgt's
                                for(int j=0;j<topo->getNode(i)->getNumInLinks();j++){
                                    double wgt,queuesize;
                                    Neighbor = tempNeighbor[getNeighborIndex(j)];
                                    if(fiftLink){
                                        queuesize = check_and_cast<L2Queue*>(topo->getNode(i)->getModule()->getSubmodule("queue", j)) -> getQueueSize();
                                        wgt = queuesize / frameCapacity * 100;//(datarate/12000) * 100;
                                        n_table[i][Neighbor] = wgt;
                                        wgt = (n_table[i][Neighbor] > 100) ? 100 :  n_table[i][Neighbor];
                                        n_table[i][Neighbor] = wgt;
                                    }
                                    else{
                                        //the first 4(0-3)links stay connected at all times
                                        if(j<3){
                                            queuesize = check_and_cast<L2Queue*>(topo->getNode(i)->getModule()->getSubmodule("queue", j)) -> getQueueSize();
                                            wgt = queuesize / frameCapacity * 100;//(datarate/12000) * 100;
                                            n_table[i][Neighbor] = wgt;
                                            wgt = n_table[i][tempNeighbor[getNeighborIndex(j)]] > 100 ? 100 :  n_table[i][tempNeighbor[getNeighborIndex(j)]] ;
                                            n_table[i][Neighbor] = wgt;
                                        }
                                        //the 5th and the 6th connection is turend off by setting value of inf
                                        else{
                                            n_table[i][Neighbor] = INT_MAX;
                                        }
                                    }

                                    //will add 1 to avoid wgt of 0
                                    wgttemp =  n_table[i][Neighbor] + 1;

                                    currNode->getLinkIn(j)->setWeight(wgttemp);
                                    currNode->getLinkOut(j)->setWeight(wgttemp);
                                }
                            }
                            schduleNewTopology();
                        }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    // lll  turn_off_link self message to disable temp links (after topo update)
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        case turn_off_link:{
                            for(int i = 0; i<MAXAMOUNTOFLINKS;i++)
                                if( tempActiveLinks[i] != -1 && active_links_array[i] == -1 )
                                    topo->getNode(mySatAddress)->getLinkIn(tempActiveLinks[i])->disable();
                            for(int i = 0; i<MAXAMOUNTOFLINKS;i++)
                                if( tempActiveLinks[i] != -1 && active_links_array[i] == -1 )
                                    topo->getNode(mySatAddress)->getLinkOut(tempActiveLinks[i])->disable();
                            saveTempLinkFlag = false;
                            keepAliveTempLinksFlag = false;
                            delete pk;
                         }break;

                        // NOTE: NOT legacy
                        case terminal_list_resend:{
                            broadcastTerminalStatus(-1, -1, true);
                            delete pk;
                            break;
                        }
                    } // End of switch statement
                } // End of destination if

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // lll   destAddr != mySatAddress
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            else{
                switch(pk->getPacketType()){
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll  message destAddr != mySatAddress
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    case message :{
                        if(BASELINE_MODE){
                            RoutingTable::iterator it = rtable.find(pk->getDestAddr());
                            int outGateIndex = (*it).second;
                            send(pk,"out",outGateIndex);
                            break;
                        }

                        //load_balance
                        if(pk -> getHopCount() > 20){
                            Routing *rt = check_and_cast<Routing*>(getParentModule()->getParentModule()->getSubmodule("rte", pk->getSrcAddr())->getSubmodule("routing"));
                            rt->packetDropCounterFromHops++;

                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            EV<<endl<<endl<<"message we DROP!@#!@"<<endl<<endl;
                        }
                        else
                            if( pk -> datadouble[0] == hot_potato_lable){
                                RoutingTable::iterator it = rtable.find(pk->getDestAddr());
                                int outGateIndex = (*it).second;
                                sendMessage(pk,outGateIndex, !pk->isSelfMessage());
                            }
                            else
                                if( pk->getSrcAddr() == mySatAddress && pk->datadouble[0] == -2 ){
                                    int i = 0;
                                    while( path[i] != -1 ){
                                        pk -> data[i] = path[i];
                                        i++;
                                    }

                                    if( rootID != -1 )
                                       pk -> datadouble[0] = -3;

                                    i = 0;
                                    while(pk->data[i] != -1)
                                        i++;
                                    i--;

                                    RoutingTable::iterator it = rtable.find(pk->data[i]);
                                    int outGateIndex = (*it).second;
                                    sendMessage(pk,outGateIndex,!pk->isSelfMessage());
                                }
                                //message on the way to root
                                //message on the way to dst
                                else{
                                    //shortest path from root to dest
                                    int rootID_temp;
                                    rootID_temp = pk->data[0];
                                    if(mySatAddress == rootID_temp)
                                        pk->datadouble[0] = -3;

                                    //message on the way to dst
                                    if( rootID_temp == mySatAddress  || pk->datadouble[0] == -3 ){
                                        if( hasGUI() && rootID == mySatAddress )
                                            this->getParentModule()->bubble("root received message");
                                        RoutingTable::iterator it = rtable.find( pk->getDestAddr() );
                                        int outGateIndex = (*it).second;
                                       sendMessage(pk,outGateIndex,!pk->isSelfMessage());
                                    }
                                    //message on the way to root
                                    else{
                                        int i = 0;
                                        while(pk->data[i] != mySatAddress && i < num_of_hosts)
                                            i++;
                                        i--;
                                        RoutingTable::iterator it = rtable.find(pk->data[i]);
                                        int outGateIndex = (*it).second;
                                        sendMessage(pk,outGateIndex,!pk->isSelfMessage());
                                    }
                                }
                    }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll  hot_potato destAddr != mySatAddress hot potato case (load balance).
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    case hot_potato :{
                        pk -> setSrcAddr(mySatAddress);
                        pk -> setDestAddr(pk->getDestAddr());
                        pk -> setPacketType(message);
                        pk -> setBitLength((int64_t) 12000);

                        int index=0;
                        while(index<num_of_hosts){
                            pk -> data[index]=-1;
                            pk ->datadouble[index] = -1;
                            index++;
                        }
                        int i = 0;

                        while( path[i] != -1 ){
                            pk -> data[i] = path[i];
                            i++;
                        }

                        i = 0;
                        while(pk->data[i] != -1)
                            i++;
                        i--;

                        int outGateIndex;
                        pk -> datadouble[0] = hot_potato_lable;
                        RoutingTable::iterator it = rtable.find(pk->getDestAddr());
                        outGateIndex = (*it).second;
                        sendMessage(pk,outGateIndex);   // XXX: Hot potato is never a self message
                    }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll  route_from_root destAddr != mySatAddress need to update route for destination.
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    case route_from_root:{
                        thisNode = topo->getNodeFor(getParentModule());
                        initRtable(topo, thisNode);
                        RoutingTable::iterator it = rtable.find(destAddr);
                        int outGateIndex = (*it).second;

                        int j=1;
                        while(pk->data[j]!=-1){
                            if(pk->data[j] == mySatAddress){
                                if(topocounter){
                                    int aaaaa=1;
                                    aaaaa++;
                                }
                                break;
                            }
                            j++;
                        }

                        if(pk->data[j] == mySatAddress){
                            sendDelayed(pk,delayTime , "out",outGateIndex);
                        }
                        else{
                            pk->data[j]=mySatAddress;
                            j=1;
                            while(pk->datadouble[j]!=-1)
                                j++;
                            pk->datadouble[j]=thisNode->getLinkOut(outGateIndex)->getWeight();
                            delayTime = 0;//exponential(0.01);
                            sendDelayed(pk,delayTime , "out",outGateIndex);
                        }
                    }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll  leader_to_roots destAddr != mySatAddress
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    case  leader_to_roots :{
                        rootCounter = pk->data[0];
                        copyTopology(this->topo, pk->getTopologyVarForUpdate(), "green");
                        setActiveLinks();
                        currTopoID = pk->getTopologyID();
                        thisNode = topo->getNodeFor(getParentModule());
                        initRtable(topo, thisNode);
                        RoutingTable::iterator it = rtable.find(destAddr);
                        int outGateIndex = (*it).second;
                        delayTime = 0;//exponential(0.01);
                        send(pk,"out",outGateIndex);
                    }break;
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // George: terminal_list is sent with invalid address (-1 = Broadcast) - always goes here
                        // NOTE: NOT legacy
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    case terminal_list:{
                        //// A neighbor satellite broadcasted a change in its terminals
                        if (pk->getHopCount() == maxHopCountForTerminalList + 1){
                            // Max hops reached - drop packet
                            delete pk;
                            EV << "Satellite " << mySatAddress << " dropped terminal_list packet (too many hops)" << endl;
                            return;
                        }

                        if (mySatAddress != pk->getSrcAddr()){
                            //// Read data to neighbor map if the source is not me
                            int loopCnt = pk->terminalListLength;
                            for(int i = 0; i < loopCnt; i++){
                                if (pk->terminalConnectionStatus[i]){
                                    // Terminal has connected to a neighbor - add terminal to neighbor map

                                    neighborTerminalMap[pk->terminalList[i]] = pk->getSrcAddr();
                                    EV << "Satellite " << mySatAddress << " added satellite " << pk->getSrcAddr() << "'s terminal " << pk->terminalList[i] << endl;
                                }
                                else {
                                    // Terminal has disconnected from a neighbor - remove terminal from neighbor map

                                    // Check if source exists in table and delete it
                                    RoutingTable::iterator it;
                                    it = neighborTerminalMap.find(pk->terminalList[i]);
                                    if (it != neighborTerminalMap.end()){
                                        neighborTerminalMap.erase(it);
                                        EV << "Satellite " << mySatAddress << " deleted satellite " << pk->getSrcAddr() << "'s terminal " << pk->terminalList[i] << endl;
                                    }
                                }
                            }

                            // Get a list of all active links
                            std::list<int> lst;
                            for(int i = 0; i < rtable.size(); i++){
                                lst.push_back(rtable[i]);
                            }
                            lst.sort(); // Necessary for lst.unique() to work properly
                            lst.unique();

                            // Broadcast terminal_list to all active links
                            std::list<int>::iterator last = --lst.end(); // Last element in list

                            for(std::list<int>::iterator it = lst.begin(); it != lst.end(); it++){
                                if (it != last)
                                    sendMessage(pk->dup(), *it, false);
                                else
                                    sendMessage(pk, *it, false);
                            }
                        }
                        else{
                            //// I'm source of the broadcast
                            delete pk;
                        }

                        break;
                    }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        // lll  default: destAddr != mySatAddress
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    default :{
                        RoutingTable::iterator it = rtable.find(destAddr);
                        if (it == rtable.end()){
                            if (DEBUG){
                                EV << "address " << destAddr << " unreachable, discarding packet " << pk->getName() << endl;
                            }

                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            return;
                        }
                        else if (currTopoID == pk->getTopologyID()){
                            for( int i=0;i<thisNode->getNumInLinks();i++){
                                if( thisNode->getLinkIn(i)->getLocalGateId() ==pk->getArrivalGateId()){
                                    linkCounter[i] += 1;
                                }
                            }
                            int outGateIndex = (*it).second;

                            // Add edge weight
                            emit(outputIfSignal, outGateIndex);
                           // send(pk, "out", outGateIndex);
                            delayTime = 0;//exponential(0.01);
                            sendDelayed(pk,delayTime,"out",outGateIndex);
                            return;
                        }
                        else{
                            emit(dropSignal, (long)pk->getByteLength());
                            if (pk->getTopologyVarForUpdate())
                                pk->deleteTopologyVar();
                            delete pk;
                            return;
                         }
                    }break;
                }//end switch case incoming message
            } // End of else


            break;  // End of legacy code
        }
        case terminal:{

            //// Kind = 1 -> new code for terminal support. [TerminalMsg] messages go here
            //positionAtGrid = false;
            if (!positionAtGrid){
                // In order to position satellite at grid's position we need to create XML files for every node
                cDisplayString *myDispStr = &getParentModule()->getSubmodule("grid")->getThisPtr()->getDisplayString();
                double posX, posY;
                std::string dispStr(myDispStr->str());                   // Read display string as string, string format is t=p: (posX\, posY\, 0) m\n ...
                std::size_t pos = dispStr.find_first_of("0123456789");  // find position of the first number

                dispStr = dispStr.substr(pos);                          // Cut string until the first number
                posX = std::stod(dispStr, &pos);                        // Extract number as double. This is the X position. Also, update [pos] the the
                                                                        //      location AFTER the first double

                dispStr = dispStr.substr(pos);                          // Remove the X position from string

                pos = dispStr.find_first_of("0123456789");              // find position of the first number
                dispStr = dispStr.substr(pos);                          // Cut string until the first number
                posY = std::stod(dispStr);                              // Extract number as double. This is the Y position

                // Print position to copy to [coordinates.txt] file, and run [buildXMLFiles.py]
                std::cout << "Satellite " << mySatAddress << " pos: (" << posX << ", " << posY << ")" << endl;
                if (mySatAddress == NUMOFNODES-1){
                    endSimulation();
                }
                return;
            }

            TerminalMsg *terMsg = check_and_cast<TerminalMsg*>(msg);
            switch (terMsg->getPacketType()){
                case terminal_message:{
                    //// Received message from a terminal

                    Packet* pk = createPacketForDestinationSatellite(terMsg);
                    if (pk){
                        // Packet created successfully
                        scheduleAt(simTime(), pk);
                        packetCounter++;
                    }
                    else{
                        // No satellite will be connected to target terminal
                        packetDropCounter++;
                        delete terMsg;
                    }
                    break;
                }
                case terminal_connect:{
                    //// Terminal wishes to connect
                    /* NOTES: 1. In this case the destination address is ignored
                     *        2. The source must be the correct address (for terminal table).
                     */

                    // Find if there is an open position in array and prepare target's gate name
                    int assignedIndex = -1, mode = -1;
                    std::string targetGate;
                    for(int i = 0; i < gateSize("terminalIn"); i++)
                        if (!isDirectPortTaken[i]){
                            assignedIndex = i;      // Index in array

                            mode = terMsg->getMode();
                            switch (mode){
                                case main:{
                                    isDirectPortTaken[i] = 1;   // Raise a flag to know that this index is taken
                                    targetGate = "mainIn";      // Find which gate to send replies/messages
                                    break;
                                }
                                case sub:{
                                    isDirectPortTaken[i] = 2;
                                    targetGate = "subIn";
                                    break;
                                }
                            }

                            myTerminalMap[terMsg->getSrcAddr()] = assignedIndex;    // Update my table
                            break;
                        }

                    // Send reply
                    /* NOTES: 1. If assignedIndex==-1 the terminal would know it had no open position.
                     *        2. Byte length is 0 - This message is auxiliary for simulation, not a real message
                     *        3. Duration of transmission is also 0, for above reason
                     * */
                    TerminalMsg * ansMsg = new TerminalMsg("Satellite reply - index for terminal", terminal);
                    ansMsg->setSrcAddr(mySatAddress);
                    ansMsg->setDestAddr(terMsg->getSrcAddr());
                    ansMsg->setPacketType(terminal_index_assign);
                    ansMsg->setReplyType(assignedIndex);
                    ansMsg->setMode(mode);
                    ansMsg->setByteLength(0);
                    sendDirect(ansMsg, 0, 0,getParentModule()->getParentModule()->getSubmodule("terminal", terMsg->getSrcAddr()),targetGate.c_str());

                    // Notify neighbors
                    broadcastTerminalStatus(terMsg->getSrcAddr(), connected);

                    delete terMsg;
                    break;
                }
                case terminal_disconnect:{
                    //// Terminal wishes to disconnect

                    // Check if source exists in table and delete it
                    RoutingTable::iterator it;
                    it = myTerminalMap.find(terMsg->getSrcAddr());
                    if (it != myTerminalMap.end()){
                        isDirectPortTaken[(*it).second] = 0;        // Release port
                        myTerminalMap.erase(it);
                    }

                    EV << "Satellite " << mySatAddress << " has forgotten terminal " << terMsg->getSrcAddr() << " completely" << endl;

                    // Notify neighbors
                    broadcastTerminalStatus(terMsg->getSrcAddr(), disconnected);

                    delete terMsg;
                    break;
                }
                case terminal_index_assign:{
                    //// Error: this is a message from a satellite to a terminal
                    delete msg;
                    std::cout << "Error! A satellite received handshake packet that needs to be sent to a terminal!" << endl;
                    endSimulation();
                    break;
                }
            }

            break;  // End of terminal related code
        }
        case ack:{
            //// [AckMsg] goes here.

            AckMsg* ackMsg = check_and_cast<AckMsg*>(msg);
            int linkIndex = ackMsg->getArrivalGate()->getIndex();

            MessageIDSet::iterator it = msgSet.find(ackMsg->getAckId());
            if (it != msgSet.end()){
                // Found the message that is ACK'ed

                // Remove it from the ID set
                msgSet.erase(it);
                EV << "Satellite " << ackMsg->getSrc() << " ACK'ed message ID: " << (*it) << " (Satellite " << mySatAddress << "). Time " << simTime().dbl() << endl;

                // Update stored delay
                updateLinkDelay(linkIndex, ackMsg->getDelaySuffered());
            }
            else{
                // Received ACK for a message that is not mine
                std::cout << endl << "Received bad ACK" << endl;
                exit(1);
            }

            delete msg;
            break;  // End of link delay prediction code
        }
    }

}

void Routing::sendAck(Packet *pk){
    /* Create and send an ACK message with total amount of delay on link
     *      ACK is sent to the gate where [pk] came from.
     * NOTE: Assuming the delays are symmetric (i.e. delay from X to Y = delay from Y to X)
     * */

    int linkIndex = pk->getArrivalGate()->getIndex();
    std::string ackName = "ACK on " + std::to_string(pk->getId());
    AckMsg *ackMsg = new AckMsg(ackName.c_str(), ack);
    ackMsg->setByteLength(22 + ACK_SIZE);
    ackMsg->setDelaySuffered(simTime().dbl() - pk->getLastHopTime());
    ackMsg->setAckId(pk->getId());
    ackMsg->setSrc(mySatAddress);

    // Update delay (Assuming symmetry)
    updateLinkDelay(linkIndex,simTime().dbl() - pk->getLastHopTime());

    send(ackMsg, "out", linkIndex);
}

double Routing::getAverageLinkDelay(){
    /* Return the average of all link delays
     * */

    double res = 0;
    for(int i = 0; i < MAXAMOUNTOFLINKS; i++){
        // Sum of all delays (not averages)
        res += linkDelay[i] * linkMsgNum[i];
    }

    return res? res/totalMsgNum : DEFAULT_DELAY;
}

void Routing::getSatellitePosition(int satAddress, double &posX, double &posY){
        /* Find the (X,Y) position of the a given satellite [satAddress].
         * The position is extracted from the mobility module.
         * Position is stored in given [posX] & [posY]
         * */

    // Read position
    inet::IMobility *mob = check_and_cast<inet::IMobility*>(getParentModule()->getParentModule()->getSubmodule("rte", satAddress)->getSubmodule("mobility"));
    posX = mob->getCurrentPosition().getX();
    posY = mob->getCurrentPosition().getY();

}

void Routing::updateLinkDelay(int linkIndex, double delay){
    /* Update the [linkIndex]-th average delay
     *
     *  Denote: old delay = T'
     *          new delay = T
     *          number of messages in link i (after increment) = n_i
     *          total number of messages = n
     *          delay suffered (ACK content) = D
     *
     *          Idea: if old average is T' = Sum[a_i, {i, 1, n_i - 1}]/(n_i - 1) then
     *                 new average is T = Sum[a_i, {i, 1, n_i}] = Sum[a_i, {i, 1, n_i - 1}]/n_i + a_n_i/n_i
     *                                  = Sum[a_i, {i, 1, n_i - 1}]/n_i * (n_i - 1)/(n_i - 1) + a_n_i/n_i
     *                                  = T' * (n_i - 1)/n_i + a_n_i/n_i = T' * (n_i - 1)/n_i + D * 1/n_i
     *                                  = [T' (n_i - 1) + D]/n_i
     *
     *          This calculation method was selected as it erases the default value as soon as a new value is found
     * */

    linkMsgNum[linkIndex]++;
    totalMsgNum++;
    linkDelay[linkIndex] = (linkDelay[linkIndex] * (linkMsgNum[linkIndex]-1) + delay)/linkMsgNum[linkIndex];
}

int Routing::getClosestSatellite(int terminalAddress, int *numOfHops, double estimatedTime){
    /* This function looks for the closest satellite to the terminal (referred with [terminalAddress]) at t = simTime() + [estimatedTime]
     * This function also returns the number of hops from this satellite to the closest satellite (value is returned via [numOfHops])
     *      if numOfHosts is not [nullptr]. In addition, if [estimatedTime] is 0 (Looking for number of hops)
     *      the function will print the estimated path + the number of hops
     *
     * */

    TerminalApp* terApp = check_and_cast<TerminalApp*>(getParentModule()->getParentModule()->getSubmodule("terminal", terminalAddress)->getSubmodule("app"));
    double terminalPosX, terminalPosY, satPosX, satPosY;
    double maxX, minX, maxY, minY;
    maxX = getParentModule()->getParentModule()->par("maxX").doubleValue();
    minX = getParentModule()->getParentModule()->par("minX").doubleValue();
    maxY = getParentModule()->getParentModule()->par("maxY").doubleValue();
    minY = getParentModule()->getParentModule()->par("minY").doubleValue();

    // Get target terminal position
    terApp->getPos(terminalPosX, terminalPosY);
    double radius = terApp->getRadius();

    // Find minimum distance satellite
    double minDistance = DBL_MAX;
    int satAddress = -1;
    for(int i = 0; i < num_of_hosts; i++){
        getSatellitePosition(i, satPosX,satPosY);

        /* Calculate new position. If the new position is outside of range, subtract the range
         *      until the position is inside again ("Completing loops")
         */
        satPosY += estimatedTime * SATELLITE_SPEED; //TODO: maybe use getCurrentVelocity()?
        while (satPosY > maxY){
            satPosY -= (maxY - minY);
        }

        // Save minimum distance and result
        double dist = sqrt(pow(terminalPosX-satPosX,2)+pow(terminalPosY-satPosY,2));
        if (dist < minDistance && dist <= radius){
            minDistance = dist;
            satAddress = i;
        }
    }

    // Calculate how many hops are needed
    if (numOfHops && satAddress != -1){
        //// pointer received, save number of hops to it

        // Get topology & nodes
        cTopology *tmpTopo = CreateTopology(topo, "traversal topology");
        cTopology::Node *targetNode = tmpTopo->getNode(satAddress), *curr;
        tmpTopo->calculateWeightedSingleShortestPathsTo(targetNode);
        curr = tmpTopo->getNodeFor(getParentModule());

        // Route to closest satellite - source print
        if (!estimatedTime)
            EV << "Route to closest satellite: " << mySatAddress;

        // Traverse topology, keep track of hops
        int hops = 0;
        while (curr != targetNode){
            // Move to next node
            cTopology::LinkOut *tempLinkOut = curr->getPath(satAddress);
            if (tempLinkOut){
                curr = tempLinkOut->getRemoteNode();
                hops++;

                // Route to closest satellite - inner + end of path print
                if (!estimatedTime)
                    EV << "->" << check_and_cast<Routing*>(curr->getModule()->getSubmodule("routing"))->getAddress();
            }
            else{
                *numOfHops = -1;
                return -1;
            }
        }

        // Print total number of hops estimated
        if (!estimatedTime)
            EV << ". Total hops: " << hops << endl;

        // Save result
        *numOfHops = hops;

        // Clean up
        delete tmpTopo;
    }

    return satAddress;
}
