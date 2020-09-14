#ifndef NODE_L2QUEUE_H_
#define NODE_L2QUEUE_H_

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include "Packet_m.h"
#include "enums.h"
#include <sstream>

using namespace omnetpp;

class  controlSignals : public cObject
{
  public:
  //  Packet *pk;
    int queueID;    //to whom i connected
    int ID;         //node address
    int loadOnLink;  //load on the link in the current moment of signal
    cModule *module;
    const char *SignalName;
    //cGate::Type gateType;
    //ool isVector;
    //friend class cComponent;
};
/**
 * Point-to-point interface module. While one frame is transmitted,
 * additional frames get queued up; see NED file for more info.
 */
class L2Queue : public cSimpleModule, public cListener
{
public:

  private:
    long frameCapacity;
    long sent=1;
    long recived=1;
    //cIListener queue_monitor;
   //cModule *links_control;
    cQueue queue;
    cQueue priorityQueue;
    cQueue tempQueue;
    cMessage *endTransmissionEvent;
    bool isBusy;
    int queueid;
    int myAddress;
    bool load_balance_mode;
    bool load_balance_state;
    double datarate;
    int linkusage = 0;
    int pkdropcounter = 0;
    cHistogram dropCountStats;
    cOutVector dropCountVector;
    cOutVector queueSizeVector;
    cOutVector throughput;

//    cHistogram throughput;
    simsignal_t qlenSignal;
    simsignal_t busySignal;
    simsignal_t queueingTimeSignal;
    simsignal_t dropSignal;

    simsignal_t txBytesSignal;
    simsignal_t rxBytesSignal;

    simsignal_t LBS;//load balance signal
    int queuesizesignal=0;
   // controlSignals test;
//    friend class cIListener;


  public:
    L2Queue();
    virtual ~L2Queue();
    void deleteMessageAndRecord(cMessage *msg,bool localAction = true);
  //  cSimpleModule *links_control;
   // Routing *links_control;

    int getQueueSize();

  protected:
   // friend class cIListener;
    virtual void initialize() override;

    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void startTransmitting(cMessage *msg);

    virtual void finish() override;


    //inter module connection
    //virtual void recieveSignal();
    simsignal_t  checkSignal;
    //cModule *rotuing_module;
    char* signalDeParser(int queuesize,int dst);
    void keepPing(cMessage *msg);
//    virtual void receiveSignal(cComponent *src, simsignal_t id, controlSignals *value,controlSignals *details);
//
//
 //   virtual void receiveSignal(cComponent *src, simsignal_t id, cObject *value,cObject *details)override;
    //virtual void receiveSignal(cComponent *, simsignal_t signalID, long l, cObject *);

};

Define_Module(L2Queue);




#endif /* NODE_L2QUEUE_H_ */
