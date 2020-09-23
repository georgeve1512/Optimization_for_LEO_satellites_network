//
// Generated file, do not edit! Created by nedtool 5.6 from node/TerminalMsg.msg.
//

#ifndef __TERMINALMSG_M_H
#define __TERMINALMSG_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0506
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



class TerminalMsg;
/**
 * Class generated from <tt>node/TerminalMsg.msg:20</tt> by nedtool.
 * <pre>
 * packet TerminalMsg
 * {
 *     int srcAddr \@packetData;
 *     int destAddr \@packetData;
 *     int packetType \@packetData;
 *     int mode \@packetData;
 *     int replyType \@packetData;
 *     int hopCount;
 * }
 * </pre>
 */
class TerminalMsg : public ::omnetpp::cPacket
{
  protected:
    int srcAddr = 0;
    int destAddr = 0;
    int packetType = 0;
    int mode = 0;
    int replyType = 0;
    int hopCount = 0;

  private:
    void copy(const TerminalMsg& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const TerminalMsg&);

  public:
    TerminalMsg(const char *name=nullptr, short kind=0);
    TerminalMsg(const TerminalMsg& other);
    virtual ~TerminalMsg();
    TerminalMsg& operator=(const TerminalMsg& other);
    virtual TerminalMsg *dup() const override {return new TerminalMsg(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getSrcAddr() const;
    virtual void setSrcAddr(int srcAddr);
    virtual int getDestAddr() const;
    virtual void setDestAddr(int destAddr);
    virtual int getPacketType() const;
    virtual void setPacketType(int packetType);
    virtual int getMode() const;
    virtual void setMode(int mode);
    virtual int getReplyType() const;
    virtual void setReplyType(int replyType);
    virtual int getHopCount() const;
    virtual void setHopCount(int hopCount);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const TerminalMsg& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, TerminalMsg& obj) {obj.parsimUnpack(b);}

#endif // ifndef __TERMINALMSG_M_H

