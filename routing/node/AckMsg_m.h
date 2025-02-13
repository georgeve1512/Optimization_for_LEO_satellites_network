//
// Generated file, do not edit! Created by nedtool 5.6 from node/AckMsg.msg.
//

#ifndef __ACKMSG_M_H
#define __ACKMSG_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0506
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



class AckMsg;
/**
 * Class generated from <tt>node/AckMsg.msg:19</tt> by nedtool.
 * <pre>
 * //
 * // TODO generated message class
 * //
 * packet AckMsg
 * {
 *     int src;				// Satellite that sent ACK
 *     long ackId;				// Which message is ACK'ed
 *     double delaySuffered;	// Total delay the packed suffered
 * }
 * </pre>
 */
class AckMsg : public ::omnetpp::cPacket
{
  protected:
    int src = 0;
    long ackId = 0;
    double delaySuffered = 0;

  private:
    void copy(const AckMsg& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const AckMsg&);

  public:
    AckMsg(const char *name=nullptr, short kind=0);
    AckMsg(const AckMsg& other);
    virtual ~AckMsg();
    AckMsg& operator=(const AckMsg& other);
    virtual AckMsg *dup() const override {return new AckMsg(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getSrc() const;
    virtual void setSrc(int src);
    virtual long getAckId() const;
    virtual void setAckId(long ackId);
    virtual double getDelaySuffered() const;
    virtual void setDelaySuffered(double delaySuffered);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const AckMsg& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, AckMsg& obj) {obj.parsimUnpack(b);}

#endif // ifndef __ACKMSG_M_H

