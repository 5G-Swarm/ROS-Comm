#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "tcpinterface.h"

class Communication : public TCPInterface{
public:
    Communication(const QString &role, const QString &id=QString("000000"));
    ~Communication() {};
private:
    QString role;
    virtual void parseData();
};

#endif // COMMUNICATION_H
