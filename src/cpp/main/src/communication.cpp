#include "communication.h"
#include <QDebug>

namespace {
    const bool DEBUG = true;
    const QString IP_ADDRESS = QString("127.0.0.1");
    const int PORT = 10001;
};

Communication::Communication(const QString &role, const QString &id) : TCPInterface(role, ::IP_ADDRESS, ::PORT, id){
    this->role = role;
}

void Communication::parseData(){
    QByteArray receivedData = socket->readAll();
    auto length = receivedData.length();
    if(DEBUG) qDebug() << "length" << length << receivedData;
}
