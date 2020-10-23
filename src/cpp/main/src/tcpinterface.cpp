#include "tcpinterface.h"
#include "utils.h"
#include <QDebug>

namespace  {
    const bool DEBUG = false;
    const int TIMEOUT = 1000;
};

TCPInterface::TCPInterface(const QString &regInfo, const QString &ip, const int &port, const QString &id, QObject *parent): QObject(parent) {
    publicIP = ip;
    publicPort = port;
    robot_id = id;
    socket = new QTcpSocket();
    connected = setup(publicPort, publicIP);
    if(!connected) return;
    register2pub(regInfo);
    startTempThread();
}

void TCPInterface::register2pub(const QString& regInfo){
    QByteArray bytes = toMessage(regInfo, robot_id, QString("register"));
    socket->write(bytes);
    socket->waitForBytesWritten(100);
}

void TCPInterface::send(const QString& regInfo, const QString& type, const QString& dest_id){
    if(!connected) {
        qDebug() << "WARNING: TCP not connected !";
        return;
    }
    QByteArray bytes = toMessage(regInfo, robot_id, type, dest_id);
    socket->write(bytes);
    socket->waitForBytesWritten(100);
    if(DEBUG) qDebug() << "Send" << bytes;
}

bool TCPInterface::setup(const int &port, const QString &address){
    socket->connectToHost(address, port);
    if(!socket->waitForConnected(::TIMEOUT)){
        qDebug() << "Connection time out!";
        return false;
    } else {
        qDebug() << "Connected successfully" << address << ":" << port ;
        return true;
    }
}

void TCPInterface::startTempThread(){
    tempThread = new std::thread([ = ] {waitForConnect();});
    tempThread->detach();
}

void TCPInterface::waitForConnect(){
    while(true) {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        QByteArray receivedData = socket->readAll();
        if (receivedData.size() == 0) continue;
        else{
            qDebug() <<"Register successfully !" << receivedData;
            break;
        }
    }
    QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(parseData()));
    qDebug() << "Ready to receive ...";
}
