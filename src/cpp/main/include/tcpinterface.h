#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H

#include <thread>
#include <QObject>
#include <QTcpSocket>

class TCPInterface : public QObject {
    Q_OBJECT
public:
    TCPInterface(const QString &regInfo, const QString &ip=QString("127.0.0.1"), const int &port=10001, const QString &id=QString("000000"), QObject *parent = Q_NULLPTR);
    bool setup(const int &port, const QString &address);
    void register2pub(const QString& regInfo);
    void startTempThread();
    void waitForConnect();
    void send(const QString& regInfo, const QString& type=QString("message"), const QString& dest_id=QString("000000"));
    QTcpSocket *socket = nullptr;
    QString ip;
    int port;
    QString publicIP;
    QString robot_id;
    int publicPort;
    bool connected = false;
    std::thread* tempThread = nullptr;
public slots:
    virtual void parseData(){};
};

#endif // TCPINTERFACE_H
