#include <QCoreApplication>
#include <QDebug>
#include "communication.h"
#include <thread>

namespace  {
    const bool isServer = true;
};

int main(int argc, char *argv[]){
    QCoreApplication a(argc, argv);

    auto role = isServer ? QString("server") : QString("client");
    auto id = isServer ? QString("111111") : QString("222222");

    Communication comm(role, id);
    for(int i =0;; i++){
        if(::isServer) comm.send(QString("654321"), QString("vision"), QString("222222"));
        else comm.send(QString("123456"), QString("vision"), QString("111111"));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return a.exec();
}
