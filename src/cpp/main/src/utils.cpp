#include "utils.h"
#include <QJsonObject>
#include <QJsonDocument>

QByteArray toMessage(const QString &data, const QString &robot_id, const QString &mtype, const QString &dest_id, const int &pri){
    QJsonObject object;
    object.insert("Mtype",mtype);
    object.insert("Pri",pri);
    object.insert("Id",robot_id);
    object.insert("Dest",dest_id);
    object.insert("Data",data);
    QJsonDocument doc;
    doc.setObject(object);
    QByteArray bytes = doc.toJson(QJsonDocument::Compact);
    return bytes;
}
