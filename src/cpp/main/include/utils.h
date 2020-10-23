#include <QObject>

QByteArray toMessage(const QString &data, const QString &robot_id, const QString &mtype, const QString &dest_id=QString("0"), const int &pri=5);
