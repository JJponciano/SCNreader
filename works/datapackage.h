#ifndef DATAPACKAGE_H
#define DATAPACKAGE_H

#include <QByteArray>
#include <QString>
#include <QDataStream>

#include "../modules/exceptions/erreur.h"
#include "../modules/pcl/ground/ToolsPCL.h"
#include<QProgressDialog>
#include <QVector>
class Datapackage
{
public:

    Datapackage();
    ~Datapackage();

    Datapackage(QByteArray datas,int start);
    void readDataFile(std::string pathname);
    friend std::ostream& operator << (std::ostream& O, const Datapackage& B);
std::string toString();
       virtual void Print(std::ostream& O) const;


    // getter and setter

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() const;
    void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &value);

    QString getId() const;
    void setId(const QString &value);

    QString getVersion() const;
    void setVersion(const QString &value);

    int getRadius() const;
    void setRadius(const int &value);

    int getRadienkorrektur() const;
    void setRadienkorrektur(const int &value);

    int getSpurweite() const;
    void setSpurweite(const int &value);

    int getNglLeftHorizontal() const;
    void setNglLeftHorizontal(const int &value);

    int getNglLeftVertical() const;
    void setNglLeftVertical(const int &value);

    int getNglLeftBanking() const;
    void setNglLeftBanking(const int &value);

    int getNglLeftGauge() const;
    void setNglLeftGauge(const int &value);

    int getNglRightHorizontal() const;
    void setNglRightHorizontal(const int &value);

    int getNglRightVertical() const;
    void setNglRightVertical(const int &value);

    int getNglRightBanking() const;
    void setNglRightBanking(const int &value);

    int getNglRightGauge() const;
    void setNglRightGauge(const int &value);

    int getPointCount() const;
    void setPointCount(const int &value);

    int getEnd() const;
    void setEnd(int value);

    int getStructuresize() const;
    void setStructuresize(int value);

    int getFootpulse() const;
    void setFootpulse(int value);

    QVector<int> getDistance() const;
    void setDistance(const QVector<int> &value);

    QVector<int> getIntensity() const;
    void setIntensity(const QVector<int> &value);

private:
    int bytesToInt(const char *buffer, int size);
    int bytesToInt(QByteArray arbuffer, int size, char *buffer);
    void readData(QByteArray datas, int start);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    QString id;
    QString version;
    int radius;
    QVector<int> distance;
     QVector<int> intensity;
    int radienkorrektur;
    int spurweite;
    int nglLeftHorizontal;
    int nglLeftVertical;
    int nglLeftBanking;
    int nglLeftGauge;
    int nglRightHorizontal;
    int nglRightVertical;
    int nglRightBanking;
    int nglRightGauge;
    int pointCount;
    int end;
    int structuresize;
    int footpulse;
};

#endif // DATAPACKAGE_H
