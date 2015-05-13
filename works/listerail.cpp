#include "listerail.h"

ListeRail::ListeRail()
{

}

ListeRail::~ListeRail()
{
    this->lesRails.clear();
    this->regions.clear();
}

QVector <int> ListeRail::switchDetected()
{
    QVector <int>a;
return a;
}

bool ListeRail::growingRegions(RailCluster rail)
{
return true;
}

bool ListeRail::growingOk(QVector <pcl::PointXYZ *> reg)
{
return true;
}

bool ListeRail::isInRegion(QVector <pcl::PointXYZ *> reg, pcl::PointXYZ * pt)
{
return true;
}


QVector <pcl::PointXYZ *> ListeRail::getRegions()
{
    return this->regions;
}

QVector <RailCluster> ListeRail::getLesRails()
{
    return this->lesRails;
}
