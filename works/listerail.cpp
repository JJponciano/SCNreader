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

}

bool growingRegions(RailCluster rail)
{

}

bool growingOk(QVector <pcl::PointXYZ *> reg)
{

}

bool isInRegion(QVector <pcl::PointXYZ *> reg, pcl::PointXYZ * pt)
{

}


QVector <pcl::PointXYZ *> getRegions()
{
    return this->regions;
}

QVector <RailCluster> getLesRails()
{
    return this->lesRails;
}
