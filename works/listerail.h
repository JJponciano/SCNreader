#ifndef LISTERAIL_H
#define LISTERAIL_H

#include <pcl/point_types.h>
#include <QVector>
#include "railcluster.h"

class ListeRail
{
public:
    ListeRail();
    ~ListeRail();

    /**
     * @brief switchDetected detect a switch and gives footpulses corresponding of this switch
     * @return the vector which contains all footpulses corresponding to a switch
     */
    QVector <int> switchDetected();
    /**
     * @brief growingRegions
     * @param rail
     * @return
     */
    bool growingRegions(RailCluster rail);
    /**
     * @brief growingOk
     * @param reg
     * @return
     */
    bool growingOk(QVector <pcl::PointXYZ *> reg);
    /**
     * @brief isInRegion
     * @param reg
     * @param pt
     * @return
     */
    bool isInRegion(QVector <pcl::PointXYZ *> reg, pcl::PointXYZ * pt);

    //accesseur
    QVector <pcl::PointXYZ *> getRegions();
    QVector <RailCluster> getLesRails();


private:
    QVector <RailCluster> lesRails;
    QVector <pcl::PointXYZ *> regions;

};

#endif // LISTERAIL_H
