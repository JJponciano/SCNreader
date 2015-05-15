/**
 * @file listerail.h
 * @brief file use for detected switch in a cloud
 * @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
 * All rights reserved.
 * This file is part of scn reader.
 *
 * scn reader is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * scn reader is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>
 * @author Jean-Jacques PONCIANO and Claire PRUDHOMME
 * Contact: ponciano.jeanjacques@gmail.com
 * @version 0.1
 */

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
     * @brief growingRegions do growing regions with the rail
     * @param rail rail added to growing regions
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
