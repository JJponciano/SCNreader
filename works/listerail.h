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

#include "railcluster.h"
#include "regionsmanager.h"
#include <QHash>
class ListeRail
{
public:
    ListeRail();
    ListeRail(int maxSize);
    //ListeRail(QVector <PointGL *> cloud, int maxSize);
    ListeRail(QVector <PointGL> cloud, int maxSize);
    ~ListeRail();

    /**
    * @brief addRail add a rail and test if the rail contains a switch
    * @param rail rail to be added
    * @return true if the oldest track is removed
    */
    bool addRail(RailCluster rail);
    /**
     * @brief growingRegions do growing regions with the rail
     * @param rail rail added to growing regions
     * @return true if a swtich is detected
     */
    bool growingRegions(RailCluster rail);



    //accesseur
    QVector<PointGL> getCloud()const;

    int getNumberSwitchDetected() const;
    QVector<int> getSwitchDetected() const;
    void setSwitchDetected(const QVector<int> &value);

    QVector<RailCluster> getLesRails() const;
    void setLesRails(const QVector<RailCluster> &value);


    int getMaxSize() const;
    void setMaxSize(int value);

    /**
     * @brief clear reinit ListeRail
     */
    void clear();

    RegionsManager getRegions() const;
    void setRegions(const RegionsManager &value);

private:
    int maxSize;
    int epsilon;// degres of precision
    QVector <int>switchDetected;///< list of the footpulste with switch
    QVector<RailCluster> lesRails;///< all rails
   RegionsManager regions;///<regions detected
    /**
     * @brief run is the treatment to detect switchs
     */
    void run();

//    /**
//     * @brief isInRegion test  if the point belongs to the region
//     * @param reg regions to be tested
//     * @param pt pt to be tested if is in reg.
//     * @return true if the point belongs to the region
//     */
//    bool isInRegion(const QVector<PointGL> reg, PointGL  pt) const;
//    /**
//     * @brief growingOk test if the region is not too big after this adding.
//     * @param reg region to be tested
//     * @return true if the regions is not too big
//     */
//    bool growingOk(const QVector<PointGL> reg) const;
//    /**
//     * @brief split split a region in two regions
//     * @param regindex index of the region to be splited
//     */
//    void split(int regindex);



//    bool emptyRegion(QVector<int> countRegions);
//    QVector<int> getRegions(PointGL currentPoint);
    void denoising();
    void initialization(QVector<PointGL> cloud);
    QVector<QVector<PointGL> > spitX(QVector<PointGL> points);
    QVector<PointGL> cleanFailPoints(QVector<QVector<PointGL> > points);
    QHash<int,int> fillFrequencyHeight(QVector<PointGL> pointsX);
    int searchCommonHeight(QHash<int, int> freqs);
    QVector<PointGL> addByHeight(QVector<PointGL> pointsX, double height);
    void initRegions();
};

#endif // LISTERAIL_H
