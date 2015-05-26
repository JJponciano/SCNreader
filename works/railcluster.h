/**
 * @file railcluster.h
 * @brief file use for isolate rails in a cloud
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

#ifndef RAILCLUSTER_H
#define RAILCLUSTER_H

#include "../modules/openGL/tools_models/Point.h"
#include <pcl/point_types.h>
#include <QVector>
/**
 * @brief The RailCluster class
 * This class represents all of  rails for one foot pulse
 */
class RailCluster
{
public:
    /**
     * @brief RailCluster constructor
     */
    RailCluster();
    /**
      * @brief RailCluster constructor
      * Initialization of parameters
      * @param height Average height of railway rail
      * @param width Average width of railway rail
      * @param spacing Average spacing between two railways rail
      * @param footpulse foopulse at analyse
      */
     RailCluster(float height, float width, float spacing,const QVector <PointGL *>footpulse);
     /**
      * @brief RailCluster constructor
      * Initialization of parameters and get all point of railways rail
      * @param height Average height of railway rail
      * @param width Average width of railway rail
      * @param spacing Average spacing between two railways rail
      * @param footpulse foopulse at analyse
      * @param rail railcluster used for growing regions and keep the continuity of the rails
      */
     RailCluster(float height, float width, float spacing, const QVector <PointGL *>footpulse,const RailCluster rail);
     /**
       * @brief RailCluster constructor
       * Initialization of parameters
       * @param height Average height of railway rail
       * @param width Average width of railway rail
       * @param spacing Average spacing between two railways rail
       * @param footpulse foopulse at analyse
       */
      RailCluster(float height, float width, float spacing,const QVector <pcl::PointXYZ *>footpulse);
      /**
       * @brief RailCluster constructor
       * Initialization of parameters and get all point of railways rail
       * @param height Average height of railway rail
       * @param width Average width of railway rail
       * @param spacing Average spacing between two railways rail
       * @param footpulse foopulse at analyse
       * @param rail railcluster used for growing regions and keep the continuity of the rails
       */
      RailCluster(float height, float width, float spacing, const QVector <pcl::PointXYZ *>footpulse,const RailCluster rail);


     /**
      * @brief RailCluster destructor
      */
    ~RailCluster();

     /**
      * @brief addPoint add point without other processing.
      * The point must have a same z than other points already added.
      * Otherwise, the point cannot be added
      * @param p point added
      * @return true if the point is added
      */
     bool addPoint(PointGL p);
    /**
     * @brief add Search isoled track
     *  This function tests for each point of pts if it is into a bounding box
     *  defined by the height and width of a railway rail and also by the spacing
     *  between two railways rail. So, if it is inside, it is added .
     * @param pts list of point to test and add
     */
    void add(const QVector<PointGL> pts);

    /**
     * @brief remove remove a point of the list
     * @param pt point to remove
     */
    void remove(PointGL pt);

    /**
     * @brief match  Search a cooresponding track
     * This function matches the points in pairs and if is necessary, it adds a point of the pair.
     * Then, it removes and adds of the blacklist all single point. For to match points two by two,
     * it tests for each point of this class if the point has a distance close to the average
     * spacing of railway railand with another point into the list given close to and test if it has a similar height.
     * @param pts list of point to match
     * @return bool said if a point has been added.
     */
    bool match(QVector <PointGL> pts);

    /**
     * @brief addSimilarePoint Add the same point to the point already added
     * This function tests for each point of pts if a point is near another point previously added
     * and having a same height.
     * @param pts list of point to add
     * @return bool said if a point has been added.
     */
    bool addSimilarePoint(QVector <PointGL > pts);
    /**
     * @brief growing
     * @param rail
     * @param pts
     */
    bool growing(RailCluster rail, QVector <PointGL > pts);

    /**
     * @brief sameHeight test if points have the sames height
     * @param p1 point to be tested
     * @param p2 point to be tested
     * @return true is the points have the sames height
     */
    bool sameHeight(PointGL p1, PointGL p2)const;
    /**
     * @brief sameWidth test if points have the sames width
     * @param p1 point to be tested
     * @param p2 point to be tested
     * @return true is the points have the sames width
     */
    bool sameWidth(PointGL p1,PointGL p2) const;
    /**
     * @brief widthDistance test if the distance between 2 points is below to the width of the track
     * @param p1  point to be tested
     * @param p2 point to be tested
     * @return true if the distance between 2 points is below to the width of the track
     */
    bool widthDistance( PointGL p1, PointGL p2)const;
    /**
     * @brief spacingDistance test if the distance between 2 points is below to the average spacing between two tracks
     * @param p1  point to be tested
     * @param p2 point to be tested
     * @return true if the distance between 2 points is below to the average spacing between two tracks
     */
    bool spacingDistance( PointGL p1,  PointGL p2) const;
    //accesseur en lecture et en ecriture
    void setEm(float e);
    void setHm(float h);
    void setLm(float l);
    float getEm() const;
    float getHm()const;
    float getLm()const;

    QVector<PointGL> getPoints() const;
    void setPoints(const QVector<PointGL> value);

    QVector<PointGL> getBlacklist() const;
    void setBlacklist(const QVector<PointGL > value);

    int getFootpulse() const;
    void setFootpulse(int value);
    /**
     * @brief getWidthDistance
     * @return return the distance of a track
     */
    float getWidthDistance();
    float getApproximately();
private:
float gap(QVector <PointGL> reg) const;
    int footpulse;
    float delta;///> approximation
    float em; ///< Average spacing between two railways rail
    float hm; ///< Average height of railway rail
    float lm; ///< Average width of railway rail
    QVector <PointGL > points;///< points of railway rail
    QVector <PointGL > blacklist;///< points tested and removed
    PointGL averagePoint(QVector<PointGL > reg) const;
    bool isContains(PointGL p)const;
    bool isBlackListed(PointGL p)const;
    int searchCorresponding(PointGL currentPoint, QVector<PointGL> *pts);
    QVector<PointGL> addByWindows(const QVector<PointGL> pts, int sens);
    void addByMountain(const QVector<PointGL> pts);
};

#endif // RAILCLUSTER_H
