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
      * @param widch Average width of railway rail
      * @param spacing Average spacing between two railways rail
      */
     RailCluster(float height, float widch, float spacing);
     /**
      * @brief RailCluster constructor
      * Initialization of parameters and get all point of railways rail
      * @param height Average height of railway rail
      * @param widch Average width of railway rail
      * @param spacing Average spacing between two railways rail
      */
     RailCluster(float height, float widch, float spacing, QVector <pcl::PointXYZ *>footpulse);
    ~RailCluster();
    /**
     * @brief add Add points with the required criteria
     *  This function tests for each point of pts if it is into a bounding box
     *  defined by the height and width of a railway rail and also by the spacing
     *  between two railways rail. So, if it is inside, it is added .
     * @param pts list of point to test and add
     */
    void add(QVector <pcl::PointXYZ *> pts);

    /**
     * @brief add Add point with the required criteria
     *  This function tests if the point is into a bounding box
     *  defined by the height and width of a railway rail and also by the spacing
     *  between two railways rail. So, if it is inside, it is added .
     * @param pt point to test and add
     */
    void add(pcl::PointXYZ * pt);

    /**
     * @brief remove remove a point of the list
     * @param pt point to remove
     */
    void remove(pcl::PointXYZ * pt);

    /**
     * @brief haveCorresponding  matches the points in pairs and adds or removes points.
     * This function matches the points in pairs and if is necessary, it adds a point of the pair.
     * Then, it removes and adds of the blacklist all single point. For to match points two by two,
     * it tests for each point of this class if the point has a distance close to the average
     * spacing of railway railand with another point into the list given close to and test if it has a similar height.
     * @param pts list of point to match
     */
    void match(QVector <pcl::PointXYZ *> pts);

    /**
     * @brief addSimilarePoint Add the same point to the point already added
     * This function tests for each point of pts if a point is near another point previously added
     * and having a same height.
     * @param pts list of point to add
     */
    void addSimilarePoint(QVector <pcl::PointXYZ *> pts);
    /**
     * @brief growing
     * @param rail
     * @param pts
     */
    void growing(RailCluster rail, QVector <pcl::PointXYZ *> pts);

    //accesseur en lecture et en ecriture
    void setEm(float e);
    void setHm(float h);
    void setLm(float l);
    float getEm();
    float getHm();
    float getLm();

private:
    float em; ///< Average spacing between two railways rail
    float hm; ///< Average height of railway rail
    float lm; ///< Average width of railway rail
    QVector <pcl::PointXYZ *> points;///< points of railway rail
    QVector <pcl::PointXYZ *> blacklist;///< points tested and removed
};

#endif // RAILCLUSTER_H
