/**
 * @file region.h
 * @brief file use for growing regions algorithms
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
#ifndef REGION_H
#define REGION_H
#include <QVector>
#include "../modules/exceptions/erreur.h"
#include "../modules/openGL/tools_models/Point.h"
class Region
{
public:
    Region();
    /**
      * @brief Region
      * @param ID identifier of the region
      * @param maxSize maximum number of points that the region may contain
      * @param neighborsDistance distance between neighbors
      */
     Region(int ID,int maxSize,float neighborsDistance);
     ~Region();
      bool operator==(Region const& r);
     /**
      * @brief clear remove all points
      */
     void clear();
     /**
      * @brief add add a point without using the distance criterion between neighbors
      * @param point point to be added
      */
     void add(PointGL point);
     int getID() const;
     /**
      * @brief isIn test if a point is in the region
      * @param pt point to be tested
      * @param distanceMax defined the maximum distance between two point for they belong to the region
      * @return true if the point is in regions
      */
     bool isIn(PointGL pt, float distanceMax) const;
     /**
      * @brief isIn test if a point is in the region with use the neightbors distance
      * @param pt point to be tested
      * @return  true if the point is in the region
      */
     bool isIn(PointGL pt) const;
     /**
      * @brief size number of point into the region
      * @return  number of point into the region
      */
     int size() const;
     /**
      * @brief growingOk test if the region is not too big
      * @param widthMax  max width of the region
      * @return true if the regions is not too big
      */
     bool check(float widthMax) const;
     /**
      * @brief growing add point using the distance criterion between neighbors
      * @param point point to be added
      * @return true if the point is added
      */
     bool growing(PointGL point);
     QVector<PointGL> getPoints() const;
     void setPoints(const QVector<PointGL> &value);

     float getIsdead() const;
     void setIsdead(float value);

private:
     float isdead;
     float neighborsDistance;
     int ID;
     QVector <PointGL>points;
     int maxSize;
};

#endif // REGION_H
