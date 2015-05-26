/**
 * @file region.cpp
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
#include "region.h"

Region::Region()
{
    this->maxSize=500;
    this->ID=0;
    this->neighborsDistance=0.4f;

}
Region::Region(int ID, int maxSize, float neighborsDistance)
{
    this->maxSize=maxSize;
    this->ID=ID;
    this->neighborsDistance=neighborsDistance;

}
void Region::clear(){
    this->points.clear();
}

void Region::add(PointGL point)
{
    // add the rail
    this->points.push_back(point);
    //test if the size is too big
    if(this->points.size()>=this->maxSize)
    {
        this->points.remove(0);
    }
}
bool Region::growing(PointGL point){
    if(this->size()==0){
        this->add(point);
        return true;
    }else{
        //for each point of the first region
        bool added=false;
        int i=0;
        // test is the point could be added to the first region and add it if is possible
        while(i<this->size()&&!added){
            PointGL currentPoint=this->points.at(i);
            //test if the points avec the same width with and height the point to be tested
            if(point.distanceX(currentPoint,this->neighborsDistance)&&point.distanceY(currentPoint,this->neighborsDistance)){
                // add the point and stop the loop
                this->add(point);
                added=true;
            }else
                i++;
        }
        return added;
    }
}
QVector<PointGL> Region::getPoints() const
{
    return points;
}

void Region::setPoints(const QVector<PointGL> &value)
{
    points = value;
}


int Region::getID() const
{
    return ID;
}
int Region::size(){
    return this->points.size();
}

bool Region::isIn(PointGL pt,float distanceMax)const
{
    //for each point of the region
    for(int i=0;i<points.size();i++){
        //test if the points avec the same width with and height the point to be tested
        if(pt.distanceX(this->points.at(i),distanceMax))
            return true;
    }
    return false;

}

bool Region::growingOk(float widthMax)const
{
    //search the extremum of the x coordinates in the region
    //search the extremum of the x coordinates in the region
    float xmin=this->points.at(0).getX();
    float xmax=this->points.at(0).getX();
    for(int i=1;i<this->points.size();i++){
        if(this->points.at(i).getX()<xmin)
            xmin=this->points.at(i).getX();
        else    if(this->points.at(i).getX()>xmax)
            xmax=this->points.at(i).getX();
    }
    //compare the distance between the extremums with the maximum authorized.
    return (xmax-xmin)<widthMax;
}





Region::~Region()
{
    this->points.clear();
}

bool Region::operator==(const Region &r)
{
    return r.getID()==this->getID();
}

