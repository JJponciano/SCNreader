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
#include "regiongrowing.h"

RegionGrowing::RegionGrowing()
{
    this->isdead=false;
    this->maxSize=500;
    this->ID=0;
    this->neighborsDistance=0.4;

}
RegionGrowing::RegionGrowing(const RegionGrowing &orig){
    this->ID=orig.getID();
    this->isdead=orig.getIsdead();
    for(int i=0;i<orig.getPoints().size();i++)
        this->points.push_back(orig.getPoints().at(i));
    this->neighborsDistance=orig.getNeighborsDistance();
    this->maxSize=orig.getMaxSize();
}

RegionGrowing::RegionGrowing(int ID, int maxSize, double neighborsDistance)
{
    this->isdead=false;
    this->maxSize=maxSize;
    this->ID=ID;
    this->neighborsDistance=neighborsDistance;

}
void RegionGrowing::clear(){
    this->points.clear();
}

void RegionGrowing::add(PointGL point)
{
    if(this->isdead)throw Erreur("adding in a dead region");
    // add the rail
    this->points.push_back(point);
    //---sort by z then by x then by y
    //std::sort(this->points.begin(),this->points.end());
    //test if the size is too big
    //    if(this->points.size()>=this->maxSize)
    //    {
    //        this->points.remove(0);
    //    }
}
bool RegionGrowing::growing(PointGL point){
    if(this->isdead)return false;
    //if the region have not a point
    if(this->size()==0){
        // the point is added
        this->add(point);
        return true;
    }else{
        // test is the point could be added to the region and add it if is possible
        if(this->isIn(point)){
            // add the point
            this->add(point);
            return true;
        }else
            //the point is not added
            return false;
    }
}
QVector<PointGL> RegionGrowing::getPoints() const
{
    return points;
}

void RegionGrowing::setPoints(const QVector<PointGL> &value)
{
    points = value;
}
double RegionGrowing::getIsdead() const
{
    return isdead;
}

void RegionGrowing::setIsdead(double value)
{
    isdead = value;
}
double RegionGrowing::getNeighborsDistance() const
{
    return neighborsDistance;
}

void RegionGrowing::setNeighborsDistance(double value)
{
    neighborsDistance = value;
}
int RegionGrowing::getMaxSize() const
{
    return maxSize;
}

void RegionGrowing::setMaxSize(int value)
{
    maxSize = value;
}





int RegionGrowing::getID() const
{
    return ID;
}
int RegionGrowing::size()const{
    return this->points.size();
}

bool RegionGrowing::isIn(PointGL pt,double distanceMax)const
{
    if(this->isdead)return false;
    //for each point of the region
    for(int i=points.size()-this->maxSize;i<points.size();i++){
        if(i<0)i=0;
        //test if the points avec the same width with and height the point to be tested
        if(pt.distanceX(this->points.at(i),distanceMax))
            return true;
    }
    return false;

}
bool RegionGrowing::isIn(PointGL pt)const
{
    if(this->isdead)return false;
    //for each point of the region
    for(int i=points.size()-this->maxSize;i<points.size();i++){
        if(i<0)i=0;
        //test if the points avec the same width with and height the point to be tested
        if(pt.distanceX(this->points.at(i),this->neighborsDistance))
            return true;
    }
    return false;

}

bool RegionGrowing::check(double widthMax)const
{
    //search the extremum of the x coordinates in the region
    //search the extremum of the x coordinates in the region
    double xmin=this->points.at(0).getX();
    double xmax=this->points.at(0).getX();
    for(int i=points.size()-this->maxSize;i<points.size();i++){
        if(i<0)i=0;
        if(this->points.at(i).getX()<xmin)
            xmin=this->points.at(i).getX();
        else    if(this->points.at(i).getX()>xmax)
            xmax=this->points.at(i).getX();
    }
    //compare the distance between the extremums with the maximum authorized.
    return (xmax-xmin)<widthMax;
}

RegionGrowing::~RegionGrowing()
{
    this->points.clear();
}

bool RegionGrowing::operator==(const RegionGrowing &r)
{
    return r.getID()==this->getID();
}

