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
    this->ID=0;
    this->neighborsDistance=0.4;
    this->ok=false;
    this->spans=10;

}
RegionGrowing::RegionGrowing(const RegionGrowing &orig){
    this->ID=orig.getID();
    this->isdead=orig.getIsdead();
    for(int i=0;i<orig.getPoints().size();i++)
        this->points.push_back(orig.getPoints().at(i));
    this->neighborsDistance=orig.getNeighborsDistance();
    this->ok=orig.isOk();
    this->spans=orig.getSpans();
}

RegionGrowing::RegionGrowing(int ID, int spans, double neighborsDistance)
{
    this->isdead=false;
    this->ID=ID;
    this->neighborsDistance=neighborsDistance;
    this->ok=false;
    this->spans=spans;

}
void RegionGrowing::clear(){
    this->points.clear();
}

void RegionGrowing::add(PointGL point)
{
    if(this->isdead)throw Erreur("adding in a dead region");
    // if the last point have a footpulse bigger than the point
    if(this->points.last().getZ()>point.getZ()){
        // add the rail
        this->points.push_back(point);
        // and sort by z then by x then by y
        std::sort(this->points.begin(),this->points.end());
    }else{
        // juste add it
        this->points.push_back(point);
    }
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
bool RegionGrowing::isOk() const
{
    return ok;
}

void RegionGrowing::setIsOk(bool value)
{
    ok = value;
}
int RegionGrowing::getSpans() const
{
    return spans;
}

void RegionGrowing::setSpans(int value)
{
    spans = value;
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
    //for each point of the region having a footpulse close to the point
    int count=0;
    int currentFP=this->points.at(0).getZ();
    int i=this->points.size()-1;
    while(i>=0&&count<this->spans){
        //test if the points avec the same width and height the point to be tested
        if(pt.distanceX(this->points.at(i),distanceMax))
            return true;
        i--;
        if(currentFP!=this->points.at(i).getZ()){
            currentFP=this->points.at(i).getZ();
            count++;
        }
    }
    return false;

}
bool RegionGrowing::isIn(PointGL pt) const
{
    return this->isIn(pt,this->neighborsDistance);
}

bool RegionGrowing::check(double widthMax)
{
    // if the region is dead, return the last check
    if(this->isdead)return this->ok;
    //search the extremum of the x coordinates in the region

    // calcule the avarage X
    double average=this->averageX();
    //calcul the error distance beetween all point and the average point
    double averageError=0;
    int count=0;
    int currentFP=this->points.at(0).getZ();
    int i=this->points.size()-1;
    while(i>=0&&count<this->spans){
        if(i<0)i=0;
        double dist=this->points.at(i).getX()-average;
        if(dist<0)dist*=-1;
        averageError+=dist;
        i--;
        if(currentFP!=this->points.at(i).getZ()){
            currentFP=this->points.at(i).getZ();
            count++;
        }
    }averageError/=double(this->points.size()-1-i);
    //compare the distance between the extremums with the maximum authorized.
    this->ok=(averageError)<widthMax;
    return this->ok;
}

double RegionGrowing::averageX(){
    double average=0;
    int count=0;
    int currentFP=this->points.at(0).getZ();
    int i=this->points.size()-1;
    while(i>=0&&count<this->spans){
        average+=this->points.at(i).getX();
        i--;
        if(currentFP!=this->points.at(i).getZ()){
            currentFP=this->points.at(i).getZ();
            count++;
        }
    }
    average/=double(this->points.size()-1-i);
    return average;
}

RegionGrowing::~RegionGrowing()
{
    this->points.clear();
}

bool RegionGrowing::operator==(const RegionGrowing &r)
{
    return r.getID()==this->getID();
}

