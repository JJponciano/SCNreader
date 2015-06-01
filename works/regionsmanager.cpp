/**
 * @file regionsmanager.cpp
 * @brief file use for managed regions class
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
#include "regionsmanager.h"

RegionsManager::RegionsManager(int minsize, double neighborsDistance, int maxSize)
{
    this->minSize=minsize;
    this->neighborsDistance=neighborsDistance;
    this->nbregions=0;
    this->maxSize=maxSize;
}
RegionsManager::RegionsManager()
{
    this->minSize=10;
    this->neighborsDistance=0.1;
    this->nbregions=0;
    this->maxSize=500;
}

RegionsManager::~RegionsManager()
{
    this->regions.clear();
}


void RegionsManager::addInNewRegion(PointGL point){
    //create a new region for add the point
    RegionGrowing r1( this->generatingID(),this->maxSize,this->neighborsDistance);
    r1.growing(point);
    this->regions.push_back(r1);
}

bool RegionsManager::addPoint(PointGL point)
{
   // std::cout<<" Point: "<<point.getX()<<" , "<<point.getZ()<<std::endl;
    bool ok=true;
    // get all region that the point could be added
    QVector<int> idRegions=this->intoRegions(point);

    // if the point not belong to any region
    if(idRegions.isEmpty()){
     //   std::cout<<" addd new"<<std::endl;
        this->addInNewRegion(point);
    }else
        // if the point belong to only one region
        if(idRegions.size()==1){
            //it is simply added
            this->add(idRegions.at(0),point);
           // std::cout<<" addd in: "<<idRegions.at(0)<<std::endl;
        }else{
            // the point belong to more than one regions
            // remove region having merged and test if this merge wasn't too small
            // if is too small, the merge is not importante
           ok=(this->removeRegions(idRegions));
            //add it in a new region
            this->addInNewRegion(point);
        }
    // std::cout<<this->regions.size()<<std::endl;
    return ok;
}
bool RegionsManager::checkRegion(double widthmax){
    //it guess that's ok
    bool ok=true;
    //for each region
    for(int j=0;j<this->regions.size();j++)
        //test if the region is not ok
        if(!this->regions.at(j).check(widthmax)){
            ok=false;
            //if a region is too large, split it
            if(this->split(this->regions.at(j).getID())){
                //if the split have a small region, all is ok, the split is not importante
                ok=true;
            }
        }
    return ok;
}
void RegionsManager::add(int ID,PointGL point){
    RegionGrowing r(ID,this->maxSize,this->neighborsDistance);
    //test if the region is known
    if(this->regions.contains(r)){
        // add the point into the region
        int index=this->regions.lastIndexOf(r);
        if(index<0||index>=this->regions.size())
            throw Erreur("Invalid Index");
        else
            this->regions[index].add(point);
    }
    else throw Erreur(" The region is not known");
}

RegionGrowing RegionsManager::getRegion(int ID){
    RegionGrowing r(ID,this->maxSize,this->neighborsDistance);
    //test if the region is known
    if(this->regions.contains(r))
        // return the region
        return this->regions.at(this->regions.lastIndexOf(r));
    else throw Erreur(" The region is not known");
}
void RegionsManager::clear(){
    for(int j=0;j<this->regions.size();j++)
        this->regions[j].clear();
    this->regions.clear();
}

void RegionsManager::remove(int ID){
    RegionGrowing r(ID,this->maxSize,this->neighborsDistance);
    //test if the region is known
    if(this->regions.contains(r)){
        // return the region
        int index=this->regions.lastIndexOf(r);
      // this->regions.remove(index);
         this->regions[index].setIsdead(true);
    }

    else throw Erreur(" The region is not known");
}

bool RegionsManager::removeRegions(QVector<int> RegionsID){
    int smallestSize=this->getRegion(RegionsID.at(0)).size();
    //search the smallest region
    for(int j=0;j<RegionsID.size();j++)
    {
        if(this->getRegion(RegionsID.at(j)).size()<smallestSize)
            smallestSize=this->getRegion(RegionsID.at(j)).size();
        //remove region
        this->remove(RegionsID.at(j));
    }
    //if a region having merged is too small
    return(smallestSize<this->minSize);
}

QVector<int> RegionsManager::intoRegions(PointGL point)const{
    // ==== Count the number of regions which currentPoint is in===
    QVector<int> countRegions;
    // test if the point belongs to regions, and counts the number of regions
    for(int j=0;j<this->regions.size();j++)
        if(this->regions.at(j).isIn(point)){
            countRegions.push_back(this->regions.at(j).getID());//add the identifier of the regions
        }
    return countRegions;
}
int RegionsManager::generatingID(){
    this->nbregions++;
    return nbregions;
}
QVector<RegionGrowing> RegionsManager::getRegions() const
{
    return regions;
}

void RegionsManager::setRegions(const QVector<RegionGrowing> &value)
{
    regions = value;
}

bool RegionsManager::split(int regionID)
{
    //split the region
    QVector<RegionGrowing> newRegions;
    RegionGrowing r1( this->generatingID(),this->maxSize,this->neighborsDistance);
    newRegions.push_back(r1);
    //for all point of the region to be splited
    QVector<PointGL>pointsRegion=this->getRegion(regionID).getPoints();
    for(int j=1;j<pointsRegion.size();j++){
        //get the current point of the region
        PointGL pt=pointsRegion.at(j);

        //try to add the point in the first regions
        bool added=newRegions[0].growing(pt);
        int i=1;
        //while the point is not added in a region
        while(i<newRegions.size()&&!added){
            added=newRegions[i].growing(pt);
            i++;
        }
        //if the point does not added
        if(!added){
            //create a new region and add it
            RegionGrowing r2( this->generatingID(),this->maxSize,this->neighborsDistance);
            r2.growing(pt);
            newRegions.push_back(r2);
        }
    }
    // remove the regions
    this->remove(regionID);
    // add the new regions
    int smallestSize=newRegions.at(0).size();
    for(int j=0;j<newRegions.size();j++){
        if(newRegions.at(j).size()<smallestSize)
            smallestSize=newRegions.at(j).size();
        this->regions.push_back(newRegions[j]);
    }
    //if a region having split is too small
    return(smallestSize<this->minSize);
}

