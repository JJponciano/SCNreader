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

RegionsManager::RegionsManager(float distanceMax, int minsize, float neighborsDistance, int maxSize)
{
    this->distanceMax=distanceMax;
     this->minSize=minsize;
    this->neighborsDistance=neighborsDistance;
    this->nbregions=0;
    this->maxSize=maxSize;
}
RegionsManager::RegionsManager()
{
    this->distanceMax=0.1f;
    this->minSize=10;
    this->neighborsDistance=0.4f;
    this->nbregions=0;
     this->maxSize=500;
}

RegionsManager::~RegionsManager()
{
   this->regions.clear();
}

Region RegionsManager::getRegion(int ID){
    Region r(ID,this->maxSize,this->neighborsDistance);
    //test if the region is known
    if(this->regions.contains(r))
        // return the region
        return this->regions.at(this->regions.lastIndexOf(r));
    else throw Erreur(" The region is not known");
}

void RegionsManager::remove(int ID){
    Region r(ID,this->maxSize,this->neighborsDistance);
    //test if the region is known
    if(this->regions.contains(r))
        // return the region
         this->regions.remove(this->regions.lastIndexOf(r));
    else throw Erreur(" The region is not known");
}

bool RegionsManager::removeRegions(QVector<int> RegionsID){
    bool notSmall=true;
    int tooSmall=this->getRegion(RegionsID.at(0)).size();
     //search the smalest region
    for(int j=0;j<RegionsID.size();j++)
    {

        if(this->getRegion(RegionsID.at(j)).size()<tooSmall)
            tooSmall=this->getRegion(RegionsID.at(j)).size();
        //remove region
        this->remove(RegionsID.at(j));
    }
    //if a region having merged is too small, this is not a switch
    if(tooSmall<this->minSize)
        notSmall=false;
    return notSmall;
}
QVector<int> RegionsManager::intoRegions(PointGL currentPoint){
    // ==== Count the number of regions which currentPoint is in===
    QVector<int> countRegions;
    // test if the point belongs to regions, and counts the number of regions
    for(int j=0;j<this->regions.size();j++)
        if(this->regions.at(j).isIn(currentPoint,this->distanceMax))
            countRegions.push_back(this->regions.at(j).getID());//add the identifier of the regions
    return countRegions;
}
int RegionsManager::generatingID(){
this->nbregions++;
    return nbregions;
}
void RegionsManager::split(int regionID)
{
    //split the region
    QVector<Region> newRegions;
    Region r1( this->generatingID(),this->maxSize,this->neighborsDistance);
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
                Region r2( this->generatingID(),this->maxSize,this->neighborsDistance);
                r2.growing(pt);
                newRegions.push_back(r2);
        }
    }
    // remove the regions
    this->remove(regionID);
    // add the new regions
     for(int j=0;j<newRegions.size();j++)
    this->regions.push_back(newRegions[j]);
}

