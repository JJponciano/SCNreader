/**
 * @file listerail.cpp
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
#include "listerail.h"

ListeRail::ListeRail()
{

}

ListeRail::~ListeRail()
{
}

void ListeRail::addRail(RailCluster rail)
{
    // add the rail
    this->lesRails.push_back(rail);
    // test if the rail contain a switch
    //        if(growingRegions(rail)){
    //            // add the footpulse to the liste of the switch
    this->switchDetected.push_back(rail.getFootpulse());
    //        }
}

bool ListeRail::growingRegions(RailCluster rail)
{
    bool mergeRegions=false;
    bool regionOK=true;
    //for each point of the rail
    for(int i=0;i<rail.getPoints().size();i++){
        QVector<int> countRegions;
        pcl::PointXYZ *currentPoint=rail.getPoints().at(i);
        // test if the point belongs to regions, and counts the number of regions
        for(int j=0;j<this->regions.size();j++)
            if(isInRegion(this->regions.at(j),currentPoint))
                countRegions.push_back(j);//add the index of the regions

        //if you have a merge of regions, you can not add the point to the region.
        if(countRegions.size()>1){
            mergeRegions=true;
            //you remove all regions which the point could be added, and create a new region.
            for(int j=0;j<countRegions.size();j++)
                this->regions.remove(countRegions.at(j));
            QVector <pcl::PointXYZ *>newRegion;
            newRegion.push_back(currentPoint);
            this->regions.push_back(newRegion);
        }else if(countRegions.size()==1){
            //the point is added into the region
            this->regions[countRegions.at(0)].push_back(currentPoint);
            //test if the region is not too big after this adding.
            regionOK=growingOk(this->regions.at(countRegions.at(0)));
            //if the regions is not ok, you have a switch
            if(!regionOK){
                //split the region
                this->split(countRegions.at(0));
            }
        }else{
            // if the point have not a region
            //create a regions for it and add it
            QVector <pcl::PointXYZ *>newRegion;
            newRegion.push_back(currentPoint);
            this->regions.push_back(newRegion);
        }
    }
    return (regionOK==false)||mergeRegions;
}
void ListeRail::split(int regindex)
{ //split the region
    QVector <pcl::PointXYZ *>newRegion1;
    QVector <pcl::PointXYZ *>newRegion2;

    //for all point of the region to be splited
    for(int j=1;j<this->regions.at(regindex).size();j++){
        pcl::PointXYZ *pt=this->regions.at(regindex).at(j);

        //Conversely growing
        //add the firs points
        if(j==0)
            newRegion1.push_back(pt);
        else{

            //for each point of the first region
            bool added=false;
            int i=0;
            // test is the point could be added to the first region
            while(i<newRegion1.size()&&!added){
                pcl::PointXYZ *currentPoint=newRegion1.at(i);
                //test if the points avec the same width with and height the point to be tested
                if(this->lesRails.at(0).sameWidth(currentPoint,pt)&&this->lesRails.at(0).sameHeight(currentPoint,pt)){
                    // add the point and stop the loop
                    newRegion1.push_back(pt);
                    added=true;
                }
                i++;
            }
            //if the point does not belong to the first region
            if(!added)
                //it is added to the other region
                newRegion2.push_back(pt);
        }
    }
    // remove the regions
    this->regions.remove(regindex);
    // add two new regions
    this->regions.push_back(newRegion1);
    this->regions.push_back(newRegion2);

}
QVector<RailCluster> ListeRail::getLesRails() const
{
    return lesRails;
}

void ListeRail::setLesRails(const QVector<RailCluster> &value)
{
    lesRails = value;
}

QVector<int> ListeRail::getSwitchDetected() const
{
    return switchDetected;
}
int ListeRail::getNumberSwitchDetected() const
{
    return switchDetected.size();
}
void ListeRail::setSwitchDetected(const QVector<int> &value)
{
    switchDetected = value;
}
QVector <pcl::PointXYZ *> ListeRail::getCloud()const{
    QVector <pcl::PointXYZ *>cloud;
    //for each rail
    for(int i=0;i<this->lesRails.size();i++){
        //add the list of the points
        for(int j=0;j<this->lesRails.at(i).getPoints().size();j++){
            cloud.push_back(this->lesRails.at(i).getPoints().at(j));
        }
    }
    return cloud;
}

bool ListeRail::growingOk(QVector <pcl::PointXYZ *> reg)
{
    float widthMax=this->lesRails.at(0).getEm();
    //search the extremum of the x coordinates in the region
    //search the extremum of the x coordinates in the region
    float xmin=reg.at(0)->x;
    float xmax=reg.at(0)->x;
    for(int i=1;i<reg.size();i++){
        if(reg.at(i)->x<xmin)xmin=reg.at(i)->x;
        else    if(reg.at(i)->x>xmax)xmax=reg.at(i)->x;
    }
    //compare the distance between the extremums with the maximum authorized.
    return (xmax-xmin)<widthMax;
}

bool ListeRail::isInRegion(QVector <pcl::PointXYZ *> reg, pcl::PointXYZ * pt)
{
    //for each point of the region
    if(this->lesRails.size()!=0)
        for(int i=0;i<reg.size();i++){
            pcl::PointXYZ *currentPoint=reg.at(i);
            //test if the points avec the same width with and height the point to be tested
            if(this->lesRails.at(0).sameWidth(currentPoint,pt)&&this->lesRails.at(0).sameHeight(currentPoint,pt))
                return true;
        }
    return false;

}
void ListeRail::clear()
{
    QVector <int>s;///< list of the footpulste with switch
    QVector <RailCluster> ls;///< all rails
    QVector<QVector <pcl::PointXYZ *>>r;///<regions detected

    this->lesRails=ls;
    this->switchDetected=s;
    this->regions=r;
}

