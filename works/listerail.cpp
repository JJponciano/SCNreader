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
#include <algorithm>
#include <QHash>
ListeRail::ListeRail()
{
    this->maxSize=500;
}

ListeRail::ListeRail(int maxSize)
{
    this->maxSize=maxSize;
}
ListeRail::ListeRail(QVector <PointGL > cloud,int maxSize)
{
    this->initialization(cloud,maxSize);
}
void ListeRail::initialization(QVector <PointGL > cloud,int maxSize){
    this->maxSize=maxSize;
    // if there are points in cloud
    if(cloud.size()>0)
    {
        //create a rail
        RailCluster rc;
        // add point to rail
        rc.addPoint(cloud.at(0));
        // add rail
        this->lesRails.push_back(rc);
    }

    //For each point of cloud
    for( int i=1; i<cloud.size();i++)
    {
        //we keep his footpulse
        int ftp=cloud.at(i).getZ();
        //bool to know if the point is added
        bool isAdd=false;
        //then we cover tracks which exists to know if it exists one with the same footpulse
        for( int j=0; j<this->lesRails.size();j++)
        {
            //if yes
            if(this->lesRails.at(j).getFootpulse()==ftp)
            {
                //we add the current point in the corresponding track
                this->lesRails[j].addPoint(cloud.at(i));
                isAdd=true;
            }
        }
        //if there are not a track with the same footpulse, we create a new tracks to add the point
        if(!isAdd)
        {
            //create a rail
            RailCluster rc;
            // add point to rail
            rc.addPoint(cloud.at(i));
            // add rail
            this->lesRails.push_back(rc);
        }
    }
    this->run();
}

ListeRail::ListeRail(QVector <PointGL *> cloud,int maxSize)
{
    this->maxSize=maxSize;
    // if there are points in cloud
    if(cloud.size()>0)
    {
        //create a rail
        RailCluster rc;
        // add point to rail
        rc.addPoint(*cloud.at(0));
        // add rail
        this->lesRails.push_back(rc);
    }

    //For each point of cloud
    for( int i=1; i<cloud.size();i++)
    {
        //we keep his footpulse
        int ftp=cloud.at(i)->getZ();
        //bool to know if the point is added
        bool isAdd=false;
        //then we cover tracks which exists to know if it exists one with the same footpulse
        for( int j=0; j<this->lesRails.size();j++)
        {
            //if yes
            if(this->lesRails.at(j).getFootpulse()==ftp)
            {
                //we add the current point in the corresponding track
                this->lesRails[j].addPoint(*cloud.at(i));
                isAdd=true;
            }
        }
        //if there are not a track with the same footpulse, we create a new tracks to add the point
        if(!isAdd)
        {
            //create a rail
            RailCluster rc;
            // add point to rail
            rc.addPoint(*cloud.at(i));
            // add rail
            this->lesRails.push_back(rc);
        }
    }
    this->run();
}

ListeRail::~ListeRail()
{
}

bool ListeRail::addRail(RailCluster rail)
{

    // add the rail
    this->lesRails.push_back(rail);
    //test if the size is too big
    if(this->lesRails.size()>=this->maxSize)
    {
        this->lesRails.remove(0);
        return true;
    }
    else
        return false;
}

QVector < QVector<PointGL> > ListeRail::spitX(  QVector <PointGL>points){
    QVector < QVector<PointGL> >pointsX;
    QVector<int> xKnown;
    //for each point
    for(int i=0;i<points.size();i++){
        //test if the point have a x coordinate known
        if(!xKnown.contains(points.at(i).getX())){
            // if it is not known
            //add the coordinate to the known coordinate
            xKnown.push_back(points.at(i).getX());
            //create a new Vector and add the point
            QVector<PointGL>temp;
            temp.push_back(points.at(i));
            pointsX.push_back(temp);
        }else{
            //if the point is known,
            //get the index vector of point which has the same coordinate
            int index=xKnown.lastIndexOf(points.at(i).getX());
            //add the point
            pointsX[index].push_back(points.at(i));
        }
    }
    return pointsX;
}

QVector<PointGL> ListeRail::cleanFailPoints(QVector <QVector<PointGL> >points){
    int epsilon=1000;// degres of precision
    QVector<PointGL>new_cloud;
    for(int i=0;i<points.size();i++){
        QVector<PointGL>pointsX=points.at(i);
        for(int j=0;j<pointsX.size();j++){
            // test the most common height
            QHash <int,int> freqs;
            for(int j=0;j<pointsX.size();j++){
                //get height of the point
                int height=pointsX.at(j).getY()*epsilon;
                //test if the height is know
                if(freqs.contains(height)){
                    // increase the frequency
                    freqs.insert(height,freqs.value(height)+1);
                }else
                    // add the frequency
                    freqs.insert(height,1);
            }
            //search the most common height
            QList<int>	keys=freqs.keys();
            float commonheight=freqs.value(keys.at(0));
            int freqMax=0;
            for(int j=1;j<keys.size();j++){
                //get frequency for each height
                int currentFreq=freqs.value(keys.at(j));
                //test if the frequency is greater than commonheight
                if(currentFreq>freqMax){
                    freqMax=currentFreq;
                    commonheight=freqs.value(keys.at(j));
                }
            }
            PointGL pheight(0,commonheight,0);
            // now add all point which has the same height of the common height.
            for(int j=0;j<pointsX.size();j++){
                if(this->lesRails.at(0).sameHeight(pointsX.at(j),pheight))
                    new_cloud.push_back(pointsX.at(j));
            }
        }
    }
}

void ListeRail::debuitage(){
    // tout les points ayant le meme x doivent avoir la meme heuteurs.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // il faut trier les points par x
    QVector <PointGL >cloud=this->getCloud();


    //new cloud
    QVector <PointGL>new_cloud;



    this->initialization(new_cloud,this->maxSize);
}


void ListeRail::run()
{
    for(int i=0;i<this->lesRails.size();i++)
        // test if the rail contain a switch
        if(growingRegions(this->lesRails.at(i))){
            //            // add the footpulse to the liste of the switch
            this->switchDetected.push_back(this->lesRails.at(i).getFootpulse());
        }
}

bool ListeRail::growingRegions(RailCluster rail)
{
    bool switchByMerge=false;
    bool regionOK=true;
    //for each point of the rail
    for(int i=0;i<rail.getPoints().size();i++){
        PointGL currentPoint=rail.getPoints().at(i);
        // Count the number of regions which currentPoint is in
        QVector<int> countRegions=this->getRegions(currentPoint);

        if(countRegions.size()==0){
            // if the point have not a region
            //create a regions for it and add it
            QVector <PointGL >newRegion;
            newRegion.push_back(currentPoint);
            this->regions.push_back(newRegion);
        }else// if regions do not merge
            if(countRegions.size()==1){
                //the point is added into the region
                this->regions[countRegions.at(0)].push_back(currentPoint);
                //test if the region is not too big after this adding.
                regionOK=growingOk(this->regions.at(countRegions.at(0)));
                //if the regions is not ok, you have a switch
                if(!regionOK){
                    //split the region
                    this->split(countRegions.at(0));
                }
            }else//if a region have merged, the current point is added in a new region.
                if(countRegions.size()>1){
                    // remove region having merged and test if the merge is a switch
                    switchByMerge=this->emptyRegion(countRegions);
                    //add a new region
                    QVector <PointGL >newRegion;
                    newRegion.push_back(currentPoint);
                    this->regions.push_back(newRegion);
                }
    }
    return (regionOK==false)||switchByMerge;
}
QVector<int> ListeRail::getRegions(PointGL currentPoint){
    // ==== Count the number of regions which currentPoint is in===
    QVector<int> countRegions;
    // test if the point belongs to regions, and counts the number of regions
    for(int j=0;j<this->regions.size();j++)
        if(isInRegion(this->regions.at(j),currentPoint))
            countRegions.push_back(j);//add the index of the regions
    return countRegions;
}

bool ListeRail::emptyRegion(QVector<int> countRegions){
    bool mergeRegions=true;
    //you remove all regions which the point could be added, and create a new region.
    int tooSmall=this->regions.at(countRegions.at(0)).size();
    for(int j=0;j<countRegions.size();j++)
    {
        //search the smalest region
        if(this->regions.at(countRegions.at(j)).size()<tooSmall)
            tooSmall=this->regions.at(countRegions.at(j)).size();
        //empty region
        this->regions[countRegions.at(j)].clear();
    }

    //remove all empty regions
    for(int j=0;j<this->regions.size();j++)
    {
        if(this->regions.at(j).isEmpty())
        {
            this->regions.remove(j);
            j--;
        }
    }
    //if a region having merged is too small, this is not a switch
    if(tooSmall<10)
        mergeRegions=false;
    return mergeRegions;
}

void ListeRail::split(int regindex)
{ //split the region
    QVector <PointGL>newRegion1;
    QVector <PointGL>newRegion2;

    //for all point of the region to be splited
    for(int j=1;j<this->regions.at(regindex).size();j++){
        PointGL pt=this->regions.at(regindex).at(j);

        //Conversely growing
        //add the firs points
        if(j==0)
            newRegion1.push_back(pt);
        else{

            //for each point of the first region
            bool added=false;
            int i=0;
            // test is the point could be added to the first region and add it if is possible
            while(i<newRegion1.size()&&!added){
                PointGL currentPoint=newRegion1.at(i);
                //test if the points avec the same width with and height the point to be tested
                if(this->lesRails.at(0).sameWidth(currentPoint,pt)&&this->lesRails.at(0).sameHeight(currentPoint,pt)){
                    // add the point and stop the loop
                    newRegion1.push_back(pt);
                    added=true;
                }else
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
int ListeRail::getMaxSize() const
{
    return maxSize;
}

void ListeRail::setMaxSize(int value)
{
    maxSize = value;
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
QVector <PointGL > ListeRail::getCloud()const{
    QVector <PointGL >cloud;
    //for each rail
    for(int i=0;i<this->lesRails.size();i++){
        //add the list of the points
        for(int j=0;j<this->lesRails.at(i).getPoints().size();j++){
            cloud.push_back(this->lesRails.at(i).getPoints().at(j));
        }
    }
    return cloud;
}

bool ListeRail::growingOk(const QVector <PointGL> reg)const
{
    float widthMax=this->lesRails.at(0).getEm();
    //search the extremum of the x coordinates in the region
    //search the extremum of the x coordinates in the region
    float xmin=reg.at(0).getX();
    float xmax=reg.at(0).getX();
    for(int i=1;i<reg.size();i++){
        if(reg.at(i).getX()<xmin)xmin=reg.at(i).getX();
        else    if(reg.at(i).getX()>xmax)xmax=reg.at(i).getX();
    }
    //compare the distance between the extremums with the maximum authorized.
    return (xmax-xmin)<widthMax;
}

bool ListeRail::isInRegion(const QVector<PointGL> reg, PointGL pt)const
{
    //for each point of the region
    if(this->lesRails.size()!=0)
        for(int i=reg.size()-50;i<reg.size();i++){
            if(i<0)i=0;
            //test if the points avec the same width with and height the point to be tested
            if(this->lesRails.at(0).widthDistance(reg.at(i),pt))
                return true;
        }
    return false;

}
void ListeRail::clear()
{

    this->lesRails.clear();
    this->switchDetected.clear();
    this->regions.clear();
}

