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
    this->maxSize=500;
}

ListeRail::ListeRail(int maxSize)
{
    this->maxSize=maxSize;
    this->epsilon=1000;// degres of precision
}
ListeRail::ListeRail(QVector <PointGL > cloud,int maxSize)
{
    this->maxSize=maxSize;
    this->epsilon=1000;// degres of precision
    this->initialization(cloud);
}
ListeRail::~ListeRail()
{
    this->lesRails.clear();
    this->regions.clear();
    this->switchDetected.clear();
}

void ListeRail::initialization(QVector <PointGL > cloud){
    this->lesRails.clear();
    this->regions.clear();
    this->switchDetected.clear();
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

        //---sort by footpulse
        std::sort(this->lesRails.begin(), this->lesRails.end());
        //----
        this->initRegions();
        this->run();
    }
}
void ListeRail::initRegions(){
    this->regions.clear();
    double regionMaxSize= (double)(this->maxSize)*0.05f;
    double minsize= (double)(this->maxSize)*0.05f;
    double neighborsDistance=this->lesRails.at(0).getLm()*2;
    double widthMax=this->lesRails.at(0).getEm()*0.3;
    RegionsManager rm(minsize,neighborsDistance,regionMaxSize,widthMax);
    this->regions=rm;
}

bool ListeRail::addRail(RailCluster rail)
{

    // add the rail
    this->lesRails.push_back(rail);
    //test if the size is too big
    if(this->lesRails.size()>this->maxSize)
    {
        this->lesRails.remove(0);
        return true;
    }
    else{
        //test if the track is the last
        if(this->lesRails.size()==this->maxSize){
            //performance of denoising
            this->denoising();
        }
        return false;
    }
}

void ListeRail::run()
{
    for(int i=0;i<this->lesRails.size();i++){
        // test if the rail contain a switch
        this->growingRegions(this->lesRails.at(i));
    }
    // get the merged points
    QVector<PointGL>merged=this->regions.getPointsMerged();
    // get all switch detected
    for (int i = 0; i < merged.size(); ++i) {
          this->switchDetected.push_back(merged.at(i).getZ());
    }
}

void ListeRail::growingRegions(RailCluster rail)
{    //for each point of the rail
    for(int i=0;i<rail.getPoints().size();i++){
        PointGL currentPoint=rail.getPoints().at(i);
        this->regions.growing(currentPoint);
    }
}


QVector < QVector<PointGL> > ListeRail::spitX(  QVector <PointGL>points){
    QVector < QVector<PointGL> >pointsX;
    QVector<int> xKnown;
    //for each point
    for(int i=0;i<points.size();i++){
        int x=(int)(points.at(i).getX()*epsilon);
        //test if the point have a x coordinate known
        if(!xKnown.contains(x)){
            // if it is not known
            //add the coordinate to the known coordinate
            xKnown.push_back(x);
            //create a new Vector and add the point
            QVector<PointGL>temp;
            temp.push_back(points.at(i));
            pointsX.push_back(temp);
        }else{
            //if the point is known,
            //get the index vector of point which has the same coordinate
            int index=xKnown.lastIndexOf(x);
            //add the point
            pointsX[index].push_back(points.at(i));
        }
    }
    return pointsX;
}

QHash <int,int> ListeRail::fillFrequencyHeight( QVector<PointGL> pointsX){

    // <height , frequency>
    QHash <int,int> freqs;
    for(int j=0;j<pointsX.size();j++){
        //get height of the point
        int height=pointsX.at(j).getY()*this->epsilon;
        //test if the height is know
        if(freqs.contains(height)){
            // increase the frequency
            freqs.insert(height,freqs.value(height)+1);
        }else
            // add the frequency
            freqs.insert(height,1);
    }
    return freqs;
}

int ListeRail::searchCommonHeight(QHash <int,int> freqs){
    //search the most common height
    QList<int>	keys=freqs.keys();
    int commonheight=freqs.value(keys.at(0));
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
    return commonheight;
}

QVector<PointGL> ListeRail::cleanFailPoints(QVector <QVector<PointGL> >points){

    QVector<PointGL>new_cloud;
    for(int i=0;i<points.size();i++){
        QVector<PointGL>pointsX=points.at(i);
        // <height , frequency>
        QHash <int,int> freqs=this->fillFrequencyHeight(pointsX);
        //search the most common height
        int commonheight=this->searchCommonHeight(freqs);
        //convert commonheight in double
        double heightFound=(double)commonheight/(double)epsilon;
        PointGL pheight(0,heightFound,0);
        //adds all points, without duplicates, which has the same height of the common height.
        for(int j=0;j<pointsX.size();j++){
            if(!new_cloud.contains(pointsX.at(j))&&this->lesRails.at(0).sameHeight(pointsX.at(j),pheight))
                new_cloud.push_back(pointsX.at(j));
        }
    }
    return new_cloud;
}


void ListeRail::denoising(){
    // tout les points ayant le meme x doivent avoir la meme heuteurs.
    // il faut trier les points par x
    QVector <PointGL >cloud=this->getCloud();
    std::cout<<"number of points starting: "<<cloud.size()<<std::endl;

    QVector < QVector<PointGL> >pointByX=this->spitX(cloud);
    //new cloud
    QVector <PointGL>new_cloud=this->cleanFailPoints(pointByX);
    std::cout<<"      After the denoising: "<<new_cloud.size()<<std::endl;

    this->initialization(new_cloud);
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

void ListeRail::clear()
{

    this->lesRails.clear();
    this->switchDetected.clear();
    this->regions.clear();
}
RegionsManager ListeRail::getRegions() const
{
    return regions;
}

void ListeRail::setRegions(const RegionsManager &value)
{
    regions = value;
}


