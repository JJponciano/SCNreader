/**
 * @file railcluster.h
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
#include "railcluster.h"

RailCluster::RailCluster()
{

}

RailCluster::RailCluster(float height, float width, float spacing)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
}

RailCluster::RailCluster(float height, float width, float spacing, QVector <pcl::PointXYZ *>footpulse)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->points = footpulse;
}

RailCluster::~RailCluster()
{
    this->points.clear();
}

void RailCluster::add(QVector <pcl::PointXYZ *> pts)
{
    //Selecting points following with a similar height to create a sequence
    QVector <pcl::PointXYZ *> seq;
    bool again=true;

    for( int i=0;i<pts.size();i++){
        //if is a first points
        if(i==0){
            seq.push_back(pts.at(i));
        }else{
            pcl::PointXYZ *currentPoint=pts.at(i);
            pcl::PointXYZ *previous=seq.at(i-1);
            //if the point is at a height below hm/2 from the previous point, it is adding
            if((currentPoint->y-previous->y)<this->hm/2.0) seq.push_back(pts.at(i));
            // else, the sequence is finiched
            else again=false;
        }
        //if the sequence is finiched
        if(!again||(i+1)==pts.size()){
            // start a new sequence
            again=true;
            // test if the size of the sequence is below to lm+lm/3
            if(seq.size()<(this->lm+this->lm/3.0)){
                //points of the sequence are a track and are added
                for(int j=0;j<seq.size();j++){
                    this->points.push_back(seq.at(j));
                }
            }
            //this sequence is may be not a track
            for(int j=0;j<seq.size();j++){
                seq[i]=0;
            }seq.clear();

        }
    }
    //  This function tests if the point is into a bounding box
    //  defined by the height and width of a railway rail. So, if it is inside, it is added .
}

void RailCluster::add(pcl::PointXYZ * pt)
{

}

void RailCluster::remove(pcl::PointXYZ * pt){

}

void RailCluster::match(QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::addSimilarePoint(QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::growing(RailCluster rail, QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::setEm(float e){
    this->em=e;
}

void RailCluster::setHm(float h){
    this->hm=h;
}

void RailCluster::setLm(float l){
    this->lm=l;
}

float RailCluster::getEm(){
    return this->em;
}

float RailCluster::getHm(){
    return this->hm;
}

float RailCluster::getLm(){
    return this->lm;
}
