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
    this->delta=3.0;
}

RailCluster::RailCluster(float height, float width, float spacing, QVector <pcl::PointXYZ *>footpulse)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=3.0;
    this->footpulse=footpulse.at(0)->z;

    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(footpulse);

    //refinement
    bool add1=true;
    bool add2=true;
    while(add1||add2){
        // 2) search a corresponding track
        add1= this->match(footpulse);
        // 3) search neighbour points
        add2=this->addSimilarePoint(footpulse);
    }
}

RailCluster::RailCluster(float height, float width, float spacing, QVector <pcl::PointXYZ *>footpulse,RailCluster rail)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=3.0;
    this->footpulse=footpulse.at(0)->z;
    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(footpulse);

    //4) growing
    this->growing(rail,footpulse);

    //refinement
    bool add1=true;
    bool add2=true;
    while(add1||add2){
        // 2) search a corresponding track
        add1= this->match(footpulse);
        // 3) search neighbour points
        add2=this->addSimilarePoint(footpulse);
    }

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
            if(this->sameHeight(currentPoint,previous))
                seq.push_back(pts.at(i));
            // else, the sequence is finiched
            else again=false;
        }
        //if the sequence is finiched
        if(!again||(i+1)==pts.size()){
            // start a new sequence
            again=true;
            // test if the size of the sequence is below to lm+lm/3
            if(seq.size()<(this->lm+this->lm/this->delta)){
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
}
void RailCluster::remove(pcl::PointXYZ * pt){
   int i= this->points.lastIndexOf(pt);
   if(i>=0)
    this->points.remove(i);
}

bool RailCluster::match(QVector <pcl::PointXYZ *> pts)
{
    bool pointadded=false;
    /*  This function matches the points in pairs and if is necessary, it adds a point of the pair.
        * Then, it removes and adds of the blacklist all single point. For to match points two by two,
        * it tests for each point of this class if the point has a distance close to the average
        * spacing of railway railand with another point into the list given close to and test if it has a similar height.*/
    //for each point keeped
    for(int i=0;i<this->points.size();i++){
        //test if it exists points which have the same height and which is at a distance of as em
        pcl::PointXYZ *currentPoint=this->points.at(i);
        //bool for see if the point has a corresponding
        bool corresp=false;
        //test if is not black listed
        if(!this->blacklist.contains(currentPoint)){
            for(int j=0;j<pts.size();j++){
                pcl::PointXYZ *testPoint=pts.at(i);
                if(this->spacingDistance(currentPoint,testPoint)&&this->sameHeight(currentPoint,testPoint)){
                    this->points.push_back(testPoint);
                    pointadded=true;
                    corresp=true;
                }
            }
            // if the point has not a corresponding, it is removed
            if(!corresp){
                this->points.remove(i);
                // and it is black listed
                this->blacklist.push_back(currentPoint);
            }
        }
    }
    return pointadded;
}

bool RailCluster::addSimilarePoint(QVector <pcl::PointXYZ *> pts)
{
    bool pointadded=false;
    for(int i=0;i<this->points.size();i++){
        //test if it exists points which have the same height and which is at a distance of as lm
        pcl::PointXYZ *currentPoint=this->points.at(i);
        //test if is not black listed
        if(!this->blacklist.contains(currentPoint)){
            for(int j=0;j<pts.size();j++){
                pcl::PointXYZ *testPoint=pts.at(i);
                if(this->widthDistance(currentPoint,testPoint)&&this->sameHeight(currentPoint,testPoint)){
                    this->points.push_back(testPoint);
                    pointadded=true;
                }
            }
        }
    }
    return pointadded;
}

void RailCluster::growing(RailCluster rail, QVector <pcl::PointXYZ *> pts)
{
    for(int i=0;i<rail.getPoints().size();i++){
        //test if it exists points which have the same height and which is at a distance of as em
        pcl::PointXYZ *currentPoint=rail.getPoints().at(i);
        //test if is not black listed
        for(int j=0;j<pts.size();j++){
            pcl::PointXYZ *testPoint=pts.at(i);
            if(this->widthDistance(currentPoint,testPoint)&&this->sameHeight(currentPoint,testPoint)){
                this->points.push_back(testPoint);
            }
        }
    }
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

float RailCluster::getEm()const{
    return this->em;
}

float RailCluster::getHm()const{
    return this->hm;
}

float RailCluster::getLm()const{
    return this->lm;
}
QVector<pcl::PointXYZ *> RailCluster::getPoints() const
{
    return points;
}

void RailCluster::setPoints(const QVector<pcl::PointXYZ *> &value)
{
    points = value;
}
QVector<pcl::PointXYZ *> RailCluster::getBlacklist() const
{
    return blacklist;
}

void RailCluster::setBlacklist(const QVector<pcl::PointXYZ *> &value)
{
    blacklist = value;
}
int RailCluster::getFootpulse() const
{
    return footpulse;
}

void RailCluster::setFootpulse(int value)
{
    footpulse = value;
}


bool RailCluster::sameHeight(pcl::PointXYZ *p1, pcl::PointXYZ *p2) const
{
    return (std::abs(p1->y-p2->y)<this->hm/this->delta);
}
bool RailCluster::sameWidth(pcl::PointXYZ *p1, pcl::PointXYZ *p2)const
{
    return (std::abs(p1->y-p2->y)<this->lm/this->delta);
}
bool RailCluster::widthDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2)const
{
    return( std::abs(p1->x-p2->x)<(this->lm+this->lm/this->delta)&& (std::abs(p1->x-p2->x)>(this->lm-this->lm/this->delta)));
}

bool RailCluster::spacingDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2)const
{
    return( std::abs(p1->x-p2->x)<(this->em+this->em/this->delta)&& (std::abs(p1->x-p2->x)>(this->em-this->em/this->delta)));

}


