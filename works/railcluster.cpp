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
#include "../modules/exceptions/erreur.h"
RailCluster::RailCluster()
{
    this->delta=0.05;
    this->hm = 0.18;
    this->lm = 0.08;
    this->em = 1.5;
}

RailCluster::RailCluster(float height, float width, float spacing, const QVector<pcl::PointXYZ *> footpulse)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=0.03;
    this->footpulse=footpulse.at(0)->z;

    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(footpulse);
/*
    //refinement
    bool add1=true;
    bool add2=true;

    while(add1||add2){
        // 2) search a corresponding track
        add1= this->match(footpulse);
        // 3) search neighbour points
        add2=this->growing(*this,footpulse);
    }*/
}

RailCluster::RailCluster(float height, float width, float spacing,  const QVector<pcl::PointXYZ *> footpulse,  const RailCluster rail)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=0.04;
    this->footpulse=footpulse.at(0)->z;
    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(footpulse);
    //4) growing
    // this->growing(rail,footpulse);

    this->match(footpulse);

    //segmentation again
    QVector <pcl::PointXYZ *> ptemp=this->points;
    for(int i=0;i<this->points.size();i++){
                pcl::PointXYZ *p=new pcl::PointXYZ(0.0,0.0,0.0);
                this->points[i]=p;

    }  this->points.clear();
     this->add(ptemp);

    //this->growing(*this,footpulse);
 /*
*/
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
        if(seq.size()==0){
            seq.push_back(pts.at(i));
        }else{
            pcl::PointXYZ *currentPoint=pts.at(i);
            pcl::PointXYZ averageP=this->averagePoint(seq);
            //if the point is at a height below hm/2 from the previous point, it is adding
            if(this->sameHeight(currentPoint,&averageP))
                seq.push_back(pts.at(i));
            //else, the sequence is finiched
            else {
                again=false;
                i--;
            }
        }
        //if the sequence is finiched
        if(!again||(i+1)==pts.size()){
            // start a new sequence
            again=true;
            // test if the size of the sequence is below to lm+lm/3
            float distance=this->lm;
            //float dinf=this->lm-(this->delta);
            float legap=gap(seq);
            if(legap<=distance){
                //points of the sequence are a track and are added
                for(int j=0;j<seq.size();j++){
                    if(!this->points.contains(seq.at(j)))
                        this->points.push_back(seq.at(j));
                }
            }
            /*********************************************************************************/
            /******************ACHTUNG: FUITE MEMOIRE POSSIBLE*******************************/
            //this sequence is may be not a track
            for(int j=0;j<seq.size();j++){
                pcl::PointXYZ *p=new pcl::PointXYZ(0.0,0.0,0.0);
                seq[j]=p;
            }
            seq.clear();
            /*
            QVector <pcl::PointXYZ *> seq2;
            seq=seq2;*/
            /*********************************************************************************/
            /*********************************************************************************/

        }
    }

}
void RailCluster::remove(pcl::PointXYZ * pt){
    int i= this->points.lastIndexOf(pt);
    if(i>=0)
        this->points.remove(i);
}
float RailCluster::gap(QVector <pcl::PointXYZ *> reg)const
{
    //search the extremum of the x coordinates in the region
    float xmin=reg.at(0)->x;
    float xmax=reg.at(0)->x;
    for(int i=1;i<reg.size();i++){
        if(reg.at(i)->x<xmin)
            xmin=reg.at(i)->x;
        else
            if(reg.at(i)->x>xmax)
                xmax=reg.at(i)->x;
    }
    //compare the distance between the extremums with the maximum authorized.
    return (xmax-xmin);
}
pcl::PointXYZ RailCluster::averagePoint(QVector <pcl::PointXYZ *> reg)
{
    pcl::PointXYZ p;
    float xm=0;
    float ym=0;
    for(int i=0;i<reg.size();i++){
        ym+=reg.at(i)->y;
        xm+=reg.at(i)->x;
    }
    xm/=reg.size();
    ym/=reg.size();
    p.x=xm;
    p.y=ym;
    p.z=this->footpulse;
    return p;
}
bool RailCluster::match(QVector <pcl::PointXYZ *> pts)
{
    bool pointadded=false;
    /*  This function matches the points in pairs and if is necessary, it adds a point of the pair.
        * Then, it removes and adds of the blacklist all single point. For to match points two by two,
        * it tests for each point of this class if the point has a distance close to the average
        * spacing of railway railand with another point into the list given close to and test if it has a similar height.*/
    //for each point keeped
    int nbpoint=this->points.size();
    for(int i=0;i<nbpoint;i++){
        //test if it exists points which have the same height and which is at a distance of as em
        pcl::PointXYZ *currentPoint=this->points.at(i);
        //bool for see if the point has a corresponding
        bool corresp=false;
        // for each point not yet keeped search a corresponding
        for(int j=0;j<pts.size();j++){
            pcl::PointXYZ *testPoint=pts.at(j);
            if(!this->blacklist.contains(testPoint)){
                // if the point have a corresponding
                if(this->spacingDistance(currentPoint,testPoint)&&
                        this->sameHeight(currentPoint,testPoint)){
                    corresp=true;
                    //test if it is already added
                    if(!this->points.contains(testPoint)){
                        // if is not already added, add it
                        this->points.push_back(testPoint);
                        // flag: it have added a point
                        pointadded=true;
                    }
                }
            }
        }
        // if the point has not a corresponding, it is removed
        if(!corresp){
            // it is black listed
            this->blacklist.push_back(currentPoint);
            //remove fail point
            pcl::PointXYZ *p=new pcl::PointXYZ(0.0,0.0,0.0);
            this->points[i]=p;
            this->points.remove(i);
            //since a point is removed, the size of the vector decrease of 1
            i--;
            nbpoint--;

        }
    }
    return pointadded;
}

bool RailCluster::addSimilarePoint(QVector <pcl::PointXYZ *> pts)
{
    throw Erreur("Unused");
}

bool RailCluster::growing(RailCluster rail, QVector <pcl::PointXYZ *> pts)
{    bool pointadded=false;
     //for each point of the rail
     for(int i=0;i<rail.getPoints().size();i++){
         //test if it exists points which have the same height and which is at a distance of as em
         pcl::PointXYZ *currentPoint=rail.getPoints().at(i);
         //test if it is not black listed
         if(!this->blacklist.contains(currentPoint)){
             // for each point in the footpulse
             for(int j=0;j<pts.size();j++){
                 pcl::PointXYZ *testPoint=pts.at(j);
                 //test if it is not black listed and if it is not already contained.
                 if(!this->blacklist.contains(testPoint)  && !this->points.contains(testPoint)){
                     //test if there have the same width and same height
                     if(this->sameWidth(currentPoint,testPoint)&&this->sameHeight(currentPoint,testPoint)){
                         //add it
                         this->points.push_back(testPoint);
                         pointadded=true;
                     }
                 }
             }
         }
     }
      return pointadded;
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
    if(p1==nullptr||p2==nullptr) throw Erreur("one of points, which are given in sameHeight, is a null pointer");
    else{

        //get distance between p1 and p2
        float y1=p1->y;
        float y2=p2->y;
        float pv=y1-y2;
        if(pv<0.0)
            pv=-1.0*pv;
        //approximation
        bool lesbool=(pv<this->delta);

        return lesbool;
    }
}
bool RailCluster::sameWidth(pcl::PointXYZ *p1, pcl::PointXYZ *p2)const
{
    if(p1==nullptr||p2==nullptr) throw Erreur("one of points, which are given in sameWidth, is a null pointer");
    else{
        //get distance between p1 and p2
        float x1=p1->x;
        float x2=p2->x;
        float pv=x1-x2;
        if(pv<0.0)
            pv=-1.0*pv;
        //approximation
        bool lesbool=(pv<this->delta);
        return lesbool;
    }

}
bool RailCluster::widthDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2)const
{
    if(p1==nullptr||p2==nullptr) throw Erreur("one of points, which are given in spacing Distance, is a null pointer");
    else{
        //get distance between p1 and p2 - average spacing between 2 tracks
        float x1=p1->x;
        float x2=p2->x;
        float pv=x1-x2;
        if(pv<0.0)
            pv=-1.0*pv;
        //approximation

        float dsup= this->lm+this->delta;
        float dinf=this->lm-this->delta;
        bool lesbool=(pv<=dsup&&pv>=dinf);
        return lesbool;
    }
}

bool RailCluster::spacingDistance(pcl::PointXYZ *p1, pcl::PointXYZ *p2)const
{

    if(p1==nullptr||p2==nullptr) throw Erreur("one of points, which are given in spacing Distance, is a null pointer");
    else{
        //get distance between p1 and p2 - average spacing between 2 tracks
        float x1=p1->x;
        float x2=p2->x;
        float pv=x1-x2;
        if(pv<0.0)
            pv=-1.0*pv;
        //approximation

        float dsup= this->em+this->delta;
        float dinf=this->em-this->delta;
        bool lesbool=(pv<=dsup&&pv>=dinf);
        return lesbool;
    }
}


