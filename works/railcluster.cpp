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
    this->delta=0.04;
    this->hm = 0.18;
    this->lm = 0.08;
    this->em = 1.5;
}
RailCluster::RailCluster(double height, double width, double spacing, const QVector<pcl::PointXYZ *> footpulse)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=0.04;

    //convert QVector<pcl::PointXYZ *> to QVector<PointGL *>
    QVector<PointGL > okfoot;
    for(int i=0;i<footpulse.size();i++){
        PointGL point(footpulse.at(i)->x,footpulse.at(i)->y,footpulse.at(i)->z);
        okfoot.push_back(point);
    }
    this->footpulse=okfoot.at(0).getZ();
    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(okfoot);
     this->match(okfoot);
    //remove temporary vector
    okfoot.clear();
}

RailCluster::RailCluster(double height, double width, double spacing,  const QVector<pcl::PointXYZ *> footpulse,  const RailCluster rail)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=0.04;
    //convert QVector<pcl::PointXYZ *> to QVector<PointGL *>
    QVector<PointGL > okfoot;
    for(int i=0;i<footpulse.size();i++){
        PointGL point(footpulse.at(i)->x,footpulse.at(i)->y,footpulse.at(i)->z);
        okfoot.push_back(point);
    }
    this->footpulse=okfoot.at(0).getZ();

    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    QVector<PointGL >p1=this->addByWindows(okfoot,1);
    QVector<PointGL >p2=this->addByWindows(okfoot,-1);
    for(int i=0;i<p1.size();i++){
        if(p2.contains(p1.at(i)))this->addPoint(p1.at(i));
    }


    this->match(okfoot);
    //segmentation again
   /* QVector <PointGL > ptemp=this->points;
    this->points.clear();
    this->add(ptemp);*/
    //remove temporary vector
    okfoot.clear();
}
RailCluster::RailCluster(double height, double width, double spacing, const QVector<PointGL *> footpulse)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=0.04;
    this->footpulse=footpulse.at(0)->getZ();
    QVector<PointGL > okfoot;
    for(int i=0;i<footpulse.size();i++){
        PointGL point(*footpulse.at(i));
        okfoot.push_back(point);
    }
    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(okfoot);
    this->match(okfoot);
}

RailCluster::RailCluster(double height, double width, double spacing,  const QVector<PointGL *> footpulse,  const RailCluster rail)
{
    this->hm = height;
    this->lm = width;
    this->em = spacing;
    this->delta=0.04;
    this->footpulse=footpulse.at(0)->getZ();

    QVector<PointGL > okfoot;
    for(int i=0;i<footpulse.size();i++){
        PointGL point(*footpulse.at(i));
        okfoot.push_back(point);
    }
    //1) For each footpulse we search a sequence of points which have the same heught and the width of this sequence
    this->add(okfoot);
    //4) growing
    // this->growing(rail,footpulse);

    this->match(okfoot);

    //segmentation again
  /*  QVector <PointGL> ptemp=this->points;
    this->points.clear();
    this->add(ptemp);*/

}

RailCluster::~RailCluster()
{
    this->points.clear();
    this->blacklist.clear();
}

bool RailCluster::addPoint(PointGL p)
{
    //test if is a first point added
    if(this->points.empty()){
        //create a point
        PointGL newpoint(p);
        //add the point
        this->points.push_back(newpoint);
        //set the footpulse
        this->footpulse=p.getZ();
        return true;
    }else // if the point have the same footpulse than other points already added and is not already added
        if(p.getZ()==this->footpulse&&!this->isContains(p)){
            //create a point
            PointGL newpoint(p);
            //add the point
            this->points.push_back(newpoint);
            return true;
        }
    return false;
}

bool RailCluster::isContains( PointGL p)const {
    // test if points containe p
    for(int i=0;i<this->points.size();i++){
        if(p.equals2D(this->points.at(i)))
            return true;
    }
    return false;
}
bool RailCluster::isBlackListed( PointGL p)const{
    // test if points containe p
    for(int i=0;i<this->blacklist.size();i++){
        if(p.equals2D(this->blacklist.at(i)))
            return true;
    }
    return false;
}

QVector <PointGL> RailCluster::addByWindows(const QVector<PointGL> pts,int sens)
{
    QVector <PointGL> pointsFinded;
    //Selecting points following with a similar height to create a sequence
    QVector <PointGL> seq;
    bool again=true;
    //it is a first point of the sequence
    seq.push_back(pts.at(0));
    if(sens>0)
    for( int i=1;i<pts.size();i++) {

        PointGL averageP=this->averagePoint(seq);
        //if the point is at a height below hm/2 from the previous point and
        //if the distance between two points is less than width of the track, it is adding
        if(this->sameHeight(pts.at(i),averageP))
            seq.push_back(pts.at(i));
        //else, the sequence is finiched
        else {
            again=false;
        }
        //if the sequence is finiched
        if(!again||(i+1)==pts.size()){
            // start a new sequence
            again=true;
            // test if the size of the sequence is below to lm
            // if the last point of a track is below another point, if not a tack
            if((gap(seq)<=this->lm*3)&&(seq.last().getY()>pts.at(i).getY())){

                //points of the sequence are a track and are added
                for(int j=0;j<seq.size();j++){
                   pointsFinded.push_back(seq.at(j));
                }
            }
            //this sequence is may be not a track
            seq.clear();
            seq.push_back(pts.at(i));
        }
    }
    else{     seq.push_back(pts.last());
            for( int i=pts.size()-2;i>=0;i--) {

            PointGL averageP=this->averagePoint(seq);
            //if the point is at a height below hm/2 from the previous point and
            //if the distance between two points is less than width of the track, it is adding
            if(this->sameHeight(pts.at(i),averageP))
                seq.push_back(pts.at(i));
            //else, the sequence is finiched
            else {
                again=false;
            }
            //if the sequence is finiched
            if(!again||(i+1)==pts.size()){
                // start a new sequence
                again=true;
                // test if the size of the sequence is below to lm
                // if the last point of a track is below another point, if not a tack
                if((gap(seq)<=this->lm*3)&&(seq.last().getY()>pts.at(i).getY())){

                    //points of the sequence are a track and are added
                    for(int j=0;j<seq.size();j++){
                       pointsFinded.push_back(seq.at(j));
                    }
                }
                //this sequence is may be not a track
                seq.clear();
                seq.push_back(pts.at(i));
            }
        }


    }return pointsFinded;
}
void RailCluster::addByMountain(const QVector<PointGL> pts)
{
    //Selecting points following with a similar height to create a sequence
    QVector <PointGL> seq;
    bool again=true;
    //it is a first point of the sequence
    seq.push_back(pts.at(0));
    for( int i=1;i<pts.size();i++) {

        PointGL averageP=this->averagePoint(seq);
        //if the point is at a height below hm/2 from the previous point, it is adding
        //if the distance between two points  is greater than
        if(this->sameHeight(pts.at(i),averageP))
            seq.push_back(pts.at(i));
        //else, the sequence is finiched
        else {
            again=false;
        }
        //if the sequence is finiched
        if(!again||(i+1)==pts.size()){
            // start a new sequence
            again=true;
            // test if the size of the sequence is below to lm
            // if the last point of a track is below another point, if not a tack
            if((gap(seq)<=this->lm*3)&&(seq.last().getY()>pts.at(i).getY())){

                //points of the sequence are a track and are added
                for(int j=0;j<seq.size();j++){
                    this->addPoint(seq.at(j));
                }
            }
            //this sequence is may be not a track
            seq.clear();
            seq.push_back(pts.at(i));
        }
    }
}

void RailCluster::add(const QVector<PointGL> pts)
{
    //Selecting points following with a similar height to create a sequence
    QVector <PointGL> seq;
    bool again=true;
    //it is a first point of the sequence
    seq.push_back(pts.at(0));
    for( int i=1;i<pts.size();i++) {

        PointGL averageP=this->averagePoint(seq);
        //if the point is at a height below hm/2 from the previous point, it is adding
        if(this->sameHeight(pts.at(i),averageP))
            seq.push_back(pts.at(i));
        //else, the sequence is finiched
        else {
            again=false;
        }
        //if the sequence is finiched
        if(!again||(i+1)==pts.size()){
            // start a new sequence
            again=true;
            // test if the size of the sequence is below to lm
            // if the last point of a track is below another point, if not a tack
            if((gap(seq)<=this->lm)&&(seq.last().getY()>pts.at(i).getY())){

                //points of the sequence are a track and are added
                for(int j=0;j<seq.size();j++){
                    this->addPoint(seq.at(j));
                }
            }
            //this sequence is may be not a track
            seq.clear();
            seq.push_back(pts.at(i));
        }
    }
}

void RailCluster::remove(PointGL  pt){
    // search the lasr index of the point
    int i= this->points.lastIndexOf(pt);
    if(i>=0){
        //test if the point is not blacklisted
        if(!this->isBlackListed(this->points.at(i)))
            //blackliste the point
            this->blacklist.push_back(this->points.at(i));
        // remove point
        this->points.remove(i);
    }
}
double RailCluster::gap(QVector <PointGL> reg)const
{
    //search the extremum of the x coordinates in the region
    double xmin=reg.at(0).getX();
    double xmax=reg.at(0).getX();
    for(int i=1;i<reg.size();i++){
        if(reg.at(i).getX()<xmin)
            xmin=reg.at(i).getX();
        else
            if(reg.at(i).getX()>xmax)
                xmax=reg.at(i).getX();
    }
    //compare the distance between the extremums with the maximum authorized.
    return (xmax-xmin);
}
PointGL RailCluster::averagePoint(QVector <PointGL> reg)const
{
    PointGL p;
    double xm=0;
    double ym=0;
    //calcule the average point
    for(int i=0;i<reg.size();i++){
        ym+=reg.at(i).getY();
        xm+=reg.at(i).getX();
    }
    xm/=reg.size();
    ym/=reg.size();
    p.setX(xm);
    p.setY(ym);
    p.setZ(this->footpulse);
    return p;
}
bool RailCluster::match(QVector<PointGL> pts)
{
    bool pointadded=false;
    //for each point keeped
    QVector <PointGL > ptemp=this->points;
    for(int i=0;i<ptemp.size();i++){
        //test if it exists points which have the same height and which is at a distance of as em
        PointGL currentPoint=ptemp.at(i);
        //bool for see if the point has a corresponding
        int corresp=this->searchCorresponding(currentPoint,&pts);
        // if the point has not a corresponding, it is removed
        if(corresp<=0){
            //remove fail point
            this->remove(currentPoint);
        }else
            // if a point is added
            if(corresp==2){
                // flag: it have added a point
                pointadded=true;
            }
    }
    return pointadded;
}
int RailCluster::searchCorresponding(PointGL currentPoint,QVector<PointGL> *pts){
    bool corresp=false;
    bool pointadded=false;
    // for each point not yet keeped search a corresponding
    for(int j=0;j<pts->size();j++){
        PointGL testPoint=pts->at(j);
        if(!this->isBlackListed(testPoint)){
            // if the point have a corresponding
            if(this->spacingDistance(currentPoint,testPoint)&& this->sameHeight(currentPoint,testPoint)){
                corresp=true;
                //test if it is already added and if is not already added, add i
                 if(this->addPoint(testPoint)){
                // flag: it have added a point
                pointadded=true;
                }

            }
        }
    }
    int val=0;
    if(corresp)val++;
    if(pointadded)val++;
    return val;
}

bool RailCluster::addSimilarePoint(QVector <PointGL > pts)
{
    throw Erreur("Unused");
}

bool RailCluster::growing(RailCluster rail, QVector<PointGL> pts)
{    bool pointadded=false;
     //for each point of the rail
     for(int i=0;i<rail.getPoints().size();i++){
         //test if it exists points which have the same height and which is at a distance of as em
         PointGL currentPoint=rail.getPoints().at(i);
         //test if it is not black listed
         if(!this->isBlackListed(currentPoint)){
             // for each point in the footpulse
             for(int j=0;j<pts.size();j++){
                 PointGL testPoint=pts.at(j);
                 //test if it is not black listed
                 if(!this->isBlackListed(testPoint)){
                     //test if there have the same width and same height
                     if(this->sameWidth(currentPoint,testPoint)&&this->sameHeight(currentPoint,testPoint)){
                         //add it
                         pointadded=this->addPoint(testPoint);
                     }
                 }
             }
         }
     }
      return pointadded;
}

void RailCluster::setEm(double e){
    this->em=e;
}

void RailCluster::setHm(double h){
    this->hm=h;
}

void RailCluster::setLm(double l){
    this->lm=l;
}

double RailCluster::getEm()const{
    return this->em;
}

double RailCluster::getHm()const{
    return this->hm;
}

double RailCluster::getLm()const{
    return this->lm;
}
QVector<PointGL > RailCluster::getPoints() const
{
    return points;
}

void RailCluster::setPoints(const QVector<PointGL> value)
{
    points = value;
}
QVector<PointGL > RailCluster::getBlacklist() const
{
    return blacklist;
}

void RailCluster::setBlacklist(const QVector<PointGL > value)
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


bool RailCluster::sameHeight(PointGL p1, PointGL p2) const
{
 return p1.distanceY(p2,this->delta);
}
bool RailCluster::sameWidth( PointGL p1,  PointGL p2)const
{
    return p1.distanceX(p2,this->delta);
}
bool RailCluster::widthDistance( PointGL p1,  PointGL p2)const
{
    double dsup= this->lm-this->delta;
   return p1.distanceX(p2,dsup);
}
double RailCluster::getWidthDistance()const{
    return this->lm-this->delta;
}

bool RailCluster::spacingDistance( PointGL p1,  PointGL p2)const
{
    //get distance between p1 and p2 - average spacing between 2 tracks
    double x1=p1.getX();
    double x2=p2.getX();
    double pv=x1-x2;
    if(pv<0.0)
        pv=-1.0*pv;
    //approximation

    double dsup= this->em+this->delta;
    double dinf=this->em-this->delta;
    bool lesbool=(pv<=dsup&&pv>=dinf);
    return lesbool;
}


