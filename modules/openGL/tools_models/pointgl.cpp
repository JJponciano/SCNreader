/**
*  @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
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
#include "pointgl.h"

PointGL::PointGL() {
    rad=(double)3.14159265/(double)180.0;
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->epsilon=1000;
}

PointGL::PointGL(double x, double y,double z) {
    rad=(double)3.14159265/(double)180.0;
    this->x = x;
    this->y = y;
    this->z = z;
    this->epsilon=1000;
}

PointGL::PointGL(const PointGL& orig) {
    rad=(double)3.14159265/(double)180.0;
    this->x = orig.x;
    this->y = orig.y;
    this->z = orig.z;
    this->epsilon=1000;
}


double PointGL::truncation(int trunc, double f){
    int ftrunc=(int)(f*trunc);
    double f2=((double)ftrunc)/((double)trunc);
            return f2;
}
int PointGL::getEpsilon() const
{
    return epsilon;
}

void PointGL::setEpsilon(int value)
{
    if(value%10==0&&value>0)
    epsilon = value;
}


bool PointGL::operator==(const PointGL &a)const
{
    //rounded to the thousandth
    int roundX=int(this->x*epsilon );
    int roundY=(int)(this->y*epsilon );
    int roundZ=(int)(this->z*epsilon) ;

    int aX=(int)(a.getX()*epsilon );
    int aY=(int)(a.getY()*epsilon );
    int aZ=(int)(a.getZ()*epsilon );

    return roundX== aX&&roundY== aY &&roundZ== aZ;

}

bool PointGL::operator<(const PointGL &r)const
{
    //sort by Z
    if(this->getZ()!=r.getZ()){
        return this->getZ()<r.getZ();
    }else{
        //sort by X
        if(this->getX()!=r.getX()){
            return this->getX()<r.getX();
        }else{
            //sort by Y
            return this->getY()<r.getY();
        }
    }
}
bool PointGL::operator!=(const PointGL &a)
{
    //rounded to the thousandth
    int roundX=int(this->x*epsilon) ;
    int roundY=int(this->y*epsilon) ;
    int roundZ=int(this->z*epsilon) ;

    int aX=int(a.getX()*epsilon) ;
    int aY=int(a.getY()*epsilon) ;
    int aZ=int(a.getZ()*epsilon) ;

    return roundX!= aX||roundY!= aY ||roundZ!= aZ;
}

void PointGL::operator-(const PointGL &a)
{
    this->x-=a.getX();
    this->y-=a.getY();
    this->z-=a.getZ();
}

bool PointGL::distanceX(const PointGL point,double distance)const
{
    if(distance<0.0)
        distance=-1.0*distance;
        //get distance between p1 and p2 - average spacing between 2 tracks
        double x1=this->getX();
        double x2=point.getX();
        double pv= x1-x2;
        if(pv<0.0)
            pv=-1.0*pv;
          //std::cout<<" Point: "<<point.getX()<<" , "<<this->getX()<<" | "<<distance<<"->"<<(pv<distance)<<std::endl;
        return pv<distance;

}
bool PointGL::distanceY(const PointGL point,double distance)const
{
    if(distance<0.0)
        distance=-1.0*distance;
        //get distance between p1 and p2 - average spacing between 2 tracks
        double y1=this->getY();
        double y2=point.getY();
        double pv=y1-y2;
        if(pv<0.0)
            pv=-1.0*pv;
        return pv<distance;
}
bool PointGL::equals2D(const PointGL a)const{
    //rounded to the thousandth
    int roundX=int(this->x*epsilon );
    int roundY=int(this->y*epsilon );


    int aX=int(a.getX()*epsilon );
    int aY=int(a.getY()*epsilon );

    return roundX== aX&&roundY== aY;
}

PointGL::~PointGL() {
}
void PointGL::setNormalMoyenne(std::vector<double> n){
    this->normalMoyenne.clear();
    double z=n[2];
    double y=n[1];
    double x=n[0];
    this->normalMoyenne.push_back(x);
    this->normalMoyenne.push_back(y);
    this->normalMoyenne.push_back(z);
}
void PointGL::setNormalMoyenne(double x, double y, double z){
     this->normalMoyenne.clear();
    this->normalMoyenne.push_back(x);
    this->normalMoyenne.push_back(y);
    this->normalMoyenne.push_back(z);
}
double PointGL::getX() const {
    return this->x;
}
std::vector<double> PointGL::getNormal() const{
    return this->normalMoyenne;
}
double PointGL::getY() const {
    return this->y;
}

double PointGL::getZ() const {
    return this->z;
}

void PointGL::setX(double a) {
    this->x=a;
}

void PointGL::setY(double a) {
    this->y=a;
}

void PointGL::setZ(double a) {
    this->z=a;
}

void PointGL::rotation(double a, int x, int y, int z){
     if(x==1){
        double ay=this->getY();
        double az=this->getZ();
    this->setY(ay*cos(a*rad)-sin(a*rad)*az);
    this->setZ(ay*sin(a*rad)+az*cos(a*rad));
    }
    
    if(y==1){
        double ax=this->getX();
        double az=this->getZ();
    this->setX(ax*cos(a*rad)+az*sin(a*rad));
    this->setZ(-ax*sin(a*rad)+az*cos(a*rad));
    }
    
    if(z==1){
        double ax=this->getX();
        double ay=this->getY();
    this->setX(ax*cos(a*rad)-ay*sin(a*rad));
    this->setY(ax*sin(a*rad)+ay*cos(a*rad));
}
}
void PointGL::translation(double tx, double ty, double tz){
    this->x=this->x+tx;
    this->y=this->y+ty;
    this->z=this->z+tz;
}

void PointGL::scale(double sx, double sy, double sz){
    this->x=this->x*sx;
    this->y=this->y*sy;
    this->z=this->z*sz;
}
