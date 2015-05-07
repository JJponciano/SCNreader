/*
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 13 Mars 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale - 
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante 
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
 */
#include "BSpline.h"

BSpline::BSpline(int nb) {
    this->nbp=nb;
}

BSpline::~BSpline() {
    for(int i=0; i<this->Points.size(); i++)
    {
        delete this->Points[i];
        this->Points[i]=0;
    }
}

void BSpline:: Trace(std::vector<PointGL*> P){
    float pas;
    int N=P.size();
        if(N>4){
            pas=1.0/(this->nbp*(N-3));
            for(int i=4; i<=N;i++)
            {
                float t=0.0;
                while(t<1.0)
                {
                    float x=FuncX(t,P[i-4],P[i-3],P[i-2],P[i-1]);
                    float y=FuncY(t,P[i-4],P[i-3],P[i-2],P[i-1]);
                    float z=FuncZ(t,P[i-4],P[i-3],P[i-2],P[i-1]);
                    PointGL* p=new PointGL(x,y,z);
                    p->setNormalMoyenne(FuncNormale(t,P[i-4],P[i-3],P[i-2],P[i-1]));
                    this->Points.push_back(p);
                    t=t+pas;
                }
            }
    }
   else if(N==4)
        {
            pas=1.0/this->nbp;
            float t=0.0;
                while(t<1.0)
                {
                    float x=FuncX(t,P[N-4],P[N-3],P[N-2],P[N-1]);
                    float y=FuncY(t,P[N-4],P[N-3],P[N-2],P[N-1]);
                    float z=FuncZ(t,P[N-4],P[N-3],P[N-2],P[N-1]);
                    PointGL* p=new PointGL(x,y,z);
                    p->setNormalMoyenne(FuncNormale(t,P[N-4],P[N-3],P[N-2],P[N-1]));
                    this->Points.push_back(p);
                    t=t+pas;
                }
         }
}

float BSpline::FuncX(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4){
    float x=(-(1.0/6.0)*t*t*t+(1.0/2.0)*t*t-(1.0/2.0)*t+(1.0/6.0))*p1->getX();
    x+=((1.0/2.0)*t*t*t-1*t*t+(2.0/3.0))*p2->getX();
    x+=(-(1.0/2.0)*t*t*t+(1.0/2.0)*t*t+(1.0/2.0)*t+(1.0/6.0))*p3->getX();
    x+=(1.0/6.0)*t*t*t*p4->getX();
    return x;
}

float BSpline::FuncY(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4){
    float y=(-(1.0/6.0)*t*t*t+(1.0/2.0)*t*t-(1.0/2.0)*t+(1.0/6.0))*p1->getY();
    y+=((1.0/2.0)*t*t*t-1*t*t+(2.0/3.0))*p2->getY();
    y+=(-(1.0/2.0)*t*t*t+(1.0/2.0)*t*t+(1.0/2.0)*t+(1.0/6.0))*p3->getY();
    y+=(1.0/6.0)*t*t*t*p4->getY();
    return y;
}

float BSpline::FuncZ(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4){
    float z=(-(1.0/6.0)*t*t*t+(1.0/2.0)*t*t-(1.0/2.0)*t+(1.0/6.0))*p1->getZ();
    z+=((1.0/2.0)*t*t*t-1*t*t+(2.0/3.0))*p2->getZ();
    z+=(-(1.0/2.0)*t*t*t+(1.0/2.0)*t*t+(1.0/2.0)*t+(1.0/6.0))*p3->getZ();
    z+=(1.0/6.0)*t*t*t*p4->getZ();
    return z;
}

std::vector <float> BSpline::FuncNormale(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4){
    std::vector <float> normal;
    
    float x=(-(1.0/6.0)*3*t*t+(1.0/2.0)*2*t-(1.0/2.0))*p1->getX();
    x+=((1.0/2.0)*3*t*t-1*2*t)*p2->getX();
    x+=(-(1.0/2.0)*3*t*t+(1.0/2.0)*2*t+(1.0/2.0))*p3->getX();
    x+=(1.0/6.0)*3*t*t*p4->getX();
    normal.push_back(x);
    
    float y=(-(1.0/6.0)*3*t*t+(1.0/2.0)*2*t-(1.0/2.0))*p1->getY();
    y+=((1.0/2.0)*3*t*t-1*2*t)*p2->getY();
    y+=(-(1.0/2.0)*3*t*t+(1.0/2.0)*2*t+(1.0/2.0))*p3->getY();
    y+=(1.0/6.0)*3*t*t*p4->getY();
    normal.push_back(y);
    
    float z=(-(1.0/6.0)*3*t*t+(1.0/2.0)*2*t-(1.0/2.0))*p1->getZ();
    z+=((1.0/2.0)*3*t*t-1*2*t)*p2->getZ();
    z+=(-(1.0/2.0)*3*t*t+(1.0/2.0)*2*t+(1.0/2.0))*p3->getZ();
    z+=(1.0/6.0)*3*t*t*p4->getZ();
    normal.push_back(z);
    
    return normal;
}

void BSpline:: Affichage(){
    
        glPointSize(2.0);
        for(int i=0;i<this->Points.size();i++)
        {
            glBegin(GL_POINTS);
                    glColor3f(0.0,1.0,1.0);
                    glVertex2f(this->Points[i]->getX(),this->Points[i]->getY());
            glEnd();
        }
}

std::vector<PointGL*> BSpline:: getP(){
    return this->Points;
}