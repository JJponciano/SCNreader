/**
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

#include "Bezier.h"
Bezier::Bezier(int n) {
    this->nbp = n;
}

Bezier::Bezier(const Bezier& orig){
    
}

Bezier::~Bezier() {
    for(int i=0; i<this->Points.size(); i++)
    {
        delete this->Points[i];
        this->Points[i]=0;
    }
}

void Bezier::Trace(std::vector<PointGL*> P) {
       

    float N = P.size();
      float nb=3.0*this->nbp/(N-1);
    if (true) {
        float t = 0.0;
        float pas = 1.0 / nb;
        for(int i=0;i<N-3;i+=3)
        {
            t = 0.0;
            while (t <= 1.0) {

                float x = FuncX(t, P[i], P[i + 1], P[i + 2], P[i + 3]);
                float y = FuncY(t, P[i], P[i + 1], P[i + 2], P[i + 3]);
                float z = FuncZ(t, P[i], P[i + 1], P[i + 2], P[i + 3]);
                PointGL* p = new PointGL(x, y, z);
                p->setNormalMoyenne(this->FuncNormale(t, P[i], P[i + 1], P[i + 2], P[i + 3]));
                this->Points.push_back(p);
                
                t = t + pas;
            }
        }
    }
    /*for (int i = 0; i < P.size(); i++) {
        delete P[i];
        P[i] = 0;
    }*/
}


float Bezier::FuncX(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4) {
    float x = (1.0 - t)*(1.0 - t)*(1.0 - t) * p1->getX();
    x += 3.0 * t * (1.0 - t)*(1.0 - t) * p2->getX();
    x += 3.0 * t * t * (1.0 - t) * p3->getX();
    x += t * t * t * p4->getX();
    return x;
}


float Bezier::FuncY(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4) {
    float y = (1.0 - t)*(1.0 - t)*(1.0 - t) * p1->getY();
    y += 3.0 * t * (1.0 - t)*(1.0 - t) * p2->getY();
    y += 3.0 * t * t * (1.0 - t) * p3->getY();
    y += t * t * t * p4->getY();
    return y;
}

float Bezier::FuncZ(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4) {
    float z = (1.0 - t)*(1.0 - t)*(1.0 - t) * p1->getZ();
    z += 3.0 * t * (1.0 - t)*(1.0 - t) * p2->getZ();
    z += 3.0 * t * t * (1.0 - t) * p3->getZ();
    z += t * t * t * p4->getZ();
    return z;
}

std::vector<float> Bezier::FuncNormale(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4) {
    std::vector <float> normal;
    
    float x = (-3*t*t + 6*t -3)* p1->getX();
    x += (9*t*t - 12*t +3)* p2->getX();
    x += (-9*t*t + 6*t)* p3->getX();
    x += (3*t*t)* p4->getX();
    normal.push_back(x);
    
    float y = (-3*t*t + 6*t -3)* p1->getY();
    y += (9*t*t - 12*t +3)* p2->getY();
    y += (-9*t*t + 6*t)* p3->getY();
    y += (3*t*t)* p4->getY();
    normal.push_back(y);
    
    float z = (-3*t*t + 6*t -3)* p1->getZ();
    z += (9*t*t - 12*t +3)* p2->getZ();
    z += (-9*t*t + 6*t)* p3->getZ();
    z += (3*t*t)* p4->getZ();
    normal.push_back(z);
    
    return normal;
}

void Bezier::Affichage() {

    glPointSize(2.0);
    for (int i = 0; i<this->Points.size(); i++) {
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        glVertex2f(this->Points[i]->getX(), this->Points[i]->getY());
        glEnd();
    }
}

std::vector<PointGL*> Bezier::getP() {
    return this->Points;
}
