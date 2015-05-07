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

#include "SRevolution.h"
/**
 * @brief Constructeur d'une surface de révolution
 * @param c vecteur de points correspondant au points appartenant à la courbe
 * de référence sur laquelle repose la surface. Exemple:
 * @code
 * Bezier* b=new Bezier(10);
 * b->Trace(vectorPtsControleC, nbp);
 * SRevolution* sl=new SRevolution(b.getP(), nbp2);
 * @endcode 
 * @param n nombre de méridien pour la surface
 */
SRevolution::SRevolution(std::vector<PointGL*> c, int n):Surface(c, n) {
    this->pi=3.14159265;
    this->creation();
    this->construction();
}

SRevolution::SRevolution(const SRevolution& orig) : Surface(orig) {
}

/**
 * @brief destructeur de la classe qui détruit tous les pointeurs
 */
SRevolution::~SRevolution() {
    //destruction du tableau des points de la surface
    for (int i = 0; i < this->points.size(); i++) {
        delete this->points[i];
        this->points[i] = 0;
    }
    //destruction du tableau des points de la courbe
    for (int i = 0; i < this->Courbe.size(); i++) {
        delete this->Courbe[i];
        this->Courbe[i] = 0;
    }
}

/**
 * @brief Construction de la surface par ajout des points dans Surface
 * L'ajout se fait selon l'ordre suivant:
 * Pour chaque points de la courbe on donne tous les points du cercle
 * correspondant au niveau du point de la courbe (<=> parallèle par parallèle)
 * méridiens=courbe
 * parallèle=cercle
 */
void SRevolution::creation(){
   float pas=(2.0*this->pi)/this->nbc;
   float incI=1.0/(float)this->Courbe.size();
    float incJ=1.0/(float)2.0*this->pi;
    for(int i=0; i< this->Courbe.size(); i++)
    { 
        for(float t=0; t<2.0*this->pi; t+=pas)
        {
            //rotation de chaque point autour de l'axe y
            float x=this->Courbe[i]->getX()*cos(t);
            float y=this->Courbe[i]->getY();
            float z=-this->Courbe[i]->getX()*sin(t);
            //Création du point appartenant à la surface et correspondant à l'avancement sur le segment [po,p1]
            PointGL * p= new PointGL(x,y,z);
            this->points.push_back(p);
             this->texels.push_back(new Texel(incI*i,incJ*t));
        }
    }
}
