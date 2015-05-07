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

#include "SLineaire.h"
/**
 * @brief Constructeur d'une surface linéaire
 * @param c vecteur de points correspondant au points appartenant à la courbe
 * de référence sur laquelle repose la surface. Exemple:
 * @code
 * Bezier* b=new Bezier(10);
 * b->Trace(vectorPtsControleC, nbp);
 * SLineaire* sl=new SLineaire(b.getP(), nbp2, p1, p2);
 * @endcode 
 * @param n nombre de méridien pour la surface
 * @param p1 premier point du segment [p1,p2] permettant la construction 
 * linéaire le long de la courbe
 * @param p2 second point du segment [p1,p2] permettant la construction 
 * linéaire le long de la courbe
 * @brief Le segment [p1,p2] est le segment que l'on balade le long 
 * de la courbe pour obtenir la surface
 */
SLineaire::SLineaire(std::vector<PointGL*> c, int n, PointGL * p1, PointGL * p2):Surface(c, n) {
    this->p0=p1;
    this->p1=p2;
    this->Creation();
    this->construction();
}

SLineaire::SLineaire(const SLineaire& orig): Surface(orig) {
}
/**
 * @brief destructeur de la classe qui détruit tous les pointeurs
 */
SLineaire::~SLineaire(){
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
 * Pour chaque points de la courbe on donne tous les points du segment
 * qui a pour origine le point de la courbe (<=> parallèle par parallèle)
 * méridiens=courbe
 * parallèle=segment
 */
void SLineaire::Creation(){
    float pas=1.0/(this->nbc-1);
    for(int i=0; i< this->Courbe.size(); i++)
    {
        this->points.push_back(this->Courbe[i]);
        for(float t=pas; t<=1.0; t+=pas)
        {
            //Calcul du décalage en x, y, et z selon l'avancement sur le segment [po,p1]
            float dx=this->InterpoLineaire(t, this->p0->getX(), this->p1->getX())-this->p0->getX();
            float dy=this->InterpoLineaire(t, this->p0->getY(), this->p1->getY())-this->p0->getY();
            float dz=this->InterpoLineaire(t, this->p0->getZ(), this->p1->getZ())-this->p0->getZ();
            //Création du point appartenant à la surface et correspondant à l'avancement sur le segment [po,p1]
            PointGL * p= new PointGL(this->Courbe[i]->getX()+dx,this->Courbe[i]->getY()+dy,this->Courbe[i]->getZ()+dz);
            this->points.push_back(p);
        }
    }
}
/**
 * @brief Fonction permettant de calculer l'interpolation linéaire entre p0 et
 * p1 à l'instant t
 * @param t instant/avancement de l'interpolation linéaire 
 * @param p0 point de départ de l'interpolation linéaire
 * @param p1 point d'arrivée de l'interpolation linéaire
 * @return la valeur correspondant à l'interpolation linéaire entre p0 et
 * p1 à l'instant t
 */
float SLineaire::InterpoLineaire(float t, float p0, float p1){
    float r=(1.0-t)*p0+t*p1;
    return r;
}
/**
 * @brief Fonction permettant l'affichage de la surface
 */
/*void SLineaire::Affichage(){
    glColor3f(1.0,0.0,1.0);
    for(int i =0; i<nbl-1; i++)
        {
                for(int j =0; j<nbc-1; j++)
                {
                    glBegin(GL_QUADS);
                        glVertex3f(this->points[i*nbc+j]->getX(),this->points[i*nbc+j]->getY(),this->points[i*nbc+j]->getZ());
                        glVertex3f(this->points[i*nbc+j+1]->getX(),this->points[i*nbc+j+1]->getY(),this->points[i*nbc+j+1]->getZ());
                        glVertex3f(this->points[(i+1)*nbc+j+1]->getX(),this->points[(i+1)*nbc+j+1]->getY(),this->points[(i+1)*nbc+j+1]->getZ());
                        glVertex3f(this->points[(i+1)*nbc+j]->getX(),this->points[(i+1)*nbc+j]->getY(),this->points[(i+1)*nbc+j]->getZ());
                    glEnd();
                }
        }
}*/

void SLineaire::construction(){
    for (int i = 0; i <nbl-1; i++) {
        for (int j = 0; j < nbc-1; j++) {
            Face* face = new Face();
                
            face->add(this->points[i*nbc+j], new Texel(0, 0));
            face->add(this->points[(i+1)*nbc+j],new Texel(1, 0));
            face->add(this->points[(i+1)*nbc+j+1],new Texel(1, 1));
            face->add(this->points[i*nbc+j+1],new Texel(0, 1));
            
            this->faces.push_back(face);
        }
    }
    this->calculeNormal();
}
