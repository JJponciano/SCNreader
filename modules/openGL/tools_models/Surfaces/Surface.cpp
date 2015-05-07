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



#include "Surface.h"

/**
 * @brief Constructeur d'une surface
 * @param c vecteur de points correspondant au points appartenant à la courbe
 * de référence sur laquelle repose la surface. Exemple:
 * @code
 * Bezier* b=new Bezier(10);
 * b->Trace(vectorPtsControleC, nbp);
 * Surface* sl=new Surface(b.getP(), nbp2);
 * @endcode 
 * @param n nombre de méridien pour la surface
 */
Surface::Surface(std::vector<PointGL*> c, int n) : Objet3D(0,0,0) {
    this->Courbe = c;
    this->nbl = this->Courbe.size();
    this->nbc = n;
}

Surface::Surface(const Surface& orig)  : Objet3D(orig){
    this->largeur = orig.largeur;
    this->hauteur = orig.hauteur;
    this->longueur = orig.longueur;
}

/**
 * @brief destructeur de la classe qui détruit tous les pointeurs
 */
Surface::~Surface() {
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
 * @brief Fonction permettant l'affichage de la surface
 */
void Surface::construction() {
    float incI=1.0/(nbl);
    float incJ=1.0/(nbc);
    for (int i = 0; i <nbl-1; i++) {
        for (int j = 0; j < nbc; j++) {
            Face* face = new Face();
            face->add(this->points[i * nbc + j], new Texel(i*incI,j*incJ));
            face->add(this->points[(i + 1) * nbc + j], new Texel((i+1)*incI,j*incJ));
            face->add(this->points[(i + 1) * nbc + ((j + 1)%nbc)], new Texel((i+1)*incI,(j+1)*incJ));
            face->add(this->points[i * nbc + ((j + 1)%nbc)],new Texel((i)*incI,(j+1)*incJ));
            
            this->faces.push_back(face);
     /*       if(i<=1)
            std::cout<<i * nbc + j<<", "<< (i + 1) * nbc + j<<", "<<(i + 1) * nbc + ((j + 1)%nbc)<<", "<< i * nbc + ((j + 1)%nbc)<<std::endl;
       
        
       std::cout<<this->points[i * nbc + j]->getX()<<", "<<this->points[i * nbc + j]->getY()<<", "<< this->points[i * nbc + j]->getZ()<<std::endl;
            std::cout<<this->points[(i + 1) * nbc + j]->getX()<<", "<<this->points[(i + 1) * nbc + j]->getY()<<", "<<   this->points[(i + 1) * nbc + j]->getZ()<<", "<<std::endl;
            std::cout<<this->points[(i + 1) * nbc + ((j + 1)%nbc)]->getX()<<", "<<this->points[(i + 1) * nbc + ((j + 1)%nbc)]->getY()<<", "<< this->points[(i + 1) * nbc + ((j + 1)%nbc)]->getZ()<<std::endl;
               std::cout<<this->points[i * nbc + ((j + 1)%nbc)]->getX()<<", "<<this->points[i * nbc + ((j + 1)%nbc)]->getY()<<", "<<this->points[i * nbc + ((j + 1)%nbc)]->getZ()<<std::endl;
       */
        }
    }
    this->calculeNormal();
}

void Surface::affichage() {
 for (int i = 0; i<this->faces.size(); i++) 
        this->faces[i]->afficher(true);
}
