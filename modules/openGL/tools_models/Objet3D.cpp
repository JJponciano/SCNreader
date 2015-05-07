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

#include "Objet3D.h"


Objet3D::Objet3D(float largeur, float hauteur, float longueur) {
    this->hauteur = hauteur;
    this->largeur = largeur;
    this->longueur = longueur;
    this->mat = new Materiel();
}

Objet3D::Objet3D(const Objet3D& orig) {
    this->hauteur = orig.hauteur;
    this->largeur = orig.largeur;
    this->longueur = orig.longueur;
}

Objet3D::~Objet3D() {
    for (int i = 0; i < points.size(); i++) {
        delete points[i];
        points[i] = 0;
    }
    for (int i = 0; i < texels.size(); i++) {
        delete texels[i];
        texels[i] = 0;
    }
    for (int i = 0; i < faces.size(); i++) {
        delete faces[i];
        faces[i] = 0;
    }
    delete this->mat;

}

void Objet3D::affichePoints() const {
    for (int i = 0; i < points.size(); i++) {
        glPushMatrix();
        float r = (float) i / (float) points.size();
        if (i == 0)
            glColor3f(1, 0, 0);
        else
            if (i == 1)
            glColor3f(0, 1, 0);
        else
            if (i == 2)
            glColor3f(0, 0, 1);
        else
            if (i == 3)
            glColor3f(0, 1, 1);
        else
            if (i == points.size() - 1)
            glColor3f(1, 0, 1);
        else
            glColor3f(1, 1, 1);
        glPointSize(5);
        glBegin(GL_POINTS);

        glVertex3f(points[i]->getX(), points[i]->getY(), points[i]->getZ());
        glEnd();
        glPopMatrix();
    }
}

void Objet3D::affichage() const {

}

void Objet3D::calculeNormal() {
    //pour chaque point 
    for (int i = 0; i<this->points.size(); i++) {
        // récupération des faces contenant le point
        float nbFace = 0;
        float mx = 0;
        float my = 0;
        float mz = 0;
        for (int j = 0; j<this->faces.size(); j++) {
            // si la face contient le sommet
            if (this->faces[j]->contain(this->points[i])) {
                nbFace++;
                mx += this->faces[j]->getNormal()[0];
                my += this->faces[j]->getNormal()[1];
                mz += this->faces[j]->getNormal()[2];
            }
        }
        // calcule de la moyenne des normales des faces contenant le point
        mx /= nbFace;
        my /= nbFace;
        mz /= nbFace;
        //affectation de la normale moyenne au point
        this->points[i]->setNormalMoyenne(mx, my, mz);
    }

}

void Objet3D::construction() {

}

float Objet3D::getHauteur() const {
    return this->hauteur;
}

float Objet3D::getLargeur() const {
    return this->largeur;
}

float Objet3D::getLongeur() const {
    return this->longueur;
}

void Objet3D::calculeFacette() {
    //pour chaque face
    for (int i = 0; i < this->points.size() / 4; i++) {
        //Création de la face
        Face* face = new Face();
        for (int j = 0; j < 4; j++) {
            face->add(this->points[i * 4 + j], this->texels[i * 4 + j]);
        }
        //ajout de la face
        this->faces.push_back(face);
    }
}

void Objet3D::animationLineaire(std::vector<PointGL*> pointsCles, float a ) {
    // si les deux tableau n'ont pas le même nombre de points, il y a une erreur
    if (this->pointsReferents.size() != pointsCles.size())
        std::cerr <<this->pointsReferents.size() << " nombre de points clés différent du nombre de points de l'objet à animer" << pointsCles.size()<< std::endl;
    else {           

        //pour chaque point de l'objets
        for (int i = 0; i<this->pointsReferents.size(); i++) {
            // récupération des coordonnées du point
            float x = this->pointsReferents[i]->getX();
            float y = this->pointsReferents[i]->getY();
            float z = this->pointsReferents[i]->getZ();
            // interpolation linéaire  jusqu'au point clé selon l'avancement a
            x = (1 - a) * x + a * pointsCles[i]->getX();
            y = (1 - a) * y + a * pointsCles[i]->getY();
            z = (1 - a) * z + a * pointsCles[i]->getZ();
            // affectation des changements
            this->pointsReferents[i]->setX(x);
            this->pointsReferents[i]->setY(y);
            this->pointsReferents[i]->setZ(z);
        }
    }
}

std::vector <PointGL*> Objet3D::getP(){
    return this->points;
}
