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

#include "Objets.h"

void Objets::affichePoints() const {
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

Objets::Objets() {
    this->textureActive = false;
}

Objets::Objets(const Objets& orig) {

}

void Objets::setTexture(Texture* text) {
    this->textureActive = true;
    this->t = text;
}

void Objets::affichage() const {
    //affichage des facettes
    for (int i = 0; i < this->faces.size(); i++) {
        this->faces[i]->afficher(true);
    }
}

void Objets::calculeNormal() {
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

void Objets::construction() {

}

Objets::~Objets() {

}

