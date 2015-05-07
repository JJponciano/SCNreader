/*
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 19 Octobre 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale - 
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante 
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
 */

#include "Couleur.h"

Couleur::Couleur() {
    this->r=1;
    this->g=1;
    this->b=1;
}

Couleur::Couleur(const Couleur& orig) {
    this->r=orig.r;
    this->g=orig.g;
    this->b=orig.b;
}

Couleur::Couleur(float r, float g, float b) {
    this->r=r;
    this->g=g;
    this->b=b;
}

Couleur::~Couleur() {
}

float Couleur::getB() const {
    return this->b;
}

float Couleur::getG() const {
    return this->g;
}

float Couleur::getR() const {
    return this->r;
}

