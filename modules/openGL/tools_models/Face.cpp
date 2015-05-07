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
#include "Face.h"


Face::Face() {
    this->setCouleur(1,1,1);
}

void Face::add(PointGL* p,Texel*t) {
    // ajout du points au tableau dynamique
    this->points.push_back(p);
    this->texels.push_back(t);
    // dés que la face à trois points, calcule de sa normal
    if (this->points.size() == 3)this->Normale();
}

void Face::remove() {
    this->points.pop_back();
}

Face::Face(const Face& orig) {
    for (int i = 0; i < orig.points.size(); i++) {
        this->add(new PointGL(*orig.points[i]),new Texel(*orig.texels[i]));
    }
}

void Face::afficher(bool lisage) {
   
    glBegin(GL_POLYGON);
    for (int i = 0; i<this->points.size(); i++) {
        if(lisage){
        // récupération du vecteur normal moyen au sommet
        std::vector < float > normal = this->points[i]->getNormal();
        this->Vecteur_Unite(normal);
        //affectation
        glNormal3f(normal[0], normal[1], normal[2]);
         }
        glTexCoord2f(this->texels[i]->getX(),this->texels[i]->getY());
        glVertex3f(points[i]->getX(), points[i]->getY(), points[i]->getZ());
    }
    glEnd();
}

void Face::setCouleur(float r, float g, float b) {
    this->couleur = new Couleur(r, g, b);
}

void Face::setCouleur(const Couleur& c) {
    this->couleur = new Couleur(c);
}

void Face::afficherPoints() {
    for (int i = 0; i < points.size(); i++) {
        glPushMatrix();
        glColor3f(this->couleur->getR(), this->couleur->getG(), this->couleur->getB());
        glBegin(GL_POINTS);
        glVertex3f(points[i]->getX(), points[i]->getY(), points[i]->getZ());
        glEnd();
        glPopMatrix();
    }
}

void Face::Vecteur_Unite(std::vector<float> vector) {
    float length;

    // Calcul de la norme du vecteur                
    length = (float) sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]) + (vector[2] * vector[2]));

    if (length == 0.0f) length = 1.0f; //évite une violente erreur
    vector[0] /= length;
    vector[1] /= length;
    vector[2] /= length;
}

// Points p1, p2, & p3 spécifiés dans le sens trigonométrique

void Face::Normale() {
    // Calcul de 2 vecteurs à partir des 3 points
    float v1x = this->points[0]->getX() - this->points[1]->getX();
    float v1y = this->points[0]->getY() - this->points[1]->getY();
    float v1z = this->points[0]->getZ() - this->points[1]->getZ();
    float v2x = this->points[1]->getX() - this->points[2]->getX();
    float v2y = this->points[1]->getY() - this->points[2]->getY();
    float v2z = this->points[1]->getZ() - this->points[2]->getZ();

    this->normal.clear();
    // calcul du produit vectoriel
    this->normal.push_back(v1y * v2z - v1z * v2y);
    this->normal.push_back(v1z * v2x - v1x * v2z);
    this->normal.push_back(v1x * v2y - v1y * v2x);

    // on le réduit à un vecteur unité
    this->Vecteur_Unite(this->normal);
}

std::vector<float>Face::getNormal() const {
    return this->normal;
}

Face::~Face() {
    for (int i = 0; i < points.size(); i++) {
        delete points[i];
        points[i] = 0;
    }
    delete couleur;
    couleur = 0;
    ;
}

bool Face::contain(PointGL* p) const {
    // test si le point appartient à un point de la face
    for (int i = 0; i<this->points.size(); i++) {
        if (this->points[i] == p)return true;
    }
    return false;
}
