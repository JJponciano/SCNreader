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

#include "Cubique.h"

Cubique::Cubique(float largeur,float hauteur, float longueur,Texture* d,Texture* f,Texture* autre) : Objet3D(largeur, hauteur, longueur) {
    this->d = d;
    this->f=f;
    this->autre=autre;
    this->construction();
}

Cubique::Cubique(const Cubique& orig) : Objet3D(orig) {
    this->d = orig.d;
    this->largeur = orig.largeur;
    this->hauteur = orig.hauteur;
    this->longueur = orig.longueur;
}

Cubique::~Cubique() {
}

void Cubique::affichage() {
        glPushMatrix();
        glScalef(this->largeur, this->hauteur, this->longueur);
        cube();
        glPopMatrix();
}

void Cubique::construction() {
    this->constructionCube();
    this->calculeFacette();
    this->calculeNormal();
}

void Cubique::cube() {
        this->mat->boisClaire();
    glEnable(GL_TEXTURE_2D);
    //=============Declaration de la texture
    this->f->charger();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    for (int i = 0; i < this->faces.size(); i++) {
        this->faces[i]->afficher(true);
    } glDisable(GL_TEXTURE_2D);
    //======Fin de la création

}

void Cubique::constructionCube() {
    //====1
    //A
    this->texels.push_back(new Texel(0, 0));
    this->points.push_back(new PointGL(-0.5, -0.5, -0.5));
    //B
    this->texels.push_back(new Texel(0, 1));
    this->points.push_back(new PointGL(-0.5, 0.5, -0.5));
    //C
    this->texels.push_back(new Texel(1, 1));
    this->points.push_back(new PointGL(0.5, 0.5, -0.5));
    //D
    this->texels.push_back(new Texel(1, 0));
    this->points.push_back(new PointGL(0.5, -0.5, -0.5));

    //====2
    //A'
    this->texels.push_back(new Texel(0, 0));
    this->points.push_back(new PointGL(-0.5, -0.5, 0.5));
    //D'
    this->texels.push_back(new Texel(0, 1));
    this->points.push_back(new PointGL(0.5, -0.5, 0.5));
    //C'
    this->texels.push_back(new Texel(1, 1));
    this->points.push_back(new PointGL(0.5, 0.5, 0.5));
    //B'
    this->texels.push_back(new Texel(1, 0));
    this->points.push_back(new PointGL(-0.5, 0.5, 0.5));

    //====3
    //B
    this->texels.push_back(new Texel(0, 0));
    this->points.push_back(new PointGL(-0.5, 0.5, -0.5));
    //B'
    this->texels.push_back(new Texel(0, 1));
    this->points.push_back(new PointGL(-0.5, 0.5, 0.5));
    //C'
    this->texels.push_back(new Texel(1, 1));
    this->points.push_back(new PointGL(0.5, 0.5, 0.5));
    //C
    this->texels.push_back(new Texel(1, 0));
    this->points.push_back(new PointGL(0.5, 0.5, -0.5));
    //====4
    //A
    this->texels.push_back(new Texel(0, 0));
    this->points.push_back(new PointGL(-0.5, -0.5, -0.5));
    //A'
    this->texels.push_back(new Texel(0, 1));
    this->points.push_back(new PointGL(-0.5, -0.5, 0.5));
    //B'
    this->texels.push_back(new Texel(1, 1));
    this->points.push_back(new PointGL(-0.5, 0.5, 0.5));
    //B
    this->texels.push_back(new Texel(1, 0));
    this->points.push_back(new PointGL(-0.5, 0.5, -0.5));




    //====5
    //A
    this->texels.push_back(new Texel(0, 0));
    this->points.push_back(new PointGL(-0.5, -0.5, -0.5));

    //A'
    this->texels.push_back(new Texel(0, 1));
    this->points.push_back(new PointGL(-0.5, -0.5, 0.5));


    //D'
    this->texels.push_back(new Texel(1, 1));
    this->points.push_back(new PointGL(0.5, -0.5, 0.5));

    //D
    this->texels.push_back(new Texel(1, 0));
    this->points.push_back(new PointGL(0.5, -0.5, -0.5));

    //====6
    //D
    this->texels.push_back(new Texel(0, 0));
    this->points.push_back(new PointGL(0.5, -0.5, -0.5));

    //D'
    this->texels.push_back(new Texel(1, 0));
    this->points.push_back(new PointGL(0.5, -0.5, 0.5));

    //C'
    this->texels.push_back(new Texel(1, 1));
    this->points.push_back(new PointGL(0.5, 0.5, 0.5));

    //C
    this->texels.push_back(new Texel(0, 1));
    this->points.push_back(new PointGL(0.5, 0.5, -0.5));

}
