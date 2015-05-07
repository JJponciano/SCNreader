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

#include "Sphere.h"



Sphere::Sphere() : Objet3D(1,1,1) {
    this->meridien = 10;
    this->parallele = 10;
    this->rayon = 1;
    this->o = (3.14159265 * 2) / this->meridien;
    this->g = 3.14159265 / this->parallele;
    this->construction();
}
//==================GESTION COULEUR=============
void Sphere::setCouleur(float r, float g, float b) {
    this->couleur= new Couleur(r,g,b);
    //mise à jour de la couleur
    this->majCouleur();
}
void Sphere::setCouleur(const Couleur& c) {
    this->couleur= new Couleur(c);
    //mise à jour de la couleur
    this->majCouleur();
}
void Sphere::majCouleur() {
    // parcour toutes les facettes
    for(int i=0;i<this->faces.size();i++)
        this->faces[i]->setCouleur(*this->couleur);
}
//==================================================

Sphere::Sphere(float r,int parallele, int meridien) : Objet3D(1,1,1) {
    this->meridien = meridien;
    this->parallele = parallele;
    this->rayon = r;
     this->couleur=new Couleur(1.0,1.0,1.0);
    this->o = (3.14159265 * 2) / this->meridien;
    this->g = 3.14159265 / this->parallele;
    this->construction();
}

Sphere::Sphere(const Sphere& orig) : Objet3D(orig){
    this->meridien = orig.meridien;
    this->parallele = orig.parallele;
    this->rayon = orig.rayon;
     this->couleur=new Couleur(1.0,1.0,1.0);
    this->o = (3.14159265 * 2) / this->meridien;
    this->g = (3.14159265) / this->parallele;
}

void Sphere::construction() {
    float incI=1.0/(float)this->meridien;
    float incJ=1.0/(float)this->parallele;
    //================================ CONSTRUCTION DES POINTS==================
    float angleO(0);
    float angleG(0);
    //pour toutes les méridiens
    
    this->points.push_back(new PointGL(0, -rayon, 0));
     this->texels.push_back(new Texel(0,0));
    for (int i = 0; i<this->meridien; i++) {
        angleO = i * this->o;

        //pour tous les  parallèles méridiens
        for (int j = 1; j<this->parallele; j++) {
            angleG = -(3.14159265 / 2) + j * this->g;
            float px = (cos(angleG) * this->rayon) * sin(angleO);
            float py = (this->rayon) * sin(angleG);
            float pz = (cos(angleG) * this->rayon) * cos(angleO);
            this->points.push_back(new PointGL(px, py, pz));
             this->texels.push_back(new Texel(incI*i,incJ*j));
        }

    }
    this->points.push_back(new PointGL(0, rayon, 0));
     this->texels.push_back(new Texel(1,1));
    //==========================CONSTRUCTION DES FACETTES======================

    //POLES
    for (int i = 0; i < this->meridien; i++) {

        //_______POLE SUD_________
        // indice deuxième point de la facette
        int j = i * (this->parallele - 1) + 1;
        //indice troisième point de la facette
        int k = ((i + 1)*(this->parallele - 1)) % ((this->meridien)*(this->parallele - 1)) + 1;
        //Création et ajout de la face
        Face* face = new Face();
        face->add(points[0],this->texels[0]);
        face->add(points[j],this->texels[j]);
        face->add(points[k],this->texels[k]);
        this->faces.push_back(face);
        //_______POLE NORD_________
        // symétrie des points
        int l = this->points.size() - 1;
        //Création et ajout de la face
        Face* face2 = new Face();
        face2->add(points[this->points.size() - 1],this->texels[this->points.size() - 1]);
        face2->add(points[l - j],this->texels[l - j]);
        face2->add(points[l - k],this->texels[l - k]);
        this->faces.push_back(face2);

        //_____CORPS____
        for (int p = 0; p < this->parallele - 1; p++) {

            // indice premier point de la facette
            int a =1+i * (this->parallele-1) + p;
            // indice deuxième point de la facette
            int b1=1+((i + 1) % this->meridien) * (this->parallele-1)+ p;

            //Création et ajout de la face
            Face* face3 = new Face();
            face3->add(points[a],new Texel(i*incI,p*incJ));
            face3->add(points[b1],new Texel((i+1)*incI,(p)*incJ));
            face3->add(points[b1 + 1],new Texel((i+1)*incI,(p+1)*incJ));
            face3->add(points[a + 1],new Texel((i)*incI,(p+1)*incJ));
            this->faces.push_back(face3);
        }
    }
    this->calculeNormal();
}

void Sphere::affichePoints() {
    Objet3D::affichePoints();
}

void Sphere::affichage() {
    for (int i = 0; i<this->faces.size(); i++) 
        this->faces[i]->afficher(true);
}



Sphere::~Sphere() {
    delete couleur;
    this->couleur=0;
    
}


