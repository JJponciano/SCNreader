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
#include "Tore.h"

Tore::Tore():Objets(){
    this->couleur=new Couleur(1.0,1.0,1.0);
    this->R = 2;
    this->r = 1;
    this->parallele = 100;
    this->meridien = 100;
    this->o = (3.14159265 * 2) / 100;
    this->g = (3.14159265 * 2) / 100;
    this->construction();
}

Tore::Tore(float r,float R, int parallele, int meridien):Objets() {
    this->R = R;
    this->couleur=new Couleur(1.0,1.0,1.0);
    this->r = r;
    this->parallele = parallele;
    this->meridien = meridien;
    this->o = (3.14159265 * 2) / meridien;
    this->g = (3.14159265 * 2) / parallele;
    this->construction();
    
}

Tore::Tore(const Tore& orig){
    this->couleur=new Couleur(1.0,1.0,1.0);
    this->R = orig.R;
    this->r = orig.r;
    this->parallele = orig.parallele;
    this->meridien = orig.meridien;
    this->o = (3.14159265 * 2) / meridien;
    this->g = (3.14159265 * 2) / parallele;
    this->construction();
}

void Tore::affichePoints()const{
    Objets::affichePoints();
}
//==================GESTION COULEUR=============
void Tore::setCouleur(float r, float g, float b) {
    this->couleur= new Couleur(r,g,b);
    //mise à jour de la couleur
    this->majCouleur();
}
void Tore::setCouleur(const Couleur& c) {
    this->couleur= new Couleur(c);
    //mise à jour de la couleur
    this->majCouleur();
}
void Tore::majCouleur() {
    // parcour toutes les facettes
    for(int i=0;i<this->faces.size();i++)
        this->faces[i]->setCouleur(*this->couleur);
}
//==================================================

void Tore::construction() {
    //================================ CONSTRUCTION DES POINTS==================
    float angleO(0);
    float angleG(0);
    //pour toutes les méridiens
    for (int i = 0; i<this->meridien; i++) {
        angleO = i * this->o;

        //pour tous les  parallèles méridiens
        for (int j = 0; j<this->parallele; j++) {
            angleG = j * this->g;
            float px = (this->R + cos(angleG) * this->r) * cos(angleO);
            float py = (this->R + cos(angleG) * this->r) * sin(angleO);
            float pz = (this->r) * sin(angleG);
            this->points.push_back(new PointGL(px, py, pz));
        }
    }
    //==========================CONSTRUCTION DES FACETTES=======================
     //parcours tous les points du tore
    for (int i = 0; i < points.size(); i++) {
        // création d'une facette 

        //premier point du disque suivant
        int j = (i + this->parallele) % this->points.size();
        //second point du disque suivant
        int k = (i + 1 + this->parallele) % this->points.size();
        // si k déborde du disque
        if (k == 0)k = this->points.size() - this->parallele;
        else
            if ((k) % this->parallele == 0) {
            // on le rammène au debut du disque
            k -= this->parallele;
        }
        // deuxième point du disque
        int l = (i + 1);
        // si l déborde du disque
        if ((l) % this->parallele == 0) {
            // on le rammène au debut du disque
            l -= this->parallele;
        }

        //Création et ajout de la face
        Face* face= new Face();
        face->add(points[i],new Texel(0,0));
        face->add(points[j],new Texel(0,1));
        face->add(points[k],new Texel(1,1));
        face->add(points[l],new Texel(1,0));
        face->setCouleur(*this->couleur);
        this->faces.push_back(face);
    }
    this->calculeNormal();
}

void Tore::affichage()const {
    Objets::affichage();
}

Tore::~Tore() {
    for (int i = 0; i < points.size(); i++) {
        delete points[i];
        points[i] = 0;
    }
    for (int i = 0; i < points.size(); i++) {
          this->points.pop_back();
      }
}

