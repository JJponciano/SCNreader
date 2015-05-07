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


#include "LesTextures.h"
LesTextures::LesTextures() {
    this->key=0;
}

LesTextures::LesTextures(const LesTextures& orig) {
}

void LesTextures::add(Texture* t){
    if(nomValide(t->getNnom())){
        this->textures.push_back(t);
    }
}

bool LesTextures::nomValide(std::string nom) {
    // teste si le nom est déjà connue pour chaque texture 
    for(int i=0;i<this->textures.size();i++){
        if(this->textures[i]->getNnom()==nom)return false;
    }return true;
}

void LesTextures::add(QString adresse, std::string nom,GLuint id){
     if(nomValide(nom)){ 
    //ajout de la nouvelle texture
    this->textures.push_back(new Texture(adresse,nom,id));
     }
}
void LesTextures::add(QString adresse, std::string nom){
     if(nomValide(nom)){
       this->key++;
    //ajout de la nouvelle texture
    this->textures.push_back(new Texture(adresse,nom,this->key));
     }
}

void LesTextures::afficheTexture(std::string nom) const{
    //parcours toutes les textures
    int i=0;
    bool trouve=false;
    while(i<this->textures.size()&&!trouve){
        // si la texture courante porte le nom souhaité
        std::string tnom=this->textures[i]->getNnom();
        if(tnom==nom){
            //fin de la boucle
            trouve=true;
            //affichage de la texture
            this->textures[i]->charger();
        }else i++;
    }
    
}
    
Texture* LesTextures::getTexture(std::string nom){
  for(int i=0;i<this->textures.size();i++){
      if(this->textures[i]->getNnom()==nom)
    return this->textures[i];
  }
  return textures[0];
}
LesTextures::~LesTextures() {
    for(int i=0;i<this->textures.size();i++){
     delete this->textures[i];
     this->textures[i]=0;
    }
}

