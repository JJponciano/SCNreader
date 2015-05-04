/**
*  @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
* All rights reserved.
* This file is part of scn reader.
*
* scn reader is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* scn reader is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar.  If not, see <http://www.gnu.org/licenses/>
* @author Jean-Jacques PONCIANO and Claire PRUDHOMME
* Contact: ponciano.jeanjacques@gmail.com
* @version 0.1
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

