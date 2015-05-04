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
#include "model_ground_gl.h"

model_ground_GL::model_ground_GL()
{
    lt=new LesTextures();
    this->angleZoome=45;
    this->scale=0.5;
}

model_ground_GL::~model_ground_GL()
{

}
void model_ground_GL::addTexture(QString path, QString nom){
    this->lt->add(path, nom.toStdString());
}

Texture * model_ground_GL::getTexture(QString name){
    return this->lt->getTexture(name.toStdString());
}

float model_ground_GL::getZoom(){
    return this->angleZoome;
}

void model_ground_GL::setZoom(float newZoom){
    if(newZoom<0)throw Erreur(" The zoom for openGL view is less than 0");
    else this->angleZoome=newZoom;
}

void model_ground_GL::setCameraLocation(float x, float y,float z){
    this->scene.setCameraLocation(x,y,z);
}

void model_ground_GL::setCameraLookAt(float x,float y,float z){
    this->scene.setCameraLookAt(2,0,0);
}
void model_ground_GL::initDisplay(){
    this->scene.affichage();
}

//===============================================
//              Clavier et souris
//===============================================
