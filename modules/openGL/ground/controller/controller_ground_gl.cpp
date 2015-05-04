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
#include "controller_ground_gl.h"

controller_ground_GL::controller_ground_GL(model_ground_GL *model)
{
    this->model=model;
}

controller_ground_GL::~controller_ground_GL()
{

}


void controller_ground_GL::initializeGL()
{
    //glGenTextures(10, texture);
   //lt->add("../data/sang.jpg", "sang");

    //Exemple affichage Texture
    //cube=new Cubique(10,10,10,this->lt->getTexture("sang"),this->lt->getTexture("sang"),this->lt->getTexture("sang"));
    //this->cavite=new Cavite3D(lt->getTexture("peau"));
    this->model->setCameraLocation(2,2,-10);
    this->model->setCameraLookAt(2,0,0);



}


void controller_ground_GL::keyPressEvent(QKeyEvent *keyEvent)
{
    switch(keyEvent->key())
    {
    case Qt::Key_F1:
        this->model->scene.reset();
        this->model->setCameraLocation(0,2,-10);
        this->model->setCameraLookAt(2,0,0);
        break;
    case Qt::Key_A:
        this->zoom();
        break;
    case Qt::Key_Z:
        this->dezoom();
        break;
    }
}
void controller_ground_GL::mouseMoveEvent(QMouseEvent *event){
    this->model->scene.mouseClicked(event);
}
void controller_ground_GL::mousePressEvent(QMouseEvent *event){
    this->model->scene.mouseClicked(event);
}
void controller_ground_GL::mouseReleaseEvent(QMouseEvent *event){
    this->model->scene.mouseClicked(event);
}

void  controller_ground_GL::zoom(){
    this->model->setZoom(this->model->getZoom()+1);
}
void  controller_ground_GL::dezoom(){
    this->model->setZoom(this->model->getZoom()-1);
}
