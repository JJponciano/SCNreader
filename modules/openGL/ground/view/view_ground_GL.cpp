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
#include "view_ground_GL.h"

View_ground_GL::View_ground_GL(QWidget *parent)
    : GLWidget(60, parent, "OpenGL et Qt")
{
   this->model =new model_ground_GL();
     this->controller =new controller_ground_GL(this->model);

}
View_ground_GL::~View_ground_GL(){
    delete this->model;
    delete this->controller;
}

void View_ground_GL::initializeGL()
{
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    this->controller->initializeGL();



}

void View_ground_GL::resizeGL(int width, int height)
{
    this->ratio=(GLfloat)width/(GLfloat)height;
    if(height == 0)
        height = 1;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(this->model->getZoom(), (GLfloat)width/(GLfloat)height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void View_ground_GL::paintGL()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // affichage
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(this->model->getZoom(), this->ratio, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    this->model->initDisplay();
    glPushMatrix();
    //glScalef(this->scale,this->scale,this->scale);

}
void View_ground_GL::setCamera(float pX, float pY, float pZ, float lX, float lY, float lZ){
    this->model->scene.setCameraLocation(pX,pY,pZ);
     this->model->scene.setCameraLookAt(lX,lY,lZ);
}

//===============================================
//              Clavier et souris
//===============================================
void View_ground_GL::keyPressEvent(QKeyEvent *keyEvent)
{
   this->controller->keyPressEvent(keyEvent);
}
void View_ground_GL::mouseMoveEvent(QMouseEvent *event){
    this->controller->mouseMoveEvent(event);
}
void View_ground_GL::mousePressEvent(QMouseEvent *event){
    this->controller->mousePressEvent(event);
}
void View_ground_GL::mouseReleaseEvent(QMouseEvent *event){
    this->controller->mouseReleaseEvent(event);
}
