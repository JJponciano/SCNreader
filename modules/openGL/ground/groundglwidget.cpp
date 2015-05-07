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
#include "groundglwidget.h"

groundGLWidget::groundGLWidget(QWidget *parent)
    : View_ground_GL(parent)
{
    this->pX=0;
   this->pY=0;
   this->pZ=-10;
   this->lX=0;
   this->lY=0;
   this->lZ=0;
}
groundGLWidget::~groundGLWidget()
{
}

// ------------------------------------------ OPENGL Functions ------------------------------------------
void groundGLWidget::initializeGL()
{
    View_ground_GL::initializeGL();
     this->setCamera(pX,pY,pZ,lX,lY,lZ);
     glDisable(GL_LIGHTING);
}

void groundGLWidget::resizeGL(int width, int height)
{
    View_ground_GL::resizeGL( width,  height);
}

void groundGLWidget::paintGL()
{
    //call the superclass function
    View_ground_GL::paintGL();
}


//===============================================
//             Keyboard and mouse events
//===============================================
void groundGLWidget::keyPressEvent(QKeyEvent *keyEvent)
{
View_ground_GL::keyPressEvent(keyEvent);
}
void groundGLWidget::mouseMoveEvent(QMouseEvent *event){
    View_ground_GL::mouseMoveEvent(event);
}
void groundGLWidget::mousePressEvent(QMouseEvent *event){
    View_ground_GL::mousePressEvent(event);
}
void groundGLWidget::mouseReleaseEvent(QMouseEvent *event){
    View_ground_GL::mouseReleaseEvent(event);
}


