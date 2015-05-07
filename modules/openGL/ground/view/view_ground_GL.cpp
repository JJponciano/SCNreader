/*
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 19 Octobre 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale -
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
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
