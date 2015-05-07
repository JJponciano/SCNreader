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
