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
