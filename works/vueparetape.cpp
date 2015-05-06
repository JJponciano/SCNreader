/**
 * @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
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
#include "vueparetape.h"

VueParEtape::VueParEtape(QWidget *parent): groundGLWidget(parent)
{
    this->step=500;
    this->pCourant=0;
    this->pSuiv=0;

    this->ftpCourant=0;
    this->ftpdeDepart=0;


}

VueParEtape::~VueParEtape()
{

}


// ------------------------------------------ OPENGL Functions ------------------------------------------
void VueParEtape::initializeGL()
{
    groundGLWidget::initializeGL();
    glDisable(GL_LIGHTING);
}

void VueParEtape::resizeGL(int width, int height)
{
    groundGLWidget::resizeGL( width,  height);
}

void VueParEtape::paintGL()
{
    //call the superclass function
    groundGLWidget::paintGL();
    this->setCamera(pX,pY,pZ,lX,lY,lZ);
    //------------------------------------------------------------------
    // definition size dot
    glPointSize(1);
    //draw clouds step by step
    glPushMatrix();
    //glRotatef(-90,0,0,1);
    glBegin(GL_POINTS);


    for(int j=0;j<this->scnreaderFond.getClouds().size();j++)
    {
         for (int i=0;i<this->scnreaderFond.getClouds(j)->points.size();i++){
                glColor3f(1.0/(float)(j+1),1.0/(float)(this->scnreaderFond.getClouds().size()-j),1.0);
                //normalizes points with model->max of cordinates previously finded
                float x=this->scnreaderFond.getClouds(j)->points[i].x;//scnreaderFond.getMaxX()*10;
                float y=this->scnreaderFond.getClouds(j)->points[i].y;//scnreaderFond.getMaxY()*10;
                float z=(this->scnreaderFond.getClouds(j)->points[i].z-this->ftpdeDepart)*0.01;//scnreaderFond.getMaxZ()*10;
                glVertex3f(x,y,z);
            }
    }
    glEnd();
    glPopMatrix();
    //----------------------------------------------------------------------*/
}

// ------------------------------------------ Action Functions ------------------------------------------

void VueParEtape::loadCloudFromTXT(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(NULL,"Load", "","Load a cloud (*.txt);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            this->scnreaderFond.addCloudFromTXT(fileName.toStdString());

        }

    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }

}
void VueParEtape::loadCloud(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(0,"Load", "","Load a cloud (*.pcd);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            this->scnreaderFond.addCloud(fileName.toStdString());
            this->ftpdeDepart=this->scnreaderFond.getClouds(0)->points[0].z;
            this->ftpCourant=this->scnreaderFond.getClouds(0)->points[0].z;
        }

    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void VueParEtape::saveClouds(){
    try{
        //Opens a window allowing the user to select the file to save
        QString fileName = QFileDialog::getSaveFileName(0,"Save", "","save all clouds (*.pcd);;All Files (*)");

        // si l'utilisateur a sélectionné un nom
        if (!fileName.isEmpty())
        {
            this->scnreaderFond.saveClouds(fileName.toStdString());
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void VueParEtape::saveCloudsFromTXT(){
    try{
        //Opens a window allowing the user to select the file to save
        QString fileName = QFileDialog::getSaveFileName(0,"Save", "","save all clouds (*.txt);;All Files (*)");

        // si l'utilisateur a sélectionné un nom
        if (!fileName.isEmpty())
        {
            this->scnreaderFond.saveCloudsFromTXT(fileName.toStdString());
            this->ftpdeDepart=this->scnreaderFond.getClouds(0)->points[0].z;
            this->ftpCourant=this->scnreaderFond.getClouds(0)->points[0].z;
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}

void VueParEtape::loadFromSCN(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(0,"Read", "","Read file (*.scn);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            std::cout<<"start"<<std::endl;
            this->scnreaderFond.loadFromSCN(fileName.toStdString());
this->ftpdeDepart=this->scnreaderFond.getClouds(0)->points[0].z;
            std::cout<<"end"<<std::endl;
        }
    }catch(std::exception const& e){
        QMessageBox::critical(this, "Error", e.what());
    }
}

void VueParEtape::clear(){
    this->scnreaderFond.clear();
}
void VueParEtape::planarSegmentation(int i){
    this->scnreaderFond.planarSegmentation(i);
}

void VueParEtape::extractionCloud(int i){
    this->scnreaderFond.planarSegmentation(i);
}

//===============================================
//              Clavier et souris
//===============================================
void VueParEtape::keyPressEvent(QKeyEvent *keyEvent)
{

    if(keyEvent->key()==Qt::Key_W){
        this->pZ++;
        this->lZ++;
    }
    else if(keyEvent->key()==Qt::Key_S){
        this->pZ--;
        this->lZ--;
    }
    else if(keyEvent->key()==Qt::Key_D){
        this->pX--;
        this->lX--;
    }
    else if(keyEvent->key()==Qt::Key_A){
        this->pX++;
        this->lX++;
    }
    else if(keyEvent->key()==Qt::Key_Q){
        this->pY++;
        this->lY++;
    }
    else if(keyEvent->key()==Qt::Key_E){
        this->pY--;
        this->lY--;
    }
    else if(keyEvent->key()==Qt::Key_8){
        this->lY++;
    }
    else if(keyEvent->key()==Qt::Key_5){
        this->lY--;
    }
    else if(keyEvent->key()==Qt::Key_J){
        if( this->scnreaderFond.getClouds().size()>0)
            if( this->pSuiv < (this->scnreaderFond.getClouds(0)->points.size()))
            {
                this->ftpCourant+=step;
                this->pPrec.push_back(this->pCourant);
                this->pCourant=this->pSuiv;
                std::cout<<this->pCourant<<"/"<<this->scnreaderFond.getClouds(0)->points.size()<<std::endl;
            }
    }
    else if(keyEvent->key()==Qt::Key_L){
        if( this->scnreaderFond.getClouds().size()>0)
            if(this->pPrec.size()>0)
            {
                this->ftpCourant-=step;
                this->pCourant=this->pPrec.at(this->pPrec.size()-1);
                this->pPrec.pop_back();
                std::cout<<this->pCourant<<"/"<<this->scnreaderFond.getClouds(0)->points.size()<<std::endl;
            }
    }
    else if(keyEvent->key()==Qt::Key_D){
        if( this->scnreaderFond.getClouds().size()>0)
        {
            this->ftpCourant=this->ftpdeDepart;
            this->pCourant=0;
            this->pPrec.clear();
        }
    }
    else groundGLWidget::keyPressEvent(keyEvent);
}
void VueParEtape::mouseMoveEvent(QMouseEvent *event){
    groundGLWidget::mouseMoveEvent(event);
}
void VueParEtape::mousePressEvent(QMouseEvent *event){
    groundGLWidget::mousePressEvent(event);
}
void VueParEtape::mouseReleaseEvent(QMouseEvent *event){
    groundGLWidget::mouseReleaseEvent(event);
}
