
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

    //this->ftpD=0;
    //this->ftpF=0;
    this->ftpDI=0;
    this->ftpFI=0;

    this->affs=false;
    this->affe=false;

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
    View_ground_GL::paintGL();

    //------------------------------------------------------------------
    // definition size dot
    glPointSize(1);
    glPushMatrix();
     glBegin(GL_POINTS);
        if(affc)
        {
            int j=this->ftpDI;
            bool stop=false;
            while( !stop && j<=this->ftpFI)
            {
                if(this->scnreaderFond.getNuage().contains(j))
                {

                    QVector <pcl::PointXYZ*>* v=this->scnreaderFond.getNuage().value(j);

                    for(int i=0; i<v->size(); i++)
                    {
                        glPointSize(1);
                                glColor3f(1.0,1.0,1.0);
                                //normalizes points with model->max of cordinates previously finded
                                float x=(* (v->at(i))).x;//scnreaderFond.getMaxX()*10;
                                float y=(* (v->at(i))).y;//scnreaderFond.getMaxY()*10;
                                float z=((* (v->at(i))).z-this->ftpDI)*0.1;//scnreaderFond.getMaxZ()*10;
                                glVertex3f(x,y,z);
                    }

                }
                else
                    stop=true;

                j++;
            }
             glEnd();
    }

    std::stringstream ss;
    ss << "S_" << this->ftpDI <<"_" << this->ftpFI;
    QString chaine=QString::fromStdString (ss.str());
    if(affs)
    {
     /*  if(!this->scnreaderFond.getSegmentation().size()>0)
        {
            this->scnreaderFond.planar_segmentation(this->ftpDI, this->ftpFI);
        }
*/
        QHash<QString, QVector<pcl::PointXYZ*>*> h=this->scnreaderFond.getSegmentation();

       if(!h.contains(chaine))
        {
            try{
                this->scnreaderFond.planar_segmentation(this->ftpDI, this->ftpFI);
            }catch(std::exception const& e){
                QMessageBox::critical(0, "Error", e.what());
            }
        }
         QVector <pcl::PointXYZ*>* vect=this->scnreaderFond.getSegmentation().value(QString::fromStdString (ss.str()));
        glBegin(GL_POINTS);
            for(int j=0;j<vect->size();j++)
            {
                glColor3f(0.0,0.0,1.0);
                glVertex3f((* (vect->at(j))).x,(*(vect->at(j))).y,((* (vect->at(j))).z-this->ftpDI)*0.1);
            }
        glEnd();

    }
    if(affe)
    {

    }
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
            this->scnreaderFond.loadCloudFromTXT2(fileName.toStdString());
            this->nomFichier=fileName.toStdString();
            if(this->scnreaderFond.getNuage().size() > 0)
            {
                this->ftpDI=this->scnreaderFond.getFtpd();
                this->ftpFI=this->scnreaderFond.getFtpd();
            }
            else throw Erreur("Le fichier ne contient pas de point");
            //this->ftpdeDepart=this->scnreaderFond.getClouds(0)->points[0].z;
            //this->ftpCourant=this->scnreaderFond.getClouds(0)->points[0].z;
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
            this->nomFichier=fileName.toStdString();

            if(this->scnreaderFond.getNuage().size() > 0)
            {
                this->ftpDI=this->scnreaderFond.getFtpd();
                this->ftpFI=this->scnreaderFond.getFtpd();
            }
            else throw Erreur("Le fichier ne contient pas de point");
           // this->ftpdeDepart=this->scnreaderFond.getClouds(0)->points[0].z;
            //this->ftpCourant=this->scnreaderFond.getClouds(0)->points[0].z;
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
            std::cout<<"end"<<std::endl;
            this->nomFichier=fileName.toStdString();
            if(this->scnreaderFond.getNuage().size() > 0)
            {
                this->ftpDI=this->scnreaderFond.getFtpd();
                this->ftpFI=this->scnreaderFond.getFtpd();
            }
            else throw Erreur("Le fichier ne contient pas de point");
        }
    }catch(std::exception const& e){
        QMessageBox::critical(this, "Error", e.what());
    }
}

void VueParEtape::clear(){
    this->scnreaderFond.clear();
}
void VueParEtape::planarSegmentation(int d, int f){
    try{
        this->scnreaderFond.planar_segmentation( d, f);
    }catch(std::exception const& e){
        QMessageBox::critical(this, "Error", e.what());
    }
}

void VueParEtape::extractionCloud(int i){
     this->scnreaderFond.extractionCloud(i,i);
}

//===============================================
//              Clavier et souris
//===============================================
void VueParEtape::keyPressEvent(QKeyEvent *keyEvent)
{

    if(keyEvent->key()==Qt::Key_W){
        this->scnreaderFond.setMaxX(this->scnreaderFond.getMaxX()*1.2);
        this->scnreaderFond.setMaxY(this->scnreaderFond.getMaxY()*1.2);
        this->scnreaderFond.setMaxZ(this->scnreaderFond.getMaxZ()*1.2);
    }
    else if(keyEvent->key()==Qt::Key_Q){
        this->scnreaderFond.setMaxX(this->scnreaderFond.getMaxX()*0.8);
        this->scnreaderFond.setMaxY(this->scnreaderFond.getMaxY()*0.8);
        this->scnreaderFond.setMaxZ(this->scnreaderFond.getMaxZ()*0.8);
    }
    /*else if(keyEvent->key()==Qt::Key_J){
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
    }*/
    else View_ground_GL::keyPressEvent(keyEvent);
}
/*void VueParEtape::setFtpD(){
    this->ftpD=this->scnreaderFond.getFtpd();
}

void VueParEtape::setFtpF(){
    this->ftpF=this->scnreaderFond.getFtpf();
}*/

int VueParEtape::getFtpD(){
    return this->scnreaderFond.getFtpd();
}

int VueParEtape::getFtpF(){
    return this->scnreaderFond.getFtpf();
}

void VueParEtape::setFtpDI(int di){
    this->ftpDI=di;
}

void VueParEtape::setFtpFI(int fi){
    this->ftpFI=fi;
}

int VueParEtape::getFtpDI(){
    return this->ftpDI;
}

int VueParEtape::getFtpFI(){
    return this->ftpFI;
}

std::string VueParEtape::getNomF(){
    return this->nomFichier;
}

void VueParEtape::setaffC(bool b){
    this->affc=b;
}

void VueParEtape::setaffS(bool b){
    this->affs=b;
}

void VueParEtape::setaffE(bool b){
    this->affe=b;
}

int VueParEtape::getTaille(){
    return this->scnreaderFond.getNuage().size();
}

void VueParEtape::mouseMoveEvent(QMouseEvent *event){
    View_ground_GL::mouseMoveEvent(event);
}
void VueParEtape::mousePressEvent(QMouseEvent *event){
    View_ground_GL::mousePressEvent(event);
}
void VueParEtape::mouseReleaseEvent(QMouseEvent *event){
    View_ground_GL::mouseReleaseEvent(event);
}
