
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
    this->px=0;
    this->py=0;
    this->pz=0;
    this->affs=false;
    this->affe=false;
    this->affr=false;
    this->affc=true;
    this->mirx=0;
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

    //------------------------------------------------------------------
    // definition size dot
//    glPointSize(4);
//    glPushMatrix();
//        glBegin(GL_POINTS);
//        glColor3f(1.0,1.0,1.0);
//           for(int i=-100; i<100; i++)
//          glVertex3f(mirx,i*0.01, 0);
//        glEnd();
//    glPopMatrix();

    glPointSize(1);
    glPushMatrix();

    if(affc)
    {
        this->affichageCloud();
    }
    if(affswitch)
    {
        this->affichageSwitch();
    }
    if(affs)
    { 
        this->affichageSegm();
    }
    if(affe)
    {
        QVector <pcl::PointXYZ *> rails=this->scnreaderFond.getLesRails().getCloud();
        //std::cout<<(rails.size())<<std::endl;

        glBegin(GL_POINTS);
        for(int i=0; i<rails.size(); i++)
        {
            glColor3f(0.0,1.0,1.0);
            glVertex3f(rails.at(i)->x, rails.at(i)->y, (rails.at(i)->z-this->ftpDI)*0.1);
        }
        glEnd();
    }

    if(affr)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr resultRANSAC=this->scnreaderFond.getResultRANSAC();
        QVector <int> switchs= scnreaderFond.getLesRailsOptimize().getSwitchDetected();
        glBegin(GL_POINTS);
        for(int i=0; i<resultRANSAC->points.size(); i++)
        {
            int z=resultRANSAC->points.at(i).z;
            if(switchs.contains(z))
                glColor3f(1.0,1.0,0.0);
            else
                glColor3f(1.0,0.0,1.0);
            glVertex3f(resultRANSAC->points.at(i).x, resultRANSAC->points.at(i).y, (z-this->ftpDI)*0.1);

        }
        glEnd();
    }
    glPopMatrix();
    //----------------------------------------------------------------------*/
}

// ------------------------------------------ Action Functions ------------------------------------------
void VueParEtape::affichageSwitch()
{
    //TODO
}

void VueParEtape::affichageCloud()
{
    glBegin(GL_POINTS);
    int j=this->ftpDI;
    bool stop=false;
    QVector <int> switchs= scnreaderFond.getLesSwitchs();
    while( !stop && j<=this->ftpFI)
    {
        if(this->scnreaderFond.getNuage().contains(j))
        {

            QVector <pcl::PointXYZ*>* v=this->scnreaderFond.getNuage().value(j);
            for(int i=0; i<v->size(); i++)
            {
                glPointSize(1);

                //normalizes points with model->max of cordinates previously finded
                float x=(* (v->at(i))).x;//scnreaderFond.getMaxX()*10;
                float y=(* (v->at(i))).y;//scnreaderFond.getMaxY()*10;
                float z=((* (v->at(i))).z-this->ftpDI)*0.1;//scnreaderFond.getMaxZ()*10;
                //                                glVertex3f(x,int(y*1000)/100,z);
                if(switchs.contains((* (v->at(i))).z))
                    glColor3f(1.0,0.0,0.0);
                else
                    glColor3f(1.0,1.0,1.0);
                glVertex3f(x,y,z);
            }

        }
        else
            stop=true;

        j++;
    }
    glEnd();
}
void VueParEtape::affichageSegm()
{
    std::stringstream ss;
    ss << "S_" << this->ftpDI <<"_" << this->ftpFI;
    QString chaine=QString::fromStdString (ss.str());

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
bool VueParEtape::getAffswitch() const
{
    return affswitch;
}

void VueParEtape::setAffswitch(bool value)
{
    affswitch = value;
}


void VueParEtape::loadCloudFromTXT(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(NULL,"Load", "","Load a cloud (*.txt);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {

            this->nomFichier=fileName.toStdString();
            this->scnreaderFond.setNomFile(this->KeepName(fileName));

            this->scnreaderFond.loadCloudFromTXT2(fileName.toStdString());

            if(this->scnreaderFond.getNuage().size() > 0)
            {
                this->ftpDI=this->scnreaderFond.getFtpd();
                this->ftpFI=this->scnreaderFond.getFtpd();
                QVector <pcl::PointXYZ *> rails=this->scnreaderFond.getLesRails().getCloud();
              /*  for(int i=0;i<rails.size();i++){
                    if( rails[i]->x<0)
                              rails[i]->y+=1;
                }*/
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

    if(keyEvent->key()==Qt::Key_M){
        this->mirx-=0.01;
        std::cout<<this->mirx<<std::endl;
    }
    else
    if(keyEvent->key()==Qt::Key_N){
        this->mirx+=0.01;
        std::cout<<this->mirx<<std::endl;
    }
    else
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
    else groundGLWidget::keyPressEvent(keyEvent);
}

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

void VueParEtape::setaffR(bool b){
    this->affr=b;
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
QString VueParEtape::KeepName(QString fileName)
{
    QStringList result =fileName.split("/");
    QString n=result.at(result.size()-1);
    result =n.split(".");
    n="";
    for(int i=0; i<result.size()-1; i++)
        n.push_back(result.at(i));
    return n;
}
