<<<<<<< HEAD
#include "scnreader_view.h"

scnreader_view::scnreader_view(QWidget *parent): View_pcl(parent)
{
}

scnreader_view::~scnreader_view()
{

}

void scnreader_view::paintGL()
{
    //call the superclass function
    View_ground_GL::paintGL();

    //------------------------------------------------------------------
    // definition size dot
    glPointSize(1);
    //draw clouds
    glPushMatrix();
    glBegin(GL_LINE);
    for(int j=0;j<this->scnreaderFond.getClouds().size();j++)
        for (int i=0;i<this->scnreaderFond.getClouds(j)->points.size();i++){
            glColor3f(1.0/(float)(j+1),1.0/(float)(this->scnreaderFond.getClouds().size()-j),1.0);
            //normalizes points with model->max of cordinates previously finded
            glVertex3f(this->scnreaderFond.getClouds(j)->points[i].x/scnreaderFond.getMaxX(),
                       this->scnreaderFond.getClouds(j)->points[i].y/scnreaderFond.getMaxY(),
                       this->scnreaderFond.getClouds(j)->points[i].z/scnreaderFond.getMaxZ());
        }
    glEnd();
    glPopMatrix();
}

void scnreader_view::loadCloudFromTXT(){
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

void scnreader_view::loadFromSCN(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(0,"Read", "","Read file (*.scn);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            std::cout<<"start"<<std::endl;
            this->scnreaderFond.loadFromSCN(fileName.toStdString());
            std::cout<<"end"<<std::endl;
        }
    }catch(std::exception const& e){
        QMessageBox::critical(this, "Error", e.what());
    }
}
=======
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
#include "scnreader_view.h"

scnreader_view::scnreader_view(QWidget *parent): View_pcl(parent)
{
}

scnreader_view::~scnreader_view()
{

}

void scnreader_view::paintGL()
{
    //call the superclass function
    View_ground_GL::paintGL();

    //------------------------------------------------------------------
    // definition size dot
    glPointSize(1);
    //draw clouds
    glPushMatrix();
    glBegin(GL_POINTS);
    for(int j=0;j<this->scnreaderFond.getClouds().size();j++)
        for (int i=0;i<this->scnreaderFond.getClouds(j)->points.size();i++){

            glColor3f(1.0/(float)(j+1),1.0/(float)(this->scnreaderFond.getClouds().size()-j),1.0);
            //normalizes points with model->max of cordinates previously finded
            glVertex3f(this->scnreaderFond.getClouds(j)->points[i].x/scnreaderFond.getMaxX(),
                       this->scnreaderFond.getClouds(j)->points[i].y/scnreaderFond.getMaxY(),
                       this->scnreaderFond.getClouds(j)->points[i].z/scnreaderFond.getMaxZ());

        }
    glEnd();
    glPopMatrix();
}
void scnreader_view::loadFromSCN(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(0,"Read", "","Read file (*.scn);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            std::cout<<"start"<<std::endl;
            this->scnreaderFond.loadFromSCN(fileName.toStdString());
            std::cout<<"end"<<std::endl;
        }
    }catch(std::exception const& e){
        QMessageBox::critical(this, "Error", e.what());
    }
}
>>>>>>> master
