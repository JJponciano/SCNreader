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
