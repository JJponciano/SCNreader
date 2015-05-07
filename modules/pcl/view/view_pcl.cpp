#include "view_pcl.h"

View_pcl::View_pcl(QWidget *parent)
    : View_ground_GL(parent)
{
}

// ------------------------------------------ OPENGL Functions ------------------------------------------
void View_pcl::initializeGL()
{
    View_ground_GL::initializeGL();
     glDisable(GL_LIGHTING);
}

void View_pcl::resizeGL(int width, int height)
{
    View_ground_GL::resizeGL( width,  height);
}

void View_pcl::paintGL()
{
    //call the superclass function
    View_ground_GL::paintGL();

    //------------------------------------------------------------------
    // definition size dot
    glPointSize(1);
    //draw clouds
    glPushMatrix();
    glBegin(GL_POINTS);
    for(int j=0;j<this->toolspcl.getClouds().size();j++)
        for (int i=0;i<this->toolspcl.getClouds(j)->points.size();i++){
            glColor3f(1.0/(float)(j+1),1.0/(float)(this->toolspcl.getClouds().size()-j),1.0);
            //normalizes points with model->max of cordinates previously finded
            glVertex3f(this->toolspcl.getClouds(j)->points[i].x/toolspcl.getMaxX(),
                       this->toolspcl.getClouds(j)->points[i].y/toolspcl.getMaxY(),
                       this->toolspcl.getClouds(j)->points[i].z/toolspcl.getMaxZ());
        }
    glEnd();
    glPopMatrix();
    //----------------------------------------------------------------------*/
}

// ------------------------------------------ Action Functions ------------------------------------------

void View_pcl::loadCloudFromTXT(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(NULL,"Load", "","Load a cloud (*.txt);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            this->toolspcl.addCloudFromTXT(fileName.toStdString());
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void View_pcl::loadCloud(){
    try{
        //Opens a window allowing the user to select the file to load
        QString fileName = QFileDialog::getOpenFileName(0,"Load", "","Load a cloud (*.pcd);;All Files (*)");

        // if user have seleted a directory
        if (!fileName.isEmpty())
        {
            this->toolspcl.addCloud(fileName.toStdString());
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void View_pcl::saveClouds(){
    try{
        //Opens a window allowing the user to select the file to save
        QString fileName = QFileDialog::getSaveFileName(0,"Save", "","save all clouds (*.pcd);;All Files (*)");

        // si l'utilisateur a sélectionné un nom
        if (!fileName.isEmpty())
        {
            this->toolspcl.saveClouds(fileName.toStdString());
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void View_pcl::saveCloudsFromTXT(){
    try{
        //Opens a window allowing the user to select the file to save
        QString fileName = QFileDialog::getSaveFileName(0,"Save", "","save all clouds (*.txt);;All Files (*)");

        // si l'utilisateur a sélectionné un nom
        if (!fileName.isEmpty())
        {
            this->toolspcl.saveCloudsFromTXT(fileName.toStdString());
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void View_pcl::clear(){
    this->toolspcl.clear();
}
void View_pcl::planarSegmentation(int i){
     this->toolspcl.planarSegmentation(i);
}

void View_pcl::extractionCloud(int i){
     this->toolspcl.planarSegmentation(i);
}

//===============================================
//              Clavier et souris
//===============================================
void View_pcl::keyPressEvent(QKeyEvent *keyEvent)
{

    if(keyEvent->key()==Qt::Key_W){
        this->toolspcl.setMaxX(this->toolspcl.getMaxX()*1.2);
        this->toolspcl.setMaxY(this->toolspcl.getMaxY()*1.2);
        this->toolspcl.setMaxZ(this->toolspcl.getMaxZ()*1.2);
    }
    else if(keyEvent->key()==Qt::Key_Q){
        this->toolspcl.setMaxX(this->toolspcl.getMaxX()*0.8);
        this->toolspcl.setMaxY(this->toolspcl.getMaxY()*0.8);
        this->toolspcl.setMaxZ(this->toolspcl.getMaxZ()*0.8);
    }
    else View_ground_GL::keyPressEvent(keyEvent);
}
void View_pcl::mouseMoveEvent(QMouseEvent *event){
    View_ground_GL::mouseMoveEvent(event);
}
void View_pcl::mousePressEvent(QMouseEvent *event){
    View_ground_GL::mousePressEvent(event);
}
void View_pcl::mouseReleaseEvent(QMouseEvent *event){
    View_ground_GL::mouseReleaseEvent(event);
}



View_pcl::~View_pcl()
{
}

