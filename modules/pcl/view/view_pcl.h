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
#ifndef VIEW_PCL_H
#define VIEW_PCL_H

#include "../../openGL/ground/groundglwidget.h"
#include "../../exceptions/erreur.h"
#include "../ground/ToolsPCL.h"



class View_pcl : public groundGLWidget
{
    Q_OBJECT
public:
    View_pcl(QWidget* parent = 0);
    ~View_pcl();
    void resizeGL(int ratio, int height);
    void initializeGL();
    void paintGL();
    virtual void keyPressEvent( QKeyEvent *keyEvent );
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);


    /**
     * @brief clear remove all clouds
     */
    void clear();
    /**
     * @brief extractionCloud create new clouds representing different parts of the cloud located at the ith position in the clouds vector
     * @param i index of the cloud position in this Qvector clouds
     */
    void extractionCloud(int i);
    /**
     * @brief planarSegmentation  create a new cloud representing the most larger plan of the cloud located at the ith position in the clouds vector
     * @param i index of the cloud position in this Qvector clouds
     */
    void planarSegmentation(int i);


    /**
     * @brief loadCloud Opens a window allowing the user to select the file to load and load the cloud from file.
     * The format of the file is "pcd"
     */
    void loadCloud();
    /**
     * @brief loadCloudFromTXT  Opens a window allowing the user to select the file to load and load the cloud from file.
     * The format of the file is "text"
     */
    void loadCloudFromTXT();
    /**
     * @brief saveCloudsFromTXT Opens a window allowing the user to select destination to save all clouds in "txt" format file and save it
     */
    void saveCloudsFromTXT();
    /**
     * @brief saveClouds  Opens a window allowing the user to select destination to save all clouds in "pcd" format file and save it
     */
    void saveClouds();
protected:
    ToolsPCL toolspcl;
    int sizeCloud;
};

#endif // VIEW_PCL_H
