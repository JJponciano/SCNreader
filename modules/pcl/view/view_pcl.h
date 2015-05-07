#ifndef VIEW_PCL_H
#define VIEW_PCL_H

#include <QObject>
#include <QWidget>
#include "../../openGL/ground/view/view_ground_GL.h"
#include <cstdlib>
#include <QMouseEvent>
#include <QList>
#include <QString>
#include <QStringList>
#include "GL/glut.h"
#include "GL/freeglut_std.h"
#include "GL/freeglut.h"
#include "../../exceptions/erreur.h"
#include <QVector>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "../ground/ToolsPCL.h"



class View_pcl : public View_ground_GL
{
    Q_OBJECT
public:
    View_pcl(QWidget* parent = 0);
    ~View_pcl();
    void resizeGL(int ratio, int height);
    void initializeGL();
    virtual void paintGL();
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
  virtual  void  loadCloudFromTXT();
    /**
     * @brief saveCloudsFromTXT Opens a window allowing the user to select destination to save all clouds in "txt" format file and save it
     */
   virtual void  saveCloudsFromTXT();
    /**
     * @brief saveClouds  Opens a window allowing the user to select destination to save all clouds in "pcd" format file and save it
     */
 virtual   void saveClouds();
protected:
    ToolsPCL toolspcl;
    int sizeCloud;
};

#endif // VIEW_PCL_H
