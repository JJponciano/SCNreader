#ifndef VUEPARETAPE_H
#define VUEPARETAPE_H

#include <QObject>
#include <QWidget>
#include "../modules/openGL/ground/view/view_ground_GL.h"
#include <cstdlib>
#include <QMouseEvent>
#include <QList>
#include <QString>
#include <QStringList>
#include "GL/glut.h"
#include "GL/freeglut_std.h"
#include "GL/freeglut.h"
#include "../modules/exceptions/erreur.h"

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

#include "scnreader_model.h"


class VueParEtape: public View_ground_GL
{
public:
    VueParEtape(QWidget *parent = 0);
    ~VueParEtape();

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
     * @param int d, footpulde to begin, int f, footpulse to finish
     */
    void planarSegmentation(int d, int f);


    /**
     * @brief loadCloud Opens a window allowing the user to select the file to load and load the cloud from file.
     * The format of the file is "pcd"
     */
    void loadCloud();
    /**
     * @brief loadCloudFromTXT  Opens a window allowing the user to select the file to load and load the cloud from file.
     * The format of the file is "text"
     */
    void  loadCloudFromTXT();
    /**
     * @brief saveCloudsFromTXT Opens a window allowing the user to select destination to save all clouds in "txt" format file and save it
     */
    void  saveCloudsFromTXT();
    /**
     * @brief saveClouds  Opens a window allowing the user to select destination to save all clouds in "pcd" format file and save it
     */
   void saveClouds();


   void loadFromSCN();

   //accesseur en lecture et en écriture
   int getFtpD();
   int getFtpF();
   int getFtpDI();
   int getFtpFI();

   std::string getNomF();

   int getTaille();
   //void setFtpD(int d);
   //void setFtpF(int f);
   void setFtpDI(int di);
   void setFtpFI(int fi);
   void setaffS(bool b);
   void setaffE(bool b);
   void setaffC(bool b);
private:

  scnreader_model scnreaderFond;
  //int ftpdeDepart;
  //int ftpCourant;
  //QVector<int> pPrec;
  //int pCourant;
  //int pSuiv;
  //int step;
  //bool avance;

  //footpulse de début et de fin du cloud entier
    //int ftpD;
   // int ftpF;
  //footpulse de début et de fin de l'intervalle de travail
    int ftpDI;
    int ftpFI;
  //boolean permettant de gérer l'affichage
    bool affs;
    bool affe;
    bool affc;

    std::string nomFichier;

  int sizeCloud;
};

#endif // VUEPARETAPE_H
