
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

#ifndef VUEPARETAPE_H
#define VUEPARETAPE_H

#include <QObject>
#include <QWidget>
#include "../modules/openGL/ground/groundglwidget.h"
#include "scnreader_model.h"
#include <cstdlib>
#include <QMouseEvent>
#include <QList>
#include <QString>
#include <QStringList>

/**
 * @class VueParEtape
 * @brief VueParEtape is a class which manages the display of the big cloud of points.
 * This class allows to see the cloud part by part, where you choose what footpulse and how many footpulse you want to see.
 * When you watch the tracks, you can see footpulses which corresponding to a switch in red.
 * Or you can choose to watch switch by switch (switchs which have been detected).
 *
 * @details
 *
 * \subsection{How to use}
 *
 *
*/


class VueParEtape: public groundGLWidget
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
   void setaffR(bool b);

   bool getAffswitch() const;
   void setAffswitch(bool value);

   int getNumS();
   void setNumS(int value);

   int getPosSwitch() const;
   void IncreasePosSwitch();
   void DecreasePosSwitch();
   void calculNumWithPos();

private:

   void affichageCloud();
   void affichageSegm();
   void affichageSwitch();
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
    bool affswitch;
    bool affe;
    bool affc;
    bool affr;
    float px;
    float py;
    float pz;
    bool firstP;
    std::string nomFichier;
    float mirx;
    int sizeCloud;
    QString KeepName(QString fileName);

    //numero of switch
    int numS;
    //position of switch
    int posSwitch;
    //all switch which are detected
    QVector < QVector <int> > SwitchDetected;
    int sizeAllSwitch();
    void LectureSw(QString nameF);
    bool AucunSwitch();
    bool SwitchContenu(int ftp);
};

#endif // VUEPARETAPE_H
