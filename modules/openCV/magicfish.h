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
#ifndef MAGICFISH_H
#define MAGICFISH_H
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include "opencv2/core/types_c.h"
#include "opencv/cxcore.h"
#include "math.h"
#include <QImage>
#include <QColor>
#include <QRgb>
#include <QVector>
#include <QHash>
#include <QSettings>
#include <QFile>
#include <QString>
#include <QList>
#include <sstream>
#include <string>
#include <iostream>
#include <QDirIterator>

class Magicfish
{
public:
    /**
     * @brief Magicfish constructeur
     */
    Magicfish();

    QImage processing(QImage qi);
    //QImage Ipl2QImage(const IplImage *newImage)  ;
    /**
     * @brief Modification
     * @param intensite coefficient d'intensité
     * @param brillance coefficient de brillance
     * @param seuil de tolérance de la teinte/couleur
     * @param i coordonnée du pixel servant de référence
     * @param j coordonnée du pixel servant de référence
     * @return l'image résultante de la modification d'intensité(valeur) et de brillance(saturation)
     */
    QImage Modification(int intensite, int brillance, float seuil, int x, int y, QImage im);
    void initialization(std::string input);
    static void initDonneesSystem ();
    QImage difference(QImage I, QImage I2, int threshold, int closing_size, int last_closing_size, int opening_size);
    QImage getFish(QImage imOr,QImage imF1, QImage imF2,QImage imF3, int threshold,int closing_size,int last_closing_size,int opening_size);
    void video(std::string videoName, std::string pathOutput);
    void whriteVideo (std::string input, std::string output);
    void whriteVideoDefBackground (std::string input, std::string output);
    QImage RecupImage(cv::Mat matrice);

    QImage getBackground(int indiceBackground);
    int  getThreshold();
    int  getclosing_size();
    int  getLast_closing_size();
    int  getOpening_size();
    void setBackground(int indiceBackground,QImage qi);
    void  setThreshold(int t);
    void  setclosing_size(int c);
    void  setLast_closing_size(int c);
    void  setOpening_size(int o);
    // background image to isolate fish
    QImage background1;
    QImage background2;
    QImage background3;
    int threshold;
    int closing_size;
    int last_closing_size;
    int opening_size;
    int intensite;
    int brillance;
    float seuil;
    std::string inputfilename;
    //coordinated pixel to processing
    int x;
    int y;
private:
    QImage  merge(QImage intput,QImage treatment,QImage bw);

    /**
     * @brief onlyWhite keep only white pixels joint to both images
     * @param i1
     * @param i2
     * @return
     */
    QImage onlyWhite(QImage im, QImage i2);
    /**
      * @brief countWhitePixel return number of white pixel into the image
      * @param im this image
      * @return  integer
      */
    int countWhitePixel(QImage im);
    QImage Ipl2QImage(const IplImage *newImage);
    //IplImage* QImage2IplImage(QImage *qimg);
    IplImage *QImage2IplImage(const QImage& qImage);

    QImage  cvMatToQImage( cv::Mat const& src);
    cv::Mat QImageToCvMat(const QImage &src);
    //============JJ===========
    /**
     * @brief erosion Apply the erosion at input image
     * @param src input image
     * @param erosion_size the size of structuring element in 4-connexity
     * @return output image eroded
     */
    cv::Mat erosion(cv::Mat src, int erosion_size );
    /**
      * @brief dilation Apply the dilation at input image
      * @param src input image
      * @param dilation_size the size of structuring element in 4-connexity
      * @return output image eroded
      */
    cv::Mat dilation(cv::Mat src, int dilation_size );
    /**
      * @brief opening Apply the opening at input image ( erosion followed by a dilation)
      * @param src input image
      * @param opening_elem type of struturing element for dilate
      * 0 rectangle
      * 1 cross
      * 2 ellipse
      * @param opening_size the size of structuring element in 4-connexity
      * @return output image dilate
      */
    cv::Mat opening(cv::Mat src, int opening_size);
    /**
      * @brief closing Apply the closing at input image ( dilation followed by a erosion)
      * @param src input image
      * @param closing_size the size of structuring element in 4-connexity
      * @return output image closing
      */
    cv::Mat closing(cv::Mat src, int closing_size);

    //============Claire=======

    /**
     * @brief ModifPixel
     * @param couleur du pixel que l'on souhaite modifier
     * @param Qs coefficient de saturation
     * @param Qv coefficient de valeur
     * @return la nouvelle couleur du pixel après modification de la saturation et de la valeur
     */
    QColor ModifPixel(QRgb couleur, int Qs, int Qv);

    /**
     * @brief CalculEcart
     * @param couleurRef la couleur de référence
     * @param couleurT la couleur testée
     * @return la distance euclidienne entre les deux couleurs
     */
    float CalculEcart(QRgb couleurRef, QRgb couleurT);


};
Q_DECLARE_METATYPE(Magicfish)
    QDataStream & operator << (QDataStream & out, const Magicfish & Valeur);
    QDataStream & operator >> (QDataStream & in, Magicfish & Valeur);
#endif // MAGICFISH_H
