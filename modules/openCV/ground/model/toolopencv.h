/*
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 19 Octobre 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale -
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
 */
#ifndef TOOLOPENCV_H
#define TOOLOPENCV_H
#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include "opencv2/core/types_c.h"
#include "QImage"
/**
 * @brief The ToolOpenCV class interface entre openCV et Qt
 */
class ToolOpenCV
{
public:
    ToolOpenCV();
    static QImage Ipl2QImage(const IplImage *newImage);
    static IplImage* QImage2IplImage(const QImage& qImage);

    static  QImage  cvMatToQImage( const cv::Mat &inMat );
    static cv::Mat QImageToCvMat(const QImage inImage);


    // cv::Mat QPixmapToCvMat( const QPixmap &inPixmap, bool inCloneImageData = true );
    //QPixmap cvMatToQPixmap( const cv::Mat &inMat );
};

#endif // TOOLOPENCV_H
