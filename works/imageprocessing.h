#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <math.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.h"
#include "opencv2/core/types_c.h"
#include "opencv/cxcore.h"
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

class ImageProcessing
{
public:
    ImageProcessing(int width, int height);
    ~ImageProcessing();
    /**
     * @brief increase increase of one level a pixel
     * @param r row of pixel
     * @param c col of pixel
     */
    void increase(int r, int c);


    //--------Access in reading and writing for private variables-------
    cv::Mat getImage() const;
    void setImage(const cv::Mat &value);
    //------------------------------------------------------------------

private:
    //-------------------Private Attributes--------------
    cv::Mat image;


    //-------------------Private Functions--------------
    /**
     * @brief cvMatToQImage convert a Mat in QImage
     * @param src is the Mat which will be converted
     * @return QImage corresponding
     */
    QImage  cvMatToQImage( cv::Mat const& src);

    /**
     * @brief QImageToCvMat convert a QImage in Mat
     * @param src is the QImage which will be converted
     * @return Mat corresponding
     */
    cv::Mat QImageToCvMat(const QImage &src);

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

    /**
     * @brief enregistre enregistre la matrice dans un fichier image type jpg
     */
    void enregistre();
};

#endif // IMAGEPROCESSING_H
