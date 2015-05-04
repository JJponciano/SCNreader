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
