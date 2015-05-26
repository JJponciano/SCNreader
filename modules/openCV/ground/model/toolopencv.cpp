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
#include "toolopencv.h"

ToolOpenCV::ToolOpenCV()
{
}
 IplImage* ToolOpenCV::QImage2IplImage(const QImage& qImage)
  {
    int width = qImage.width();
    int height = qImage.height();

    // Creates a iplImage with 3 channels
    IplImage *img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    char * imgBuffer = img->imageData;

    //Remove alpha channel
    int jump = (qImage.hasAlphaChannel()) ? 4 : 3;

    for (int y=0;y<img->height;y++){
      QByteArray a((const char*)qImage.scanLine(y), qImage.bytesPerLine());
      for (int i=0; i<a.size(); i+=jump){
          //Swap from RGB to BGR
          imgBuffer[2] = a[i];
          imgBuffer[1] = a[i+1];
          imgBuffer[0] = a[i+2];
          imgBuffer+=3;
      }
  }

  return img;
  }
QImage ToolOpenCV::Ipl2QImage(const IplImage *newImage)
{
    QImage qtemp;
    if (newImage && cvGetSize(newImage).width > 0)
    {
        int x;
        int y;
        char* data = newImage->imageData;

        qtemp= QImage(newImage->width, newImage->height,QImage::Format_RGB32 );
        for( y = 0; y < newImage->height; y++, data +=newImage->widthStep )
            for( x = 0; x < newImage->width; x++)
            {
                uint *p = (uint*)qtemp.scanLine (y) + x;
                *p = qRgb(data[x * newImage->nChannels+2],
                          data[x * newImage->nChannels+1],data[x * newImage->nChannels]);
            }
    }
    return qtemp;
}


QImage  ToolOpenCV::cvMatToQImage( const cv::Mat &inMat )
   {
      switch ( inMat.type() )
      {
         // 8-bit, 4 channel
         case CV_8UC4:
         {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB32 );

            return image;
         }

         // 8-bit, 3 channel
         case CV_8UC3:
         {
            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_RGB888 );

            return image.rgbSwapped();
         }

         // 8-bit, 1 channel
         case CV_8UC1:
         {
            static QVector<QRgb>  sColorTable;

            // only create our color table once
            if ( sColorTable.isEmpty() )
            {
               for ( int i = 0; i < 256; ++i )
                  sColorTable.push_back( qRgb( i, i, i ) );
            }

            QImage image( inMat.data, inMat.cols, inMat.rows, inMat.step, QImage::Format_Indexed8 );

            image.setColorTable( sColorTable );

            return image;
         }

            break;
      }

      return QImage();
   }

cv::Mat ToolOpenCV::QImageToCvMat( const QImage inImage){
    IplImage* i1=ToolOpenCV::QImage2IplImage(inImage);
    cv::Mat m(i1);
    delete i1;
    return m;
}
