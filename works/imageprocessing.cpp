#include "imageprocessing.h"

ImageProcessing::ImageProcessing(int width, int height)
{
    //creation de la matrice de base rempli de zero
    this->image=Mat::zeros(height, width, CV_8UC1);
}

ImageProcessing::~ImageProcessing()
{

}
cv::Mat ImageProcessing::getImage() const
{
    return image;
}

void ImageProcessing::setImage(const cv::Mat &value)
{
    image = value;
}


QImage  ImageProcessing::cvMatToQImage(cv::Mat const& src)
{
    cv::Mat temp; // make the same cv::Mat
    cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

cv::Mat ImageProcessing::QImageToCvMat( QImage const& src)
{
    cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
    cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
    cvtColor(tmp, result,CV_BGR2RGB);
    return result;
}

/**  @function Erosion  */
cv::Mat ImageProcessing::erosion( cv::Mat src, int  erosion_size )
{
    //initialize the final matrix after erosion
    cv::Mat erosion_dst;
    // Apply the dilatation
    cv::erode(src, erosion_dst, cv::Mat());
    for(int i=0;i<erosion_size-1;i++)
        cv::erode(erosion_dst, erosion_dst, cv::Mat());
    return erosion_dst;
}

/** @function Dilation */
cv::Mat ImageProcessing::dilation(cv::Mat src, int dilation_size )
{
    //set the final matrix after dilation
    cv::Mat dilation_dst;
    // Apply the dilatation
    cv::dilate(src, dilation_dst, cv::Mat());
    for(int i=0;i<dilation_size-1;i++)
        cv::dilate(dilation_dst, dilation_dst, cv::Mat());
    return dilation_dst;
}
cv::Mat ImageProcessing::opening(cv::Mat src,int opening_size)
{
    //output image
    cv::Mat dst=erosion(src,opening_size);
    dst=dilation(dst,opening_size);
    return dst;

}

cv::Mat ImageProcessing::closing(cv::Mat src,int closing_size)
{
    //output image
    cv::Mat dst=dilation(src,closing_size);
    dst=erosion(dst,closing_size);
    return dst;
}

void ImageProcessing::enregistre()
{
    imwrite( "imageTracks.jpg", this->image );
}

void ImageProcessing::increase(int r, int c)
{
    this->image.at<int>(r,c)=this->image.at<int>(r,c)+1;
}
