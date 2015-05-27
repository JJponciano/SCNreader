#include "imageprocessing.h"

ImageProcessing::ImageProcessing(int w, int h)
{
    //creation de la matrice de base rempli de zero
    this->width=w;
    this->height=h;
    this->image=cv::Mat::zeros(h,w,CV_32S);
}

ImageProcessing::ImageProcessing()
{
    this->width=0;
    this->height=0;
}
ImageProcessing::~ImageProcessing()
{

}

cv::Mat ImageProcessing::getImage() const
{
    return image;
}

void ImageProcessing::setImage(cv::Mat &value)
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

void ImageProcessing::enregistre(QString nom)
{
    QString titre=nom;
    titre.push_back("Transfo.jpg");
    imwrite( titre.toStdString(), this->image );
}
void ImageProcessing::increase(int r, int c)
{
    int val=this->image.at<int>(r,c);
    std::cout<<"val: "<<val<< std::endl;
    this->image.at<int>(r,c)=val+1;
}

void ImageProcessing::calibration()
{
    //we keep min and max values
    int *val=MinMax();
    int min=val[0];
    int max=val[1];
    std::cout<<"min: "<< min <<std::endl;
    std::cout<<"max: "<< max <<std::endl;
    //we apply stretching
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            int c=this->image.at<int>(i,j)-min;
            c=c*255;
            if(max-min==0)
            {
                this->image.at<int>(i,j)= 0;
            }
            else
            {
                c=c/(max-min);
                this->image.at<int>(i,j)= c;
            }
        }
    }
}

void ImageProcessing::thresholding(int s)
{
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(this->image.at<int>(i,j)<=125)
                this->image.at<int>(i,j)=0;
            else
                this->image.at<int>(i,j)=255;
        }
    }
}

int ImageProcessing::getValue(int i, int j)
{
    return this->image.at<int>(i,j);
}

int* ImageProcessing::MinMax()
{
    int * val=new int[2];
    int min=this->image.at<int>(0,0);
    int max=this->image.at<int>(0,0);

    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            int nb=this->image.at<int>(i,j);
            std::cout<<"nb: "<<nb<< std::endl;
            if(nb>max)
                max=nb;
            if(nb< min)
                min=nb;
        }
    }
    val[0]=min;
    val[1]=max;
    return val;
}

//VERSION INT [][]
//ImageProcessing::ImageProcessing(const int w, const int h)
//{
//    //creation de la matrice de base rempli de zero
//    this->width=w;
//    this->height=h;
//    int temp[h][w];
//    this->image=temp;
//}

//ImageProcessing::ImageProcessing()
//{
//    this->width=0;
//    this->height=0;
//}
//ImageProcessing::~ImageProcessing()
//{

//}
//int ** ImageProcessing::getImage() const
//{
//    return image;
//}

//void ImageProcessing::setImage(int ** value)
//{
//    image = value;
//}

//void ImageProcessing::increase(int r, int c)
//{
//    this->image[r][c]=this->image[r][c]+1;
//}

//void ImageProcessing::calibration()
//{
//    //we keep min and max values
//    int *val=MinMax();
//    int min=val[0];
//    int max=val[1];
//    //we apply stretching
//    for(int i=0; i<this->height; i++)
//    {
//        for(int j=0; j<this->width; j++)
//        {
//            this->image[i][j]= ((this->image[i][j]-min)*255)/(max-min);
//        }
//    }
//}

//void ImageProcessing::thresholding(int s)
//{
//    for(int i=0; i<this->height; i++)
//    {
//        for(int j=0; j<this->width; j++)
//        {
//            if(this->image[i][j]<=125)
//                this->image[i][j]=0;
//            else
//                this->image[i][j]=1;
//        }
//    }
//}

//int ImageProcessing::getValue(int i, int j)
//{
//    return this->image[i][j];
//}

//int* ImageProcessing::MinMax()
//{
//    int val[2];
//    int min=this->image[0][0];
//    int max=this->image[0][0];

//    for(int i=0; i<this->height; i++)
//    {
//        for(int j=0; j<this->width; j++)
//        {
//            if(this->image[i][j]>max)
//                max=this->image[i][j];
//            if(this->image[i][j]<min)
//                min=this->image[i][j];
//        }
//    }
//    val[0]=min;
//    val[1]=max;
//    return val;
//}
