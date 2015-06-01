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

void ImageProcessing::fermeture()
{
    cv::Mat imf=cv::Mat::zeros(this->height,this->width,CV_32F);
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(this->image.at<int>(i,j)>0)
                imf.at<float>(i,j)=this->image.at<int>(i,j)/255.0;
        }
    }
    imf=closing(imf,5);
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(imf.at<float>(i,j)>0.0)
                this->image.at<int>(i,j)=imf.at<float>(i,j)*255;
        }
    }
}
void ImageProcessing::enregistre(QString nom)
{
    QString titre=nom;
    titre.push_back("Transfo.png");
    //imwrite( titre.toStdString(), this->image );

    QImage imaE(this->width,this->height,QImage::Format_RGB888);
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            QRgb rgb;
            if(this->image.at<int>(i,j)==0)
                rgb=QColor(0,0,0).rgb();
            else
                rgb=cm.getQrgb(this->image.at<int>(i,j));
            imaE.setPixel(j,i,rgb);
        }
    }
    imaE.save(titre,"PNG");
}

//void ImageProcessing::enregistreRGB(QString nom)
//{
//    QString titre=nom;
//    titre.push_back("Transfo.ppm");

//    //open the file
//    QFile file(titre);

//    //If there is an error
//    QString MesErreur=" The file";
//    MesErreur.push_back(titre);
//    MesErreur.push_back("have not been saved, check the write permission!");

//    //Test if you can write into the file
//    if(file.open(QIODevice::WriteOnly )){
//        //if you can, initialize flu and line
//        QTextStream flux(&file);
//        //defines the codec of file
//        flux.setCodec("UTF-8");
//        //writing
//        flux << "P3" << endl;
//        flux << QString::number(this->width) << endl;
//        flux << QString::number(this->height) << endl;
//        flux << "# comment" << endl;
//        flux << QString::number(255) << endl;
//        QString ligne="";
//        for(int i=0; i<this->height; i++)
//        {
//             for(int j=0; j<this->width; j++)
//             {
//                 QVector<double> rgb=cm.getColor(this->image.at<int>(i,j));
//                 int r=rgb[0]*255;
//                 int g=rgb[1]*255;
//                 int b=rgb[2]*255;

//                 if(ligne.size()<58)
//                 {
//                    ligne.push_back(QString::number(r));
//                    ligne.push_back(" ");
//                    ligne.push_back(QString::number(g));
//                    ligne.push_back(" ");
//                    ligne.push_back(QString::number(b));
//                    ligne.push_back("\t");
//                 }
//                 else
//                 {
//                     flux << ligne << endl;
//                     ligne="";
//                     ligne.push_back(QString::number(r));
//                     ligne.push_back(" ");
//                     ligne.push_back(QString::number(g));
//                     ligne.push_back(" ");
//                     ligne.push_back(QString::number(b));
//                     ligne.push_back("\t");
//                 }
//             }
//         }

//        file.close();
//    }else throw Erreur(MesErreur.toStdString());
//}

void ImageProcessing::increase(int r, int c)
{
    int val=this->image.at<int>(r,c);
    this->image.at<int>(r,c)=val+1;
}

void ImageProcessing::calibration()
{
    //we keep min and max values
    int *val=MinMax();
    int min=val[0];
    int max=val[1];
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
            if(this->image.at<int>(i,j)<=s)
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

void ImageProcessing::growingRegion()
{
    //initialize of matrix
    cv::Mat im= cv::Mat::zeros(this->height,this->width,CV_32S);

    //initialize of counter
    int nbr=0;
    //we cover the image
    for(int l=0; l<this->height; l++)
    {
        //--------------------First row
        if(l==0)
        {
            //we initialize the first line
            for(int c=0; c<this->width; c++)
            {
                //--------------------First cols
                if(this->image.at<int>(l,c)==255 && c==0)
                {
                    //increase the number of region
                    nbr++;
                    //we attribute the value of new region
                    im.at<int>(l,c)=nbr;
                }
                //--------------------Other cols
                //if it is a track and it is not the first col
                if(this->image.at<int>(l,c)==255 && c>0)
                {
                    //if the previous pixel is not a track
                    if(im.at<int>(l,c-1)==0)
                    {
                        //increase the number of region
                        nbr++;
                        //we attribute the value of new region
                        im.at<int>(l,c)=nbr;
                    }
                    else
                    {
                        //else we attribute the region value of the previous pixel
                        im.at<int>(l,c)=im.at<int>(l,c-1);
                    }
                }
            }
        }
        //--------------------Other rows
        else
        {
            for(int c=0; c<this->width; c++)
            {
                //--------------------First cols
                if(this->image.at<int>(l,c)==255 && c==0)
                {
                    if(im.at<int>(l-1,c)!=0)
                    {
                        im.at<int>(l,c)=im.at<int>(l-1,c);
                    }
                    if(c<this->height-1)
                    {
                        if(im.at<int>(l,c)!=0)
                        {
                            if(im.at<int>(l-1,c+1)!=0 && im.at<int>(l-1,c+1)!=im.at<int>(l-1,c))
                            {
                                im=fusionne(im,l,c,im.at<int>(l-1,c),im.at<int>(l-1,c+1));
                            }
                        }
                        else
                        {
                            if(im.at<int>(l-1,c+1)==0)
                            {
                                //increase the number of region
                                nbr++;
                                //we attribute the value of new region
                                im.at<int>(l,c)=nbr;
                            }
                            else
                            {
                                im.at<int>(l,c)=im.at<int>(l-1,c+1);
                            }
                        }
                    }
                }
                //--------------------Other cols
                if(this->image.at<int>(l,c)==255 && c>0)
                {
                    //on regarde les voisins deja traites

                    //col-1
                    if(im.at<int>(l,c-1)!=0)
                    {
                        im.at<int>(l,c)=im.at<int>(l-1,c);
                    }
                    //row-1 et col-1
                    if(im.at<int>(l-1,c-1)!=0)
                    {
                        if(im.at<int>(l,c)!=0)
                        {
                            im=fusionne(im,l,c,im.at<int>(l,c),im.at<int>(l-1,c-1));
                        }
                        else{
                            im.at<int>(l,c)=im.at<int>(l-1,c-1);
                        }
                    }
                    //row-1 et col
                    if(im.at<int>(l-1,c)!=0)
                    {
                        if(im.at<int>(l,c)!=0)
                        {
                            im=fusionne(im,l,c,im.at<int>(l,c),im.at<int>(l-1,c));
                        }
                        else{
                            im.at<int>(l,c)=im.at<int>(l-1,c);
                        }
                    }
                    //row-1 et col+1
                    if(im.at<int>(l-1,c+1)!=0)
                    {
                        if(im.at<int>(l,c)!=0)
                        {
                            im=fusionne(im,l,c,im.at<int>(l,c),im.at<int>(l-1,c+1));
                        }
                        else{
                            im.at<int>(l,c)=im.at<int>(l-1,c+1);
                        }
                    }

                    //si aucun ne fait parti d'une region on en cree une nouvelle
                    if(im.at<int>(l,c)==0)
                    {
                        //increase the number of region
                        nbr++;
                        //we attribute the value of new region
                        im.at<int>(l,c)=nbr;
                    }
                }
            }
        }
    }

    //mise en evidence des differentes regions
    this->image=im;
    //recoloration(im, nbr);

}

cv::Mat ImageProcessing::fusionne(cv::Mat im, int l, int c, int nouvelleV, int ancienneV)
{
    for(int i=0; i<=l; i++)
    {
        for(int j=0; j<c; j++)
        {
            if(im.at<int>(i,j)==ancienneV)
            {
                im.at<int>(i,j)=nouvelleV;
            }
        }
    }
    return im;
}

void ImageProcessing::recoloration(cv::Mat im, int nbr){
    int step= 180/nbr;
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(this->image.at<int>(i,j)!=0)
            {
                this->image.at<int>(i,j)=50+((im.at<int>(i,j)-1)*step);
            }
        }
    }
//    cv::Mat imCoul=cv::Mat::zeros(this->height,this->width,CV_8UC3);
//    for(int i=0; i<this->height; i++)
//    {
//        for(int j=0; j<this->width; j++)
//        {
//            if(this->image.at<int>(i,j)!=0)
//            {
//                QVector<double> coul=cm.getColor(im.at<int>(i,j));
//                imCoul.at<int>(i,j,0)=coul.at(0)*255;
//                imCoul.at<int>(i,j,1)=coul.at(1)*255;
//                imCoul.at<int>(i,j,2)=coul.at(2)*255;
//            }
//        }
//    }
//    imwrite( "testCouleur", imCoul);
}

void ImageProcessing::Harris()
{
    cv::Mat im;
    cv::Mat imf=cv::Mat::zeros(this->height,this->width,CV_32F);
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(this->image.at<int>(i,j)>0)
                imf.at<float>(i,j)=this->image.at<int>(i,j)/255.0;
        }
    }
    cv::cornerHarris(imf,im,3,3,0.04);
    cv::Mat imR=cv::Mat::zeros(this->height,this->width,CV_32S);
    for(int i=0; i<this->height; i++)
    {
        for(int j=0; j<this->width; j++)
        {
            if(im.at<float>(i,j)>0.0)
                imR.at<int>(i,j)=im.at<float>(i,j)*255;
        }
    }
    imwrite( "testHarris.jpg", imR);
}
int ImageProcessing::getWidth() const
{
    return width;
}

void ImageProcessing::setWidth(int value)
{
    width = value;
}
int ImageProcessing::getHeight() const
{
    return height;
}

void ImageProcessing::setHeight(int value)
{
    height = value;
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
