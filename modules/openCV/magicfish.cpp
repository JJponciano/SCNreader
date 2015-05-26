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
#include "magicfish.h"
Magicfish::Magicfish()
{
    threshold=80;
    closing_size=1;
    last_closing_size=4;
    opening_size=1;
    this->intensite=20;
    this->brillance=20;
    this->seuil=5;
    //coordinated pixel to processing
    this->x=1;
    this->y=1;
    //==============================================TEST===============================
    //this->whriteVideoDefBackground("C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\a.avi","C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\testF.avi");
    //this->initialization("C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\a.avi");
    //this->whriteVideo("C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\a.avi","C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\testFpretreatment.avi");

    //test video
    //OK
    //this->video("C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\a.avi","C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\test\\");
    //KO
    // cv::Mat* m1=new cv::Mat(cv::imread("C :\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\frame-3.tif", CV_LOAD_IMAGE_COLOR));
    // QImage I1=this->cvMatToQImage(*m1);


    /*  cv::Mat* m2=new cv::Mat(cv::imread("C:\\Users\\Claire\\Documents\\svn\\projetPoisson\\filmPoisson\\frame-3.tif", CV_LOAD_IMAGE_COLOR));
    QImage I2=this->cvMatToQImage(*m2);

   QImage im2=Modification(50,100,100,100,100,I2);
   int h=im2.size().height();
   cv::Mat m3=this->QImageToCvMat(im2);
   cv::imwrite("C:\\Users\\Claire\\Documents\\svn\\projetPoisson\\test1.tif", m3);
       OK

    cv::Mat* m2=new cv::Mat(cv::imread("C :\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\frame-3.tif", CV_LOAD_IMAGE_COLOR));
    QImage I2=this->cvMatToQImage(*m2);
    cv::Mat* m3=new cv::Mat(cv::imread("C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\frame-300.tif", CV_LOAD_IMAGE_COLOR));
    QImage I3=this->cvMatToQImage(*m3);
    cv::Mat* m4=new cv::Mat(cv::imread("C:\\Users\\Jean-Jacques\\Documents\\workspace\\svn\\projetPoisson\\filmPoisson\\frame-2441.tif", CV_LOAD_IMAGE_COLOR));
    QImage I4=this->cvMatToQImage(*m4);
   // QImage qi=this->getFish(I1,I2,I3,I4,90,1,4,1);
    cv::Mat res=this->QImageToCvMat(I4);
    cv::imshow("test",res);
*/
    // cv::imshow("test",res);
    //delete m1;
    // m1=0;
    //delete m2;
    //m2=0;

}

float Magicfish::CalculEcart(QRgb couleurRef, QRgb couleurT)
{
    //récupération des valeurs rgb de la couleur de référence
    double r1=qRed(couleurRef);
    double g1=qGreen(couleurRef);
    double b1=qBlue(couleurRef);
    //std::cout<<"r1= "<<r1<<" g1= "<<g1<<" b1= "<<b1<<std::endl;

    //récupération des valeurs rgb de la couleur testée
    double r2=qRed(couleurT);
    double g2=qGreen(couleurT);
    double b2=qBlue(couleurT);
    //std::cout<<"r2= "<<r2<<" g2= "<<g2<<" b2= "<<b2<<std::endl;

    //Calcul de la distance euclidienne
    float distance= sqrt(pow(r1-r2,2)+pow(g1-g2,2)+pow(b1-b2,2));
    //std::cout<<"d= "<<distance<<std::endl;

    //retourne la distance euclidienne
    return distance;
}

QColor Magicfish::ModifPixel(QRgb couleur, int Qs, int Qv)
{
    //Création d'une couleur à partir en rgb
    QColor coulRgb=QColor(couleur);
    //Création de la couleur hsv correspondant à la couleur rgb précédente
    QColor coulHsv = coulRgb.toHsv();

    //récupération des valeurs hsv et modification
    int h=coulHsv.hsvHue();
    int s=(coulHsv.hsvSaturation())*Qs/100;
    int v=(coulHsv.value())*Qv/100;
    int a=coulHsv.alpha();

    //application de la modification
    //std::cout<<"h:"<<h<<",s:"<<s<<",v:"<<v<<std::endl;
    coulHsv.setHsv(h, s, v, a);

    //récupération de la valeur rgb de la nouvelle couleur hsv
    coulRgb=coulHsv.toRgb();
    //retourne la nouvelle couleur
    return coulRgb;
}

QImage  Magicfish::Modification(int intensite, int brillance, float seuil, int x, int y, QImage im)
{
    //création de la nouvelle image
    QImage image=im;

    //récupération des dimensions de l'image
    int dimN=im.size().height();
    int dimM=im.size().width();

    //récupération de la couleur du pixel de référence
    QRgb rgbRef=im.pixel(x,y);

    //parcourt de l'image
    for(int k=0; k<dimM; k++)
        for(int l=0; l<dimN; l++)
        {
            //pour chaque pixel, on calcul sa distance euclidienne avec le pixel de référence
            float d=CalculEcart(rgbRef,im.pixel(k,l));
            //si la distance est inférieure au seuil
            if(d<=seuil)
            {
                //on applique le changement de couleur
                QColor couleur=ModifPixel(im.pixel(k,l), brillance, intensite);
                image.setPixel(k,l,couleur.rgb());
            }
        }
    //retourne l'image modifiée
    return image;
}

QImage Magicfish::RecupImage(cv::Mat matrice)
{
    switch ( matrice.type() )
    {
    // 8-bit, 4 channel
    case CV_8UC4:
    {
        QImage image( matrice.data, matrice.cols, matrice.rows, matrice.step, QImage::Format_RGB32 );

        return image;
    }

        // 8-bit, 3 channel
    case CV_8UC3:
    {
        QImage image( matrice.data, matrice.cols, matrice.rows, matrice.step, QImage::Format_RGB888 );

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

        QImage image( matrice.data, matrice.cols, matrice.rows, matrice.step, QImage::Format_Indexed8 );

        image.setColorTable( sColorTable );

        return image;
    }

    default:
        printf("o");
        break;
    }

    return QImage();
}



/**  @function Erosion  */
cv::Mat Magicfish::erosion( cv::Mat src, int  erosion_size )
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
cv::Mat Magicfish::dilation(cv::Mat src, int dilation_size )
{
    //set the final matrix after dilation
    cv::Mat dilation_dst;
    // Apply the dilatation
    cv::dilate(src, dilation_dst, cv::Mat());
    for(int i=0;i<dilation_size-1;i++)
        cv::dilate(dilation_dst, dilation_dst, cv::Mat());
    return dilation_dst;
}
cv::Mat Magicfish::opening(cv::Mat src,int opening_size)
{
    //output image
    cv::Mat dst=erosion(src,opening_size);
    dst=dilation(dst,opening_size);
    return dst;

}

cv::Mat Magicfish::closing(cv::Mat src,int closing_size)
{
    //output image
    cv::Mat dst=dilation(src,closing_size);
    dst=erosion(dst,closing_size);
    return dst;
}
QImage Magicfish::difference(QImage I, QImage I2, int threshold,int closing_size,int last_closing_size,int opening_size){
    //initialization of final image
    QImage imFinal(I.size(),QImage::Format_RGB888);
    //fill the image of values 0
    for(int i=0;i<imFinal.width();i++)
        for(int j=0;j<imFinal.height();j++){

            //get pixels
            QRgb pixel= I.pixel(i,j);
            QRgb pixel2= I2.pixel(i,j);
            // recovery of each channel of each pixels
            float r= qRed(pixel);
            float g= qGreen(pixel);
            float b= qBlue(pixel);

            float r2= qRed(pixel2);
            float g2= qGreen(pixel2);
            float b2= qBlue(pixel2);
            //tests whether the distances between the pixels is below the threshold.
            if((sqrt(pow(r-r2,2)+pow(g-g2,2)+pow(b-b2,2)))>threshold)
                //set 1 values in the final image
                imFinal.setPixel(i,j, qRgb(255,255,255));
            else   imFinal.setPixel(i,j,qRgb(0,0,0));
        }
    //convert Qimage to Mat
    cv:: Mat I4=this->QImageToCvMat(imFinal);


    // =============TRAITEMENT MORPHOLOGIQUE=======================
    //openning
    cv::Mat I5=this->opening(I4,opening_size);
    //Closing
    I5 = this->closing(I5,closing_size);
    //large closing
    cv:: Mat Ibin =  this->closing(I5,last_closing_size);

    //convert Mat to Qimage
    imFinal=this->cvMatToQImage(Ibin);
    //return final image
    return imFinal;
}

QImage Magicfish::getFish(QImage imOr,QImage imF1, QImage imF2,QImage imF3, int threshold,int closing_size,int last_closing_size,int opening_size){
    // imOr: image d'origine dont on veut isoler le poisson
    // imF1 image servant au fond
    // imF2 deuxième image servant au fond
    // imF2 troisième image servant au fond
    //seul: seuil de correspondance entre pixel ( distance euclidienne)
    //exemple getPoisson('male.png','male.png','male2.png','male3.png',60);
    //récupération de la première image binaire avec deux emplacement de poisson
    QImage I1=this->difference(imOr,imF1, threshold, closing_size, last_closing_size, opening_size);

    //récupération de la première image binaire avec deux emplacement de poisson
    //dont un est identique à un de I1
    QImage I2=this->difference( imOr,imF2, threshold, closing_size, last_closing_size, opening_size);
    // récupération du poisson identique en binaire
    QImage I3= this->onlyWhite(I1,I2);

    // on effectue le même traitement que précedment mais avec imF3 à ma place
    // ======================de ImF2===========================================
    //récupération de la première image binaire avec deux emplacement de poisson
    I1=this->difference( imOr,imF1, threshold, closing_size, last_closing_size, opening_size);

    //récupération de la première image binaire avec deux emplacement de poisson
    //dont un est identique à un de I1
    I2=this->difference( imOr,imF3, threshold, closing_size, last_closing_size, opening_size);
    // récupération du poisson identique en binaire
    QImage I4= this->onlyWhite(I1,I2);
    // =========================================================================

    // ======================de ImF1===========================================
    //récupération de la première image binaire avec deux emplacement de poisson
    I1=this->difference( imOr,imF2, threshold, closing_size, last_closing_size, opening_size);

    //récupération de la première image binaire avec deux emplacement de poisson
    //dont un est identique à un de I1
    I2=this->difference( imOr,imF3, threshold, closing_size, last_closing_size, opening_size);
    // récupération du poisson identique en binaire
    QImage I5= this->onlyWhite(I1,I2);
    // =========================================================================

    //on prensd l'image qui à le plus de pixel blanc
    int s3=this->countWhitePixel(I3);
    int s4=this->countWhitePixel(I4);
    int s5=this->countWhitePixel(I5);
    if(s4>s3 && s4>s5)
        I3=I4;
    else{
        if(s5>s3)
            I3=I5;
    }
    return I3;
}

QImage Magicfish::onlyWhite(QImage im, QImage i2){
    //initialization of final image
    QImage imFinal(im.size(),QImage::Format_RGB888);

    for(int i=0;i<im.width();i++)
        for(int j=0;j<im.height();j++){

            //get pixels
            QRgb pixel= im.pixel(i,j);
            QRgb pixel2= i2.pixel(i,j);
            // recovery of each channel of each pixels
            float r= qRed(pixel);
            float g= qGreen(pixel);
            float b= qBlue(pixel);
            float r2= qRed(pixel2);
            float g2= qGreen(pixel2);
            float b2= qBlue(pixel2);
            if(r+g+b==255*3 &&r2+g2+b2==255*3)
                //set 1 values in the final image
                imFinal.setPixel(i,j, qRgb(255,255,255));
            else   imFinal.setPixel(i,j,qRgb(0,0,0));
        }
    return imFinal;
}

int Magicfish::countWhitePixel(QImage im){
    int count=0;
    for(int i=0;i<im.width();i++)
        for(int j=0;j<im.height();j++){

            //get pixels
            QRgb pixel= im.pixel(i,j);
            // recovery of each channel of each pixels
            float r= qRed(pixel);
            float g= qGreen(pixel);
            float b= qBlue(pixel);
            if(r+g+b==255*3)count++;
        }
    return count;
}

void Magicfish::video(std::string videoName,std::string pathOutput){
    cv::VideoCapture cap(videoName); // open the  video file for reading
    if ( !cap.isOpened() )  // if not success, exit program
    {
        std::cout << "Cannot open the video file" << std::endl;
    }
    else{
        //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

        double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

        std::cout << "Frame per seconds : " << fps << std::endl;

        // Get the properties from the video
        double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
        double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);


        // set name of result video
        std::stringstream image;
        image<<pathOutput;
        image <<"resultat.avi";

        // Create the video writer
        cv::VideoWriter video("capture.avi",-1, 30, cvSize((int)width,(int)height) );
        cv::Mat frame;

        /*while(bSuccess)
        {
            cv::Mat frame;

            bool bSuccess = cap.read(frame); // read a new frame from video
            if (bSuccess) //if not success, break loop
            {
                nbIm++;*/
        //============================================
        //=====================PROCESSING=============
        //============================================
        // Check if the video was opened
        if(!video.isOpened())
        {
            std::cerr << "Could not create video.";
        }

        std::cout << "Press Esc to stop recording." << std::endl;

        // Get the next frame until the user presses the escape key
        for(int i=0;i<1000;i++)
        {
            // Get frame from capture
            cap >> frame;

            // Check if the frame was retrieved
            if(!frame.data)
            {
                std::cerr << "Could not retrieve frame.";
            }

            // Save frame to video
            video << frame;
            // Show image
            // cv::imshow("Capture", frame);
        }
        //out_capture.release();
        video.release();
        std::cout << "Fin" << std::endl;
        // }else break;
        //============================================
        //===================== END PROCESSING=============
        //============================================


    }


}
//=========================================================================================


void Magicfish::whriteVideo (std::string input,std::string output)
{
    cv::Mat src,dst;

    bool success=0;
    cv::VideoCapture capture(input);
    if (!capture.isOpened())
    {
        std::cout<<("Ouverture du flux vidéo impossible !\n"+input)<<std::endl;
    }
    success=capture.read(src);
    cv::VideoWriter Video_rec(output, CV_FOURCC('D','I','V','X'), 25,src.size(), true);
    if (!Video_rec.isOpened())
    {
        std::cout<<("Impossible d'écrire la vidéo !\n"+output)<<std::endl;
    }
    int i=0;
    while(success  &&i<20)
    {

        // src mat -> QImage
        QImage qi=this->cvMatToQImage(src);

        // processing
        qi=this->processing(qi);
        //end processing

        // QImage -> mat dst
        dst=this->QImageToCvMat(qi);

        std::cout<<"F: "<<i<<std::endl;
        i++;
        Video_rec.write(dst);
        success=capture.read(src);
    }
    Video_rec.release();
}

void Magicfish::whriteVideoDefBackground( std::string input,std::string output)
{
    cv::Mat src,dst;

    bool success=0;
    cv::VideoCapture capture(input);
    if (!capture.isOpened())
    {
        std::cout<<("Ouverture du flux vidéo impossible !\n"+input)<<std::endl;
    }
    success=capture.read(src);
    cv::VideoWriter Video_rec(output, CV_FOURCC('D','I','V','X'), 25,src.size(), true);
    if (!Video_rec.isOpened())
    {
        std::cout<<("Impossible d'écrire la vidéo !\n"+output)<<std::endl;
    }
    int i=0;
    while(success  &&i<20)
    {

        // src mat -> QImage
        QImage qi=this->cvMatToQImage(src);

        // processing
        //end processing

        // QImage -> mat dst
        dst=this->QImageToCvMat(qi);

        std::cout<<"O: "<<i<<std::endl;
        i++;
        Video_rec.write(dst);
        success=capture.read(src);
    }
    Video_rec.release();
}
void Magicfish::initialization( std::string input)
{
    this->inputfilename=input;
    cv::Mat src,dst;
    std::string output="temp.avi";
    bool success=0;
    cv::VideoCapture captureT(input);
    if (!captureT.isOpened())
    {
        std::cout<<("Ouverture du flux vidéo impossible !\n"+input)<<std::endl;
    }
    success=captureT.read(src);
    cv::VideoWriter Video_recT(output, CV_FOURCC('D','I','V','X'), 25,src.size(), true);
    if (!Video_recT.isOpened())
    {
        std::cout<<("Impossible d'écrire la vidéo !\n"+output)<<std::endl;
    }
    int j=0;// counter of background image get
    int i=0;//counter of frame traitement
    float somTemp1=0.0;
    //=========================defined the first two background images=========================
    while(success &&i<20)
    {

        // src mat -> QImage
        QImage qi=this->cvMatToQImage(src);
        // processing
        // test if qi is the first frame of the video
        if(j==0){
            this->background1=qi;
            j++;
        }
        else//else test if the current image does not fish in the same place as the other images
            //if only get  one background image
            if(j==1){
                //test if the current image does not fish in the same place as the first image
                QImage dif=this->difference( qi,this->background1, threshold, closing_size, last_closing_size, opening_size);
                somTemp1=this->countWhitePixel(dif);
                //assign the background image
                this->background2=qi;
                j++;
            }else{
                //test if the current image does not fish in the same place as the first image
                QImage dif=this->difference( qi,this->background1, threshold, closing_size, last_closing_size, opening_size);
                // test if the current image is better than second image
                if(this->countWhitePixel(dif)>somTemp1+0.1*somTemp1){
                    somTemp1=this->countWhitePixel(dif);
                    this->background2=qi;
                }
            }
        std::cout<<i<<std::endl;
        i++;
        //test if the frame is the last frame of  the video
        success=captureT.read(src);
    }
    //================================================== END ==================================================
    success=0;
    cv::VideoCapture capture(input);
    if (!capture.isOpened())
    {
        std::cout<<("Ouverture du flux vidéo impossible !\n"+input)<<std::endl;
    }
    success=capture.read(src);
    cv::VideoWriter Video_rec(output, CV_FOURCC('D','I','V','X'), 25,src.size(), true);
    if (!Video_rec.isOpened())
    {
        std::cout<<("Impossible d'écrire la vidéo !\n"+output)<<std::endl;
    }
    somTemp1=0.0;
    //=========================defined the last background image=========================
    i=0;
    while(success &&i<20)
    {

        // src mat -> QImage
        QImage qi=this->cvMatToQImage(src);

        // You have two background image. You find the last background image

        // if we have all of background image, we test if the current image is better than the second background image
        //test if the current image does not fish in the same place as the second background image
        QImage dif=this->difference( qi,this->background2, threshold, closing_size, last_closing_size, opening_size);
        //test if the current image does not fish in the same place as the first image
        QImage dif2=this->difference( qi,this->background1, threshold, closing_size, last_closing_size, opening_size);
        // test if the current image is better than third image
        if(this->countWhitePixel(dif)+this->countWhitePixel(dif2)>somTemp1+0.1*somTemp1){
            somTemp1=this->countWhitePixel(dif)+this->countWhitePixel(dif2);
            this->background3=qi;
        }

        //end processing

        // QImage -> mat dst
        dst=this->QImageToCvMat(qi);

        //write image into video
        Video_rec.write(dst);
        std::cout<<i<<std::endl;
        i++;
        //test if the frame is the last frame of  the video
        success=capture.read(src);
    }
}




IplImage* Magicfish::QImage2IplImage(const QImage& qImage)
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
QImage Magicfish::Ipl2QImage(const IplImage *newImage)
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


QImage  Magicfish::cvMatToQImage(cv::Mat const& src)
{
    cv::Mat temp; // make the same cv::Mat
    cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

cv::Mat Magicfish::QImageToCvMat( QImage const& src)
{
    cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
    cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
    cvtColor(tmp, result,CV_BGR2RGB);
    return result;
}

QImage  Magicfish::processing(QImage qi){
    QImage fish;
    //pretreatment
    fish=this->getFish(qi,this->background1,this->background2,this->background3,this->threshold,this->closing_size,this->last_closing_size,this->opening_size);
    //treatment
    QImage treatment=this->Modification( intensite,  brillance,  seuil,  x,  y,  qi);
    //merge
    QImage final=this->merge(qi,treatment,fish);
    return final;
}
QImage  Magicfish::merge(QImage intput,QImage treatment,QImage bw){
    //initialization of final image
    QImage imFinal(intput.size(),QImage::Format_RGB888);

    for(int i=0;i<intput.width();i++)
        for(int j=0;j<intput.height();j++){

            //get pixels
            QRgb pixel= treatment.pixel(i,j);
            QRgb pixelbw= bw.pixel(i,j);
            // recovery of each channel of each pixels
            float pbw= qRed(pixelbw);
            // test if bw's pixel is white
            if(pbw>100)
                //if is white, set treatment's pixel
                imFinal.setPixel(i,j, pixel);
            else //set original pixel without treatment
                imFinal.setPixel(i,j,intput.pixel(i,j));
        }
    return imFinal;
}

QDataStream & operator << (QDataStream & out, const Magicfish &Valeur)
{
    //add background images
    out<<Valeur.background1;
    out<<Valeur.background2;
    out<<Valeur.background3;
    // add parameters
    out<<Valeur.threshold;
    out<<Valeur.closing_size;
    out<<Valeur.last_closing_size;
    out<<Valeur.opening_size;
    out<<Valeur.intensite;
    out<<Valeur.brillance;
    out<<Valeur.seuil;
    QString qs(Valeur.inputfilename.c_str());
    out<<qs;
    //coordinated pixel to processing
    out<<Valeur.x;
    out<<Valeur.y;
    return out;
}
QDataStream & operator >> (QDataStream & in, Magicfish & Valeur)
{
    //add background images
    QImage im;
    in >> im;
    Valeur.setBackground(1,im);
    in >> im;
    Valeur.setBackground(2,im);
    in >> im;
    Valeur.setBackground(3,im);

    // add parameters
    int temp;
    in >> temp;Valeur.setThreshold(temp);
    in >> temp;Valeur.setclosing_size(temp);
    in >> temp;Valeur.setLast_closing_size(temp);
    in >> temp;Valeur.setOpening_size(temp);
    in >>Valeur.intensite;
    in >>Valeur.brillance;
    in >>Valeur.seuil;
    QString qs;
    in >> qs;
    Valeur.inputfilename=qs.toStdString();
    //coordinated pixel to processing
    in >>Valeur.x;
    in >>Valeur.y;
    return in;
}


QImage Magicfish::getBackground(int indiceBackground){
    if (indiceBackground==1)return this->background1;
    else
        if (indiceBackground==2)return this->background2;
        else return this->background3;
}
int Magicfish::getclosing_size(){
    return this->closing_size;
}
int Magicfish::getLast_closing_size(){
    return this->last_closing_size;
}
int Magicfish::getOpening_size(){
    return this->opening_size;
}
int Magicfish::getThreshold(){
    return this->threshold;
}
void Magicfish::setBackground(int indiceBackground, QImage qi){
    if (indiceBackground==1) this->background1=qi;
    else
        if (indiceBackground==2) this->background2=qi;
        else  this->background3=qi;
}
void Magicfish::setclosing_size(int c){
    this->closing_size=c;
}
void Magicfish::setLast_closing_size(int c){
    this->last_closing_size=c;
}
void Magicfish::setOpening_size(int o){
    this->opening_size=o;
}
void Magicfish::setThreshold(int t){
    this->threshold=t;
}
void Magicfish::initDonneesSystem ()
{
    qRegisterMetaTypeStreamOperators<Magicfish>("Magicfish");
    qMetaTypeId<Magicfish>();				// Teste la validité de la classe Contact
}
