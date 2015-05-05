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
#include "datapackage.h"
Datapackage::Datapackage(QByteArray datas, int start)
{
    this->readData(datas,start);
}
Datapackage::Datapackage()
{
}

Datapackage::~Datapackage()
{

}
void Datapackage::readDataFile(std::string pathname){
    QFile fichier( QString(pathname.c_str()) );
    if(!fichier.open(QIODevice::ReadOnly)){
        throw Erreur("the file "+pathname +"have not been opened!");
    }
    QDataStream ds(&fichier);  ds.setByteOrder(QDataStream::LittleEndian);
    this->end=288;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //create stream without  data hearder

    qint8 header[288];
    for(int i=0;i<  this->end;i++){
        qint8 size; // Since the size you're trying to read appears to be 2 bytes
        ds >> size;
        header[i]=size;
    }

    // ================================= OK=================================
    qint8 size; // Since the size you're trying to read appears to be 2 bytes
    ds >> size;
    this->id.append(QChar(size));
    ds >> size;
    this->id.append(QChar(size));
    ds >> size;
    this->version.append(QChar(size));
    ds >> size;
    this->version.append(QChar(size));
    // ==================================================================
    quint16 result;
    int b32;// Since the size you're trying to read appears to be 2 bytes
    quint8 byte0;
    quint8 byte1;
    quint8 byte2;
    quint8 byte3;

    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->radius=result;
    ds >> byte0;
    this->radienkorrektur=byte0;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->spurweite=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglLeftHorizontal=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglLeftVertical=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglLeftBanking=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglLeftGauge=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglRightHorizontal=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglRightVertical=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglRightBanking=result;
    ds >> byte1 >> byte0;
    result = (byte0 << 8) + byte1;
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->nglRightGauge=result;
    ds >> byte3 >> byte2 >> byte1 >> byte0;
    b32 = (byte0 << 24) + (byte1 << 16)+ (byte2 << 8)+ (byte3);
    byte0=0;byte1=0;byte2=0;byte3=0;
    this->pointCount=b32;

    this->end+=29;// number of bytes previously readed

    std::cout<<this->id.toStdString()<<std::endl;
    std::cout<<this->version.toStdString()<<std::endl;
    std::cout<<this->radius<<std::endl;
    std::cout<<this->radienkorrektur<<std::endl;
    std::cout<<this->spurweite<<std::endl;
    std::cout<<this->nglLeftHorizontal<<std::endl;
    std::cout<<this->nglLeftVertical<<std::endl;
    std::cout<<this->nglLeftBanking<<std::endl;
    std::cout<<this->nglLeftGauge<<std::endl;
    std::cout<<this->nglRightHorizontal<<std::endl;
    std::cout<<this->nglRightVertical<<std::endl;
    std::cout<<this->nglRightBanking<<std::endl;
    std::cout<<this->nglRightGauge<<std::endl;
    std::cout<<this->pointCount<<std::endl;

    // Fill in the cloud data
    /* cloud->width    = this->pointCount;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
*

    for(INT32 i=0;i<this->pointCount;i++)
    {
       INT16 y;
        INT16 z;
        //read point
        ds >>y;
        ds >>z;

       // cloud->push_back(pcl::PointXYZ(0,y,z));
    }*/
    this->end+=this->pointCount;
    //affectation new cloud at the current cloud
    this->cloud=cloud;

}
/*void Datapackage::readData(QByteArray datas,int start){
    this->end=start;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //create stream without  data hearder
    QDataStream ds(datas.right(datas.size()-start));
    ds.setByteOrder(QDataStream::LittleEndian);
    // ================================= OK=================================
    qint8 size; // Since the size you're trying to read appears to be 2 bytes
    ds >> size;
    this->id.append(QChar(size));
    ds >> size;
    this->id.append(QChar(size));
    ds >> size;
    this->version.append(QChar(size));
    ds >> size;
    this->version.append(QChar(size));
    // ==================================================================
    qint16 b16;
    qint8 b8;
    qint32 b32;// Since the size you're trying to read appears to be 2 bytes
    ds >> b16;
    ds >> b16; this->radius=b16;b16=0;
    ds >> b8;this->radienkorrektur=b8;b8=0;
    ds >> b16;this->spurweite=b16;b16=0;
    ds >> b16;this->nglLeftHorizontal=b16;b16=0;
    ds >> b16;this->nglLeftVertical=b16;b16=0;
    ds >> b16;this->nglLeftBanking=b16;b16=0;
    ds >> b16;this->nglLeftGauge=b16;b16=0;
    ds >> b16;this->nglRightHorizontal=b16;b16=0;
    ds >> b16;this->nglRightVertical=b16;b16=0;
    ds >> b16;this->nglRightBanking=b16;b16=0;
    ds >> b16;this->nglRightGauge=b16;b16=0;
    ds >> b32; this->pointCount=b32;b32=0;

    this->end+=29;// number of bytes previously readed

    std::cout<<this->id.toStdString()<<std::endl;
    std::cout<<this->version.toStdString()<<std::endl;
    std::cout<<this->radius<<std::endl;
    std::cout<<this->radienkorrektur<<std::endl;
    std::cout<<this->spurweite<<std::endl;
    std::cout<<this->nglLeftHorizontal<<std::endl;
    std::cout<<this->nglLeftVertical<<std::endl;
    std::cout<<this->nglLeftBanking<<std::endl;
    std::cout<<this->nglLeftGauge<<std::endl;
    std::cout<<this->nglRightHorizontal<<std::endl;
    std::cout<<this->nglRightVertical<<std::endl;
    std::cout<<this->nglRightBanking<<std::endl;
    std::cout<<this->nglRightGauge<<std::endl;
    std::cout<<this->pointCount<<std::endl;

    // Fill in the cloud data
    /* cloud->width    = this->pointCount;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
*

    for(INT32 i=0;i<this->pointCount;i++)
    {
       INT16 y;
        INT16 z;
        //read point
        ds >>y;
        ds >>z;

       // cloud->push_back(pcl::PointXYZ(0,y,z));
    }*
    this->end+=this->pointCount;
    //affectation new cloud at the current cloud
    this->cloud=cloud;

}
//*/
void Datapackage::readData(QByteArray datas,int start){

    this->end=start;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //create stream without  data hearder
    QDataStream ds(datas.right(datas.size()-start));
    ds.setByteOrder(QDataStream::LittleEndian);
    //rad header if start equals 0
    if(start==0){
        for(int i=0;i<288;i++){
            qint8 size; // Since the size you're trying to read appears to be 2 bytes
            ds >> size;
            header[i]=size;
        }
    }

    // ================================= OK=================================
    qint8 size; // Since the size you're trying to read appears to be 2 bytes
    ds >> size;
    this->id.append(QChar(size));
    ds >> size;
    this->id.append(QChar(size));
    ds >> size;
    this->version.append(QChar(size));
    ds >> size;
    this->version.append(QChar(size));
    if(this->id!="PD"||this->version!="00"){
        std::cout<<this->id.toStdString() <<this->version.toStdString()<<" The data to reading is not a valid package."<<std::endl;
        this->end=datas.size();//4+4;throw Erreur("The data to reading is not a valid package.");
    }
    else{
        // ===================================================================
        quint8 byte0;
        quint8 byte1;
        quint8 byte2;
        quint8 byte3;
        //read structure's size
        ds >> byte3 >> byte2 >> byte1 >> byte0;
        this->structuresize = (byte0 << 24) + (byte1 << 16)+ (byte2 << 8)+ (byte3);

        //read footpulse
        ds >> byte3 >> byte2 >> byte1 >> byte0;
        this->footpulse = (byte0 << 24) + (byte1 << 16)+ (byte2 << 8)+ (byte3);

        //read 12 unknown bytes
        for(int i=0;i<  12;i++){
            qint8 size; // Since the size you're trying to read appears to be 2 bytes
            ds >> size;
            unknownbytes[i]=size;
        }
        //read numbers of points
        ds >> byte1 >> byte0;
        this->pointCount = (byte0 << 8) + byte1;

        //read 150 other unknown bytes

        for(int i=0;i<  150;i++){
            qint8 size; // Since the size you're trying to read appears to be 2 bytes
            ds >> size;
            unknownbytes2[i]=size;
        }
        //  ---------- try to find the origin coordinate
        int choix=4;
        if(choix==0){
            this->originHor = (unknownbytes2[40] << 8) + unknownbytes2[39];
            this->originVert = (unknownbytes2[42] << 8) + unknownbytes2[41];
        }else  if(choix==1){
            this->originHor = (unknownbytes2[46] << 8) + unknownbytes2[45];
            this->originVert = (unknownbytes2[48] << 8) + unknownbytes2[47];
        }else  if(choix==2){
            this->originHor = (unknownbytes2[59] << 24) +(unknownbytes2[58] << 16) +(unknownbytes2[57] << 8) + unknownbytes2[56];
            this->originVert = (unknownbytes2[63] << 24) +(unknownbytes2[62] << 16) +(unknownbytes2[61] << 8) + unknownbytes2[60];
        }else  if(choix==3){
            this->originHor = (unknownbytes2[31] << 24) +(unknownbytes2[30] << 16) +(unknownbytes2[29] << 8) + unknownbytes2[28];
            this->originVert = (unknownbytes2[35] << 24) +(unknownbytes2[34] << 16) +(unknownbytes2[33] << 8) + unknownbytes2[32];
        }
else  if(choix==4){

            this->originHor =(unknownbytes2[33] << 8) + unknownbytes2[32];
            this->originVert =(unknownbytes2[35] << 8) + unknownbytes2[34];
        }
        //  ---------- end try

        //read distance and intensity for all points
        for(int i=0;i<this->pointCount;i++){
            ds >> byte1 >> byte0;
            unsigned short dist= (byte0 << 8) + byte1;
            this->distance.push_back(dist);
            ds >> byte2;
            char inte=byte2;
            this->intensity.push_back(inte);
        }
        this->end+=4+4+4+12+2+150+this->pointCount*3;// number of bytes previously readed
        this->update();
    }
}
QVector<double> Datapackage::getY() const
{
    return y;
}

void Datapackage::setY(const QVector<double> &value)
{
    y = value;
}

QVector<double> Datapackage::getX() const
{
    return x;
}

void Datapackage::setX(const QVector<double> &value)
{
    x = value;
}



// Transformation eines Profils vom Typ tyHSPRESULT in kartesischen
// Koordinaten
void Datapackage::update(){
    // Berechnung der kartesichen Koordinaten eines Profils
    // innerhalb der Scanebene
    int  HSPPOINTS = this->pointCount;
    // Ursprung der Polarkoordinaten in Schienenkoordinaten [m]
    double ox = double(this->originHor)*0.001;
    double oy = double(this->originVert)*0.001;
    // Winkelauflösung des Profils
    double step = 2*M_PI/(HSPPOINTS);
    int points = 0; // Anzahl der gültigen Messpunkte
    double angle = 0; // Winkel zur negativen Vertikalachse
    for (int i=0; i<HSPPOINTS; i++) {
        // Gültigkeit des Messpunktes abfragen
       /* if(this->intensity.at(i)!='0')
        {*/
            // Radius des Messpunktes [m]
            double r = double(this->distance.at(i))*0.001;
            // Koordinatentransformation Polar -> Kartesisch
            x.push_back(ox + r*sin(angle));
            y.push_back(oy - r*cos(angle));
            points++;
       // }
        angle += step; // Winkel des nächsten Punktes
    }
    /*
// Anzeige des Profils im Graph
this->setCurveData(curveProfile, &x[0], &y[0], points);
this->setCurveStyle(curveProfile, QwtCurve::Dots);
this->setCurvePen(curveProfile,QPen::green);
this->replot();*/
}


int Datapackage::bytesToInt(const char *buffer, int size)
{
    QByteArray byteArr(buffer,size);
    QDataStream ds(&byteArr,QIODevice::ReadOnly);
    bool little_endian_usage=1;
    if(little_endian_usage) // little endian check or something similar here
        ds.setByteOrder(QDataStream::LittleEndian);
    else
        ds.setByteOrder(QDataStream::BigEndian);

    int ret;
    if(size == 2){
        qint16 tmp;
        ds >> tmp;
        ret = tmp;
    } else if(size == 4){
        qint32 tmp;
        ds >> tmp;
        ret = tmp;
    } else if(size == 1){
        qint8 tmp;
        ds >> tmp;
        ret = tmp;
    }
    return ret;
}
int Datapackage::bytesToInt( QByteArray arbuffer, int size,char *buffer)
{
    QDataStream input(arbuffer);

    input.readRawData(buffer, sizeof(buffer));

    QByteArray byteArr(buffer,size);
    QDataStream ds(&byteArr,QIODevice::ReadOnly);
    bool little_endian_usage=1;
    if(little_endian_usage) // little endian check or something similar here
        ds.setByteOrder(QDataStream::LittleEndian);
    else
        ds.setByteOrder(QDataStream::BigEndian);

    int ret;
    if(size == 2){
        qint16 tmp;
        ds >> tmp;
        ret = tmp;
    } else if(size == 4){
        qint32 tmp;
        ds >> tmp;
        ret = tmp;
    } else if(size == 1){
        qint8 tmp;
        ds >> tmp;
        ret = tmp;
    }
    return ret;
}
/*
void Datapackage::readData(QByteArray datas,int start){
    this->end=start;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //create stream without  data hearder
    //QDataStream ds(datas.right(datas.size()-start));

    //create buffer for read all datas one by one
    QByteArray buffer;

    // ------- read id
    //fill the buffer
    buffer.append(datas.at(start+0));
    buffer.append(datas.at(start+1));
    //load data
    this->id=QString::fromStdString(buffer.toStdString());
    //empty the buffer
    buffer.clear();
    // ------- read version
    //fill the buffer
    buffer.append(datas.at(start+2));
    buffer.append(datas.at(start+3));
    //load data
    this->version=QString::fromStdString(buffer.toStdString());;
    //empty the buffer
    buffer.clear();
    // ------- read radius
    //fill the buffer
    buffer.append(datas.at(start+4));
    buffer.append(datas.at(start+5));
    //load data
    char cbuf[2];
    this->radius=this->bytesToInt(buffer,2,cbuf);

    //empty the buffer
    buffer.clear();
    // ------- read radienkorrektur
    //fill the buffer
    buffer.append(datas.at(start+6));
    //load data
     cbuf[1];
    this->radienkorrektur=this->bytesToInt(buffer,1,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read spurweite
    //fill the buffer
    buffer.append(datas.at(start+7));
    buffer.append(datas.at(start+8));
    //load data
     cbuf[2];
    this->spurweite=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglLeftHorizontal
    //fill the buffer
    buffer.append(datas.at(start+9));
    buffer.append(datas.at(start+10));
    //load data
     cbuf[2];
    this->nglLeftHorizontal=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglLeftVertical
    //fill the buffer
    buffer.append(datas.at(start+11));
    buffer.append(datas.at(start+12));
    //load data
     cbuf[2];
    this->nglLeftVertical=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglLeftBanking
    //fill the buffer
    buffer.append(datas.at(start+13));
    buffer.append(datas.at(start+14));
    //load data
     cbuf[2];
    this->nglLeftBanking=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglLeftGauge
    //fill the buffer
    buffer.append(datas.at(start+15));
    buffer.append(datas.at(start+16));
    //load data
     cbuf[2];
    this->nglLeftGauge=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglRightHorizontal
    //fill the buffer
    buffer.append(datas.at(start+17));
    buffer.append(datas.at(start+18));
    //load data
     cbuf[2];
    this->nglRightHorizontal=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglRightVertical
    //fill the buffer
    buffer.append(datas.at(start+19));
    buffer.append(datas.at(start+20));
    //load data
     cbuf[2];
    this->nglRightVertical=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglRightBanking
    //fill the buffer
    buffer.append(datas.at(start+21));
    buffer.append(datas.at(start+22));
    //load data
     cbuf[2];
    this->nglRightBanking=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read nglRightGauge
    //fill the buffer
    buffer.append(datas.at(start+23));
    buffer.append(datas.at(start+24));
    //load data
     cbuf[2];
    this->nglRightGauge=this->bytesToInt(buffer,2,cbuf);
    //empty the buffer
    buffer.clear();
    // ------- read pointCount
    //fill the buffer
    buffer.append(datas.at(start+25));
    buffer.append(datas.at(start+26));
    buffer.append(datas.at(start+27));
    buffer.append(datas.at(start+28));

     QDataStream input2(buffer);
 cbuf[4];
     input2.readRawData(cbuf, sizeof(cbuf));
    this->pointCount=this->bytesToInt(cbuf,4);
      buffer.clear();

    this->end+=29;// number of bytes previously readed
    std::cout<<this->id.toStdString()<<std::endl;
    std::cout<<this->version.toStdString()<<std::endl;
    std::cout<<this->radius<<std::endl;
    std::cout<<this->radienkorrektur<<std::endl;
    std::cout<<this->spurweite<<std::endl;
    std::cout<<this->nglLeftHorizontal<<std::endl;
    std::cout<<this->nglLeftVertical<<std::endl;
    std::cout<<this->nglLeftBanking<<std::endl;
    std::cout<<this->nglLeftGauge<<std::endl;
    std::cout<<this->nglRightHorizontal<<std::endl;
    std::cout<<this->nglRightVertical<<std::endl;
    std::cout<<this->nglRightBanking<<std::endl;
    std::cout<<this->nglRightGauge<<std::endl;
    std::cout<<this->pointCount<<std::endl;

    // Fill in the cloud data
/* cloud->width    = this->pointCount;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
*//*

    for(INT32 i=0;i<this->pointCount;i++)
    {
      // INT16 y;
        //INT16 z;
        //read point
      //  ds >>y;
       // ds >>z;

       // cloud->push_back(pcl::PointXYZ(0,y,z));
    }//*//*
    this->end+=this->pointCount;
    //affectation new cloud at the current cloud
    this->cloud=cloud;

}*/

void Datapackage::Print(std::ostream& O) const
{
    O << "id + version: "<<this->id.toStdString()<<this->version.toStdString()<<std::endl;
    O << "size of structure: "<<this->structuresize<<" Bytes"<<std::endl;
    O << "Footpulse: "<<this->footpulse<<std::endl;
    O << "Points count: "<<this->pointCount<<std::endl;
    O << "origine vertical: "<<this->originVert<<std::endl;
    O << "origine horizontal: "<<this->originHor<<std::endl;
    for(int i=0;i<this->pointCount;i++){
        O << "Point: "<<i<<std::endl;
        // O << "      distance: "<<this->distance.at(i)<<std::endl;
        //O << "      intensity "<<this->intensity.at(i)<<std::endl;
        O << "      x: "<<this->x.at(i)<<std::endl;
        O << "      y "<<this->y.at(i)<<std::endl;
        O << "      z "<<this->footpulse<<std::endl;

    }
}

std::ostream& operator << (std::ostream& O, const Datapackage& B)
{
    B.Print(O);
    return O;
}
std::string Datapackage::toString() const{

    std::stringstream ss;
    this->Print(ss);
    std::string myString = ss.str();
    return myString;
}

// -------------------------------- GETTER and SETTER --------------------------------
int Datapackage::getEnd() const
{
    return end;
}

void Datapackage::setEnd(int value)
{
    end = value;
}

int Datapackage::getPointCount() const
{
    return pointCount;
}

void Datapackage::setPointCount(const int &value)
{
    pointCount = value;
}

int Datapackage::getNglRightGauge() const
{
    return nglRightGauge;
}

void Datapackage::setNglRightGauge(const int &value)
{
    nglRightGauge = value;
}

int Datapackage::getNglRightBanking() const
{
    return nglRightBanking;
}

void Datapackage::setNglRightBanking(const int &value)
{
    nglRightBanking = value;
}

int Datapackage::getNglRightVertical() const
{
    return nglRightVertical;
}

void Datapackage::setNglRightVertical(const int &value)
{
    nglRightVertical = value;
}

int Datapackage::getNglRightHorizontal() const
{
    return nglRightHorizontal;
}

void Datapackage::setNglRightHorizontal(const int &value)
{
    nglRightHorizontal = value;
}

int Datapackage::getNglLeftGauge() const
{
    return nglLeftGauge;
}

void Datapackage::setNglLeftGauge(const int &value)
{
    nglLeftGauge = value;
}

int Datapackage::getNglLeftBanking() const
{
    return nglLeftBanking;
}

void Datapackage::setNglLeftBanking(const int &value)
{
    nglLeftBanking = value;
}

int Datapackage::getNglLeftVertical() const
{
    return nglLeftVertical;
}

void Datapackage::setNglLeftVertical(const int &value)
{
    nglLeftVertical = value;
}

int Datapackage::getNglLeftHorizontal() const
{
    return nglLeftHorizontal;
}

void Datapackage::setNglLeftHorizontal(const int &value)
{
    nglLeftHorizontal = value;
}

int Datapackage::getSpurweite() const
{
    return spurweite;
}

void Datapackage::setSpurweite(const int &value)
{
    spurweite = value;
}

int Datapackage::getRadienkorrektur() const
{
    return radienkorrektur;
}

void Datapackage::setRadienkorrektur(const int &value)
{
    radienkorrektur = value;
}

int Datapackage::getRadius() const
{
    return radius;
}

void Datapackage::setRadius(const int &value)
{
    radius = value;
}

QString Datapackage::getVersion() const
{
    return version;
}

void Datapackage::setVersion(const QString &value)
{
    version = value;
}

QString Datapackage::getId() const
{
    return id;
}

void Datapackage::setId(const QString &value)
{
    id = value;
}


QVector<char> Datapackage::getIntensity() const
{
    return intensity;
}

void Datapackage::setIntensity(const QVector<char> &value)
{
    intensity = value;
}

QVector<unsigned short> Datapackage::getDistance() const
{
    return distance;
}

void Datapackage::setDistance(const QVector<unsigned short> &value)
{
    distance = value;
}



int Datapackage::getFootpulse() const
{
    return footpulse;
}

void Datapackage::setFootpulse(int value)
{
    footpulse = value;
}

int Datapackage::getStructuresize() const
{
    return structuresize;
}

void Datapackage::setStructuresize(int value)
{
    structuresize = value;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Datapackage::getCloud() const
{
    return cloud;
}

void Datapackage::setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &value)
{
    cloud = value;
}
