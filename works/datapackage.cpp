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
#include <QtCore/qmath.h>
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
void Datapackage::read(QByteArray datas,int start){

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
        }this->end+=4+4+4+12+2+150;
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
            quint16 dist= (byte0 << 8) + byte1;
            this->distance.push_back(dist);
            ds >> byte2;
             quint8 inte=byte2;
            this->intensity.push_back(inte);
        }/*
        this->pointCount=0;

        bool again=true;
        while(again&&end<datas.size()){
            ds >> byte1 >> byte0;
            ds >> byte2;
            if(QChar(byte1)=='P'&& QChar(byte0)=='D' && QChar(byte2)=='0')
                again=false;
            else{
                this->pointCount++;
                quint16 dist= (byte0 << 8) + byte1;
                this->distance.push_back(dist);
                quint8 inte=byte2;
                this->intensity.push_back(inte);
                this->end+=3;
            }
        }
        //*/  this->end+=this->pointCount*3;// number of bytes previously readed
    }
}
void Datapackage::readData(QByteArray datas,int start){
    this->read(datas,start);
    this->decompression();
    this->update();

}


//remove unuse point
void Datapackage::decompression(){
    this->radDist =this->distance; // nächster Abstandswer
    this->sqrtInt=this->intensity; // nächster Intensitätswert
    int HSPPOINTS = 3600;
    int pn = 0;
   // std::cout<<"nombre de point a lire: "<<this->pointCount<<std::endl;
    for(int i=0; i < this->pointCount; i++) {
        quint8 iv =this->intensity.at(i); // nächster Intensitätswert
        quint16 dv =this->distance.at(i); // nächster Abstandswert
        // i: Punktindex innerhalb tyPOMSDATAHEADER
       /* if(pn >= HSPPOINTS) // Notbremse
            break;*/
        // iv: Zeiger auf Intensität; (*iv): Intensitätswert
        if((iv)==0) { // Ungültige Messpunkte
            // n: Anzahl der ungültigen Messpunkte
            quint16 n = (dv);//std::cout<<"n: "<<n<<std::endl;
            if(n==0) n=1; // mindestens ein Messpunkt
            // n ungültige Messpunkte in tyHSPRESULT eintragen

             for(int j=0; j < n; j++) {
                // Ungültige Messpunkte eintragen
                  /*  this->radDist.push_back(dv);
                    this->sqrtInt.push_back(0);*/
                    pn++;
            }
            // Nächster Messpunkt in tyPOMSDATAHEADER
        }
        else { // Gültiger Abstandswert
            // Gültigen Messpunkt kopieren
          /*  this->radDist.push_back(dv);
            this->sqrtInt.push_back(iv);*/
            // Nächster Messpunkt in tyPOMSDATAHEADER
            // Nächster Messpunkt in tyHSPRESULT
            pn++;
        }
    }this->pointCount=pn;//std::cout<<"nbn: "<<nbN<<std::endl;
    // Anzahl*/

}

// Transformation eines Profils vom Typ tyHSPRESULT in kartesischen
// Koordinaten
void Datapackage::update(){
    // Berechnung der kartesichen Koordinaten eines Profils
    // innerhalb der Scanebene
   double HSPPOINTS = 3600;
   //  double HSPPOINTS = this->pointCount;
    //double x[HSPPOINTS]; // Horizontale Position [m]
    // double y[HSPPOINTS]; // Vertikale Position [m]
    // Ursprung der Polarkoordinaten in Schienenkoordinaten [m]
    double ox = double(this->originHor)*0.001;
    double oy = double(this->originVert)*0.001;
    // Winkelauflösung des Profils
    double step = 2.0*M_PI/HSPPOINTS;
    // int points = 0; // Anzahl der gültigen Messpunkte
    double angle = 0; // Winkel zur negativen Vertikalachse
    for (int i=0; i<HSPPOINTS; i++) {
        // Gültigkeit des Messpunktes abfragen
        if(i<this->sqrtInt.size()){
             if(this->sqrtInt.at(i) == 0){
                 quint16 n = (this->radDist.at(i));//std::cout<<"n: "<<n<<std::endl;
                 if(n==0) n=1;
                 angle += step;
             }else
            if(this->sqrtInt.at(i) > 0) {
                // Radius des Messpunktes [m]
                double r = double(this->radDist.at(i))*0.001;
                // Koordinatentransformation Polar -> Kartesisch
                x.push_back(ox + r*sin(angle));
                y.push_back(oy - r*cos(angle));
                //  points++;
                angle += step;// Winkel des nächsten Punktes
            }
        }
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


QVector<quint16> Datapackage::getDistance() const
{
    return distance;
}

void Datapackage::setDistance(const QVector<quint16> &value)
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
