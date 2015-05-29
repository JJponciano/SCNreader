/**
*  @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
* All rights reserved.
* This file is part of scn reader.
*
* scn_reader is free software: you can redistribute it and/or modify
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
* @version 1.0
*/
#include "datapackage.h"
Datapackage::Datapackage(QByteArray datas, int start)
{
    this->readData(datas,start);
}

Datapackage::Datapackage(const Datapackage &orig)
{
    this->x.clear();
    this->y.clear();
    this->footpulse=orig.getFootpulse();
    for(int i=0;i<orig.getX().size();i++){
        this->x.push_back(orig.getX().at(i));
        this->y.push_back(orig.getY().at(i));
    }
}
Datapackage::Datapackage()
{
}

Datapackage::~Datapackage()
{

}
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
        this->end=datas.size();
    }
    else{
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
        this->originHor =(unknownbytes2[33] << 8) + unknownbytes2[32];
        this->originVert =(unknownbytes2[35] << 8) + unknownbytes2[34];

        for(int i=0;i<this->pointCount;i++){
            ds >> byte1 >> byte0;
            unsigned short dist= (byte0 << 8) + byte1;
            this->distance.push_back(dist);
        }
        for(int i=0;i<this->pointCount;i++){
            ds >> byte1;
            QChar c=QChar(byte1);
            this->intensity.push_back(c);
        }
        this->end+=4+4+4+12+2+150+this->pointCount*3;// number of bytes previously readed
        this->update();
    }
}

void Datapackage::update(){
    // Berechnung der kartesichen Koordinaten eines Profils
    // innerhalb der Scanebene
    int  hspoint= 3600;
    // Ursprung der Polarkoordinaten in Schienenkoordinaten [m]
    double ox = double(this->originHor)*0.001;
    double oy = double(this->originVert)*0.001;
    // Winkelauflösung des Profils
    double step = 2*M_PI/(double)(hspoint);
    double angle = 0; // Winkel zur negativen Vertikalachse
    for (int i=0; i<hspoint; i++) {
        if(i<this->distance.size()){
            if(this->intensity.at(i)==0){
                angle+=step*this->distance.at(i);
            }else{
                if(this->distance.at(i)>1){
                    double r = double(this->distance.at(i));

                    r*=0.001;
                    double xt=ox + r*sin(angle);
                    double yt=oy - r*cos(angle);
                    x.push_back(xt);
                    y.push_back(yt);
                }
                angle += step;
            }
        }
    }
}

// -------------------------------- GETTER and SETTER --------------------------------
QVector<double> Datapackage::getY() const
{
    return y;
}
QVector<double> Datapackage::getX() const
{
    return x;
}
int Datapackage::getEnd() const
{
    return end;
}
int Datapackage::getPointCount() const
{
    return pointCount;
}
QString Datapackage::getVersion() const
{
    return version;
}
QString Datapackage::getId() const
{
    return id;
}

int Datapackage::getFootpulse() const
{
    return footpulse;
}

int Datapackage::getStructuresize() const
{
    return structuresize;
}
