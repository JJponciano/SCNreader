/**
 * @file scndata.h
 * @brief file to manage scn format file
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
#ifndef DATAPACKAGE_H
#define DATAPACKAGE_H

#include <QByteArray>
#include <QString>
#include <QDataStream>
#include <math.h>
#include "../modules/exceptions/erreur.h"
#include "../modules/pcl/ground/ToolsPCL.h"
#include<QProgressDialog>
#include <QVector>

/**
 * @class Datapackage
 * @brief The Datapackage class containd only one package of point and header of this package of SCN file
 * This class is used to read only one package, in a array of bytes, of a SCN file format.
 *
 * @details
 *
 * @section s1 How to use
 *
 * @subsection ss1 How to read a package
 *
 * For read a package, you could use two possibilities.
 * The first possibility is of given the array and the index of reading in the constructor of the class.
 * Example:
 * @code
 * QFile fichier( QString(pathname.c_str()) );
 *  if(fichier.open(QIODevice::ReadOnly)){
 *      //read all bytes of the files
 *      QByteArray datas= fichier.readAll();
 *        //read block of byte and create a data package
 *      int indexreader=0;// or 288 for do not read the header of SCN file
 *      Datapackage dp(datas,indexreader);
 *       //update index reader after readed one package
 *      this->indexreader=dp.getEnd();
 *   }
 *
 * @endcode
 * Or you can also use readData(QByteArray datas, int start).
 * Example:
 * @code
 * Datapackage dp;
 * dp.readData(datas,indexreader);
 * @endcode
 *
 * @subsection ss2 Print or view package
 *
 * For print packages, you could use  toString()
 *
 */
class Datapackage
{
public:

    Datapackage();
    ~Datapackage();

    Datapackage(QByteArray datas,int start);
    /**
     * @brief readData
     * @param datas
     * @param start
     *
     * @todo it remain to read the coordinate origin
     */
    void readData(QByteArray datas, int start);
    void readDataFile(std::string pathname);


    friend std::ostream& operator << (std::ostream& O, const Datapackage& B);
    /**
     * @brief toString return the details of the values contained  in package
     * @return  the details of the values contained in package
     */
    std::string toString()const;
    virtual void Print(std::ostream& O) const;


    // getter and setter

    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud() const;
    void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &value);

    QString getId() const;
    void setId(const QString &value);

    QString getVersion() const;
    void setVersion(const QString &value);

    int getRadius() const;
    void setRadius(const int &value);

    int getRadienkorrektur() const;
    void setRadienkorrektur(const int &value);

    int getSpurweite() const;
    void setSpurweite(const int &value);

    int getNglLeftHorizontal() const;
    void setNglLeftHorizontal(const int &value);

    int getNglLeftVertical() const;
    void setNglLeftVertical(const int &value);

    int getNglLeftBanking() const;
    void setNglLeftBanking(const int &value);

    int getNglLeftGauge() const;
    void setNglLeftGauge(const int &value);

    int getNglRightHorizontal() const;
    void setNglRightHorizontal(const int &value);

    int getNglRightVertical() const;
    void setNglRightVertical(const int &value);

    int getNglRightBanking() const;
    void setNglRightBanking(const int &value);

    int getNglRightGauge() const;
    void setNglRightGauge(const int &value);

    int getPointCount() const;
    void setPointCount(const int &value);

    int getEnd() const;
    void setEnd(int value);

    int getStructuresize() const;
    void setStructuresize(int value);

    int getFootpulse() const;
    void setFootpulse(int value);

    QVector<unsigned short> getDistance() const;
    void setDistance(const QVector<unsigned short> &value);

    QVector<char> getIntensity() const;
    void setIntensity(const QVector<char> &value);

    QVector<double> getX() const;
    void setX(const QVector<double> &value);

    QVector<double> getY() const;
    void setY(const QVector<double> &value);

private:
    void update();
    /**
     * @brief decompression remove unuse point
     */
    void decompression() ;
 void read(QByteArray datas, int start);
    int bytesToInt(const char *buffer, int size);
    int bytesToInt(QByteArray arbuffer, int size, char *buffer);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    QString id;
    QString version;
    int radius;
    QVector<unsigned short> distance;
    QVector<quint8> intensity;
    QVector<unsigned short>radDist;
    QVector<quint8>sqrtInt;
    QVector<double>x;
    QVector<double>y;
    int radienkorrektur;
    int spurweite;
    int nglLeftHorizontal;
    int nglLeftVertical;
    int nglLeftBanking;
    int nglLeftGauge;
    int nglRightHorizontal;
    int nglRightVertical;
    int nglRightBanking;
    int nglRightGauge;
    int pointCount;
    int end;
    int structuresize;
    int footpulse;
    int originHor;
    int originVert;
    qint8 header[288];
    qint8 unknownbytes[12];
    qint8 unknownbytes2[150];
};

#endif // DATAPACKAGE_H
