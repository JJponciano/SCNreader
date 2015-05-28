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
#define _USE_MATH_DEFINES
#include <QByteArray>
#include <QString>
#include <QDataStream>
#include <cmath>
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
 *       //update index reader after read one package
 *      this->indexreader=dp.getEnd();
 *   }
 *
 * @endcode
 */
class Datapackage
{
public:

    Datapackage();
    ~Datapackage();

    Datapackage(QByteArray datas,int start);
    Datapackage(const Datapackage& orig);

    // getter and setter
    QString getId() const;
    QString getVersion() const;
    int getPointCount() const;
    int getEnd() const;
    int getStructuresize() const;
    int getFootpulse() const;
    QVector<double> getX() const;
    QVector<double> getY() const;
private:
    /**
     * @brief readData
     * @param datas
     * @param start
     *
     * @todo it remain to read the coordinate origin
     */
    void readData(QByteArray datas, int start);
    void Datapackage::update();
    QString id;
    QString version;
    QVector<unsigned short> distance;
    QVector<QChar> intensity;
    QVector<double>x;
    QVector<double>y;
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
