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
 * @version 1.0
 */
#ifndef SCNDATA_H
#define SCNDATA_H
#include<QFile>
#include<QProgressDialog>
#include "datapackage.h"
/**
 * @class ScnData
 * @brief The ScnData class for load a scn file format
 * This class is used to read a scn file format and save into TXT file format.
 *
 * @details
 *
 * @section s1 How to read a SCN file format
 *
 * For read a SCN file format, you could use two possibilities.
 * The first possibility is of given the path and name of the file to reading in the constructor of the class.
 * Example:
 * @code
 * std::string pathname="file.scn";
 *   //create scn data
 *   ScnData data(pathname);
 *
 *   // get points loaded in a cloud
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 *   for(int j=0;j<data.getX().size();j++){
 *      cloud->push_back(pcl::PointXYZ(data.getX().at(j),
 *                                     data.getY().at(j),
 *                                     data.getZ().at(j)
 *                                    )
 *                      );
 *   }
 * @endcode
 *
 * The other possibility is to use loadFromSCN(std::string) function.
 * Example:
 * @code
 * std::string pathname="file.scn";
 *   //create scn data
 *   ScnData data;
 *   //load data from file
 *   data.loadFromSCN(pathname);
 *
 *   // get points loaded in a cloud
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 *   for(int j=0;j<data.getX().size();j++){
 *      cloud->push_back(pcl::PointXYZ(data.getX().at(j),
 *                                     data.getY().at(j),
 *                                     data.getZ().at(j)
 *                                    )
 *                      );
 *   }
 *
 *  Get all coordinates of all points of the SCN file read with using getX(), getY() and getZ()
 * @endcode
 */
class ScnData
{
public:
    ScnData();
    ScnData(std::string pathname);
    ~ScnData();

    /**
     * @brief loadFromSCN read a SCN file and get all points in the vectors x, y, z
     * @param pathname the path of the SCN file to be read
     */
    void  loadFromSCN(std::string pathname);

    QVector<Datapackage *> getPackages() const;
    int getIndexreader() const;
    QVector<double> getX() const;
    QVector<double> getY() const;
    QVector<double> getZ() const;

private:
    QVector<double>x;///< contains x coordinate of all points in the scn file read
    QVector<double>y;///< contains y coordinate of all points in the scn file read
    QVector<double>z;///< contains z coordinate of all points in the scn file read
    void fill();
    void assigningDatas();
    QVector<Datapackage*>packages;
    int indexreader;
    QByteArray alldatas;

};

#endif // SCNDATA_H
