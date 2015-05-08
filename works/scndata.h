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
#ifndef SCNDATA_H
#define SCNDATA_H

#include "datapackage.h"
#include "../modules/pcl/ground/ToolsPCL.h"
/**
 * @class ScnData
 * @brief The ScnData class for load a scn file format
 * This class is used to read a scn file format and save into TXT file format.
 *
 * @details
 *
 * @section s1 How to use
 *
 * @subsection ss1 How to read a SCN file format
 *
 * For read a SCN file format, you could use two possibilities.
 * The first possibility is of given the path and name of the file to reading in the constructor of the class.
 * Example:
 * @code
 * std::string pathname="file.scn";
 *    //create scn data
 *     ScnData data(pathname);
 *   //get  cloud
 *  this->clouds.push_back(data.getcloud());
 * @endcode
 *
 * The other possibility is to use loadFromSCN(std::string) function.
 * Example:
 * @code
 * std::string pathname="file.scn";
 *    //create scn data
 *     ScnData data;
 *    //load data from file
 *   data.loadFromSCN(pathname);
 *
 *    //get  cloud
 *  this->clouds.push_back(data.getcloud());
 * @endcode
 *
 * @subsection ss2 How to save a TXT file format
 *
 * You could save all packages of data readed from a SCN file or save package one by one.
 * @code
 *  //create scn data
 *   ScnData data;
 *   //load data from file
 *   data.loadFromSCN(pathname);
 *
 *   //save package one by one to txt file format
 *  for(int i=0;i<this->packages.size();i++){
 *   datas.saveToTXT(int i);
 * @endcode
 *  Or easy:
 * * @code
 *  //create scn data
 *   ScnData data;
 *   //load data from file
 *   data.loadFromSCN(pathname);
 *
 *   //save all packages to txt file format
 *   datas.save_all_toTXT();
 * @endcode
 *
 */
class ScnData
{
public:
    ScnData();
    ScnData(std::string pathname);
    ~ScnData();


    /**
     * @brief loadFromSCN
     * @param pathname the path of the load file
     * @return the cloud loaded
     */
    void  loadFromSCN(std::string pathname);

    /**
     * @brief save_all_toTXT save all packages in txt format file
     *
     * This functions create as many files as packets, in the current directory of this program
     */
    void save_all_toTXT()const;
    /**
     * @brief saveToTXT  save a package int txt format file
     * @param i the ith package to save
     */
    void saveToTXT(int i)const;

    /**
     * @brief getcloud get a cloud with all point  previously loaded from scn format file
     * @return  return a cloud of points
     *
     * @todo it has n*(m+1) complexity  with n is a number of packages inf scn file and m is
     *  the average number of point in each package. You can perhaps improve in n*m complexity
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getcloud();
    QVector<Datapackage> getPackages() const;
    void setPackages(const QVector<Datapackage> &value);

    QByteArray getAlldatas() const;
    void setAlldatas(const QByteArray &value);

    int getIndexreader() const;
    void setIndexreader(int value);


private:
    void assigningDatas();
    QVector<Datapackage>packages;
    QByteArray alldatas;
    int indexreader;

};

#endif // SCNDATA_H
