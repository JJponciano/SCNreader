/**
 * @file ToolsPCL.h
 * @brief file to the managements of pcl
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
#ifndef MODEL_PCL_H
#define MODEL_PCL_H

#include "pcl/ModelCoefficients.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include "../../exceptions/erreur.h"
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QInputDialog>
#include <QIODevice>
#include <QFile>
#include <QMessageBox>
#include <QVector>
#include <QProgressDialog>
/**
 * @class ToolsPCL
 * @brief The ToolsPCL class  for manage pcl
 * This class is used to loading, reading and processing of point clouds.
 *
 * @details
 *
 * \subsection{How to use}
 *
 *
 * \subsection{Load cloud}
 *
 * \subsubsection{From pcd file format}
 *  if you want load a cloud from a pcd file format:
 * @code
 *   std::string pathname="myfile.pcd";
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloud(pathname.c_str());
 * @endcode
 *
 * \subsubsection{From TXT file format}
 *  if you want load a cloud from a TXT file format:
 * @code
 *   std::string pathname="myfile.TXT";
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloudFromTXT(pathname.c_str());
 * @endcode
 *
 * \subsection{Save cloud}
 *
 * \subsubsection{To pcd file format}
 *  if you want save a cloud to a pcd file format:
 * @code
 *    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 *   ToolsPCL::saveCloud(cloud,"myfile.pcd");
 * @endcode
 *
 * \subsubsection{To TXT file format}
 *  if you want save a cloud to a TXT file format:
 * @code
 *    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 *   ToolsPCL::saveCloudFromTXT(cloud,"myfile.TXT");
 * @endcode
 *
 *
 * \subsection{Processing}
 *
 * \subsubsection{Planar segmentation}
 * if you want to execute a planar segmentation:
 *  @code
 * std::string pathname="myfile.pcd";
 * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloud(pathname.c_str());
 * pcl::PointCloud<pcl::PointXYZ>::Ptr plan=this->planar_segmentation(cloud);
 * @endcode
 *
 * \subsubsection{Extraction cluster}
 * if you want to execute a extraction of different cluser in a cloud:
 *  @code
 * std::string pathname="myfile.pcd";
 * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloud(pathname.c_str());
 * QStringList cloudsName= this->extractionCloudInList(cloud);
 * @endcode
 * After, you can load all clusters created by using of loadCloud with each element in cloudsName.
 * Or you can use extractionCloud(int i).
 * See also exemple
 *
 * \subsection{Exemple}
 * These functions use a array of cloud ( "clouds" in parameters ).
 * You can access this array through getClouds(int i) or getClouds() functions.
 * You load a cloud and apply a extraction after a  planar segmentation.
 *
 * @code
 *   std::string pathname="myfile.pcd";
 *  // load cloud
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloud(pathname.c_str());
 * //apply planar segmentation
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr plan=this->planar_segmentation(cloud);
 * // save in file
 *  ToolsPCL::saveCloudFromTXT(plan,"myfile.TXT");
 * // add this plan in array of cloud
 *  ToolsPCL tools();
 * tools.addCloudFromTXT("myfile.TXT");
 * // apply extraction
 * tools.extractionCloud(0);
 * //get the original cloud and all cluster created
 * QVector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustersAndcloud= getClouds();// the original cloud is at position 0
 * // for fun, save all clouds int txt and pcd files formats
 *   saveCloudsFromTXT("allc.TXT");
 *   saveClouds("allc.pcd");
 *
 * @endcode
*/
class ToolsPCL
{
public:
    ToolsPCL();
    ToolsPCL( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    ~ToolsPCL();
    /**
     * @brief extractionCloud create new clouds representing different parts of the cloud located at the ith position in the clouds vector
     * @param i index of the cloud position in this Qvector clouds
     */
    void extractionCloud(int i);
    /**
     * @brief planarSegmentation  create a new cloud representing the most larger plan of the cloud located at the ith position in the clouds vector
     * @param i index of the cloud position in this Qvector clouds
     */
    void planarSegmentation(int i);
    /**
     * @brief addCloud load a cloud from a "pcd" format file and add it in clouds
     * @param pathname path of the file for load cloud.
     */
    void addCloud(std::string pathname);
    /**
     * @brief addCloud load a cloud from a "txt" format file and add it in clouds
     * @param pathname path of the file for load cloud.
     */
    void addCloudFromTXT(std::string pathname);
    /**
     * @brief saveCloudsFromTXT save all clouds in "txt" format file
     * @param pathname the file path
     */
    void saveCloudsFromTXT(std::string pathname);
    /**
     * @brief saveClouds save all clouds in "pcd" format file
     * @param pathname the file path
     */
    void saveClouds(std::string pathname);
    /**
     * @brief getClouds getter of clouds
     * @return a copy of all clouds
     */
    QVector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getClouds();
    /**
     * @brief getClouds get the cloud has the index i
     * @param i index of the clouds in class's vector. Start at 0.
     * @return the cloud located at index i of the vector
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getClouds(int i);

    /**
     * @brief ransac create a segmentation for finding the lines in cloud
     * @param cloud input cloud
     * @return cloud with the lines found
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief getCloudGray load a gray cloud from file which format is: X Y Z grayscale
     * @param pathname path of the file
     * @return rgb cloud with r=g=b
     */
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudGray(std::string pathname);
    /**
     * @brief loadCloudFromTXT load a cloud from a "txt" format file
     * @param pathname path of the file for load cloud.
     * @return  cloud
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloudFromTXT(std::string pathname);
    /**
     * @brief saveCloud save all clouds in "pcd" format file
     * @param cloud the cloud save
     * @param newName the path of the backup file
     */
    static void saveCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string newName);
    /**
     * @brief saveCloudFromTXT save all clouds in "txt" format file
     * @param cloud  the cloud save
     * @param pathname the path of the backup file
     */
    static void saveCloudFromTXT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string pathname);
    /**
     * @brief loadCloudRGB
     * @param pathname the path of the loqd file
     * @return   the cloud loaded
     */
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadCloudRGB(std::string pathname);
    /**
     * @brief loadCloud
     * @param pathname the path of the load file
     * @return   the cloud save loaded
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloud(std::string pathname);



    /**
     * @brief planar_segmentation create a new cloud representing the most larger plan of the cloud
     * @param cloud input cloud
     * @return A cloud representing the most larger plan of the input cloud in cloud
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segmentation( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    /**
     * @brief clear remove all clouds
     */
    void clear();
    float getMaxX() const;
    void setMaxX(float value);
    float getMaxY() const;
    void setMaxY(float value);
    float getMaxZ() const;
    void setMaxZ(float value);
protected:
    void searchMAX();
    float maxX;
    float maxY;
    float maxZ;
    // get point cloud from cloud file previously saved
    QVector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
private:

    QStringList extractionCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    QStringList extractionCloudInList(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};

#endif // TOOLSPCL_H
