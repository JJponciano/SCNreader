#ifndef SCNREADER_MODEL_H
#define SCNREADER_MODEL_H


#include "../modules/exceptions/erreur.h"
#include "scndata.h"
#include <QVector>
#include <QHash>
#include <QDataStream>

class scnreader_model: public ToolsPCL
{
public:
    scnreader_model();
    ~scnreader_model();

    void clear();

    /**
     * @brief scnreader_model::loadCloudFromTXT create and add data from file
     * @param pathname the file in txt format
     */
    void loadCloudFromTXT2(std::string pathname);

    /**
     * @brief addCloud load a cloud from a "txt" format file and add it in clouds
     * @param pathname path of the file for load cloud.
     */
    virtual void addCloudFromTXT(std::string pathname);


    /**
     * @brief scnreader_model::loadFromSCN create and add data from file
     * @param pathname the file in scn format
     */
    void  loadFromSCN(std::string pathname);

    /**
     * @brief scnreader_model::extractionCloud extract a part of cloud
     * @param d is the first footpulse of the cloud's part and f is the last footpulse of the cloud's part
     */
    void extractionCloud(int d, int f);
    /**
     * @brief scnreader_model::planar_segmentation segmentate a part of cloud
     * @param d is the first footpulse of the cloud's part and f is the last footpulse of the cloud's part
     */
    void planar_segmentation( int d, int f);

    //accesseur en lecture et en écriture
       int getFtpd();
       int getFtpf();

       void setFtpd(int d);
       void setFtpf(int f);

   ScnData getData(int i) const;
   void setDatas(const QVector<ScnData> &value);


   /**
    * @brief acces to QHash
    * @return the Qhash which contains all points footpulse by footpulse
    */
    QHash <int, QVector<pcl::PointXYZ *> *> getNuage();

    /**
     * @brief acces to QHash
     * @return the Qhash which contains all points of segmentations of a cloud's part
     */
     QHash <QString, QVector<pcl::PointXYZ*>*> getSegmentation();

     /**
      * @brief acces to QHash
      * @return the Qhash which contains all points of exractions of a cloud's part
      */
      QHash <QString, QVector<pcl::PointXYZ *>*> getExtraction();

private:
    /**
     * @brief readData read a data byte by byte
     * @param bytePosition posision of the data
     * @param length length of bytes of the data
     * @param pathname path of file data
     * @return the data to QString
     */
    QString readData(int bytePosition, int length, std::string pathname, bool isnumber);

    /**
     * @brief loadCloudFromTXT load a cloud from a "txt" format file
     * @param pathname path of the file for load cloud.
     */
    void loadCloudFromTXT(std::string pathname);

    /**
     * @brief IsFootpulse find what axe is the footpulse
     * @param pathname the file in txt format
     * @return int which is the number corresponding of axe (0=x, 1=y, 2=z)
     */
    int IsFootpulse(std::string pathname);

    /**
     * @brief scnreader_model::getPartInCloud transform the part of works in a cloud
     * @param d is the footpulse which begin the part
     * @param f is the footpulse which finish the part
     * @return the cloud corresponding to the part of cloud which we work on
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPartInCloud(int d, int f, QVector<int>* tailles);

    /**
     * @brief scnreader_model::getPtWithInd take the points correponding to indices which are given
     * @param d is the footpulse which begin the part
     * @param f is the footpulse which finish the part
     * @param ind is the vector of indices
     * @return the vector of points corresponding to indices
     */
    QVector<pcl::PointXYZ*>* getPtWithInd(int d, int f, std::vector<int> indices, QVector<int>* tailles);

    QVector <ScnData> datas;


    int ftpd;
    int ftpf;

    /**
     * @brief the Qhash which contains all points footpulse by footpulse
     *
     */
    QHash <int, QVector<pcl::PointXYZ *> *> nuage;
    /**
     * @brief the Qhash which contains all points of segmentations of a cloud's part
     *
     */
     QHash <QString, QVector<pcl::PointXYZ*>*> segmentation;
     /**
      * @brief the Qhash which contains all points of extractions of a cloud's part
      *
      */
     QHash <QString, QVector<pcl::PointXYZ *>*> extraction;
};

#endif // SCNREADER_MODULE_H
