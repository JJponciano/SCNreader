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
#include "ToolsPCL.h"

 
ToolsPCL::ToolsPCL( )
{
    this->maxY=1;
    this->maxX=1;
    this->maxZ=1;
}
ToolsPCL::ToolsPCL( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    this->maxY=1;
    this->maxX=1;
    this->maxZ=1;
    this->clouds.push_back(cloud);
    this->searchMAX();
}


ToolsPCL::~ToolsPCL()
{
   this->clouds.clear();
}

void ToolsPCL::clear(){
    this->clouds.clear();
}
void ToolsPCL::addCloud(std::string pathname){
    try{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloud(pathname.c_str());
        this->clouds.push_back(cloud);
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
     this->searchMAX();
}

void ToolsPCL::addCloudFromTXT(std::string pathname){
    try{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=ToolsPCL::loadCloudFromTXT(pathname.c_str());
        this->clouds.push_back(cloud);
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
     this->searchMAX();
}

void ToolsPCL::saveClouds(std::string pathname){
    try{
        QString name=QString::fromStdString(pathname);
        // for all the clouds
        for(int i=0;i<this->clouds.size();i++){

            //create full name of file
            QString full;
            int ind= name.lastIndexOf('.');
            //test if the indiex is ok
            if(ind>0&&ind<name.size())
                full=name.left(ind);
            else full=name;
            //get the index of the beginning to the file format
            full.append(QString::number(i));
            full.append(".pcd");
            //save cloud one by one
            ToolsPCL::saveCloud(this->clouds.at(i),full.toStdString());

        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}

void ToolsPCL::saveCloudsFromTXT(std::string pathname){
    try{
        QString name=QString::fromStdString(pathname);
        // for all the clouds
        for(int i=0;i<this->clouds.size();i++){
            //create full name of file
            QString full;
            int ind= name.lastIndexOf('.');
            //test if the indiex is ok
            if(ind>0&&ind<name.size())
                full=name.left(ind);
            else full=name;
            //get the index of the beginning to the file format
            full.append(QString::number(i));
            full.append(".txt");
            //save cloud one by one
            ToolsPCL::saveCloudFromTXT(this->clouds.at(i),full.toStdString());
        }
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr ToolsPCL::getClouds(int i){
    return this->clouds.at(i);
}


QVector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ToolsPCL::getClouds(){
    return this->clouds;
}

//----------------------Static function --------------
void ToolsPCL::saveCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string newName){

    cloud->is_dense = true;
    pcl::PCDWriter writer;
    std::stringstream ss;
    //definition of the title of the file
    ss << newName.c_str();
    //saving cloud to a file with newName file
    writer.write<pcl::PointXYZ> (ss.str(), *cloud, false);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr ToolsPCL::loadCloudFromTXT(std::string pathname){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    QFile fichier( QString(pathname.c_str()) );
    if(fichier.open(QIODevice::ReadOnly | QIODevice::Text)) // ce  si le fichier n'est pas ouvert
    {
        QTextStream flux(&fichier);
        QString  ligne; // variable contenant chaque ligne lue
        std::vector<float> px;
        std::vector<float> py;
        std::vector<float> pz;
        // number of line in the file
        int nline=10000000;
        int counter=0;
        // create progress dialog to inform the user of progress if the task has done is too long
        QProgressDialog progress("Loading cloud...", "Stop loading", 0, nline, 0);
        //said that the window is modal
        progress.setWindowModality(Qt::WindowModal);

        while(!flux.atEnd())
        {  //Increment the counter
            counter++;
            //update progress Dialog
            if(counter%(nline/100)==0)
                progress.setValue(counter);
            ligne= flux.readLine();
            // split the line with space as a separator character
            QStringList result =ligne.split("\t");
            //convert coordonated Qstring to float coordinates to add in vector
            //test if the first line define the number of lines into the file
            if( result.size()==1){
                nline=result.at(0).toInt();
                //update maximum of progress dialog
                progress.setMaximum(nline);
            }else
                if(result.size()<3)throw Erreur(" Error reading file!");
                else{
                    QString r=result.at(0);
                    px.push_back(r.toFloat());
                    r=result.at(1);
                    py.push_back(r.toFloat());
                    r=result.at(2);
                    pz.push_back(r.toFloat());
                }


            //if user want to stop loading, the reading is finished
            if (progress.wasCanceled())
                break;
        }
        //close automatically the progress dialog
        progress.setValue(nline);

        //close file
        fichier.close();
        // create cloud point and cloud file pcl

        // Fill in the cloud data
        cloud->width    = px.size();
        cloud->height   = 1;
        cloud->is_dense = false;
        cloud->points.resize (cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = px[i];
            cloud->points[i].y =py[i];
            cloud->points[i].z = pz[i];
        }
        return cloud;
    }else throw Erreur("the file "+pathname +"have not been opened!");

}

void ToolsPCL::saveCloudFromTXT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,std::string pathname){

    //open the file
    QFile file(QString::fromStdString(pathname));

    //Test if you can write into the file
    if(file.open(QIODevice::WriteOnly )){
        //if you can, initialize flu and line
        QTextStream flux(&file);
        //defines the codec of file
        flux.setCodec("UTF-8");
        //get coordinates  points of cloud and write it
        for(int i=0;i<cloud->points.size(); ++i){
            flux << QString::number(cloud->points[i].x) << " "
                 << QString::number(cloud->points[i].y) << " "
                 << QString::number(cloud->points[i].z) << " " << endl;
        }

        file.close();
    }else throw Erreur(" The file"+pathname+ "have not been saved, check the write permission!");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ToolsPCL::loadCloud(std::string pathname){
    //test if the file  exists
    if(!QFile::exists(pathname.c_str())){
        //if the file do not exists, throw exeption
        std::string se=" in ToolsPCL::loadCloud, error argument: "+pathname+" file do not exists";
        throw Erreur(se);
    }else{
        // Read in the cloud data  //-------------------------------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile (pathname, cloud_blob);
        pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
        return cloud;
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ToolsPCL::getCloudGray(std::string pathname){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // le constructeur de ifstream permet d'ouvrir un fichier en lecture
    std::ifstream fichier( pathname );

    if ( fichier ) // test if file is open
    {
        std::string ligne; // variable contenant chaque ligne lue
        std::vector<float> px;
        std::vector<float> py;
        std::vector<float> pz;
        std::vector<float> prgb;
        // continue as long as it remains a line reading in the file
        while ( std::getline( fichier, ligne ) )
        {
            // split the line with space as a separator character
            QStringList result = QString(ligne.c_str()).split(" ");
            //convert coordonated Qstring to float coordinates to add in vector
            QString r=result.at(0);
            px.push_back(r.toFloat());
            r=result.at(1);
            py.push_back(r.toFloat());
            r=result.at(2);
            pz.push_back(r.toFloat());
            r=result.at(3);
            prgb.push_back(r.toFloat());
        }
        // create cloud point and cloud file pcl

        // Fill in the cloud data
        cloud->width    = prgb.size();
        cloud->height   = 1;
        cloud->is_dense = false;
        cloud->points.resize (cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = px[i];
            cloud->points[i].y =py[i];
            cloud->points[i].z = pz[i];
            cloud->points[i].r = prgb[i];
            cloud->points[i].g =prgb[i];
            cloud->points[i].b = prgb[i];
        }

        return cloud;
    }else throw Erreur("the file "+pathname +"have not been opened!");

}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ToolsPCL::loadCloudRGB(std::string pathname){
    //test if the file  exists
    if(!QFile::exists(pathname.c_str())){
        //if the file do not exists, throw exeption
        std::string se=" in ToolsPCL::loadCloud, error argument: "+pathname+" file do not exists";
        throw Erreur(se);
    }else{
        // Read in the cloud data  //-------------------------------------------------
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCLPointCloud2 cloud_blob;
        pcl::io::loadPCDFile (pathname, cloud_blob);
        pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
        return cloud;
    }
}
void ToolsPCL::planarSegmentation(int i){
    if(i<0||i>this->clouds.size()) throw Erreur(" The parameter is incorrect!");
    else
        if( this->clouds.isEmpty())  throw Erreur(" The cloud was not found");
        else{
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=this->planar_segmentation( this->clouds[i]);
            this->clouds.push_back(cloud);
        }
}
float ToolsPCL::getMaxZ() const
{
    return maxZ;
}

void ToolsPCL::setMaxZ(float value)
{
    maxZ = value;
}

float ToolsPCL::getMaxY() const
{
    return maxY;
}

void ToolsPCL::setMaxY(float value)
{
    maxY = value;
}

float ToolsPCL::getMaxX() const
{
    return maxX;
}

void ToolsPCL::setMaxX(float value)
{
    maxX = value;
}


void ToolsPCL::extractionCloud(int i){
    if(i<0||i>this->clouds.size()) throw Erreur(" The parameter is incorrect!");
    else
        if( this->clouds.isEmpty())  throw Erreur(" The cloud was not found");
        else{

            /* this is OK and tested */
            // extract different plane from the point cloud and get pathname of the files creates
            QStringList cloudsName= this->extractionCloudInList(this->clouds[i]);
            //load all clouds created
            this->clouds.clear();
            for(int i=0;i<cloudsName.size();i++)
                this->addCloud(cloudsName.at(i).toStdString());
        }
}

//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------- PRIVATE FUNCTION -----------------------------------------------
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
void ToolsPCL::searchMAX(){
    //Research maximum coordinates to x, y and z.
    //for all clouds
    for(int j=0;j<this->clouds.size();j++)
        for (int i=0;i<clouds[j]->points.size();i++){
            //test is coordinates of current point are more than max previously defined
            if(clouds[j]->points[i].x>maxX)maxX=clouds[j]->points[i].x;
            if(clouds[j]->points[i].y>maxY)maxY=clouds[j]->points[i].y;
            if(clouds[j]->points[i].z>maxZ)maxZ=clouds[j]->points[i].z;
        }
}
QStringList ToolsPCL::extractionCloudInList(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    QStringList clusterCreates;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int  nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //*
        clusterCreates << ss.str().c_str();
        j++;
    }
    return clusterCreates;
}
QStringList ToolsPCL::extractionCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    QStringList clusterCreates;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
        clusterCreates << ss.str().c_str();
        j++;
    }
    return clusterCreates;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr ToolsPCL::planar_segmentation( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    //initialization
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    //definded coefficients of segmentation
    seg.segment (*inliers, *coefficients);

    //if we ave no indices, throw a error
    if (inliers->indices.size () == 0)
    {
        throw Erreur("Could not estimate a planar model for the given dataset.");
    }else{

        //create new cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // Fill in the cloud data

        /*  if you use second method after
         newCloud->width  = inliers->indices.size ();
         newCloud->height = 1;
         cloud->points.resize (newCloud->width * newCloud->height);
       */
        // Generate the data
        for (size_t i = 0; i < inliers->indices.size (); ++i)
        {
            newCloud->push_back(cloud->points[inliers->indices[i]]);
            /*    OR
           newCloud->points[i].x =cloud->points[inliers->indices[i]].x;
           newCloud->points[i].y = cloud->points[inliers->indices[i]].y;
           newCloud->points[i].z =cloud->points[inliers->indices[i]].z;
        */
        }
        //return  cloud
        return newCloud;
    }
}
