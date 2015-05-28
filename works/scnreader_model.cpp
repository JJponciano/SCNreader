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
#include "scnreader_model.h"

scnreader_model::scnreader_model():ToolsPCL()
{

    this->workWindows=500;
    this->ftpd=0;
    this->ftpf=0;
    ListeRail lr(workWindows);
    this->lesRails=lr;
    this->nomFile="";
    this->cfs=false;
    //constant value
    this->capacity=10000000;
    this->RansacVide=true;
}

scnreader_model::~scnreader_model()
{
    this->nuage.clear();
    this->segmentation.clear();
    this->extraction.clear();
}
/*
pcl::PointCloud<pcl::PointXYZ>::Ptr scnreader_model::getRails(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

}*/

QString scnreader_model::readData(int bytePosition, int length,std::string pathname,bool isnumber){
    std::ifstream ouverture(pathname.c_str(),std::ios::in);
    //the data to load
    QString temp;
    int intemp=0;
    //byte
    char offset0;
    //test if file is open
    if(ouverture)
    {
        //go to the start data
        ouverture.seekg(bytePosition,std::ios::beg);
        for(int i=0;i<length;i++){
            //read byte
            ouverture.get(offset0);
            //go to next byte
            ouverture.seekg(i,std::ios::beg);
            //add byte into data
            temp.append(offset0);
            if(isnumber)
                intemp<<offset0;
        }
    }
    ouverture.close();
    if(isnumber)
        temp=QString::number(intemp);
    return temp;
}

ScnData scnreader_model::getData(int i) const{
    return datas.at(i);
}

void scnreader_model::setDatas(const QVector<ScnData> &value)
{
    datas = value;}


void scnreader_model::loadFromSCN(std::string pathname){
    //create scn data
    ScnData data;
    //load data from file
    data.loadFromSCN(pathname);
    //add this data created
    this->datas.push_back(data);

}


void scnreader_model::addCloudFromTXT(std::string pathname){
    try{
        scnreader_model::loadCloudFromTXT2(pathname.c_str());
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
    this->searchMAX();
}


void scnreader_model::loadCloudFromTXT2(std::string pathname){

    int footpulse=IsFootpulse(pathname);
    bool premier=true;

    QFile fichier( QString(pathname.c_str()) );
    if(fichier.open(QIODevice::ReadOnly | QIODevice::Text)) // ce  si le fichier n'est pas ouvert
    {
        QTextStream flux(&fichier);
        QString  ligne; // variable contenant chaque ligne lue
        // number of line in the file
        int nline=10000000;
        int counter=0;
        // create progress dialog to inform the user of progress if the task has done is too long
        QProgressDialog progress("Loading cloud...", "Stop loading", 0, nline, 0);
        //said that the window is modal
        progress.setWindowModality(Qt::WindowModal);

        PointGL * p;
        int ftpcourant=-1;
        QVector<PointGL *>* v;

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
                    QString x=result.at(0);
                    QString y=result.at(1);
                    QString z=result.at(2);
                    if(footpulse==0)
                    {
                        p=new PointGL(z.toFloat(),y.toFloat(),x.toFloat());
                    }
                    else if(footpulse==1)
                    {
                        p=new PointGL(x.toFloat(),z.toFloat(),y.toFloat());
                    }
                    else
                    {
                        p=new PointGL(x.toFloat(),y.toFloat(),z.toFloat());
                    }

                    //if the footpulse is not initialized or if he is different of the previous
                    if(ftpcourant==-1 || ftpcourant!=(int) p->getZ())
                    {
                        //we update footpulse
                        ftpcourant=(int) p->getZ();
                        //we create a new vector
                        v= new QVector<PointGL *>();
                        //which we add to the hashtable
                        nuage.insert(ftpcourant, v);
                        //then we add the new point
                        v->push_back(p);
                    }
                    else
                        v->push_back(p);//else we add the new point

                    if(premier){
                        this->ftpd=p->getZ();
                        premier=false;
                    }
                    this->ftpf=p->getZ();
                }


            //if user want to stop loading, the reading is finished
            if (progress.wasCanceled())
                break;
        }
        //close automatically the progress dialog
        progress.setValue(nline);

        //close file
        fichier.close();

        //create tracks
        createRail();

        // return cloud;
    }else throw Erreur("the file "+pathname +"have not been opened!");

}


int scnreader_model::IsFootpulse(std::string pathname){
    //__________declare variables_________________
    //count
    int cx=0;
    int cy=0;
    int cz=0;
    //boolean of end
    bool find=false;
    //boolean corresponding of each axe
    bool fx=(cy>=2 && cz>=2);
    bool fy=(cx>=2 && cz>=2);
    bool fz=(cy>=2 && cx>=2);
    //previous value
    float tx,ty,tz;
    //current value
    float vx,vy,vz;
    //file
    QFile fichier( QString(pathname.c_str()) );


    // si le fichier est bien ouvert
    if(fichier.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream flux(&fichier);
        //variable contenant chaque ligne lue
        QString  ligne;

        //read the first line
        ligne= flux.readLine();
        // split the line with space as a separator character
        QStringList result1 =ligne.split("\t");

        //test if the first line define the number of lines into the file
        if( result1.size()==1){
            //read the second line
            ligne= flux.readLine();
            // split the line with space as a separator character
            result1 =ligne.split("\t");
            QString x=result1.at(0);
            QString y=result1.at(1);
            QString z=result1.at(2);
            //initialize value
            tx=x.toFloat();
            ty=y.toFloat();
            tz=z.toFloat();
        }else if(result1.size()<3)throw Erreur(" Error reading file!");
        else{
            QString x=result1.at(0);
            QString y=result1.at(1);
            QString z=result1.at(2);
            //previous value
            tx=x.toFloat();
            ty=y.toFloat();
            tz=z.toFloat();
        }

        //__________Cover of file's part_______________
        while(!flux.atEnd() && !find)
        {
            //__________read the x, y, z

            //read line by line
            ligne= flux.readLine();
            // split the line with space as a separator character
            QStringList result =ligne.split("\t");
            //convert coordonated Qstring to float coordinates to compare

            //test if the first line define the number of lines into the file
            if(result.size()<3)throw Erreur(" Error reading file!");
            else{
                QString x=result.at(0);
                QString y=result.at(1);
                QString z=result.at(2);
                //convert coordonated Qstring to float coordinates to compare
                vx=x.toFloat();
                vy=y.toFloat();
                vz=z.toFloat();

                //compare the current and the previous value
                if(cx<2)
                    if(tx!=vx)
                    {
                        cx++;
                    }
                if(cy<2)
                    if(ty!=vy)
                    {
                        cy++;
                    }
                if(cz<2)
                    if(tz!=vz)
                    {
                        cz++;
                    }

                //update the future previous value
                tx=vx;
                ty=vy;
                tz=vz;
            }

            //update of booleans
            fx=(cy>=2 && cz>=2);
            fy=(cx>=2 && cz>=2);
            fz=(cy>=2 && cx>=2);
            find= fx || fy || fz;
        }

        //close file
        fichier.close();
        // create cloud point and cloud file pcl

    }else throw Erreur("the file "+pathname +"have not been opened!");


    //return the number corresponding of the good axe
    return (0*fx+1*fy+2*fz);
}


//void scnreader_model::extractionCloud(int d, int f) {
//    /*
//    if(nuage.contains(d) && nuage.contains(f))
//    {
//        //---------------------Create the cluster to do the extraction-------------------
//        //vector wich contains the count of point by footpulse
//        QVector<int> tailles;
//        //temporarly cloud to do the segmentation
//        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
//        //Fill the cloud and the vector tailles
//        int i=d;
//        while(i<=f)
//        {
//            if(nuage.contains(i))
//            {
//                QVector <pcl::PointXYZ*> * v=nuage.value(i);
//                tailles.push_back(v->size());
//                for(int j=0; j<v->size(); j++)
//                {
//                    pcl::PointXYZ* p=v->at(j);
//                    CloudTemp->points.push_back(* p);
//                }
//                i++;
//            }
//            else throw Erreur("Interval de footpulses discontinu dans la segmentation");
//        }
//        CloudTemp->width = CloudTemp->points.size();
//        CloudTemp->height = 1;
//        CloudTemp->is_dense = false;
//        CloudTemp->points.resize (CloudTemp->width * CloudTemp->height);
////------------------------------------Extraction---------------------------------------------------
//        QStringList clusterCreates;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//        // Create the filtering object: downsample the dataset using a leaf size of 1cm
//        pcl::VoxelGrid<pcl::PointXYZ> vg;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//        vg.setInputCloud (CloudTemp);
//        vg.setLeafSize (0.01f, 0.01f, 0.01f);
//        vg.filter (*cloud_filtered);

//        // Create the segmentation object for the planar model and set all the parameters
//        pcl::SACSegmentation<pcl::PointXYZ> seg;
//        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//        pcl::PCDWriter writer;
//        seg.setOptimizeCoefficients (true);
//        seg.setModelType (pcl::SACMODEL_PLANE);
//        seg.setMethodType (pcl::SAC_RANSAC);
//        seg.setMaxIterations (100);
//        seg.setDistanceThreshold (0.02);

//        int  nr_points = (int) cloud_filtered->points.size ();
//        while (cloud_filtered->points.size () > 0.3 * nr_points)
//        {
//            // Segment the largest planar component from the remaining cloud
//            seg.setInputCloud (cloud_filtered);
//            seg.segment (*inliers, *coefficients);
//            if (inliers->indices.size () == 0)
//            {
//                break;
//            }

//            // Extract the planar inliers from the input cloud
//            pcl::ExtractIndices<pcl::PointXYZ> extract;
//            extract.setInputCloud (cloud_filtered);
//            extract.setIndices (inliers);
//            extract.setNegative (false);

//            // Get the points associated with the planar surface
//            extract.filter (*cloud_plane);

//            // Remove the planar inliers, extract the rest
//            extract.setNegative (true);
//            extract.filter (*cloud_f);
//            *cloud_filtered = *cloud_f;
//        }

//        // Creating the KdTree object for the search method of the extraction
//        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//        tree->setInputCloud (cloud_filtered);

//        std::vector<pcl::PointIndices> cluster_indices;
//        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//        ec.setClusterTolerance (0.02); // 2cm
//        ec.setMinClusterSize (100);
//        ec.setMaxClusterSize (25000);
//        ec.setSearchMethod (tree);
//        ec.setInputCloud (cloud_filtered);
//        ec.extract (cluster_indices);

//        int j = 0;
//        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//        {
//            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

//            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //

//            cloud_cluster->width = cloud_cluster->points.size ();
//            cloud_cluster->height = 1;
//            cloud_cluster->is_dense = true;

//            std::stringstream ss;
//            ss << "cloud_cluster_" << j << ".pcd";
//            writer.write<pcl::PointXYZ> (ss.str(), *cloud_cluster, false); //
//            clusterCreates << ss.str().c_str();
//            j++;

//             *  //create new cloud
//            QVector<pcl::PointXYZ*>* v= new QVector<pcl::PointXYZ*>();
//            // Fill in the cloud data

//            // Generate the data
//            for (size_t i = 0; i < inliers->indices.size (); ++i)
//            {
//                //vector which contains points corresponding to a footpulse
//                QVector<pcl::PointXYZ*>* vec;
//                //indice of point which is contained in segmentation
//                int indiceC=inliers->indices[i];
//                //indice to know what vector to use
//                int ind=0;
//                int nb;
//                //sum of size of previous vectors
//                if(ind<tailles.size())
//                {
//                    nb=tailles.at(ind);
//                }
//                else
//                {
//                    std::stringstream ss;
//                    ss << "Erreur dans TAILLE pas initialise, pour l'emplacement "<<ind<< " car vecteur de taille: "<<tailles.size();
//                    std::string message=ss.str();
//                    throw Erreur(message);
//                }
//                //int nb=tailles.at(ind);
//                //we grow up the ind to find the good vector where the point find
//                while(ind<(f-d) && indiceC>=nb)
//                {
//                    ind++;
//                    if(ind<tailles.size())
//                    {
//                        nb+=tailles.at(ind);
//                    }
//                    else
//                    {
//                        std::stringstream ss;
//                        ss << "Erreur dans TAILLE pour l'emplacement "<<ind<< " car vecteur de taille: "<<tailles.size();
//                        std::string message=ss.str();
//                        throw Erreur(message);
//                    }
//                   // nb+=tailles.at(ind);
//                }
//                //we keep the vector corresponding
//                vec=nuage.value(d+ind);
//                //we add the pointer of the corresponding point
//                if(ind>0)
//                {

//                    int num=indiceC-nb+tailles.at(ind);
//                    std::cout<<indiceC<<","<<nb<< "," << num<<std::endl;
//                    if(num<vec->size())
//                    {
//                        v->push_back(vec->at(num));
//                    }
//                    else
//                    {
//                        std::stringstream ss;
//                        ss << "Erreur dans VEC pour l'emplacement "<<num<< " car vecteur de taille: "<<vec->size();
//                        std::string message=ss.str();
//                        throw Erreur(message);
//                    }
//                }
//                else
//                    v->push_back(vec->at(indiceC));
//c
//            }
//            //add the segmentation to the hashtable
//            std::stringstream ss;
//            ss << "S_" << d <<"_" << f;
//            segmentation.insert(QString::fromStdString (ss.str()), v);
//        }
//        }
//        //return clusterCreates;
//    }
//    else throw Erreur("Interval de footpulses incorrect pour effectuer l'extraction");*/
//}



void scnreader_model::planar_segmentation( int d, int f){
    if(nuage.contains(d) && nuage.contains(f))
    {
        //initialization
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        //vector wich contains the count of point by footpulse
        QVector<int>* tailles= new QVector<int>();
        //temporarly cloud to do the segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTemp=getPartInCloud(d,f, tailles);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (CloudTemp);
        //definded coefficients of segmentation
        seg.segment (*inliers, *coefficients);

        //if we ave no indices, throw a error
        if (inliers->indices.size () == 0)
        {
            throw Erreur("Could not estimate a planar model for the given dataset.");
        }else{

            //create new cloud
            QVector<PointGL*>* v= getPtWithInd(d, f,inliers->indices, tailles);

            //add the segmentation to the hashtable
            std::stringstream ss;
            ss << "S_" << d <<"_" << f;
            segmentation.insert(QString::fromStdString (ss.str()), v);
        }
    }
    else throw Erreur("Interval de footpulses incorrect pour effectuer la segmentation");
}

QHash <int, QVector<PointGL *> *> scnreader_model::getNuage(){
    return this->nuage;
}

QHash <QString, QVector<PointGL*>*> scnreader_model::getSegmentation(){
    return this->segmentation;
}


//QHash <QString, QVector<pcl::PointXYZ *>*> scnreader_model::getExtraction(){
//    return this->extraction;
//}

void scnreader_model::setFtpd(int d){
    this->ftpd=d;
}

void scnreader_model::setFtpf(int f){
    this->ftpf=f;
}

int scnreader_model::getFtpd(){
    return this->ftpd;
}

int scnreader_model::getFtpf(){
    return this->ftpf;
}

void scnreader_model::clear()
{
    this->lesRails.clear();
    this->resultRANSAC;
    this->nuage.clear();
    this->segmentation.clear();
    this->extraction.clear();
    this->LesSwitchs.clear();
    this->RansacVide=true;
    this->lesRailsOptimize.clear();
    this->clouds.clear();
    this->datas.clear();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr scnreader_model::getPartInCloud(int d, int f, QVector<int>* tailles)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
    //Fill the cloud and the vector tailles
    int i=d;
    while(i<=f)
    {
        if(nuage.contains(i))
        {
            QVector <PointGL*> * v=nuage.value(i);
            tailles->push_back(v->size());
            for(int j=0; j<v->size(); j++)
            {
                pcl::PointXYZ p(v->at(j)->getX(),v->at(j)->getY(),v->at(j)->getZ());
                CloudTemp->points.push_back(p);
            }
            i++;
        }
        else throw Erreur("Interval de footpulses discontinu dans la segmentation");
    }
    CloudTemp->width = CloudTemp->points.size();
    CloudTemp->height = 1;
    CloudTemp->is_dense = false;
    CloudTemp->points.resize (CloudTemp->width * CloudTemp->height);
    return CloudTemp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr scnreader_model::getVectInCloud(QVector<PointGL *> vecteur)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
    //Fill the cloud with the points which are in the vector
    for(int i=0; i< vecteur.size(); i++)
    {
        float x=vecteur.at(i)->getX();
        float y=vecteur.at(i)->getY();
        float z=vecteur.at(i)->getZ();
        pcl::PointXYZ p(x,y,z);
        CloudTemp->points.push_back(p);
    }
    //update of cloud
    CloudTemp->width = CloudTemp->points.size();
    CloudTemp->height = 1;
    CloudTemp->is_dense = false;
    CloudTemp->points.resize (CloudTemp->width * CloudTemp->height);
    return CloudTemp;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr scnreader_model::getVectInCloud(QVector<PointGL > vecteur)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
    //Fill the cloud with the points which are in the vector
    for(int i=0; i< vecteur.size(); i++)
    {
        pcl::PointXYZ p;
        p.x=vecteur.at(i).getX();
        p.y=vecteur.at(i).getY();
        p.z=vecteur.at(i).getZ();
        CloudTemp->points.push_back(p);
    }
    //update of cloud
    CloudTemp->width = CloudTemp->points.size();
    CloudTemp->height = 1;
    CloudTemp->is_dense = false;
    CloudTemp->points.resize (CloudTemp->width * CloudTemp->height);
    return CloudTemp;
}


QVector<PointGL *> scnreader_model::getCloudInVect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    QVector<PointGL *> vecteur;
    //Fill the vector with the points which are in the cloud
    for(int i=0; i< cloud->points.size(); i++)
    {
        float x=cloud->points.at(i).x;
        float y=cloud->points.at(i).y;
        float z=cloud->points.at(i).z;
        PointGL *p=new PointGL(x,y,z);
        vecteur.push_back(p);
    }

    return vecteur;
}
QVector<PointGL> scnreader_model::getCloudInVectpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    QVector<PointGL> vecteur;
    //Fill the vector with the points which are in the cloud
    for(int i=0; i< cloud->points.size(); i++)
    {
        float x=cloud->points.at(i).x;
        float y=cloud->points.at(i).y;
        float z=cloud->points.at(i).z;
        PointGL p(x,y,z);
        vecteur.push_back(p);
    }

    return vecteur;
}

QVector<PointGL *>* scnreader_model::getPtWithInd(int d, int f, std::vector<int> indices, QVector<int>* tailles)
{
    //create new cloud
    QVector<PointGL*>* v= new QVector<PointGL*>();
    // Fill in the cloud data

    // Generate the data
    for (size_t i = 0; i < indices.size (); ++i)
    {
        //vector which contains points corresponding to a footpulse
        QVector<PointGL*>* vec;
        //indice of point which is contained in segmentation
        int indiceC=indices[i];
        //indice to know what vector to use
        int ind=0;
        int nb;
        //sum of size of previous vectors
        if(ind<tailles->size())
        {
            nb=tailles->at(ind);
        }
        else
        {
            std::stringstream ss;
            ss << "Erreur dans TAILLE pas initialise, pour l'emplacement "<<ind<< " car vecteur de taille: "<<tailles->size();
            std::string message=ss.str();
            throw Erreur(message);
        }
        //int nb=tailles->at(ind);
        //we grow up the ind to find the good vector where the point find
        while(ind<(f-d) && indiceC>=nb)
        {
            ind++;
            if(ind<tailles->size())
            {
                nb+=tailles->at(ind);
            }
            else
            {
                std::stringstream ss;
                ss << "Erreur dans TAILLE pour l'emplacement "<<ind<< " car vecteur de taille: "<<tailles->size();
                std::string message=ss.str();
                throw Erreur(message);
            }
            // nb+=tailles->at(ind);
        }
        //we keep the vector corresponding
        vec=nuage.value(d+ind);
        //we add the pointer of the corresponding point
        if(ind>0)
        {

            int num=indiceC-nb+tailles->at(ind);

            if(num<vec->size())
            {
                v->push_back(vec->at(num));
            }
            else
            {
                std::stringstream ss;
                ss << "Erreur dans VEC pour l'emplacement "<<num<< " car vecteur de taille: "<<vec->size();
                std::string message=ss.str();
                throw Erreur(message);
            }
        }
        else
            v->push_back(vec->at(indiceC));

    }
    return v;
}
QString scnreader_model::getNomFile() const
{
    return nomFile;
}

void scnreader_model::setNomFile(const QString &value)
{
    nomFile = value;
}
int scnreader_model::getCapacity() const
{
    return capacity;
}

void scnreader_model::setCapacity(int c)
{
    throw Erreur("The capacity must not be modified");
}

QVector<int> scnreader_model::getLesSwitchs() const
{
    return LesSwitchs;
}

void scnreader_model::setLesSwitchs(const QVector<int> &value)
{
    LesSwitchs = value;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr scnreader_model::getResultRANSAC() const
{
    return resultRANSAC;
}

void scnreader_model::setResultRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr &value)
{
    resultRANSAC = value;
}

ListeRail scnreader_model::getLesRailsOptimize() const
{
    return lesRailsOptimize;
}

void scnreader_model::setLesRailsOptimize(const ListeRail &value)
{
    lesRailsOptimize = value;
}

ListeRail scnreader_model::getLesRails() const
{
    return lesRails;
}



QVector<PointGL *> *scnreader_model::getCloudInVect2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp)
{
    /*QVector<pcl::PointXYZ *> * v;
    //TODO
    return v;*/
    throw Erreur("NOT YET IMPLEMENT, SORRY!");
}

bool scnreader_model::samePoint(PointGL *point2, PointGL *ptP)
{
    //we take the coordinates of ptP
    float x1=ptP->getX();
    float y1=ptP->getY();
    float z1=ptP->getZ();

    //we take the coordinates of pt
    float x2=point2->getX();
    float y2=point2->getY();
    float z2=point2->getZ();

    return (x1==x2) && (y1==y2) && (z1==z2);
}

void scnreader_model::createRail()
{
    QString noms=this->nomFile;
    noms.push_back("_switch.txt");
    //open the file
    QFile file(noms);

    if(!this->nuage.isEmpty())
    {
        if(this->cfs || !file.exists())
        {
            //---------------initialize footpulses which determine the beginning and the end of window-----------
            int dw=this->ftpd+1;
            int fw;
            if(this->ftpf-this->ftpd<workWindows)
                fw=this->ftpf;
            else
                fw=this->ftpd+workWindows;
            std::cout << dw << " - " << fw<<std::endl;
            //-------------------------------we initialize Listerail----------------------------------------------
            //create rails with the first footpulse
            RailCluster  r (0.18,0.08,1.5,*this->nuage.value(this->ftpd));
            RailCluster rc=r;
            this->lesRails.addRail(r);
            //and we add the others until the footpulse fw, so window is ftpd - (fw-1)
            for(int i=dw; i<fw;i++)
            {
                RailCluster r2(0.18,0.08,1.5,* (this->nuage.value(i)), rc);
                rc=r2;
                this->lesRails.addRail(r2);
            }
            cleanNoise(fw);
//            this->optimization();
//            //we keep detected switch in this window
//            int nbswitch=this->lesRailsOptimize.getSwitchDetected().size();
//            for(int i=0; i<nbswitch;i++)
//            {
//                int ftp=this->lesRailsOptimize.getSwitchDetected().at(i);
//                if(!this->LesSwitchs.contains(ftp))
//                {
//                    this->LesSwitchs.push_back(ftp);
//                }
//            }
//            //we reinit lesRailsOptimize et resultRansac
//                                    this->lesRailsOptimize.clear();
//                                    this->resultRANSAC->clear();

//                                    //we continue to cover all the cloud with a window which we move footpulse by footpulse
//                                    while(fw<=this->ftpf)
//                                    {
//                                        //we add a new track and remove the first in track in window
//                                        RailCluster r2(0.18,0.08,1.5,* (this->nuage.value(fw)), rc);
//                                        rc=r2;
//                                        this->lesRails.addRail(r2);

//                                        //we do the treatment to detect switchs in this window
//                                        this->optimization();
//                                        //we keep detected switch in this window
//                                        nbswitch=this->lesRailsOptimize.getSwitchDetected().size();
//                                        for(int i=0; i<nbswitch;i++)
//                                        {
//                                            int ftp=this->lesRailsOptimize.getSwitchDetected().at(i);
//                                            //we verify that the size of vector doesn't exceed the cvector's capacity
//                                            if(this->LesSwitchs.size()<this->capacity)
//                                            {
//                                                if(!this->LesSwitchs.contains(ftp))
//                                                    this->LesSwitchs.push_back(ftp);
//                                            }
//                                            //if it exceeds
//                                            else
//                                            {
//                                                //----------------we write footpulses in a text file

//                                                    VideEtEnregistre(noms);
//                                            }

//                                        }

//                                        //we reinit lesRailsOptimize et resultRansac
//                                        //this->lesRailsOptimize.clear();
//                                        //this->resultRANSAC->clear();

//                                        //we move the window
//                                        dw++;
//                                        fw++;
//                                    }

//            //----------------we write footpulses of switch in a text file
//            this->enregistre(noms);
        }
    }
    else throw Erreur("Les rails n'ont pas pu etre crees car le nuage de points est vide.");
}

void scnreader_model::optimization()
{
    this->RansacVide=false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=this->getVectInCloud(this->lesRails.getCloud());
    std::cout<<"check: "<<cloud->points.size()<<std::endl;
    this->resultRANSAC=this->ransac(cloud);
    //this->resultRANSAC=this->ransac(resultRANSAC);

    ListeRail lr(this->getCloudInVectpoint(this->resultRANSAC),workWindows);
    this->lesRailsOptimize=lr;
}

void scnreader_model::enregistre(QString noms)
{
    QFile fichier(noms);

    if(fichier.open(QIODevice::Append | QIODevice::Text)){
        //if you can, initialize flu and line
        QTextStream flux(&fichier);
        //defines the codec of file
        flux.setCodec("UTF-8");
        //get footpulse of switch
        for(int i=0;i<this->LesSwitchs.size(); i++){
            int sw=this->LesSwitchs.at(i);
            flux << QString::number(sw) << " " << endl;
        }
        fichier.close();
    }
    else throw Erreur(" The file finale switch.txt have not been saved, check the write permission!");

}

void scnreader_model::VideEtEnregistre(QString noms)
{
    QFile fichier(noms);

    //Test if you can write into the file
    if(fichier.open(QIODevice::Append | QIODevice::Text )){
        //if you can, initialize flu and line
        QTextStream flux(&fichier);
        //defines the codec of file
        flux.setCodec("UTF-8");
        //get footpulse of switch
        int nbToWrite=this->LesSwitchs.size()-this->workWindows;
        for(int i=0;i<nbToWrite; i++){
            int sw=this->LesSwitchs.takeFirst();
            flux << QString::number(sw)<< endl;
        }
        fichier.close();
    }else throw Erreur(" The file switch.txt have not been saved, check the write permission!");

}
bool scnreader_model::getRansacVide() const
{
    return RansacVide;
}

void scnreader_model::setRansacVide(bool value)
{
    RansacVide = value;
}

void scnreader_model::SavePartInTxt(int d, int f, QString pathname)
{
    //open the file
    QFile file(pathname);

    //If there is an error
    QString MesErreur=" The file";
    MesErreur.push_back(pathname);
    MesErreur.push_back("have not been saved, check the write permission!");

    //Test if you can write into the file
    if(file.open(QIODevice::WriteOnly )){
        //if you can, initialize flu and line
        QTextStream flux(&file);
        //defines the codec of file
        flux.setCodec("UTF-8");
        //get coordinates  points of cloud and write it
        for(int i=d;i<=f; i++){
            if(this->nuage.contains(i))
            {
                QVector<PointGL *> * vec=this->nuage.value(i);
                for(int j=0;j<vec->size(); j++){
                    flux << QString::number(vec->at(j)->getX()) << "\t"
                         << QString::number(vec->at(j)->getY()) << "\t"
                         << QString::number(vec->at(j)->getZ()) << "\t" << endl;
                }
            }
        }

        file.close();
    }else throw Erreur(MesErreur.toStdString());
}

//For each part of cloud:
void scnreader_model::cleanNoise(int f){
    //we fix the width of 1 pixel
    float dense=0.04;
    //we keep the cloud which we must clean
    QVector<PointGL> lespoints=this->lesRails.getCloud();
    //we keep distance between xmin and xmax
    float* val=distanceMinMax(lespoints);
    float dist=val[1]-val[0];
    float xmin=val[0];
    int zmin=val[2];
    //calculate width of image
    int width=(int)(dist/dense);
    //initialize the image
    ImageProcessing im(width+1, (f-this->ftpd));
    //For each point of this part:
    for(int i=0; i<lespoints.size(); i++)
    {
        //we calculate the coordinate corresponding to it
        int c=(int) ((lespoints.at(i).getX()-xmin)/dense);
        int l=(int) lespoints.at(i).getZ()-zmin;
        std::cout<<"l: "<<l<<" et c: "<<c<< std::endl;
        //we increase this position
        im.increase(l,c);
    }
    //we do a calibration of gray level between 0 and 255
    im.calibration();

    //we do a thresholding with threshold=125 => binarization
    im.thresholding(0);
    im.enregistre(this->nomFile);
    //For each point of this part:
    for(int i=0; i<lespoints.size(); i++)
    {
        //we calculate the coordinate corresponding to it
        int c=(int) ((lespoints.at(i).getX()-xmin)/dense);
        int l=(int) lespoints.at(i).getZ()-zmin;
        //if the value of position equals 0, we remove this point
        if(im.getValue(l,c)==0)
        {
            lespoints.removeAt(i);
            //update after remove
            i--;
        }
    }
    this->lesRails.initialization(lespoints,this->workWindows);
}

float * scnreader_model::distanceMinMax(QVector<PointGL> lspts)
{
    //initialization if xmin and xmax
    float xmin=lspts.at(0).getX();
    float xmax=lspts.at(0).getX();
    float zmin=lspts.at(0).getZ();
    //we cover the vector to search xmin and max
    for(int i=1; i<lspts.size(); i++)
    {
        if(xmin>lspts.at(i).getX())
            xmin=lspts.at(i).getX();
        if(xmax<lspts.at(i).getX())
            xmax=lspts.at(i).getX();
        if(zmin>lspts.at(i).getZ())
            zmin=lspts.at(i).getZ();
    }
    //we calculate the distance between them
    float * t;
    t= new float[3];
    t[0]=xmin;
    t[1]=xmax;
    t[2]=zmin;
    return t;
}
