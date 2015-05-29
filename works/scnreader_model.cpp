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

void scnreader_model::loadFromSCN(std::string pathname){
    //create scn data
    ScnData data;
    //load data from file
    data.loadFromSCN(pathname);
    //keep coordinates of points
    QVector<double> lesX=data.getX();
    QVector<double> lesY=data.getY();
    QVector<double> lesZ=data.getZ();

    //add them in Hashtable
    PointGL * p;
    int ftpcourant=-1;
    QVector<PointGL *>* v;
    for(int i=0; i<lesX.size(); i++)
    {
        p=new PointGL(lesX.at(i),lesY.at(i),lesZ.at(i));

        //if the footpulse is not initialized
        if(ftpcourant==-1)
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
        {
            //we update footpulse
            ftpcourant=(int) p->getZ();
            //if a vector with this footpulse exists
            if(nuage.contains(ftpcourant))
            {
                //we the vector which corresponding to point's footpulse
                v= nuage.value(ftpcourant);
                //then we add the new point
                v->push_back(p);
            }
            else
            {
                    //we create a new vector
                    v= new QVector<PointGL *>();
                    //which we add to the hashtable
                    nuage.insert(ftpcourant, v);
                    //then we add the new point
                    v->push_back(p);
            }
        }
    }
    int * t=ftpMinMax();
    this->ftpd=t[0];
    this->ftpf=t[1];
    this->createRail();
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
            //convert coordonated Qstring to double coordinates to add in vector
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
                        p=new PointGL(z.toDouble(),y.toDouble(),x.toDouble());
                    }
                    else if(footpulse==1)
                    {
                        p=new PointGL(x.toDouble(),z.toDouble(),y.toDouble());
                    }
                    else
                    {
                        p=new PointGL(x.toDouble(),y.toDouble(),z.toDouble());
                    }

                    if(ftpcourant==-1)
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
                    {
                        //we update footpulse
                        ftpcourant=(int) p->getZ();
                        //if a vector with this footpulse exists
                        if(nuage.contains(ftpcourant))
                        {
                            //we the vector which corresponding to point's footpulse
                            v= nuage.value(ftpcourant);
                            //then we add the new point
                            v->push_back(p);
                        }
                        else
                        {
                                //we create a new vector
                                v= new QVector<PointGL *>();
                                //which we add to the hashtable
                                nuage.insert(ftpcourant, v);
                                //then we add the new point
                                v->push_back(p);
                        }
                    }
                }

            //if user want to stop loading, the reading is finished
            if (progress.wasCanceled())
                break;
        }

        int * t=ftpMinMax();
        this->ftpd=t[0];
        this->ftpf=t[1];
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
    double tx,ty,tz;
    //current value
    double vx,vy,vz;
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
            tx=x.toDouble();
            ty=y.toDouble();
            tz=z.toDouble();
        }else if(result1.size()<3)throw Erreur(" Error reading file!");
        else{
            QString x=result1.at(0);
            QString y=result1.at(1);
            QString z=result1.at(2);
            //previous value
            tx=x.toDouble();
            ty=y.toDouble();
            tz=z.toDouble();
        }

        //__________Cover of file's part_______________
        while(!flux.atEnd() && !find)
        {
            //__________read the x, y, z

            //read line by line
            ligne= flux.readLine();
            // split the line with space as a separator character
            QStringList result =ligne.split("\t");
            //convert coordonated Qstring to double coordinates to compare

            //test if the first line define the number of lines into the file
            if(result.size()<3)throw Erreur(" Error reading file!");
            else{
                QString x=result.at(0);
                QString y=result.at(1);
                QString z=result.at(2);
                //convert coordonated Qstring to double coordinates to compare
                vx=x.toDouble();
                vy=y.toDouble();
                vz=z.toDouble();

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
        double x=vecteur.at(i)->getX();
        double y=vecteur.at(i)->getY();
        double z=vecteur.at(i)->getZ();
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
        double x=cloud->points.at(i).x;
        double y=cloud->points.at(i).y;
        double z=cloud->points.at(i).z;
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
        double x=cloud->points.at(i).x;
        double y=cloud->points.at(i).y;
        double z=cloud->points.at(i).z;
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
    double x1=ptP->getX();
    double y1=ptP->getY();
    double z1=ptP->getZ();

    //we take the coordinates of pt
    double x2=point2->getX();
    double y2=point2->getY();
    double z2=point2->getZ();

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
//            //---------------JJ
//            if(ftpf>this->ftpd+500)
//            ftpf=this->ftpd+500;
//            //-----------------

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
    //fill regions vector
     this->regions.clear();
    for(int i=0;i<  this->lesRailsOptimize.getRegions().getRegions().size();i++){
        this->regions.push_back(this->lesRailsOptimize.getRegions().getRegions().at(i).getPoints());
    }
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

QVector<QVector<PointGL> > scnreader_model::getRegions() const
{
    return regions;
}

void scnreader_model::setRegions(const QVector<QVector<PointGL> > &value)
{
    regions = value;
}

//For each part of cloud:
void scnreader_model::cleanNoise(int f){
    //we fix the width of 1 pixel
    double dense=0.04;
    //we keep the cloud which we must clean
    QVector<PointGL> lespoints=this->lesRails.getCloud();
    //we keep distance between xmin and xmax
    double* val=distanceMinMax(lespoints);
    double dist=val[1]-val[0];
    double xmin=val[0];
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
        //we increase this position
        im.increase(l,c);
    }
    //we do a calibration of gray level between 0 and 255
    im.calibration();
    //we do a thresholding with threshold=125 => binarization
    im.thresholding(125);
    im.growingRegion();
    im.calibration();
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
    //update of tracks
    this->lesRails.initialization(lespoints);
}

double * scnreader_model::distanceMinMax(QVector<PointGL> lspts)
{
    //initialization if xmin and xmax
    double xmin=lspts.at(0).getX();
    double xmax=lspts.at(0).getX();
    double zmin=lspts.at(0).getZ();
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
    double * t;
    t= new double[3];
    t[0]=xmin;
    t[1]=xmax;
    t[2]=zmin;
    return t;
}

int * scnreader_model::ftpMinMax()
{
    if(!nuage.isEmpty())
    {
        //keep values of key in nuage
        QList<int> cle=nuage.keys();
        //initialization of min and max
        double min=cle.at(0);
        double max=cle.at(0);

        //we cover the vector to search xmin and max
        for(int i=1; i<cle.size(); i++)
        {
            if(min>cle.at(i))
                min=cle.at(i);
            if(max<cle.at(i))
                max=cle.at(i);
        }
        //we calculate the distance between them
        int * t;
        t= new int[2];
        t[0]=min;
        t[1]=max;
        return t;
    }
    else Erreur("Le nuage de point est vide, nous ne pouvons determiner les footpulses de debut et de fin!");
}
