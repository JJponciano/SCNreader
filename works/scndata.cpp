#include "scndata.h"

ScnData::ScnData(std::string pathname)
{
    this->indexreader=0;
    this->loadFromSCN(pathname);
    this->assigningDatas();
}
ScnData::ScnData()
{
    this->indexreader=0;
}

ScnData::~ScnData()
{

}
/*
void ScnData::load(QString pathname){
    QFile file(pathname);
    // Ouverture du fichier en lecture seule
    if (!file.open(QIODevice::ReadOnly))
        throw Erreur(" Chargement du fichier impossible!");
    // Création de notre QDataStream à partir de notre fichier précédemment ouvert
    QDataStream binStream(&file);
    // On spécifie exactement le même type d'encodage que celui utilisé lors de l'écriture
   // binStream.setVersion(QDataStream::Qt_5_4);

    //get header
    char  header[288];
    binStream>>header;

    // add parameters
    int temp;
    binStream>>temp;this->setThreshold(temp);
    binStream>>temp;this->setclosing_size(temp);
    binStream>>temp;this->setLast_closing_size(temp);
    binStream>>temp;this->setOpening_size(temp);
    binStream>>temp;this->setIntensite(temp);
    binStream>>temp;this->setBrillance(temp);
    float tempf;
    binStream>>tempf;this->setTolerance(tempf);
    QString qs;
    binStream>>qs;
    this->setInputfilename(qs.toStdString());
    //coordinated pixel to processing
    binStream>>temp;this->setX(temp);
    binStream>>temp;this->setY(temp);
    // Fermeture du fichier
    file.close();


}*/

/*
 * QElapsedTimer timer; //for measuring speed in nsecs
timer.start();
char *temp = new char[length]; //need to change into this, compiler throws error: "expected constant expression"
input.readRawData(temp, length);
buffer.append(temp, length);
delete [] temp;
qDebug() << timer.nsecsElapsed();
*/
/*
void ScnData::loadFromSCN(std::string pathname){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //read file
    QFile fichier( QString(pathname.c_str()) );
    if(fichier.open(QIODevice::ReadOnly)) // ce  si le fichier n'est pas ouvert
    std::ifstream f (pathname, std::ios::in | std::ios::binary);

    if (f.is_open())
    {
        // QDataStream in(&fichier);    // read the data serialized from the file

        // ------------------ READ DATA ------------------
        char a;
        char b;
        //get header
        char  header[288];
           f.read ((char *)& a, sizeof(char));
        f.read ((char *)& a, sizeof(char));
        f.read ((char *)& b, sizeof(char));
        QString qs="";
        qs.append(a);
        qs.append(b);
        this->id=qs.toStdString();
        f.read ((char *)& a, sizeof(char));
        f.read ((char *)& b, sizeof(char));
        qs="";
        qs.append(a);
        qs.append(b);
        this->version=qs.toStdString();
        f.read ((char *)&  this->radius, sizeof(INT16));
        f.read ((char *)&  this->radienkorrektur, sizeof(BYTE));
        f.read ((char *)&  this->spurweite, sizeof(INT16));
        f.read ((char *)&  this->nglLeftHorizontal, sizeof(INT16));
        f.read ((char *)&  this->nglLeftVertical, sizeof(INT16));
        f.read ((char *)&  this->nglLeftBanking, sizeof(INT16));
        f.read ((char *)&  this->nglLeftGauge, sizeof(INT16));
        f.read ((char *)&  this->nglLeftGauge, sizeof(INT16));
        f.read ((char *)&  this->nglRightHorizontal, sizeof(INT16));
        f.read ((char *)&  this->nglRightVertical, sizeof(INT16));
        f.read ((char *)&  this->nglRightBanking, sizeof(INT16));
        f.read ((char *)&  this->nglRightGauge, sizeof(INT16));
        f.read ((char *)&  this->pointCount, sizeof(INT16));
        // ------------------ END READ DATA ------------------


        std::cout<<this->id<<std::endl;
        std::cout<<this->version<<std::endl;
        std::cout<<this->radius<<std::endl;
        std::cout<<this->radienkorrektur<<std::endl;
        std::cout<<this->spurweite<<std::endl;
        std::cout<<this->nglLeftHorizontal<<std::endl;
        std::cout<<this->nglLeftVertical<<std::endl;
        std::cout<<this->nglLeftBanking<<std::endl;
        std::cout<<this->nglLeftGauge<<std::endl;
        std::cout<<this->nglRightHorizontal<<std::endl;
        std::cout<<this->nglRightVertical<<std::endl;
        std::cout<<this->nglRightBanking<<std::endl;
        std::cout<<this->nglRightGauge<<std::endl;
        std::cout<<this->pointCount<<std::endl;

        // create progress dialog to inform the user of progress if the task has done is too long
        QProgressDialog progress("Loading cloud...", "Stop loading", 0, this->pointCount, 0);
        //said that the window is modal
        progress.setWindowModality(Qt::WindowModal);

        // Fill in the cloud data
        cloud->width    = this->pointCount;
        cloud->height   = 1;
        cloud->is_dense = false;
        cloud->points.resize (cloud->width * cloud->height);

        for(int i=0;i<this->pointCount;i++)
        {
            //update progress Dialog
            progress.setValue(i);
            int y;
            int z;
            //read point
            f.read ((char *)&  y, sizeof(INT16));
            f.read ((char *)&  z, sizeof(INT16));
            cloud->points[i].x = 0;
            cloud->points[i].y =y;
            cloud->points[i].z = z;


            //if user want to stop loading, the reading is finished
            if (progress.wasCanceled())
                i=this->pointCount;
        }
        //close automatically the progress dialog
        progress.setValue(this->pointCount);//
        //close file
        f.close();
        //  fichier.close();
        //affectation new cloud at the current cloud
        this->cloud=cloud;
    }else throw Erreur("the file "+pathname +"have not been opened!");

}*/

void ScnData::loadFromSCN(std::string pathname){

    this->indexreader=0;
    QFile fichier( QString(pathname.c_str()) );
    if(!fichier.open(QIODevice::ReadOnly)){
        throw Erreur("the file "+pathname +"have not been opened!");
    }
     else
    this->alldatas=fichier.readAll();
   //  dp.readDataFile(pathname);
     this->assigningDatas();


}
//assigning of datas
void ScnData::assigningDatas(){
    // the hearder has 288 bytes
    this->indexreader=288;
    //read all data's packet;
    //as long as it still a given package
    int i=0;
    while(this->indexreader<this->alldatas.size()){
        //read block of byte and create a data package
        Datapackage dp(this->alldatas,this->indexreader);
        //update index reader
       this->indexreader=dp.getEnd();
       // this->indexreader+=10948;
        // add package
        this->packages.push_back(dp);


        std::cout<<this->indexreader<<std::endl;


        // Création d'un objet QFile
        QString name=QString::number(i++);
        name+=".txt";
        QFile file(name);
        // On ouvre notre fichier en lecture seule et on vérifie l'ouverture
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
            throw Erreur("the file "+name.toStdString() +"have not been opened!");

        if(false){
        // Création d'un objet QTextStream à partir de notre objet QFile
        QTextStream flux(&file);
        // On choisit le codec correspondant au jeu de caractère que l'on souhaite ; ici, UTF-8
        flux.setCodec("UTF-8");
        // Écriture des différentes lignes dans le fichier

        flux <<QString::fromStdString(dp.toString());
        file.close();
        }
       // break only for debug datapackage because i have not debugger
    }std::cout<<"count of paquages: "<<this->packages.size()<<std::endl;
}
int ScnData::getIndexreader() const
{
    return indexreader;
}

void ScnData::setIndexreader(int value)
{
    indexreader = value;
}

QVector<Datapackage> ScnData::getPackages() const
{
    return packages;
}

void ScnData::setPackages(const QVector<Datapackage> &value)
{
    packages = value;
}
QByteArray ScnData::getAlldatas() const
{
    return alldatas;
}

void ScnData::setAlldatas(const QByteArray &value)
{
    alldatas = value;
}

