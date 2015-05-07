#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    qw=new VueParEtape(this);
    ui->gridLayout->addWidget(qw);
    ui->pushC->setChecked(true);
    QObject::connect(ui->actionLoad_from_pcl_format, SIGNAL(triggered()), this, SLOT(loadFromFile()));
    QObject::connect(ui->actionSave_to_PCL_format, SIGNAL(triggered()), this, SLOT(saveFromFile()));
    QObject::connect(ui->actionSave_to_TXT_format, SIGNAL(triggered()), this, SLOT(saveFromFileTXT()));
    QObject::connect(ui->actionLoad_from_TXT_format, SIGNAL(triggered()), this, SLOT(loadFromSCN()));
    QObject::connect(ui->actionLoad_from_TXT_format_2, SIGNAL(triggered()), this, SLOT(loadCloudFromTXT()));
    QObject::connect(ui->pushClear, SIGNAL(clicked()), this, SLOT(clear()));
    QObject::connect(ui->pushExtract, SIGNAL(clicked()), this, SLOT(extract()));
    QObject::connect(ui->pushPS, SIGNAL(clicked()), this, SLOT(planarSeg()));
    QObject::connect(ui->pushC, SIGNAL(clicked()), this, SLOT(affichageOK()));

    QObject::connect(ui->interD, SIGNAL(valueChanged(int)), this, SLOT(changeD()));
    QObject::connect(ui->interF, SIGNAL(valueChanged(int)), this, SLOT(changeF()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::keyPressEvent(QKeyEvent *keyEvent)
{
    this->qw->keyPressEvent(keyEvent);

}
void MainWindow::mouseMoveEvent(QMouseEvent *event){
    this->qw->mouseMoveEvent(event);
}
void MainWindow::mousePressEvent(QMouseEvent *event){
    this->qw->mousePressEvent(event);
}
void MainWindow::mouseReleaseEvent(QMouseEvent *event){
    this->qw->mouseReleaseEvent(event);
}

void MainWindow::loadFromFile()
{
    this->qw->loadCloud();
    this->ui->nomf->setText(QString::fromStdString (this->qw->getNomF()));
    try{
        if(this->qw->getTaille() > 0)
        {
            std::stringstream ss;
            int ftpd=this->qw->getFtpD() ;
            int ftpf=this->qw->getFtpF();
            ss << ftpd <<"-" << ftpf;
            this->ui->intervalFtp->setText(QString::fromStdString (ss.str()));

            this->ui->interD->setMinimum(ftpd);
            this->ui->interF->setMinimum(ftpd);
            this->ui->interD->setMaximum(ftpf);
            this->ui->interF->setMaximum(ftpf);
            this->ui->interD->setValue(ftpd);
            this->ui->interF->setValue(ftpd);
        }
        else throw Erreur("Le fichier ne contient pas de point");
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }


}

void MainWindow::loadFromSCN()
{
    this->qw->loadFromSCN();
    this->ui->nomf->setText(QString::fromStdString (this->qw->getNomF()));

    try{
        if(this->qw->getTaille() > 0)
        {
            std::stringstream ss;
            int ftpd=this->qw->getFtpD() ;
            int ftpf=this->qw->getFtpF();
            ss << ftpd <<"-" << ftpf;
            this->ui->intervalFtp->setText(QString::fromStdString (ss.str()));

            this->ui->interD->setMinimum(ftpd);
            this->ui->interF->setMinimum(ftpd);
            this->ui->interD->setMaximum(ftpf);
            this->ui->interF->setMaximum(ftpf);
            this->ui->interD->setValue(ftpd);
            this->ui->interF->setValue(ftpd);
        }
        else throw Erreur("Le fichier ne contient pas de point");
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}
void MainWindow::loadCloudFromTXT(){
     this->qw->loadCloudFromTXT();
    this->ui->nomf->setText(QString::fromStdString (this->qw->getNomF()));

    try{
        if(this->qw->getTaille() > 0)
        {
            std::stringstream ss;
            int ftpd=this->qw->getFtpD() ;
            int ftpf=this->qw->getFtpF();
            ss << ftpd <<"-" << ftpf;
            this->ui->intervalFtp->setText(QString::fromStdString (ss.str()));

            this->ui->interD->setMinimum(ftpd);
            this->ui->interF->setMinimum(ftpd);
            this->ui->interD->setMaximum(ftpf);
            this->ui->interF->setMaximum(ftpf);
            this->ui->interD->setValue(ftpd);
            this->ui->interF->setValue(ftpd);
        }
        else throw Erreur("Le fichier ne contient pas de point");
    }catch(std::exception const& e){
        QMessageBox::critical(0, "Error", e.what());
    }
}

void MainWindow::saveFromFileTXT()
{
   this->qw->saveCloudsFromTXT();
}
void MainWindow::saveFromFile()
{
   this->qw->saveClouds();
}
void MainWindow::clear(){
   this->qw->clear();
}
void MainWindow::extract(){
    this->qw->setaffE(ui->pushExtract->isChecked());
}
void MainWindow::planarSeg(){
    this->qw->setaffS(ui->pushPS->isChecked());
}
void MainWindow::affichageOK(){
    this->qw->setaffC(ui->pushC->isChecked());
}

void MainWindow::changeD(){
    this->qw->setFtpDI(this->ui->interD->value());
}

void MainWindow::changeF(){
    this->qw->setFtpFI(this->ui->interF->value());
}
