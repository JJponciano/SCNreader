/**
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
    QObject::connect(ui->regions, SIGNAL(clicked()), this, SLOT(affichageRegions()));
    QObject::connect(ui->affswitch, SIGNAL(clicked()), this, SLOT(affichageSwitch()));

    QObject::connect(ui->ransac, SIGNAL(clicked()), this, SLOT(afficheRansac()));

    QObject::connect(ui->interD, SIGNAL(valueChanged(int)), this, SLOT(changeD()));
    QObject::connect(ui->interF, SIGNAL(valueChanged(int)), this, SLOT(changeF()));

    QObject::connect(ui->swpcd, SIGNAL(clicked()), this, SLOT(previousSwitch()));
    QObject::connect(ui->swsuiv, SIGNAL(clicked()), this, SLOT(nextSwitch()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::keyPressEvent(QKeyEvent *keyEvent)
{
    if(keyEvent->key()==Qt::Key_O){
        int ftpd=this->qw->getFtpDI()+1;
        int ftpf=this->qw->getFtpFI()+1;

        this->ui->interD->setValue(ftpd);
        this->ui->interF->setValue(ftpf);

        this->qw->setFtpDI(this->ui->interD->value());
        this->qw->setFtpFI(this->ui->interF->value());
    }
    else
    if(keyEvent->key()==Qt::Key_L){
        int ftpd=this->qw->getFtpDI()-1;
        int ftpf=this->qw->getFtpFI()-1;

        this->ui->interD->setValue(ftpd);
        this->ui->interF->setValue(ftpf);

        this->qw->setFtpDI(this->ui->interD->value());
        this->qw->setFtpFI(this->ui->interF->value());
    }
    else
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

            int n=this->qw->getNumS();
            if(n==-1)
                this->ui->numsw->setText(QString("Whithout detected Switch"));
            else
                this->ui->numsw->setText(QString::number(n));
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

    this->ui->numsw->setText(QString(""));
    this->ui->nomf->setText(QString(""));
    this->ui->intervalFtp->setText(QString(""));
    this->ui->interD->setMinimum(0);
    this->ui->interF->setMinimum(0);
    this->ui->interD->setValue(0);
    this->ui->interF->setValue(0);

    ui->pushC->setChecked(true);
    ui->pushExtract->setChecked(false);
    ui->pushPS->setChecked(false);
    ui->ransac->setChecked(false);
    ui->affswitch->setChecked(false);
}
void MainWindow::extract(){
    this->qw->setaffE(ui->pushExtract->isChecked());
}
void MainWindow::planarSeg(){
    this->qw->setaffS(ui->pushPS->isChecked());
}
void MainWindow::affichageOK(){
    if(ui->pushC->isChecked())
    {
        ui->affswitch->setChecked(false);
        this->qw->setAffswitch(false);
    }
    this->qw->setaffC(ui->pushC->isChecked());
}
void MainWindow::affichageRegions(){
    if(ui->regions->isChecked())
    {
        ui->affswitch->setChecked(false);
        this->qw->setAffswitch(false);
    }
    this->qw->setaffReg(ui->regions->isChecked());
}
void MainWindow::affichageSwitch(){
    if(ui->affswitch->isChecked())
    {
        ui->pushC->setChecked(false);
        this->qw->setaffC(false);
    }
    this->qw->setAffswitch(ui->affswitch->isChecked());
}

void MainWindow::afficheRansac(){
    this->qw->setaffR(ui->ransac->isChecked());
}

void MainWindow::changeD(){
    this->qw->setFtpDI(this->ui->interD->value());
}

void MainWindow::changeF(){
    this->qw->setFtpFI(this->ui->interF->value());
}

void MainWindow::nextSwitch(){
    this->qw->IncreasePosSwitch();
    int n=this->qw->getNumS();
    if(n==-1)
        this->ui->numsw->setText(QString("Whithout detected Switch"));
    else
        this->ui->numsw->setText(QString::number(n));
}

void MainWindow::previousSwitch()
{
    this->qw->DecreasePosSwitch();
    int n=this->qw->getNumS();
    if(n==-1)
        this->ui->numsw->setText(QString("Whithout detected Switch"));
    else
        this->ui->numsw->setText(QString::number(n));
}
