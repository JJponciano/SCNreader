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
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    qw=new scnreader_view(this);
    ui->gridLayout->addWidget(qw);
    QObject::connect(ui->actionLoad_from_pcl_format, SIGNAL(triggered()), this, SLOT(loadFromFile()));
    QObject::connect(ui->actionSave_to_PCL_format, SIGNAL(triggered()), this, SLOT(saveFromFile()));
    QObject::connect(ui->actionSave_to_TXT_format, SIGNAL(triggered()), this, SLOT(saveFromFileTXT()));
    QObject::connect(ui->actionLoad_from_TXT_format, SIGNAL(triggered()), this, SLOT(loadFromSCN()));
    QObject::connect(ui->actionLoad_from_TXT_format_2, SIGNAL(triggered()), this, SLOT(loadCloudFromTXT()));
    QObject::connect(ui->pushClear, SIGNAL(clicked()), this, SLOT(clear()));
    QObject::connect(ui->pushExtract, SIGNAL(clicked()), this, SLOT(extract()));
    QObject::connect(ui->pushPS, SIGNAL(clicked()), this, SLOT(planarSeg()));
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
}
void MainWindow::loadFromSCN()
{
  this->qw->loadFromSCN();
}
void MainWindow::loadCloudFromTXT(){
     this->qw->loadCloudFromTXT();
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

    this->qw->extractionCloud(0);
}
void MainWindow::planarSeg(){
    this->qw->planarSegmentation(0);
}
