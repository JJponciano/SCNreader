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
* @version 1.0
*/
#include "scndata.h"
ScnData::ScnData(std::string pathname)
{
    this->loadFromSCN(pathname);
}
ScnData::ScnData()
{
    this->indexreader=0;
}

ScnData::~ScnData()
{
    this->packages.clear();
}
void ScnData::loadFromSCN(std::string pathname){

    this->indexreader=0;
    QFile fichier( QString(pathname.c_str()) );
    if(fichier.open(QIODevice::ReadOnly)){
        this->alldatas=fichier.readAll();
        this->assigningDatas();
        this->fill();
    }
}
//assigning of datas
void ScnData::assigningDatas(){
    // the hearder has 288 bytes
    this->indexreader=288;
    //read all data's packet;
    //as long as it still a given package
    // create progress dialog to inform the user of progress if the task has done is too long
    QProgressDialog progress("Loading file...", "Stop loading", 0, this->alldatas.size(), 0);
    //said that the window is modal
    progress.setWindowModality(Qt::WindowModal);
    progress.setValue(0);
    while(this->indexreader<this->alldatas.size()){
            progress.setValue(indexreader);

        //read block of byte and create a data package
        Datapackage* dp=new Datapackage(this->alldatas,this->indexreader);
        //update index reader
        this->indexreader=dp->getEnd();
        // add package
        this->packages.push_back(dp);
        //if user want to stop loading, the reading is finished
        if (progress.wasCanceled())
            break;
    }
    progress.setValue(this->alldatas.size());
}

void ScnData::fill(){
    this->x.clear();
    this->y.clear();
    this->z.clear();
    // get all points for each packages
    for(int i=0;i<this->packages.size();i++)
        for(int j=0;j<this->packages.at(i)->getX().size();j++){
            this->x.push_back(this->packages.at(i)->getX().at(j));
            this->y.push_back(this->packages.at(i)->getY().at(j));
            this->z.push_back(this->packages.at(i)->getFootpulse());
        }
}

int ScnData::getIndexreader() const
{
    return indexreader;
}
QVector<double> ScnData::getX() const
{
    return x;
}
QVector<double> ScnData::getY() const
{
    return y;
}
QVector<double> ScnData::getZ() const
{
    return z;
}

QVector<Datapackage*> ScnData::getPackages() const
{
    return packages;
}
