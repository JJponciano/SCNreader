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

}

scnreader_model::~scnreader_model()
{

}


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
    datas = value;
}





void scnreader_model::loadFromSCN(std::string pathname){
    //create scn data
    ScnData data;
    //load data from file
    data.loadFromSCN(pathname);

    //add clouds
    this->clouds.push_back(data.getcloud());
    //add this data created
    this->datas.push_back(data);
    //save the first package in data to txt file format ( only for debuging)
        this->datas.at(0).saveToTXT(0);
}









