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
#ifndef SCNREADER_MODEL_H
#define SCNREADER_MODEL_H


#include "../modules/exceptions/erreur.h"
#include "scndata.h"
#include <QVector>
#include <QDataStream>
class scnreader_model: public ToolsPCL
{
public:
    scnreader_model();
    ~scnreader_model();


    /**
     * @author Jean-Jacques PONCIANO
     *
     * @brief scnreader_model::loadFromSCN create and add data from file
     * @param pathname the file in svn format
     */
    void  loadFromSCN(std::string pathname);



   ScnData getData(int i) const;
   void setDatas(const QVector<ScnData> &value);

private:
    /**
     * @brief readData read a data byte by byte
     * @param bytePosition posision of the data
     * @param length length of bytes of the data
     * @param pathname path of file data
     * @return the data to QString
     */
    QString readData(int bytePosition, int length, std::string pathname, bool isnumber);
    QVector <ScnData> datas;


};

#endif // SCNREADER_MODULE_H
