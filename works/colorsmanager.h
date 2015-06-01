/**
 * @file colorsmanager.h
 * @brief file use for managed random color
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
#ifndef COLORSMANAGER_H
#define COLORSMANAGER_H
#include <time.h>
#include <QVector>
#include <QHash>
#include <QRgb>
#include <QColor>
class ColorsManager
{
public:
    ColorsManager();
    ~ColorsManager();
    QVector< double> getColor(int key);
    QRgb getQrgb(int k);
private:
    QHash<int,QVector< double> > colors;


};

#endif // COLORSMANAGER_H
