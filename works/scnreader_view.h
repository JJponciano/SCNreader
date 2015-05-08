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
#ifndef SCNREADER_VIEW_H
#define SCNREADER_VIEW_H

#include <QObject>
#include <QWidget>
#include "../modules/pcl/view/view_pcl.h"
#include "scnreader_model.h"
class scnreader_view : public View_pcl
{
     Q_OBJECT
public:
    scnreader_view(QWidget* parent = 0);
    ~scnreader_view();
     void paintGL();
     void loadFromSCN();
private:
    scnreader_model scnreaderFond;
};

#endif // SCNREADER_VIEW_H
