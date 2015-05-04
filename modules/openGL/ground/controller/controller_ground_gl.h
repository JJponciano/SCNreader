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
          #ifndef CONTROLLER_GROUND_GL_H
#define CONTROLLER_GROUND_GL_H
#include"../model/model_ground_gl.h";

class controller_ground_GL
{
public:
    controller_ground_GL(model_ground_GL *model);
    ~controller_ground_GL();
     void initializeGL();
     void keyPressEvent( QKeyEvent *keyEvent );
     void mousePressEvent(QMouseEvent *event);
     void mouseReleaseEvent(QMouseEvent *event);
     void mouseMoveEvent(QMouseEvent *event);
     void zoom();
     void dezoom();
private:
     model_ground_GL * model;
};

#endif // CONTROLLER_GROUND_GL_H
