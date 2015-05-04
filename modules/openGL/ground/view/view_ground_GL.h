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
#ifndef VIEW_GOUND_GL_H
#define VIEW_GOUND_GL_H

#include <QList>
#include <QMouseEvent>

#include "../controller/controller_ground_gl.h"
#include "../model/model_ground_gl.h"
class View_ground_GL : public GLWidget
{
    Q_OBJECT
public:
    explicit View_ground_GL(QWidget *parent = 0);
    ~View_ground_GL();
    void resizeGL(int ratio, int height);
    void initializeGL();
    void paintGL();
    virtual void keyPressEvent( QKeyEvent *keyEvent );
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
private:
    controller_ground_GL *controller;
    model_ground_GL *model;

    GLfloat ratio;
};

#endif // VIEW_GOUND_GL_H
