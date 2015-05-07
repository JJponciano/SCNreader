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
#ifndef GROUNDGLWIDGET_H
#define GROUNDGLWIDGET_H

/**
 * @file groundglwidget.h
 * @brief file to the managements of exeptions
 * @author Jean-Jacques PONCIANO
 * @version 0.1
 */

#include "view/view_ground_GL.h"
/**
 * @class groundGLWidget
 * @brief The groundGLWidget class  This class grouped the main function openGL.
 * Each function is already implemented and allows easy use of openGL .
 * For more functionalities, You could overloading each function into a new class.
 */
class groundGLWidget : public View_ground_GL
{
    Q_OBJECT
public:
    /**
     * @brief groundGLWidget constructor
     * @param parent
     */
    groundGLWidget(QWidget* parent = 0);
    ~groundGLWidget();
    /**
     * @brief resizeGL Resize openGL window.
     *  This function determines the most addaptees visualization parameters to the size of the openGL windows
     * @param width width of windows in pixel
     * @param height height of windows in pixel
     */
    virtual void resizeGL(int width, int height);
    /**
     * @brief initializeGL it initialize all required parameters to openGL .
     */
    virtual void initializeGL();
    /**
     * @brief paintGL draw function of openGL
     */
    virtual void paintGL();

    virtual void keyPressEvent( QKeyEvent *keyEvent );
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
protected:
    float pX;
    float pY;
    float pZ;
    float lX;
    float lY;
    float lZ;
};

#endif // groundGLWidget_H

