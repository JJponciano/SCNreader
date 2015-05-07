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
    void resizeGL(int width, int height);
    /**
     * @brief initializeGL it initialize all required parameters to openGL .
     */
    void initializeGL();
    /**
     * @brief paintGL draw function of openGL
     */
    void paintGL();


    virtual void keyPressEvent( QKeyEvent *keyEvent );
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
};

#endif // groundGLWidget_H
