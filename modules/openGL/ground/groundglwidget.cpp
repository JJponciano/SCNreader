#include "groundglwidget.h"

groundGLWidget::groundGLWidget(QWidget *parent)
    : View_ground_GL(parent)
{
}
groundGLWidget::~groundGLWidget()
{
}

// ------------------------------------------ OPENGL Functions ------------------------------------------
void groundGLWidget::initializeGL()
{
    View_ground_GL::initializeGL();
     glDisable(GL_LIGHTING);
}

void groundGLWidget::resizeGL(int width, int height)
{
    View_ground_GL::resizeGL( width,  height);
}

void groundGLWidget::paintGL()
{
    //call the superclass function
    View_ground_GL::paintGL();
}

//===============================================
//             Keyboard and mouse events
//===============================================
void groundGLWidget::keyPressEvent(QKeyEvent *keyEvent)
{
View_ground_GL::keyPressEvent(keyEvent);
}
void groundGLWidget::mouseMoveEvent(QMouseEvent *event){
    View_ground_GL::mouseMoveEvent(event);
}
void groundGLWidget::mousePressEvent(QMouseEvent *event){
    View_ground_GL::mousePressEvent(event);
}
void groundGLWidget::mouseReleaseEvent(QMouseEvent *event){
    View_ground_GL::mouseReleaseEvent(event);
}

