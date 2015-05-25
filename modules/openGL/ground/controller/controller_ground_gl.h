#ifndef CONTROLLER_GROUND_GL_H
#define CONTROLLER_GROUND_GL_H
#include"../model/model_ground_gl.h"

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
