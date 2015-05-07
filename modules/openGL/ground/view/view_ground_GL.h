/*
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 19 Octobre 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale -
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
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
