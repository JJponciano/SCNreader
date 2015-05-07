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
#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtOpenGL>
#include <QGLWidget>

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(int framesPerSecond = 0, QWidget *parent = 0, char *name = 0);
    virtual void initializeGL() = 0;
    virtual void resizeGL(int width, int height) = 0;
    virtual void paintGL() = 0;
    bool fullWindow();
public slots:
    virtual void timeOutSlot();

protected:
      bool b_Fullscreen;
private:

    QTimer *t_Timer;

};


#endif // MYGLWIDGET_H
