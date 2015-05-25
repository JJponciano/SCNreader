/**
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 13 Mars 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale - 
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante 
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
 */

#ifndef TEXTURE_H
#define	TEXTURE_H
#include <iostream>
//#include "GL/glut.h"
//#include "GL/freeglut_std.h"
#include <stdlib.h>

#include <QImage>
#include <QtOpenGL/QtOpenGL>
/**
 * Installer la librairie libtiff-devel
 * sudo yum install libtiff-devel.x86_64 
 */
class Texture {
public:
    Texture(QString adresse,std::string nom,GLuint id);

    Texture(const Texture& orig);
    /**
     * Affichage de la texture
     * @code
     * glEnable(GL_TEXTURE_2D);
    charger();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBegin(GL_POLYGON);
    for (int i = 0; i < 6; i++) {
        float x = cos(i * PI / 3.0);
        float y = sin(i * PI / 3.0);

        glTexCoord2f((-x + 1) / 2, (-y + 1) / 2);
        glVertex3f(x * 10, y * 10, 0);

    }
    glEnd();
    glDisable(GL_TEXTURE_2D);
     * @endcode
     */
    void charger()const; 
    std::string getNnom()const; 
    virtual ~Texture();
    GLuint getId()const;
private:  
     void loadTexture();
    void construction();
    unsigned char image[256 * 256 * 3];
    unsigned char texture1[256][256][3];
    GLuint id;
    int largimg;
   int  hautimg;
    std::string nom;
    QString adresse;
};

#endif	/* TEXTURE_H */

