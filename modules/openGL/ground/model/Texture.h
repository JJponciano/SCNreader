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

#ifndef TEXTURE_H
#define	TEXTURE_H
#include <iostream>
#include "GL/glut.h"
#include <stdlib.h>
#include "GL/freeglut_std.h"
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

