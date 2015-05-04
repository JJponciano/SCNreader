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
#ifndef LESTEXTURES_H
#define	LESTEXTURES_H

#include "Texture.h"
#include <vector>
#include <iostream>
#include <string>
#include "GL/glut.h"
#include <stdlib.h>
#include "GL/freeglut_std.h"
#include <cmath>
class LesTextures {
public:
    /**
     * Constructeur
     */
    LesTextures();
    LesTextures(const LesTextures& orig);
    /**
     * Ajoute une texture
     * @param t texture à ajouter 
     */
    void add(Texture* t);
    /**
     * Ajoute et crée une texture
     * @param adresse adresse de l'image
     * @param nom nom de la texture
     */
    void add(QString adresse,std::string nom,GLuint id);
    /**
     * @brief add ajoute un texture avec son nom l'identifiant
     * @param adresse adresse ou charger la texture
     * @param nom identifiant de la texture
     */
    void add(QString adresse, std::string nom);
    /**
     * Affiche/applique la texture
     * @param nom nom de la texture à affiche/applique
     */
    void afficheTexture(std::string nom)const ;
    /**
     * Récupère la texture
     * @param nom nom de la texture à récupérer
     * @return texture portant le nom donné 
     */
    Texture* getTexture(std::string nom);
    virtual ~LesTextures();
private:
    GLuint key;
    bool nomValide(std::string nom);
 std::vector<Texture*> textures;
};

#endif	/* LESTEXTURES_H */

