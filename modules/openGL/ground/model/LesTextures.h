/**
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * @date 13/03/2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale - 
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante 
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
 * Copyright  2014  
 * 
 * Contact: ponciano.jeanjacques@gmail.com
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

