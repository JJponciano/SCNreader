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

#ifndef OBJETS_H
#define	OBJETS_H
#include <iostream>
#include <vector>
#include "Point.h"
#include "Texel.h"
#include "Face.h"
#include "../ground/model/Texture.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "GL/glut.h"
#include <stdlib.h>
#include "GL/freeglut_std.h"

class Objets {
public:
    Objets();
    /**
     * Calcule tous les points de l'objet
     */
    virtual void construction();
    /**
     * Affiche chaque point de l'objet
     */
    void affichePoints()const;
    /**
     * Affiche  l'objet sous forme de facette
     */
    void affichage()const;
    /**
     * Changement de texture
     * @param text nouvelle texture à affecter
     */
    void setTexture(Texture* text);
    Objets(const Objets& orig);
    virtual ~Objets();
protected:
    std::vector<PointGL*> points;
    std::vector<Texel*> texels;
    std::vector<Face*> faces;
    Texture* t;
    bool textureActive;
    /**
     *calcule les normales moyenne en chaque sommet. 
     * A exécuter impérativement à la fin de la construction de l'objet
     */
    void calculeNormal();

};

#endif	/* OBJETS_H */

