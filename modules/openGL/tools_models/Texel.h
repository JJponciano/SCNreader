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

#ifndef TEXEL_H
#define	TEXEL_H
#include "GL/glut.h"
#include "GL/freeglut_std.h"

class Texel {
public:
    /**
     * Construction du point
     * @param x abscisse sur l'image 
     * @param y ordonnée dans l'image 
     */
    Texel(float x, float y);
    Texel(const Texel& orig);
    /**
     * Récupération de l'abcisse du point
     * @return  l'abscisse sur l'image
     */
    float getX()const;
    /**
     * Récupération de l'ordonnée du point
     * @return  l'ordonnée sur l'image 
     */
    float getY()const;
    virtual ~Texel();
private:
    // abscisse
    float x;
    // ordonnée
    float y;
};

#endif	/* TEXEL_H */

