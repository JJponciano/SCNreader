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

#ifndef GAZINIERE_H
#define	GAZINIERE_H
#include "../Objet3D.h"

class Cubique : public Objet3D {
public:
    /**
     * Construteur d'un objet sous forme de cube texturé
     * @param largeur largeur de l'objet
     * @param hauteur hauteur de l'objet
     * @param longueur longueur de l'objet
     * @param d texture à appliquer sur le dessus  
     * @param f texture à appliquer sur le devant
     * @param autre texture à appliquer sur les autres côtés
     */
    Cubique(float largeur, float hauteur, float longueur, Texture* d, Texture* f, Texture* autre);
    Cubique(const Cubique& orig);
    virtual ~Cubique();
    /**
     * Affichage de l'objet
     */
    void affichage();
    

private:
    /**
     * Construction de l'objet
     */
    void construction();
    /**
     * Affichage d'un cube
     */
    void cube();
    /**
     * Construction d'un cube
     */
    void constructionCube();
    Texture* d;
    Texture* f;
    Texture* autre;
};

#endif	/* GAZINIERE_H */

