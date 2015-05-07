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

#ifndef SPHERE_H
#define	SPHERE_H
#include <iostream>
#include <vector>
#include "../Point.h"
#include "../Face.h"
#include "../Objet3D.h"
#include "GL/freeglut_std.h"
#include "GL/glut.h"
#include "../Texel.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
class Sphere :public Objet3D{
public:
    Sphere();
    Sphere(float r, int parallele, int meridien);
    Sphere(const Sphere& orig);
    virtual ~Sphere();
     /**
     * Calcule tous les points de la sphere
     */
    void construction();
     /**
     * Affecte une couleur aux faces 
     * @param r canal rouge
     * @param g canal vert
     * @param b canal bleu
     */
     void setCouleur(float r, float g,float b);
     /**
     * Affecte une couleur aux faces 
     * @param c couleur 
     */
     void setCouleur(const Couleur& c);
    /**
     * Affiche chaque point de la sphere
     */
    void affichePoints();
    /**
     * Affiche  l'objet
     */
    void affichage();
private:
    void majCouleur();
    Couleur* couleur;
    float rayon;
    int parallele;
    int meridien; 
    float o;
    float g;

};

#endif	/* SPHERE_H */

