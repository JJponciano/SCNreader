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
#ifndef OBJET3D_H
#define	OBJET3D_H
#include <vector>
#include <cmath>
#include <iostream>
#include "../ground/model/Materiel.h"
#include "Point.h"
#include "Texel.h"
#include "Face.h"
#include "../ground/model/Texture.h"
#include <exception>
#include <iostream>
#include <stdexcept>
#include "GL/glut.h"
#include "GL/freeglut_std.h"

class Objet3D {
public:
    /**
     * Constructeur
     * @param largeur largeur de l'objet
     * @param hauteur hauteur de  l'objet
     * @param longueur longueur de l'objet
     */
    Objet3D(float largeur, float hauteur, float longueur);
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
    Objet3D(const Objet3D& orig);
    /**
     * Récupération de la hauteur de l'objet
     * @return hauteur de l'objet
     */
    float getHauteur()const;
    /**
     * Récupération de la largeur de l'objet
     * @return  largeur de l'objet
     */
    float getLargeur()const;
    /**
     * Récupération de la longueur de l'objet
     * @return longueur de l'objet
     */
    float getLongeur()const;

    /**
     * Animation de l'objets par interpolation linaire entre chaque points 
     * du tableau référent "pointsReferents" avec chaque point du tableau 
     * "pointsCles"
     * @param pointsCles tableau de points référent représentant une position clé
     *  de l'objet 
     * @param a vitesse d'avancement de l'interpolation linéaire.
     */
    void animationLineaire(std::vector<PointGL*> pointsCles, float a);

    /**
     * Accesseur en lecture au tableau de points
     * @return le tableau contenant les points de l'objet3D
     */
    std::vector <PointGL*> getP();
    virtual ~Objet3D();

protected:
    /**
     *calcule les normales moyenne en chaque sommet. 
     * A exécuter impérativement à la fin de la construction de l'objet
     */
    void calculeNormal();
    /**
     * Calcule toutes les faces à partir du tableau de point "points"
     */
    void calculeFacette();

    // tableaux de points pointant sur tous les points de l'objets
    std::vector<PointGL*> pointsReferents;
    // tous les points de l'objet
    std::vector<PointGL*> points;
    //tous les texels correspondants aux points de même indice dans "points"
    std::vector<Texel*> texels;
    // toutes les faces de l'objet
    std::vector<Face*> faces;
    // hauteur de l'objet
    float hauteur;
    //largeur de l'objet
    float largeur;
    // longueur de l'objet
    float longueur;
    // matériaux de l'objet
    Materiel* mat;
    // activation et désactivation de l'animation dans le déplacement des pièces
    bool animationActive;
};

#endif	/* OBJET3D_H */

