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

#ifndef TORE_H
#define	TORE_H
#include <iostream>
#include <vector>
#include "../Point.h"
#include "../Face.h"
#include "../Objets.h"
#include "GL/freeglut_std.h"
#include "GL/glut.h"
#include <cmath>
#include <stdlib.h>
/**
 * Création d'un tore facétisé
 * @code 
 *      Tore tore(4,3,100,100); 
 *      tore.affichage();
 * @endcode
 */
class Tore :public Objets{
public:
    /**Constructeur
     * @brief Construit un tore avec 100 parallèles et meridient de diamètre 2 et 1.
     */
    Tore();
    /**
     * Constructeur 
     * @param R distance entre le centre du tube et le centre du tore
     * @param r est le rayon du cercle 
     * @param parallele nomvre de parallèles
     * @param meridien nombre de méridient
     * @code 
     *      Tore tore(4,3,100,100); 
     * @endcode
     */
    Tore( float r,float R, int parallele, int meridien);
    /**
     * Constructeur de copie
     * @param orig tore à copier
     */
    Tore(const Tore& orig);
    /**
     * Calcule tous les points du tore
     */
    void construction();
    /**
     * Affiche chaque point du tore
     */
    void affichePoints()const;
    /**
     * Affiche le tore sous forme de facette
     */
    void affichage()const;
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
    virtual ~Tore();
private:
     void majCouleur();
    Couleur* couleur;
    float R;
    float r;
    int parallele;
    int meridien;
    float o;
    float g;
};

#endif	/* TORE_H */

