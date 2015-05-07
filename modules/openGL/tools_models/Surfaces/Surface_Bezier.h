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

#ifndef SURFACE_BEZIER_H
#define	SURFACE_BEZIER_H

#include "../Point.h"
#include <math.h>
#include "GL/glut.h"
#include <cmath>
#include <iostream>
#include <string>
#include "GL/freeglut_std.h"
#include <vector>
#include "../Objet3D.h"

class Surface_Bezier :public Objet3D {
    public:
    
    /**
     * @brief Constructeur de la surface de type Bézier
     * @param lespoints vecteur à 2 dimensions représentant la matrice 
     * des points de controle de la surface
     * @param nbli nombre de lignes de la matrice
     * @param nbco nombre de colonnes de la matrice
     */
    Surface_Bezier(std::vector< std::vector<PointGL*> > lespoints, int nbli, int nbco, float largeur, float hauteur, float longueur);
    
    /**
     * @brief Constructeur en copie
     * @param orig
     */
    Surface_Bezier(const Surface_Bezier& orig);
    
    /**
     *  @brief Destructeur de la classe qui détruit les pointeurs
     */
    virtual ~Surface_Bezier();
    
    /**
     * @brief Fonction permattant l'affichage des points de la surface
     */
    void Affichage();
    
private:
    /**
     * @brief Fonction calculant le produit de deux matrices
     * @param matrice1 première matrice destinée au produit
     * @param nbl1 nombre de lignes de la première matrice
     * @param nbc1 nombre de colonnes de la première matrice
     * @param matrice2 seconde matrice destinée au produit
     * @param nbl2 nombre de lignes de la seconde matrice
     * @param nbc2 nombre de colonnes de la seconde matrice
     * @return la matrice correspondant au produit de matrice1 et matrice2
     */
    std::vector< std::vector<float> > ProduitMat(std::vector< std::vector<float> > matrice1, int nbl1, int nbc1, std::vector< std::vector<float> > matrice2, int nbl2, int nbc2);

    /**
     * @brief Fonction renvoyant la transposée d'une matrice
     * @param matrice matrice dont on souhaite obtenir la transposée
     * @param nbl nombre de lignes de la matrice
     * @param nbc nombre de colonnes de la matrice
     */
    std::vector< std::vector<float> > TransposerMat(std::vector< std::vector<float> > matrice, int nbl, int nbc);
    
    //fonction renvoyant l'inverse d'une matrice
    //std::vector< std::vector<float> > InverseMat(std::vector< std::vector<float> > matrice, int nbl, int nbc);
    
    /**
     * @brief Fonction calculant l'abscisse de chaque point
     * @param u instant u, avancement horizontale de la courbe 
     * @param v instant v, avancement verticale de la courbe 
     */
    float FuncX(float u, float v);

    /**
     * @brief Fonction calculant l'ordonnée de chaque point
     * @param u instant u, avancement horizontale de la courbe 
     * @param v instant v, avancement verticale de la courbe 
     */
    float FuncY(float u, float v);

    /**
     * @brief Fonction calculant la profondeur de chaque point
     * @param u instant u, avancement horizontale de la courbe 
     * @param v instant v, avancement verticale de la courbe 
     */
    float FuncZ(float u, float v);

    /**
     * @brief Fonction calculant la normale en chaque point
     * @param u instant u, avancement horizontale de la courbe 
     * @param v instant v, avancement verticale de la courbe 
     */
    std::vector<float> FuncNormale(float u, float v);

    /**
     * @brief Fonction permettant de calculer tous les points de la surface
     */
    void CalculeSurface();
    
    /**
     * @brief Fonction permettant de calculer les facettes
     */
    void calculeFacette();
    
    //Matrice caractéristique de Bézier
    std::vector< std::vector<float> > bezier;
    //nombre de lignes et de colonnes du tableau de points de controle
    int nbl,nbc;
    //Tableau contenant tous les points de controle permettant de construire la surface
    std::vector< std::vector<PointGL*> > ptsControle;
    //nombre de décalage en ligne et en colonne à faire pour construire tous les morceaux de surfaces
    int nbdecL, nbdecC;
    //Tableau contenant les points de controles courants
    std::vector< std::vector<PointGL*> > ptsCtrlC;
    
    

};

#endif	/* SURFACE_BEZIER_H */

