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
 * 
 * Contact: ponciano.jeanjacques@gmail.com
 */
#ifndef CATMULLROM_H
#define	CATMULLROM_H

#include "../Point.h"
#include <math.h>
#include "GL/glut.h"
#include <cmath>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "GL/freeglut_std.h"
#include <vector>


class CatmullRom {
public:
    /**
    * @brief Constructeur d'une courbe type Catmull-Rom
    * @param s coefficient de la courbe (en générale, égale à 1/2)
    * @param n nombre de points voulu pour la courbe
    */
    CatmullRom(float s, int n);
    
    /**
     * @brief Constructeur de copie
     */
    CatmullRom(const CatmullRom& orig);
    
    /**
    * @brief Destructeur de la classe qui détruit tous les pointeurs
    */
    virtual ~CatmullRom();
    
    /**
    * @brief Fonction permettant de créer la courbe, en remplissant 
    * le vecteur de points Points
    * @param P vecteur contenant les points de contrôle de la courbe
    * nécessaires à sa construction
    */
    void trace(std::vector<PointGL*> P);
    
    /**
    * @brief accesseur en écriture au vecteur de points contenant les points de la courbe
    * @param s coefficient de la courbe (en générale, égale à 1/2)
    */
    void setInds(float s);;
    
    /**
    * @brief Fonction permettant l'affichage de la courbe
    */
    void affichage();
    
    /**
    * @brief accesseur en lecture au vecteur de points contenant les points de la courbe
    * @return le vecteur de points Points
    */
    std::vector<PointGL*> getP();
    
private:
    /**
    * @brief Fonction permettant le calcul des abscisses pour chaque point
    * @param t instant t, avancement de la courbe
    * @param p1 premier point de contrôle nécessaire au calcul
    * @param p2 second point de contrôle nécessaire au calcul
    * @param p3 troisième point de contrôle nécessaire au calcul
    * @param p4 quatrième point de contrôle nécessaire au calcul
    * @return l'abscisse du point
    */
    float FuncX(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4);
    
    /**
    * @brief Fonction permettant le calcul des ordonnées pour chaque point
    * @param t instant t, avancement de la courbe
    * @param p1 premier point de contrôle nécessaire au calcul
    * @param p2 second point de contrôle nécessaire au calcul
    * @param p3 troisième point de contrôle nécessaire au calcul
    * @param p4 quatrième point de contrôle nécessaire au calcul
    * @return l'ordonnée du point
    */
    float FuncY(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4);
    
    /**
    * @brief Fonction permettant le calcul de la profondeur pour chaque point
    * @param t instant t, avancement de la courbe
    * @param p1 premier point de contrôle nécessaire au calcul
    * @param p2 second point de contrôle nécessaire au calcul
    * @param p3 troisième point de contrôle nécessaire au calcul
    * @param p4 quatrième point de contrôle nécessaire au calcul
    * @return la profondeur du point
    */
    float FuncZ(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4);
    
    /**
    * @brief Fonction permettant le calcul de la normale pour chaque point
    * @param t instant t, avancement de la courbe
    * @param p1 premier point de contrôle nécessaire au calcul
    * @param p2 second point de contrôle nécessaire au calcul
    * @param p3 troisième point de contrôle nécessaire au calcul
    * @param p4 quatrième point de contrôle nécessaire au calcul
    * @return le vecteur tangent
    */
    std::vector<float> FuncNormale(float t, PointGL* p1, PointGL* p2, PointGL* p3, PointGL* p4);
    
    //coefficient de la courbe (en générale, égale à 1/2)
    float inds;
    
    //nombre de points sur la courbe
    int nbp;
    
    //Points de la courbe
    std::vector<PointGL*> Points;
};

#endif	/* CATMULLROM_H */

