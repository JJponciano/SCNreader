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

#ifndef SURFACE_H
#define	SURFACE_H
#include "../Point.h"
#include <math.h>
#include "GL/glut.h"
#include <cmath>
#include <iostream>
#include <string>
#include "GL/freeglut_std.h"
#include <vector>
#include "../Objet3D.h"
class Surface: public Objet3D  {
public:
    Surface(std::vector<PointGL*> c, int n);
    Surface(const Surface& orig);
    virtual ~Surface();
    void creation();
    void construction();
    void affichage();
    
protected:
    //Tableau contenant les points de la courbe
    std::vector<PointGL*> Courbe;
    //tableau contenant tous les points de la surfaces
    //nombre de points selon la latitude
    int nbl;
    //nombre de points selon la longitude
    int nbc;
};

#endif	/* SURFACE_H */

