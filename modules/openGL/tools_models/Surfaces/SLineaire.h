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

#ifndef SLINEAIRE_H
#define	SLINEAIRE_H
#include "../Point.h"
#include <math.h>
#include "GL/glut.h"
#include <cmath>
#include <iostream>
#include <string>
#include "GL/freeglut_std.h"
#include <vector>
#include "Surface.h"

class SLineaire:public Surface {
public:
    SLineaire(std::vector<PointGL*> c, int n, PointGL * p1, PointGL * p2);
    SLineaire(const SLineaire& orig);
    virtual ~SLineaire();
    void Creation();
    //void Affichage();
    void construction();
    float InterpoLineaire(float t, float p0, float p1);
private:
    //points représentants la droite
    PointGL* p0;
    PointGL* p1;
};

#endif	/* SLINEAIRE_H */

