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

#ifndef SREVOLUTION_H
#define	SREVOLUTION_H
#include "../Point.h"
#include <math.h>
#include "GL/glut.h"
#include <cmath>
#include <iostream>
#include <string>
#include "GL/freeglut_std.h"
#include <vector>
#include "Surface.h"

class SRevolution : public Surface {
public:
    SRevolution(std::vector<PointGL*> c, int n);
    SRevolution(const SRevolution& orig);
    virtual ~SRevolution();
    void creation();
private:
    //Constante PI
    float pi;
};

#endif	/* SREVOLUTION_H */

