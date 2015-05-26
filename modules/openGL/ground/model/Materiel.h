/**
*  @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
* All rights reserved.
* This file is part of scn reader.
*
* scn reader is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* scn reader is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar.  If not, see <http://www.gnu.org/licenses/>
* @author Jean-Jacques PONCIANO and Claire PRUDHOMME
* Contact: ponciano.jeanjacques@gmail.com
* @version 0.1
*/

#ifndef MATERIEL_H
#define	MATERIEL_H
#include <stdlib.h>
#include "GL/glut.h"
#include "GL/freeglut_std.h"
#include "GL/freeglut.h"
class Materiel {
public:
    /**
     * Constructeur
     */
    Materiel();
    Materiel(const Materiel& orig);
    virtual ~Materiel();
    /**
     * Charge un matériau
     */
    void carrelage();
    /**
     * Charge un matériau
     */
    void emerald();

    /**
     * Charge un matériau
     */
    void jade();
    /**
     * Charge un matériau
     */ 
    void obsidian();
    /**
     * Charge un matériau
     */ 
    void pearl();
    /**
     * Charge un matériau
     */
    void ruby();
    /**
     * Charge un matériau
     */ 
    void turquoise();
    /**
     * Charge un matériau
     */ 
    void brass();
    /**
     * Charge un matériau
     */ 
    void bronze();
    /**
     * Charge un matériau
     */ 
    void chrome();
    /**
     * Charge un matériau
     */ 
    void copper();
    /**
     * Charge un matériau
     */ 
    void gold();
    /**
     * Charge un matériau
     */ 
    void silver();
    /**
     * Charge un matériau
     */ 
    void blackplastic();
    /**
     * Charge un matériau
     */ 
    void cyanplastic();
    /**
     * Charge un matériau
     */ 
    void greenplastic();
    /**
     * Charge un matériau
     */ 
    void redplastic();
    /**
     * Charge un matériau
     */ 
    void whiteplastic();
    /**
     * Charge un matériau
     */ 
    void yellowplastic();
    /**
     * Charge un matériau
     */ 
    void blackrubber();
    /**
     * Charge un matériau
     */ 
    void cyanrubber();
    /**
     * Charge un matériau
     */ 
    void greenrubber();
    /**
     * Charge un matériau
     */ 
    void redrubber();
    /**
     * Charge un matériau
     */ 
    void whiterubber();
    /**
     * Charge un matériau
     */ 
    void yellowrubber();
    /**
     * Charge un matériau
     */
    void selectMateriel(int i);
    /**
     * Charge un matériau
     */ 
    void boisClaire();
    /**
     * Charge un matériau
     */ 
    void peinture();
    /**
     * Charge un matériau
     */ 
    void BoisFonce();
    /**
     * Charge un matériau
     */ 
    void vitre();
    /**
     * Charge un matériau
     */ 
    void porcelaine();
private:

};

#endif	/* MATERIEL_H */

