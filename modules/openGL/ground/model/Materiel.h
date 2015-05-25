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

