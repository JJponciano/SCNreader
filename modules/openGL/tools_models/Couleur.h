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
#ifndef COULEUR_H
#define	COULEUR_H

class Couleur {
public:
    /**
     * Construit une couleur blanche
     */
    Couleur();
    /**
     * Construction de la couleur rgb
     * @param r canal rouge
     * @param g canal vert
     * @param b canal bleu
     */
      Couleur(float r, float g,float b);
    Couleur(const Couleur& orig);
    virtual ~Couleur();
    float getR()const;
    float getG()const;
    float getB()const;
private:
    float r;
    float g;
    float b;
};

#endif	/* COULEUR_H */

