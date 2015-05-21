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

#ifndef POINT_H
#define	POINT_H
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <stdlib.h>
class PointGL {
public:
    /**
     * Construteur par défaut construisant le point à l'origine du repère
     */
    PointGL();
    /**
     * Construction du point
     * @param x abscisse
     * @param y ordonnée
     * @param z profondeur
     */
    PointGL(float x, float y,float z);
    PointGL(const PointGL& orig);

    bool operator==(PointGL const& a);
    virtual ~PointGL();
    /**
     * Récupération de l'abscisse du point
     * @return  abscisse du point 
     */
    float getX()const;
    /**
     * Récupération de l'ordonnée du point
     * @return  ordonnée du point 
     */
    float getY()const;
    /**
     * Récupération de la profondeur du point
     * @return  profondeur du point
     */
    float getZ()const;
    /**
     * Modification de l'abscisse du point
     * @param a nouvel abscisse
     */
     void setX(float a);
      /**
     * Modification de l'ordonné du point
     * @param a nouvel ordonné
     */
    void setY(float a);
     /**
     * Modification de la prodondeur du point
     * @param a nouvelle prodondeur
     */
    void setZ(float a);
    
    /**
     * Modifie la valeur de la normal moyenne  en ce point
     * @param n normale moyenne à affecter
     */
    void setNormalMoyenne(std::vector<float>n);
    /**
     * Modifie la valeur de la normal moyenne  en ce point
     * @param x 
     * @param y
     * @param z
     */
    void setNormalMoyenne(float x, float y, float z);
    /**
     * Récupération du vecteur normal à la face
     * @return le vecteur normal 
     */
    std::vector<float> getNormal()const;
    /**
     * Opération de translation
     * @param x
     * @param y
     * @param z
     */
    void translation(float x,float y,float z);
    /**
     * Opération de rotation 
     * @param angle angle de rotation
     * @param x axe de rotation 
     * @param y axe de rotation 
     * @param z axe de rotation 
     */
    void rotation(float a, int x,int y,int z);
    /**
     * Opération de mise à l'échelle
     * @param x
     * @param y
     * @param z
     */
    void scale(float x,float y,float z);

    /**
     * @brief truncation rounded to trunc th
     * @param trun significant figure guard, power of 10
     * @param f float to be truncated
     * @return the float truncated
     */
    float truncation(int trunc,float f);
    int getEpsilon() const;
    /**
     * @brief setEpsilon setter
     * @param value  Significant figure guard, power of 10(10 or 100 or 1000...)
     */
    void setEpsilon(int value);
    /**
     * @brief equals2D test if points are equals in 2D ( without z coordinate)
     * @param a point to be tested
     * @return true if points are similars
     */
    bool equals2D(const PointGL a) const;
private:
    float rad;
    float x;
    float y;
    float z;
    int epsilon;///< use for tested if float are equals. Significant figure guard, power of 10
    std::vector<float>normalMoyenne;
};

#endif	/* POINT_H */

