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

#ifndef FACE_H
#define	FACE_H
#include <iostream>
#include <vector>
#include "Point.h"
#include "Couleur.h"
#include "Texel.h"
#include <iostream>
#include <cmath>
#include "GL/glut.h"
#include <stdlib.h>
#include "GL/freeglut_std.h"
class Face {
public:
    /**
     * Constructeur
     */
    Face();
    /**
     * Constructeur de copie
     * @param orig face à copier
     * @return renvoie la face copiée
     */
    Face(const Face& orig);
    /**
     * Ajout un point à la face 
     * @param p point à ajouter
     */
    void add(PointGL* p,Texel*t);
    /**
     * Supprime le dernier point ajouté à la face
     */
   
    void remove ();
    /**
     * Affecte une couleur à la face 
     * @param r canal rouge
     * @param g canal vert
     * @param b canal bleu
     */
     void setCouleur(float r, float g,float b);
     /**
     * Affecte une couleur à la face 
     * @param c couleur 
     */
     void setCouleur(const Couleur& c);
    /**
     * Affiche la face en couleur 
     * @code 
     * //affichage avec lissage de phong:
     *  afficher(true);
     * @endcode
     * @param lisage affiche avec ou sans le lissage 
     */
    void afficher(bool lisage);
    /**
     * Affiche les points de la face
     */
    void afficherPoints();
    /**
     * Test si le point p appartient à la face
     * @param p point à tester 
     * @return vrai si le point appartient à la face, faux sinon
     */
    bool contain(PointGL* p)const;
    /**
     * Récupération du vecteur normal à la face
     * @return le vecteur normal à la face
     */
    std::vector<float> getNormal()const;
    virtual ~Face();
private:
    /**
     * Transforme un vecteur en vecteur unitaire
     * @param vecteur à transformer
     */
    void Vecteur_Unite(std::vector<float>);
    std::vector<float>normal;
    /**
     * Calcule des normals
     */
    void Normale();
     std::vector<PointGL*> points;
      std::vector<Texel*> texels;
     Couleur* couleur;

};

#endif	/* FACE_H */

