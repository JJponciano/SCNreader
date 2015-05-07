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

#ifndef SCENE_H
#define	SCENE_H

#include "Materiel.h"
#include <iostream>
#include <string>

#include <stdlib.h>
#include <GL/glut.h>
#include <GL/freeglut_std.h>
#include <cmath>
#include <QEvent>
#include <QMouseEvent>
#include <QtOpenGL>
#include <QGLWidget>
class Scene {
public:
    /**
     * Constructeur
     */
    Scene();
    Scene(const Scene& orig);
    /**
     * Initialisation des paramètres glut et OpenGL
     */
    void initialisation();
    /**
     * Traitement des actions effectué avec le clavier
     * @param touche touche pressée
     */
    void clavier(int touche);
    /**
     * @brief mouse
     * @param button bouton utilisé ( 1=>gauche 2=> droit)
     * @param state
     * @param x
     * @param y
     */
    void mouseClicked(QMouseEvent *event);
    /**
     * Action continue de la souris 
     * @param x abscisse du curseur de la souris
     * @param y ordonnée  du curseur de la souris
     */
    void mousemotion(int x, int y);

    /**
     * Affichage minimaliste de la scène
     */
    void affichage();
    /**
     * Prend en compte ou non l'éclairage de la scène.
     * @param e true si prise ne compte de l'éclairage, false sinon.
     */
    void eclairage(bool e);
    /**
     * affichage des lumière
     */
    void lumiere();
    /**
     * Affichage d'un spot
     */
    void spot(GLenum lum);
    virtual ~Scene();
    /**
     * Affichage du grille au sol pour faciliter la construction d'objets
     */
    void grille()const;
    /**
     * Interpolation par image cré avec la caméra
     * @param px abscisse de la position de l'image clé de la caméra
     * @param py ordonnée de la position de l'image clé de la caméra
     * @param pz profondeur de la position de l'image clé de la caméra
     * @param lx abscisse de la position de l'image clé de la caméra
     * @param ly ordonnée de la position de l'image clé de la caméra
     * @param lz profondeur de la position de l'image clé de la caméra
     * @param zoome zoome de la caméra
     * @param a avancement de l'interpolation linéaire
     */
    void animationLineaire(float px, float py, float pz, float lx,float ly, float lz, float zoome,float a);

    /**
     * Gestion des déplacement
     * @param int touche du clavier indiquant le choix du déplacement
     * 1 => en avant
     * 2 => en arrière
     * 3 => à droite
     * 4 => à gauche
     * 5 => augmentation de la hauteur de la caméra
     * 6 => diminution de la hauteur de la caméra
     * 7 => Zoom
     * 8 =>Dezoom
     */
    void deplacement(int touche);
    /**
     * @brief setCameraLocation permet d'affecter de nouvelles coordonnées à la caméra
     * @param x coordonnée en X
     * @param y coordonnée en Y
     * @param z coordonnée en Z
     */
    void setCameraLocation(int x, int y,int z);
    /**
     * @brief setCameraLookAt permet d'affecter de nouvelles coordonnées au point de visé de la caméra
     * @param x coordonnée en X
     * @param y coordonnée en Y
     * @param z coordonnée en Z
     */
    void setCameraLookAt(int x, int y,int z);

    /**
     * @brief getPosx
     * @return position en x de la caméra
     */
    float  getPosx();
    /**
     * @brief getPosy
     * @return position en y de la caméra
     */
    float  getPosy();
    /**
     * @brief getPosz
     * @return position en z de la caméra
     */
    float  getPosz();
    /**
     * @brief getLookx
     * @return position en x du point de visé de la caméra
     */
    float  getLookx();
    /**
     * @brief getLooky
     * @return  position en y du point de visé de la caméra
     */
    float  getLooky();
    /**
     * @brief getLookz
     * @return  position en z du point de visé de la caméra
     */
    float  getLookz();

    void zoom();
    void dezoom();
    void reset();
private:
    /**
     * fait un pas en avant ou en arriere dans la trajectoire de visé
     * dep =1 pour avancer, -1 pour reculer
     */
    void calculedeplacement(float dep);
    /**
     * Déplacement de la caméra en arrière
     */
    void reculer();
    /**
     * Déplacement de la caméra en avant
     */
    void avancer();
    /**
     * Déplacement de la caméra à gauche
     */
    void gauche();
    /**
     * Déplaceent de la caméra à droite 
     */
    void droite();
    /**
     * Affichage d'un repère
     */
    void repere();
    /**
     * Calcule les rotations de caméra
     */
    void calculeRotation();
    float ratio;
    //possition de la camera 
    float posx;
    float posy;
    float posz;
    //point de visé de la camera 
    float lookX;
    float lookY;
    float lookZ;
    //angle de visé de la camera 
    float aX;
    float aY;
    float aZ;
    //angle de Zoome de la caméra
    float angleZoome;
    // vitesse de déplacement de la caméra
    int vit;
    // si bouton droit de la souris pressé
    bool droit;
    // si un bouton de la souris est pressé
    int presse;
    // possion curseur de la souris
    int x;
    int y;
    int xold;
    int yold;
    int anglex, angley;
    int ortho;
    Materiel mat;
    int largeurFenetre;
    int hauteurFenetre;
    //rayon de la sphère de visé
    float rvueSimple;
    //===========================
    float projx;
    float projy;
    float projz;
    float expx;
    float expy;
    float expz;
    
};

#endif	/* SCENE_H */

