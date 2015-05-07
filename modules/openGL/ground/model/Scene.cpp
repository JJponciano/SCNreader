/*
 *
 * @author PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Copyright  2014  PONCIANO Jean-Jacques et PRUDHOMME Claire
 * Contact: ponciano.jeanjacques@gmail.com
 * Créé le 13 Mars 2014
 *
 * Cette oeuvre, création, code, site ou texte est sous licence Creative Commons  Attribution - Pas d’Utilisation Commerciale -
 * Partage dans les Mêmes Conditions 4.0 International. Pour accéder à une copie de cette licence, merci de vous rendre à l'adresse suivante
 * http://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr ou envoyez un courrier à Creative Commons, 444 Castro Street, Suite 900,
 * Mountain View, California, 94041, USA.
 */
#include "Scene.h"
using namespace std;
float pi = 3.14159265;

Scene::Scene() {
    anglex=0;
    angley=0;
    this->ratio = 1;
    //possition de la camera
    posx = 20;
    posy = 2;
    posz = 0;
    //point de visé de la camera
    lookX = -2;
    lookY = 0;
    lookZ = 0;
    ortho = 100;
    aX = 0;
    aY = 0;
    //angle de Zoome de la caméra
    angleZoome = 90;
    // vitesse de déplacement de la caméra
    vit = 1;
    // si bouton droit de la souris pressé
    droit = false;
    // si un bouton de la souris est pressé
    presse = false;
    // possion curseur de la souris
    x = 0;
    y = 0;
    xold = 0;
    yold = 0;

    //rayon de la sphère de visé
    rvueSimple = sqrt(lookX * lookX + lookZ * lookZ);


    //===========================
    projx = posx - 3;
    projy = posy - 8;
    projz = posz - 3;
    expx = projx;
    expy = projy;
    expz = projz;
    this->largeurFenetre = 1248;
    this->hauteurFenetre = 672;
}

Scene::Scene(const Scene& orig) {
}
// initialisation de la fenetre glut


void Scene::eclairage(bool e) {
    if (e) glEnable(GL_LIGHTING);
    else glDisable(GL_LIGHTING);
}

// traitement des actions associées au clavier

void Scene::clavier(int touche) {
    //CAMERA
    this->deplacement(touche);
    switch (touche) {
    case 'a':
        glEnable(GL_LIGHT1);

        break;
    case 'A':
        glDisable(GL_LIGHT1);

        break;
    case 'b':
        glEnable(GL_LIGHT2);

        break;
    case 'B':
        glDisable(GL_LIGHT2);

        break;
    case 'c':
        glEnable(GL_LIGHT3);

        break;
    case 'C':
        glDisable(GL_LIGHT3);
        break;

    case 'f':
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        break;
    case 'F':
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        break;
    case 't': /* Texture*/
        glEnable(GL_TEXTURE_2D);

        break;
    case 'T': /* Texture*/
        glDisable(GL_TEXTURE_2D);

        break;
    }
}


// ========================Traitement des actions souris
//action déclanché par le clique d'un bouton de la souris

void Scene::mouseClicked(QMouseEvent *mousevent) {
    //===============BOUTON PRESSE
    float x=mousevent->x();
    float y=mousevent->y();

    if(mousevent->type()==QEvent::MouseButtonPress){

        /* si on appuie sur le bouton gauche */
        if (mousevent->button() == Qt::LeftButton) {
            presse = true; /* le booleen presse passe a 1 (vrai) */
            xold = x ; /* on sauvegarde la position de la souris */
            yold = y ;
        } else
            /* si on appuie sur le bouton Droit */
            if (mousevent->button() == Qt::RightButton) {
                droit = true;
                xold = x ; /* on sauvegarde la position de la souris */
                yold = y ;
            }
    }else
        //============DEPLACEMENT SOURIS
        if(mousevent->type()== QEvent::MouseMove){
            this->mousemotion(x,y);

        }
    //===============BOUTON RELACHE
        else{
            /* si on relache le bouton gauche */
            if (mousevent->button() == Qt::LeftButton)
                presse = false; /* le booleen presse passe a 0 (faux) */
            else
                /* si on relache le bouton Droit */
                if (mousevent->button() ==Qt::RightButton)
                    droit = false; /* le booleen presse passe a 0 (faux) */
        }


}


// action continue de la souris

void Scene::mousemotion(int x, int y) {
    if (presse)
        /* si le bouton gauche est presse */ {
        /* on modifie les angles de rotation de l'objet
           en fonction de la position actuelle de la souris et de la derniere
           position sauvegardee */
        anglex = anglex + (x - xold);
        angley = angley + (y - yold);
        /* on demande un rafraichissement de l'affichage */
    } else if (droit) /* si le bouton droit est presse */ {
        /* on modifie les angles de rotation de l'objet
           en fonction de la position actuelle de la souris et de la derniere
           position sauvegardee */
        aX += (x - xold);
        lookY -= (y - yold);
        this->calculeRotation();
    }

    xold = x; /* sauvegarde des valeurs courante de le position de la souris */
    yold = y;
}
//affichage minimaliste de la scène

void Scene::affichage() {

    gluLookAt(posx, posy, posz, lookX, lookY, lookZ, 0.0, 1.0, 0.0);

    //création des lumières

    glRotatef(-angley, 1.0, 0.0, 0.0);
    glRotatef(-anglex, 0.0, 1.0, 0.0);
    this->lumiere();

    //création du repère
    repere();
    //grille();
}
void Scene::reset(){
    anglex=0;
    angley=0;
}

Scene::~Scene() {
}
// repère standard

void Scene::repere() {
    //Repère
    //axe x en rouge
    glBegin(GL_LINES);
    this->mat.redplastic();
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0, 0, 0.0);
    glVertex3f(500, 0, 0.0);
    glEnd();
    //axe des y en vert
    glBegin(GL_LINES);
    this->mat.jade();
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0, 0, 0.0);
    glVertex3f(0, 500, 0.0);
    glEnd();
    //axe des z en bleu
    glBegin(GL_LINES);
    this->mat.gold();
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 500);
    glEnd();


    //axe x en majenta
    glBegin(GL_LINES);
    this->mat.redplastic();
    glColor3f(1.0, 0.0, 1.0);
    glVertex3f(0, 0, 0.0);
    glVertex3f(-500, 0, 0.0);
    glEnd();
    //axe des y en jaune
    glBegin(GL_LINES);
    this->mat.jade();
    glColor3f(1.0, 1.0, 0.0);

    glVertex3f(0, 0, 0.0);
    glVertex3f(0, -500, 0.0);
    glEnd();
    //axe des z en bleu
    glBegin(GL_LINES);
    this->mat.gold();
    glColor3f(0.0, 1.0, 1.0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, -500);
    glEnd();
}

void Scene::grille() const {
    for (int i = -50; i < 50; i += 1) {
        for (int k = -50; k < 50; k += 1) {
            glBegin(GL_LINES);
            glColor3f(1, 1, 1);
            glVertex3f(i, 0, k);
            glVertex3f(i + 1, 0, k);
            glEnd();
            glBegin(GL_LINES);
            glColor3f(1, 1, 1);
            glVertex3f(i, 0, k);
            glVertex3f(i, 0, k + 1);
            glEnd();
        }
    }
}

void Scene::animationLineaire(float px, float py, float pz, float lx, float ly, float lz, float zoome, float a) {
    //===============CIBLE===========
    // récupération des coordonnées du point
    float x = this->lookX;
    float y = this->lookY;
    float z = this->lookZ;
    // interpolation linéaire  jusqu'au point clé selon l'avancement a
    x = (1 - a) * x + a * lx;
    y = (1 - a) * y + a * ly;
    z = (1 - a) * z + a * lz;
    // affectation des changements
    this->lookX = x;
    this->lookY = y;
    this->lookZ = z;
    //=============POSITION============
    x = this->posx;
    y = this->posy;
    z = this->posz;
    // interpolation linéaire  jusqu'au point clé selon l'avancement a
    x = (1 - a) * x + a * px;
    y = (1 - a) * y + a * py;
    z = (1 - a) * z + a * pz;
    // affectation des changements
    this->posx = x;
    this->posy = y;
    this->posz = z;
    //=============zoome============
    z = this->angleZoome;
    // interpolation linéaire  jusqu'au point clé selon l'avancement a
    z = (1 - a) * z + a * zoome;
    this->angleZoome=z;
}

void Scene::lumiere() {
    //alumage d'un spot
    glEnable(GL_LIGHT1);

    //definition de la position du spot
    GLfloat position[] = {0.0f, 20.0f, .0f, 1.0f};
    glLightfv(GL_LIGHT1, GL_POSITION, position);


    //======caractéristique de lumière======
    //lumière diffuse
    float Light2Dif[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    glLightfv(GL_LIGHT1, GL_DIFFUSE, Light2Dif);
    //lumière ambiante
    float Light2Amb[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    glLightfv(GL_LIGHT1, GL_AMBIENT, Light2Amb);

    //alumage lampe
    glEnable(GL_LIGHT2);
    glPushMatrix();
    glTranslatef(0, 18, 28);
    glRotatef(40, 1, 0, 0);
    //glutSolidCube(1);
    this->spot(GL_LIGHT2);
    glPopMatrix();

    //alumage d'un spot
    glEnable(GL_LIGHT3);
    glPushMatrix();
    glTranslatef(0, 24, 5);
    this->spot(GL_LIGHT3);
    glPopMatrix();
}

void Scene::spot(GLenum lum) {

    //definition de la position du spot
    GLfloat position[] = {0.0f, 1.0f, 0.0f, 1.0f};
    glLightfv(lum, GL_POSITION, position);
    //direction
    float Light2Dir[3] = {0.0f, -1.0f, 0.0f};
    glLightfv(lum, GL_SPOT_DIRECTION, Light2Dir);

    //======caractéristique de lumière======
    //lumière diffuse
    float Light2Dif[4] = {1.0f, 1.0f, 0.0f, 1.0f};
    glLightfv(lum, GL_DIFFUSE, Light2Dif);

    //lumière spéculaire
    float Light2Spec[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(lum, GL_SPECULAR, Light2Spec);

    //lumière ambiante
    float Light2Amb[4] = {0.5f, 0.5f, 0.0f, 1.0f};
    glLightfv(lum, GL_AMBIENT, Light2Amb);

    //lumière d'ouverture transformant la lumière en spot
    glLightf(lum, GL_SPOT_CUTOFF, 70.0);

    // équavalent au fallof du spot
    glLightf(lum, GL_SPOT_EXPONENT, 30.0);



}

//===============================CAMERA
float Scene::getPosx(){
    return this->posx;
}
float Scene::getPosy(){
    return this->posy;
}
float Scene::getPosz(){
    return this->posz;
}
float Scene::getLookx(){
    return this->lookX;
}
float Scene::getLooky(){
    return this->lookY;
}
float Scene::getLookz(){
    return this->lookZ;
}
void  Scene::zoom(){
    this->angleZoome++;
}
void  Scene::dezoom(){
    this->angleZoome--;
}

//déplacement de la caméra
void Scene::deplacement(int touche) {
    switch (touche) {
    case 1://avancer
        avancer();

        break;
    case 2://reculer
        reculer();

        break;
    case 3:
        droite(); //pas chasse a droite

        break;
    case 4:
        gauche(); //pas chasse a gauche

        break;
    case 5:
        // augmentation de la hauteur de la caméra
        this->posy += 0.1;

        break;
    case 6:
        // diminution de la hauteur de la caméra
        this->posy -= 0.1;

        break;

    case 7:
        //zoom
        this->angleZoome++;
        break;
    case 8:

        //dezoom
        this->angleZoome--;
        break;

    }
}

void Scene::setCameraLocation(int x, int y, int z){
    //affectation des nouvelles coordonnées
    this->posx=x;
    this->posy=y;
    this->posz=z;
}
void Scene::setCameraLookAt(int x, int y, int z){
    this->lookX=x;
    this->lookY=y;
    this->lookZ=z;
}

void Scene::calculeRotation() {
    // ====================rotation horizontal====================
    //calcule du rayon
    float ray = sqrt((lookX - posx)*(lookX - posx)+(lookZ - posz)*(lookZ - posz));
    //calcule des coordonnées du nouveau point
    this->lookX = ray * cos((aX / 360) * pi) + this->posx;
    this->lookZ = ray * sin((aX / 360) * pi) + this->posz;
}


/**
 * fait un pas en avant ou en arriere dans la trajectoire de visé
 * dep =1 pour avancer, -1 pour reculer
 */
void Scene::calculedeplacement(float dep) {
    float t = 0.1 * dep;
    // interpolation linéaire entre la position et le points visée
    float newx = t * lookX + (1 - t) * posx;
    float newz = t * lookZ + (1 - t) * posz;
    // la distance entre px et lx, réciproquement entre pz et lz s'est réduite
    // On calcule de combient pour la combler
    lookX += newx - posx;
    lookZ += newz - posz;
    posz = newz;
    posx = newx;
}

void Scene::reculer() {
    calculedeplacement(-1);
}

void Scene::avancer() {
    calculedeplacement(1);
}

void Scene::gauche() {
    //rotation vectoriel
    float newx = posx, newz = posz;
    newx -= (this->lookX / 9.0) * cos(3.14159265 / 2.0) - (this->lookZ / 9.0) * sin(
                3.14159265 / 2.0);
    newz -= (this->lookX / 9.0) * sin(3.14159265 / 2.0) - (this->lookZ / 9.0) * cos(
                3.14159265 / 2.0);


    posz = newz;
    posx = newx;
}

void Scene::droite() {
    //rotation vectoriel
    float newx = posx, newz = posz;

    newx += (this->lookX / 9.0) * cos(3.14159265 / 2.0) - (this->lookZ / 9.0) * sin(
                3.14159265 / 2.0);
    newz += (this->lookX / 9.0) * sin(3.14159265 / 2.0) - (this->lookZ / 9.0) * cos(
                3.14159265 / 2.0);


    posz = newz;
    posx = newx;
}
