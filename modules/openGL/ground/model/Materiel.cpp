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

#include "Materiel.h"

Materiel::Materiel() {
}

Materiel::Materiel(const Materiel& orig) {
}

void Materiel::emerald() {
    GLfloat amb[] = {0.0215, 0.1745, 0.0215, 1.0};
    GLfloat diff[] = {0.07568, 0.61424, 0.07568, 1.0};
    GLfloat spec[] = {0.633, 0.727811, 0.633, 1.0};
    float e = 0.6;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::jade() {
    GLfloat amb[] = {0.135, 0.2225, 0.1575, 1.0};
    GLfloat diff[] = {0.54, 0.89, 0.63, 1.0};
    GLfloat spec[] = {0.316228, 0.316228, 0.316228, 1.0};
    float e = 0.1;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::obsidian() {
    GLfloat amb[] = {0.05375, 0.05, 0.06625, 1.0};
    GLfloat diff[] = {0.18275, 0.17, 0.22525, 1.0};
    GLfloat spec[] = {0.332741, 0.328634, 0.346435, 1.0};
    float e = 0.3;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::carrelage() {
    float MatSpec[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float MatDif[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float MatAmb[4] = {0.3f, 0.3f, 0.3f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, MatSpec); //On applique les paramètres du matériau
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MatDif);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MatAmb);
}

void Materiel::pearl() {
    GLfloat amb[] = {0.25, 0.20725, 0.20725, 1.0};
    GLfloat diff[] = {1, 0.829, 0.829, 1.0};
    GLfloat spec[] = {0.296648, 0.296648, 0.296648, 1.0};
    float e = 0.088;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::ruby() {
    GLfloat amb[] = {0.1745, 0.01175, 0.01175, 1.0};
    GLfloat diff[] = {0.61424, 0.04136, 0.04136, 1.0};
    GLfloat spec[] = {0.727811, 0.626959, 0.626959, 1.0};
    float e = 0.6;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::turquoise() {
    GLfloat amb[] = {0.1, 0.18725, 0.1745, 1.0};
    GLfloat diff[] = {0.396, 0.74151, 0.69102, 1.0};
    GLfloat spec[] = {0.297254, 0.30829, 0.306678, 1.0};
    float e = 0.1;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::brass() {
    GLfloat amb[] = {0.329412, 0.223529, 0.027451, 1.0};
    GLfloat diff[] = {0.780392, 0.568627, 0.113725, 1.0};
    GLfloat spec[] = {0.992157, 0.941176, 0.807843, 1.0};
    float e = 0.21794872;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::bronze() {
    GLfloat amb[] = {0.2125, 0.1275, 0.054, 1.0};
    GLfloat diff[] = {0.714, 0.4284, 0.18144, 1.0};
    GLfloat spec[] = {0.393548, 0.271906, 0.166721, 1.0};
    float e = 0.2;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::chrome() {
    GLfloat amb[] = {0.25, 0.25, 0.25, 1.0};
    GLfloat diff[] = {0.4, 0.4, 0.4, 1.0};
    GLfloat spec[] = {0.774597, 0.774597, 0.774597, 1.0};
    float e = 0.6;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::copper() {
    GLfloat amb[] = {0.19125, 0.0735, 0.0225, 1.0};
    GLfloat diff[] = {0.7038, 0.27048, 0.0828, 1.0};
    GLfloat spec[] = {0.256777, 0.137622, 0.086014, 1.0};
    float e = 0.1;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::gold() {
    GLfloat amb[] = {0.24725, 0.1995, 0.0745, 1.0};
    GLfloat diff[] = {0.75164, 0.60648, 0.22648, 1.0};
    GLfloat spec[] = {0.628281, 0.555802, 0.366065, 1.0};
    float e = 0.4;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::silver() {
    GLfloat amb[] = {0.19225, 0.19225, 0.19225, 1.0};
    GLfloat diff[] = {0.50754, 0.50754, 0.50754, 1.0};
    GLfloat spec[] = {0.508273, 0.508273, 0.508273, 1.0};
    float e = 0.4;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::blackplastic() {
    GLfloat amb[] = {0.0, 0.0, 0.0, 1.0};
    GLfloat diff[] = {0.01, 0.01, 0.01, 1.0};
    GLfloat spec[] = {0.50, 0.50, 0.50, 1.0};
    float e = 0.25;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::cyanplastic() {
    GLfloat amb[] = {0.0, 0.1, 0.06, 1.0};
    GLfloat diff[] = {0.0, 0.50980392, 0.50980392, 1.0};
    GLfloat spec[] = {0.50196078, 0.50196078, 0.50196078, 1.0};
    float e = 0.25;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::greenplastic() {
    GLfloat amb[] = {0.0, 0.0, 0.0, 1.0};
    GLfloat diff[] = {0.1, 0.35, 0.1, 1.0};
    GLfloat spec[] = {0.45, 0.55, 0.45, 1.0};
    float e = 0.25;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::redplastic() {
    GLfloat amb[] = {0.0, 0.0, 0.0, 1.0};
    GLfloat diff[] = {0.5, 0.0, 0.0, 1.0};
    GLfloat spec[] = {0.7, 0.6, 0.6, 1.0};
    float e = 0.25;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::whiteplastic() {
    GLfloat amb[] = {0.0, 0.0, 0.0, 1.0};
    GLfloat diff[] = {0.55, 0.55, 0.55, 1.0};
    GLfloat spec[] = {0.70, 0.70, 0.70, 1.0};
    float e = 0.25;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::yellowplastic() {
    GLfloat amb[] = {0.0, 0.0, 0.0, 1.0};
    GLfloat diff[] = {0.5, 0.5, 0.0, 1.0};
    GLfloat spec[] = {0.60, 0.60, 0.50, 1.0};
    float e = 0.25;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::blackrubber() {
    GLfloat amb[] = {0.02, 0.02, 0.02, 1.0};
    GLfloat diff[] = {0.01, 0.01, 0.01, 1.0};
    GLfloat spec[] = {0.4, 0.4, 0.4, 1.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::cyanrubber() {
    GLfloat amb[] = {0.0, 0.05, 0.05, 1.0};
    GLfloat diff[] = {0.4, 0.5, 0.5, 1.0};
    GLfloat spec[] = {0.04, 0.7, 0.7, 1.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::greenrubber() {
    GLfloat amb[] = {0.0, 0.05, 0.0, 1.0};
    GLfloat diff[] = {0.4, 0.5, 0.4, 1.0};
    GLfloat spec[] = {0.04, 0.7, 0.04, 1.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::redrubber() {
    GLfloat amb[] = {0.05, 0.0, 0.0, 1.0};
    GLfloat diff[] = {0.5, 0.4, 0.4, 1.0};
    GLfloat spec[] = {0.7, 0.04, 0.044, 1.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::whiterubber() {
    GLfloat amb[] = {0.05, 0.05, 0.05, 1.0};
    GLfloat diff[] = {0.5, 0.5, 0.5, 1.0};
    GLfloat spec[] = {0.7, 0.7, 0.7, 1.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::yellowrubber() {
    GLfloat amb[] = {0.05, 0.05, 0.0, 1.0};
    GLfloat diff[] = {0., 0.5, 0.4, 1.0};
    GLfloat spec[] = {0.7, 0.7, 0.04, 1.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::boisClaire() {
   float MatSpec[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float MatDif[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float MatAmb[4] = {0.8f, 0.8f, 0.8f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, MatSpec); //On applique les paramètres du matériau
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MatDif);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MatAmb);
}

void Materiel::peinture() {
    float MatSpec[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float MatDif[4] = {0.8f, 0.8f, 0.6f, 1.0f};
    float MatAmb[4] = {0.3f, 0.3f, 0.3f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, MatSpec); //On applique les paramètres du matériau
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MatDif);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MatAmb);
}

void Materiel::BoisFonce() {
  GLfloat amb[] = {0.2125, 0.1275, 0.054, 1.0};
    GLfloat diff[] = {0.214, 0.1284, 0.08144, 1.0};
    GLfloat spec[] = {0.393548, 0.271906, 0.166721, 1.0};
    float e = 0.2;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}
void Materiel::vitre() {
   GLfloat amb[] = {1, 0.05, 0.0, 0.0};
    GLfloat diff[] = {0., 0.5, 0.4, 0.0};
    GLfloat spec[] = {0.7, 0.7, 0.04, 0.0};
    float e = 0.078125;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::porcelaine() {
    GLfloat amb[] = {0.8125, 0.8125, 0.8125, 1.0};
    GLfloat diff[] = {0.814, 0.814, 0.814, 1.0};
    GLfloat spec[] = {1,1,1, 1.0};
    float e = 0.2;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0);
}

void Materiel::selectMateriel(int i) {
    switch (i) {
        case 0:
            this->emerald();
            break;
        case 1: jade();
            break;
        case 2: obsidian();
            break;
        case 3: pearl();
            break;
        case 4: ruby();
            break;
        case 5: turquoise();
            break;
        case 6: brass();
            break;
        case 7: bronze();
            break;
        case 8: chrome();
            break;
        case 9: copper();
            break;
        case 10: gold();
            break;
        case 11: silver();
            break;
        case 12: blackplastic();
            break;
        case 13: cyanplastic();
            break;
        case 14: greenplastic();
            break;
        case 15: redplastic();
            break;
        case 16: whiteplastic();
            break;
        case 17: yellowplastic();
            break;
        case 18: blackrubber();
            break;
        case 19: cyanrubber();
            break;
        case 20: greenrubber();
            break;
        case 21: redrubber();
            break;
        case 22: whiterubber();
            break;
        case 23: yellowrubber();
            exit(0);
    }
}

Materiel::~Materiel() {
}

