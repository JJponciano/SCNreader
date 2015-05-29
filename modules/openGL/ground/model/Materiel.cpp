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
    GLfloat amb[] = {0.0215f, 0.1745f, 0.0215f, 1.0f};
    GLfloat diff[] = {0.07568f, 0.61424f, 0.07568f, 1.0f};
    GLfloat spec[] = {0.633f, 0.727811f, 0.633f, 1.0f};
    float e = 0.6f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::jade() {
    GLfloat amb[] = {0.135f, 0.2225f, 0.1575f, 1.0f};
    GLfloat diff[] = {0.54f, 0.89f, 0.63f, 1.0f};
    GLfloat spec[] = {0.316228f, 0.316228f, 0.316228f, 1.0f};
    float e = 0.1;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::obsidian() {
    GLfloat amb[] = {0.05375f, 0.05f, 0.06625f, 1.0f};
    GLfloat diff[] = {0.18275f, 0.17f, 0.22525f, 1.0f};
    GLfloat spec[] = {0.332741f, 0.328634f, 0.346435f, 1.0f};
    float e = 0.3f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
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
    GLfloat amb[] = {0.25f, 0.20725f, 0.20725f, 1.0f};
    GLfloat diff[] = {1.0f, 0.829f, 0.829f, 1.0f};
    GLfloat spec[] = {0.296648f, 0.296648f, 0.296648f, 1.0f};
    float e = 0.088f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::ruby() {
    GLfloat amb[] = {0.1745f, 0.01175f, 0.01175f, 1.0f};
    GLfloat diff[] = {0.61424f, 0.04136f, 0.04136f, 1.0f};
    GLfloat spec[] = {0.727811f, 0.626959f, 0.626959f, 1.0f};
    float e = 0.6f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::turquoise() {
    GLfloat amb[] = {0.1f, 0.18725f, 0.1745f, 1.0f};
    GLfloat diff[] = {0.396f, 0.74151f, 0.69102f, 1.0f};
    GLfloat spec[] = {0.297254f, 0.30829f, 0.306678f, 1.0f};
    float e = 0.1f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::brass() {
    GLfloat amb[] = {0.329412f, 0.223529f, 0.027451f, 1.0f};
    GLfloat diff[] = {0.780392f, 0.568627f, 0.113725f, 1.0f};
    GLfloat spec[] = {0.992157f, 0.941176f, 0.807843f, 1.0f};
    float e = 0.21794872f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::bronze() {
    GLfloat amb[] = {0.2125f, 0.1275f, 0.054f, 1.0f};
    GLfloat diff[] = {0.714f, 0.4284f, 0.18144f, 1.0f};
    GLfloat spec[] = {0.393548f, 0.271906f, 0.166721f, 1.0f};
    float e = 0.2f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::chrome() {
    GLfloat amb[] = {0.25f, 0.25f, 0.25f, 1.0f};
    GLfloat diff[] = {0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat spec[] = {0.774597f, 0.774597f, 0.774597f, 1.0f};
    float e = 0.6;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::copper() {
    GLfloat amb[] = {0.19125f, 0.0735f, 0.0225f, 1.0f};
    GLfloat diff[] = {0.7038f, 0.27048f, 0.0828f, 1.0f};
    GLfloat spec[] = {0.256777f, 0.137622f, 0.086014f, 1.0f};
    float e = 0.1f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::gold() {
    GLfloat amb[] = {0.24725f, 0.1995f, 0.0745f, 1.0f};
    GLfloat diff[] = {0.75164f, 0.60648f, 0.22648f, 1.0f};
    GLfloat spec[] = {0.628281f, 0.555802f, 0.366065f, 1.0f};
    float e = 0.4f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::silver() {
    GLfloat amb[] = {0.19225f, 0.19225f, 0.19225f, 1.0f};
    GLfloat diff[] = {0.50754f, 0.50754f, 0.50754f, 1.0f};
    GLfloat spec[] = {0.508273f, 0.508273f, 0.508273f, 1.0f};
    float e = 0.4f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::blackplastic() {
    GLfloat amb[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat diff[] = {0.01f, 0.01f, 0.01f, 1.0f};
    GLfloat spec[] = {0.50f, 0.50f, 0.50f, 1.0f};
    float e = 0.25f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::cyanplastic() {
    GLfloat amb[] = {0.0f, 0.1f, 0.06f, 1.0f};
    GLfloat diff[] = {0.0f, 0.50980392f, 0.50980392f, 1.0f};
    GLfloat spec[] = {0.50196078f, 0.50196078f, 0.50196078f, 1.0f};
    float e = 0.25f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::greenplastic() {
    GLfloat amb[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat diff[] = {0.1f, 0.35f, 0.1f, 1.0f};
    GLfloat spec[] = {0.45f, 0.55f, 0.45f, 1.0f};
    float e = 0.25f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::redplastic() {
    GLfloat amb[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat diff[] = {0.5f, 0.0f, 0.0f, 1.0f};
    GLfloat spec[] = {0.7f, 0.6f, 0.6f, 1.0f};
    float e = 0.25f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::whiteplastic() {
    GLfloat amb[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat diff[] = {0.55f, 0.55f, 0.55f, 1.0f};
    GLfloat spec[] = {0.70f, 0.70f, 0.70f, 1.0f};
    float e = 0.25f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::yellowplastic() {
    GLfloat amb[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat diff[] = {0.5f, 0.5f, 0.0f, 1.0f};
    GLfloat spec[] = {0.60f, 0.60f, 0.50f, 1.0f};
    float e = 0.25f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::blackrubber() {
    GLfloat amb[] = {0.02f, 0.02f, 0.02f, 1.0f};
    GLfloat diff[] = {0.01f, 0.01f, 0.01f, 1.0f};
    GLfloat spec[] = {0.4f, 0.4f, 0.4f, 1.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::cyanrubber() {
    GLfloat amb[] = {0.0f, 0.05f, 0.05f, 1.0f};
    GLfloat diff[] = {0.4f, 0.5f, 0.5f, 1.0f};
    GLfloat spec[] = {0.04f, 0.7f, 0.7f, 1.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::greenrubber() {
    GLfloat amb[] = {0.0f, 0.05f, 0.0f, 1.0f};
    GLfloat diff[] = {0.4f, 0.5f, 0.4f, 1.0f};
    GLfloat spec[] = {0.04f, 0.7f, 0.04f, 1.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::redrubber() {
    GLfloat amb[] = {0.05f, 0.0f, 0.0f, 1.0f};
    GLfloat diff[] = {0.5f, 0.4f, 0.4f, 1.0f};
    GLfloat spec[] = {0.7f, 0.04f, 0.044f, 1.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::whiterubber() {
    GLfloat amb[] = {0.05f, 0.05f, 0.05f, 1.0f};
    GLfloat diff[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat spec[] = {0.7f, 0.7f, 0.7f, 1.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::yellowrubber() {
    GLfloat amb[] = {0.05f, 0.05f, 0.0f, 1.0f};
    GLfloat diff[] = {0.0f, 0.5f, 0.4f, 1.0f};
    GLfloat spec[] = {0.7f, 0.7f, 0.04f, 1.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
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
  GLfloat amb[] = {0.2125f, 0.1275f, 0.054f, 1.0f};
    GLfloat diff[] = {0.214f, 0.1284f, 0.08144f, 1.0f};
    GLfloat spec[] = {0.393548f, 0.271906f, 0.166721f, 1.0f};
    float e = 0.2f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}
void Materiel::vitre() {
   GLfloat amb[] = {1.0f, 0.05f, 0.0f, 0.0f};
    GLfloat diff[] = {0.0f, 0.5f, 0.4f, 0.0f};
    GLfloat spec[] = {0.7f, 0.7f, 0.04f, 0.0f};
    float e = 0.078125f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
}

void Materiel::porcelaine() {
    GLfloat amb[] = {0.8125f, 0.8125f, 0.8125f, 1.0f};
    GLfloat diff[] = {0.814f, 0.814f, 0.814f, 1.0f};
    GLfloat spec[] = {1.0f,1.0f,1.0f, 1.0f};
    float e = 0.2f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, e * 128.0f);
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

