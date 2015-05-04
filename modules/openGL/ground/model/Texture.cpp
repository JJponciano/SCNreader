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
#include "Texture.h"

Texture::Texture(QString adresse, std::string nom, GLuint id) {
    this->adresse = adresse;
    this->nom = nom;
    this->id = id;
    this->largimg = 256;
    this->hautimg = 256;
    // pour jpeg
    //this->construction();
    //pour png
    loadTexture();

}

Texture::Texture(const Texture& orig) {
    this->adresse = orig.adresse;
    this->nom = orig.nom;
    this->id = orig.id;
    // pour jpeg
    //this->construction();
    //pour png
    loadTexture();
}

void Texture::charger() const {
      glBindTexture(GL_TEXTURE_2D, id);
}
void Texture::loadTexture()
{
    QImage qim_Texture;
    QImage qim_TempTexture;
    qim_TempTexture.load(adresse);
    qim_Texture = QGLWidget::convertToGLFormat( qim_TempTexture );
    glBindTexture( GL_TEXTURE_2D, id );
    glTexImage2D( GL_TEXTURE_2D, 0, 3, qim_Texture.width(), qim_Texture.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, qim_Texture.bits() );
}
/*si pas Qt
void Texture::construction() {

    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE *file;
    unsigned char *ligne;


    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    file = fopen(this->adresse.c_str(), "rb");
    if (file == NULL) {
        fprintf(stderr, "Erreur : impossible d'ouvrir le fichier de texture\n");
    }
    jpeg_stdio_src(&cinfo, file);
    jpeg_read_header(&cinfo, TRUE);
    unsigned int temp = largimg;
    if ((cinfo.image_width != temp) || (cinfo.image_height != temp)) {
        fprintf(stdout, "Erreur : l'image doit etre de taille 256x256\n");
    }
    if (cinfo.jpeg_color_space == JCS_GRAYSCALE) {
        fprintf(stdout, "Erreur : l'image doit etre de type RGB\n");
    }

    jpeg_start_decompress(&cinfo);
    ligne = image;
    while (cinfo.output_scanline < cinfo.output_height) {
        ligne = image + 3 * largimg * cinfo.output_scanline;
        jpeg_read_scanlines(&cinfo, &ligne, 1);
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    int i, j;

    // creation de la texture en fonction de l'image diagonale entre deux
    for (i = 0; i < largimg; i++)
        for (j = 0; j < hautimg; j++) {
            texture1[i][j][0] = image[i * largimg * 3 + j * 3];
            texture1[i][j][1] = image[i * largimg * 3 + j * 3 + 1];
            texture1[i][j][2] = image[i * largimg * 3 + j * 3 + 2];
        }

    //================================================chargement
    //lier texture 1
    glBindTexture(GL_TEXTURE_2D, id);
    

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, largimg, hautimg, 0, GL_RGB, GL_UNSIGNED_BYTE, texture1);
}*/

std::string Texture::getNnom() const {
    return this->nom;
}

GLuint Texture::getId() const {
    return this->id;
}

Texture::~Texture() {
}

