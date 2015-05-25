#ifndef MODEL_GROUND_GL_H
#define MODEL_GROUND_GL_H

#include "GLWidget.h"
#include "Scene.h"
#include "LesTextures.h"
#include <QVector>
#include "../../../exceptions/erreur.h"
class model_ground_GL
{
public:
    model_ground_GL();
    ~model_ground_GL();
    void resizeGL(int ratio, int height);
    void initializeGL();
    void paintGL();
    /**
     * @brief addTexture load a new texture from a "pgn" format file and add it.
     * @param path path of localisation of the texture
     * @param nom assigned name has the texture
     */
    void addTexture(QString path, QString nom);
    /**
    * @brief getTexture get the texture of which his name is specified
    * @param name specific texture name
    * @return the texture which his name is specified
    */
    Texture* getTexture(QString name);
    /**
    * @brief getZoom getter of zoom
    * @return zoom
    */
    float getZoom();
    /**
    * @brief setZoom setter of zoom
    * @param newZoom the new zoom that will be affected
    */
    void  setZoom(float newZoom);
    /**
     * @brief setCameraLocation set the new position of the camera
     * @param x
     * @param y
     * @param z
     */
    void setCameraLocation(float x, float y,float z);
    /**
     * @brief setCameraLookAt set the new look at of the camera
     * @param x
     * @param y
     * @param z
     */
    void setCameraLookAt(float x,float y,float z);

    void initDisplay();
    Scene scene;
private:

    //angle de Zoome de la caméra
    float angleZoome;
    LesTextures *lt;
    float scale;

};

#endif // MODEL_GROUND_GL_H
