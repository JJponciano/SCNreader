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
#include "SCirculaire.h"
/**
 * @brief Constructeur d'une surface circulaire
 * @param c vecteur de points correspondant au points appartenant à la courbe
 * de référence sur laquelle repose la surface. Exemple:
 * @code
 * Bezier* b=new Bezier(10);
 * b->Trace(vectorPtsControleC, nbp);
 * SCirculaire* sl=new SCirculaire(b.getP(), nbp2, centre);
 * @endcode 
 * @param n nombre de méridien pour la surface
 * @param centre est le centre de la rotation effectuée par la courbe 
 * pour créer la surface
 */
SCirculaire::SCirculaire(std::vector<PointGL*> c, int n, float rayon) : Surface(c, n){
    this->pi=3.14159265;
    this->C=this->Courbe[0];
    this->r=rayon;

    this->Creation();
    this->construction();
}

SCirculaire::SCirculaire(const SCirculaire& orig): Surface(orig){
}

/**
 * @brief destructeur de la classe qui détruit tous les pointeurs
 */
SCirculaire::~SCirculaire() {
    //destruction du tableau des points de la surface
    for (int i = 0; i < this->points.size(); i++) {
        delete this->points[i];
        this->points[i] = 0;
    }
    //destruction du tableau des points de la courbe
    for (int i = 0; i < this->Courbe.size(); i++) {
        delete this->Courbe[i];
        this->Courbe[i] = 0;
    }
}

/**
 * @brief Construction de la surface par ajout des points dans Surface
 * L'ajout se fait selon l'ordre suivant:
 * Pour chaque points de la courbe on donne tous les points du cercle
 * correspondant au niveau du point de la courbe (<=> parallèle par parallèle)
 * méridiens=courbe
 * parallèle=cercle
 */
void SCirculaire::Creation(){
    this->BuildCircle();
 
    for(int j=0; j<this->Courbe.size();j++)
    {
        std::vector <float> n=this->Courbe[j]->getNormal();
        float x=n[0]-this->Courbe[j]->getX();
        float y=n[1]-this->Courbe[j]->getY();
        float z=n[2]-this->Courbe[j]->getZ();
        this->Courbe[j]->setNormalMoyenne(x,y,z);
    }
    
    float incI=1.0/(float)this->Courbe.size();
    float incJ=1.0/(float)2.0*this->pi;
    /*for(int i=0; i<this->cercle.size();i++)
    {
        this->points.push_back(new Point(this->cercle[i]->getX()+this->C->getX(),this->cercle[i]->getY()+this->C->getY(),this->cercle[i]->getZ()+this->C->getZ()));
        this->texels.push_back(new Texel(0,incJ*i));
    }
     */
    for(int j=0; j<this->Courbe.size();j++)
    {
        float phi= this->CalculPhi(this->Courbe[j]);
        
        for(int i=0; i<this->cercle.size();i++)
        {
            //Calcul du décalage selon la courbe
             float dx=this->Courbe[j]->getX();
             float dy=this->Courbe[j]->getY();
             float dz=this->Courbe[j]->getZ();
             //Calcul des coordonnées du point après la rotation phi autour de x
             float x=this->cercle[i]->getX()*cos(phi)-this->cercle[i]->getY()*sin(phi);
             float y=this->cercle[i]->getX()*sin(phi)+this->cercle[i]->getY()*cos(phi);
             float z=this->cercle[i]->getZ();
             /*//Calcul des coordonnées du point après la rotation theta autour de y
             x=x*cos(theta)-sin(theta)*z;
             y=y;
             z=x*sin(theta)+cos(theta)*z;*/
             //Calcul des coordonnées du point après translation
             x=x+dx;
             y=y+dy;
             z=z+dz;
             //Création du point appartenant à la surface
             PointGL * p=new PointGL(x,y,z);
             //Ajout du point à la surface
             this->points.push_back(p);
             this->texels.push_back(new Texel(incI*j,incJ*i));
        }
    }   
}

void SCirculaire::construction() {
    for (int i = 0; i <nbl-1; i++) {
        for (int j = 0; j < nbc; j++) {
            Face* face = new Face();
           
            face->add(this->points[i * nbc + j], this->texels[i * nbc + j]);
            face->add(this->points[(i + 1) * nbc + j],this->texels[(i + 1) * nbc + j]);
            face->add(this->points[(i + 1) * nbc + ((j + 1)%nbc)],this->texels[(i + 1) * nbc + ((j + 1)%nbc)]);
            face->add(this->points[i * nbc + ((j + 1)%nbc)],this->texels[i * nbc + ((j + 1)%nbc)]);
           
            this->faces.push_back(face);
        }
    }
     
    this->calculeNormal();
}

/**
 *@brief Fonction permettant de construire le cercle qui par son avancement 
 * le long de la courbe créera la surface.
 */
void SCirculaire::BuildCircle(){
    float pas=(2*this->pi)/this->nbc;
    for (float i=0.0; i<2*this->pi; i+=pas){
        float x= 0;
        float y= this->r*sin(i);
        float z= this->r*cos(i);
        PointGL * p=new PointGL(x,y,z);
        this->cercle.push_back(p);
    }
}

float SCirculaire::CalculPhi(PointGL* p){
    std::vector <float> n=p->getNormal();
    float f=n[0]/sqrt(n[0]*n[0]+n[1]*n[1]);
    f=acos(f);
    if(n[1]<0||n[0]<0)f=-f;
    return f;
}

float SCirculaire::CalculTheta(PointGL* p){
    std::vector <float> n=p->getNormal();
    float f=n[2]/sqrt(n[0]*n[0]+n[2]*n[2]);
    f=acos(f);
    return f;
}
/**
 * @brief Fonction permettant l'affichage de la surface
 */
/*void SCirculaire::Affichage(){
    glColor3f(1.0,0.0,1.0);
    for(int i =0; i<nbl-1; i++)
        {
                for(int j =0; j<nbc; j++)
                {
                    glBegin(GL_QUADS);
                        glVertex3f(this->points[i*nbc+j]->getX(),this->points[i*nbc+j]->getY(),this->points[i*nbc+j]->getZ());
                        glVertex3f(this->points[i*nbc+((j+1)%nbc)]->getX(),this->points[i*nbc+((j+1)%nbc)]->getY(),this->points[i*nbc+((j+1)%nbc)]->getZ());
                        glVertex3f(this->points[(i+1)*nbc+((j+1)%nbc)]->getX(),this->points[(i+1)*nbc+((j+1)%nbc)]->getY(),this->points[(i+1)*nbc+((j+1)%nbc)]->getZ());
                        glVertex3f(this->points[(i+1)*nbc+j]->getX(),this->points[(i+1)*nbc+j]->getY(),this->points[(i+1)*nbc+j]->getZ());
                    glEnd();
                }
        }
}*/

std::vector<Face*> SCirculaire::getFace(){
    return this->faces;
}