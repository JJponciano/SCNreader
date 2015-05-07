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
#include "Surface_Bezier.h"

Surface_Bezier::Surface_Bezier(const Surface_Bezier& orig):Objet3D(orig){
    this->largeur = orig.largeur;
    this->hauteur = orig.hauteur;
    this->longueur = orig.longueur;
}

Surface_Bezier::Surface_Bezier(std::vector< std::vector<PointGL *> > lespoints, int nbli, int nbco, float largeur, float hauteur, float longueur) : Objet3D(largeur, hauteur, longueur){
    //=====Remplissage du tableau caractéristique de bézier======//
    std::vector <float> t1;
    t1.push_back(-1);
    t1.push_back(3);
    t1.push_back(-3);
    t1.push_back(1);
    bezier.push_back(t1);
    
    std::vector <float> t2;
    t2.push_back(3);
    t2.push_back(-6);
    t2.push_back(3);
    t2.push_back(0);
    bezier.push_back(t2);
    
    std::vector <float> t3;
    t3.push_back(-3);
    t3.push_back(3);
    t3.push_back(0);
    t3.push_back(0);
    bezier.push_back(t3);
    
    std::vector <float> t4;
    t4.push_back(1);
    t4.push_back(0);
    t4.push_back(0);
    t4.push_back(0);
    bezier.push_back(t4);
    
    //======================================================//
    //Récupération du tableau des points de controles
    for(int i=0; i<lespoints.size(); i++)
                this->ptsControle.push_back(lespoints[i]);
    //Récupération de la taille du tableau des points de controles
    this->nbl=nbli;
    this->nbc=nbco;
    //Calcule du décalage
    this->nbdecL=this->nbl-3;
    this->nbdecC=this->nbc-3;
    this->CalculeSurface();
    
}


Surface_Bezier::~Surface_Bezier() {
    //destruction du tableau des points de la surface
    for (int i = 0; i < this->points.size(); i++) {
        delete this->points[i];
        this->points[i] = 0;
    }
    //destruction du tableau des points de controle
    for (int i = 0; i < this->nbc; i++)
        for (int j = 0; j < this->nbl; j++)
        {
            delete this->ptsControle[i][j];
            this->ptsControle[i][j] = 0;
        }
    //destruction du tableau des points de controle courant
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
        {
            delete this->ptsCtrlC[i][j];
            this->ptsCtrlC[i][j] = 0;
        }
}

std::vector< std::vector<float> > Surface_Bezier::ProduitMat(std::vector< std::vector<float> > matrice1, int nbl1, int nbc1, std::vector< std::vector<float> > matrice2, int nbl2, int nbc2){
    std::vector< std::vector<float> > produit;
    if(nbc1!=nbl2)
    {
        std::cout<<"Erreur: Il n'est pas possible de faire le produit de ces deux matrices car elles sont incompatibles"<<std::endl;       
    }
    else{
        for(int i=0; i<nbl1; i++)
        {
            std::vector<float> p;
            for(int j=0; j<nbc2; j++)
            {
                float somme=0;
                for(int k=0; k<nbc1; k++)
                {
                        somme+=matrice1[i][k]*matrice2[k][j];
                }
                p.push_back(somme);
            }
            produit.push_back(p);
        }
    }
    return produit;
}

std::vector< std::vector<float> > Surface_Bezier::TransposerMat(std::vector< std::vector<float> > matrice, int nbl, int nbc){
    std::vector< std::vector<float> > transposer;
    for(int i=0; i<nbc;i++)
    {
        for(int j=0; j<nbl;j++)
        {
            transposer[i][j]=matrice[j][i];
        }
    }
    std::vector< std::vector<float> > t;
    for(int i=0; i<nbc; i++)
        {
            std::vector<float> p;
            for(int j=0; j<nbl; j++)
            {
                p.push_back(transposer[i][j]);
            }
            t.push_back(p);
        }
    return t;
}

/*std::vector< std::vector<float> > Surface_Bezier::InverseMat(std::vector< std::vector<float> > matrice, int nbl, int nbc){
    float transposer[nbc][nbl];
    for(int i=0; i<nbl;i++)
    {
        for(int j=0; j<nbc;j++)
        {
            transposer[i][j]=matrice[i][nbc-1-j];
        }
    }
    std::vector< std::vector<float> > t;
    for(int i=0; i<nbc; i++)
        {
            std::vector<float> p;
            for(int j=0; j<nbl; j++)
            {
                p.push_back(transposer[i][j]);
            }
            t.push_back(p);
        }
    return t;
}*/

float Surface_Bezier::FuncX(float u, float v){
    //Création du vecteur U(t)
    std::vector< std::vector<float> > ut;
    std::vector<float> u1;
    u1.push_back(u*u*u);
    u1.push_back(u*u);
    u1.push_back(u);
    u1.push_back(1);
    ut.push_back(u1);
    //Création du vecteur V(t)
    std::vector< std::vector<float> > vt;
    std::vector<float> v1;
    v1.push_back(v*v*v);
    v1.push_back(v*v);
    v1.push_back(v);
    v1.push_back(1);
    vt.push_back(v1);
    //Création de la transposée du vecteur V(t)
    std::vector< std::vector<float> > vtT=this->TransposerMat(vt, 1, 4);
    //Création de la transposée de la matrice caractéristique
    //std::vector< std::vector<float> > bezierT=this->InverseMat(this->bezier, 4, 4);
    
    //Récupération des pts de controle en x
   std::vector< std::vector<float> > ptCt;
    for(int i=0; i<4; i++)
    {
        std::vector<float> p;
        for(int j=0; j<4; j++)
        {
            p.push_back(this->ptsCtrlC[i][j]->getX());
        }
        ptCt.push_back(p);
    }
    
    std::vector< std::vector<float> > r1=this->ProduitMat(ut, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r2=this->ProduitMat(r1, 1, 4, ptCt, 4, 4);
    std::vector< std::vector<float> > r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r4=this->ProduitMat(r3, 1, 4, vtT, 4, 1);
   
    return r4[0][0];
}

float Surface_Bezier::FuncY(float u, float v){
    //Création du vecteur U(t)
    std::vector< std::vector<float> > ut;
    std::vector<float> u1;
    u1.push_back(u*u*u);
    u1.push_back(u*u);
    u1.push_back(u);
    u1.push_back(1);
    ut.push_back(u1);
    //Création du vecteur V(t)
    std::vector< std::vector<float> > vt;
    std::vector<float> v1;
    v1.push_back(v*v*v);
    v1.push_back(v*v);
    v1.push_back(v);
    v1.push_back(1);
    vt.push_back(v1);
    //Création de la transposée du vecteur V(t)
    std::vector< std::vector<float> > vtT=this->TransposerMat(vt, 1, 4);
    
    //Création de la transposée de la matrice caractéristique
    //std::vector< std::vector<float> > bezierT=this->InverseMat(this->bezier, 4, 4);
    
    //Récupération des pts de controle en y
    std::vector< std::vector<float> > ptCt;
    for(int i=0; i<4; i++)
    {
        std::vector<float> p;
        for(int j=0; j<4; j++)
        {
            p.push_back(this->ptsCtrlC[i][j]->getY());
        }
        ptCt.push_back(p);
    }
    
    std::vector< std::vector<float> > r1=this->ProduitMat(ut, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r2=this->ProduitMat(r1, 1, 4, ptCt, 4, 4);
    std::vector< std::vector<float> > r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r4=this->ProduitMat(r3, 1, 4, vtT, 4, 1);

    return r4[0][0];
}

float Surface_Bezier::FuncZ(float u, float v){
    //Création du vecteur U(t)
    std::vector< std::vector<float> > ut;
    std::vector<float> u1;
    u1.push_back(u*u*u);
    u1.push_back(u*u);
    u1.push_back(u);
    u1.push_back(1);
    ut.push_back(u1);
    //Création du vecteur V(t)
    std::vector< std::vector<float> > vt;
    std::vector<float> v1;
    v1.push_back(v*v*v);
    v1.push_back(v*v);
    v1.push_back(v);
    v1.push_back(1);
    vt.push_back(v1);
    //Création de la transposée du vecteur V(t)
    std::vector< std::vector<float> > vtT=this->TransposerMat(vt, 1, 4);
    
    //Création de la transposée de la matrice caractéristique
    //std::vector< std::vector<float> > bezierT=this->InverseMat(this->bezier, 4, 4);
  
    //Récupération des pts de controle en x
    std::vector< std::vector<float> > ptCt;
    for(int i=0; i<4; i++)
    {
        std::vector<float> p;
        for(int j=0; j<4; j++)
        {
            p.push_back(this->ptsCtrlC[i][j]->getZ());
        }
        ptCt.push_back(p);
    }
    
    std::vector< std::vector<float> > r1=this->ProduitMat(ut, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r2=this->ProduitMat(r1, 1, 4, ptCt, 4, 4);
    std::vector< std::vector<float> > r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r4=this->ProduitMat(r3, 1, 4, vtT, 4, 1);
    
    return r4[0][0];
}
/*
 * fonction permettant de calculer la surface
 */
void Surface_Bezier::CalculeSurface(){
    //Pour chaque morceau de surface
    for(int i=0; i<this->nbl; i+=4)
    {
        for(int j=0; j<this->nbc; j+=4)
        {
            //remplissage de la matrice courante de points de controle
            std::vector<PointGL*> pt1;
            pt1.push_back(this->ptsControle[i][j]);
           
            pt1.push_back(this->ptsControle[i][j+1]);
            pt1.push_back(this->ptsControle[i][j+2]);
            pt1.push_back(this->ptsControle[i][j+3]);
            this->ptsCtrlC.push_back(pt1);
            
            std::vector<PointGL*> pt2;
            pt2.push_back(this->ptsControle[i+1][j]);
            pt2.push_back(this->ptsControle[i+1][j+1]);
            pt2.push_back(this->ptsControle[i+1][j+2]);
            pt2.push_back(this->ptsControle[i+1][j+3]);
            this->ptsCtrlC.push_back(pt2);
            
            std::vector<PointGL*> pt3;
            pt3.push_back(this->ptsControle[i+2][j]);
            pt3.push_back(this->ptsControle[i+2][j+1]);
            pt3.push_back(this->ptsControle[i+2][j+2]);
            pt3.push_back(this->ptsControle[i+2][j+3]);
            this->ptsCtrlC.push_back(pt3);
            
            std::vector<PointGL*> pt4;
            pt4.push_back(this->ptsControle[i+3][j]);
            pt4.push_back(this->ptsControle[i+3][j+1]);
            pt4.push_back(this->ptsControle[i+3][j+2]);
            pt4.push_back(this->ptsControle[i+3][j+3]);
            this->ptsCtrlC.push_back(pt4);
            
            //Parcourir pour u allant de 0 à 1
           for(float u =0.0; u<1.1; u+=0.1)
            {
                //et pour v allant de 0 à 1
                for(float v =0.0; v<1.1;v+=0.1)
                { 
                  //Tous les points à ajouter à la surface
                    float x=this->FuncX(u,v);
                    float y=this->FuncY(u,v);
                    float z=this->FuncZ(u,v);
                    PointGL *p=new PointGL(x,y,z);
                    p->setNormalMoyenne(this->FuncNormale(u,v));
                    this->points.push_back(p);
                }
            }
            //on vide la matrice courante des points de contrôle
            this->ptsCtrlC.clear();
        }
    }
    this->calculeFacette();
   // this->calculeNormal();
}

void Surface_Bezier::Affichage(){
    glPushMatrix();
        glScalef(this->largeur, this->hauteur, this->longueur);
        
        glColor3f(1.0,0.0,0.0);
        //pour un morceau de surface
        /*for(int i =0; i<10; i++)
        {
                for(int j =0; j<10; j++)
                {
                    glBegin(GL_QUADS);
                        glVertex3f(this->points[i*11+j]->getX(),this->points[i*11+j]->getY(),this->points[i*11+j]->getZ());
                        glVertex3f(this->points[i*11+j+1]->getX(),this->points[i*11+j+1]->getY(),this->points[i*11+j+1]->getZ());
                        glVertex3f(this->points[(i+1)*11+j+1]->getX(),this->points[(i+1)*11+j+1]->getY(),this->points[(i+1)*11+j+1]->getZ());
                        glVertex3f(this->points[(i+1)*11+j]->getX(),this->points[(i+1)*11+j]->getY(),this->points[(i+1)*11+j]->getZ());
                    glEnd();
                }
        }*/

        for (int i = 0; i<this->faces.size(); i++) 
        this->faces[i]->afficher(true);
        
     glPopMatrix();         
}

 std::vector<float> Surface_Bezier::FuncNormale(float u, float v){
    //Création du vecteur U(t)
    std::vector< std::vector<float> > ut;
    std::vector<float> u1;
    u1.push_back(u*u*u);
    u1.push_back(u*u);
    u1.push_back(u);
    u1.push_back(1);
    ut.push_back(u1);
    
    //Création du vecteur U'(t)
    std::vector< std::vector<float> > dUt;
    std::vector<float> u2;
    u2.push_back(3*u*u);
    u2.push_back(2*u);
    u2.push_back(1);
    u2.push_back(0);
    dUt.push_back(u2);
    
    //Création du vecteur V(t)
    std::vector< std::vector<float> > vt;
    std::vector<float> v1;
    v1.push_back(v*v*v);
    v1.push_back(v*v);
    v1.push_back(v);
    v1.push_back(1);
    vt.push_back(v1);
    //Création de la transposée du vecteur V(t)
    std::vector< std::vector<float> > vtT=this->TransposerMat(vt, 1, 4);
    
    //Création du vecteur V'(t)
    std::vector< std::vector<float> > dVt;
    std::vector<float> v2;
    v2.push_back(3*v*v);
    v2.push_back(2*v);
    v2.push_back(1);
    v2.push_back(0);
    dVt.push_back(v2);
    //Création de la transposée du vecteur V'(t)
    std::vector< std::vector<float> > dvtT=this->TransposerMat(dVt, 1, 4);
    
    //Création de la transposée de la matrice caractéristique
    //std::vector< std::vector<float> > bezierT=this->InverseMat(this->bezier, 4, 4);
    
    //Récupération des pts de controle en x
    std::vector< std::vector<float> > ptCtx;
    //Récupération des pts de controle en y
    std::vector< std::vector<float> > ptCty;
    //Récupération des pts de controle en z
    std::vector< std::vector<float> > ptCtz;
    for(int i=0; i<4; i++)
    {
        std::vector<float> px;
        std::vector<float> py;
        std::vector<float> pz;
        for(int j=0; j<4; j++)
        {
            px.push_back(this->ptsCtrlC[i][j]->getX());
            py.push_back(this->ptsCtrlC[i][j]->getY());
            pz.push_back(this->ptsCtrlC[i][j]->getZ());
        }
        ptCtx.push_back(px);
        ptCty.push_back(py);
        ptCtz.push_back(pz);
    }
    
    std::vector<float> vect1;
    std::vector<float> vect2;
    std::vector<float> normale;
    
    //Calcul du premier vecteur vect1
    std::vector< std::vector<float> > r1=this->ProduitMat(dUt, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r2=this->ProduitMat(r1, 1, 4, ptCtx, 4, 4);
    std::vector< std::vector<float> > r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    std::vector< std::vector<float> > r4=this->ProduitMat(r3, 1, 4, vtT, 4, 1);
    vect1.push_back(r4[0][0]);
    r2=this->ProduitMat(r1, 1, 4, ptCty, 4, 4);
    r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    r4=this->ProduitMat(r3, 1, 4, vtT, 4, 1);
    vect1.push_back(r4[0][0]);
    r2=this->ProduitMat(r1, 1, 4, ptCtz, 4, 4);
    r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    r4=this->ProduitMat(r3, 1, 4, vtT, 4, 1);
    vect1.push_back(r4[0][0]);
    
    //Calcul du second vecteur vect2
    r1=this->ProduitMat(ut, 1, 4, this->bezier, 4, 4);
    r2=this->ProduitMat(r1, 1, 4, ptCtx, 4, 4);
    r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    r4=this->ProduitMat(r3, 1, 4, dvtT, 4, 1);
    vect2.push_back(r4[0][0]);
    r2=this->ProduitMat(r1, 1, 4, ptCty, 4, 4);
    r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    r4=this->ProduitMat(r3, 1, 4, dvtT, 4, 1);
    vect2.push_back(r4[0][0]);
    r2=this->ProduitMat(r1, 1, 4, ptCtz, 4, 4);
    r3=this->ProduitMat(r2, 1, 4, this->bezier, 4, 4);
    r4=this->ProduitMat(r3, 1, 4, dvtT, 4, 1);
    vect2.push_back(r4[0][0]);
    
    //Calcul de la normale par le produit vectorielle des deux vecteurs
    float x=vect1[1]*vect2[2]-vect1[2]*vect2[1];
    float y=vect1[2]*vect2[0]-vect1[0]*vect2[2];
    float z=vect1[0]*vect2[1]-vect1[1]*vect2[0];
    normale.push_back(x);
    normale.push_back(y);
    normale.push_back(z);
    
    return normale;
 }
 
 void Surface_Bezier::calculeFacette() {
//pour chaque face
     int nc=this->nbc/4;
     int nl=this->nbl/4;
     float inc=1.0/(11);
     for(int k =0; k<nl; k++)
     {
         for(int l =0; l<nc; l++)
        {
     
            for(int i =0; i<10; i++)
           {
                   for(int j =0; j<10; j++)
                   {
                       Face* face = new Face();

                       face->add(this->points[(121*(k+l))+i*11+j], new Texel(i*inc,j*inc));
                       face->add(this->points[(121*(k+l))+(i+1)*11+j], new Texel((i+1)*inc,j*inc));
                       face->add(this->points[(121*(k+l))+(i+1)*11+j+1], new Texel((i+1)*inc,(j+1)*inc));
                       face->add(this->points[(121*(k+l))+i*11+j+1], new Texel(i*inc,(j+1)*inc));

                       //ajout de la face
                       this->faces.push_back(face);
                   }
           }
         }
     }
}
