/**
 * @file railcluster.h
 * @copyright 2015 Jean-Jacques PONCIANO, Claire PRUDHOMME
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
#include "railcluster.h"

RailCluster::RailCluster()
{

}

RailCluster::~RailCluster()
{
    this->points.clear();
}

void RailCluster::add(QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::add(pcl::PointXYZ * pt)
{

}

void RailCluster::remove(pcl::PointXYZ * pt){

}

void RailCluster::match(QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::addSimilarePoint(QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::growing(RailCluster rail, QVector <pcl::PointXYZ *> pts)
{

}

void RailCluster::setEm(float e){
    this->em=e;
}

void RailCluster::setHm(float h){
    this->hm=h;
}

void RailCluster::setLm(float l){
    this->lm=l;
}

float RailCluster::getEm(){
    return this->em;
}

float RailCluster::getHm(){
    return this->hm;
}

float RailCluster::getLm(){
    return this->lm;
}
