/*
 * Cube.cpp
 *
 *  Created on: 01.12.2011
 *      Author: daniel
 */

#include "Cube.hpp"
#include <sstream>
#include <math.h>

Cube::Cube()
{
  // TODO Auto-generated constructor stub

}

Cube::~Cube()
{
  // TODO Auto-generated destructor stub
}

CvPoint Cube::getCenter() const
{
  return center;
}

Color Cube::getColor() const
{
  return color;
}

CvPoint Cube::getMaxXy() const
{
  return maxXY;
}

CvPoint Cube::getMinXy() const
{
  return minXY;
}

void Cube::setCenter(CvPoint center)
{
  this->center = center;
}

void Cube::setColor(Color color)
{
  this->color = color;
}

void Cube::setMaxXy(CvPoint maxXy)
{
  maxXY = maxXy;
}

void Cube::setMinXy(CvPoint minXy)
{
  minXY = minXy;
}

string Cube::toString()
{
  std::stringstream tmp;
  tmp << "(" << center.x << "|" << center.y << "); min:(" << minXY.x << "|" << minXY.y << "); max:(" << maxXY.x
      << "|" << maxXY.y << "); size: " << getSize();
  return tmp.str();
}

int Cube::getSize(){
return sqrt((maxXY.x - minXY.x)*(maxXY.x - minXY.x)+(maxXY.y - minXY.y)+(maxXY.x - minXY.x));
}

