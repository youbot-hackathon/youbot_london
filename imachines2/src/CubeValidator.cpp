/*
 * getReliableCubes.cpp
 *
 *  Created on: 01.12.2011
 *      Author: daniel
 */

#include "CubeValidator.hpp"

#include <math.h>

CubeValidator::CubeValidator()
{
  numOfScans = 0;

}

CubeValidator::~CubeValidator()
{
  // TODO Auto-generated destructor stub
}

void CubeValidator::addScan(std::vector<Cube> cubes)
{
  numOfScans++;
  for (std::vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); it++)
    addCube(*it);
  cleanup();
}

void CubeValidator::cleanup()
{
  std::vector<int> toBeDeleted;
  for (int i = 0; i < believe.size(); i++)
  {
    if (believe[i] < -3)
      toBeDeleted.push_back(i);
  }
  for (int i = toBeDeleted.size() - 1; i >= 0; i--)
  {
    believe.erase(believe.begin() + toBeDeleted[i]);
    cubes.erase(cubes.begin() + toBeDeleted[i]);
  }

}

std::vector<Cube> CubeValidator::getReliableCubes()
{
  std::vector<Cube> reliables;
  for (int i = 0; i < cubes.size(); i++)
  {
    if (believe[i] > 5)
      reliables.push_back(cubes[i]);
  }
  return reliables;
}

double CubeValidator::getDistance(Cube& c1, Cube& c2)
{
  return sqrt(
      pow((float)(c1.getCenter().x - c2.getCenter().x), 2) + pow((float)(c1.getCenter().y - c2.getCenter().y), 2));
}

void CubeValidator::ageBeliefs()
{
  for (int i = 0; i < believe.size(); i++)
  {
    believe[i]--;
  }
}

void CubeValidator::addCube(Cube cube)
{
  bool found = false;
  for (int i = 0; i < cubes.size(); i++)
  {
    Cube cube_ptr = cubes.at(i);
    if (getDistance(cube_ptr, cube) <= cube.getSize() / 2.0 && cube_ptr.getColor() == cube.getColor())
    {
      believe[i] += 2;
      cubes[i] = cube;
      found = true;
      break;
    }
  }
  if (!found)
  {
    cubes.push_back(cube);
    believe.push_back(1);
  }
  ageBeliefs();

}
