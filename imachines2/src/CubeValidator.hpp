/*
 * getReliableCubes.h
 *
 *  Created on: 01.12.2011
 *      Author: daniel
 */

#ifndef GETRELIABLECUBES_H_
#define GETRELIABLECUBES_H_

#include "Cube.hpp"
#include <vector>

class CubeValidator
{
public:
  CubeValidator();
  virtual ~CubeValidator();
  void addScan(std::vector<Cube> cubes);
  std::vector<Cube> getReliableCubes();
private:
  int numOfScans;
  std::vector<Cube> cubes;
  std::vector<int> believe;
  void addCube(Cube cube);
  double getDistance(Cube& c1, Cube& c2);
  void ageBeliefs();
  void cleanup();
};

#endif /* GETRELIABLECUBES_H_ */
