/*
 * ColorBlobDetector.hpp
 *
 *  Created on: 30.11.2011
 *      Author: daniel
 */

#ifndef COLORBLOBDETECTOR_HPP_
#define COLORBLOBDETECTOR_HPP_

#include <cvblob.h>
#include "Cube.hpp"


class ColorBlobDetector
{
public:
  ColorBlobDetector(CvSize imgSize);
  virtual ~ColorBlobDetector();
  std::vector<Cube> getCubes();
  void setImage(IplImage* img);

private:
  cvb::CvBlobs getBlobsforColor(Color c, int numExpected, int sizeExpected);
  IplImage* img;
  IplConvKernel* morphKernel;
  CvSize imgSize;
};

#endif /* COLORBLOBDETECTOR_HPP_ */
