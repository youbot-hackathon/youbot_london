/*
 * ColorBlobDetector.cpp
 *
 *  Created on: 30.11.2011
 *      Author: daniel
 */

#include "ColorBlobDetector.hpp"

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <iostream>
//#include <stdlib.h>

using namespace cvb;

const int MIN_AREA = 15000;
const int MAX_AREA = 37000;


void ColorBlobDetector::setImage(IplImage* img)
{
  this->img = img;

}

ColorBlobDetector::ColorBlobDetector(CvSize imgSize)
{
  this->imgSize = imgSize;
  morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
}

ColorBlobDetector::~ColorBlobDetector()
{
  // TODO Auto-generated destructor stub
}

CvBlobs ColorBlobDetector::getBlobsforColor(Color color, int numberExpected, int sizeexpected)
{
  CvBlobs blobs;

  IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

  cvConvertScale(img, frame, 1, 0);
  IplImage *segmentated = cvCreateImage(imgSize, 8, 1);
  for (unsigned int j = 0; j < imgSize.height; j++)
    for (unsigned int i = 0; i < imgSize.width; i++)
    {
      //hol das pixel an Stelle i,j
      CvScalar c = cvGet2D(frame, j, i);

      //hole grb-Werte
      double b = ((double)c.val[0]) / 255.;
      double g = ((double)c.val[1]) / 255.;
      double r = ((double)c.val[2]) / 255.;

      unsigned char f;

      //set the color detection borders
      switch (color)
      {
        case RED:
          f = 255 * ((r > 0.1 + g) && (r > 0.1 + b));
          break;
        case BLUE:
          f = 255 * ((b > 0.1 + g) && (b > 0.1 + r));
          break;
        case GREEN:
          f = 255 * ((g > 0.03 + b) && (g > 0.03 + r));
          break;

      }

      //setze den pixel
      cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
    }

  cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN, 1);
  IplImage *labelImg = cvCreateImage(imgSize, IPL_DEPTH_LABEL, 1);

  cvLabel(segmentated, labelImg, blobs);


  cvFilterByArea(blobs, MIN_AREA, MAX_AREA);
  return blobs;
}

std::vector<Cube> ColorBlobDetector::getCubes()
{
  static const int CUBESIZE = 9000;
  std::vector<Cube> cubes;
  CvBlobs redblobs = getBlobsforColor(RED, 2, CUBESIZE);
  for (CvBlobs::iterator it = redblobs.begin(); it != redblobs.end(); it++)
  {
    Cube newCube;
    newCube.setCenter(cvPoint((*it).second->centroid.x, (*it).second->centroid.y));
    newCube.setMinXy(cvPoint((*it).second->minx, (*it).second->miny));
    newCube.setMaxXy(cvPoint((*it).second->maxx, (*it).second->maxy));
    newCube.setColor(RED);
    cubes.push_back(newCube);
  }
  CvBlobs blueblobs = getBlobsforColor(BLUE, 1, CUBESIZE);
  for (CvBlobs::iterator it = blueblobs.begin(); it != blueblobs.end(); it++)
  {
    Cube newCube;
    newCube.setCenter(cvPoint((*it).second->centroid.x, (*it).second->centroid.y));
    newCube.setMinXy(cvPoint((*it).second->minx, (*it).second->miny));
    newCube.setMaxXy(cvPoint((*it).second->maxx, (*it).second->maxy));
    newCube.setColor(BLUE);
    cubes.push_back(newCube);
  }
  CvBlobs greenblobs = getBlobsforColor(GREEN, 3, CUBESIZE);
  for (CvBlobs::iterator it = greenblobs.begin(); it != greenblobs.end(); it++)
  {
    Cube newCube;
    newCube.setCenter(cvPoint((*it).second->centroid.x, (*it).second->centroid.y));
    newCube.setMinXy(cvPoint((*it).second->minx, (*it).second->miny));
    newCube.setMaxXy(cvPoint((*it).second->maxx, (*it).second->maxy));
    newCube.setColor(GREEN);
    cubes.push_back(newCube);
  }
  return cubes;
}
