/*
 * Cube.h
 *
 *  Created on: 01.12.2011
 *      Author: daniel
 */

#ifndef CUBE_H_
#define CUBE_H_

#include <opencv2/opencv.hpp>


enum Color
{
  RED, BLUE, GREEN
};

class Cube
{
public:
  Cube();
  virtual ~Cube();
    CvPoint getCenter() const;
    Color getColor() const;
    CvPoint getMaxXy() const;
    CvPoint getMinXy() const;
    void setCenter(CvPoint center);
    void setColor(Color color);
    void setMaxXy(CvPoint maxXy);
    void setMinXy(CvPoint minXy);
    string toString();
    int getSize();
private:
  CvPoint center;
  CvPoint minXY;
  CvPoint maxXY;
  Color color;
};

#endif /* CUBE_H_ */
