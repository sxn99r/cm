#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "itpp/itcomm.h"


//------------------for it++3.7.3 and above
/*
complex<double> double_complex(double r, double i)
{
  return complex<double>(r, i);
}
*/
double log(int x){
  return std::log((double)x);
}
double pow(int x, int y){
  return std::pow((double)x, (double)y);
}
double pow(int x, double y){
  return std::pow((double)x, y);
}
double sqrt(int x){
  return std::sqrt((double)x);
}
//--END-------------for it++3.7.3 and above
