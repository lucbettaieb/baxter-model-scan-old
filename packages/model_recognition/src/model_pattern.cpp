/*
 * model_pattern 
 *
 * a class to describe a k-means clustering pattern in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include "model_recognition/model_pattern.h"
#include <string>

ModelPattern::ModelPattern(std::string name,
                           float r, float g, float b,
                           float l, float w, float h)
{
  pattern_label = name;

  red = r;
  green = g;
  blue = b;

  length = l;
  width = w;
  height = h;

  cluster_label = "none";
}
ModelPattern::~ModelPattern()
{
}

float ModelPattern::EuclidianDistance(std::vector<float> cen_vec)
{
  float distance = 0;

  distance += (cen_vec[0] - c_r)*(cen_vec[0] - c_r);
  distance += (cen_vec[1] - c_g)*(cen_vec[1] - c_g);
  distance += (cen_vec[2] - c_b)*(cen_vec[2] - c_b);

  distance += (cen_vec[3] - c_l)*(cen_vec[3] - c_l);
  distance += (cen_vec[4] - c_w)*(cen_vec[4] - c_w);
  distance += (cen_vec[5] - c_h)*(cen_vec[5] - c_h);

  return std::sqrt(distance);
}
