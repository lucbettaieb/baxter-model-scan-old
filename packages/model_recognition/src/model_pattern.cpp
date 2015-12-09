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

  cluster_id = 0;
}
ModelPattern::~ModelPattern()
{
}
