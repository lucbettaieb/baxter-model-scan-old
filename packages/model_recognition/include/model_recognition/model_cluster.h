/*
 * model_cluster 
 *
 * a class to describe a k-means cluster
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#ifndef MODEL_CLUSTER_H
#define MODEL_CLUSTER_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "model_recognition/model_pattern.h"

class ModelCluster
{
public:
  ModelCluster(std::string label);
  virtual ~ModelCluster();

private:
  std::vector<boost::shared_ptr<ModelPattern>> p_patterns;

  std::string cluster_label;

  float r_centroid;
  float g_centroid;
  float b_centroid;

  float l_centroid;
  float w_centroid;
  float h_centroid;

public:
  void addToCluster(boost::shared_ptr<ModelPattern> p_pattern);
  void removeFromCluster(std::string label);

};

#endif  // MODEL_CLUSTER_H
