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

class ModelCluster
{
public:
  ModelCluster()
  virtual ~ModelCluster();

private:
  std::string cluster_label;

  
};

#endif  // MODEL_CLUSTER_H
