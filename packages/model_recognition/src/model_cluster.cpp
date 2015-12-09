/*
 * model_cluster 
 *
 * a class to describe a k-means cluster
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include <string>

#include "model_recognition/model_cluster.h"

ModelCluster::ModelCluster(std::string label)
{
  cluster_label = label;

  r_centroid = 0;
  b_centroid = 0;
  g_centroid = 0;

  l_centroid = 0;
  w_centroid = 0;
  h_centroid = 0;
}

ModelCluster::~ModelCluster()
{
}

void ModelCluster::addToCluster(boost::shared_ptr<ModelPattern> p_pattern)
{
  p_patterns.push_back(p_pattern);
}

void ModelCluster::removeFromCluster(std::string label)
{
  for (size_t i = 0; i < p_patterns.size(); i++)
  {
    if (label == p_patterns.at(i)->getLabel())
      p_patterns.erase(p_patterns.begin()+i);
  }
}
