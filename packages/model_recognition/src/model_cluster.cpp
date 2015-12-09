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
#include <vector>

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

void ModelCluster::initCluster(ModelPattern pattern)
{
  patterns.push_back(pattern);
  r_centroid = pattern.getRed();
  g_centroid = pattern.getGreen();
  b_centroid = pattern.getBlue();

  l_centroid = pattern.getLength();
  w_centroid = pattern.getWidth();
  h_centroid = pattern.getHeight();
}

void ModelCluster::addToCluster(ModelPattern pattern)
{
  patterns.push_back(pattern);

  r_centroid *= patterns.size()/(patterns.size()+1);
  r_centroid += (1/(patterns.size()+1)) * pattern.getRed();

  g_centroid *= patterns.size()/(patterns.size()+1);
  g_centroid += (1/(patterns.size()+1)) * pattern.getGreen();

  b_centroid *= patterns.size()/(patterns.size()+1);
  b_centroid += (1/(patterns.size()+1)) * pattern.getBlue();

  l_centroid *= patterns.size()/(patterns.size()+1);
  l_centroid += (1/(patterns.size()+1)) * pattern.getLength();

  w_centroid *= patterns.size()/(patterns.size()+1);
  w_centroid += (1/(patterns.size()+1)) * pattern.getWidth();

  h_centroid *= patterns.size()/(patterns.size()+1);
  h_centroid += (1/(patterns.size()+1)) * pattern.getHeight();
}

void ModelCluster::removeFromCluster(ModelPattern pattern)
{
  std::cout << "pat size before erase: " << patterns.size() << std::endl;
  std::string label = pattern.getLabel();
  for (size_t i = 0; i < patterns.size(); i++)
  {
    if (label == patterns.at(i).getLabel())
      patterns.erase(patterns.begin()+i);
  }

  std::cout << "pat size after erase: " << patterns.size() << std::endl;
  std::cout << "r_centroid: " << r_centroid << std::endl;
  std::cout << "pattern getRed: " << pattern.getRed() << std::endl;
  r_centroid *= patterns.size()/(patterns.size()-1);
  r_centroid -= (1/(patterns.size()-1)) * pattern.getRed();

  std::cout << "2" << std::endl;
  g_centroid *= patterns.size()/(patterns.size()-1);
  g_centroid -= (1/(patterns.size()-1)) * pattern.getGreen();

  std::cout << "3" << std::endl;
  b_centroid *= patterns.size()/(patterns.size()-1);
  b_centroid -= (1/(patterns.size()-1)) * pattern.getBlue();

  std::cout << "4" << std::endl;
  l_centroid *= patterns.size()/(patterns.size()-1);
  l_centroid -= (1/(patterns.size()-1)) * pattern.getLength();

  std::cout << "5" << std::endl;
  w_centroid *= patterns.size()/(patterns.size()-1);
  w_centroid -= (1/(patterns.size()-1)) * pattern.getWidth();

  std::cout << "6" << std::endl;
  h_centroid *= patterns.size()/(patterns.size()-1);
  h_centroid -= (1/(patterns.size()-1)) * pattern.getHeight();
}

void ModelCluster::printCentroid()
{
  std::cout << cluster_label << "| cR: " << r_centroid << ", cG: " << g_centroid
            << ", cB: " << b_centroid << ", cL: " << l_centroid
            << ", cW: " << w_centroid << ", cH: " << h_centroid << std::endl;
}

std::vector<float> ModelCluster::getCentroid()
{
  std::vector<float> v;

  v.push_back(r_centroid);
  v.push_back(g_centroid);
  v.push_back(b_centroid);

  v.push_back(l_centroid);
  v.push_back(w_centroid);
  v.push_back(h_centroid);

  return v;
}
