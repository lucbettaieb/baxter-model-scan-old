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

void ModelCluster::addToCluster(ModelPattern &pattern)
{
  float pat_size = static_cast<float>(patterns.size());

  r_centroid = r_centroid * (pat_size/(pat_size+1)) + ((1/(pat_size+1)) * pattern.getRed());
  std::cout << "new r: " << r_centroid << std::endl;

  std::cout << "old g: " << g_centroid << std::endl;
  g_centroid *= pat_size/(pat_size+1);
  g_centroid += (1/(pat_size+1)) * pattern.getGreen();
  std::cout << "new g: " << g_centroid << std::endl;

  std::cout << "old b: " << b_centroid << std::endl;
  b_centroid *= pat_size/(pat_size+1);
  b_centroid += (1/(pat_size+1)) * pattern.getBlue();
  std::cout << "new b: " << b_centroid << std::endl;

  std::cout << "old l: " << l_centroid << std::endl;
  l_centroid *= pat_size/(pat_size+1);
  l_centroid += (1/(pat_size+1)) * pattern.getLength();
  std::cout << "new l: " << l_centroid << std::endl;

  std::cout << "old w: " << w_centroid << std::endl;
  w_centroid *= pat_size/(pat_size+1);
  w_centroid += (1/(pat_size+1)) * pattern.getWidth();
  std::cout << "new w: " << w_centroid << std::endl;

  std::cout << "old h: " << h_centroid << std::endl;
  h_centroid *= pat_size/(pat_size+1);
  h_centroid += (1/(pat_size+1)) * pattern.getHeight();
  std::cout << "new h: " << h_centroid << std::endl;

  std::cout << std::endl;
  patterns.push_back(pattern);
}

void ModelCluster::removeFromCluster(ModelPattern pattern)
{
  float pat_size = static_cast<float>(patterns.size());

  std::string label = pattern.getLabel();
  for (size_t i = 0; i < patterns.size(); i++)
  {
    if (label == patterns.at(i).getLabel())
      patterns.erase(patterns.begin()+i);
  }
  
  r_centroid *= pat_size/(pat_size-1);
  r_centroid -= (1/(pat_size-1)) * pattern.getRed();

  std::cout << "2" << std::endl;
  g_centroid *= pat_size/(pat_size-1);
  g_centroid -= (1/(pat_size-1)) * pattern.getGreen();

  std::cout << "3" << std::endl;
  b_centroid *= pat_size/(pat_size-1);
  b_centroid -= (1/(pat_size-1)) * pattern.getBlue();

  std::cout << "4" << std::endl;
  l_centroid *= pat_size/(pat_size-1);
  l_centroid -= (1/(pat_size-1)) * pattern.getLength();

  std::cout << "5" << std::endl;
  w_centroid *= pat_size/(pat_size-1);
  w_centroid -= (1/(pat_size-1)) * pattern.getWidth();

  std::cout << "6" << std::endl;
  h_centroid *= pat_size/(pat_size-1);
  h_centroid -= (1/(pat_size-1)) * pattern.getHeight();
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
