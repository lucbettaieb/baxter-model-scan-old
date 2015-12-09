/*
 * cluster_models
 *
 * a ros node to perform k-means clustering in feature-space
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <vector>

#include "model_recognition/model_cluster.h"

#include "model_recognition/model_pattern_set.h"
#include "model_recognition/model_cluster_set.h"

typedef std::vector<ModelCluster> ModelClusterSet;

bool g_debug = false;

uint N_CLUSTERS = 3;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_models");

  ModelPatternSet pattern_set;
  ModelClusterSet cluster_set;

  std::fstream feature_info("/home/luc/indigo/feature_info.dat", std::ios_base::in);

  std::string read_in;

  uint count_iter = 0;
  uint n_features = 7;

  std::string p_name;
  float feat [n_features - 1];

  while (feature_info >> read_in)
  {
    if (count_iter % n_features == 0)
    {
      // PATTERN NAME
      p_name = read_in;
      // std::cout << "name: " << p_name << std::endl;
    }
    else if (count_iter % n_features == 1)
    {
      feat[0] = std::stof(read_in);
      // std::cout << "red: " << feat[0] << std::endl;
    }
    else if (count_iter % n_features == 2)
    {
      // PATTERN GREEN
      feat[1] = std::stof(read_in);
      // std::cout << "grn: " << feat[1] << std::endl;
    }
    else if (count_iter % n_features == 3)
    {
      // PATTERN BLUE
      feat[2] = std::stof(read_in);
      // std::cout << "blu: " << feat[2] << std::endl;
    }
    else if (count_iter % n_features == 4)
    {
      // PATTERN LENGTH
      feat[3] = std::stof(read_in);
      // std::cout << "l: " << feat[3] << std::endl;
    }
    else if (count_iter % n_features == 5)
    {
      // PATTERN WIDTH
      feat[4] = std::stof(read_in);
      // std::cout << "w: " << feat[4] << std::endl;
    }
    else if (count_iter % n_features == 6)
    {
      // PATTERN HEIGHT
      feat[5] = std::stof(read_in);
      // std::cout << "h: " << feat[5] << std::endl;

      // END READ LINE
      ModelPattern pattern(p_name,
                          feat[0], feat[1], feat[2],
                          feat[3], feat[4], feat[5]);

      pattern_set.pushBackPattern(pattern);
    }
    count_iter++;
  }

  ROS_INFO("Got raw data.");
  if (g_debug)
  {
    for (uint i = 0; i < pattern_set.getSize(); i++)
    {
      ModelPattern p = pattern_set.getPattern(i);
      std::cout << p.getLabel() << " " << p.getRed() << " " << p.getGreen()
                << " " <<p.getBlue() << " " << p.getLength() << " "
                << p.getWidth() << " " <<p.getHeight() << std::endl;

    }
  }
  ROS_INFO("Time to pre-process data.  Scaling features.");

  pattern_set.scaleFeatures();

  ROS_INFO("Features have been scaled.");

  if (g_debug)
  {
    for (uint i = 0; i < pattern_set.getSize(); i++)
    {
      ModelPattern p = pattern_set.getPattern(i);
      std::cout << p.getLabel() << " " << p.getRed() << " " << p.getGreen()
                << " " <<p.getBlue() << " " << p.getLength() << " "
                << p.getWidth() << " " <<p.getHeight() << std::endl;

    }
  }

  ROS_INFO("Assigning clusters to contain first pattern of each model");
  // gonna cheat a little bit.  initialize each cluster to the first
  // model scan data of each data set.  for our purposes, this
  // simplifies the clustering process and will allow for the easy
  // introduction of new data.

  std::string label = "";

  for (size_t i = 0; i < pattern_set.getSize(); i++)
  {
    if (pattern_set.getPattern(i).getLabel() != label)
    {
      label = pattern_set.getPattern(i).getLabel();
      ModelCluster m(label);
      m.initCluster(pattern_set.getPattern(i));

      cluster_set.push_back(m);

      pattern_set.getPattern(i).setClusterLabel(m.getLabel());
      //std::cout << "!!! should be: " << m.getLabel() << " is: " << pattern_set.getPattern(i).getClusterLabel() << std::endl;
    }
  }

  if (g_debug)
  {
    for (size_t i = 0; i < cluster_set.size(); i++)
    {
      cluster_set.at(i).printCentroid();
    }
  }

  ROS_INFO("Clusters initialized!  Ready for pattern processing.");

  uint nChanges = 0;

  for (size_t p = 0; p < pattern_set.getSize(); p++)
  {
    if (pattern_set.getPattern(p).getClusterLabel().compare("none") == 0)
      uint rand_clust = rand() % 3;

      cluster_set.at(rand_clust).addToCluster(pattern_set.getPattern(p));
      pattern_set.getPattern(p).setClusterLabel(cluster_set.at(rand_clust).getLabel());
      //nChanges += 1;
    }
  }

  do
  {
    nChanges = 0;
    for (size_t c = 0; c < cluster_set.size(); c++)
    {
      std::cout << "c: " << c << "cluster_set size: " << cluster_set.size() << std::endl;
      for (size_t p = 0; p < pattern_set.getSize(); p++)
      {
        std::cout << "p: " << p << ", pattern_set size: " << pattern_set.getSize() << std::endl;

        // if (pattern_set.getPattern(p).getClusterLabel().compare("none") == 0)
        // {
        //   std::cout << "making none some: " << std::endl;
        //   uint rand_clust = rand() % 3;
        //   std::cout << 
        //   cluster_set.at(rand_clust).addToCluster(pattern_set.getPattern(p));
        //   pattern_set.getPattern(p).setClusterLabel(cluster_set.at(rand_clust).getLabel());
        //   nChanges += 1;
        // }

        // Find the cluster index in which the current pattern is currently assigned
        uint asgn_clust_i;
        for (size_t i = 0; i < cluster_set.size(); i++)
        {
          if (pattern_set.getPattern(p).getClusterLabel() == cluster_set.at(i).getLabel())
            asgn_clust_i = i;
        }

        // std::cout << "about to check if we need to reassign clusters." << std::endl;
        // std::cout << "(new) dist from p to c: " << pattern_set.getPattern(p).EuclidianDistance(cluster_set.at(c).getCentroid()) 
        //           << ", (old) dist from p to asgn_clust_i: " << pattern_set.getPattern(p).EuclidianDistance(cluster_set.at(asgn_clust_i).getCentroid())
        //           << std::endl;
        
        if (pattern_set.getPattern(p).getLabel() != "none" &&
            pattern_set.getPattern(p).EuclidianDistance(cluster_set.at(c).getCentroid()) <  // new cluster distance
            pattern_set.getPattern(p).EuclidianDistance(cluster_set.at(asgn_clust_i).getCentroid()))  // old cluster distance
        {
          std::cout << "remove from cluster " << std::endl;
          std::cout << "old cluster label: " << cluster_set.at(asgn_clust_i).getLabel() << std::endl;
          std::cout << "pattern label: " << pattern_set.getPattern(p).getLabel() << std::endl;
          // The pattern should be removed from asgn_clust_i and assigned to c
          cluster_set.at(asgn_clust_i).removeFromCluster(pattern_set.getPattern(p));
          
          std::cout << "set cluster label" << std::endl;
          pattern_set.getPattern(p).setClusterLabel(cluster_set.at(c).getLabel());
          std::cout << "add 2 cluster" << std::endl;
          cluster_set.at(c).addToCluster(pattern_set.getPattern(p));
          
          
          nChanges += 2;
        }
      }
    }
  } while (nChanges != 0);

  ROS_INFO("DONE!");
  ROS_INFO("report: ");
  for (size_t i = 0; i < cluster_set.size(); i++)
  {
    std::cout << cluster_set.at(i).getLabel() << std::endl;
    for (size_t j = 0; j < cluster_set.at(i).getPatternVec().size(); j++)
    {
      std::cout << "pat label: " << cluster_set.at(i).getPatternVec().at(j).getLabel() << std::endl;
      std::cout << "clust label: " << cluster_set.at(i).getPatternVec().at(j).getClusterLabel() << std::endl;
    }
    std::cout << std::endl;
  }
}
