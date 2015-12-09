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

#include "model_recognition/model_pattern_set.h"
#include "model_recognition/model_cluster_set.h"

bool g_debug = true;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cluster_models");

  ModelPatternSet pattern_set;

  std::fstream feature_info("/home/luc/indigo/feature_info.dat", std::ios_base::in);

  std::string read_in;

  uint count_iter = 0;
  uint n_features = 7;

  std::string p_name;
  float feat[n_features - 1];

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
      std::cout << p.getLabel() << " " << p.getRed() << " " << p.getBlue()
                << " " <<p.getGreen() << " " << p.getLength() << " "
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
      std::cout << p.getLabel() << " " << p.getRed() << " " << p.getBlue()
                << " " <<p.getGreen() << " " << p.getLength() << " "
                << p.getWidth() << " " <<p.getHeight() << std::endl;
    }
  }

}
