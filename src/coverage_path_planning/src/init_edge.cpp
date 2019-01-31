
/* Ce code permet de lire le fichier .txt des coordonnées du contour et d'en générer un message Path. 
*/
/*------------------------------------------------------------------Modifié-par-Luc-ZHENG-le-12-septembre-2018---------------------------------------------------------------------*/


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;



nav_msgs::Path lecture_file(){

  nav_msgs::Path path_msgs;
  path_msgs.header.frame_id = "odom";
  path_msgs.header.stamp = ros::Time::now();

  int nbLignes = 1;
  string ligne;

  // Lecture du fichier txt
  ifstream myfile("path.txt");
  if (myfile.is_open()){
    geometry_msgs::PoseStamped pose_file;
    while ( getline (myfile,ligne) ){


      // Initialisation du point à rajouter
      pose_file.header.frame_id = "odom";
      pose_file.header.stamp = ros::Time::now();       

      pose_file.pose.position.z = 0;

      pose_file.pose.orientation.w = 1;
      pose_file.pose.orientation.x = 0;
      pose_file.pose.orientation.y = 0;
      pose_file.pose.orientation.z = 0;

      // Récupération de la coordonnée x du point n
      if (nbLignes == 1){
        pose_file.pose.position.x = atof(ligne.c_str());
        nbLignes = 2;
      }

      // Récupération de la coordonnée y du point n
      else if (nbLignes == 2){
        pose_file.pose.position.y = atof(ligne.c_str());
        path_msgs.poses.push_back(pose_file);
        nbLignes = 1;
      }
    }
  }

  return path_msgs;
}


int main(int argc, char *argv[]){

  srand(time(0));
  ros::init(argc, argv, "init_edge");

  ros::NodeHandle init_edge;

  ros::Publisher pub = init_edge.advertise<nav_msgs::Path>("/path_planned/edge",1000);

  ros::Rate loop_rate(1);

  nav_msgs::Path contour;

  contour = lecture_file();

  while (ros::ok()){
    pub.publish(contour);
    loop_rate.sleep();
  }

  return 0;
}
