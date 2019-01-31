#ifndef CLASS_TREE
#define CLASS_TREE

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include "sensor_msgs/PointCloud.h"
#include <string>
#include "geometry_msgs/Point32.h"
#include <cmath>
#include <climits>


class Tree{
  public :
    struct Node{
      public:
        float x;           // coordonnée x du milieu de la cellule
        float y;           // coordonnée y du milieu de la cellule
        bool occupied;     // info présence obstacle (true) ou non (false)

        bool background;   // info cellule hors de la zone de tonte (true) ou dans la zone de tonte (false)

        bool visited;      // info cellule déjà explorée par le robot (true) ou non (false) (dans l'algorithme de planification de trajectoire)
        bool edge;         // info cellule qui fait partie du contour (zone de manoeuvre)

        bool mowed;        // info cellule qui est tondue réellement (true) ou non (false)
        std::vector<std::vector<int> >  node_voisin;  // index des cellules voisines
    };

    // Stocke tous les obstacles répertoriés (permet de gérer les replanifications de trajectoire si nouvel obstacle ou suppression d'obstacle)
    std::vector<std::vector<float> > obstacle; 

    std::vector<std::vector<Node> > tree;
    
    std::vector<std::vector<int> > past_goal;
    
    std::vector<std::vector<int> > past_mowed;

    nav_msgs::Path field_path;

    sensor_msgs::PointCloud mowed_field;

    sensor_msgs::PointCloud goal_field_msgs;

    bool relaunch;

    float diametre_tondeuse; 

    float ratio;

    float norme_A_star;

    float edge_inflation;

    float moitie_cell;

    float grass_z ;     // position z du point cloud illustrant les zones non tondues

    float cut_grass_z; // position z du point cloud illustrant les zones tondues



  Tree();
  void obstacleNode();
  void addNode(int L, int l, float x_min, float y_min);
  void printTree();
  void nodeNearEdge(nav_msgs::Path edge);
  void nodeInArea();
  void addNeighbors();
  std::vector<std::vector<int> > crossAround(int x_current,int y_current,int goal_x,int goal_y);
  void coveragePathPlanning(nav_msgs::Odometry odom,geometry_msgs::PoseStamped goal);

  std::vector<int> getNeighbors(int x, int y);
  bool checkNeighbors(int x, int y);

  std::vector<std::vector<int> > reconstruct_path(std::vector<std::vector<std::vector<int> > > cameFrom, std::vector<int> goal, int start_x, int start_y);
  std::vector<std::vector<int> > A_star(int start_x, int start_y,int goal_x, int goal_y);
  float distance_path(std::vector<std::vector<int> > path);

  void pathPushback(int index_x, int index_y);

  void getMowedCell(float x, float y);
  void reinitPathPlanning();

  void actualiser_z(int index_i, int index_j, float z);
  
};


#endif //CLASS_TREE
