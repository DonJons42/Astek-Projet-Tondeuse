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
#include "sensor_msgs/PointCloud.h"
#include <string>
#include "geometry_msgs/Point32.h"
#include <cmath>
#include <climits>

#include "classTree.h"

using namespace std;

float x_min;
float x_max;

float y_min;
float y_max;

bool relaunch_path_planning;

nav_msgs::Path edge;

nav_msgs::Odometry odom;

geometry_msgs::PoseStamped goal;

Tree treeField;  




float signe(float A){
  if(A < 0)
    return -1;
  else
    return 1;
}

float arrondi_2_chiffre(float A){
  return signe(A)*ceil(abs(A)*100)/100;
}

// Calcul des extremums en x et y du contour
void edge_recu(const nav_msgs::Path &path){
  x_min = path.poses[0].pose.position.x;
  x_max = path.poses[0].pose.position.x;

  y_min = path.poses[0].pose.position.y;
  y_max = path.poses[0].pose.position.y;

  for(int i = 1; i<path.poses.size(); i++){
    if(path.poses[i].pose.position.x <= x_min){
      x_min = path.poses[i].pose.position.x;
    }
    else if(path.poses[i].pose.position.x >= x_max){
      x_max = path.poses[i].pose.position.x;
    }
    if(path.poses[i].pose.position.y <= y_min){
      y_min = path.poses[i].pose.position.y;
    }
    else if(path.poses[i].pose.position.y >= y_max){
      y_max = path.poses[i].pose.position.y;
    }
  }
  x_min = arrondi_2_chiffre(x_min - treeField.edge_inflation * treeField.diametre_tondeuse);    
  x_max = arrondi_2_chiffre(x_max + treeField.edge_inflation * treeField.diametre_tondeuse);
  y_min = arrondi_2_chiffre(y_min - treeField.edge_inflation * treeField.diametre_tondeuse);
  y_max = arrondi_2_chiffre(y_max + treeField.edge_inflation * treeField.diametre_tondeuse);

  cout << "x_min = " << x_min << " x_max = " << x_max << endl;
  cout << "y_min = " << y_min << " y_max = " << y_max << endl;
}


void calcul_border(){
  int x;
  int y;
  x = abs(ceil((x_max - x_min)/(treeField.ratio*treeField.diametre_tondeuse)));                    // x ??????????        y ?????????????
  y = abs(ceil((y_max - y_min)/(treeField.ratio*treeField.diametre_tondeuse)));
  cout << "NOMBRE MAX DE CASES A TONDRE : " << x << " lignes et " << y << " colonnes " << endl;   
  treeField.addNode(x,y,x_min,y_min);
  cout << "SURFACE MAX A TONDRE : " << x_max - x_min << " x " << y_max - y_min << endl;            // ??????????????????? carré ?????????????????????
}


// Fonction appelée lorsqu'on détecte un obstacle. On modifie l'information occupied = true de la cellule où l'obstacle se trouve
// ainsi que les cellules correspondantes à l'inflation
void ajouter_occupied(int index){
  bool flag = false;
  for(int i = 0; i < treeField.tree.size(); i++){
    for(int j = 0; j < treeField.tree[i].size(); j++){
      if((treeField.obstacle[index][0] <= treeField.tree[i][j].x + treeField.moitie_cell) && (treeField.obstacle[index][0] >= treeField.tree[i][j].x - treeField.moitie_cell) && (treeField.obstacle[index][1] <= treeField.tree[i][j].y + treeField.moitie_cell) && (treeField.obstacle[index][1] >= treeField.tree[i][j].y - treeField.moitie_cell)){
        //cout << "occupied cell de l'obstacle" << endl;
        treeField.tree[i][j].occupied = true;
        treeField.actualiser_z(i, j, -10000);
        for(int k = 0; k<treeField.tree[i][j].node_voisin.size(); k++){
          int index_x = treeField.tree[i][j].node_voisin[k][0];
          int index_y = treeField.tree[i][j].node_voisin[k][1];
          float norme_voisin = sqrt(pow(treeField.tree[index_x][index_y].x - treeField.obstacle[index][0],2) + pow(treeField.tree[index_x][index_y].y - treeField.obstacle[index][1],2));
          if(norme_voisin < treeField.diametre_tondeuse){
            treeField.tree[index_x][index_y].occupied = true;
            treeField.actualiser_z(index_x, index_y, -10000);
          }
        }
        flag = true;
        break;
      }
    }
    if(flag){
      break;
    }
  }
}


// Fonction appelée pour les objets dynamiques. Permet de remettre l'information occupied = false dans les cellules qui sont de nouveau libres
void supprimer_occupied(int index){
  bool flag = false;
  for(int i = 0; i < treeField.tree.size(); i++){
    for(int j = 0; j < treeField.tree[i].size(); j++){
      if((treeField.obstacle[index][0] <= treeField.tree[i][j].x + treeField.moitie_cell) && (treeField.obstacle[index][0] >= treeField.tree[i][j].x - treeField.moitie_cell) && (treeField.obstacle[index][1] <= treeField.tree[i][j].y + treeField.moitie_cell) && (treeField.obstacle[index][1] >= treeField.tree[i][j].y - treeField.moitie_cell)){
        treeField.tree[i][j].occupied = false;
        // On supprime les zones d'inflation autour de l'obstacle
        for(int k = 0; k <treeField.tree[i][j].node_voisin.size(); k++){
          int index_x = treeField.tree[i][j].node_voisin[k][0];
          int index_y = treeField.tree[i][j].node_voisin[k][1];
          treeField.tree[index_x][index_y].occupied = false;
          treeField.actualiser_z(index_x, index_y, treeField.grass_z);

        }
        flag = true;
        break;
      }
    }
    if(flag){
      break;
    }
  }
}



// Fonction appelée par le subscriber des obstacles envoyés par l'hokuyo
// Permet d'analyser ces obstacles et de déclencher ou non la replanification de trajectoire (si nouvel obstacle ou obstacle supprimé)
void obstacle_recu(const sensor_msgs::PointCloud& cloud_obstacle){
  vector<int> obstacle_a_supprimer; // On stocke les indices des obstacles à supprimer
  bool suppr_obstacle;
  relaunch_path_planning = false;

  for(int i = 0; i < treeField.obstacle.size(); i++){
    // On parcourt les obstacles répertoriés
    suppr_obstacle = true;
    for(int j = 0; j < cloud_obstacle.points.size();j++){
      // On parcourt les obstacles détectés par l'hokuyo
      if(cloud_obstacle.points[j].x == treeField.obstacle[i][0] && cloud_obstacle.points[j].y == treeField.obstacle[i][1]){
        // L'obstacle est toujours présent (on ne le supprime pas)
        suppr_obstacle = false;
        break;
      }
    }
    if(suppr_obstacle == true){
      obstacle_a_supprimer.push_back(i); // On rajoute l'indice de l'obstacle à supprimer
      supprimer_occupied(i);         // supprimer l'occupied de tous les voisins autour (on va devoir rajouter l'inflation autour des obstacles non
      // non supprimés mais dont leur inflation à été modifié) 
      relaunch_path_planning = true; // On a supprimé un obstacle on doit replanifier la trajectoire
    }
  }

  for(int i = obstacle_a_supprimer.size() - 1; i >= 0; i--){
    // On supprime les obstacles qui étaient répertoriés
    treeField.obstacle.erase(treeField.obstacle.begin()+obstacle_a_supprimer[i]);
  }

  // Boucle qui permet de remettre les zone occupied autour des obstacles dont les zone d'inflation on été touché par la suppression d'un obstacle proche)
  if (suppr_obstacle == true){
    for (int l = 0; l < treeField.obstacle.size(); l++){
      ajouter_occupied(l);
    }
  }


  bool new_obstacle;
  // Partie où l'on regarde si un nouvel obstacle est apparu
  for(int i = 0; i < cloud_obstacle.points.size(); i++){
    new_obstacle = true;
    for(int j = 0; j < treeField.obstacle.size(); j++){
      if(cloud_obstacle.points[i].x == treeField.obstacle[j][0] && cloud_obstacle.points[i].y == treeField.obstacle[j][1]){
        // L'obstacle est déjà répertorié donc pas de nouvel obstacle
        new_obstacle = false;
        break;
      }
    }

    if (new_obstacle != false){
      // Pour le moment l'obstacle est considéré comme nouveau. On vérifie désormais si cet obstacle est dans la zone de tonte.
      // S'il n'est pas dans la zone ça ne sert à rien de replanifier. Pour cela on verifie si la cellule où l'obstacle se trouve
      // est  en background ou non
      bool flag = false;
      for(int k = 0; k < treeField.tree.size(); k++){
        for(int l = 0; l < treeField.tree[k].size(); l++){
          if((cloud_obstacle.points[i].x <= treeField.tree[k][l].x + treeField.moitie_cell) && (cloud_obstacle.points[i].x >= treeField.tree[k][l].x - treeField.moitie_cell) && (cloud_obstacle.points[i].y <= treeField.tree[k][l].y + treeField.moitie_cell) && (cloud_obstacle.points[i].y >= treeField.tree[k][l].y - treeField.moitie_cell)){
            if (treeField.tree[k][l].background == true){
              new_obstacle = false;
            }
            flag = true;
            break;
          }
        }
        if (flag){
          break;
        }
      }
    }

    // Un nouvel obstacle a été détecté, on le rajoute à la liste des obstacles répertoriés et on met à jour la zone occupied
    if(new_obstacle == true){

      vector<float> newObstacle;
      newObstacle.resize(2);
      newObstacle[0] = cloud_obstacle.points[i].x;
      newObstacle[1] = cloud_obstacle.points[i].y;
      treeField.obstacle.push_back(newObstacle);

      ajouter_occupied(treeField.obstacle.size()-1);
      relaunch_path_planning = true;
    }
  }
}


geometry_msgs::Point32 mowed_cell;
int mowed_cell_header_seq;
int last_seq_header = 1;

// Fonction qui permet d'actualiser la liste des zones tondues. On reçoit l'information via un message publié par next_goal.cpp
void mowed_cell_recu(const geometry_msgs::PoseStamped& cell_reached){
  cout << "debut mowed_cell_recu" << endl;
  mowed_cell.x = cell_reached.pose.position.x;
  mowed_cell.y = cell_reached.pose.position.y;
  mowed_cell.z = treeField.cut_grass_z;
  cout << "cell reached = " << cell_reached.header.seq << endl;
  cout << "last_seq_header = " << last_seq_header << endl;
  
  // Si une nouvelle zone a été tondue on met à jour la grille
  if (last_seq_header == cell_reached.header.seq){
    last_seq_header++;
    treeField.mowed_field.points.push_back(mowed_cell);
    treeField.getMowedCell(mowed_cell.x, mowed_cell.y);
  }
  cout << "fin mowed_cell_recu" << endl;
}

void goal_recu(const geometry_msgs::PoseStamped& PoseStamped_goal){
  goal.pose.position.x = PoseStamped_goal.pose.position.x;
  goal.pose.position.y = PoseStamped_goal.pose.position.y;
}


int main(int argc, char *argv[]){
  /*---------INITIALISATION DE ROS---------*/
  srand(time(0));
  ros::init(argc, argv, "create_path");
  ros::NodeHandle create_path;
  ros::Rate loop_rate(10);
  /*---------------------------------------*/

  // Initialisation du publisher sur le topic /path_planned/field de type sensor_msgs::PointCloud
  ros::Publisher pub1 = create_path.advertise<sensor_msgs::PointCloud>("/path_planned/field",1000);
  ros::Publisher pub2 = create_path.advertise<nav_msgs::Path>("/path_planned/path",1000);
  ros::Publisher pub3 = create_path.advertise<sensor_msgs::PointCloud>("/path_planned/mowed_field",1000);
  ros::Publisher pub4 = create_path.advertise<sensor_msgs::PointCloud>("/detection_obstacle/inflation",1000);

  ros::Subscriber sub1 = create_path.subscribe("/detection_obstacle/coord",1000,obstacle_recu);
  ros::Subscriber sub2 = create_path.subscribe("/path_planned/mowed_cell",1000,mowed_cell_recu);
  ros::Subscriber sub3 = create_path.subscribe("path_planned/goal",1000,goal_recu);
  
  if(!create_path.getParam("/General/diametre_tondeuse",treeField.diametre_tondeuse)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load" << endl;
    return 0; 
  }

  cout << "DIAMETRE TONDEUSE : " << treeField.diametre_tondeuse << endl;

  if(!create_path.getParam("/CreatePath/ratio",treeField.ratio)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load" << endl;
    return 0;
  }

  cout << "RATION TONDEUSE/CELLULE : " << treeField.ratio << endl;

  if(!create_path.getParam("/CreatePath/norme_A_star",treeField.norme_A_star)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load" << endl;
    return 0;
  }

  cout << "NORME A* : " << treeField.norme_A_star << endl;

  if(!create_path.getParam("/CreatePath/edge_inflation",treeField.edge_inflation)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load" << endl;
    return 0;
  }

  cout << "INFLATION DU EDGE : " << treeField.edge_inflation << endl;

  treeField.moitie_cell = treeField.diametre_tondeuse * treeField.ratio / 2 ;

   /*--------------------------------------------------------------------------------------------*\
  | Utilisation de waitForMessage pour recuperer une seule fois les données de /path_planned/edge  |
   \*--------------------------------------------------------------------------------------------*/

  boost::shared_ptr<nav_msgs::Path const> sharedEdge;
  
  sharedEdge = ros::topic::waitForMessage<nav_msgs::Path>("/path_planned/edge", create_path);
  if(sharedEdge != NULL){
    edge = *sharedEdge;
  }


  edge_recu(edge);
  calcul_border();
  treeField.nodeNearEdge(edge);
  treeField.nodeInArea();
  treeField.addNeighbors();
  //treeField.printTree();

  boost::shared_ptr<nav_msgs::Odometry const> sharedOdom;
  
  sharedOdom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered_map", create_path);
  if(sharedOdom != NULL){
    odom = *sharedOdom;
  }

  treeField.coveragePathPlanning(odom,goal);
  
  treeField.goal_field_msgs.header.frame_id = "odom";
  for(int i = 0; i < treeField.tree.size(); i++){
    for(int j = 0; j < treeField.tree[i].size(); j++){
      geometry_msgs::Point32 point;
      point.x = treeField.tree[i][j].x;
      point.y = treeField.tree[i][j].y;
      if(treeField.tree[i][j].background == false){
        point.z = treeField.grass_z;
      }
      else{
        point.z = -1000;
      }

      treeField.goal_field_msgs.points.push_back(point);
    }
  }

  treeField.mowed_field.header.frame_id = "odom";

  // Point cloud représentant l'inflation dû aux obstacles
  sensor_msgs::PointCloud inflation;
  inflation.points.clear();
  inflation.header.frame_id = "odom";

  cout << " GO TO WHILE " << endl;

  treeField.field_path.header.frame_id = "odom";

  while (ros::ok()){
    //cout << "boucle while" << endl;
    ros::spinOnce();
    treeField.field_path.header.stamp = ros::Time::now();
    treeField.goal_field_msgs.header.stamp = ros::Time::now();

    // On recalcule la trajectoire puisque la grille a été mise à jour (nouvel obstacle détecté ou suppression obstacle dynamique)

    if (relaunch_path_planning){
      cout << "début relaunch" << endl;
      treeField.field_path.poses.clear(); // efface les infos du path avant de le recalculer

      treeField.reinitPathPlanning(); // reinitialise les infos visited et node_voisin

      treeField.obstacleNode();
      // On actualise la position du robot pour replanifier la trajectoire
      sharedOdom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered_map", create_path);
      if(sharedOdom != NULL){
        odom = *sharedOdom;
      }
      treeField.coveragePathPlanning(odom,goal);
      cout << "fin relaunch" << endl;
    }

    pub1.publish(treeField.goal_field_msgs);
    pub2.publish(treeField.field_path);
    pub3.publish(treeField.mowed_field);
    pub4.publish(inflation);
    loop_rate.sleep();
  }
  return 0;
}
