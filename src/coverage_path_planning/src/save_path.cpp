

/* Ce code permet de récupérer les coordonnées du contour lors de la tonte manuelle. Pour cela, nous souscrivons au topic "/odometry/filtered".
   Nous enregistrons les coordonnées dans un fichier .txt comportant une seule colonne de valeurs et dont les lignes impaires correspondent aux x et les lignes paires aux y.
   Pour obtenir un contour fermé, nous prenons en compte les coordonnées du point de départ et du point d'arrivée. 
   Si la distance entre ces deux points est supérieure à une certaine valeur, nous créons des points qui se trouvent sur la droite formée par ces deux points. 
   Dans le cas contraire, nous considérons le contour comme fermé.
*/
/*------------------------------------------------------------------Modifié-par-Luc-ZHENG-le-16-octobre-2018---------------------------------------------------------------------*/


// Ajout des bibliothèques nécessaires.
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

using namespace std;

float x_current;
float y_current;
float larg_tondeuse;
float norme_save;

void pose_recu(const nav_msgs::Odometry& poses){   
  float x = poses.pose.pose.position.x;
  float y = poses.pose.pose.position.y;
  // Norme à considérer pour prendre un nouveau point. Si il est plus loin que "standard_new_point" à savoir 0.5m (défini dans param.yaml).
  if((x_current == 0 && y_current == 0) || sqrt(pow((x_current - x),2) + pow((y_current - y),2)) >= norme_save){
    x_current = x;
    y_current = y;
  }
}


int main(int argc, char *argv[]){ // Début de la fonction principale du node.

  srand(time(0));
	ros::init(argc, argv, "init_path"); // Initialisation du node "init_path". Cette fonction contacte le master et enregistre le node dans le système. 

	ros::NodeHandle save_path; // Cette fonction est requise pour les interactions avec le système comme suscribe ou advertise ci-dessous. 

	ros::Subscriber sub = save_path.subscribe("/odometry/filtered",1000,pose_recu); // Souscrit au topic "/odometry/filtered" 

  ros::Publisher pub = save_path.advertise<nav_msgs::Path>("first_path",1000); // Envoie un message au topic "first_path"

  ros::Rate loop_rate(10); // Définit la fréquence des actions à répétitions.

  nav_msgs::Path path_msgs; // Message "nav" de type "Path" qui permet de gérer des cartes ou des données de localisation de robot.

  cout << x_current << " " << y_current << endl;

  if(!save_path.getParam("/General/diametre_tondeuse",larg_tondeuse)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load parameters.yaml"  << endl;
    return 0;
  }

  cout << "DIAMETRE TONDEUSE: " << larg_tondeuse << endl;

  if(!save_path.getParam("/SavePath/norme_save",norme_save)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load parameters.yaml"  << endl;
    return 0;
  }

  cout << "NORME SAVE: " << norme_save << endl;


  
  int count = 0; // Compte le nombre de messages envoyés et permet de créer une chaîne unique de caractères pour chaque message.
  while (ros::ok()){ // Vérification du bon fonctionnement de ROS. Si le master est arrêté ou si un signal a été envoyé pour arrêter le système, ros::ok() renverra false.
    ros::spinOnce(); // Traite tous les messages entrants.

    path_msgs.header.frame_id = "odom";
    path_msgs.header.stamp = ros::Time::now();

    // Initialisation + on vérifie si le dernier point rajouté est différent des coordonnées (x,y) les plus récentes du robot
    if(path_msgs.poses.size() == 0 || (path_msgs.poses[count-1].pose.position.x != x_current && path_msgs.poses[count-1].pose.position.y != y_current) ){
      
      geometry_msgs::PoseStamped pose_msgs; // Message "geometry" de type "PoseStamped" traitant des informations concernant la position, l'orientation, la vitesse ou l'accélération d'un objet.
      pose_msgs.header.frame_id = "odom";
      pose_msgs.header.stamp = ros::Time::now();
      pose_msgs.pose.position.x = x_current;
      pose_msgs.pose.position.y = y_current;
      pose_msgs.pose.position.z = 0;

      pose_msgs.pose.orientation.w = 1;
      pose_msgs.pose.orientation.x = 0;
      pose_msgs.pose.orientation.y = 0;
      pose_msgs.pose.orientation.z = 0;
      path_msgs.poses.push_back(pose_msgs);
      count++;
    }
    //cout << "x =" << x_current << " y = " << y_current << endl;
    pub.publish(path_msgs); // La fonction"publish" permet d'envoyer des messages. L'objet du message est mis en paramètre.
    loop_rate.sleep();
  }


  // Message "geometry" de type "PoseStamped" traitant des informations concernant la position, l'orientation, la vitesse ou l'accélération d'un objet.
  geometry_msgs::PoseStamped pose_depart;
  pose_depart = path_msgs.poses[1];

  geometry_msgs::PoseStamped pose_arrivee;
  pose_arrivee = path_msgs.poses.back();

/* Il est nécessaire d'avoir un contour fermé. C'est pourquoi nous analysons les points de départ et d'arrivée de la tondeuse.
   Si ces deux points sont trop distants l'un de l'autre, nous rajoutons de nouveaux points suivant la droite qu'ils forment, ce qui permet de fermer le contour.
*/


  // Calcul de la norme entre les points de départ et d'arrivée
  float distance = sqrt(pow(pose_arrivee.pose.position.x - pose_depart.pose.position.x,2) + pow(pose_arrivee.pose.position.y - pose_depart.pose.position.y,2));
  



  // Si cette norme est trop importante, nous générons des  points entre les points de départ et d'arrivée.
  if(distance > larg_tondeuse){
    float x_diff = pose_arrivee.pose.position.x - pose_depart.pose.position.x;
    float y_diff = pose_arrivee.pose.position.y - pose_depart.pose.position.y;
    // Calcul du nombre de points à rajouter entre le départ et l'arrivée
    int nb_point = floor(distance / (larg_tondeuse / 2));

    // On part du dernier point généré pour le contour (à savoir le point d'arrivée) et rajoutons de nouveaux points jusqu'au point d'arrivée du contour.
    for (int i = nb_point; i >= 0; i--){
      // Les coordonnées des nouveaux points sont calculées
      float new_x = (i * x_diff / nb_point ) + pose_depart.pose.position.x;
      float new_y = (i * y_diff / nb_point ) + pose_depart.pose.position.y;

      // cout << " x= " << new_x << " et new_y= " << new_y << endl; // Affiche les points générés



      // On rajoute ces points à la suite du path message qui correspond au contour.
      geometry_msgs::PoseStamped pose_msgs;
      pose_msgs.header.frame_id = "odom";
      pose_msgs.header.stamp = ros::Time::now();
      pose_msgs.pose.position.x = new_x;
      pose_msgs.pose.position.y = new_y;
      pose_msgs.pose.position.z = 0;

      pose_msgs.pose.orientation.w = 1;
      pose_msgs.pose.orientation.x = 0;
      pose_msgs.pose.orientation.y = 0;
      pose_msgs.pose.orientation.z = 0;
      path_msgs.poses.push_back(pose_msgs);
      
    }
  }

/* Ecriture des coordonnées du contour dans un fichier .txt. 
   Ce fichier comporte une seule colonne de valeurs, les lignes impaires correspondent aux x et les lignes paires aux y.
*/
  ofstream path_file; 
  path_file.open("path.txt");
  for(int i = 1; i < path_msgs.poses.size(); i++){
    path_file << path_msgs.poses[i].pose.position.x << "\n" << path_msgs.poses[i].pose.position.y << "\n";
  }
  path_file.close();

  return 0;
}

