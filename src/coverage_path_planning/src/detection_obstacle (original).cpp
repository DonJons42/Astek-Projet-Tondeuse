#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>

using namespace std;

/*  Ce programme permet de détecter les obstacles statiques et dynamiques et d'obtenir leurs coordonnées globales dans odom. On utilise pour cela
    l'hokuyo et le message LaserScan qu'il envoie.
    Cette détection d'obstacles permet de mettre à jour la zone à tondre après que le contour soit délimité. Les coordonnées des obstacles
    sont stockées dans un point cloud qui sera publié dans le topic "/detection_obstacle/coord".
    On utilise une norme entre les coordonnées des obstacles rencontrés pour ne pas surcharger leur analyse.
*/

float PI = 3.14159;
// Calcul des extremums du contour
float x_min;
float x_max;
float y_min;
float y_max;

float diametre_tondeuse = 0.6;  // Diamètre de la tondeuse du prototype utilisé
float norme_new_obstacle;       // Norme pour considérer un nouvel obstacle à rajouter

float norme_obs_dynamique = 0.25;  // Norme utilisée pour considérer un obstacle dynamique

struct Point{
  float x;
  float y;
};

struct InfoObstacle{
  float distance;
  float angle;
};

float signe(float A){ 
  if(A < 0){
    return -1;
  }
  else{
    return 1;
  }
}

float arrondi_2_chiffre(float A){
  return signe(A)*ceil(abs(A)*100)/100;  // ceil: plus petit entier supérieur ou égal au nombre donné
}

void edge_recu(const nav_msgs::Path &path){  // Calcul des extremums du contour
  x_min = path.poses[0].pose.position.x;
  x_max = path.poses[0].pose.position.x;

  y_min = path.poses[0].pose.position.y;
  y_max = path.poses[0].pose.position.y;

  for(int i=1; i<path.poses.size(); i++){
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
  x_min = arrondi_2_chiffre(x_min - 0.5 * diametre_tondeuse);          //--------------------------------------------0.5 ?--------------------------------------------------
  x_max = arrondi_2_chiffre(x_max + 0.5 * diametre_tondeuse);
  y_min = arrondi_2_chiffre(y_min - 0.5 * diametre_tondeuse);
  y_max = arrondi_2_chiffre(y_max + 0.5 * diametre_tondeuse);

  cout << "x_min = " << x_min << " x_max = " << x_max << endl;
  cout << "y_min = " << y_min << " y_max = " << y_max << endl;
}


nav_msgs::Path edge;  // Message du contour

vector<vector<float> > obs_repertories;

sensor_msgs::PointCloud pointcloud; // Regroupe les coordonnées des obstacles détectés

Point position; // Position de l'hokuyo
float angle_robot;

bool listenerOK = false;

void analyse_scan(const sensor_msgs::LaserScan &scan){
  if (listenerOK){ // Si le listener est initialisé, on peut analyser le scan de l'Hokuyo
    vector<vector<float> > info_scan_ranges;  // vecteur dans lequel les coordonnées des obstacles dans le repère du lidar sont stockées
    vector<int> info_scan_angle;              // vecteur dans lequel on stocke l'angle dans le scan
    float offset_angle = (scan.angle_max - scan.angle_min)/2; // offset qui permet de recadrer l'axe de l'hokuyo dans la bonne direction

    float range_max_hokuyo = scan.range_max;  
    int angle_indice_max = floor(scan.angle_max/scan.angle_increment);

    for (int i=0; i<scan.ranges.size(); i++){
      if (!isinf(scan.ranges[i])){  // si la valeur du scan est différente de "Inf" alors il y a un obstacle
        InfoObstacle info_obstacle;
        info_obstacle.distance = scan.ranges[i];                        // On récupère la distance de l'obstacle par rapport à l'hokuyo
        info_obstacle.angle = -offset_angle + scan.angle_increment * i; // Calcul de l'angle de l'obstacle dans le repère de l'hokuyo

        vector<float> obstacle;
        obstacle.resize(2);
        obstacle[0] = cos(info_obstacle.angle)*info_obstacle.distance; // Calcul des coordonées (x,y) de l'obstacle dans le repère de l'hokuyo
        obstacle[1] = sin(info_obstacle.angle)*info_obstacle.distance;

        if (info_scan_ranges.empty()){
          // Si le vecteur est vide on rajoute le premier obstacle détecté
          info_scan_ranges.push_back(obstacle);
          info_scan_angle.push_back(i);
        }

        else{
          // Sinon on analyse les obstacles suivant et on rajoute un nouvel obstacle si celui-ci est à une distance assez grande (diametre_tondeuse)
          bool ajout_info_scan_ranges;
            for (int j = 0; j<info_scan_ranges.size(); j++){
            ajout_info_scan_ranges = true;
            float norme = sqrt(pow(obstacle[0]-info_scan_ranges[j][0],2)+pow(obstacle[1]-info_scan_ranges[j][1],2));
            if(norme < norme_new_obstacle){
              ajout_info_scan_ranges = false;
              break;
            }

//adrien


	    if(norme < diametre_tondeuse/2){
              ajout_info_scan_ranges = false;
              break;
            }
          }
          
          if (ajout_info_scan_ranges){
            // si le nouvel obstacle respecte les conditions ci-dessus on rajoute ces coordonnées dans info_scan_ranges
            // et on rajoute l'angle du faisceau i
            info_scan_ranges.push_back(obstacle);
            info_scan_angle.push_back(i);
        
          }
        }
      }
    }


    bool suppr_obstacle;
    int norme_obs = 1000;
   
    for (int i = obs_repertories.size()-1; i >= 0; i--){
      // On parcourt la liste des obstacles déjà répertoriés (de manière décroissante) => FIFO ?
      norme_obs = 1000;
      suppr_obstacle = true;
      vector<float> obstacle;
      obstacle.resize(2);
      // Calcul des coordonnées des obstacles répertoriés dans le repère de l'hokuyo
      obstacle[0] = cos(angle_robot)*obs_repertories[i][0] + sin(angle_robot)*obs_repertories[i][1] - cos(angle_robot)*position.x - sin(angle_robot)*position.y;
      obstacle[1] = -sin(angle_robot)*obs_repertories[i][0] + cos(angle_robot)*obs_repertories[i][1] + sin(angle_robot)*position.x - cos(angle_robot)*position.y;

      float dist_obs_hokuyo = sqrt(pow(obstacle[0],2) + pow(obstacle[1],2)); // Distance entre l'obstacle et l'hokuyo
      float angle_obs_hokuyo = atan2(obstacle[1],obstacle[0]);               // Angle entre l'obstacle et l'hokuyo


      // Si l'obstacle repertorié est dans le champ de vision de l'hokuyo on regarde si c'est un obstacle dynamique ou non
      if((dist_obs_hokuyo <= range_max_hokuyo) && (abs(angle_obs_hokuyo) <= offset_angle)){

        int angle_indice = floor((angle_obs_hokuyo+offset_angle)/scan.angle_increment); // On calcul l'indice du faisceau qui pointe sur l'obstacle répertorié
        int diff_angle = 10000;
        int angle_plus_proche = 10000; // angle du faisceau qui se veut le plus proche de l'angle du faisceau pointant sur l'obstacle répertorié
        float diff_t;
        int record_j;

        for (int j = 0; j<info_scan_angle.size(); j++){ // On parcourt la liste des obstacles que l'on voit à l'instant t avec l'hokuyo
          // On cherche le faisceau le plus proche du faisceau pointant sur l'obstacle répertorié
          diff_t = abs(angle_indice-info_scan_angle[j]);
          if (diff_t < diff_angle){
            diff_angle = diff_t;
            angle_plus_proche = info_scan_angle[j];
            record_j = j;
          }
        }

        // Il faut être sûr que le faisceau trouvé soit cohérent avec le faisceau de l'obstacle répertorié. C'est pour cela qu'on met en place
        // une condition sur la différence (arbitraire) entre les deux faisceaux
        if (angle_plus_proche < scan.ranges.size() && abs(angle_plus_proche - angle_indice) < scan.ranges.size()/10){
          float distance = sqrt(pow(info_scan_ranges[record_j][0],2) + pow(info_scan_ranges[record_j][1],2));
          // On calcul la distance entre l'hokuyo et l'obstacle 

          if (distance < dist_obs_hokuyo){
            // Il y a un obstacle devant l'obstacle répertorié donc on ne peut pas le supprimer
            suppr_obstacle = false;
          }
        }

        // Calcul de la plus petite distance entre l'obstacle vu par l'hokuyo et l'obstacle répertorié
        for (int j = 0; j < info_scan_ranges.size(); j++){
          float norme = sqrt(pow(info_scan_ranges[j][0]-obstacle[0],2) + pow(info_scan_ranges[j][1]-obstacle[1],2));
          if (norme < norme_obs){
                norme_obs = norme; // On prend la norme la plus petite norme trouvée entre les obstacles dans le laser scan et l'obstacle déjà répertorié
          }
        }

        // On définit une norme pour être sûr que tous les obstacles détectés sont assez éloignés de l'obstacle, répertoriés et analysés
        if (norme_obs < norme_obs_dynamique){
          suppr_obstacle = false;        
        }

//adrien : on supprime l'obstacle si celui-ci se trouve à portée du robot
	if (dist_obs_hokuyo < diametre_tondeuse/2){ //rayon de la tondeuse
          suppr_obstacle = true;        
        }

        if(suppr_obstacle){
          // Si après toutes ces analyses suppr_obstacle est toujours true on supprime l'obstacle répertorié en question
          obs_repertories.erase(obs_repertories.begin()+i);
        }
      }
    }

    bool new_obstacle;
    // Analyse d'un nouvel obstacle
    for (int i = 0; i < info_scan_ranges.size(); i++){
      // On parcourt la liste des obstacles scannés par l'hokuyo
      vector<float> obstacle;
      obstacle.resize(2);
      // Changement de repère de l'hokuyo au repère de odom
      obstacle[0] = info_scan_ranges[i][0]*cos(angle_robot) - sin(angle_robot)*info_scan_ranges[i][1] + position.x;
      obstacle[1] = info_scan_ranges[i][0]*sin(angle_robot) + cos(angle_robot)*info_scan_ranges[i][1] + position.y;

      if ((obstacle[0] <= x_max) && (obstacle[0] >= x_min) && (obstacle[1] <= y_max) && (obstacle[1] >= y_min)){
        new_obstacle = true;
        // On parcourt la liste des obstacles répertoriés pour voir s'il est déjà dedans (analyse avec une norme)
        for(int j = 0; j < obs_repertories.size(); j++){
          float norme = sqrt(pow(obstacle[0]-obs_repertories[j][0],2) + pow(obstacle[1]-obs_repertories[j][1],2));
          if(norme < norme_new_obstacle){
            // L'obstacle est déjà répertorié, on ne le rajoute pas et on sort de la boucle
            new_obstacle = false;
            break;
          }
        }
        // Si l'obstacle n'est pas dans la liste des obstacles répertoriés on le rajoute à la fin
        if(new_obstacle){
          obs_repertories.push_back(obstacle);
        }
      }
    }

   // On va mettre à jour le message poincloud à publier
    geometry_msgs::Point32 obstacle_pointcloud;
    pointcloud.points.clear();
    for (int i = 0; i < obs_repertories.size(); i++){
      // On parcourt la liste des obstacles répertoriés et les push back dans le msg point clouds
      obstacle_pointcloud.x = obs_repertories[i][0];
      obstacle_pointcloud.y = obs_repertories[i][1];
      pointcloud.points.push_back(obstacle_pointcloud);
    }

  }
}

int main(int argc, char *argv[]){

  srand(time(0));
  ros::init(argc, argv, "detection_obstacle");
  ros::NodeHandle detection_obstacle;

  //Pour la simulation réelle 
  ros::Subscriber sub = detection_obstacle.subscribe("/scan",1000,analyse_scan);

  //Pour la simulation informatique
  //ros::Subscriber sub = detection_obstacle.subscribe("/hokuyo/laser/scan",1000,analyse_scan);

  ros::Publisher pub1 = detection_obstacle.advertise<sensor_msgs::PointCloud>("/detection_obstacle/coord",1000); // topic qui va permettre

  if(!detection_obstacle.getParam("/General/norme_new_obstacle",norme_new_obstacle)){
    cout << "Veuillez charger les paramètres avec la commande : rosparam load" << endl;
    return 0;
  }

  // Récupérer les données du contour
  boost::shared_ptr<nav_msgs::Path const> sharedEdge;
  
  sharedEdge = ros::topic::waitForMessage<nav_msgs::Path>("/path_planned/edge", detection_obstacle);
  if(sharedEdge != NULL){
    edge = *sharedEdge;
  }

  edge_recu(edge);

  ros::Rate loop_rate(10);

  pointcloud.header.frame_id = "/odom";

  // Foncion appelée par le subscriber pour calculer les coordonées de l'obstacle dans le repère de l'hokuyo
  tf::TransformListener listener; // Listener de la TF entre odom et l'hokuyo
  tf::StampedTransform transform; // Récupère la TF entre odom et l'hokuyo

  cout << "DETECTION OBSTACLE" << endl;
  while (ros::ok()){

    ros::spinOnce();

    try{
      listenerOK = true; // Permet de savoir si le listener est à jour. Utile dans la fonction analyse_scan du subscriber 
      listener.lookupTransform("/odom", "/base_laser_link", ros::Time(0), transform);

      position.x = transform.getOrigin().x();            // Position de l'hokuyo dans le repère de odom
      position.y = transform.getOrigin().y();
      angle_robot = tf::getYaw(transform.getRotation()); // Orientation du robot suivant l'axe z
    }

  catch (tf::TransformException ex){
  ROS_ERROR("%s",ex.what());
  ros::Duration(1.0).sleep();
  listenerOK = false;
  }

  // on publie le nouveau pointcloud
  pub1.publish(pointcloud);
  loop_rate.sleep();
  }

  return 0;
}
