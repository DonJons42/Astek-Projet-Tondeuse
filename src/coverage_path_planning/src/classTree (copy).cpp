#include "classTree.h"

using namespace std;

Tree::Tree(){
	relaunch = false;
	grass_z = -0.1;     
	cut_grass_z = -0.3; 
}


void Tree::obstacleNode(){
  for(int i = 0; i < obstacle.size(); i++){
    for(int j = 0; j < tree.size(); j++){
      for(int k = 0; k < tree[j].size(); k++){
        if(abs(obstacle[i][0] - tree[j][k].x) < moitie_cell && abs(obstacle[i][1] - tree[j][k].y) < moitie_cell ){
          for(int l = 0; l < tree[j][k].node_voisin.size(); l++){
            int index_x = tree[j][k].node_voisin[l][0];
            int index_y = tree[j][k].node_voisin[l][1];
            if(abs(tree[index_x][index_y].x-obstacle[i][0]) < 2*moitie_cell && abs(tree[index_x][index_y].y-obstacle[i][1]) < 2*moitie_cell){
              tree[tree[j][k].node_voisin[l][0]][tree[j][k].node_voisin[l][1]].occupied = true;
              actualiser_z(index_x, index_y, -10000);
            }
          }
          tree[j][k].occupied = true;
          actualiser_z(j,k,-10000);
        }
      }
    }
  }
}

// Permet de retourner l'index du premier noeud voisin de (x, y) non visité, non tondu et non occupé (càd qu'il ne représente pas un obstacle du terrain)
vector<int> Tree::getNeighbors(int x, int y){
  vector<int> index;
  index.resize(2);
  for(int i = 0; i < tree[x][y].node_voisin.size(); i++){
    if((tree[tree[x][y].node_voisin[i][0]][tree[x][y].node_voisin[i][1]].visited == false) && (tree[tree[x][y].node_voisin[i][0]][tree[x][y].node_voisin[i][1]].mowed == false) && (tree[tree[x][y].node_voisin[i][0]][tree[x][y].node_voisin[i][1]].occupied == false)){
      index[0] = tree[x][y].node_voisin[i][0];
      index[1] = tree[x][y].node_voisin[i][1];
      return index;
    }
  }
}

// Booléen verifiant si le noeud actuel (x, y) possède un voisin non visité, non tondu et non occupé
bool Tree::checkNeighbors(int x, int y){
  for(int i = 0; i < tree[x][y].node_voisin.size(); i++){
    if((tree[tree[x][y].node_voisin[i][0]][tree[x][y].node_voisin[i][1]].visited == false) && (tree[tree[x][y].node_voisin[i][0]][tree[x][y].node_voisin[i][1]].mowed == false) && (tree[tree[x][y].node_voisin[i][0]][tree[x][y].node_voisin[i][1]].occupied == false)){
      return true;
    }
  }
  return false;
}

void Tree::pathPushback(int index_x, int index_y){
  geometry_msgs::PoseStamped first_goal;
  first_goal.header.frame_id = "odom";
  first_goal.header.stamp = ros::Time::now();
  first_goal.pose.position.x = tree[index_x][index_y].x;
  first_goal.pose.position.y = tree[index_x][index_y].y;
  first_goal.pose.position.z = 0;
  first_goal.pose.orientation.w = 1;
  first_goal.pose.orientation.x = 0;
  first_goal.pose.orientation.y = 0;
  first_goal.pose.orientation.z = 0;

  field_path.poses.push_back(first_goal);
}

/* coveragePathPlanning construit le chemin pour visiter toute la zone de tonte */
void Tree::coveragePathPlanning(nav_msgs::Odometry odom,geometry_msgs::PoseStamped goal){

  cout << "DEBUT COVERAGE" <<endl;
  float norme = 1000;

  int x_start;
  int y_start;


  if (relaunch == false) {
    for(int i = 0; i < tree.size(); i++){
      for(int j = 0; j < tree[i].size(); j++){
        if(tree[i][j].occupied == 0 && tree[i][j].background == 0){
          float norme_current = sqrt((odom.pose.pose.position.x-tree[i][j].x)*(odom.pose.pose.position.x-tree[i][j].x)+(odom.pose.pose.position.y-tree[i][j].y)*(odom.pose.pose.position.y-tree[i][j].y));

          if(norme_current < norme){
            norme = norme_current;
            x_start = i;
            y_start = j;
            relaunch = true;
          }
        }
      }
    }
  }

  else {
    for(int i = 0; i < tree.size(); i++){
      for(int j = 0; j < tree[i].size(); j++){
        if (goal.pose.position.x == tree[i][j].x && goal.pose.position.y == tree[i][j].y){
          x_start = i;
          y_start = j;
        }
      }
    }
  }

  cout << " POINT DE DEPART : " << x_start << " " << y_start << endl;

  tree[x_start][y_start].visited = true;
  tree[x_start][y_start].mowed = true;

  
  geometry_msgs::Point32 mowed_cell;
  mowed_cell.x = tree[x_start][y_start].x;
  mowed_cell.y = tree[x_start][y_start].y;
  mowed_cell.z = cut_grass_z;
  actualiser_z(x_start, y_start, -10000);  // On fait disparaitre la zone qui vient juste d'être tondue (elle sera remplacée par le point cloud mowed_field)
  mowed_field.points.push_back(mowed_cell);

  pathPushback(x_start,y_start);

  bool neighbors_available = true;

  int x_current = x_start;
  int y_current = y_start;
  
  vector<int> index;
  index.resize(2);

  vector<int> ind_edge;
  ind_edge.resize(2);

  index[0] = x_current;
  index[1] = y_current;

  int goal_x;
  int goal_y;

  past_goal.push_back(index);

  while(neighbors_available){

    // CAS NORMAL :  on prend le premier voisin non visité du noeud actuel
    if(checkNeighbors(index[0],index[1]) == true){
      index = getNeighbors(index[0],index[1]);
      pathPushback(index[0],index[1]);
      x_current = index[0];
      y_current = index[1];
      tree[x_current][y_current].visited = true;
      past_goal.push_back(index);
    }

    else{
      float norme;
      bool past_neighbors_available = true;
      cout << " CAS SPECIAL 1 : START : " << index[0] << " " << index[1] << endl;
      cout << "voisin :";
      for(int i = 0; i< tree[index[0]][index[1]].node_voisin.size();i++){
        cout << " [" << tree[index[0]][index[1]].node_voisin[i][0] << " " << tree[index[0]][index[1]].node_voisin[i][1] << "] " << tree[tree[index[0]][index[1]].node_voisin[i][0]][tree[index[0]][index[1]].node_voisin[i][1]].background ;
      }
      cout << endl;
      // CAS SPECIAL : si le noeud actuel n'a pas de voisin non visité, on regarde si un des noeuds visités précédement a un voisin non visité
      for(int i = past_goal.size()-1; i >= 0; i--){
        if(checkNeighbors(past_goal[i][0],past_goal[i][1]) == true){
          index = getNeighbors(past_goal[i][0],past_goal[i][1]);

          float norme = sqrt(pow(tree[index[0]][index[1]].x - tree[x_current][y_current].x,2)+pow(tree[index[0]][index[1]].y - tree[x_current][y_current].y,2));
          cout << "norme : " << norme << " " << norme_A_star << endl;
          if ( norme > norme_A_star){
            cout << "EXECUTION A*" << endl;
            vector<vector<int> > ind_edge = crossAround(x_current,y_current,index[0],index[1]);
            cout << ind_edge.size() << endl;
            for(int j = ind_edge.size()-1; j >= 0; j--){
              cout << "[" << ind_edge[j][0] << " " << ind_edge[j][1] << "] ";
              past_goal.push_back(ind_edge[j]);
              pathPushback(ind_edge[j][0],ind_edge[j][1]);
            }
            cout << endl;
          }
          pathPushback(index[0],index[1]);
          x_current = index[0];
          y_current = index[1];
          tree[x_current][y_current].visited = true;

          cout << " END : " << index[0] << " " << index[1] << " " << tree[index[0]][index[1]].background << endl;
          past_goal.push_back(index);
          break;
        }

        // si AUCUN noeud visité précédement n'a de voisin dispo, on doit donc verifier le CAS SPECIAL 2 
        else if(i == 0){
          past_neighbors_available = false;
        }
      }
      
      // CAS SPECIAL 2 : si aucun noeud visité précédement n'a de voisin non visité, on regarde si un des noeuds tondu précédement a un voisin non visité
      if(!past_neighbors_available){
        // on verifie si le vecteur des noeuds tondu précédement n'est pas vide
        if(past_mowed.size() != 0){
          for(int i = past_mowed.size() - 1; i>=0; i--){
            cout << i << endl;
            if(checkNeighbors(past_mowed[i][0],past_mowed[i][1]) == true){
              index = getNeighbors(past_mowed[i][0],past_mowed[i][1]);

              float norme = sqrt(pow(tree[index[0]][index[1]].x - tree[x_current][y_current].x,2)+pow(tree[index[0]][index[1]].y - tree[x_current][y_current].y,2));
              if ( norme > norme_A_star){
                vector<vector<int> > ind_edge_mowed = crossAround(x_current,y_current,index[0],index[1]);
                for(int j = ind_edge_mowed.size()-1; j >= 0; j--){
                  past_goal.push_back(ind_edge_mowed[j]);
                  pathPushback(ind_edge_mowed[j][0],ind_edge_mowed[j][1]);
                }
              }

              pathPushback(index[0],index[1]);
              
              x_current = index[0];
              y_current = index[1];

              tree[x_current][y_current].visited = true;

              past_goal.push_back(index);
              break;
            }
            // si AUCUN noeud tondu précédement n'a de voisin dispo, alors le programme s'arrete car on aura planifié un chemin qui parcourt tout les noeuds dispo
            else if(i == 0){
              neighbors_available = false;
            }
          }
        }
        // si le vecteur des noeuds tondu précédement est vide, on arrete le programme
        else{
          neighbors_available = false;
        }
      }
    }
  }
}

/* addNeighbors ajoute, pour chaque cellule, dans le vecteur node_voisin les cellules voisines en connexité 8 qui ont le booléen background a false
*/
void Tree::addNeighbors(){
  vector <int> index;
  index.resize(2);

// Ces booléens permettent de savoir si la cellule actuelle possède un voisin au dessus ou au dessous dans sa colonne, ce qui donne des cas particuliers d'organisation des voisins
  bool edge_haut;
  bool edge_bas;

  for(int i = 0; i < tree.size(); i++){
    for(int j = 0; j < tree[i].size(); j++){
      edge_haut = true;
      edge_bas = true;
      if(tree[i][j].background == 0){ // Si la celllule [i][j] on est dans la zone de tonte
        //BAS
        if(i > 0 && tree[i-1][j].background == 0){ // et que la cellule en-dessous [i-1][j] est aussi dans la zone 
          index[0] = i-1;
          index[1] = j;
          tree[i][j].node_voisin.push_back(index);
          edge_haut = false;
        }


        //HAUT
        if(i < tree.size()-1 && tree[i+1][j].background == 0){
          index[0] = i+1;
          index[1] = j;
          tree[i][j].node_voisin.push_back(index);
          edge_bas = false;
        }

        if(edge_haut == false){  // La cellule actuelle possède un voisin au dessus d'elle
          //DROITE
          if(j< tree[i].size()-1 && tree[i][j+1].background == 0){
            index[0] = i;
            index[1] = j+1;
            tree[i][j].node_voisin.push_back(index);
          }

          //HAUT DROITE
          if(i > 0 && j< tree[i].size()-1 && tree[i-1][j+1].background == 0){
            index[0] = i-1;
            index[1] = j+1;
            tree[i][j].node_voisin.push_back(index);
          }

          //BAS DROITE
          if(i < tree.size()-1 && j < tree[i].size()-1 && tree[i+1][j+1].background == 0){
          index[0] = i+1;
          index[1] = j+1;
          tree[i][j].node_voisin.push_back(index);
          }
        }
        
        if(edge_haut == true){  // La cellule actuelle ne possède pas un voisin au dessus d'elle
          //HAUT DROITE
          if(i > 0 && j< tree[i].size()-1 && tree[i-1][j+1].background == 0){
            index[0] = i-1;
            index[1] = j+1;
            tree[i][j].node_voisin.push_back(index);
          }

          //DROITE
          if(j< tree[i].size()-1 && tree[i][j+1].background == 0){
            index[0] = i;
            index[1] = j+1;
            tree[i][j].node_voisin.push_back(index);
          }

          //BAS DROITE
          if(i < tree.size()-1 && j < tree[i].size()-1 && tree[i+1][j+1].background == 0){
          index[0] = i+1;
          index[1] = j+1;
          tree[i][j].node_voisin.push_back(index);
          }
        }
        
        if(edge_bas == false){  // La cellule actuelle possède un voisin en dessous d'elle
           //GAUCHE
          if(j>0 && tree[i][j-1].background == 0){
            index[0] = i;
            index[1] = j-1;
            tree[i][j].node_voisin.push_back(index);
          }

          //BAS GAUCHE
          if(j>0 && i < tree.size()-1 && tree[i+1][j-1].background == 0){
          index[0] = i+1;
          index[1] = j-1;
          tree[i][j].node_voisin.push_back(index);
          }

          //HAUT GAUCHE
          if(i > 0 && j>0 && tree[i-1][j-1].background == 0){
          index[0] = i-1;
          index[1] = j-1;
          tree[i][j].node_voisin.push_back(index);
          }
        }
        if(edge_bas == true){ // La cellule actuelle ne possède pas de voisin en dessous d'elle
          //BAS GAUCHE
          if(j>0 && i < tree.size()-1 && tree[i+1][j-1].background == 0){
            index[0] = i+1;
            index[1] = j-1;
            tree[i][j].node_voisin.push_back(index);
          }
             //GAUCHE
          if(j>0 && tree[i][j-1].background == 0){
            index[0] = i;
            index[1] = j-1;
            tree[i][j].node_voisin.push_back(index);
          }

          //HAUT GAUCHE
          if(i > 0 && j>0 && tree[i-1][j-1].background == 0){
            index[0] = i-1;
            index[1] = j-1;
            tree[i][j].node_voisin.push_back(index);
          }
        }
      }
    }
  }
}

/* reconstruct_path permet de retourner le chemin le plus court une fois l'algo A_star arrivé a son terme */
vector<vector<int> > Tree::reconstruct_path(vector<vector<vector<int> > > cameFrom, vector<int> goal, int start_x, int start_y){

  vector<int> current;
  current.resize(2);
  current[0] = goal[0];
  current[1] = goal[1];

  vector<int> temp;
  temp.resize(2);
  temp[0] = current[0];
  temp[1] = current[1];

  vector<vector<int> > total_path;
  total_path.push_back(current);
  while(current[0]!=start_x || current[1]!=start_y){
    current[0]=cameFrom[temp[0]][temp[1]][0];
    current[1]=cameFrom[temp[0]][temp[1]][1];
    total_path.push_back(current);

    temp[0] = current[0];
    temp[1] = current[1];
    
  }
  cout << "Chemin le plus court pour aller de [" << start_x << " " << start_y << "] à [" << goal[0] << " " << goal[1] << "] trouvé !" << endl;
  return total_path;
}

/* On effectue l'algo A_star uniquement sur les noeuds avec edge == true pour trouver le chemin le plus court en longeant le contour de la zone a tondre */
vector<vector<int> > Tree::A_star(int start_x, int start_y,int goal_x, int goal_y){

  vector<vector<int> > closedEdge;  // Ce vecteur contient les noeuds deja exploré par l'algo
  vector<vector<int> > openEdge;    // Ce vecteur contient les noeuds voisins actuel qui ne sont pas encore exploré

  vector<int> index;
  index.resize(2);
  index[0]=start_x;
  index[1]=start_y;
  openEdge.push_back(index);

/* Chaque noeud de cameFrom contient l'index (x, y), qui représente l'index du noeud depuis lequel on accede le plus facilement au noeud donné 
   (Un exemple pour mieux comprendre ! Supposons que l'on veut savoir comment aller au noeud d'index (25,9), on regarde donc la matrice cameFrom,
    et on trouve : cameFrom[25][9] = [25 10], ce qui signifie que pour aller en (25,9), il faut passer par le noeud (25,10) pour etre le plus efficace possible)

   De plus, si on trouve plusieurs noeud qui permettent d'aller avec la même efficacité au noeud donné, on garde toujours le premier index trouvé !

   Cette matrice permet donc de retourner du noeud actuel au noeud de départ, en passant d'index en index, et donc de construire le chemin finale
*/  
  vector<vector<vector<int> > > cameFrom; 


/* La matrice gScore contient dans chaque noeud la distance noeud à noeud a parcourir pour aller du point de départ a ce noeud
*/
  vector<vector<float> > gScore;


/* La matrice fScore contient dans chaque noeud la distance noeud à noeud et heuristique pour aller du point de départ au point d'arrivé, en passant par le noeud donné
   En effet, pour aller du noeud de départ au noeud donné, on connait la distance noeud à noeud, et pour aller du noeud donné au noeud d'arrivé, on prend la norme euclidienne (c'est la partie heuristique)
*/
  vector<vector<float> > fScore;


  // Construction et initialisation des différentes matrices utiles
  cameFrom.resize(tree.size());
  gScore.resize(tree.size());
  fScore.resize(tree.size());
  for(int i = 0; i < tree.size();i++){
    cameFrom[i].resize(tree[i].size());
    gScore[i].resize(tree[i].size());
    fScore[i].resize(tree[i].size());
    for(int j = 0; j < tree[i].size(); j++){
      cameFrom[i][j].resize(2);
      gScore[i][j] = INT_MAX;       // A l'initialisation, les valeurs de gScore et fScore sont toutes a l'infini
      fScore[i][j] = INT_MAX;
    }
  }

  gScore[start_x][start_y]=0;       // Pour aller du départ au départ, on a pas besoin de bouger, donc gScore de 0
  
  fScore[start_x][start_y]=sqrt(pow((tree[start_x][start_y].x)-(tree[goal_x][goal_y].x),2)+pow((tree[start_x][start_y].y)-(tree[goal_x][goal_y].y),2));


// Tant qu'on a des noeuds voisin non explorés, on continue
  while (!openEdge.empty()){
    

    // On cherche dans openEdge l'index du noeud voisin non visité qui a la plus petite valeur de fScore 
    index[0]=openEdge[0][0];
    index[1]=openEdge[0][1];
    float fScore_min = fScore[index[0]][index[1]];
    int index_i = 0;
    for(int i=1; i<openEdge.size(); i++){
      index[0]=openEdge[i][0];
      index[1]=openEdge[i][1];
      if(fScore[index[0]][index[1]]<fScore_min){
        
        fScore_min=fScore[index[0]][index[1]];
        index_i = i;
      }
    }
    index[0]=openEdge[index_i][0];
    index[1]=openEdge[index_i][1];



    // Si l'index trouvé est l'index du goal, on execute reconstruct_path, et la fonction se termine
    if(index[0] == goal_x && index[1] == goal_y){

      return reconstruct_path(cameFrom,index,start_x,start_y);
    }

    // On retire de openEdge l'index qui a été choisi, et on l'ajoute a closedEdge
    openEdge.erase(openEdge.begin()+index_i);
    closedEdge.push_back(index);

    // Pour le noeud d'index choisi, on regarde tout ses voisins
    for( int i=0; i< tree[index[0]][index[1]].node_voisin.size(); i++){
      bool evaluated = false;
      bool newNode = true;
      vector<int> voisin;
      voisin.resize(2);
      voisin[0] = tree[index[0]][index[1]].node_voisin[i][0];
      voisin[1] = tree[index[0]][index[1]].node_voisin[i][1];
      // Si le voisin actuel du noeud choisi est un edge
      if(tree[voisin[0]][voisin[1]].edge == true){

        // On regarde si le voisin actuel est dans closedEdge (cad est ce qu'il est deja connu)
        for(int j = 0 ; j < closedEdge.size(); j++){
          if (closedEdge[j][0]==voisin[0] && closedEdge[j][1]==voisin[1]){
            evaluated = true;
            break;
          }
        }

        // Si le voisin actuel n'est pas dans closedEdge, on regarde si il est dans openEdge (cad est ce qu'il est deja en attente d'être exploré)
        if(evaluated == false){
          for(int j=0; j<openEdge.size(); j++){
            if (openEdge[j][0] == voisin[0] && openEdge[j][1] == voisin[1]){
              newNode = false;
              break;
            }
          }

          // Si le voisin actuel n'est pas dans openEdge, on ajoute son index à openEdge
          if(newNode == true){
            openEdge.push_back(voisin);
          }

          // Le gScore potentiel du voisin actuel est egal au gScore du noeud choisi + la norme euclidienne entre le noeud choisi et le voisin actuel
          float tentative_gScore = gScore[index[0]][index[1]]+ sqrt(pow((tree[voisin[0]][voisin[1]].x)-(tree[index[0]][index[1]].x),2)+pow((tree[voisin[0]][voisin[1]].y)-(tree[index[0]][index[1]].y),2));
          bool better_path = true;

          // On compare le gScore potentiel avec le gScore actuel du voisin actuel
          if(tentative_gScore >= gScore[voisin[0]][voisin[1]]){
            better_path = false;
          }
          // Si le gScore potentiel est meilleurs, met a jour les valeurs de cameFrom, gScore et fScore pour le voisin actuel 
          if(better_path == true){
            cameFrom[voisin[0]][voisin[1]][0]=index[0];
            cameFrom[voisin[0]][voisin[1]][1]=index[1];
            gScore[voisin[0]][voisin[1]] = tentative_gScore;
            fScore[voisin[0]][voisin[1]] = gScore[voisin[0]][voisin[1]] + sqrt(pow((tree[voisin[0]][voisin[1]].x)-(tree[goal_x][goal_y].x),2)+pow((tree[voisin[0]][voisin[1]].y)-(tree[goal_x][goal_y].y),2));
          }
        }
      }
    }
  }

  // Si on arrive la, c'est que openEdge est vide, donc on a exploré tout les noeuds edge, sans trouver le noeud objectif. Si on se retrouve ici, c'est qu'il y a un bug, car c'est normalement impossible de ne pas trouver l'objectif !!!
  return vector<vector<int> >();  

}

/* distance_path calcule la norme euclidienne du chemin donné en argument */
float Tree::distance_path(vector<vector<int> > path){
  float sum_distance = 0;
  for(int i = 1; i < path.size(); i++){
    sum_distance += sqrt(pow((tree[path[i-1][0]][path[i-1][1]].x)-(tree[path[i][0]][path[i][1]].x),2)+pow((tree[path[i-1][0]][path[i-1][1]].y) - (tree[path[i][0]][path[i][1]].y),2));
  }

  return sum_distance;
}

/* crossAround permet de trouver le noeud avec edge == true le plus proche de la position actuelle (x_current, y_current) 
   et trouve aussi les noeuds edge les plus proche au dessus et en dessous de l'objectif (goal_x, goal_y) 
   puis on effectue l'algo A_star et on trouve le plus petit chemin possible */ 
vector<vector<int> > Tree::crossAround(int x_current,int y_current,int goal_x,int goal_y){
  vector <int> ind_edge_start;
  ind_edge_start.resize(2);
  vector <int> ind_edge_goal_haut;
  ind_edge_goal_haut.resize(2);
  vector <int> ind_edge_goal_bas;
  ind_edge_goal_bas.resize(2);
  int edge_x;
  int edge_y;
  float edge_pose_x;
  float edge_pose_y;
  float norme;
  float norme_min = 10000;
  geometry_msgs::PoseStamped next_goal;

  // On cherche les edge le plus proche de la position actuelle
  for(int i = 0; i < tree.size(); i++){
    if(tree[i][y_current].edge == true){
      edge_pose_x = tree[i][y_current].x;
      edge_pose_y = tree[i][y_current].y;
      float norme = sqrt((edge_pose_x - tree[x_current][y_current].x)*(edge_pose_x - tree[x_current][y_current].x)+(edge_pose_y - tree[x_current][y_current].y)*(edge_pose_y - tree[x_current][y_current].y));
      if (norme < norme_min){
        norme_min = norme;
        ind_edge_start[0] = i;
        ind_edge_start[1] = y_current;
      }         
    }
  }

  // index du edge le plus proche du start
  edge_x=ind_edge_start[0];
  edge_y=ind_edge_start[1];


  // On cherche le edge superieur le plus proche de l'objectif 
  for(int i = goal_x; i >=0; i--){
    if(tree[i][goal_y].edge == true){
      ind_edge_goal_haut[0] = i;
      ind_edge_goal_haut[1] = goal_y;
      break;
    }
  }

  // On cherche le edge inferieur le plus proche de l'objectif 
  for(int i = goal_x; i < tree.size(); i++){
    if(tree[i][goal_y].edge == true){            
      ind_edge_goal_bas[0] = i;
      ind_edge_goal_bas[1] = goal_y;
      break;
    }
  }

  vector<vector<int> > path_haut;
  vector<vector<int> > path_bas;


  path_haut = A_star(edge_x,edge_y,ind_edge_goal_haut[0],ind_edge_goal_haut[1]);
  float distance_haut = distance_path(path_haut);


  /* EXPLICATION SUR LE IF CI DESSOUS :
     On regarde si le noeud edge haut est différent du noeud edge bas
     En effet, il est possible que le noeud (goal_x, goal_y) soit lui meme un edge
     Dans ce cas, les boucles de recherche des edges superieur et inférieur vont s'arreter a l'initialisation car (goal_x, goal_y) est un edge
     On aura alors pas besoin d'effectuer l'algo A_star pour le edge inférieur, car on va obtenir le même résultat que pour le edge supérieur 
  */
  if(ind_edge_goal_haut[0] != ind_edge_goal_bas[0]){
    path_bas = A_star(edge_x,edge_y,ind_edge_goal_bas[0],ind_edge_goal_bas[1]);
    float distance_bas = distance_path(path_bas);

    // Cependant, si les edges supérieur et inférieur sont différents, on doit alors prendre le chemin le plus court entre les 2
    if(distance_haut > distance_bas){
      cout << "CHEMIN LE PLUS RAPIDE PAR LE BAS" << endl;
      return path_bas;
    }
  }
  cout << "CHEMIN LE PLUS RAPIDE PAR LE HAUT" << endl;
  return path_haut;
}


/* addNode construit la matrice de taille lignes x colonnes et initialise les booléens de chaque cellule aux valeurs par défaut */
void Tree::addNode(int L, int l, float x_min, float y_min){
  
  for(int i = 0;i<L;i++){
    vector<Node> branch;
    for(int j = 0;j<l;j++){
      Tree::Node newNode;
      newNode.x = x_min + (edge_inflation + i)*ratio*diametre_tondeuse;
      newNode.y = y_min + (edge_inflation + j)*ratio*diametre_tondeuse;
      newNode.occupied = false;
      newNode.background = true;
      newNode.visited = false;
      newNode.edge = false;
      newNode.mowed = false;
      branch.push_back(newNode);
    }
    tree.push_back(branch);
  }
}

void Tree::printTree(){
  cout << endl;
  for(int i = 0; i<tree.size(); i++){
    for(int j = 0; j<tree[i].size();j++){
      cout << "[" << tree[i][j].background << "] " ; 
    }
    cout << endl;
  }
  cout << endl;
}

/*  nodeNearEdge calcule la distance eucliendienne entre une feuille de l'arbre, et les points du edge/contour.
    Si une feuille est suffisament proche d'un noeud, alors on lui attribut false au booléen background, et true au booléen edge
*/
void Tree::nodeNearEdge(nav_msgs::Path edge){
  float edge_x;
  float field_x;
  float edge_y;
  float field_y;
  cout << " ENTER NEAR EDGE PROGRAM" << endl;
  for(int i = 0; i < tree.size(); i++){
    for(int j = 0; j < tree[i].size(); j++){
      for(int k = 0; k < edge.poses.size(); k++){
        field_x = tree[i][j].x;
        edge_x = edge.poses[k].pose.position.x;
        field_y = tree[i][j].y;
        edge_y = edge.poses[k].pose.position.y;
        float norme = sqrt((field_x - edge_x)*(field_x-edge_x)+(field_y-edge_y)*(field_y-edge_y));
        if( norme < 0.75*diametre_tondeuse){
          tree[i][j].background = false;
          tree[i][j].edge = true;
        }
      }
    }
  }
}


/*  nodeInArea verifie que une feuille de l'arbre se trouve entre 4 feuilles non occupés (cad avec false en valeur)
    si les 4 conditions sont verifiees, alors la feuille est libre, et on lui donne la valeur false
*/
void Tree::nodeInArea(){
  cout << " ENTER AREA PROGRAM " << endl;
  for(int i = 0; i < tree.size(); i++){
    for(int j = 0; j < tree[i].size(); j++){
      bool left_ok = false;
      bool right_ok = false;
      bool up_ok = false;
      bool down_ok = false;
     
      for(int k = i; k>=0; k--){
        if(tree[k][j].background == false ){
            left_ok = true;
            break;
          }
        }
      for(int l = i; l<tree.size(); l++){
        if(tree[l][j].background == false ){
          right_ok = true;
          break;
        }
      }
      for(int m = j; m>=0; m--){
        if(tree[i][m].background == false ){
          down_ok = true;
          break;
        }
      }
      for(int n = j; n<tree[i].size(); n++){
        if(tree[i][n].background == false ){
          up_ok = true;
          break;
        }
      }
      if(left_ok == true && right_ok == true && up_ok == true && down_ok == true){
        tree[i][j].background = false;
      }
      else{
      }
    }
  }
}


// Permet de réinitialiser la grille pour le nouveau calcul de la planification de trajectoire dû à la détection d'un nouvel obstacle.
void Tree::reinitPathPlanning(){
  for(int i = 0; i < tree.size(); i++){
    for(int j = 0; j < tree[i].size(); j++){
      tree[i][j].visited = false;
      tree[i][j].occupied = false;
      //tree[i][j].node_voisin.clear();
    }
  }
  past_goal.clear();
}


// Met à jour l'information "tondue" ou "pas tondue" de la cellule (information x et y recupérés du message path_planned/mowed_cell)
void Tree::getMowedCell(float x, float y){
  for(int i = 0; i < tree.size(); i++){
    bool flag = false;
    for(int j = 0; j < tree[i].size(); j++){
      if ((tree[i][j].x == x) && (tree[i][j].y == y)){
        tree[i][j].mowed = true;
        actualiser_z(i, j, -10000);
        flag = true;

        vector<int> index;
        index.resize(2);

        index[0] = i;
        index[1] = j;

        past_mowed.push_back(index);
        break;
      }
    }
    if (flag)
      break;
  }
}

// Fonction qui permet de gérer le visuel du point cloud (suivant z) pour faire apparaitre ou disparaitre les zones tondues ou non
void Tree::actualiser_z(int index_i, int index_j, float z){
  for (int l = 0; l < goal_field_msgs.points.size(); l++){
    if ((tree[index_i][index_j].x == goal_field_msgs.points[l].x) && (tree[index_i][index_j].y == goal_field_msgs.points[l].y)){
      goal_field_msgs.points[l].z = z;
    }
  }
}
