#include "ibex.h"
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include "rrt_lib/Point3.hpp"
#include "CSTL.h"

using namespace ibex;


#define __PREC__ 1e-7
#define __METH__ HEUN
#define __DURATION__ 0.1

const int X_orig = 0;
const int Y_orig = 0;
const double Z_orig = 0;

int path_index = -1;

double ytrajet(double t) {
    return (1 - std::exp(-1*t))*(1 - std::exp(-1*t));
}
class Box {
public:
    double x; // Center of the upper face x-coordinate
    double y; // Center of the upper face y-coordinate
    double z; // Center of the upper face z-coordinate
    double width; // Width of the box
    double height; // Height of the box
    double ratio;
    Box(double x, double y, double z, double width, double height, double ratio)
        : x(x), y(y), z(z), width(width), height(height), ratio(ratio) {}

    IntervalVector toIntervalVector() const {
        IntervalVector iv(6);
        iv[0] = Interval(x - width / 2, x + width / 2); // x interval
        iv[1] = Interval(y - ratio * width / 2, y + ratio*width / 2); // y interval
        iv[2] = Interval(z - height, z);                // z interval (assuming z is the upper face)
        iv[3] = Interval(-1e8, 1e8); // x interval
        iv[4] = Interval(-1e8, 1e8); // y interval
        iv[5] = Interval(-1e8, 1e8);
        return iv;
    }
};

std::vector<std::vector<double>> read_file(const std::string& file_path) {
    std::vector<std::vector<double>> data;
    std::ifstream file(file_path);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (ss >> value) {
            row.push_back(std::stod(value));
        }

        if (!row.empty()) {
            data.push_back(row);
        }
    }
    return data;
}

double get_value(const std::vector<std::vector<double>>& data, int i, int j) {
    if (i < data.size() && j < data[i].size()) {
        return data[i][j];
    } else {
        std::cerr << "Indices out of range" << std::endl;
        return -1; // Or handle the error appropriately
    }
}

void writeObstaclesToFile(const std::list<Box>& obstacles_boxes, const std::string& filename) {
    std::ofstream obstacles(filename);
    for (const auto& box : obstacles_boxes) {
        IntervalVector iv = box.toIntervalVector();
        obstacles << iv[0] << " ; " << iv[1] << " ; " << iv[2] << std::endl;
    }
    obstacles.close();
}


geometry_msgs::PoseStamped coord;

void callBackPosition(const geometry_msgs::PoseStamped msg) {
    coord.pose.position.x = msg.pose.position.x;
    coord.pose.position.y = msg.pose.position.y;
    coord.pose.position.z = msg.pose.position.z;
    //ROS_INFO("Current position: (%g, %g, %g)", coord.x, coord.y, coord.z);
}
void pathIndexCallback(const std_msgs::Int32::ConstPtr& msg) {
    path_index = msg->data;
    //ROS_INFO("Updated path_index: %d", path_index);
}

int main(int argc, char ** argv){

//Initializing ROS variables correctly
ros::init(argc, argv, "MoniteurMP");
ros::NodeHandle nh1;

ros::NodeHandle nh_pos;

std_msgs::Empty msg1;
std_msgs::Empty msg2;

// Subscriber for /path_index topic
ros::Subscriber path_index_sub = nh1.subscribe("/path_index", 100, pathIndexCallback);


ros::Subscriber pos_sub;
pos_sub = nh_pos.subscribe<geometry_msgs::PoseStamped>("/mocap_node/Robot_1/pose",1000, callBackPosition);

ros::Rate loop_rate(95);
  // disturbance bounds
  //const int k[] = {0,1,0,3,0,1,0,3,0,4,0,6,0,7,0,7,0,0,0}; ///tableau de motion primitives
  //const int k[] = {0,1,3,1,3,4,6,7,7,0,0,0,0};
  const int k[] = {0,0,3,2,3,9,0,0,0,0,0};
      int N_node = sizeof(k) / sizeof(k[0]) - 4;;
  int Nto = 3; //horrizon glissant
  const int Nto_init = Nto;
  int Xway, Yway, Zway;
  // first row is an initial value
  const int n = 6;
  //double to = 3;
  int to = 3;
  /*int Xway = 1;
  int Yway = 0;
  int Zway = 0; */

  double Kpref = 0.8;
  double Ka = 1.4;
  bool collision = false;
  bool stayEnv = true;
  BoolInterval collisionItv;
  BoolInterval Monitor_itv;
bool inclu_obs = false;
  IntervalVector Enviro(6);
  Enviro[0] = Interval(-1, 3);    // x interval
  Enviro[1] = Interval(-0.5, 3.5); // y interval
  Enviro[2] = Interval(-0.5, 2);      // z interval

  Enviro[3] = Interval(-1e8, 1e8); // x interval
  Enviro[4] = Interval(-1e8, 1e8); // y interval
  Enviro[5] = Interval(-1e8, 1e8);

  /*int nobs = 1;
  IntervalVector Obstacles(3);
  Obstacles[0] = Interval(-0.5, 0.5);
  Obstacles[1] = Interval(0.5,1);
  Obstacles[2] = Interval(0, 1);
X_orig
    ofstream obstacles;
    obstacles.open("obstacles.txt");
  for (int a = 0; a<nobs; a++){
    obstacles<< Obstacles[0] <<" ; " <<Obstacles[1] <<" ; " <<Obstacles[2] << std::endl;
  }
  obstacles.close(); 
*/
 std::list<Box> obstacles_boxes;
    obstacles_boxes.push_back(Box(0.5 + X_orig, 0.65 + Y_orig, 0.7, 0.35, 1.2, 2.5));
    obstacles_boxes.push_back(Box(0.0 + X_orig, 2.8 + Y_orig,1, 0.5, 1.5, 2));
    obstacles_boxes.push_back(Box(1.25 + X_orig, 1.25 + Y_orig, 1.2, 0.7, 1.7, 0.5));
    //obstacles_boxes.push_back(Box(8 + X_orig, -5 + Y_orig, 2.5, 1.0, 2.50));
    //obstacles_boxes.push_back(Box(6.2 + X_orig, -6.5 + Y_orig, 2.2, 2.0, 2.20));
    //obstacles_boxes.push_back(Box(3.5 + X_orig, -5 + Y_orig, 2.5, 1.0, 2.50));
    // à définir avec l'optitrack

    writeObstaclesToFile(obstacles_boxes, "obstacles.txt");

    // Convert boxes to IntervalVectors
    std::list<IntervalVector> obstacles_list;
    for (const auto& box : obstacles_boxes) {
        obstacles_list.push_back(box.toIntervalVector());
    }
	
	std::vector<IntervalVector> predicate_list;
    for (const auto& box : obstacles_boxes) {
        predicate_list.push_back(box.toIntervalVector());
    }
	
  IntervalVector yinit(n);
  IntervalVector Box(n); ///visualisation des trajectoires
  IntervalVector tmp(n);
  Variable y(n);
  
  yinit[0] = Interval(-0.01, 0.01)+ Interval(X_orig); // erreur position
  yinit[1] = Interval(-0.01, 0.01)+ Interval(Y_orig);
  yinit[2] = Interval(Z_orig) + Interval(-0.01, 0.01);
  yinit[3] = Interval(X_orig); ///modifier pour revenir à 0 pour l'origine
  yinit[4] = Interval(Y_orig);
  yinit[5] = Interval(Z_orig);

  // modèle dynamique en interval, Ka*(Xref - X)
  Interval Ax = Interval(1) + Interval(-0.12, 0.10); // coeff commande en vitesse
  Interval Bx = Interval(-0.15, 0.15);
  Interval Az = Interval(1) + Interval(-0.05, 0.05); // coeff commande en vitesse
  Interval Bz = Interval(-0.15, 0.15);

    AF_fAFFullI::setAffineNoiseNumber(20);
  Interval Xobj(0.0); 
  Interval Yobj(0.0);
  Interval Zobj(0.0);
  
    ofstream trajectory_boxes;
    trajectory_boxes.open("/home/antoinebc/catkin2_ws/src/tello_driver/src/trajectory_boxes.txt");

    ofstream monitor_flag;
    monitor_flag.open("monitor_flag.txt");


    Affine2Vector yinit_aff(yinit, true);



    double PosX = 0;
    double PosY = 0;
    double PosZ = 0;

    double PosXMP = 0;
    double PosYMP = 0;
    double PosZMP = 0;

    double VX = 0;
    double VY = 0;
    double VZ = 0;

    int tmp_index = 0; //record the path index

	const int length = sizeof(k) / sizeof(k[0]);

	    // Tableau pour stocker les positions (X, Y)
	std::cout<<length<<std::endl;
	double trajectory[length][3];

	    // Initialisation du point de départ (X = 0, Y = 0)
	    trajectory[0][0] = 0;  // X initial
	    trajectory[0][1] = 0;  // Y initial
		trajectory[0][2] = 0;
	    // Boucle pour calculer les positions successives
	    for (int i = 1; i < length; ++i) {
			  switch (k[i-1]) {
			            case 1: Xway = 1; Yway = 0; Zway = 0; break;
			            case 2: Xway = 1; Yway = 1; Zway = 0; break;
			            case 3: Xway = 0; Yway = 1; Zway = 0; break;
			            case 4: Xway = -1; Yway = 1; Zway = 0; break;
			            case 5: Xway = -1; Yway = 0; Zway = 0; break;
			            case 6: Xway = -1; Yway = -1; Zway = 0; break;
			            case 7: Xway = 0; Yway = -1; Zway = 0; break;
			            case 8: Xway = 1; Yway = -1; Zway = 0; break;
			            case 9: Xway = 0; Yway = 0; Zway = 1; break;
			            case 10: Xway = 0; Yway = 0; Zway = -1; break;
			            case 0: Xway = 0; Yway = 0; Zway = 0; break;
			            default: Xway = 0; Yway = 0; Zway = 0; break; // décollage
			        }  

	        // Calcul des nouvelles positions X et Y
	        double t = 3;  // Vous pouvez ajuster cette valeur
	        trajectory[i][0] = trajectory[i-1][0] + Xway * ytrajet(t); // Nouvelle valeur de X
	        trajectory[i][1] = trajectory[i-1][1] + Yway * ytrajet(t); // Nouvelle valeur de Y
	        trajectory[i][3] = trajectory[i-1][3] + Zway * ytrajet(t); // Nouvelle valeur de Z
	    }

	    while(coord.pose.position.x == 0){
	        ros::spinOnce();
	        loop_rate.sleep();
	    }
	    Point3 initial_position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z +0.3);

    while (ros::ok() && path_index == -1) {
        ros::spinOnce();  // Process the callback functions
        loop_rate.sleep();  // Maintain the loop rate
    }
    
std::cout << "Starting Monitor Node"<<std::endl;
while (ros::ok() && path_index < N_node)
{
	PosX = coord.pose.position.x-initial_position.x();
	PosY = coord.pose.position.y-initial_position.y();
	PosZ = coord.pose.position.z-initial_position.z();

	PosXMP = trajectory[path_index][0];
	PosYMP = trajectory[path_index][1];
	PosZMP = trajectory[path_index][2];


	if (tmp_index!= path_index) {
	  yinit[0] = Interval(-0.05, 0.05)+ Interval(PosX); // erreur position
	  yinit[1] = Interval(-0.05, 0.05)+ Interval(PosY);
	  yinit[2] = Interval(-0.02, 0.02) + Interval(PosZ);
	  yinit[3] = Interval(PosXMP);
	  yinit[4] = Interval(PosYMP);
	  yinit[5] = Interval(PosZMP);
	  std::cout << "-----------------Starting Interval Analysis----------------------"<<std::endl;
ROS_INFO("MP init: %f, %f", PosXMP, PosYMP);
ROS_INFO("POS init: %f, %f", PosX, PosY);
ROS_INFO("Updated path_index: %d", path_index);
			for (int i=0; i<Nto; i++){
				std::cout<<"Number of simulation:"<<i<<std::endl;
			  switch (k[path_index + i]) {
						case 1: Xway = 1; Yway = 0; Zway = 0; break;
						case 2: Xway = 1; Yway = 1; Zway = 0; break;
						case 3: Xway = 0; Yway = 1; Zway = 0; break;
						case 4: Xway = -1; Yway = 1; Zway = 0; break;
						case 5: Xway = -1; Yway = 0; Zway = 0; break;
						case 6: Xway = -1; Yway = -1; Zway = 0; break;
						case 7: Xway = 0; Yway = -1; Zway = 0; break;
						case 8: Xway = 1; Yway = -1; Zway = 0; break;
						case 9: Xway = 0; Yway = 0; Zway = 1; break;
			            case 10: Xway = 0; Yway = 0; Zway = -1; break;
						case 0: Xway = 0; Yway = 0; Zway = 0; break;
						default: Xway = 0; Yway = 0; Zway = 0; break; // décollage
					}

			  Xobj = Interval(1*Xway) + yinit[3]; // point de commande
			  Yobj = Interval(1*Yway) + yinit[4];
			  Zobj = Interval(1*Zway) + yinit[5];
			  std::cout<<"Obj waypoint:"<< Xobj<< Yobj<<std::endl;
			  //std::cout<< yinit[3] << " MP|init "<< yinit[4] << std::endl;
			  Function ydot = Function(y, Return(
				//Ax*(Kpref*(Xway) + y[2]*(-Kpref)) + Bx,
				//Ax*(Kpref*(Yway) + y[3]*(-Kpref)) + Bx,
				Ax*(Ka*(-y[0]) + Kpref*(Xobj) + y[3]*(Ka-Kpref)) + Bx,
				Ax*(Ka*(-y[1]) + Kpref*(Yobj) + y[4]*(Ka-Kpref)) + Bx,
				Az*(Ka*(-y[2]) + Kpref*(Zobj) + y[5]*(Ka-Kpref)) + Bz,
				Kpref*(Xobj - y[3]),
				Kpref*(Yobj - y[4]),
				Kpref*(Zobj - y[5])
			  ));

			  ivp_ode problem = ivp_ode(ydot, 0, yinit, SYMBOLIC);
			  simulation simu = simulation(&problem, to, HEUN, 1e-3, 0.01);

			  simu.run_simulation();
			  yinit = simu.get_last_aff();
			//yinit = simu.get_last();
			  //std::cout<< Xobj << " | "<< Yobj << std::endl;
			 // tmp = simu.get_last();

			vector<Satisf_Signal> Phi1; //declaration of subformulas satisfaction signals
			vector<Satisf_Signal> Phi2; //declaration of subformulas satisfaction signals
			std::vector<vector<Satisf_Signal>> P_Satisfaction_signals = predicate_satisfaction(simu, p_list); //evaluation on the simulated tube

	        V_p=P_Satisfaction_signals[0]; //assigning every predicate to its satisfaction signals
	        W_p=P_Satisfaction_signals[1];
	        P_p=P_Satisfaction_signals[2];
	        Q_p=P_Satisfaction_signals[3];
				
					for (const auto& obstacles_element : obstacles_list) {

						
						collisionItv = simu.has_crossed_b(obstacles_element);
						
						if (collisionItv == MAYBE || collisionItv == YES){ break;}
					}
			  if (simu.one_in(&obstacles_list) != -1){inclu_obs = true;}
			//std::cout<<simu.one_in(&obstacles_list)<<"inclusion time"<<std::endl;
			 if(not simu.stayed_in(Enviro)){stayEnv = false;}

			 if(not stayEnv){collisionItv = YES;}
			 if (inclu_obs){collisionItv = YES;}

			  for (int k = 0; k<10*to; k++){
			  Box = simu.get(0.1*k);
				trajectory_boxes<< Box[0] <<" ; " <<Box[1] <<" ; " <<Box[2] << std::endl;
			  }

			  }
			//std::cout <<"-------- || collision : " <<collisionItv<<"--stay in env :"<< stayEnv  << std::endl;

			switch (collisionItv) {
					   case NO: Monitor_itv = YES; break;
					   case MAYBE: Monitor_itv = MAYBE; break;
					   case YES: Monitor_itv = NO; break;
					   default: Monitor_itv = NO; break; // décollage
				   }
			std::cout <<"Monitor Go_flag: " << Monitor_itv <<std::endl;
			monitor_flag <<"Monitor Go_flag: " << Monitor_itv <<std::endl;
			stayEnv = true;
			inclu_obs = false;
			//collision = false;
			//trajectory_boxes<<"------------------------------------------------------------------"<< std::endl;
			//trajectory_boxes.close();
}
	tmp_index = path_index;
    ros::spinOnce();
    loop_rate.sleep();
}

////complete pour l'affichage

  trajectory_boxes.close();
  monitor_flag.close();
  return 0;
}
