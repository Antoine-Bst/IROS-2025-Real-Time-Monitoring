#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include "rrt_lib/Point3.hpp"
#include "rrt_lib/Segment.hpp"
#include "rrt_lib/Env.hpp"
#include "rrt_lib/command.hpp"
#include "rrt_lib/Node.hpp"
#include "rrt_lib/RRT_STAR.hpp"
#include "rrt_lib/RRT.hpp"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped coord;
geometry_msgs::PoseStamped coord_cible;

void callBackPosition(const geometry_msgs::PoseStamped msg) {
    coord.pose.position.x = msg.pose.position.x;
    coord.pose.position.y = msg.pose.position.y;
    coord.pose.position.z = msg.pose.position.z;
    //ROS_INFO("Current position: (%g, %g, %g)", coord.x, coord.y, coord.z);
}

void callBackCible(const geometry_msgs::PoseStamped msg) {
    coord_cible.pose.position.x = msg.pose.position.x;
    coord_cible.pose.position.y = msg.pose.position.y;
    coord_cible.pose.position.z = msg.pose.position.z;
    //ROS_INFO("Current position: (%g, %g, %g)", coord.x, coord.y, coord.z);
}

double ytrajet(double t) {
    return (1 - std::exp(-1*t))*(1 - std::exp(-1*t));
}

double ucommande(double t) {
    return std::exp(-1*t);
}

//const int k[] = {0,1,0,3,0,1,0,3,0,4,0,6,0,7,0,7,0};
//const int k[] = {0,1,3,1,3,4,6,7,7,0};
const int k[] =   {0,0,3,2,3,9,0,0};
// mettre dans le tableau un 0 de tel sorte que la commande soit nulle
const int MPlen = sizeof(k) / sizeof(k[0]) - 1;
int Xway, Yway, Zway;

double Uxref = 0;
double Uyref = 0;
double Uzref = 0;

bool landing = false;
//int path_index = 0;
double mptime=0;



int main(int argc, char ** argv){
	double Xref =0;
	double Yref = 0;
	double Zref = 0.3;
    int path_index = 0;
	ofstream trajectory_log;
	trajectory_log.open("/home/antoinebc/catkin2_ws/src/tello_driver/src/trajectory_log.txt");
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
	        trajectory[i][2] = trajectory[i-1][2] + Zway * ytrajet(t); // Nouvelle valeur de Z
	    }
	    std::cout << "Tableau des trajectoires (X, Y):" << std::endl;
	        for (int i = 0; i < length; ++i) {
	            std::cout << "Point " << i << ": X = " << trajectory[i][0] << ", Y = " << trajectory[i][1]<< ", Z = " << trajectory[i][2] << std::endl;
	        }

//Initializing ROS variables correctly
	ros::init(argc, argv, "asservissement_PI");
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::NodeHandle nh_move;
    ros::NodeHandle nh_pos;
    ros::NodeHandle nh_cible;

    std_msgs::Int32 path_msg;
    std_msgs::Empty msg1;
    std_msgs::Empty msg2;

    geometry_msgs::Twist cmd;

    ros::Publisher path_index_pub = nh1.advertise<std_msgs::Int32>("/path_index", 100);

    ros::Publisher take_off_pub;
    take_off_pub = nh1.advertise<std_msgs::Empty>("/tello/takeoff", 100);

    ros::Publisher move_pub ;
    move_pub = nh_move.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1000);

    ros::Publisher land_pub;
    land_pub = nh2.advertise<std_msgs::Empty>("/tello/land", 100);

    ros::Subscriber pos_sub;
    pos_sub = nh_pos.subscribe<geometry_msgs::PoseStamped>("/mocap_node/Robot_1/pose",1000, callBackPosition);

    ros::Subscriber cible_sub;
    cible_sub = nh_cible.subscribe<geometry_msgs::PoseStamped>("/mocap_node/Cible/pose",1000, callBackCible);

    ros::Rate loop_rate(95);


//Initialization of the various positions by optitrack (call the Callback functions )
    while(coord.pose.position.x == 0){
        ros::spinOnce();
        loop_rate.sleep();
    }
    Point3 initial_position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z + 0.3);

    double Kp = 1;                       //Coefficient proportionnel du PID
    double saturation = 0.8;               //Valeur maximale de la commande dans une direction


//COMMAND
        Point3 position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
        sleep(5);
        ROS_INFO("Taking off...");
        take_off_pub.publish(msg1); //message décollage
        sleep(3);
        ros::Time begin = ros::Time::now();

        while (ros::ok() && not landing) {
        	path_msg.data = path_index;
        	path_index_pub.publish(path_msg);
            position = Point3(coord.pose.position.x, coord.pose.position.y, coord.pose.position.z);
            double secs =ros::Time::now().toSec() - begin.toSec();
            mptime = fmod(secs, 3); //Motion primitive de 2s
            path_index = (int) secs/3;
            //std::cout<<path_index<<std::endl;
            if(path_index > MPlen){
            	path_index = MPlen;
            	landing = true;
            }
            //ROS_INFO("index: %d", path_index);
			  switch (k[path_index]) {
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
			    //définir Y(t) //cree les noeuds artificielle (condition initiale pour Y(t%2)
			    //définir U(t)
			    ///ROS_INFO("cmdX: %d", Xway);
			    Uxref = Xway * ucommande(mptime); //selecteur de mode ci dessus
			    Uyref = Yway * ucommande(mptime);
			    Uzref = Zway * ucommande(mptime);

			    Xref = trajectory[path_index][0]+ Xway*ytrajet(mptime);
			    Yref = trajectory[path_index][1]+ Yway*ytrajet(mptime);
			    Zref = trajectory[path_index][2]+ Zway*ytrajet(mptime);
			    //ROS_INFO("Pos_Xref: %f", Xref);
			    //ROS_INFO("Pos_Yref: %f", Yref);
                    //double cmdX = Kp * (Xref - position.x() + initial_position.x()) + Uxref;
                    //double cmdY = Kp * (Yref - position.y() + initial_position.y()) + Uyref;
                    //double cmdZ = Kp * (Zref - position.z() + initial_position.z());
			    	double relative_X = position.x()-initial_position.x();
			    	double relative_Y = position.y()-initial_position.y();
			    	double relative_Z = position.z()-initial_position.z();
			    	double error_X = Xref - relative_X;
			    	double error_Y = Yref - relative_Y;
			    	double error_Z = Zref - relative_Z;
                    //ROS_INFO("err_X: %f", error_X);
                    //ROS_INFO("err_Y: %f", error_Y);
                    //ROS_INFO("err_Z: %f", error_Z);
                    double cmdX = 1*Uxref + 1.4*error_X;
                    double cmdY = 1*Uyref + 1.4*error_Y;
                    double cmdZ = 0.5*Uzref + 2.5*error_Z;
                    
                    //double cmdZ = 3*(error_Z);
                    //double cmdX = 1.1*Uxref;
                    //double cmdY = 1.1*Uyref;
                    //double cmdZ = Kp * 0;
                    cmd.linear.x = cmdX;
                    cmd.linear.y = cmdY;
                    cmd.linear.z = cmdZ;
                    //ROS_INFO("cmdX: %f", cmdX);
                    //ROS_INFO("cmdY: %f", cmdY);
                    move_pub.publish(cmd);
                    ros::spinOnce();
                    loop_rate.sleep();
                    trajectory_log<<relative_X <<"	" <<relative_Y<<"	" <<relative_Z<< "	"<< Xref<< "	" << Yref<< "	"<< Zref<< "	" << cmdX << "	" << cmdY<< "	" << cmdZ << "	" << path_index <<std::endl;
                }
        sleep(1);
        trajectory_log.close();
                cmd.linear.x = 0;
                cmd.linear.y = 0;
                cmd.linear.z = 0;
                move_pub.publish(cmd);  //publier la commande
                land_pub.publish(msg2); //message atterissage
               /// take_off_pub.publish(msg2); //message décollage
                ROS_INFO("Landed");

    return 0;
}
