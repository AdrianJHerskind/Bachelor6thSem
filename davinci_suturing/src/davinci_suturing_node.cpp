#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <time.h> 
#include "std_msgs/String.h"
#include <math.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <armadillo>
#include <curses.h>

#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include "davinci_suturing/jointValues.h"

using namespace arma;

#define pi 3.141592653589793
#define N_samples 3000 //2000*0.01 = 20s

/*** Function definitions ***/
//KDL::JntArray davinci_inverse_kinematics(KDL::Frame eeFrame);



double joint6 = 0.0;
double joint6Factor = 1.0;

double joint5 = 0.0;
double joint5Factor = 1.0;

double joint4 = 0.0;
double joint4Factor = 1.0;

double joint3 = 0.0;
double joint3Factor = 1.0;

/*KDL::JntArray davinci_inverse_kinematics(KDL::Frame eeFrame){
	double xEE;
	double yEE;
	double zEE;
	double q1;
	double q2;
	double q3;
	xEE = eeFrame.p.data[0];
	yEE = eeFrame.p.data[1];
	zEE = eeFrame.p.data[2];
	//std::cout << eeFrame.p.data[1] <<std::endl;
	
	q3 = sqrt (pow(xEE, 2)+pow(yEE, 2)+pow(zEE, 2));
	if(q3==0){// Generalized coordinates q1 and q3 can be chosen arbitrarily
		q1 = 0;
		q2 = 0;
	}else if(yEE==0 && zEE==0){
		q1 = 0;// Coordinate q1 can be chosen arbitrarily 
		if(xEE>0){
			q2 = pi/2;
		}else{
			q2 = -pi/2;
		}	
	}else{
		q2 = (2*pi-acos(xEE/q3))+pi/2;
		q1 = atan2(-yEE,zEE);
	}
	KDL::JntArray qDavinci(3);
	qDavinci(0) = q1;
	qDavinci(1) = q2;
	qDavinci(2) = q3;
	
	return(qDavinci);
}*/

/*** -------------------------------------------------------------------------------- ***/
/*** -------------------------------------------------------------------------------- ***/
/*** -------------------------------------------------------------------------------- ***/
/*** -------------------------------------------------------------------------------- ***/
/*** -------------------------------------------------------------------------------- ***/
/*** -------------------------------------------------------------------------------- ***/




int main(int argc, char *argv[])
{
  ros::init(argc, argv, "davinci_suturing_node");
  ros::NodeHandle node;
  KDL::Tree my_tree;
     std::string robot_desc_string;
     node.param("robot_description", robot_desc_string, std::string());
     if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
     {
        ROS_ERROR("Failed to construct kdl tree");
        return -1;
     }

     KDL::Chain my_chain;
     std::string root_link("p4_rcm_base");
     std::string tip_link("needle_driver_jawbone_right");
     if (!my_tree.getChain(root_link, tip_link, my_chain))
     {
         ROS_ERROR("Failed to get chain from tree");
         return -1;
     }

     for (unsigned int i = 0; i < my_chain.getNrOfSegments(); ++i)
     {
         std::cout << my_chain.getSegment(i).getName() << "(" << my_chain.getSegment(i).getJoint().getName() << ")" << std::endl;
     }

     //Create solver based on kinematic chain
     KDL::ChainFkSolverPos_recursive fksolver(my_chain);
     KDL::ChainIkSolverVel_pinv iksolverv(my_chain);
     KDL::ChainIkSolverPos_NR iksolver = KDL::ChainIkSolverPos_NR(my_chain,fksolver,iksolverv,100,1e-6);
KDL::Frame cartpos;
     KDL::JntArray q(my_chain.getNrOfJoints());
     KDL::JntArray q_init(my_chain.getNrOfJoints());

  KDL::JntArray jointpositions(my_chain.getNrOfJoints());
	
// Get joints names
	std::vector<std::string> joints;
	node.param("/davinci/p4_hand_controller/joints", joints, std::vector<std::string>());
	//ros::NodeHandle node;

	ros::Rate r(100);

 
	
while(ros::ok())
{  	
	ros::spinOnce();

	int axesUsed;
        if(node.getParam("/axesUsed", axesUsed))
{
	double startArray[axesUsed];
	node.getParam("/startArray", startArray[axesUsed]);

	int viaPointSize;

	node.getParam("/viaPointSize", viaPointSize);
	cout << "Via Point Size " << viaPointSize;
	double viaPointArray[viaPointSize][3][axesUsed];
	node.getParam("/viaPointArray", viaPointArray[viaPointSize][3][axesUsed]);
}


	if(joint6 > 1.075)
	{
		joint6Factor = -1.0;
	}
	else if(joint6 < -1.7)
	{
		joint6Factor = 1.0;
	}
	joint6 = joint6 + (joint6Factor*0.001);

	if(joint5 > 1.07)
	{
		joint5Factor = -1.0;
	}
	else if(joint5 < -0.5)
	{
		joint5Factor = 1.0;
	}
	joint5 = joint5 + (joint5Factor*0.001);

	if(joint4 > 0.041)
	{
		joint4Factor = -1.0;
	}
	else if(joint4 < -0.166)
	{
		joint4Factor = 1.0;
	}
	joint4 = joint4 + (joint4Factor*0.0001);

	if(joint3 > 0.099)
	{
		joint3Factor = -1.0;
	}
	else if(joint3 < -0.099)
	{
		joint3Factor = 1.0;
	}
	joint3 = joint3 + (joint3Factor*0.0001);

	if(joint6 > 1.075)
	{
		joint6 = 1.075;
	}
	else if(joint6 < -1.7)
	{
		joint6 = -1.7;
	}

	if(joint5 > 1.07)
	{
		joint5 = 1.07;
	}
	else if(joint5 < -0.5)
	{
		joint5 = -0.5;
	}

	if(joint4 > 0.041)
	{
		joint4 = 0.041;
	}
	else if(joint4 < -0.166)
	{
		joint4 = -0.166;
	}

jointpositions(0) = joint6;
jointpositions(1) = joint5;
//jointpositions(2) = joint5;
//jointpositions(3) = -joint5;
jointpositions(4) = joint4;
jointpositions(5) = 0;
jointpositions(6) = 0;
jointpositions(7) = 0;

bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);

  if(kinematics_status>=0){
        //std::cout << cartpos <<std::endl;
      //  printf("%s \n","Succes, thanks KDL!");
    }
  else{
       // printf("%s \n","Error: could not calculate forward kinematics :(");
    }
std::cout << "Joint6 " << joint6 << std::endl;
std::cout << "Joint5 " << joint5 << std::endl;
std::cout << "Joint4 " << joint4 << std::endl;

//Set destination frame
	 
	    //std::cout << "Set end-effector position <x y z>:" << std::endl;

	    KDL::Vector dest_pos(cartpos.operator()(0,3),cartpos.operator()(1,3),cartpos.operator()(2,3));
	    KDL::Frame dest_frame(dest_pos);
	    int ret = iksolver.CartToJnt(q_init,dest_frame,q);

	        for (unsigned int i = 0; i < q.rows(); ++i)
	        {
	       //     std::cout << "Joint #" << i << ": " << q(i) << std::endl;
	        }
	
	
	  node.setParam("/joint6", joint6);
	  node.setParam("/joint5", joint5);
	  node.setParam("/joint4", joint4);

	//std::string str = boost::lexical_cast<std::string>(joint6);
	//std::cout << "Joint6 " + str;
	ros::spinOnce();
 	r.sleep();
}
return 0;

  
}













