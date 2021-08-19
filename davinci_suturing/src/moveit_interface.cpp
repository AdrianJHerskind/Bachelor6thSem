#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>

#include "davinci_imageProcessing/nextStitchPoint.h"

int upperLeftX = 108; int upperLeftY = 242;
geometry_msgs::Pose upperLeft;

int lowerLeftX = 108; int lowerLeftY = 372;
geometry_msgs::Pose lowerLeft;


int upperRightX = 504; int upperRightY = 185;
geometry_msgs::Pose upperRight;


int lowerRightX = 504; int lowerRightY = 384;
geometry_msgs::Pose lowerRight;




int nextStitchPointEntryX, nextStitchPointEntryY, nextStitchPointExitX, nextStitchPointExitY;

geometry_msgs::Pose transformPixelToCartesian()
{
	double xRelativeDistance = (nextStitchPointEntryX - (double)upperLeftX) / ((double)upperRightX-(double)upperLeftX);
	std::cout << "xRelativeDistance " << xRelativeDistance << "\n";
	geometry_msgs::Pose betweenULUR;
	betweenULUR.position.x = upperLeft.position.x * (1.0 - xRelativeDistance) + upperRight.position.x * xRelativeDistance;
	betweenULUR.position.y = upperLeft.position.y * (1.0 - xRelativeDistance) + upperRight.position.y * xRelativeDistance;
	betweenULUR.position.z = upperLeft.position.z * (1.0 - xRelativeDistance) + upperRight.position.z * xRelativeDistance;
	
	geometry_msgs::Pose betweenLLLR;
	betweenLLLR.position.x = lowerLeft.position.x * (1.0 - xRelativeDistance) + lowerRight.position.x * xRelativeDistance;
	betweenLLLR.position.y = lowerLeft.position.y * (1.0 - xRelativeDistance) + lowerRight.position.y * xRelativeDistance;
	betweenLLLR.position.z = lowerLeft.position.z * (1.0 - xRelativeDistance) + lowerRight.position.z * xRelativeDistance;

	double yRelativeDistanceLeft = ((double)lowerLeftY - nextStitchPointEntryY) / ((double)lowerLeftY - (double)upperLeftY);
	double yRelativeDistanceRight = ((double)lowerRightY - nextStitchPointEntryY) / ((double)lowerRightY - (double)upperRightY);

	double yRelativeDistanceAtX = yRelativeDistanceLeft * (1.0 - xRelativeDistance) + yRelativeDistanceRight * xRelativeDistance;
	std::cout << "yRelativeDistanceLeft " << yRelativeDistanceLeft << "\n";
	std::cout << "yRelativeDistanceRighr " << yRelativeDistanceRight << "\n";
	std::cout << "yRelativeDistanceAtX " << yRelativeDistanceAtX << "\n";
	geometry_msgs::Pose goalPose;
	goalPose.position.x = betweenULUR.position.x * yRelativeDistanceAtX + betweenLLLR.position.x * (1.0-yRelativeDistanceAtX);
	goalPose.position.y = betweenULUR.position.y * yRelativeDistanceAtX + betweenLLLR.position.y * (1.0-yRelativeDistanceAtX);
	goalPose.position.z = betweenULUR.position.z * yRelativeDistanceAtX + betweenLLLR.position.z * (1.0-yRelativeDistanceAtX);

	goalPose.position.y = goalPose.position.y;
	goalPose.position.z = goalPose.position.z - 0.005;
	std::cout << "Goal Pose Pos X " << goalPose.position.x << "\n";
	std::cout << "Goal Pose Pos Y " << goalPose.position.y << "\n";
	std::cout << "Goal Pose Pos Z " << goalPose.position.z << "\n" << "\n";
	//How far is 325 between 109 and 502?
	//How far is 251 between 242 and 372?
	/*(325-109) / ((502 - 109)) * 100 = 54.9618320611
	(251-242) / ((372 - 242)) * 100 = 6.92307692308

	(325-107) / ((506 - 107)) * 100 = 54.6365914787
	(251-185) / ((384 - 185)) * 100 = 33.1658291457*/

	return goalPose;
	
}

bool getNextStitchPoint(ros::NodeHandle nh)
{
ros::ServiceClient nextPointClient = nh.serviceClient<davinci_imageProcessing::nextStitchPoint>("get_next_stitch_point");

davinci_imageProcessing::nextStitchPoint nextStitchPointMsg;
	if (nextPointClient.call(nextStitchPointMsg))
	{
		ROS_INFO("SUCCES");
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return false;
	}
	std::cout << "SRV RESPONSE ENTRYX " << nextStitchPointMsg.response.EntryX << "\n";
	std::cout << "SRV RESPONSE ENTRYX " << nextStitchPointMsg.response.EntryY << "\n";
	std::cout << "SRV RESPONSE ENTRYX " << nextStitchPointMsg.response.ExitX << "\n";
	std::cout << "SRV RESPONSE ENTRYX " << nextStitchPointMsg.response.ExitY << "\n";
	nextStitchPointEntryX = nextStitchPointMsg.response.EntryX;
	nextStitchPointEntryY = nextStitchPointMsg.response.EntryY;
	nextStitchPointExitX = nextStitchPointMsg.response.ExitX;
	nextStitchPointExitY = nextStitchPointMsg.response.ExitY;
		return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_interface");
ros::NodeHandle node_handle;






  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("Hand1");
  moveit::planning_interface::MoveGroup::Plan my_plan;
std::map<std::string, double> joints;

tf::Quaternion newQuaternions;
double roll, pitch, yaw;
std::cout << "ROLL! " << "\n";
std::cin >> roll;
std::cout << "PITCH" << "\n";
std::cin >> pitch;
std::cout << "YAW" << "\n";
std::cin >> yaw;
newQuaternions = tf::createQuaternionFromRPY(roll, pitch, yaw);
std::cout << newQuaternions.length() << "\n";
std::cout << newQuaternions.x() << "\n";
std::cout << newQuaternions.y() << "\n";
std::cout << newQuaternions.z() << "\n";
std::cout << newQuaternions.w() << "\n";

tf::Quaternion q(newQuaternions.x(), newQuaternions.y(), newQuaternions.z(), newQuaternions.w());
tf::Matrix3x3 m(q);
m.getRPY(roll, pitch, yaw);
std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
char input2;
std::cin >> input2;

geometry_msgs::Pose offset;
offset.position.x = 2.14516;
offset.position.y = 0.000138753;
offset.position.z = 0.680336;
offset.orientation.x = -0.707071;
offset.orientation.y = -0.707142;
offset.orientation.z = 6.01047e-06;
offset.orientation.w = 6.25531e-06;

/*float w, vx, vy, vz;

float theta_x = 90;
std::cout << "input Euler X " << "\n";
std::cin >> theta_x;
float theta_y = 0;
std::cout << "input Euler Y " << "\n";
std::cin >> theta_y;
float theta_z = 0;
std::cout << "input Euler Z " << "\n";
std::cin >> theta_z;

theta_x = theta_x * 3.14159265359 / 180.0;
theta_y = theta_y * 3.14159265359 / 180.0;
theta_z = theta_z * 3.14159265359 / 180.0;
std::cout << "theta_x " << theta_x << "\n";
std::cout << "theta_y " << theta_y << "\n";
std::cout << "theta_z " << theta_z << "\n";
		float cos_z_2 = cosf(0.5*theta_z);
		float cos_y_2 = cosf(0.5*theta_y);
		float cos_x_2 = cosf(0.5*theta_x);

		float sin_z_2 = sinf(0.5*theta_z);
		float sin_y_2 = sinf(0.5*theta_y);
		float sin_x_2 = sinf(0.5*theta_x);

		// and now compute quaternion
		w  = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
		vx = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
		vy = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
		vz = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;*/

geometry_msgs::Pose home;
home.position.x = 2.14516;
home.position.y = 0.000138756;
home.position.z = 0.671336;
home.orientation.x = 3.54778e-05;
home.orientation.y = -0.707101;
home.orientation.z = 3.52329e-05;
home.orientation.w = 0.707113;

upperLeft.position.x = 2.14688;
upperLeft.position.y = -0.0422789;
upperLeft.position.z = 0.616183;
upperLeft.orientation.x = 0.498564;
upperLeft.orientation.y = -0.374666;
upperLeft.orientation.z = 0.693409;
upperLeft.orientation.w = 0.360892;

lowerLeft.position.x = 2.15401;
lowerLeft.position.y = -0.0612329;
lowerLeft.position.z = 0.617685;
lowerLeft.orientation.x = 0.469553;
lowerLeft.orientation.y = -0.34305;
lowerLeft.orientation.z = 0.721712;
lowerLeft.orientation.w = 0.375459;

upperRight.position.x = 2.22379;
upperRight.position.y = -0.0280917;
upperRight.position.z = 0.621466;
upperRight.orientation.x = 0.378588;
upperRight.orientation.y = -0.442572;
upperRight.orientation.z = 0.778246;
upperRight.orientation.w = 0.234806;

lowerRight.position.x = 2.22716;
lowerRight.position.y = -0.0477439;
lowerRight.position.z = 0.621564;
lowerRight.orientation.x = 0.363279;
lowerRight.orientation.y = -0.400273;
lowerRight.orientation.z = 0.802218;
lowerRight.orientation.w = 0.25349;

geometry_msgs::Pose safe;
safe.position.x = (upperLeft.position.x + lowerLeft.position.x + upperRight.position.x + lowerRight.position.x)/4.0;
safe.position.y = (upperLeft.position.y + lowerLeft.position.y + upperRight.position.y + lowerRight.position.y)/4.0;
safe.position.z = (upperLeft.position.z + lowerLeft.position.z + upperRight.position.z + lowerRight.position.z)/4.0 + 0.07;
safe.orientation.x = 0.5991;
safe.orientation.y = -0.339299;
safe.orientation.z = 0.589743;
safe.orientation.w = 0.422087;


geometry_msgs::Pose betweenULUR;
betweenULUR.position.x = upperLeft.position.x *0.8 + upperRight.position.x *0.2;
betweenULUR.position.y = upperLeft.position.y *0.8 + upperRight.position.y *0.2;
betweenULUR.position.z = upperLeft.position.z *0.8 + upperRight.position.z *0.2 + 0.005;
betweenULUR.orientation.x = newQuaternions.w(); // w
betweenULUR.orientation.y = newQuaternions.x(); // x
betweenULUR.orientation.z = newQuaternions.y(); // y
betweenULUR.orientation.w = newQuaternions.z(); // z

roll = roll; pitch += 0.3; yaw = yaw;
newQuaternions = tf::createQuaternionFromRPY(roll, pitch, yaw);
geometry_msgs::Pose betweenULUR2;
betweenULUR2.position.x = upperLeft.position.x *0.8 + upperRight.position.x *0.2;
betweenULUR2.position.y = upperLeft.position.y *0.8 + upperRight.position.y *0.2;
betweenULUR2.position.z = upperLeft.position.z *0.8 + upperRight.position.z *0.2 + 0.005;
betweenULUR2.orientation.x = newQuaternions.w(); // w
betweenULUR2.orientation.y = newQuaternions.x(); // x
betweenULUR2.orientation.z = newQuaternions.y(); // y
betweenULUR2.orientation.w = newQuaternions.z(); // z

int counter = 0;

geometry_msgs::Pose nextStitchPointPose;

double PI = 3.14159265359;

tf::Quaternion newQuaternionsCircle;
double rollCircleRad, pitchCircleRad, yawCircleRad;
double rollCircleDeg, pitchCircleDeg, yawCircleDeg;
rollCircleDeg = 175.0;// - (double)counter * 4.5;
pitchCircleDeg = 155.0 - (double)counter * 15.0;
yawCircleDeg = 55.0;// + (double)counter * 2.0;

rollCircleRad = rollCircleDeg / 180.0 * PI;
pitchCircleRad = pitchCircleDeg / 180.0 * PI;
yawCircleRad = yawCircleDeg / 180.0 * PI;
newQuaternionsCircle = tf::createQuaternionFromRPY(rollCircleRad, pitchCircleRad, yawCircleRad);



geometry_msgs::Pose circCenter;
/*circCenter.position.x = 2.14516;
circCenter.position.y = 0.000138756;
circCenter.position.z = 0.660336;*/
circCenter.position.x = upperLeft.position.x *0.5 + upperRight.position.x *0.5;
circCenter.position.y = upperLeft.position.y *0.5 + upperRight.position.y *0.5;
circCenter.position.z = upperLeft.position.z *0.5 + upperRight.position.z *0.5 - 0.001;
if(getNextStitchPoint(node_handle))
{
	circCenter = transformPixelToCartesian();
}
circCenter.orientation.x = newQuaternionsCircle.w();
circCenter.orientation.y = newQuaternionsCircle.x();
circCenter.orientation.z = newQuaternionsCircle.y();
circCenter.orientation.w = newQuaternionsCircle.z();

double r = 0.0105; // radius of needle

    ros::Time last = ros::Time::now();
    ros::Rate rosr(1);



geometry_msgs::Pose nextPose;
std::map<std::string, double> nextPoint;
group.setStartStateToCurrentState();

	nextPose = home;
	group.setPoseTarget(nextPose);
	bool successFirst = group.plan(my_plan);
	char done;
	std::cin >> done;
	nextPose = safe;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	/*nextPose = upperLeft;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = safe;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = lowerLeft;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = safe;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = upperRight;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = safe;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = lowerRight;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;
	nextPose = safe;
	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);
	std::cin >> done;	*/

	/*rollCircleDeg = 175.0;// - (double)counter * 4.5;
	pitchCircleDeg = 155.0 - (double)counter * 15.0;
	yawCircleDeg = 55.0;// + (double)counter * 2.0;*/

	rollCircleDeg = 90.0;// - (double)counter * 4.5;
	pitchCircleDeg = 0.0 - (double)counter * 15.0;
	yawCircleDeg = 0.0;// + (double)counter * 2.0;

	rollCircleRad = rollCircleDeg / 180.0 * PI;
	pitchCircleRad = pitchCircleDeg / 180.0 * PI;
	yawCircleRad = yawCircleDeg / 180.0 * PI;

	std::cout << "RCR " << rollCircleRad << "\n";
	std::cout << "PCR " << pitchCircleRad << "\n";
	std::cout << "YCR " << yawCircleRad << "\n";
	newQuaternionsCircle = tf::createQuaternionFromRPY(rollCircleRad, pitchCircleRad, yawCircleRad);

	nextPose = circCenter;
	nextPose.position.x = circCenter.position.x + 0;
	nextPose.position.y = circCenter.position.y + r * sin((200.0-(double)counter*10.0) * PI / 180.0);
	nextPose.position.z = circCenter.position.z - r * cos((200.0-(double)counter*10.0) * PI / 180.0);
	nextPose.orientation.x = newQuaternionsCircle.w();
	nextPose.orientation.y = newQuaternionsCircle.x();
	nextPose.orientation.z = newQuaternionsCircle.y();
	nextPose.orientation.w = newQuaternionsCircle.z();

	nextPose.orientation.x = 3.54778e-05;
	nextPose.orientation.y = -0.707101;
	nextPose.orientation.z = 3.52329e-05;
	nextPose.orientation.w = 0.707113;

	group.setPoseTarget(nextPose);
	successFirst = group.plan(my_plan);

	std::cin >> done;


while(ros::ok())
{
/*geometry_msgs::Pose nextPose;
std::map<std::string, double> nextPoint;*/
group.setStartStateToCurrentState();

	
	rollCircleDeg = 175.0;// - (double)counter * 4.5;
	if(pitchCircleDeg > 150)
	{
		pitchCircleDeg = pitchCircleDeg - 5.0;
	}
	else if(pitchCircleDeg <= 150 && pitchCircleDeg > 110)
	{
		pitchCircleDeg = pitchCircleDeg - 3.5;
	}
	else if(pitchCircleDeg <= 110 && pitchCircleDeg > 70)
	{
		pitchCircleDeg = pitchCircleDeg - 3.0;
	}
	else if(pitchCircleDeg <= 70 && pitchCircleDeg > 20)
	{
		pitchCircleDeg = pitchCircleDeg - 3.5;
	}
	else if(pitchCircleDeg <= 20 && pitchCircleDeg > 0)
	{
		pitchCircleDeg = pitchCircleDeg - 5.0;
	}
	
	//pitchCircleDeg = 165.0 - (double)counter * 3.5;
	yawCircleDeg = 55.0;// + (double)counter * 2.0;

	rollCircleRad = rollCircleDeg / 180.0 * PI;
	pitchCircleRad = pitchCircleDeg / 180.0 * PI;
	yawCircleRad = yawCircleDeg / 180.0 * PI;

	std::cout << "RCR " << rollCircleRad << "\n";
	std::cout << "PCR " << pitchCircleRad << "\n";
	std::cout << "YCR " << yawCircleRad << "\n";
	newQuaternionsCircle = tf::createQuaternionFromRPY(rollCircleRad, pitchCircleRad, yawCircleRad);

	nextPose = circCenter;
	nextPose.position.x = circCenter.position.x + 0;
	nextPose.position.y = circCenter.position.y + r * sin((200.0-(double)counter*2.0) * PI / 180.0);
	nextPose.position.z = circCenter.position.z - r * cos((200.0-(double)counter*2.0) * PI / 180.0);
	nextPose.orientation.x = newQuaternionsCircle.w();
	nextPose.orientation.y = newQuaternionsCircle.x();
	nextPose.orientation.z = newQuaternionsCircle.y();
	nextPose.orientation.w = newQuaternionsCircle.z();

	/*if(counter == 0)
	{
		nextPose = home;
	}
	if(counter == 1)
	{
		nextPose = upperLeft;
	}
	if(counter == 2)
	{
		nextPose = home;
	}
	if(counter == 3)
	{
		nextPose = upperRight;
	}
	if(counter == 4)
	{
		nextPose = home;
	}
	if(counter == 5)
	{
		nextPose = lowerLeft;
	}
	if(counter == 6)
	{
		nextPose = home;
	}
	if(counter == 7)
	{
		nextPose = lowerRight;
	}*/
	group.setPoseTarget(nextPose);
	bool success = group.plan(my_plan);





	

	/*std::map<std::string, double> home2;
	home2["p4_hand_roll"] = 0.0;
	home2["p4_hand_pitch"] = 0.0;
	home2["p4_instrument_slide"] = 0.0;
	home2["p4_instrument_roll"] = 0.0;
	home2["p4_instrument_pitch"] = 0.000;
	home2["p4_instrument_jaw_left"] = 0.0;
	group.setJointValueTarget(nextPoint);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
	group.move();*/

	//group.move();

	geometry_msgs::PoseStamped currentPose = group.getCurrentPose();
	std::cout << "Current Position X " << currentPose.pose.position.x << /*" but wanted to write " << nextPose.position.x <<*/ "\n";
	std::cout << "Current Position Y " << currentPose.pose.position.y << /*" but wanted to write " << nextPose.position.y <<*/ "\n";
	std::cout << "Current Position Z " << currentPose.pose.position.z << /*" but wanted to write " << nextPose.position.z <<*/ "\n";
	std::cout << "Current Quaternion X " << currentPose.pose.orientation.x << /*" but wanted to write " << nextPose.orientation.x <<*/ "\n";
	std::cout << "Current Quaternion Y " << currentPose.pose.orientation.y << /*" but wanted to write " << nextPose.orientation.y <<*/ "\n";
	std::cout << "Current Quaternion Z " << currentPose.pose.orientation.z << /*" but wanted to write " << nextPose.orientation.z <<*/ "\n";
	std::cout << "Current Quaternion W " << currentPose.pose.orientation.w << /*" but wanted to write " << nextPose.orientation.w <<*/ "\n";
tf::Quaternion q2(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w);
tf::Matrix3x3 m2(q2);
m2.getRPY(roll, pitch, yaw);
std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;


	std::vector<double> currentJointValues = group.getCurrentJointValues();
	std::vector< std::string > list = group.getJoints();
	/*std::cout << "list size " << list.size() << "\n";
	std::cout << "list 0 " << list.at(0) << " is " << currentJointValues.at(0) << "\n";
	std::cout << "list 1 " << list.at(1) << " is " << currentJointValues.at(1) <<"\n";
	std::cout << "list 2 " << list.at(2) << " is " << currentJointValues.at(2) <<"\n";
	std::cout << "list 3 " << list.at(3) << " is " << currentJointValues.at(3) <<"\n";
	std::cout << "list 4 " << list.at(4) << " is " << currentJointValues.at(4) <<"\n";
	std::cout << "list 5 " << list.at(5) << " is " << currentJointValues.at(5) <<"\n";
	std::cout << "list 6 " << list.at(6) << " is " << currentJointValues.at(6) <<"\n";*/

	/*char input;
	std::cin >> input;
	if(input == 'd')
	{
		//if(reachCheckPoint(nextPose, currentPose))
		{
			counter++;
			if(counter > 10)
			{
				counter = 10;
			}
		}
	}
	else if(input == 'a')
	{
		//if(reachCheckPoint(nextPose, currentPose))
		{
			counter--;
			if(counter < 0)
			{
				counter = 0;
			}
		}
	}
	else if(input == 's')
	{
		ros::shutdown();
	}*/
	/*char wait;
	std::cin >> wait;*/
	std::cout << "counter " << counter << "\n";
	counter = counter + 1;
	if(counter > 70.0)
	{
		ros::shutdown();
	}
        rosr.sleep();

}
}
