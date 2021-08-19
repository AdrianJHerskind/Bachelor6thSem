#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <ros/console.h>

ros::Publisher pub;
	int input;
//Mat image = Mat(;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  //sensor_msgs::Image image;
 // sensor_msgs::PointCloud output2;
 output = *input;
//pcl::PointCloud<pcl::PointXYZRGB> cloud;
//pcl::fromROSMsg(output, cloud);
//pcl::toROSMsg(cloud, output2);
//output = *input;
//pcl::toROSMsg(output, image);
//std::cout << pcl::getFieldsList(output) << "\n";
  // Do data processing here...


  // Publish the data.
  pub.publish (output);
 /* std::cout << "height of PointCloud " << input->height << "\n";
  std::cout << "width of PointCloud " << input->width << "\n";
  std::cout << "Field Data Name " << input->fields[3] << "\n";
  std::cout << "Is this data bigendian? " << input->is_bigendian << "\n";
  std::cout << "Length of a point in bytes " << input->point_step << "\n";
  std::cout << "Length of a row in bytes " << input->row_step << "\n";
  std::cout << "Data.size() " << input->data.size() << "\n";
  std::cout << "True if there are no invalid points " << input->is_dense << "\n";*/
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "davinci_imageProcessing_node2");
  ros::NodeHandle nh;
while(ros::ok())
{
	std::cin >> input;
	std::cout << "input " << input << "\n";
 	nh.setParam("continue", input);
 	nh.setParam("continue", 2);
	//nh.setParam("continue", 2);
  // Create a ROS subscriber for the input point cloud
 // ros::Subscriber sub = nh.subscribe ("output", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
	//pub = nh.advertise<sensor_msgs::Image> ("output2", 1000);

  // Spin
  ros::spinOnce();
}
}
