#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h> 
#include <pcl/segmentation/sac_segmentation.h>

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
void draw_marker(pcl::ModelCoefficients::Ptr cof);

ros::Publisher pub1;
ros::Publisher pub2;
int main (int argc, char** argv)
{
  ros::init(argc, argv, "extract_line");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe ("/pcl_nodelet/cluster_decomposer/output00", 1, cloud_cb);
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output_cloud",1);
  pub2 = nh.advertise<visualization_msgs::Marker>("marker_arm_line", 0);
  ros::spin();
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  sensor_msgs::PointCloud2 output_cloud;
  //  std::vector<int> inliers;

  // pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
  //   model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
  //     pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);

  //     ransac.setDistanceThreshold (.005);
  //     ransac.computeModel();
  //     ransac.getInliers(inliers);
  //     pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
      
  //     Eigen::VectorXf model_coefficients;
  //     model_l -> computeModelCoefficients (inliers, model_coefficients);  
  
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  
  // Create the segmentation object  
  pcl::SACSegmentation<pcl::PointXYZ> seg;  
  // Optional  
  seg.setOptimizeCoefficients (true);  
  // Mandatory  
  seg.setModelType (pcl::SACMODEL_LINE);  
  seg.setMethodType (pcl::SAC_LMEDS);  
  seg.setDistanceThreshold (0.05);  
  
  seg.setInputCloud (cloud -> makeShared());  
  seg.segment (*inliers, *coefficients);
  
  std::cout<<*coefficients<<std::endl;
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *final);
 
  draw_marker(coefficients);
  pcl::toROSMsg(*final, output_cloud);
      pub1.publish(output_cloud);
}

void draw_marker(pcl::ModelCoefficients::Ptr cof)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = cof -> header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point p;
  p.x = cof -> values[0] - 1 * cof -> values[3];
  p.y = cof -> values[1] - 1 * cof -> values[4];
  p.z = cof -> values[2] - 1 * cof -> values[5];
  marker.points.push_back(p);
  p.x = cof -> values[0] + 1 * cof -> values[3];
  p.y = cof -> values[1] + 1 * cof -> values[4];
  p.z = cof -> values[2] + 1 * cof -> values[5];
  marker.points.push_back(p);

  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  pub2.publish(marker);
}
