#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include "jsk_pcl_ros/BoundingBoxArray.h"
#include "jsk_pcl_ros/BoundingBoxArray.h"

class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  //  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  //std::vector<std::string> frame_ids_;
  CvFont font_;
  std::vector<jsk_pcl_ros::BoundingBox> boxes_;
  std::vector<jsk_pcl_ros::BoundingBox>::iterator boxes_it_;
  ros::Subscriber boxes_sub_;
  std::vector<cv::Point2d> pos_;
  std::vector<cv::Point2d>::iterator pos_it_;
  std::vector< std::vector<cv::Point2d> > vertices_arr_; 
  std::vector<cv::Point2d>::iterator vertices_it_;
  std::vector< std::vector<cv::Point2d> >::iterator vertices_arr_it_; 
  bool cam_info_ok_;
  
public:
  FrameDrawer()
    : it_(nh_)
  {
    std::cout<<"framedrawer created"<<std::endl;
    std::string image_topic = nh_.resolveName("image");
    boxes_sub_ = nh_.subscribe("/cluster_decomposer_final/boxes",1, &FrameDrawer::boxesCb,this);
    image_sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    image_pub_ = it_.advertise("image_out",1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    cam_info_ok_ = false;
  }

  void boxesCb(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
  {
    boxes_ = msg->boxes;
    boxes_it_ = boxes_.begin();
    pos_.clear();
    vertices_arr_.clear();
    ROS_INFO("BoxesCb, Total boxes: %lu", boxes_.size());
    
    if(!cam_info_ok_){
      ROS_INFO("Waiting for camera info");
    }
    else{
      while(boxes_it_ != boxes_.end())
	{
	  jsk_pcl_ros::BoundingBox box = *boxes_it_;
	  geometry_msgs::Point pos = box.pose.position;
	  if(pos.z < 1.50)
	    {
	      std::vector<cv::Point2d> vertices;
	      geometry_msgs::Quaternion orient = box.pose.orientation;
	      geometry_msgs::Vector3 dim = box.dimensions;
	      cv::Vec3d pos_world(pos.x, pos.y, pos.z);
	      
	      int i,j,k,a,b,c;
	      for(i=0; i<2; i++){
	        for(j=0; j<2; j++){
	      	for(k=0;k<2; k++){
	      	  a = i==0 ? 1: -1;
	      	  b = j==0 ? 1: -1;
	      	  c = k==0 ? 1: -1;
	      	  cv::Vec3d vertex_tmp = cv::Vec3d(a*dim.x/2, b*dim.y/2, c*dim.z/2);
	      	  vertex_tmp = this->quatToMatrix(vertex_tmp, orient);
	      	  vertex_tmp = vertex_tmp + pos_world;
		    vertices.push_back(cam_model_.project3dToPixel(vertex_tmp));
	      	}}}
	      vertices_arr_.push_back(vertices);
	      pos_.push_back(cam_model_.project3dToPixel(pos_world));
	    }
	  ++boxes_it_;
	}
    }
  }
  cv::Vec3d quatToMatrix(cv::Vec3d v, geometry_msgs::Quaternion q)
  {
    cv::Mat rot = cv::Mat_<double>(3,3);
    cv::Mat vec = (cv::Mat_<double>(3,1)<< v[0], v[1], v[2]);
    double sqw = q.w*q.w;
    double sqx = q.x*q.x;
    double sqy = q.y*q.y;
    double sqz = q.z*q.z;

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    rot.at<double>(0,0) = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    rot.at<double>(1,1) = (-sqx + sqy - sqz + sqw)*invs ;
    rot.at<double>(2,2) = (-sqx - sqy + sqz + sqw)*invs ;
    
    double tmp1 = q.x*q.y;
    double tmp2 = q.z*q.w;
    rot.at<double>(1,0) = 2.0 * (tmp1 + tmp2)*invs ;
    rot.at<double>(0,1) = 2.0 * (tmp1 - tmp2)*invs ;
    
    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    rot.at<double>(2,0) = 2.0 * (tmp1 - tmp2)*invs ;
    rot.at<double>(0,2) = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    rot.at<double>(2,1) = 2.0 * (tmp1 + tmp2)*invs ;
    rot.at<double>(1,2) = 2.0 * (tmp1 - tmp2)*invs ;
    
    cv::Mat mul = cv::Mat_<double>(3,1);
    //    std::cout<<rot<<std::endl<<std::endl<<vec<<std::endl;
    mul = rot * vec;
    v = cv::Vec3d(mul.at<double>(0,0),mul.at<double>(1,0),mul.at<double>(2,0));
    return v;
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
	       const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_INFO("ImageCb");
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try
      {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex)
      {
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    cam_info_ok_ = cam_model_.fromCameraInfo(info_msg);
    
    pos_it_ = pos_.begin();
    vertices_arr_it_ = vertices_arr_.begin();
    while(pos_it_ != pos_.end() && vertices_arr_it_ != vertices_arr_.end())
      {
	cv::circle(image, *pos_it_, 5, CV_RGB(255, 0, 0), -1);
	vertices_it_ = (*vertices_arr_it_).begin();

	while(vertices_it_ != (*vertices_arr_it_).end())
	  {
	    cv::circle(image, *vertices_it_, 1, CV_RGB(0, 0, 0), -1);
	    ++vertices_it_;
	  }
	++pos_it_;
	++vertices_arr_it_;
      }
    image_pub_.publish(input_bridge->toImageMsg()); 
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extract_object_image");
  FrameDrawer drawer;
  ros::spin();
}
