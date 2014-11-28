#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <math.h>
#include "jsk_pcl_ros/BoundingBoxArray.h"
#include "jsk_pcl_ros/BoundingBoxArray.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
  std::map<int, jsk_pcl_ros::BoundingBox> boxes_;
  std::map<int, jsk_pcl_ros::BoundingBox>::iterator boxes_it_;
  jsk_pcl_ros::BoundingBox selected_box_;
  ros::Subscriber boxes_sub_;
  std::map<int, cv::Point2d> pos_;
  std::map<int, cv::Point2d>::iterator pos_it_;
  std::vector< std::vector<cv::Point2d> > vertices_arr_; 
  std::vector<cv::Point2d>::iterator vertices_it_;
  std::vector< std::vector<cv::Point2d> >::iterator vertices_arr_it_; 
  std::vector<cv::Point2d> selected_box_vertices_;
  ros::Subscriber selected_box_sub_;
  ros::Publisher selected_box_pub_;
  cv::Point2d selected_box_pos_;
  
  bool cam_info_ok_;
  bool boxes_ok_;
  bool box_selected_ok_;
  cv::Mat image_src_;
  cv::Mat image_dst_;
  cv_bridge::CvImagePtr input_bridge_;
  cv_bridge::CvImagePtr output_bridge_;
  int max_dist_;
  const int max_dist_limit_;
  int max_size_;
  const int max_size_limit_;
  
  
public:
  FrameDrawer()
    : it_(nh_),
      max_dist_limit_(256),
      max_size_limit_(500)
  {
    std::cout<<"framedrawer created"<<std::endl;
    std::string image_topic = nh_.resolveName("image");
    boxes_sub_ = nh_.subscribe("/cluster_decomposer_final/boxes",1, &FrameDrawer::boxesCb,this);
    selected_box_sub_ = nh_.subscribe("/bounding_box_marker/selected_box",1, &FrameDrawer::selectedBoxCb, this);
    selected_box_pub_ = nh_.advertise<jsk_pcl_ros::BoundingBox>("/bounding_box_marker/selected_box",1);
    image_sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    image_pub_ = it_.advertise("image_out",1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    cam_info_ok_ = false;
    boxes_ok_ = false;
    box_selected_ok_ = false;
    cv::namedWindow("OUTPUT_WINDOW");
    cv::setMouseCallback("OUTPUT_WINDOW", &FrameDrawer::mouseCbRelay, this);
    cv::createTrackbar("max_dist", "OUTPUT_WINDOW", &(this -> max_dist_), max_dist_limit_);
    cv::createTrackbar("max_size", "OUTPUT_WINDOW", &(this -> max_size_), max_size_limit_);
  }
  ~FrameDrawer()
  {
    cv::destroyWindow("OUTPUT_WINDOW");
  }

  void boxesCb(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
  {
    this->boxes_ok_ = true;
    boxes_.clear();
    pos_.clear();
    vertices_arr_.clear();
    std::vector<jsk_pcl_ros::BoundingBox> bs = msg->boxes;
    std::vector<jsk_pcl_ros::BoundingBox>::iterator bs_it = bs.begin();
    
    int i = 0;
    while (bs_it != bs.end())
      {
	boxes_.insert(std::make_pair(i, *bs_it));
	++bs_it;
	++i;
      }
    ROS_INFO("BoxesCb, Total boxes: %lu", boxes_.size());

    if(!cam_info_ok_){
      ROS_INFO("Waiting for camera info");
    }
    else{
      boxes_it_ = boxes_.begin();
      while(boxes_it_ != boxes_.end())
	{
	  jsk_pcl_ros::BoundingBox box = (*boxes_it_).second;
	  geometry_msgs::Point pos = box.pose.position;
	  geometry_msgs::Vector3 dim = box.dimensions;
	  double box_size = dim.x * dim.y * dim.z;
	  if ((pos.z < 5.0 * (double)max_dist_ / max_dist_limit_) && (box_size < 1.5 *pow((double)max_size_ / max_size_limit_, 3)))
	    {
	      std::cout<<pos.z<<"   "<<box_size<<std::endl;
	      std::vector<cv::Point2d> vertices;
	      geometry_msgs::Quaternion orient = box.pose.orientation;
	      cv::Vec3d pos_world(pos.x, pos.y, pos.z);

	      cv::Mat rot = this->quatToMatrix(orient);
	      
	      int i,j,k,a,b,c;
	      for(i=0; i<2; i++){
	        for(j=0; j<2; j++){
	      	for(k=0;k<2; k++){
	      	  a = i==0 ? 1: -1;
	      	  b = j==0 ? 1: -1;
	      	  c = k==0 ? 1: -1;
	      	  cv::Vec3d vertex = cv::Vec3d(a*dim.x/2, b*dim.y/2, c*dim.z/2);
		  cv::Mat vertex_tmp = (cv::Mat_<double>(3,1)<< vertex[0], vertex[1], vertex[2]);
		  vertex_tmp = rot * vertex_tmp;
		  vertex = cv::Vec3d(vertex_tmp.at<double>(0,0),vertex_tmp.at<double>(1,0),vertex_tmp.at<double>(2,0));
	      	  vertex = vertex + pos_world;
		  vertices.push_back(cam_model_.project3dToPixel(vertex));
	      	}}}
	      vertices_arr_.push_back(vertices);
	      pos_.insert(std::make_pair( (*boxes_it_).first, cam_model_.project3dToPixel(pos_world) ));
	    }
	  ++boxes_it_;
	}
      

    }
  }
  
  inline void selectedBoxCb(const jsk_pcl_ros::BoundingBox::ConstPtr& msg){
    selected_box_vertices_.clear();
    selected_box_ = *msg;
    geometry_msgs::Point pos = selected_box_.pose.position;
    geometry_msgs::Quaternion orient = selected_box_.pose.orientation;
    geometry_msgs::Vector3 dim = selected_box_.dimensions;
    cv::Vec3d pos_world(pos.x, pos.y, pos.z);
    cv::Mat rot = this->quatToMatrix(orient);
    int i,j,k,a,b,c;
    for(i=0; i<2; i++){
      for(j=0; j<2; j++){
	for(k=0;k<2; k++){
	  a = i==0 ? 1: -1;
	  b = j==0 ? 1: -1;
	  c = k==0 ? 1: -1;
	  cv::Vec3d vertex = cv::Vec3d(a*dim.x/2, b*dim.y/2, c*dim.z/2);
	  cv::Mat vertex_tmp = (cv::Mat_<double>(3,1)<< vertex[0], vertex[1], vertex[2]);
	  vertex_tmp = rot * vertex_tmp;
	  vertex = cv::Vec3d(vertex_tmp.at<double>(0,0),vertex_tmp.at<double>(1,0),vertex_tmp.at<double>(2,0));
	  vertex = vertex + pos_world;
	  selected_box_vertices_.push_back(cam_model_.project3dToPixel(vertex));
	}}}
    selected_box_pos_ = cam_model_.project3dToPixel(pos_world);
    box_selected_ok_ = true;
  }
    

  
  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
	       const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_INFO("ImageCb");
    //    cv::Mat image;
    try
      {
      input_bridge_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      output_bridge_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      
      }
    catch (cv_bridge::Exception& ex)
      {
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }
    cam_info_ok_ = cam_model_.fromCameraInfo(info_msg);
    
    if (this->boxes_ok_)
      this->imageDraw();
  }

  void imageDraw()
  {
    cv::Mat image = output_bridge_->image;
    std::vector< std::vector<cv::Point2f> > hull(vertices_arr_.size());
    std::vector< std::vector<cv::Point2f> >::iterator hull_it;
    pos_it_ = pos_.begin();
    vertices_arr_it_ = vertices_arr_.begin();
            
    if(box_selected_ok_)
      {
    	std::vector<cv::Point2f> selected_box_hull;
	std::vector<cv::Point2f> vertices_tmp;
	cv::Mat(selected_box_vertices_).copyTo(vertices_tmp);
     	cv::convexHull(cv::Mat(vertices_tmp), selected_box_hull, false);
    	for(int i=0; i<selected_box_hull.size(); i++){
    	  cv::line(image, selected_box_hull[i], selected_box_hull[ i+1 < selected_box_hull.size()? i+1 : 0 ], cv::Scalar(0,255,255), 2, CV_AA);
    	}
      }
    
    hull_it = hull.begin();
    while(pos_it_ != pos_.end() && vertices_arr_it_ != vertices_arr_.end())
      {
	cv::circle(image, (*pos_it_).second, 2, CV_RGB(255, 0, 0), -1);
	
	std::vector<cv::Point2f> vertices_arr_tmp;
	cv::Mat(*vertices_arr_it_).copyTo(vertices_arr_tmp);
	cv::convexHull(cv::Mat(vertices_arr_tmp), *hull_it, false);
	++pos_it_;
	++vertices_arr_it_;
	++hull_it;
      }
    //    cv::drawContours( image, hull, -1, cv::Scalar(0,0,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point2f(0,0) );
    
    for(int i=0; i<hull.size(); i++){
      for(int j=0; j<hull[i].size(); j++){
	cv::line(image, hull[i][j], hull[i][ j+1 < hull[i].size()? j+1 : 0 ], cv::Scalar(100,100,200), 2, CV_AA);
      }
    }

    cv::imshow("OUTPUT_WINDOW",output_bridge_ -> image);
    cv::waitKey(3);
    image_pub_.publish(output_bridge_->toImageMsg()); 
  }

  static void mouseCbRelay(int event, int x, int y, int flags, void* pt){
    FrameDrawer *self = static_cast<FrameDrawer*>(pt);
    self-> mouseCb(event, x, y, flags, pt);
  }

  void mouseCb(int event, int x, int y, int flags, void* pt)
  {
    // FrameDrawer *self = static_cast<FrameDrawer*>(userdata);
    FrameDrawer *self = static_cast<FrameDrawer*>(pt);
    if (event == cv::EVENT_LBUTTONDOWN){
      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
      jsk_pcl_ros::BoundingBox pub_box;
      int cloest_box_idx = 0;
      double dist = 0;
      double cloest_dist = 0;
      std::cout<< self -> max_dist_<<std::endl;
      pos_it_ = pos_.begin();
    	while(pos_it_ != pos_.end())
    	{
    	  dist =  pow((*pos_it_).second.x - x, 2) + pow((*pos_it_).second.y - x, 2);
    	  if (dist < cloest_dist || cloest_dist == 0)
    	    {
    	      cloest_dist = dist;
    	      cloest_box_idx = (*pos_it_).first;
    	    }
    	  pos_it_++;
    	}
      boxes_it_ = boxes_.find(cloest_box_idx);
      pub_box = (*boxes_it_).second;
      pub_box.header.stamp = ros::Time::now();
      selected_box_pub_.publish(pub_box);
    }
  }

    

  cv::Mat quatToMatrix(geometry_msgs::Quaternion q)
  {
    cv::Mat rot = cv::Mat_<double>(3,3);

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
    
    return rot;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extract_object_image");
  FrameDrawer drawer;
  ros::spin();
}
