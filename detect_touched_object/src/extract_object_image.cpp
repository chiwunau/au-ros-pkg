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
  std::vector<jsk_pcl_ros::BoundingBox> boxes_;
  std::vector<jsk_pcl_ros::BoundingBox>::iterator boxes_it_;
  ros::Subscriber boxes_sub_;
  std::vector<cv::Point2d> pos_;
  std::vector<cv::Point2d>::iterator pos_it_;
  std::vector< std::vector<cv::Point2d> > vertices_arr_; 
  std::vector<cv::Point2d>::iterator vertices_it_;
  std::vector< std::vector<cv::Point2d> >::iterator vertices_arr_it_; 
  bool cam_info_ok_;
  bool box_ok_;
  cv::Mat image_src_;
  cv::Mat image_dst_;
  cv_bridge::CvImagePtr input_bridge_;
  cv_bridge::CvImagePtr output_bridge_;
  int max_dist_;
  const int max_dist_limit_;
  
  
public:
  FrameDrawer()
    : it_(nh_),
      max_dist_limit_(256)      
  {
    std::cout<<"framedrawer created"<<std::endl;
    std::string image_topic = nh_.resolveName("image");
    boxes_sub_ = nh_.subscribe("/cluster_decomposer_final/boxes",1, &FrameDrawer::boxesCb,this);
    image_sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    image_pub_ = it_.advertise("image_out",1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
    cam_info_ok_ = false;
    box_ok_ = false;
    cv::namedWindow("OUTPUT_WINDOW");
    cv::setMouseCallback("OUTPUT_WINDOW", &FrameDrawer::MouseCb, NULL);
    cv::createTrackbar("max_dist", "OUTPUT_WINDOW", &(this -> max_dist_), max_dist_limit_);
  }
  ~FrameDrawer()
  {
    cv::destroyWindow("OUTPUT_WINDOW");
  }

  void boxesCb(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
  {
    this->box_ok_ = true;
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
	  if(pos.z < (5.0 *(double) max_dist_ / max_dist_limit_))
	    {
	      std::vector<cv::Point2d> vertices;
	      geometry_msgs::Quaternion orient = box.pose.orientation;
	      geometry_msgs::Vector3 dim = box.dimensions;
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
	      pos_.push_back(cam_model_.project3dToPixel(pos_world));
	    }
	  ++boxes_it_;
	}
      

    }
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
    
    if (this->box_ok_)
      this->imageDraw();
  }

  void imageDraw()
  {
    cv::Mat image = output_bridge_->image;
    std::vector< std::vector<cv::Point2f> > hull( vertices_arr_.size());
    pos_it_ = pos_.begin();
    vertices_arr_it_ = vertices_arr_.begin();
        
    int i=0;
    int j=0;
    while(pos_it_ != pos_.end() && vertices_arr_it_ != vertices_arr_.end())
      {
	cv::circle(image, *pos_it_, 5, CV_RGB(255, 0, 0), -1);
	
	vertices_it_ = (*vertices_arr_it_).begin();
	while(vertices_it_ != (*vertices_arr_it_).end())
	  {
	    cv::circle(image, *vertices_it_, 1, CV_RGB(0, 0, 0), -1);
	    ++vertices_it_;
	  }

	cv::Mat test = (cv::Mat_<float>(3,2) << 10,20,30,40,50,60);
	std::vector<cv::Point2f> hull_t;
	//std::cout<<test<<std::endl;

	std::vector<cv::Point2f> vertices_arr_tmp;
	cv::Mat(*vertices_arr_it_).copyTo(vertices_arr_tmp);
	cv::convexHull(cv::Mat(vertices_arr_tmp), hull[i], false);
	//	std::cout<<hull[i]<<std::endl;
	++pos_it_;
	++vertices_arr_it_;
	++i;
      }
    //    cv::drawContours( image, hull, -1, cv::Scalar(0,0,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point2f(0,0) );
    
    
    for(i=0; i<hull.size(); i++){
      for( j=0; j<hull[i].size(); j++){
	cv::line(image, hull[i][j], hull[i][ j+1 < hull[i].size()? j+1 : 0 ], cv::Scalar(100,100,200), 2, CV_AA);
      }
    }
    cv::imshow("OUTPUT_WINDOW",output_bridge_ -> image);
    cv::waitKey(3);
    image_pub_.publish(output_bridge_->toImageMsg()); 
  }

  static void MouseCb(int event, int x, int y, int flags, void* userdata)
  {
    if (event == cv::EVENT_LBUTTONDOWN){
      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
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
