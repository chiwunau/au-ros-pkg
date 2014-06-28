#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <jsk_pcl_ros/BoundingBoxArray.h>
#include <jsk_pcl_ros/BoundingBox.h>

class FrameDrawer
{
  ros::NodeHandle nh_;
  ros::Subscriber boxs_sub_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;
  
  
public:
  FrameDrawer()
    : it_(nh_)
  {
    std::string image_topic = nh_.resolveName("image");
    boxs_sub_ = nh_.subscriber<jsk_pcl_ros::BoundingBoxArray>("/cluster_decomposer_final/boxes", 1, &FrameDrawer::boundingBoxCb);
    image_sub_ = it.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    image_pub_ = it_.advertise("image_out",1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
  }

  void boundingBoxCb(const jsk_pcl_ros::BoundingBoxArray& msg)
  {
    jsk_pcl_ros::BoundingBox *boxes = msg->boxes;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extract_object_image");
  FrameDrawer drawer();
  ros::spin();
}
