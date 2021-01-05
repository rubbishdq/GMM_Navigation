#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "submap/KMeans.h"
#include "submap/GMM.h"
#include <submap/gmm.h> //gmm msg

class rviz_gmm {
public:
  rviz_gmm() {  }
  ~rviz_gmm() {}
void initVisual(ros::NodeHandle n);
void gmmCallback(const submap::gmm gmm);

private:
    visualization_msgs::Marker marker;
    ros::Subscriber gmm_sub_;
    ros::Publisher marker_pub_;
};


void rviz_gmm::initVisual(ros::NodeHandle node_){
    gmm_sub_ = node_.subscribe<submap::gmm>("/gmm_after_trans", 1000, &rviz_gmm::gmmCallback, this);
      std::cout<<"dddd"<<std::endl;
    marker_pub_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
      std::cout<<"eeee"<<std::endl;
}

void rviz_gmm::gmmCallback(const submap::gmm gmm){
  if(gmm.mix_num>0){
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    marker.pose.position.x = gmm.x[5];
    marker.pose.position.y = gmm.y[5];
    marker.pose.position.z = gmm.z[5];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    while (marker_pub_.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub_.publish(marker);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "rviz_gmm");
  ros::NodeHandle n("~");
  std::cout<<"aaaa"<<std::endl;
  rviz_gmm visualization;
    std::cout<<"bbbb"<<std::endl;
  visualization.initVisual(n);
    std::cout<<"cccc"<<std::endl;
  ros::spin();
  // return 0;
    // // Cycle between different shapes
    // switch (shape)
    // {
    // case visualization_msgs::Marker::CUBE:
    //   shape = visualization_msgs::Marker::SPHERE;
    //   break;
    // case visualization_msgs::Marker::SPHERE:
    //   shape = visualization_msgs::Marker::ARROW;
    //   break;
    // case visualization_msgs::Marker::ARROW:
    //   shape = visualization_msgs::Marker::CYLINDER;
    //   break;
    // case visualization_msgs::Marker::CYLINDER:
    //   shape = visualization_msgs::Marker::CUBE;
    //   break;
    // }


}