#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
    class SubscribeAndPublish  
    {  
    public:  
      SubscribeAndPublish()  
      {  
        //Topic you want to publish  
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("/points", 1);  
      
        //Topic you want to subscribe  
        sub_ = n_.subscribe("/camera/depth/points", 1, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
        cnt_=0;
      }  
      
      void callback(const sensor_msgs::PointCloud2& input)  
      {  
          if (cnt_<3){
              cnt_++;
          }
          else{
        //.... do something with the input and generate the output...  
        pub_.publish(input);  
              cnt_=0;
          }

      }  
      
    private:  
      ros::NodeHandle n_;   
      ros::Publisher pub_;  
      ros::Subscriber sub_;  
      int cnt_;
      
    };//End of class SubscribeAndPublish  
      
    int main(int argc, char **argv)  
    {  
      //Initiate ROS  
      ros::init(argc, argv, "subscribe_and_publish");  
      
      //Create an object of class SubscribeAndPublish that will take care of everything  
      SubscribeAndPublish SAPObject;  
      
      ros::spin();  
      
      return 0;  
    }  
