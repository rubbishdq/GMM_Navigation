#include "submap_extraction.h"


void Submap::initMap(ros::NodeHandle nh_){
    mapcnt_=0;
    submap_num_=0;
    subgmm_num_=0;
    dis_threshold_=3.0;
    GlobalGMM_=new GMM(dim_,cluster_num_);
    std::cout<<"init begin"<<std::endl;
    //second param is queue length
    map_sub_=nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1000, &Submap::mapCallback,this);///lio_sam/deskew/cloud_deskewed  /points
    gobalmap_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/globalmap",1000,this);
    gmm_pub_=nh_.advertise<submap::gmm>("/subgmm",1000,this);
    gmm_list_pub_=nh_.advertise<submap::gmmlist>("/subgmm_list",1000,this);
    path_pub_= nh_.advertise<nav_msgs::Path>("/trajectory",1, true); 
    current_time_= ros::Time::now(); 
    path_.header.stamp=current_time_; 
	path_.header.frame_id="map"; 
    
    // submap_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/submap_list",1000,this);
    // subTF_pub_=nh_.advertise<tf::StampedTransform& transform>("/subTF_list",1000,this);
    std::thread mythread1_(&Submap::Global_Pointcloud_Publisher, this);
    mythread1_.detach();
    std::thread mythread2_(&Submap::Global_GMM_Publisher, this);// pub gmm submap & gmm list
    mythread2_.detach();
    std::thread mythread3_(&Submap::Submap_GMM_building, this);
    mythread3_.detach();
    std::thread mythread4_(&Submap::Loop_Detection, this);
    mythread4_.detach();
    std::cout<<"init end"<<std::endl;
}
 
//Since this function is used in the callback function, using .back() here is ok
 void Submap::Mapmerge(){
    pcl::PointCloud<pcl::PointXYZ> cloud_1;  
    pcl::PointCloud<pcl::PointXYZ> temp;

    pcl::fromROSMsg(Submap_list_.back(),cloud_1);

    Eigen::Matrix4f trans_tmp;
    trans_tmp=TransformToMatrix(SubTF_list_.back());
        //-------------------self-made transform "fromROSMsg"------------------------------
        // Eigen::Matrix3f rotation_temp;
        // Eigen::Quaternionf quaternion_temp(
        //     SubTF_list_[i].getRotation().getW(),SubTF_list_[i].getRotation().getX(),SubTF_list_[i].getRotation().getY(),SubTF_list_[i].getRotation().getZ());
        // rotation_temp=quaternion_temp.toRotationMatrix();
        // trans(0,0)=rotation_temp(0,0);
        // trans(0,1)=rotation_temp(0,1);
        // trans(0,2)=rotation_temp(0,2);
        // trans(1,0)=rotation_temp(1,0);
        // trans(1,1)=rotation_temp(1,1);
        // trans(1,2)=rotation_temp(1,2);
        // trans(2,0)=rotation_temp(2,0);
        // trans(2,1)=rotation_temp(2,1);
        // trans(2,2)=rotation_temp(2,2);
        // trans(0,3)=SubTF_list_[i].getOrigin().getX();
        // trans(1,3)=SubTF_list_[i].getOrigin().getY();
        // trans(2,3)=SubTF_list_[i].getOrigin().getZ();
        //---------------------end------------------------
        // std::cout<<trans<<std::endl;

    pcl::transformPointCloud(cloud_1, temp, trans_tmp);
    global_cloud_ = global_cloud_ + temp;
    Globalmap_mutex_.lock();
    pcl::toROSMsg(global_cloud_,Globalmap_);// to do: add other info, like tf, to this msg
    Globalmap_.header.frame_id="/map";
    Globalmap_mutex_.unlock();
    // mythread1_.join();
    std::cout<<"publish Globalmap finish"<<std::endl;
}


void Submap::GMM_training(int novel_frame){ 
    pcl::PointCloud<pcl::PointXYZ> cloud_input;  
    pcl::fromROSMsg(Submap_list_[novel_frame],cloud_input);
    
    //already move "removeNaN" to mapCallback()
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(cloud_input,cloud_input, indices);

    //In the current version, no pointcloud transform here, subgmm_msg contain tf info
    // pcl::PointCloud<pcl::PointXYZ> cloud_global;  
    // Eigen::Matrix4f trans_tmp;
    // trans_tmp=TransformToMatrix(SubTF_list_[novel_frame]);
    // pcl::transformPointCloud(cloud_input, cloud_global, trans_tmp);

    const int size = cloud_input.width*cloud_input.height; //Number of samples
    double *data = new double[size*3];
        for (int i=0; i < size; i++)
{
    data[i*dim_+0] = cloud_input.points[i].x;
    data[i*dim_+1]  = cloud_input.points[i].y;
    data[i*dim_+2]  = cloud_input.points[i].z;
}
//     int cnt=0;
//     for (int i=0; i<size; i++){
//         if(cloud_input.points[i].z>0.1){
//             cnt++;
//         }
        
//     }
//     double *data = new double[cnt*3];
//     std::cout<< size<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<cnt<<std::endl;
//     for (int i=0; i < size; i++)
// {
//     if(cloud_input.points[i].z>0.1){
//     data[i*dim_+0] = cloud_input.points[i].x;
//     data[i*dim_+1]  = cloud_input.points[i].y;
//     data[i*dim_+2]  = cloud_input.points[i].z;
//     }
// }
    // std::cout<<"---------------------------data input finish!"<<std::endl;
    GMM *gmm = new GMM(dim_,cluster_num_); 
    gmm->Train(data,size); //Training GMM
    // std::cout<<gmm->Mean(0)[2]<<"------"<<gmm->Mean(1)[2]<<std::endl;
    SubGMM_list_.push_back(gmm);
    // delete gmm;
}

 // pub GlobalMap
void Submap::Global_Pointcloud_Publisher()
{
    std::cout<<"global_publisher_Start!"<<std::endl;
            while (ros::ok()){
                // std::cout<<"pt_pub"<<std::endl;
                Globalmap_mutex_.lock();
                gobalmap_pub_.publish(Globalmap_);
                Globalmap_mutex_.unlock();
                 ros::Duration(0.1).sleep();
            }
}

void Submap::Global_GMM_Publisher(){
    std::cout<<"GMM_pub_start!"<<std::endl;
    //  static int subgmm_num=0;
     
        while (ros::ok()){
                if(SubGMM_list_.size()>subgmm_num_){
                    //G2G_merging(subgmm_num_);                // change GlobalGMM_ inside this function
                    
                    //generate GlobalGMM message
                    std::vector<float> x_tmp;
                    std::vector<float> y_tmp;
                    std::vector<float> z_tmp;
                    std::vector<float> xvar_tmp;
                    std::vector<float> yvar_tmp;
                    std::vector<float> zvar_tmp;
                    std::vector<float> prior_tmp;
                    for (int i=0; i<SubGMM_list_[subgmm_num_]->GetMixNum();i++){ 
                        x_tmp.push_back(SubGMM_list_[subgmm_num_]->Mean(i)[0]);
                        y_tmp.push_back(SubGMM_list_[subgmm_num_]->Mean(i)[1]);
                        z_tmp.push_back(SubGMM_list_[subgmm_num_]->Mean(i)[2]);
                        xvar_tmp.push_back(SubGMM_list_[subgmm_num_]->Variance(i)[0]);
                        yvar_tmp.push_back(SubGMM_list_[subgmm_num_]->Variance(i)[1]);
                        zvar_tmp.push_back(SubGMM_list_[subgmm_num_]->Variance(i)[2]);
                        prior_tmp.push_back(SubGMM_list_[subgmm_num_]->Prior(i)); 
                    }

                    gmm_msg_.mix_num=SubGMM_list_[subgmm_num_]->GetMixNum();
                    gmm_msg_.prior=prior_tmp;
                    gmm_msg_.x=x_tmp;
                    gmm_msg_.y=y_tmp;
                    gmm_msg_.z=z_tmp;
                    gmm_msg_.x_var=xvar_tmp;
                    gmm_msg_.y_var=yvar_tmp;
                    gmm_msg_.z_var=zvar_tmp;

                    //generate pose in GMM message
                    geometry_msgs::Pose pose_tmp;
                    geometry_msgs::Point position_tmp;
                    geometry_msgs::Quaternion orientation_tmp;
                    position_tmp.x=SubTF_list_[subgmm_num_].getOrigin().getX();
                    position_tmp.y=SubTF_list_[subgmm_num_].getOrigin().getY();
                    position_tmp.z=SubTF_list_[subgmm_num_].getOrigin().getZ();
                    pose_tmp.position=position_tmp;
                    orientation_tmp.x=SubTF_list_[subgmm_num_].getRotation().getX();
                    orientation_tmp.y=SubTF_list_[subgmm_num_].getRotation().getY();
                    orientation_tmp.z=SubTF_list_[subgmm_num_].getRotation().getZ();
                    orientation_tmp.w=SubTF_list_[subgmm_num_].getRotation().getW();
                    pose_tmp.orientation=orientation_tmp;
                    gmm_msg_.pose=pose_tmp;
                    current_time_= ros::Time::now(); 
                    gmm_msg_.header.stamp=current_time_;
                    gmm_msg_.header.frame_id="camera_depth_optical_frame";

                    gmm_list_msg_.data.push_back(gmm_msg_);
                    gmm_pub_.publish(gmm_msg_); 
                    gmm_list_pub_.publish(gmm_list_msg_);           
                    subgmm_num_++;
                    std::cout<<"GMM_global_pub"<<std::endl;
                }else
                {
                    //no update, publish the former msg
                    gmm_pub_.publish(gmm_msg_); //for visualization
                    gmm_list_pub_.publish(gmm_list_msg_);
                }
                 ros::Duration(0.1).sleep();
            }
}

void Submap::Loop_Detection(){
    std::cout<<"Loop_Detection_Start!"<<std::endl;
    while (ros::ok()){
        if(loop_detection()){
            back_end_optimazation();
        }
        ros::Duration(0.1).sleep();
    }
}

bool Submap::loop_detection(){
    
}

void Submap::back_end_optimazation(){

}

void Submap::Submap_GMM_building(){
        // static int submap_num=0;
        std::cout<<"GMM_building_Start!"<<std::endl;
            while (ros::ok()){
                if(Submap_list_.size()>submap_num_){
                    GMM_training(submap_num_);
                    submap_num_++;
                    std::cout<<"GMM_NO:"<<submap_num_<<std::endl;
                }
                ros::Duration(0.1).sleep();
            }
}

void Submap::G2G_merging(int current_frame){
    if (current_frame==0){
        GlobalGMM_=SubGMM_list_[current_frame];
    }
    else
    {
        //calculate W distance & decide novel components
        // Not sure???????????????????It seems like this version of code needs two map has the same number of components
        double* distance=new double(GlobalGMM_->GetMixNum());
        int* label=new int (GlobalGMM_->GetMixNum());
        bool* GMM_add_left=new bool (SubGMM_list_[current_frame]->GetMixNum());
        for (int i=0; i<SubGMM_list_[current_frame]->GetMixNum(); i++){
            GMM_add_left[i]=false;
        }
        int novel_num=0;
        for (int i=0; i<GlobalGMM_->GetMixNum();i++){              
                distance[i]=DBL_MAX;//initialization
        for (int j=0; j<SubGMM_list_[current_frame]->GetMixNum();j++){
            double dis_tmp;
            dis_tmp=CalDistance(SubGMM_list_[current_frame]->GetDimNum(), GlobalGMM_->Mean(i), GlobalGMM_->Variance(i), SubGMM_list_[current_frame]->Mean(j), SubGMM_list_[current_frame]->Variance(j));
            if(dis_tmp<distance[i]){
                distance[i]=dis_tmp;
                label[i]=j;
            }
        }
        if (distance[i]<dis_threshold_){
            novel_num++;
        }
        }
        //merge!
        if (novel_num<SubGMM_list_[current_frame]->GetMixNum()/20){
            // change nothing, so the current Submap brings no new information to the globalmap.
        }
        else //merge GlobalGMM_ & SubGMM_list_[current_frame]
        {
            int MixNum_aftermerge=GlobalGMM_->GetMixNum()+SubGMM_list_[current_frame]->GetMixNum()-novel_num;
            GMM *gmm_tmp = new GMM(GlobalGMM_->GetDimNum(), MixNum_aftermerge); 
            double weights_sum=0;
            int i=0;
            for (i; i<GlobalGMM_->GetMixNum();i++){
                if (distance[i]<dis_threshold_){ // merge similar components
                    for (int j=0; j<GlobalGMM_->GetDimNum();j++){
                        gmm_tmp->Mean(i)[j]=0.5*(GlobalGMM_->Mean(i)[j]+SubGMM_list_[current_frame]->Mean(label[i])[j]);
                        gmm_tmp->Variance(i)[j]=0.25*(SubGMM_list_[current_frame]->Mean(label[i])[j]+GlobalGMM_->Variance(i)[j]);
                    }
                    gmm_tmp->setPrior(i,0.5*(GlobalGMM_->Prior(i)+SubGMM_list_[current_frame]->Prior(label[i])));
                    weights_sum=weights_sum+gmm_tmp->Prior(i);
                    GMM_add_left[label[i]]=true;
                }
                else{
                    gmm_tmp->setMean(i,GlobalGMM_->Mean(i));
                    gmm_tmp->setVariance(i,GlobalGMM_->Variance(i));
                    gmm_tmp->setPrior(i,GlobalGMM_->Prior(i));
                    weights_sum=weights_sum+gmm_tmp->Prior(i);
                }
            }
            //directly add novel components
            for (int j=0; j<SubGMM_list_[current_frame]->GetMixNum(); j++){
            if (GMM_add_left[i]==false){
                    gmm_tmp->setMean(i,SubGMM_list_[current_frame]->Mean(j));
                    gmm_tmp->setVariance(i,SubGMM_list_[current_frame]->Variance(j));
                    gmm_tmp->setPrior(i,SubGMM_list_[current_frame]->Prior(j));
                    weights_sum=weights_sum+gmm_tmp->Prior(i);
            }
            }
            //weights normalization
            for (int j=0; j<MixNum_aftermerge; j++){
                gmm_tmp->setPrior(i,gmm_tmp->Prior(j)/weights_sum);
            }
            GlobalGMM_=gmm_tmp;
        }
        
    }
}


Eigen::Matrix4f  Submap::TransformToMatrix(const tf::StampedTransform& transform) 
{
    Eigen::Matrix4f transform_matrix;
    Eigen::Translation3f tl_btol(
    transform.getOrigin().getX(), 
    transform.getOrigin().getY(), 
    transform.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    return transform_matrix;
}

//Question: Can we make sure that the img&pose are presenting the same frame of map in this way?
void Submap::mapCallback(sensor_msgs::PointCloud2 img){
    if (mapcnt_<4){
        generate_path();
        mapcnt_++;
        std::cout<<mapcnt_<<std::endl;
    }
    else
    {
        generate_path();
        std::cout<<"receive submap"<<std::endl;
        pcl::PointCloud<pcl::PointXYZ> cloud_input;  
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        pcl::fromROSMsg(img,cloud_input);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(cloud_input,cloud_input, indices);
        sor.setInputCloud(cloud_input.makeShared());
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(cloud_input);
        sensor_msgs::PointCloud2 img_tmp;
        pcl::toROSMsg(cloud_input,img_tmp);// to do: add other info, like tf, to this msg
        img_tmp.header=img.header;
        tf::StampedTransform trans_temp;
        Submap_list_.push_back(img_tmp);
        TFlistener_.lookupTransform( "/map","/camera_depth_optical_frame",ros::Time(0), trans_temp);//output is the transform form "/camera_depth_optical_frame" to "/map"
        SubTF_list_.push_back(trans_temp);
        if (SubTF_list_.size()>1){
            //generate relative transition
            geometry_msgs::Pose pose_tmp;
            geometry_msgs::Point position_tmp;
            geometry_msgs::Quaternion orientation_tmp;
            position_tmp.x=SubTF_list_[SubTF_list_.size()].getOrigin().getX()-SubTF_list_[SubTF_list_.size()-1].getOrigin().getX();
            position_tmp.y=SubTF_list_[SubTF_list_.size()].getOrigin().getY()-SubTF_list_[SubTF_list_.size()-1].getOrigin().getY();
            position_tmp.z=SubTF_list_[SubTF_list_.size()].getOrigin().getZ()-SubTF_list_[SubTF_list_.size()-1].getOrigin().getZ();
            pose_tmp.position=position_tmp;
            orientation_tmp.x=SubTF_list_[SubTF_list_.size()].getRotation().getX()-SubTF_list_[SubTF_list_.size()-1].getRotation().getX();
            orientation_tmp.y=SubTF_list_[SubTF_list_.size()].getRotation().getY()-SubTF_list_[SubTF_list_.size()-1].getRotation().getY();
            orientation_tmp.z=SubTF_list_[SubTF_list_.size()].getRotation().getZ()-SubTF_list_[SubTF_list_.size()-1].getRotation().getZ();
            orientation_tmp.w=SubTF_list_[SubTF_list_.size()].getRotation().getW()-SubTF_list_[SubTF_list_.size()-1].getRotation().getW();
            pose_tmp.orientation=orientation_tmp;   
            Trans_origin_.push_back(pose_tmp);
            // std::cout<< pose_tmp<<std::endl;
        }
        std::cout<<"begin merging"<<std::endl;
        // build GMM_Submap here will create huge latency, so we create another thread to do it
        Mapmerge();
        mapcnt_=0;
    }
}


void Submap::generate_path(){
        current_time_= ros::Time::now(); 
        path_.header.stamp=current_time_; 
        tf::StampedTransform trans_temp;
        TFlistener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(4.0));
        TFlistener_.lookupTransform( "/map","/base_link",ros::Time(0), trans_temp);//output is the transform form "/base_link" to "/map"
		geometry_msgs::PoseStamped this_pose_stamped; 
		this_pose_stamped.pose.position.x = trans_temp.getOrigin().getX(); 
		this_pose_stamped.pose.position.y = trans_temp.getOrigin().getY(); 
        this_pose_stamped.pose.position.z = trans_temp.getOrigin().getZ(); 
		this_pose_stamped.pose.orientation.x = trans_temp.getRotation().getX(); 
		this_pose_stamped.pose.orientation.y = trans_temp.getRotation().getY(); 
		this_pose_stamped.pose.orientation.z = trans_temp.getRotation().getZ(); 
		this_pose_stamped.pose.orientation.w = trans_temp.getRotation().getW(); 

		this_pose_stamped.header.stamp=current_time_; 
		this_pose_stamped.header.frame_id="map"; 
		path_.poses.push_back(this_pose_stamped); 

		path_pub_.publish(path_); 

}

// Wasserstein Distance
// https://en.wikipedia.org/wiki/Wasserstein_metric
// https://zhuanlan.zhihu.com/p/58506295
double Submap::CalDistance(int dim,double* mean1, double*var1, double* mean2, double*var2){
    double distance;
    distance=pow((mean1-mean2),2)+trace(dim, Madd(dim, var1,var2,MM(dim, var1,var2)));
    return distance;
}

double Submap::trace(int dim, double* var){
    float trace=0.0;
    for (int i=0; i<dim; i++){
        trace=trace+var[i];
    }
    return trace;
}


//Matrix multi Matrix for Wasserstein Distance
double* Submap::MM(int dim, double* var1, double* var2){
    double* result= var1;
    for (int i=0; i<dim; i++){
        result[i]=sqrt(var2[i])*result[i]*sqrt(var2[i]);
        result[i]=2*sqrt(result[i]);
    }
    return result;
}

//Matrix addMatrix for Wasserstein Distance
double* Submap::Madd(int dim, double* var1, double* var2, double* var3){
double* result= var1;
    for (int i=0; i<dim; i++){
        result[i]=result[i]+var2[i];
        result[i]=result[i]+var3[i];
    }
    return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Submap_building");
//   ros::NodeHandle node;
  ros::NodeHandle nh("~");
  std::cout<<"Robot1 begin"<<std::endl;
  Submap Robot1;
  Robot1.initMap(nh);
  std::cout<<"Robot1 end"<<std::endl;
  ros::spin();
  return 0;
}