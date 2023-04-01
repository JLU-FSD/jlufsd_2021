#pragma once
#include <ros/ros.h>
#include <pose.h>
#include "std_msgs/Int32.h"
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <proj_api.h>
#include <pcl/filters/uniform_sampling.h>        //pcl1.8
// #include <pcl/keypoints/uniform_sampling.h>         //pcl1.7
#include <opencv/cv.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// #include <lidar_perception/sensor_gps.h>
// #include <lidar_perception/output_point.h>
// #include <lidar_perception/center_point.h>

#include "auto_msgs/Map.h"
#include "auto_msgs/Cone.h"
#include <auto_msgs/sensor_gps.h>
#include <auto_msgs/BoundingBoxes.h>
#include <auto_msgs/BoundingBox.h>
#include <geometry_msgs/Point.h>
#include <auto_msgs/Cones.h>
// #include <std_msgs/String.h>
// #include <auto_msgs/Cone.h>


#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <ctime>


using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::JacobiSVD;

using std::sin;
using std::cos;
using std::atan2;



typedef pcl::PointXYZI ip;
typedef pcl::PointCloud<pcl::PointXYZI> cloud; 

bool systemInited=false;
int systemnumber=0;
int systemDelay=100;
double value_lidar_to_imu=0.85;

bool point_cmp(ip a,ip b){
    return a.z<b.z;
}
cloud::Ptr seeds_pc(new cloud());
cloud::Ptr ground_pc(new cloud());
cloud::Ptr not_ground_pc(new cloud());
//  int id(1);

bool systime=false;

//定义参数
// float lidar_to_camera_distance_x=1.45;
// float lidar_to_camera_distance_z=0.67638746;
// std::string r="red_cone";
// std::string b="blue_cone";
// std::string y="yellow_cone";
// std::string yh="yellow_cone_high";



class LidarPerception{
    private:
    //ros::Subscriber sub_imu_info;
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_camera_distance_;
    ros::Subscriber sub_GPS;
    ros::Publisher pub_cluster_;
    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_ground_points_;
    ros::Publisher pub_no_ground_points_;
    ros::Publisher pub_marker_array;
    ros::Publisher pub_center_point;

    //void IMUInfo_callback(const sensor_msgs::Imu::ConstPtr& imuIn);
    void point_callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_ptr);
    void gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_data);
    void camera_centerpoint_callback(const auto_msgs::ConesConstPtr& boxes);
    void estimate_plane_(void);
    void extract_initial_seeds_(const cloud& p_sored);
    void cluster_callback(const sensor_msgs::PointCloud2ConstPtr& in_points);
    void getMarkerArray(const std::vector<cloud>& cluster_cloud_v,visualization_msgs::MarkerArray& marker_array);
    auto_msgs::sensor_gps LH2XY(const auto_msgs::sensor_gps &point);
    //void motionCompensation(const sensor_msgs::PointCloud2ConstPtr& point);

    //void AccumulateIMUShift();
    
    // void ShiftToStartIMU(float pointTime);
    // void VeloToStartIMU();
    // void TransformToStartIMU(ip *p)

    

    int num_seg_;
    int num_iter_;
    int num_plr_;
    double th_seeds_;
    double th_dist_;
    
    float d_;
    MatrixXf normal_;
    float th_dist_d_;
    float cluster_;
    int minclustersize_;
    int maxclustersize_;
    sensor_msgs::NavSatFix gps_data; 
    Pose pose;
   
    auto_msgs::sensor_gps sensor_gps;
    std::vector<auto_msgs::sensor_gps> gps_v;
    //std::vector<lidar_perception::center_point> center_points_v;
    // lidar_perception::center_point center_points;
    auto_msgs::Map center_points;


    std::vector<auto_msgs::Cone> boundingbox_v;

    float lidar_to_camera_distance_x;
    float lidar_to_camera_distance_z,error_distance;
    std::string r,b,y,yh;
 


    public:
    LidarPerception(ros::NodeHandle &nh);
    ~LidarPerception();
    void Spin();

};

LidarPerception::LidarPerception(ros::NodeHandle &nh){
    // sub_GPS=nh.subscribe("/gps",1,gps_callback);
    // while(ros::ok()){ 
        //舍弃前二十个点以及gps
        // if(gps_data->x==0 || gps_data->y==0){
        //     ros::spinOnce();
        //     r.sleep();
        //     continue;
        // }
        // else{
            //获取参数
            nh.param<float>("lidar_to_camera_distance_x",lidar_to_camera_distance_x,1.45);
            nh.param<float>("lidar_to_camera_distance_z",lidar_to_camera_distance_z,0.67638746);
            nh.param<float>("error_distance",error_distance,0.1);
            nh.param<std::string>("r",r,"red_cone");
            nh.param<std::string>("b",b,"blue_cone");   
            nh.param<std::string>("y",y,"yellow_cone");
            nh.param<std::string>("yh",yh,"yellow_cone_high");          
            
             

            std::cerr<<"开始进行感知\n";
            sub_GPS=nh.subscribe("/gps",1,&LidarPerception::gps_callback,this);
            //sub_imu_info=nh.subscribee("/imu/data",50,&LidarPerception::IMUInfo_callback,this)
            sub_point_cloud_=nh.subscribe("/velodyne_points",10,&LidarPerception::point_callback,this);
            sub_camera_distance_=nh.subscribe("/cones_info",1,&LidarPerception::camera_centerpoint_callback,this);
            
            pub_filtered_points_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_points",10);
            //sub_cess_=nh.subscribe("/filtered_points",10,&LidarPerception::removal_callback,this);
            pub_ground_points_=nh.advertise<sensor_msgs::PointCloud2>("/ground_points",10);
            pub_no_ground_points_=nh.advertise<sensor_msgs::PointCloud2>("/no_ground_points",10);
            //sub_clouster_=nh.subscribe("/no_ground_points",10,&LidarPerception::cluster_callback,this);
            pub_cluster_=nh.advertise<sensor_msgs::PointCloud2>("cluster_points",10);
            pub_marker_array=nh.advertise<visualization_msgs::MarkerArray>("MarkerArray",10);

            pub_center_point=nh.advertise<auto_msgs::Map>("center_points",10);

            //较多时，应写初始化函数
            num_seg_=1;
            num_iter_=6;
            num_plr_=20;
            th_seeds_=0.2;
            th_dist_=0.04;
            
            float cluster_=0.02;
            int minclustersize_=1;
            int maxclustersize_=1000;
            ros::spin();
            // r.sleep();
    //     }
    // }
    
}

LidarPerception::~LidarPerception(){}
//GPS——data handele
auto_msgs::sensor_gps LidarPerception::LH2XY(const auto_msgs::sensor_gps &point){
            auto_msgs::sensor_gps nowpoint;
            
            const char* wgs84="+proj=latlong +ellps=WGS84 +datum=NAD83";
            const char* beijng54="+proj=tmerc +ellps=krass +x_0=500000.0000000000 +y_0=0 +towgs84=77,-88,-99,0,0,0,0 +lon_0=111e +lat_0=0.0000000000 +units=m +no_defs";
            projPJ pj_wgs84=pj_init_plus(wgs84);
            projPJ pj_bj54=pj_init_plus(beijng54);
            
            double x =point.lon;
            double y =point.lat;
            double z =0;
            //std::cerr<<"cerror-890  "<<x<<std::endl;
            //std::cerr<<"cerror-890  "<<y<<std::endl;
            x *=DEG_TO_RAD;
            y *=DEG_TO_RAD;
            //std::cerr<<"cerror-790  "<<x<<std::endl;
            //std::cerr<<"cerror-790  "<<y<<std::endl;
            pj_transform(pj_wgs84,pj_bj54,1,1,&x,&y,NULL);
            //std::cerr<<"cerror-690  "<<x<<std::endl;
            //std::cerr<<"cerror-690  "<<y<<std::endl;
            nowpoint.x =x;
            nowpoint.y =y;
            //std::cerr<<"cerror-590  "<<nowpoint.x<<std::endl;
            //std::cerr<<"cerror-590  "<<nowpoint.y<<std::endl;
            return nowpoint;
    }


void LidarPerception::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps_data){
    sensor_gps.lon=gps_data->longitude;
    sensor_gps.lat=gps_data->latitude;
    auto_msgs::sensor_gps gps_;
    gps_=LH2XY(sensor_gps);
    sensor_gps.x=gps_.x;
    sensor_gps.y=gps_.y;
    std::cerr<<"gps数据\t"<<sensor_gps.x<<"****"<<sensor_gps.y<<std::endl;
}

void LidarPerception::camera_centerpoint_callback(const auto_msgs::ConesConstPtr& boxes){
    boundingbox_v.clear();
    for(auto a:boxes->cones)
    {
        std::cerr<<a.color<<std::endl;
        boundingbox_v.push_back(a);
    }
}

void LidarPerception::getMarkerArray(const std::vector<cloud>& cluster_cloud_v,visualization_msgs::MarkerArray& marker_array){
    
    std::cerr<<"标记框\n";
    
    if(cluster_cloud_v.empty()){
        std::cerr<<"没有产生聚类"<<std::endl;
        return;
    }
    marker_array.markers.clear();
    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id="/velodyne";
    bbox_marker.header.stamp=ros::Time::now();
    bbox_marker.ns="吉大吉速车队";
    bbox_marker.color.r=0.0f;
    bbox_marker.color.g=1.0f;
    bbox_marker.color.b=0.0f;
    bbox_marker.color.a=0.3;
    bbox_marker.lifetime=ros::Duration();
    bbox_marker.frame_locked=true;
    bbox_marker.type=visualization_msgs::Marker::CUBE;
    bbox_marker.action=visualization_msgs::Marker::ADD;
    int marker_id=0;

    for(size_t i=0;i<cluster_cloud_v.size();++i){
        pcl::PointXYZI min_pt,max_pt;
        pcl::getMinMax3D(cluster_cloud_v[i],min_pt,max_pt);
        bbox_marker.id=marker_id;
        
        // std::cerr<<"开始对物体加框\t"<<i<<std::endl;
        // std::cerr<<"点的数目"<<cluster_cloud_v.size()<<std::endl;

        
        // std::cerr<<"x-最大最小点"<<max_pt.x<<"....."<<min_pt.x<<std::endl;
        // std::cerr<<"y-最大最小点"<<max_pt.y<<"....."<<min_pt.y<<std::endl;
        // std::cerr<<"z-最大最小点"<<max_pt.z<<"....."<<min_pt.z<<std::endl;

        bbox_marker.pose.position.x=(max_pt.x+min_pt.x)/2;
        bbox_marker.pose.position.y=(max_pt.y+min_pt.y)/2;
        bbox_marker.pose.position.z=(max_pt.z+min_pt.z)/2;
        // std::cerr<<bbox_marker.pose.position.x<<"...."<<bbox_marker.pose.position.y<<"...."<<bbox_marker.pose.position.z<<std::endl;
        // std::cerr<<"下一步"<<std::endl;
        bbox_marker.scale.x=max_pt.x-min_pt.x;
        bbox_marker.scale.y=max_pt.y-min_pt.y;
        bbox_marker.scale.z=max_pt.z-min_pt.z;

        // std::cerr<<"x-聚类的大小"<<bbox_marker.scale.x<<std::endl;
        // std::cerr<<"y-聚类的大小"<<bbox_marker.scale.y<<std::endl;
        // std::cerr<<"z-聚类的大小"<<bbox_marker.scale.z<<std::endl;

        marker_array.markers.push_back(bbox_marker);
        // std::cerr<<bbox_marker.scale.x<<"....."<<bbox_marker.scale.y<<"....."<<bbox_marker.scale.z<<std::endl;
        ++marker_id;
    }

    unsigned int max_marker_size_=0;
    if (marker_array.markers.size()>max_marker_size_){
        max_marker_size_=marker_array.markers.size();
    }
    // std::cerr<<"wutiuang shumu "<<max_marker_size_<<std::endl;
    for(size_t i=marker_id;i<max_marker_size_;++i){
        bbox_marker.id=i;
        // std::cerr<<"id"<<bbox_marker.id<<std::endl;
        bbox_marker.pose.position.x=0;
        bbox_marker.pose.position.y=0;
        bbox_marker.pose.position.z=0;
        bbox_marker.scale.x=0;
        bbox_marker.scale.y=0;
        bbox_marker.scale.z=0;
        marker_array.markers.push_back(bbox_marker);
        ++marker_id;
    }
}
// void LidarPercepion::gps_callback(const sensor_msgs::NavSatFixConstPtr& gps){
//     gps_data.
// }

//自定义gps消息类型包含xy


void LidarPerception::estimate_plane_(){
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*ground_pc,cov,pc_mean);
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    normal_=(svd.matrixU().col(2));
    //std::cerr<<"法向量"<<normal_<<std::endl;
    Eigen::Vector3f seeds_mean_=pc_mean.head<3>();
    d_=-(normal_.transpose()*seeds_mean_)(0,0);
   // std::cerr<<"平面距离"<<d_<<std::endl;
    th_dist_d_=th_dist_-d_;
    //std::cerr<<"yuzhi"<<th_dist_d_<<std::endl;
}

void LidarPerception::extract_initial_seeds_(const cloud& p_sored){
    int a(0);
    double sum(0);
    for(size_t i=0;i<p_sored.points.size()&&a<num_plr_;i++){    //num_plr=20
        sum+=p_sored.points[i].z;
        a++;
    }
    double seeds_means=(a!=0)?sum/a:0; //三元运算符
    //std::cerr<<"种子点平均高度"<<seeds_means<<std::endl;
    for(size_t i=0;i<p_sored.points.size();i++){
        if(p_sored.points[i].z<seeds_means+th_seeds_){  //  当锥桶的点数较少  or   地面点云较多时    最低种子的平均高度+0.2
            seeds_pc->points.push_back(p_sored.points[i]);
        }
        else break;
    }
}

void LidarPerception::point_callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud){
    // std::cerr<<"执行点云处理\n";
    // if(sensor_gps.x==0 || sensor_gps.y==0)
    // {
    //     ROS_INFO("没有传入GPSIMU数据");
    //     return;
    // }
    //延迟10s
    if(!systemInited)
    {
        systemnumber++;
        if(systemnumber>=systemDelay){
            systemInited=true;
        }
        return;
    }

    gps_v.push_back(sensor_gps);
    //ROS_INFO("全局坐标系起始点，x=%d;y=%d",gps_v[0].x,gps_v[0].y);

    //锥桶数清零
    center_points.blue_cone.clear();
    center_points.red_cone.clear();
    center_points.yellow_cone.clear();
    center_points.yellow_high_cone.clear();

    cloud::Ptr current_cloud_ptr(new cloud);
    cloud::Ptr passthrough_cloud_ptr(new cloud);
    cloud::Ptr passthrough_cloud_ptr1(new cloud);
    cloud::Ptr passthrough_cloud_ptr2(new cloud);
    cloud::Ptr uniformsampling_cloud_ptr(new cloud);
    cloud::Ptr filtered_points_ptr(new cloud);

    pcl::fromROSMsg(*input_cloud,*current_cloud_ptr);
    pcl::PassThrough<ip> pt;
    pt.setInputCloud(current_cloud_ptr);
    pt.setFilterFieldName("y");
    pt.setFilterLimits(-2.5,2.5);
    pt.setKeepOrganized(true);
    pt.setFilterLimitsNegative(false);
    pt.filter(*passthrough_cloud_ptr1);

    pcl::PassThrough<ip> pt2;
    pt2.setInputCloud(passthrough_cloud_ptr1);
    pt2.setFilterFieldName("x");
    pt2.setFilterLimits(0,10);
    pt2.setKeepOrganized(true);
    pt2.setFilterLimitsNegative(false);
    pt2.filter(*passthrough_cloud_ptr2);

    pcl::PassThrough<ip> pt3;
    pt3.setInputCloud(passthrough_cloud_ptr2);
    pt3.setFilterFieldName("z");
    pt3.setFilterLimits(-2,0.8);
    pt3.setKeepOrganized(true);
    pt3.setFilterLimitsNegative(false);
    pt3.filter(*passthrough_cloud_ptr);

    //均匀采样
    pcl::UniformSampling<ip> us;
    us.setInputCloud(passthrough_cloud_ptr);
    us.setRadiusSearch(0.01f);
    us.filter(*uniformsampling_cloud_ptr);


    //去除离群点
    pcl::StatisticalOutlierRemoval<ip> sta;
    sta.setInputCloud(uniformsampling_cloud_ptr);
    //修改参数
    sta.setMeanK(50);
    sta.setStddevMulThresh(1);
    sta.filter(*filtered_points_ptr);


// 去地面
    cloud cloud_in;
    //pcl::fromROSMsg(*input_cloud_ptr,cloud_in);
    cloud_in=*filtered_points_ptr;
    cloud cloud_org(cloud_in);
    sort(cloud_in.points.begin(),cloud_in.points.end(), point_cmp);
    /*
    cloud::iterator it=cloud_in.points.begin();
    for(size_t i=0,i<cloud_in.points.size(),i++){
        if(cloud_in.point[i].z<-1.5*sensor_height_)
        it++;
        else break;
    }
    cloud_in.points.erase(cloud_in.points.begin(),it);
    */
    extract_initial_seeds_(cloud_in);//地面种子点
    ground_pc=seeds_pc;

    for(int i=0;i<num_iter_;i++){
        estimate_plane_();
        ground_pc->clear();
        not_ground_pc->clear();
        MatrixXf points(cloud_org.points.size(),3);
        int j(0);
        for(auto p:cloud_org.points){
            points.row(j++)<< p.x,p.y,p.z;
        }
        VectorXf result= points*normal_;
        for(int q=0;q<points.rows();q++){
            if(result[q]<th_dist_d_){
                ground_pc->points.push_back(cloud_org[q]);
            }else{
                not_ground_pc->points.push_back(cloud_org[q]);
            }
        }
    }
    sensor_msgs::PointCloud2 ground_pcg,no_ground_pcg;
    pcl::toROSMsg(*ground_pc,ground_pcg);
    pcl::toROSMsg(*not_ground_pc,no_ground_pcg);
    ground_pcg.header.stamp=input_cloud->header.stamp;
    ground_pcg.header.frame_id="/velodyne";
    no_ground_pcg.header.stamp=input_cloud->header.stamp;
    no_ground_pcg.header.frame_id="/velodyne";
    pub_ground_points_.publish(ground_pcg);
    pub_no_ground_points_.publish(no_ground_pcg);
//去地面

//聚类
    cloud::Ptr cluster_orig_ptr(new cloud);
    cloud::Ptr cluster_points_all(new cloud);
    //pcl::fromROSMsg(*in_points,*cluster_orig_ptr);
    cluster_orig_ptr=not_ground_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_marker_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cluster_cloud_v;
    
    pcl::search::KdTree<ip>::Ptr tree(new pcl::search::KdTree<ip>);
    tree->setInputCloud(cluster_orig_ptr);
    
    pcl::EuclideanClusterExtraction<ip> ec;
    ec.setClusterTolerance(0.2);   //搜索半径，
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(350);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cluster_orig_ptr);
    ec.extract(cluster_indices);
    int oo=0;

    for(std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();it!=cluster_indices.end();++it){
        cloud::Ptr cluster_points_ptr(new cloud);
        for(std::vector<int>::const_iterator sn=it->indices.begin();sn!=it->indices.end();++sn)
        {   cluster_points_ptr->points.push_back(cluster_orig_ptr->points[*sn]);
            //std::cerr<<"22222"<<cluster_points_ptr<<std::endl;
            // pcl::PointXYZI p;
            // p.x=cluster_orig_ptr->points[*sn].x;
            // p.y=cluster_orig_ptr->points[*sn].y;
            // p.z=cluster_orig_ptr->points[*sn].z;
            // cluster_points_ptr->points.push_back(p);
           
        }
        pcl::PointXYZI min_point,max_point;

        auto_msgs::Cone  output_point;
        auto_msgs::Cone  output_point_cone;

        pcl::getMinMax3D(*cluster_points_ptr,min_point,max_point);
       //雷达坐标下中心点
        output_point.position.x=(min_point.x+max_point.x)/2+value_lidar_to_imu;
        output_point.position.y=(min_point.y+max_point.y)/2;
        output_point.position.z=(min_point.z+max_point.z)/2;
        output_point.distance=sqrt(output_point.position.x*output_point.position.x+
                                output_point.position.y*output_point.position.y+
                                output_point.position.z*output_point.position.z);
        std::cerr<<"lidar_to_cone_distance:"<<output_point.distance<<std::endl;
        //雷达转xiangji
        //std::cerr<<"ymax_param:"<<lidar_to_camera_distance_x<<std::endl;
        geometry_msgs::Point lidar_in_camera_point;
        lidar_in_camera_point.x=output_point.position.x+lidar_to_camera_distance_x;
        lidar_in_camera_point.y=output_point.position.y;
        lidar_in_camera_point.z=output_point.position.z-lidar_to_camera_distance_z;
        double distance=sqrt(lidar_in_camera_point.x*lidar_in_camera_point.x+
                            lidar_in_camera_point.y*lidar_in_camera_point.y+
                            lidar_in_camera_point.z*lidar_in_camera_point.z);
        //添加点的颜色
        std::cerr<<"lidar_distance:\t"<<distance<<std::endl;
        for(auto m:boundingbox_v)
        {
            double dis=distance-m.distance;
            //std::cerr<<"lidar_to_camera_distance:\t"<<dis<<std::endl;
            if(fabs(dis<error_distance))
            {
                output_point.color=m.color;
            }
            // else if(output_point.position.y>0)
            // {
            //     output_point.color=r;
            // }
            // else if (output_point.position.y,0)
            // {
            //     output_point.color=b;
            // }
        }
       
        

        //转换到大地坐标系
        float diff_x=sensor_gps.x-gps_v[0].x;
        float diff_y=sensor_gps.y-gps_v[0].y;
        //float diff_z=sensor_gps.z-gps_v[0].z;

        float angle=atan2(diff_y,diff_x)*180/M_PI;
        
        output_point_cone.position.x =cos(angle)*output_point.position.x-sin(angle)*(output_point.position.y)+diff_x;
        output_point_cone.position.y =sin(angle)*output_point.position.x+cos(angle)*(output_point.position.y)+diff_y;
        output_point_cone.position.z =output_point.position.z;
        //output_point.z += output_point.x+diff_z;

        //std::cerr<<"全局坐标点, x=  "<<output_point_cone.position.x <<",  y=  "<<output_point_cone.position.y<<std::endl;


        //添加聚类的中心点，红  蓝  锥桶
        output_point_cone.color=output_point.color;
        std::cerr<<"cone_color:\t"<<output_point_cone.color<<std::endl;
        if(output_point_cone.color==r)
        {
            center_points.red_cone.push_back(output_point_cone);
            std::cerr<<"red_cone_num:\t"<<center_points.red_cone.size()<<"\n";
        }
        else if(output_point_cone.color==b)
        {
            center_points.blue_cone.push_back(output_point_cone);
            std::cerr<<"blue_cone_num:\t"<<center_points.blue_cone.size()<<"\n";
        }
        else if(output_point_cone.color==y)
        {
            center_points.yellow_cone.push_back(output_point_cone);
            std::cerr<<"yellow_cone_num:\t"<<center_points.yellow_cone.size()<<"\n";
        }
        else if(output_point_cone.color==yh)
        {
            center_points.yellow_high_cone.push_back(output_point_cone);
            std::cerr<<"yellow_high_cone_num:\t"<<center_points.yellow_high_cone.size()<<"\n";
        }


        std::cerr<<"\n"<<"\n"<<std::endl;
 




        ++oo;
        //std::cerr<<"聚类个数"<<oo<<std::endl;
        //std::cerr<<"the number of cluster"<<oo<<std::endl;
        cluster_cloud_v.push_back(*cluster_points_ptr);
        //std::cerr<<cluster_cloud_v.size()<<std::endl;
        cluster_points_ptr->width=cluster_points_ptr->points.size();
        cluster_points_ptr->height=1;
        cluster_points_ptr->is_dense=true;
        //std::cerr<<"11"<<std::endl;
        *cluster_points_all+=*cluster_points_ptr;
        //std::cerr<<cluster_points_all->points.size()<<std::endl;

        visualization_msgs::MarkerArray marker_array;
        getMarkerArray(cluster_cloud_v,marker_array);
        // getMarkerArray(*cluster_points_ptr,marker_array);
        pub_marker_array.publish(marker_array);
        
    }


    //公布中线点
    pub_center_point.publish(center_points);

    sensor_msgs::PointCloud2 cluster_points;
    pcl::toROSMsg(*cluster_points_all,cluster_points);
    cluster_points.header.stamp=input_cloud->header.stamp;
    cluster_points.header.frame_id="/velodyne";
    pub_cluster_.publish(cluster_points);

    //聚类

    //发布话题，去除订阅 ，不好处理
    sensor_msgs::PointCloud2 pub_to_pc;
    pcl::toROSMsg(*filtered_points_ptr,pub_to_pc);
    pub_to_pc.header=input_cloud->header;
    pub_filtered_points_.publish(pub_to_pc);

    
}
