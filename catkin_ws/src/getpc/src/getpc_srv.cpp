// listen to point cloud and has a service that returns the point cloud(s) 
// given bin number and object id (e.g. oreo)
// 
// roslaunch getpc getpc.launch
// roslaunch apc_config kinect2_bridge.launch
// rosservice call pointcloud_service
#include "ros/console.h"
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "getpc/GetObjectPointCloud2.h"
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/Marker.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <vector>
#include <mutex>
#include <string>
#include <ctime>
#include <map>
using namespace std;
using sensor_msgs::PointCloud2;
using geometry_msgs::Point;


typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloudXYZRGB;

class PointCloudSource {
public:
    PointCloudSource(ros::NodeHandle& nh, string pointcloud_source_topic): cachedPCmsgFlag_(false), cachedPC2msgFlag_(false) {
        subKinect_ = nh.subscribe(pointcloud_source_topic, 2, &PointCloudSource::kinectCallback, this);
        subKinectPc2_ = nh.subscribe(pointcloud_source_topic, 2, &PointCloudSource::kinectPC2Callback, this);
    }

    void kinectPC2Callback(const PointCloud2::ConstPtr& msg){
        mtx2_.lock();
        frame_id_ = msg->header.frame_id;
        cachedPC2msgFlag_ = true;
        cachedPC2msg_ = *msg;
        mtx2_.unlock();
    }
    
    void kinectCallback(const PCLPointCloudXYZRGB::ConstPtr& msg){
        mtx_.lock();
        cachedPCmsg_ = *msg;
        cachedPCmsgFlag_ = true;
        mtx_.unlock();
    }
    
    PCLPointCloudXYZRGB getPointCloud(){
        mtx_.lock();
        PCLPointCloudXYZRGB ret;
        if(cachedPCmsgFlag_)
            ret = cachedPCmsg_;
        else
            ROS_ERROR("No pointcloud received yet 1, check your kinect");
        mtx_.unlock();
        return ret;
    }
    
    PointCloud2 getPointCloud2(){
        mtx2_.lock();
        PointCloud2 ret;
        if(cachedPC2msgFlag_)
            ret = cachedPC2msg_;
        else
            ROS_ERROR("No pointcloud received yet 2, check your kinect");
        mtx2_.unlock();
        return ret;
    }
    
    string getFrameID() {
        return frame_id_;
    }
private:
    PCLPointCloudXYZRGB cachedPCmsg_;
    PointCloud2 cachedPC2msg_;
    bool cachedPCmsgFlag_, cachedPC2msgFlag_;
    ros::Subscriber subKinect_;
    ros::Subscriber subKinectPc2_;
    mutex mtx_, mtx2_;
    string frame_id_;
};

class ShelfFilter{
public:
    ShelfFilter(ros::NodeHandle& nh, PointCloudSource& pcs, const ros::Publisher& vis_pub): nh_(nh), pcs_(pcs), vis_pub_(vis_pub){
        // init obj_dim_
        obj_dim_["oreo_mega_stuf"]=118;
        obj_dim_["crayola_64_ct"]=145;
        obj_dim_["paper_mate_12_count_mirado_black_warrior"]=192;
        obj_dim_["mead_index_cards"]=128;
        obj_dim_["rolodex_jumbo_pencil_cup"]=135;
        obj_dim_["mark_twain_huckleberry_finn"]=178;
        obj_dim_["laugh_out_loud_joke_book"]=202;
        obj_dim_["sharpie_accent_tank_style_highlighters"]=118;
        obj_dim_["stanley_66_052"]=136;
        obj_dim_["expo_dry_erase_board_eraser"]=130;
        obj_dim_["champion_copper_plus_spark_plug"]=96;
        obj_dim_["feline_greenies_dental_treats"]=180;
        obj_dim_["kong_air_dog_squeakair_tennis_ball"]=190;
        obj_dim_["dr_browns_bottle_brush"]=305;
        obj_dim_["kong_duck_dog_toy"]=115;
        obj_dim_["kong_sitting_frog_dog_toy"]=110;
        obj_dim_["munchkin_white_hot_duck_bath_toy"]=130;
        obj_dim_["mommys_helper_outlet_plugs"]=130;
        obj_dim_["kyjen_squeakin_eggs_plush_puppies"]=116;
        obj_dim_["first_years_take_and_toss_straw_cup"]=160 + 60;   // +60 to include the top pad
        obj_dim_["highland_6539_self_stick_notes"]=116;
        obj_dim_["safety_works_safety_glasses"]=160;
        obj_dim_["genuine_joe_plastic_stir_sticks"]=142;
        obj_dim_["cheezit_big_original"]=227;
        obj_dim_["elmers_washable_no_run_school_glue"]=147;
    }
    // return pointcloud in kinect frame
    bool GetObjectPointCloud2_kinect(getpc::GetObjectPointCloud2::Request &req,
                              getpc::GetObjectPointCloud2::Response &res) 
    {
        clock_t tstart = clock();
        ROS_ERROR("GetObjectPointCloud2");
        
        size_t bin_num = req.bin_num;          // bin_num is of the format like 0-11
        string obj_id = req.obj_id;
        double obj_max_height = obj_dim_[obj_id] / 1000.0 + 0.03;  // mm to meter, 0.03 is shelf lip height
        PCLPointCloudXYZRGB pc_source_pcl = pcs_.getPointCloud();     // in kinect frame
        PCLPointCloudXYZRGB pc_shelf_pcl;
        PointCloud2 pc2_source = pcs_.getPointCloud2();     // in kinect frame
        
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        // pcl::PointCloud<pcl::PointXYZ> cloud;
        // pcl::fromROSMsg (*input, cloud);
        
        
        
        ROS_INFO("GetPointCloud2: %.6lf secs", (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        string kinect_frame = pcs_.getFrameID();
        if (kinect_frame == "")
            return false;
        
        tstart = clock(); 
        int cnt_retry = 0;
        while (nh_.ok() && cnt_retry < 10){
            try{
                tfListener_.waitForTransform(kinect_frame, "/shelf", pc2_source.header.stamp, ros::Duration(1));
                ROS_INFO("before transformPointCloud");
                bool ret = pcl_ros::transformPointCloud (string("/shelf"), pc2_source.header.stamp, 
                                   pc_source_pcl, 
                                   kinect_frame, 
                                   pc_shelf_pcl, 
                                   tfListener_);
                if(!ret){
                    ROS_ERROR("cnt_retry=%d", cnt_retry);
                    pc2_source.header.stamp = ros::Time::now();
                    ros::Duration(0.1).sleep();
                    cnt_retry++;
                    continue;
                }
                ROS_INFO("after transformPointCloud");
                break;
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s, cnt_retry=%d", ex.what(), cnt_retry);
                pc2_source.header.stamp = ros::Time::now();
                ros::Duration(0.1).sleep();
                cnt_retry++;
            }
        }
                               
        ROS_INFO("TransformPointCloud2: %.6lf secs", (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        tstart = clock(); 
        vector<unsigned char> mask(pc_shelf_pcl.height * pc_shelf_pcl.width, 0);  // true means use it
        ROS_INFO("Initmask: %.6lf secs", (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        
        tstart = clock(); 
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/shelf";
        marker.header.stamp = pc2_source.header.stamp;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.003;
        marker.scale.y = 0.003;
        marker.scale.z = 0.003;
        marker.color.a = 0.4;
        marker.color.r = 0;
        marker.color.g = 0.5;
        marker.color.b = 0;
        
        int cnt = 0;
        const size_t npoint = pc_shelf_pcl.points.size();
        for (size_t i = 0; i < npoint; i++){
            pcl::PointXYZRGB& pt = pc_shelf_pcl.points[i];
            if(inside_bin(pt.x, pt.y, pt.z, bin_num, obj_max_height)) {
                mask[i] = 1;
                
                Point p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
                marker.points.push_back(p);
                cnt++;
            }
        }
        ROS_INFO("points.size = %d", cnt);
        ROS_INFO("Compute mask: %.6lf secs", (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        vis_pub_.publish(marker);
        
        res.pc2 = pc2_source;
        res.foreground_mask = mask;
        return true;
    }
    
    // return pointcloud in shelf frame // not used
    bool GetObjectPointCloud2_shelf(getpc::GetObjectPointCloud2::Request &req,
                              getpc::GetObjectPointCloud2::Response &res) 
    {
        clock_t tstart = clock();
        ROS_ERROR("GetObjectPointCloud2_shelf");
        
        size_t bin_num = req.bin_num;          // bin_num is of the format like 0-11
        string obj_id = req.obj_id;            // todo: need multiple object max height
        double obj_max_height = obj_dim_[obj_id] / 1000.0 + 0.03;  // mm to meter, 0.03 is shelf lip height
        
        PointCloud2 pc2_source_ros = pcs_.getPointCloud2();     // in kinect frame
        PCLPointCloudXYZRGB pc_source_pcl;
        pcl::PCLPointCloud2 pc2_source_pcl;
        pcl::fromROSMsg(pc2_source_ros, pc_source_pcl);
        //pcl_conversions::toPCL(pc2_source, pc2_source_pcl);  // this is for return pointcloud in shelf frame
        
        ROS_INFO("GetPointCloud2: %.6lf secs",  (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        // Transform pc_source_pcl to shelf frame
        PCLPointCloudXYZRGB pc_shelf_pcl;
        //pcl::PCLPointCloud2 pc2_shelf_pcl;
        PointCloud2 pc2_shelf_ros;
        string kinect_frame = pcs_.getFrameID();
        if (kinect_frame == "")
            return false;
        
        tstart = clock();  
        
        // bool pcl_ros::transformPointCloud	(	const std::string & 	target_frame,
        // const pcl::PointCloud< PointT > & 	cloud_in,
        // pcl::PointCloud< PointT > & 	cloud_out,
        // const tf::TransformListener & 	tf_listener 
        // )
        
        tfListener_.waitForTransform(kinect_frame, "/shelf", pc2_source_ros.header.stamp, ros::Duration(1));  
        pcl_ros::transformPointCloud (string("/shelf"), pc2_source_ros.header.stamp, 
                               pc_source_pcl, 
                               kinect_frame, 
                               pc_shelf_pcl, 
                               tfListener_);
                       
        pcl_ros::transformPointCloud (string("/shelf"), 
                               pc2_source_ros, 
                               pc2_shelf_ros, 
                               tfListener_);
                               
        ROS_INFO("TransformPointCloud2: %.6lf secs",  (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        tstart = clock(); 
        vector<unsigned char> mask(pc_shelf_pcl.height * pc_shelf_pcl.width, 0);  // true means use it
        ROS_INFO("Initmask: %.6lf secs",  (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        
        tstart = clock(); 
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/shelf";
        marker.header.stamp = pc2_source_ros.header.stamp;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.003;
        marker.scale.y = 0.003;
        marker.scale.z = 0.003;
        marker.color.a = 0.4;
        marker.color.r = 0;
        marker.color.g = 0.5;
        marker.color.b = 0;
        
        int cnt = 0;
        const size_t npoint = pc_shelf_pcl.points.size();
        for (size_t i = 0; i < npoint; i++){
            pcl::PointXYZRGB& pt = pc_shelf_pcl.points[i];
            if(inside_bin(pt.x, pt.y, pt.z, bin_num, obj_max_height)) {
                mask[i] = 1;
                
                Point p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
                marker.points.push_back(p);
                cnt++;
            }
        }
        ROS_INFO("points.size = %d", cnt);
        ROS_INFO("Compute mask: %.6lf secs",  (clock() - tstart) / (double)CLOCKS_PER_SEC);
        
        vis_pub_.publish(marker);
        
        //pc_shelf_pcl
        //pcl_conversions::fromPCL(pc2_source_pcl, pc_shelf_ros);
        //pc_shelf_ros.header.stamp = pc2_source.header.stamp;
        //pc_shelf_ros.header.frame_id = pc2_source.header.frame_id;
        res.pc2 = pc2_shelf_ros;
        res.foreground_mask = mask;
        return true;
    }
    
    
    inline bool inside_bin(const double& x, const double& y, const double& z, const size_t& bin_num, const double& obj_max_height) {
        const double* cnstr = shelf_bin_bound_[bin_num];
        // return (  for kinect
            // x > cnstr[0]+0.02 && x < cnstr[1]-0.025 &&
            // y > cnstr[2]+0.1 && y < cnstr[3]-0.01 &&
            // z > cnstr[4]-0.01 && z < cnstr[5]-0.03 && z < cnstr[4]+obj_max_height);
        const float rightOffsets[] = {0.01,0.01,0.036};
        const float leftOffsets[] = {-0.036,-0.01,-0.01};
        float rightOffsets2[12] = {};
        float leftOffsets2[12] = {};
        leftOffsets2[1] = -0.003;
        leftOffsets2[2] = -0.005;
        leftOffsets2[11] = +0.002;
        rightOffsets2[4] = +0.005;
        rightOffsets2[10] = +0.003;
        
        const int lr_ind = bin_num % 3;
        return (
            x > cnstr[0]+rightOffsets[lr_ind]+rightOffsets2[bin_num] && x < cnstr[1]+leftOffsets[lr_ind]+leftOffsets2[bin_num]  &&   // right, left
            y > cnstr[2]+0.1 && y < cnstr[3]+0.005 &&    // back, front
            z > cnstr[4]-0.01 && z < cnstr[5]-0.03 && z < cnstr[4]+obj_max_height);  // needs to be tuned // bottom, top
    }
    
private:
    //vector< vector<double> > shelf_bin_bound_;
    const double shelf_bin_bound_[12][6] = {{0.1554, 0.42926, 0, 0.42, 1.52393, 1.79063},
             {-0.1494, 0.1494, 0, 0.42, 1.52393, 1.79063},
             {-0.42926, -0.1554, 0, 0.42, 1.52393, 1.79063},
             {0.1554, 0.42926, 0, 0.42, 1.29533, 1.52393},
             {-0.1494, 0.1494, 0, 0.42, 1.29533, 1.52393},
             {-0.42926, -0.1554, 0, 0.42, 1.29533, 1.52393},
             {0.1554, 0.42926, 0, 0.42, 1.06673, 1.29533},
             {-0.1494, 0.1494, 0, 0.42, 1.06673, 1.29533},
             {-0.42926, -0.1554, 0, 0.42, 1.06673, 1.29533},
             {0.1554, 0.42926, 0, 0.42, 0.800027, 1.06673},
             {-0.1494, 0.1494, 0, 0.42, 0.800027, 1.06673},
             {-0.42926, -0.1554, 0, 0.42, 0.800027, 1.06673}};
    PointCloudSource& pcs_;
    tf::TransformListener tfListener_;
    const ros::Publisher& vis_pub_;
    map<string, double > obj_dim_;
    ros::NodeHandle& nh_;
};

//vis_pub

int main(int argc, char **argv){
    ros::init(argc, argv, "getpc_server", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    string input_topic;
    
    if (!nh.getParam("input_topic", input_topic))
    {
        ROS_ERROR("Get param input_topic error");
        return 1;
    }
    PointCloudSource pcs(nh, input_topic);
    
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    ShelfFilter sf(nh, pcs, vis_pub);
    
    
    // startPointCloudService:
    
    ros::ServiceServer service = nh.advertiseService("get_filtered_pointcloud2_service", &ShelfFilter::GetObjectPointCloud2_kinect, &sf);
    
    ros::spin();
    
    // # self testing
    // selftest = False
    // if selftest == True:
        // rospy.sleep(1)
        // req = GetObjectPointCloudRequest()
        // req.bin_num = 5
        // with Timer('getObjectPointCloud'):
            // ret = getObjectPointCloud(req)
        // 
        // req = GetObjectPointCloud2Request()
        // req.bin_num = 5
        // with Timer('getObjectPointCloud2'):
            // ret = getObjectPointCloud2(req)
    // 
    // createService = True
    // if createService:
        // startPointCloudService()
        // rospy.spin()
}


