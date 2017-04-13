#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <ros/time.h>

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include <mavlink.h>

using namespace std;

/*
a  34.015182 113.699765
b  34.020344 113.693130
c  34.031795 113.700876
d  34.023093 113.717725
*/

double lat_0 = 0.5* (34.015182 + 34.031795);
double lon_0 = 0.5* (113.693130 + 113.717725); 
double x_0 = -40;
double y_0 = -40;
double z_0 = 0.15;

/*
double lon_0 =  37.1234568;
double lat_0 =  28.1397845;
//double z_0 =  50;
double x_0 = 200;
double y_0 = 300;
double z_0 = 50;
*/

double pi = 3.1415926;
double R_1 = 111000; // meter
double Dis,lon_1,lat_1,z_1,angle;

struct
{
	double x;
	double y;
	double z;
    double q0;
    double q1;
    double q2;
    double q3;
}tag_pos,uav_pos,car_pos;
struct qauternion
{
  float x;
  float y;
  float z;
  float w;
}qua;
struct euler
{
  float roll;
  float pitch;
  float yaw;
}eu;


void subtag(const geometry_msgs::PoseArray& msg)
{   

    tag_pos.x = msg.poses[0].position.x;
    tag_pos.y = msg.poses[0].position.y;
    tag_pos.z = msg.poses[0].position.z;
    tag_pos.q0 = msg.poses[0].orientation.x ;
    tag_pos.q1 = msg.poses[0].orientation.y ;
    tag_pos.q2 = msg.poses[0].orientation.z ;
    tag_pos.q3 = msg.poses[0].orientation.w ;
}

void subuav(const geometry_msgs::PoseStamped& msg)
{
    uav_pos.x = msg.pose.position.x;
    uav_pos.y = msg.pose.position.y;
    uav_pos.z = msg.pose.position.z;
    uav_pos.q0 = msg.pose.orientation.x ;
    uav_pos.q1 = msg.pose.orientation.y ;
    uav_pos.q2 = msg.pose.orientation.z ;
    uav_pos.q3 = msg.pose.orientation.w ;
    
}
void subcar(const nav_msgs::Odometry& msg)
{
    car_pos.x = msg.pose.pose.position.x;
    car_pos.y = msg.pose.pose.position.y;
    car_pos.z = msg.pose.pose.position.z;
    car_pos.q0 = msg.pose.pose.orientation.x ;
    car_pos.q1 = msg.pose.pose.orientation.y ;
    car_pos.q2 = msg.pose.pose.orientation.z ;
    car_pos.q3 = msg.pose.pose.orientation.w ;

}
//  calculate long and lat

void cal_lon_lat(double lon,double lat, double x,double y,double z)
{
     Dis = sqrt(x*x+y*y+z*z);
     angle = asin(y/Dis);
    

     lon_1 = lon + Dis*sin(angle) / (R_1*cos(lat*pi/180));
     lat_1 = lat + Dis*cos(angle) / R_1;
     z_1 = z;
}


//  calculate Distance

double TwoPointsHorDistance(double lon1,double lat1, double lon2,double lat2)
{
    double temp;
    double temp2;
    double Cclat,Sclat,Calat,Salat,Ca_c;
    double R = 6371004;

    Ca_c = cos((lon2 - lon1)*pi/180);
    Cclat = cos(lat1*pi/180);
    Sclat = sin(lat1*pi/180);
    Calat = cos(lat2*pi/180);
    Salat = sin(lat2*pi/180);
    temp2=Calat*Cclat*Ca_c + Salat*Sclat;
    if(temp2<-1)
        temp2=-1;
    if(temp2>1)
        temp2=1;
    temp = ( R * acos(temp2));
    return temp;
}

void SendHeartBeat(int ID, serial::Serial *my_serial)
{
    mavlink_heartbeat_t cmd;

    mavlink_message_t outmsg;
    mavlink_msg_heartbeat_encode(ID, 0, &outmsg, &cmd);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &outmsg);
    my_serial->write(buffer, len);
}

void SendPosition(int ID, serial::Serial *my_serial, double positionLon, double positionLat, double positionAlt)
{
    mavlink_global_position_int_t cmd;
    cmd.lon=positionLon*1E7;
    cmd.lat=positionLat*1E7;
    cmd.relative_alt =positionAlt*1000;
    cmd.alt=0;

    mavlink_message_t outmsg;
    mavlink_msg_global_position_int_encode(ID, 0, &outmsg, &cmd);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &outmsg);
    my_serial->write(buffer, len);
}

void SendAttitude(int ID, serial::Serial *my_serial, double attitudeRoll, double attitudePitch, double attitudeYaw)
{
    mavlink_attitude_t cmd;
    cmd.roll=attitudeRoll;
    cmd.pitch=attitudePitch;
    cmd.yaw=attitudeYaw;

    mavlink_message_t outmsg;
    mavlink_msg_attitude_encode(ID, 0, &outmsg, &cmd);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &outmsg);
    my_serial->write(buffer, len);
}


void serial_send(string port, unsigned long baud, double position[3], double attitude[3])
{



  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
   cout << " Yes." << endl;
  else
   cout << " No." << endl;

/*
  for(int i=0;i<3;i++)
  {
    size_t bytes_wrote = my_serial.write(position);
  }
*/
  static int seq = 0;

  SendHeartBeat(99, &my_serial);
  SendPosition(99, &my_serial, position[0], position[1], position[2]);
  SendAttitude(99, &my_serial, attitude[0], attitude[1], attitude[2]);

  return;
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    
    geometry_msgs::PoseStamped pose_Index;
   // std_msgs::Float64MultiArray data_view;
    //data_view.data.resize(6);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_calc", 10);
    //ros::Publisher pub_data = nh.advertise<std_msgs::Float64MultiArray>("/data_view", 10);
    ros::Subscriber sub_tag_pose = nh.subscribe("/tag_detections_pose", 10, &subtag);
    ros::Subscriber sub_uav_pose = nh.subscribe("/ardrone/ground_truth/pose", 10, &subuav);
    ros::Subscriber sub_car_pose = nh.subscribe("/odom", 10, &subcar);

    static tf::TransformBroadcaster br,cr;
    static tf::TransformListener listener;
    tf::Transform transform,transformcr;
    tf::StampedTransform transformer;
    tf::Quaternion q,p;
    q.setRPY(3.14159, 0, -1.57075); // 4*1 body to camera
    p.setRPY(-3.14159, 0, -1.57075); 

   cal_lon_lat(lon_0,lat_0,x_0,y_0,z_0);
   double lon_world_orgin = lon_1;
   double lat_world_orgin = lat_1;
   double height_world_orgin = z_1;



  /*
  a  34.015182 113.699765
  b  34.020344 113.693130
  c  34.031795 113.700876
  d  34.023093 113.717725
  */

 /* //  Test 
  double lat_a = 34.031795;
  double lon_a = 113.693130; 
  double lat_b = 34.031795;
  double lon_b = 113.717725; 
  double lat_c = 34.015182;
  double lon_c = 113.693130; 
 
  double D_ab = TwoPointsHorDistance(lon_a,lat_a, lon_b,lat_b);
  double D_ac = TwoPointsHorDistance(lon_a,lat_a, lon_c,lat_c); 
  ROS_INFO("l1=%.8f,l1=%.8f",D_0 - D_1);
*/
   double D_0 = TwoPointsHorDistance(lon_0,lat_0, lon_world_orgin,lat_world_orgin);
   double D_1 = sqrt(x_0*x_0 + y_0*y_0);
   ROS_INFO("err_dis=%.8f",D_0 - D_1);

   
   

   // q.setRPY(0, -1.57075,0);
    ros::Rate loop_rate(10.0);
   	while(ros::ok())
   	{
   		//////ros::spinOnce();

    //  UAV body2world pose
      
  //     ros::Subscriber sub_car_pose = nh.subscribe("/odom", 10, &subcar);
      
    // Tag2camera pose 
       

    // camera to body pose

    //ROS_INFO("car_pos=%.8f",car_pos.y);
    // 
    transform.setOrigin( tf::Vector3(0.0, 0.0, -0.05) ); // 3*1 length in body (body to camera)
    transformcr.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    transform.setRotation(q); // 3*3 body to camera
    transformcr.setRotation(p);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ardrone/ground_truth/ardrone/ground_truth/odometry_sensorgt_link", "ardrone/down_cam_optical_frame"));
    cr.sendTransform(tf::StampedTransform(transformcr, ros::Time::now(), "tag_0", "polaris_ranger"));
   
    try{
     listener.lookupTransform("world","tag_0",ros::Time(0),transformer);
    }
   
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }

    double x_index = car_pos.x;
    double y_index = car_pos.y;    
    double z_index = car_pos.z;

  /*
    double x_index = transformer.getOrigin().x();
    double y_index = transformer.getOrigin().y();    
    double z_index = transformer.getOrigin().z();
    */
    cal_lon_lat(lon_world_orgin,lat_world_orgin,x_index,y_index,z_index);
    double lon_world_target = lon_1;
    double lat_world_target = lat_1;
    double height_world_target = z_1; 


   double D_index0 = TwoPointsHorDistance(lon_world_orgin,lat_world_orgin,lon_world_target,lat_world_target);
   double D_index1 = sqrt(x_index*x_index + y_index*y_index);


   //ROS_INFO("err_dis=%.8f",D_index0 - D_index1);


   qua.x = transformer.getRotation().x();
   qua.y = transformer.getRotation().y();
   qua.z = transformer.getRotation().z();
   qua.w = transformer.getRotation().w();
   eu.roll = atan2(2.0*(qua.w*qua.x+qua.y*qua.z),1.0-2.0*(qua.x*qua.x+qua.y*qua.y));
   eu.pitch = asin(2*(qua.w*qua.y-qua.z*qua.x));
   eu.yaw  = atan2(2.0*(qua.w*qua.z+qua.y*qua.x),1.0-2.0*(qua.z*qua.z+qua.y*qua.y));

    pose_Index.header.stamp = ros::Time::now();

    pose_Index.pose.position.x = lon_world_target;
    pose_Index.pose.position.y = lat_world_target;
    pose_Index.pose.position.z = height_world_target;

    pose_Index.pose.orientation.x = eu.roll;
    pose_Index.pose.orientation.y = eu.pitch;
    pose_Index.pose.orientation.z = eu.yaw;
    pose_Index.pose.orientation.w = 0; 
    

    // cout<< lon_world_target<< " "<< lat_world_target << " " << height_world_target <<endl;
    double position[3] = {lon_world_target, lat_world_target, height_world_target};
    double attitude[3] = {eu.roll, eu.pitch, eu.yaw};
    
    serial_send("/dev/ttyUSB0", 57600, position, attitude);

   double secs =ros::Time::now().toSec();

   //ROS_INFO("lon_world_target=%.9f",lon_world_target);  
   
    // quaternion1:world to body


    // quaternion2:body to camera


    // camera position in body

    // Taget position in camera

    // quaternion3: camera to Taget 


////  output Taget position and quaternion4 in world


	//pub.publish(pose_Index);
   // ROS_INFO("A = %.2f",q.x());

     ros::spinOnce();

    loop_rate.sleep();

   	}    

  
    //ros::spin();
    return 0;
}
