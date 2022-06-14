#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>
#include <cstdlib>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include<std_msgs/Int32.h>
#include <cmath>
#include <vector>
#include <tf/transform_broadcaster.h>
#include "dubins.h"
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
static double DELTA = 0.05;

typedef std::pair<double,double> Point2d;
static double begin_cell_x = 0.0;
static double begin_cell_y = 0.0;
static double begin_angle = 0.0;
static double target_cell_x = 1.0;
static double target_cell_y = 1.0;
static double f ;
static double h ;
static double f_cur ;
static double g_curr = 0;
static int i=0;
static bool FlagBegin = false;
static bool FlagTarget = false;
static int end = 0;
static double r_min = 0.8;
static double PI = 3.1415;
static double resolution;
static double circle_begin_x;
static double circle_begin_y;

static double circle_target_x ;
static double circle_target_y;
static double wheelbase = 0.12;

static double pt1_x;
static double pt1_y;

static double pt2_x;
static double pt2_y;

static uint u = 0;
static uint t = 0;

geometry_msgs::PoseStamped point;

static std::vector<double> vector_angle;
static std::vector<double> path_x;
static std::vector<double> path_y;
static std::vector<double> path_tet;
nav_msgs::Odometry  odom;
static geometry_msgs::Quaternion odom_quat;


nav_msgs::OccupancyGrid map_pol;



void callbackMap(const nav_msgs::OccupancyGrid &map){
    resolution = map.info.resolution;

    map_pol = map;

}


int printConfiguration(double q[3], double x, void* user_data) {
    printf("%f,%f,%f,%f, %d\n", q[0], q[1], q[2], x , u);
    path_x.push_back(double(q[0]));
    path_y.push_back(double(q[1]));
    path_tet.push_back(double(q[2]));
    u++;


    return 0;
}







void callbackTargetCell(const geometry_msgs::PointStamped &targ){
    target_cell_x = targ.point.x;
    target_cell_y = targ.point.y;
    FlagTarget = true;
}

void callbackPoint(const nav_msgs::Odometry &odom){
    begin_cell_x = odom.pose.pose.position.x;
    begin_cell_y = odom.pose.pose.position.y;
    odom_quat.w = odom.pose.pose.orientation.w;
    odom_quat.x = odom.pose.pose.orientation.x;
    odom_quat.y = odom.pose.pose.orientation.y;
    odom_quat.z = odom.pose.pose.orientation.z;
    Eigen::Quaternionf q_e;
    q_e.x() = odom_quat.x;
    q_e.y() = odom_quat.y;
    q_e.z() = odom_quat.z;
    q_e.w() = odom_quat.w;
    auto euler = q_e.toRotationMatrix().eulerAngles(0, 1, 2);
    begin_angle = euler(2);



}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dubins_path_node");
    ros::NodeHandle nh;

    // ros::Subscriber begin_point = nh.subscribe("/clicked_point1",1, callbackBegCell);
    ros::Subscriber target_point = nh.subscribe("/clicked_point2",1,callbackTargetCell);

    //ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    //nav_msgs::GetMap srv;

    ros::Subscriber map_sub = nh.subscribe("/global_map",1, callbackMap);
    ros::Subscriber position = nh.subscribe("/rtabmap/odom",1, callbackPoint);

    ros::Publisher map = nh.advertise<nav_msgs::OccupancyGrid>("/map1", 50);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_local", 50);
    ros::Publisher map_ul_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_uluch", 50);
    ros::Publisher puth_odom_pub = nh.advertise<nav_msgs::Path>("/path_odomm",  6000);
    ros::Publisher puth_dubins_pub = nh.advertise<nav_msgs::Path>("/path_dubins",  50);
    ros::Publisher puth_teta = nh.advertise<geometry_msgs::PoseStamped>("/teta",6000);

    ros::Publisher flag = nh.advertise<std_msgs::Int32>("end_path",50);

    //    if (client.call(srv)){

    //    }


    nav_msgs::OccupancyGrid map_ulush;
    nav_msgs::Path path;
    nav_msgs::Path dubins;

    geometry_msgs::PoseStamped tet_dub;
    bool endPuth=true;

    ros::Rate loop_rate(10);
    ros::Rate r(1);

    while(begin_cell_x < target_cell_x && begin_cell_y < target_cell_y) //(ros::ok())
    {
        ros::spinOnce();
        dubins.poses.clear();


        double global_param = 0.25;

        //расширение препятствий, для проезда робота не касаясь их
        int col = ceil((sqrt(2)*global_param/2)/resolution);
        //resolution = srv.response.map.info.resolution;

        map_ulush=map_pol;

        // заполнение карты

        for(uint s = 0; s < map_pol.data.size(); s++){
            if(map_pol.data[s] ==-1 ){
                map_pol.data[s] = 50;
            }
            if(map_pol.data[s] > 80 ){
                for (int k = -col; k<=col; k++){
                    for (int j = - col ; j<=col ; j++){
                        int stroka = trunc(s/map_pol.info.height);
                        int stolbec = s-stroka*map_pol.info.height;
                        map_ulush.data[(stroka+k)*map_pol.info.height+(stolbec+j)]=100;
                    }
                }
            }


        } // конец заполнения карты

        if( FlagTarget){

            double begin_x = begin_cell_x;
            double begin_y = begin_cell_y;
            double target_x = target_cell_x;
            double target_y = target_cell_y;

            int begin_x_cell = round(begin_x/map_ulush.info.resolution);
            int begin_y_cell = round(begin_y/map_ulush.info.resolution);

            int begin_cell = map_ulush.info.width/2 +begin_x_cell + (map_ulush.info.height/2 + begin_y_cell)*map_ulush.info.width;

            int target_x_cell = round(target_x/map_ulush.info.resolution);
            int target_y_cell = round(target_y/map_ulush.info.resolution);

            int target_cell = map_ulush.info.width/2 +target_x_cell + (map_ulush.info.height/2 + target_y_cell)*map_ulush.info.width;

            nav_msgs::OccupancyGrid msg2;
            msg2=map_ulush;
            msg2.header.stamp=ros::Time::now();
            msg2.header.frame_id = "odom";
            map_ul_pub.publish(msg2);
            int iter = 0;

            while(endPuth  && iter < 1000){


                iter++;

                double x=begin_x;
                double y=begin_y;
                double x_n=begin_x;
                double y_n=begin_y;

                // ROS_INFO("Begin x %f, y %f",begin_x,begin_y);
                //ROS_INFO("End x %f, y %f",target_x,target_y);

                int stroka = floor(begin_cell/map_ulush.info.width);
                int stolbec = begin_cell-stroka*map_ulush.info.width;

                int time =0;
                for (int k = -2; k<=2; k++) {
                    for (int j = - 2 ; j<=2 ; j++){





                        g_curr = map_ulush.data[(stroka+k)*map_ulush.info.width+(stolbec+j)];




                        if((stroka+k)>=map_ulush.info.height/2){
                            y=((stroka+k)-map_ulush.info.height/2)*map_ulush.info.resolution;
                        }
                        else{
                            y=-(map_ulush.info.height/2-(stroka+k))*map_ulush.info.resolution;
                        }

                        if((stolbec+j)>=map_ulush.info.width/2){
                            x=((stolbec+j)-map_ulush.info.width/2)*map_ulush.info.resolution;
                        }

                        else{
                            x=-(map_ulush.info.width/2-(stolbec+j))*map_ulush.info.resolution;
                        }

                        h = sqrt(pow(x-target_x,2)+pow(y-target_y,2));
                        g_curr =0;// map_ulush.data[(stroka+k)*map_ulush.info.width+(stolbec+j)];


                        f = g_curr + h;
                        ROS_INFO(" f  %f",f);
                        //ROS_INFO(" g %f",g_curr);
                        //ROS_INFO(" h %f",h);


                        if ( time == 0){
                            f_cur = f;
                        }

                        if(f <= f_cur ){
                            if(g_curr >= 70){



                                map_ulush.data[(stroka+k)*map_ulush.info.width+(stolbec+j)]=100;
                                begin_cell = (stroka+k-1)*map_ulush.info.height+(stolbec+j-1);
                                x=x_n;
                                y=y_n;

                            }

                            else{
                                begin_cell = (stroka+k)*map_ulush.info.height+(stolbec+j);
                                f_cur = f;
                                //ROS_INFO(" f < f_cur x = %f",x);
                                //ROS_INFO(" f < f_cur y = %f",y);
                                x_n=x;
                                y_n=y;
                            }

                        }



                        time++;






                    }

                }


                path.poses.resize(i+1);

                path.poses[i].pose.position.x = x_n;
                path.poses[i].pose.position.y = y_n;

                if (i == 0 ){
                    path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(begin_angle);
                }

                else{
                    double ang = atan2(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
                    vector_angle.push_back(ang);
                    path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(ang);

                }

                //ROS_INFO(" x_n = %f",x_n);
                // ROS_INFO("y_n = %f",y_n);
                //ROS_INFO("FlagBegin %d",FlagBegin);
                path.header.frame_id = "odom";
                path.header.stamp = ros::Time::now();
                puth_odom_pub.publish(path);
                i++;
                if(begin_cell== target_cell){
                    endPuth=false;
                    FlagBegin = false;
                    FlagTarget = false;
                    end = 1;


                }

            }// конец while

            endPuth=true;
            FlagTarget = true;
        }// конец проверки условия флагов, на наличие начальной и корнечной точки

        nav_msgs::OccupancyGrid msg;
        msg=map_pol;
        msg.header.stamp=ros::Time::now();
        msg.header.frame_id = "odom";
        map_pub.publish(msg);



        std_msgs::Int32 numb;
        numb.data = end;
        flag.publish(numb);
        //ROS_INFO(" end %d",numb);

        //loop_rate.sleep();

        int count = 0;


        while(count < i - 1){


            double x1;
            double y1;
            double angle_x1;


            int step =count + int(r_min/resolution);

            double x4;
            double y4;
            double angle_x4;


            if ( count < i - 1){
                x1 = path.poses[count].pose.position.x;
                y1 = path.poses[count].pose.position.y;
                angle_x1 = vector_angle[count];



                if (step >= i - 1){
                    x4 = path.poses[i - 1].pose.position.x;
                    y4 = path.poses[i - 1].pose.position.y;
                    angle_x4 = vector_angle[i - 1];


                }

                else{

                    x4 = path.poses[step].pose.position.x;
                    y4 = path.poses[step].pose.position.y;
                    angle_x4 = vector_angle[step];
                }


            } // end of if ( count < i)




            if ( angle_x1 > angle_x4 || angle_x1 < angle_x4 ){

                if (sqrt(pow(x1-x4,2)+pow(y1-y4,2)) < (r_min * 2) ){
                    point.pose.position.x = x1;
                    point.pose.position.y = y1;
                    point.pose.position.z = 0;
                    point.pose.orientation = tf::createQuaternionMsgFromYaw(angle_x1);

                    dubins.poses.push_back(point);

                    point.pose.position.x = x4;
                    point.pose.position.y = y4;
                    point.pose.position.z = 0;
                    point.pose.orientation = tf::createQuaternionMsgFromYaw(angle_x4);

                    dubins.poses.push_back(point);
                }

                else{


                    double q0[] = { x1, y1, angle_x1 };
                    double q1[] = { x4, y4, angle_x4 };
                    DubinsPath pathl;
                    dubins_shortest_path(&pathl, q0, q1, r_min);

                    printf("#x,y,theta,t\n");
                    dubins_path_sample_many(&pathl,  0.8, printConfiguration, NULL);


                    for (uint r = t;r < u; r++) {
                        ROS_INFO("start");

                        point.pose.position.x = path_x[r];
                        ROS_INFO("path_x[%d] %f",r,path_x[r]);
                        point.pose.position.y = path_y[r];
                        point.pose.position.z = 0;
                        point.pose.orientation = tf::createQuaternionMsgFromYaw(path_tet[r]);

                        dubins.poses.push_back(point);

                }
                t = u;


                }

//                dubins.header.frame_id = "map";
//                dubins.header.stamp = ros::Time::now();
//                puth_dubins_pub.publish(dubins);
            }

            else {

                point.pose.position.x = x1;
                point.pose.position.y = y1;
                point.pose.position.z = 0;
                point.pose.orientation = tf::createQuaternionMsgFromYaw(angle_x1);

                dubins.poses.push_back(point);

                point.pose.position.x = x4;
                point.pose.position.y = y4;
                point.pose.position.z = 0;
                point.pose.orientation = tf::createQuaternionMsgFromYaw(angle_x4);

                dubins.poses.push_back(point);


            }



            count = step;



        } //конец while, построение пути окончено

        ROS_INFO("End of Dubins");



        i = 0;

        count =0;
        dubins.header.frame_id = "odom";
        dubins.header.stamp = ros::Time::now();
        puth_dubins_pub.publish(dubins);


        path.poses.clear();
        dubins.poses.clear();
        ROS_INFO("Dubins size %f",dubins.poses.size());


        ros::spinOnce();






    }// end while dubins






    return 0;
} // конец main




