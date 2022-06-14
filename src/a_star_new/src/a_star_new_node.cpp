#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include <nav_msgs/Path.h>
#include <cstdlib>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include<std_msgs/Int32.h>


static double begin_cell_x;
static double begin_cell_y;
static double target_cell_x;
static double target_cell_y;
static double f ;
static double g;
static double h ;
static double f_cur ;
static double g_curr = 0;
static int i=0;
static bool FlagBegin = false;
static bool FlagTarget = false;
int end = 0;
static int dub = 7;

void callbackBegCell(const geometry_msgs::PointStamped &beg){
    begin_cell_x = beg.point.x;
    begin_cell_y = beg.point.y;
    FlagBegin = true;
}
void callbackTargetCell(const geometry_msgs::PointStamped &targ){
    target_cell_x = targ.point.x;
    target_cell_y = targ.point.y;
    FlagTarget = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star_new_node");
    ros::NodeHandle nh;

    ros::Subscriber begin_point = nh.subscribe("/clicked_point1",1, callbackBegCell);
    ros::Subscriber target_point = nh.subscribe("/clicked_point2",1,callbackTargetCell);

    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap srv;

    ros::Publisher map = nh.advertise<nav_msgs::OccupancyGrid>("/map", 50);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_local", 50);
    ros::Publisher map_ul_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_uluch", 50);
    ros::Publisher puth_odom_pub = nh.advertise<nav_msgs::Path>("/path_odomm",  6000);

    ros::Publisher flag = nh.advertise<std_msgs::Int32>("end_path",50);

    if (client.call(srv)){

    }


    nav_msgs::OccupancyGrid map_ulush;
    nav_msgs::Path path;
    bool endPuth=true;

    ros::Rate loop_rate(10);
    ros::Rate r(1);

    while(end == 0) //(ros::ok())
    {
      ros::spinOnce();

      double global_param = 0.15;

      //расширение препятствий, для проезда робота не касаясь их
      int col = ceil((sqrt(2)*global_param/2)/srv.response.map.info.resolution);

      map_ulush=srv.response.map;

      // заполнение карты

      for(uint s = 0; s < srv.response.map.data.size(); s++){
        if(srv.response.map.data[s] > 80){
          for (int k = -col; k<=col; k++){
            for (int j = - col ; j<=col ; j++){
              int stroka = trunc(s/srv.response.map.info.height);
              int stolbec = s-stroka*srv.response.map.info.height;
              map_ulush.data[(stroka+k)*srv.response.map.info.height+(stolbec+j)]=100;
            }
          }
        }

      }

      if(FlagBegin && FlagTarget){

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
        msg2.header.frame_id = "map";
        map_ul_pub.publish(msg2);

        while(endPuth  ){

          double x=begin_x;
          double y=begin_y;
          double x_n=begin_x;
          double y_n=begin_y;

          ROS_INFO("Begin x %f, y %f",begin_x,begin_y);
          ROS_INFO("End x %f, y %f",target_x,target_y);

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
                g_curr = map_ulush.data[(stroka+k)*map_ulush.info.width+(stolbec+j)];


                f = g_curr + h;
                ROS_INFO(" f  %f",f);
                ROS_INFO(" g %f",g_curr);
                ROS_INFO(" h %f",h);


                if ( time == 0){
                  f_cur = f;
                }

                if(f <= f_cur ){
                  if(g_curr >= 50){



                    map_ulush.data[(stroka+k)*map_ulush.info.width+(stolbec+j)]=100;
                    begin_cell = (stroka+k-1)*map_ulush.info.height+(stolbec+j-1);
                    x=x_n;
                    y=y_n;

                  }

                  else{
                    begin_cell = (stroka+k)*map_ulush.info.height+(stolbec+j);
                    f_cur = f;
                    ROS_INFO(" f < f_cur x = %f",x);
                    ROS_INFO(" f < f_cur y = %f",y);
                    x_n=x;
                    y_n=y;
                  }

                }

                time++;

              }

          }

          ROS_INFO(" i %d",i);
          //ROS_INFO(" end %d",end);
          path.poses.resize(i+1);

          path.poses[i].pose.position.x = x_n;
          path.poses[i].pose.position.y = y_n;

          ROS_INFO(" x_n = %f",x_n);
          ROS_INFO("y_n = %f",y_n);
          ROS_INFO("FlagBegin %d",FlagBegin);
          path.header.frame_id = "map";
          path.header.stamp = ros::Time::now();
          puth_odom_pub.publish(path);
          i++;
          if(begin_cell== target_cell){
              endPuth=false;
              FlagBegin = false;
              FlagTarget = false;
              end = 1;


        }

      }
      }

      nav_msgs::OccupancyGrid msg;
      msg=srv.response.map;
      msg.header.stamp=ros::Time::now();
      msg.header.frame_id = "map";
      map_pub.publish(msg);



      std_msgs::Int32 numb;
      numb.data = end;
      flag.publish(numb);
      ROS_INFO(" end %d",numb);

      

//      path.header.frame_id = "map";
//      path.header.stamp = ros::Time::now();
//      puth_odom_pub.publish(path);


      loop_rate.sleep();


    }
    ROS_INFO("path x0 %f",path.poses[dub].pose.position.x);
    ROS_INFO("path orintation %f",path.poses[dub].pose.orientation.x);
    return 0;
}



