#include <ros/ros.h>
#include <nav_msgs/Path.h>

static double PI = 3.14;
static double x_1 = 0.0;
static double y_1 = 0.0;

static double x_2 = 3.0;
static double y_2 = 5.0;
static double pt1_x;
static double pt1_y;
static double pt2_x;
static double pt2_y;

static double angle_1 = 30 * PI / 180;
static double angle_2 = 0 * PI / 180;
static double r_min = 1;
int t = 0;
int r = 0;
int k = 0;
int s = 0;
bool Flag = 0;
int f=0;

double Center_x (const double x, const double angle){
  return (x + r_min* cos(angle ));
}

double Center_y ( const double y , const double angle){
  return (y + r_min* sin(angle));
}

void Tangent_Cicle(const bool begin, const bool targer, const double circle_begin_x,const double circle_begin_y,
                   const double circle_target_x,const double circle_target_y ){

  if(begin == targer){
    //поиск внешних касательных
    double D = sqrt( pow(circle_begin_x*circle_target_x,2) + pow(circle_begin_y*circle_target_y ,2));
    double Y = sqrt(pow(D,2)+ pow(r_min,2));
    double teta = acos( ( pow(r_min,2) + pow(D,2) - pow(Y,2) )/ (2* r_min * D) ) + atan2(circle_begin_y*circle_target_y,circle_begin_x*circle_target_x);

    pt1_x = circle_begin_x +r_min * cos(teta);
    pt1_y = circle_begin_y +r_min * sin(teta);

    double teta1 = acos( ( pow(r_min,2) + pow(Y,2) - pow(D,2) )/ (2* r_min * Y) ) + atan2(circle_begin_y*circle_target_y,circle_begin_x*circle_target_x);

    pt2_x = circle_target_x +r_min * cos(teta);
    pt2_y = circle_target_y +r_min * sin(teta);

  }


}

double ArcLenght (const double x1,const double y1,const double x2,const double y2,const double cx1, const double cy1, const bool state ){
  double teta = atan2(y1 - cy1,x1 - cx1) - atan2(y2 - cy1, x2 - cx1);
  if ( teta < 0 && state == 1){
    teta = teta + 2* PI;
  }

  else if(teta > 0 && state == 0){
    teta = teta - 2* PI;
  }
  return teta;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dubins_test_node");

  ros::NodeHandle nh;

  ros::Publisher puth_odom_pub = nh.advertise<nav_msgs::Path>("/path_odomm",  6000);
  ros::Publisher puth_cent1_pub = nh.advertise<nav_msgs::Path>("/path_centr1",  6000);
  ros::Publisher puth_cent2_pub = nh.advertise<nav_msgs::Path>("/path_centr2",  6000);

  ros::Publisher puth_tangent_pub = nh.advertise<nav_msgs::Path>("/path_tan",  6000);
  ros::Publisher puth_arc1_pub = nh.advertise<nav_msgs::Path>("/path_arc1",  6000);

  // вычесление центра с данными координатами
  double centr_x1 = Center_x(x_1, angle_1);
  double centr_y1 = Center_y(y_1, angle_1);
  double centr_x2 = Center_x(x_2, angle_2);
  double centr_y2 = Center_y(y_2, angle_2);

  while(!Flag){
    nav_msgs::Path dubins;
    nav_msgs::Path cent1;
    nav_msgs::Path cent2;
    nav_msgs::Path tan;
    nav_msgs::Path arc1;


    double pov = ( 90) *PI/180 ;
    double v = 0.1;
    int tet = pov/ v;
    dubins.poses.resize(t+1);
    dubins.poses[t].pose.position.x = x_1;
    dubins.poses[t].pose.position.y= y_1;
    dubins.header.frame_id = "map";
    dubins.header.stamp = ros::Time::now();
    puth_odom_pub.publish(dubins);
    t++;
    dubins.poses.resize(t+1);
    dubins.poses[t].pose.position.x = x_2;
    dubins.poses[t].pose.position.y= y_2;
    dubins.header.frame_id = "map";
    dubins.header.stamp = ros::Time::now();
    puth_odom_pub.publish(dubins);
    t++;

    cent1.poses.resize(r+1);
    cent1.poses[r].pose.position.x = centr_x1;
    cent1.poses[r].pose.position.y= centr_y1;
    cent1.header.frame_id = "map";
    cent1.header.stamp = ros::Time::now();
    puth_cent1_pub.publish(cent1);
    r++;
    cent1.poses.resize(r+1);
    cent1.poses[r].pose.position.x = x_1;
    cent1.poses[r].pose.position.y= y_1;
    cent1.header.frame_id = "map";
    cent1.header.stamp = ros::Time::now();
    puth_cent1_pub.publish(cent1);
    r++;


    cent2.poses.resize(k+1);
    cent2.poses[k].pose.position.x = x_2;
    cent2.poses[k].pose.position.y= y_2;
    cent2.header.frame_id = "map";
    cent2.header.stamp = ros::Time::now();
    puth_cent2_pub.publish(cent2);
    k++;

    cent2.poses.resize(k+1);
    cent2.poses[k].pose.position.x = centr_x2;
    cent2.poses[k].pose.position.y= centr_y2;
    cent2.header.frame_id = "map";
    cent2.header.stamp = ros::Time::now();
    puth_cent2_pub.publish(cent2);
    k++;


    Tangent_Cicle(1, 1, centr_x1,centr_y1, centr_x2,centr_y2 );



    tan.poses.resize(s+1);
    tan.poses[s].pose.position.x = pt1_x;;
    tan.poses[s].pose.position.y= pt1_y;;
    tan.header.frame_id = "map";
    tan.header.stamp = ros::Time::now();
    puth_tangent_pub.publish(tan);
    s++;

     tan.poses.resize(s+1);
     tan.poses[s].pose.position.x = pt2_x;;
     tan.poses[s].pose.position.y= pt2_y;;
     tan.header.frame_id = "map";
     tan.header.stamp = ros::Time::now();
     puth_tangent_pub.publish(tan);
     s++;

     double arcl1 = ArcLenght ( x_1, y_1, pt1_x,pt1_y,centr_x1, centr_y1, 1 );
     double L1 = arcl1 * r_min;


     int delt = L1 / 0.05 ;

     double x = x_1;
     double y = y_1;

     double teta = 0.05 ;
     arc1.poses.clear();
     while (teta < arcl1 ){
       x = centr_x1+ r_min *cos(teta);
       y = centr_y1 +r_min * sin(teta);
       teta += 0.05;

       geometry_msgs::PoseStamped point;
       point.pose.position.x = x;
       point.pose.position.y = y;
       point.pose.position.z = 0;
       arc1.poses.push_back(point);

       f++;

       ROS_INFO("arc x %f , arc y %f", x, y);
     }

     arc1.header.stamp = ros::Time::now();
     arc1.header.frame_id = "map";
     puth_arc1_pub.publish(arc1);



     }



  return 0;
}
