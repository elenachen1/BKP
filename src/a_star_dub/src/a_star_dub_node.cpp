#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

double e1;
double ang;
bool Ny;


void chatEncoderone(const sensor_msgs::JointState &enc_one)
{

  //ROS_INFO("velocity_joint: %f", enc_one.velocity[1]);
  //ROS_INFO("steering_joint: %f", enc_one.position[0]);
  e1 = enc_one.velocity[1];//перемещение
  ang = enc_one.position[0];//угол поворота
  Ny = true;
}


int main(int argc, char** argv){
ros::init(argc, argv, "a_star_dub_node");

ros::NodeHandle n;
ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
ros::Publisher odom_publ = n.advertise<geometry_msgs::TransformStamped>("odom_trans", 50);
tf::TransformBroadcaster odom_broadcaster;

ros::Subscriber sub = n.subscribe("/joint_states", 1, chatEncoderone);
//Ny = false;

double k = 0.11;
double l = 0.69;
double imp = 33000.0;
double pi = 3.1415;



//Инициализация параметров пространственного положения
double x = 0.0;
double y = 0.0;
double th = 0.0;

double delta_th = 0.0;

//Инициализация параметров временных штампов
ros::Time current_time, last_time;
current_time = ros::Time::now();
last_time = ros::Time::now();
ros::Rate R (100.0);


while(ros::ok()){
    if( Ny){

    current_time = ros::Time::now();



    double d =  e1 / imp;


    double sig = ( k * ang * pi) / 180 ;
    //расстояние до мгновенного центра


    delta_th =  d /(l / tan(sig));

    //Вычисление параметров перемещения


    th +=delta_th;
    x += d*cos(th);;
    y += -d*sin(th);


    //Формируем кватернион на основе значения угла курса
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-th);

    //Формируем сообщение, содержащее трансформацию систем координат
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //Отправляем сообщение
    odom_broadcaster.sendTransform(odom_trans);

    //Формируем сообщение, содержащее параметры одометрии
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //Параметры положения
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //Параметры скорости
    odom.child_frame_id = "base_link";

    //Публикуем сообщение с одометрией
    odom_pub.publish(odom);
    odom_publ.publish(odom_trans);
    last_time = current_time;
    Ny = false;
    }
    ros::spinOnce();
    R.sleep();

   }
}

