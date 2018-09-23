/*************************************************************************

   SPDR ARDUINO MainScript

   This script uses a modified version of the original FreeNove-Core in a way that it
   allows us to monitor and control the Bot using ROS topics.

   The ArduinoMega has a MPU9250 gyro connected on I2C.
   We use ROSSerial to publish all relevant data to ROS and subscribe
   to our command-topics.

   version: 0.00001

 **************************************************************************/

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <SPDR1_FNQRComm.h>
#include "quaternionFilters.h"
//#include "quaternion.h"
#include "MPU9250.h"
#include <Wire.h>

#include "quaternionFilters.h"

MPU9250 myIMU;

#define AHRS false         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging
#define DO_CALIBRATION true

#define LIDAR_PIN 2
#define LIDAR_MIN 90
#define LIDAR_OFF 0

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS 0x68

ros::NodeHandle  nh;
Communication robot;

geometry_msgs::TransformStamped t;
geometry_msgs::Vector3 vec;

tf::TransformBroadcaster broadcaster;
char world[] = "/earth";
char maplink[] = "/map";
char base_link[] = "/base_link";
char base_stabilized[] = "/base_stabilized";
char base_footprint[] = "/base_footprint";
char odom[] = "/odom";
char laser[] = "/laser";

// callback actions

void action_cb( const std_msgs::UInt16& cmd_msg) {
  switch (cmd_msg.data) {

    case 'a': robot.ActiveMode(); break;
    case 's': robot.SleepMode(); break;
    case 'w': welcomeMoves(); break;
  }

}
void move_cb( const geometry_msgs::Twist& cmd_msg) {
  // TBD set crawlheight = z
  if (cmd_msg.linear.x < 0) robot.TurnLeft();
  else if (cmd_msg.linear.x > 0) robot.TurnRight();

  if (cmd_msg.linear.y < 0) robot.CrawlForward();
  else if (cmd_msg.linear.y > 0) robot.CrawlBackward();

  // TBD use angular to "steer"
}

void bodyMove_cb( const geometry_msgs::Twist& cmd_msg) {
  robot.MoveBody(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z);
  robot.RotateBody(cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z, cmd_msg.linear.z);
}

void lidarspeed_cb( const std_msgs::UInt16& cmd_msg) {
  analogWrite(LIDAR_PIN, cmd_msg.data);
}

// pub/sub definitions

std_msgs::String str_msg;
std_msgs::UInt16 mode_msg;
std_msgs::UInt16 int_msg;
std_msgs::Float32 float_msg;
sensor_msgs::Imu imu;

ros::Subscriber<std_msgs::UInt16> subAction("body/cmd_mode", action_cb);
ros::Subscriber<geometry_msgs::Twist> subMoveBody("body/cmd_twist", bodyMove_cb);
ros::Subscriber<geometry_msgs::Twist> subMove("body/cmd_vel", move_cb);
ros::Subscriber<std_msgs::UInt16> subLidarSpeed("body/cmd_lidarSpeed", lidarspeed_cb);

ros::Publisher chatter("/oled/text", &str_msg);
ros::Publisher imu_pub("/body/imu", &imu);
ros::Publisher angle_pub("angle", &vec);
ros::Publisher velocity_pub("velocity", &vec);
ros::Publisher magnet_pub("magnet", &vec);
ros::Publisher mode_pub("mode", &mode_msg);
ros::Publisher voltage_pub("voltage", &float_msg);
ros::Publisher distance_pub("distance", &float_msg);

// WelcomeMove on startup
void welcomeMoves() {
  robot.RotateBody(1, 0, 0, 20);
  delay(1000);
  robot.RotateBody(-1, 0, 1, -20);
  delay(1000);
  robot.RotateBody(1, 0, 1, 20);
  delay(1000);
  robot.MoveBody(0, 0, 25);
  delay(1000);
  robot.robotAction.InitialState();
  robot.SleepMode();
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Serial.println("starting CORE");

  robot.Start();

  pinMode(LIDAR_PIN, OUTPUT);

  Serial.println("CORE is alive");

  robot.robotAction.speedMoveBody = 3.45f;
  robot.robotAction.speedRotateBody = 3.45f;

  Serial.println("inititiating MPU Gyro");

  initMPU();

  Serial.println("connecting to ROS...");

  nh.initNode();
  nh.subscribe(subLidarSpeed);
  nh.subscribe(subAction);
  nh.subscribe(subMoveBody);
  nh.subscribe(subMove);
  nh.advertise(chatter);
  nh.advertise(distance_pub);
  nh.advertise(voltage_pub);
  nh.advertise(angle_pub);
  nh.advertise(imu_pub);
  nh.advertise(velocity_pub);
  nh.advertise(magnet_pub);

  /*for (int i = 0; i < 4; i++)
    {
    String leg="/body/legs/";
    leg+=i;

    ros::Publisher leg1goal_pub(String(leg+"/point/goal").c_str(), &vec);
    ros::Publisher leg1now_pub("/body/legs/1/point/now", &vec);
    nh.advertise(leg1goal_pub);
    nh.advertise(leg1now_pub);

    ros::Publisher leg1nowa_pub("/body/legs/1/joint/a/anglenow", &float_msg);
    ros::Publisher leg1nows_pub("/body/legs/1/joint/a/servonow", &float_msg);
    ros::Publisher leg1nowo_pub("/body/legs/1/joint/a/offset", &int_msg);
    nh.advertise(leg1nowa_pub);
    nh.advertise(leg1nows_pub);
    nh.advertise(leg1nowo_pub);
    }
  */

  broadcaster.init(nh);
  Serial.println("ROS connection alive. Welcome.");

  // Wait some seconds, then say hello
  delay(3000);
  welcomeMoves();

  analogWrite(LIDAR_PIN, LIDAR_OFF);
}

// TBD cleanups

long lastm2 =  0;
long lastm3 =  0;

void publishLegs() {

}

int readycount = 0;

void loop() {
  getMPUValues();

  if (millis() - lastm2 > 3000) {

    // VOLTAGE

    float_msg.data = robot.GetSupplyVoltage();
    voltage_pub.publish(&float_msg);

    // PING
    String ready = "READY";
    for (int i = 0; i < readycount; i++)
      ready += ".";
    str_msg.data = ready.c_str();
    chatter.publish( &str_msg );
    readycount++;
    readycount = readycount % 4;
    lastm2 = millis();

  }

  if (millis() - lastm3 > 100) {

    // ANGLE

    ros::Time n = nh.now();

    imu.orientation.x = *(getQ() + 1);
    imu.orientation.y = *(getQ() + 2);
    imu.orientation.z = *(getQ() + 3);
    imu.orientation.w = *(getQ() + 0);

    imu.angular_velocity.x = myIMU.gx;
    imu.angular_velocity.y = myIMU.gy;
    imu.angular_velocity.z = myIMU.gz;
    imu.linear_acceleration.x = myIMU.ax;
    imu.linear_acceleration.y = myIMU.ay;
    imu.linear_acceleration.z = myIMU.az;
    imu.header.frame_id = base_link;
    imu.header.stamp = n;
    imu_pub.publish(&imu);



    t.header.frame_id = world;
    t.child_frame_id = maplink;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    //t.header.stamp = n;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    broadcaster.sendTransform(t);

    t.header.frame_id = maplink;
    t.child_frame_id = odom;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.header.stamp = n;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    //  broadcaster.sendTransform(t);
    //
    t.header.frame_id = odom;
    t.child_frame_id = base_footprint;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    //t.header.stamp = n;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    broadcaster.sendTransform(t);

    t.header.frame_id = base_footprint;
    t.child_frame_id = base_stabilized;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.05;
    //t.header.stamp = n;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    broadcaster.sendTransform(t);


    t.header.frame_id = base_stabilized;
    t.child_frame_id = base_link;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    //t.header.stamp = n;
    t.transform.rotation.x = 0.0;//*(getQ() + 1);
    t.transform.rotation.y = 0.0;//*(getQ() + 2);
    t.transform.rotation.z = 0.0;//*(getQ() + 3);
    t.transform.rotation.w = 1.0;//*(getQ() + 0);
    broadcaster.sendTransform(t);


    t.header.frame_id = base_link;
    t.child_frame_id = laser;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.05;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    //t.header.stamp = n;
    broadcaster.sendTransform(t);


    lastm3 = millis();
  }

  nh.spinOnce();
  delay(10);
}
