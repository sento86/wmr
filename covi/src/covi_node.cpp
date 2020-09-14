#include <ros/ros.h>
#include <stdio.h>      // perror()
#include <stdlib.h>
#include <cstdio>       // printf
#include <time.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>     // exit / usleep
#include <error.h>
#include <errno.h>
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <iostream>
#include <sstream>
#include <algorithm>    // std::min

/*To reuse definition types*/
#include "covi/covi.h"

//#include <safetrans_msgs/bt_board.h>
//#include <safetrans_msgs/Float64Stamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define ARDUINO
#define IGNORE_CRC

struct timeval start;

INPUT_DATA_T input_data;
STATE_DATA_T state_data;
OUTPUT_DATA_T output_data;
SETUP_DATA_T setup_data;

//safetrans_msgs::bt_board msg_bt_board;
//safetrans_msgs::Float64Stamped msg_steer, msg_accel, msg_brake;

short *input_data_ptr;
short *state_data_ptr;
short *output_data_ptr;
short *setup_data_ptr;

unsigned int rxPos = 0;
unsigned int last_pos = 0;
unsigned int rxInputCount = 0;
unsigned int rxOutputCount = 0;
unsigned int numInputs = 0;
unsigned int numOutputs = 0;
//unsigned int numCollision=0;

unsigned char rx_msg[1000];
//char buf[200];

//double ACCEL_SLOPE, ACCEL_OFFSET;
//double BRAKE_SLOPE, BRAKE_OFFSET;
//double STEER_SLOPE, STEER_OFFSET;

//double steeringWheel = 0.0, throttlePedal = 0.0, brakePedal = 0.0;

unsigned int cnt=0;
int mode=0;

int new_message = 0;

int iteration = 0;

int state=0; // State machine variable

ros::Time time0;

double dt = 0.0;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double v = 0.0;
double w = 0.0;
double vref = 0.0;
double wref = 0.0;
double wL = 0.0;
double wR = 0.0;
double vL = 0.0;
double vR = 0.0;
double vLref = 0.0;
double vRref = 0.0;
double wLref = 0.0;
double wRref = 0.0;
double L = 0.39;   // Wheel Separation, aka Wheelbase
double R = 0.0775; // Wheel Diameter: D = 0.155 m; Wheel Radius: R = D/2 = 0.0775 m
double uL0 = 0.0;
double uR0 = 0.0;

int16_t uL = 0.0;
int16_t uR = 0.0;

//double Kp = 1.0, Ki = 0.0, Kd = 0.0;
double kp = 1.0, ki = 0.0, kd = 0.0;

//char base_link[] = "/base_link";
char base_link[] = "/base_footprint";
char odom[] = "/odom";

nav_msgs::Odometry odom_msg;
geometry_msgs::Twist twist_msg;
geometry_msgs::TwistStamped twistStp_msg;

geometry_msgs::TransformStamped t;

ros::Time time_now, time_old;

struct struct_config
{
  //Names
  std::string serialPortName;
  std::string baudRateName;
  std::string publishRateName;

  //Not configured through launch script, but static to this application
  int baudRate;
  int publishRate;
  std::string serialPort;

  struct_config()
  {
    serialPortName = "serialPort";
    baudRateName = "baudRate";
    publishRateName = "publishRate";
  }
};

struct struct_config config;

void getParameters(ros::NodeHandle nh, ros::NodeHandle np, struct struct_config* config)
{
  ROS_INFO("Getting Program parameters from Parameter Server ...");

  np.param(config->serialPortName, config->serialPort, std::string("/dev/ttyACM0"));
  //np.param(config->serialPortName, config->serialPort, std::string("/dev/rfcomm0"));
  np.param(config->baudRateName, config->baudRate, int(115200));
  np.param(config->publishRateName, config->publishRate, int(1));

  np.param(std::string("mode"), mode, 0);
  np.param(std::string("kp"), kp, 1.0);
  np.param(std::string("ki"), ki, 0.0);
  np.param(std::string("kd"), kd, 0.0);

  setup_data.mode = short(mode);
  setup_data.kp = short(round(kp*10));
  setup_data.ki = short(round(ki*10));
  setup_data.kd = short(round(kd*10));
}

#ifdef ARDUINO

#ifndef IGNORE_CRC
char CRC(char *str, int len);
#endif

#ifndef IGNORE_CRC
char CRC(char *str, int len)
{
	int i;
	unsigned char crc=0;
	for (i=0;i<len;i++)
	  crc^=str[i];
	return crc;
}
#endif

//@I4;%d;%d;%d;%d;O2;%d;%d;
unsigned int decodeRxMsg(char c)
{
	int16_t valI;
	int16_t valO;

	if ((rxPos==0)&&(c=='@'))
	{
		rx_msg[0]=c;
		rxPos++;
		numInputs=0;
		rxInputCount=0;
		input_data_ptr=&input_data.cpsL;
		numOutputs=0;
		rxOutputCount=0;
		state_data_ptr=&state_data.mode;
		last_pos=2;
	}
	else if ((rxPos>0)&&(numInputs==0))
	{
		if (c==';')
		{
			rx_msg[rxPos]=0; //Final de string
			//sscanf(&rx_msg[last_pos],"%d",&numInputs);
			numInputs=rx_msg[last_pos];
                        //printf("numInputs:%d\n",numInputs);
			rx_msg[rxPos]=c;
			rxPos++;
			rxInputCount=0;
			input_data_ptr=&input_data.cpsL;
			last_pos=rxPos;
		}
		else
		{
			rx_msg[rxPos]=c;
			rxPos++;
		}
	}
	else if (rxInputCount<numInputs)
	{
		if ((c==';')&&((rxPos-last_pos)==2))
		{
			rx_msg[rxPos]=0;
			//sscanf(&rx_msg[last_pos],"%d",&valI);
			valI=(int16_t)(((uint16_t)rx_msg[last_pos])<<8)|(((uint16_t)rx_msg[last_pos+1])&0x00FF);
                        //printf("pos:%d %d\n",rx_msg[last_pos],rx_msg[last_pos+1]);
			//printf("I:%d\n",valI);
			*input_data_ptr++=valI;
			last_pos=rxPos+1;
			rxInputCount++;
		}
		rx_msg[rxPos]=c;
		rxPos++;
	}
        else if ((rxPos>0)&&(numOutputs==0))
	{
		if (c==';')
		{
			rx_msg[rxPos]=0;
			//sscanf(&rx_msg[last_pos+1],"%d",&numOutputs);
			numOutputs=rx_msg[last_pos+1];
                        //printf("numOutputs:%d\n",numOutputs);
			rx_msg[rxPos]=c;
			rxPos++;
			rxOutputCount=0;
			state_data_ptr=&state_data.mode;
			last_pos=rxPos;
		}
		else
		{
			rx_msg[rxPos]=c;
			rxPos++;
		}
	}
	else if (rxOutputCount<numOutputs)
	{
		if ((c==';')&&((rxPos-last_pos)==2))
		{
			rx_msg[rxPos]=0;
			//sscanf(&rx_msg[last_pos],"%d",&valI);
			//valO=(((short)rx_msg[last_pos])<<8)|(rx_msg[last_pos+1]&0x00FF);
			valO=(int16_t)(((uint16_t)rx_msg[last_pos])<<8)|(((uint16_t)rx_msg[last_pos+1])&0x00FF);
			//printf("O:%d\n",valO);
			*state_data_ptr++=valO;
			last_pos=rxPos+1;
			rxOutputCount++;
		}
		rx_msg[rxPos]=c;
		rxPos++;
	}
	else if ((rxOutputCount==numOutputs)&&(rxOutputCount!=0))
	{
		rx_msg[rxPos]=c;
#ifndef IGNORE_CRC
		if (CRC(&rx_msg[1],rxPos)==0)
		{
		  rxPos=0;
		  return 1;
		}
#else
		rxPos=0;
		rxInputCount=0;
		rxOutputCount=0;
		input_data_ptr=&input_data.cpsL;
		state_data_ptr=&state_data.mode;
		last_pos=0;
		numInputs=0;
		numOutputs=0;
		return 1;
#endif
	}
	else
	{
		rxPos=0;
		rxInputCount=0;
		rxOutputCount=0;
		input_data_ptr=&input_data.cpsL;
		state_data_ptr=&state_data.mode;
		last_pos=0;
		numInputs=0;
		numOutputs=0;
	}
	return 0;
}
#endif

/** Callback for joy topic **/
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    twist_msg = *msg;
    vref = twist_msg.linear.x;
    wref = twist_msg.angular.z;
    //if(vref<0.0)
    //  wref = -wref;
}

/** Callback for joy topic **/
void twistStp_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    twistStp_msg = *msg;
    vref = twistStp_msg.twist.linear.x;
    wref = twistStp_msg.twist.angular.z;
    //if(vref<0.0)
    //  wref = -wref;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "covi_node");
  ROS_INFO("Initializing COVI Node Test");
  ros::NodeHandle nh, np("~");

  // Get parameters from server
  getParameters(nh, np, &config);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>( "odom",0);
  //ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, twist_cb);
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, twistStp_cb);

  tf::TransformBroadcaster broadcaster;

#ifdef ARDUINO
  //int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  int fd = open(config.serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
    perror("open_port: Unable to open serial port - \n");
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr (fd, &tty ) != 0 )
  {
    //cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    perror("tcgerattr ");
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, B115200);
  cfsetispeed (&tty, B115200);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;        // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;
  tty.c_cflag     &=  ~CRTSCTS;       // no flow control
  tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
  tty.c_oflag     =   0;                  // no remapping, no delays
  tty.c_cc[VMIN]      =   0;                  // read doesn't block
  tty.c_cc[VTIME]     =   1;                  // 0.1 seconds read timeout

  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
  tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
  tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  tty.c_oflag     &=  ~OPOST;              // make raw
  tty.c_iflag	  &=  ~INLCR;  // turn off translate \n to \r
  tty.c_iflag	  &=  ~ICRNL;  // turn off translate \r to \n

  /* Flush Port, then applies attributes */
  tcflush(fd, TCIFLUSH );

  if ( tcsetattr (fd, TCSANOW, &tty ) != 0)
  {
    //cout << "Error " << errno << " from tcsetattr" << endl;
  }

  tcflush(fd, TCIFLUSH );

#endif

  //***********************************
  output_data.pwmL = 0;
  output_data.pwmR = 0;

  float rpm2pwm  = 255.0 / (2.5 * 2.0*M_PI);
  float cps2rads = 2.0*(2.0*M_PI/36.0); // rad/s -> encoder*2 [counts/s] * 2*pi [rad/rev] / 36 [counts/rev]
  //output_data.cpsL = int16_t((input_data.pwmL/rpm2pwm)/cps2rads);
  //output_data.cpsR = int16_t((input_data.pwmR/rpm2pwm)/cps2rads);

  time_old = ros::Time::now();

  // Loop
  ros::Rate loop_rate(config.publishRate); //Desired frequency in Hz
  while (ros::ok())
  {
#ifdef ARDUINO
    /* Read variables */
    char c;
    int n;
    //int new_message = 0;
    new_message = 0;

    rxPos = 0;
    last_pos = 0;
    rxInputCount = 0;
    rxOutputCount = 0;
    numInputs = 0;
    numOutputs = 0;

    do
    {
      n = read(fd, &c, 1);
      if (n > 0)
      {
        new_message = decodeRxMsg(c);
        if (new_message == 1)
        {
          tcflush(fd, TCIFLUSH);
          break;
        }
      }
    } while (n > 0);

#else

    new_message=1;

#endif

    if (new_message ==1 )
    {

      time_now = ros::Time::now();
      dt = (time_now - time_old).toSec();
      time_old = time_now;

      wL = float(input_data.cpsL)*cps2rads;//2.0*(2.0*M_PI/36.0); // rad/s -> encoder*2 [counts/s] * 2*pi [rad/rev] / 36 [counts/rev]
      //wL = -wL; // Opposite sign
      wR = float(input_data.cpsR)*cps2rads;//2.0*(2.0*M_PI/36.0); // rad/s -> encoder*2 [counts/s] * 2*pi [rad/rev] / 36 [counts/rev]
      //wR = -wR; // Opposite sign (already implemented in Arduino)

      // UPDATE ODOMETRY

      vL = wL*R;
      vR = wR*R;

      v = (vR + vL)/2.0;
      w = (vR - vL)/L;

      x += cos(th)*v*dt;
      y += sin(th)*v*dt;
      th += w*dt;
      if(th > M_PI)
        th -= 2.0*M_PI;
      else if(th < -M_PI)
        th += 2.0*M_PI;

      // UPDATE PID CONTROL

      //vLref = twist_msg.linear.x - twist_msg.angular.z * L/2.0;
      //vRref = twist_msg.linear.x + twist_msg.angular.z * L/2.0;
      //vLref = twistStp_msg.twist.linear.x + twistStp_msg.twist.angular.z * L/2.0;
      //vRref = twistStp_msg.twist.linear.x - twistStp_msg.twist.angular.z * L/2.0;
      vLref = vref - wref * L/2.0;
      vRref = vref + wref * L/2.0;

      wLref = vLref/R;
      wRref = vRref/R;

      //double rpm2pwm = 255.0 / (2.5 * 2.0*M_PI);
      //uL0 = wLref * rpm2pwm; // wLref * PWMmax / wLmax [2.5 rev/s * 2*pi rad/rev]
      //uR0 = wRref * rpm2pwm; // wRref * PWMmax / wRmax [2.5 rev/s * 2*pi rad/rev]

      //uL = (int16_t) (wLref * rpm2pwm);
      //uR = (int16_t) (wRref * rpm2pwm);

      uL = (int16_t) (wLref + kp*(wLref-wL)) * rpm2pwm;
      uR = (int16_t) (wRref + kp*(wRref-wR)) * rpm2pwm;

      //uL = uL+(int16_t) kp*(wLref-wL) * rpm2pwm;
      //uR = uR+(int16_t) kp*(wRref-wR) * rpm2pwm;

      //output_data.pwmL = std::min(255,std::max(-255,uL));
      //output_data.pwmR = std::min(255,std::max(-255,uR));

      //output_data.pwmL = uL;
      //output_data.pwmR = uR;

      if(uL>255)
        output_data.pwmL = 255;
      else if(uL<-255)
        output_data.pwmL = -255;
      else
        output_data.pwmL = uL;

      if(uR>255)
        output_data.pwmR = 255;
      else if(uR<-255)
        output_data.pwmR = -255;
      else
        output_data.pwmR = uR;

      //output_data.pwmL = (int16_t)(vref*254);
      //output_data.pwmR = (int16_t)(vref*254);

      //updateControl = false;

      //output_data.pwmL = input_data.cpsL;//+1;
      //output_data.pwmR = input_data.cpsR+1;

      ROS_ERROR("pwm: %d %d",output_data.pwmL,output_data.pwmR);
      ROS_ERROR("cps: %d %d",input_data.cpsL,input_data.cpsR);
      ROS_ERROR("wW: %f %f",wL,wR);

    }

    // SEND CONTROL COMMAND

#ifdef ARDUINO
      /* Write a command */
      std::stringstream cmd;
      //cmd << "@C8;";
      cmd << "@";
      cmd << "C";
      cmd << 2;
      cmd << ";";
      //cmd << std::setfill('0') << std::setw(2) << output_data.cpsLref << ";";
      //cmd << std::setfill('0') << std::setw(2) << output_data.cpsRref << ";";
      cmd << (unsigned char)(output_data.pwmL>>8);
      cmd << (unsigned char)(0x00FF&output_data.pwmL);
      cmd << ";";
      cmd << (unsigned char)(output_data.pwmR>>8);
      cmd << (unsigned char)(0x00FF&output_data.pwmR);
      cmd << ";";
      //cmd << "S";
      cmd << 4;
      cmd << ";";
      //cmd << std::setfill('0') << std::setw(2) << setup_data.mode << ";";
      //cmd << std::setfill('0') << std::setw(2) << setup_data.kp << ";";
      //cmd << std::setfill('0') << std::setw(2) << setup_data.ki << ";";
      //cmd << std::setfill('0') << std::setw(2) << setup_data.kd << ";";
      cmd << (unsigned char)(setup_data.mode>>8);
      cmd << (unsigned char)(0x00FF&setup_data.mode);
      cmd << ";";
      cmd << (unsigned char)(setup_data.kp>>8);
      cmd << (unsigned char)(0x00FF&setup_data.kp);
      cmd << ";";
      cmd << (unsigned char)(setup_data.ki>>8);
      cmd << (unsigned char)(0x00FF&setup_data.ki);
      cmd << ";";
      cmd << (unsigned char)(setup_data.kd>>8);
      cmd << (unsigned char)(0x00FF&setup_data.kd);
      cmd << ";";
      cmd<< "\n";
      int n_written = write(fd,cmd.str().c_str(),cmd.str().size());
      //ROS_INFO("%s",cmd.str().c_str());
#endif

    //***************************************************
    // PUBLISH DATA
    //***************************************************

    // Transform
    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    t.transform.rotation = tf::createQuaternionMsgFromYaw(th);
    //t.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, th);

    // Odometry
    //msg_odom.header.frame_id=std::string("base_link");
    odom_msg.header.frame_id = odom;
    odom_msg.child_frame_id = base_link;

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = t.transform.rotation.x;
    odom_msg.pose.pose.orientation.y = t.transform.rotation.y;
    odom_msg.pose.pose.orientation.z = t.transform.rotation.z;
    odom_msg.pose.pose.orientation.w = t.transform.rotation.w;

    odom_msg.pose.covariance[0]  = 9.0;
    odom_msg.pose.covariance[7]  = 9.0;
    odom_msg.pose.covariance[14] = 9.0;
    odom_msg.pose.covariance[21] = 1.0;
    odom_msg.pose.covariance[28] = 1.0;
    odom_msg.pose.covariance[35] = 1.0;

    odom_msg.twist.twist.linear.x = v;
    //odom_msg.twist.twist.linear.y = 0.0;
    //odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.linear.y = (float(input_data.cpsL)/rpm2pwm)/cps2rads;
    odom_msg.twist.twist.linear.z = (float(input_data.cpsR)/rpm2pwm)/cps2rads;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = w;

    odom_msg.twist.covariance[0]  = 4.0;
    odom_msg.twist.covariance[7]  = 4.0;
    odom_msg.twist.covariance[14] = 4.0;
    odom_msg.twist.covariance[21] = 1.0;
    odom_msg.twist.covariance[28] = 1.0;
    odom_msg.twist.covariance[35] = 1.0;

    // Publish
    //t.header.stamp = nh.now();
    t.header.stamp = ros::Time::now();
    broadcaster.sendTransform(t);

    //msg_odom.header.stamp = ros::Time::now();
    odom_msg.header.stamp = t.header.stamp;
    odom_pub.publish(odom_msg);

    //printf("Iteration: %d\n",iteration);
    cnt++;
    iteration++;
    ros::spinOnce();
    loop_rate.sleep();
  }

#ifdef ARDUINO
  close(fd);
#endif
  printf("Quitting...\n");
}
