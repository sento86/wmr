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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

//#define ARDUINO
#define IGNORE_CRC

#ifndef ARDUINO
    #define NETHAT
    #ifdef NETHAT

        #include "cifxlinux.h"
        #include "cifXEndianess.h"
        #include "rcX_Public.h"
        #include <errno.h>
        #include <fcntl.h>
        #include <string.h>
        #include <stdio.h>
        #include <stdlib.h>
        #include <termios.h>
        #include <unistd.h>
        #include <sys/mman.h>
        #include <time.h>
        #include <signal.h>
        //#include "mariadb/mysql.h"

        #define CIFX_DEV "cifX0"

        #ifndef UNREFERENCED_PARAMETER
          #define UNREFERENCED_PARAMETER(a) (a=a)
        #endif

    #endif
#endif

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
#define TWIST_STAMPED
#ifdef TWIST_STAMPED
geometry_msgs::TwistStamped twist_msg;
#else
geometry_msgs::Twist twist_msg;
#endif
sensor_msgs::LaserScan laser_msg;
sensor_msgs::PointCloud2 points_msg;

geometry_msgs::TransformStamped t;

ros::Time time_now, time_old;

#ifdef NETHAT
// PROEMISA DEFINITIONS
CIFXHANDLE hChannel = NULL;
CIFXHANDLE hDriver = NULL;
int32_t    lRet;
#endif

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

#ifdef NETHAT

/*****************************************************************************/
/*! Displays cifX error
*   \param lError     Error code                                             */
/*****************************************************************************/
void ShowError( int32_t lError )
{
  if( lError != CIFX_NO_ERROR)
  {
    char szError[1024] ={0};
    xDriverGetErrorDescription( lError,  szError, sizeof(szError));
    printf("Error: 0x%X, <%s>\n", (unsigned int)lError, szError);
  }
}


/*****************************************************************************/
/*! Displays a hex dump on the debug console (16 bytes per line)
*   \param pbData     Pointer to dump data
*   \param ulDataLen  Length of data dump                                    */
/*****************************************************************************/
void DumpData(unsigned char* pbData, unsigned long ulDataLen)
{
	unsigned long ulIdx;
#ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
#endif
  for(ulIdx = 0; ulIdx < ulDataLen; ++ulIdx)
  {
    if(0 == (ulIdx % 16))
      printf("\r\n");

    printf("%02X ", pbData[ulIdx]);
  }
  printf("\r\n");
}


/*****************************************************************************/
/*! Dumps a rcX packet to debug console
*   \param ptPacket Pointer to packed being dumped                           */
/*****************************************************************************/
void DumpPacket(CIFX_PACKET* ptPacket)
{
#ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
#endif
  printf("Dest   : 0x%08lX      ID   : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulDest),  (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulId));
  printf("Src    : 0x%08lX      Sta  : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulSrc),   (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulState));
  printf("DestID : 0x%08lX      Cmd  : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulDestId),(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulCmd));
  printf("SrcID  : 0x%08lX      Ext  : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulSrcId), (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulExt));
  printf("Len    : 0x%08lX      Rout : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulLen),   (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulRout));

  printf("Data:");
  DumpData(ptPacket->abData, HOST_TO_LE32(ptPacket->tHeader.ulLen));
}


/*****************************************************************************/
/*! Function to display driver information
 *   \param  hDriver  Handle to cifX driver
 *   \param  ptVTable Pointer to cifX API function table
 *   \return CIFX_NO_ERROR on success                                        */
/*****************************************************************************/
void DisplayDriverInformation (void)
{
  int32_t            lRet             = CIFX_NO_ERROR;
  DRIVER_INFORMATION tDriverInfo      = {{0}};
  char               szDrvVersion[32] = "";
  CIFXHANDLE         hDriver          = NULL;
 
  if (CIFX_NO_ERROR == (lRet = xDriverOpen(&hDriver)))
  {
    printf("\n---------- Display Driver Version ----------\n");
    if( CIFX_NO_ERROR != (lRet = xDriverGetInformation(NULL, sizeof(tDriverInfo), &tDriverInfo)) )
      ShowError( lRet);
    else if ( CIFX_NO_ERROR != (lRet = cifXGetDriverVersion( sizeof(szDrvVersion)/sizeof(*szDrvVersion), szDrvVersion)))
      ShowError( lRet);
    else
      printf("Driver Version: %s, based on %.32s \n\n", szDrvVersion, tDriverInfo.abDriverVersion);
    
    /* close previously opened driver */
    xDriverClose(hDriver);
    
  } 
  
  printf(" State = 0x%08X\r\n", (unsigned int)lRet);
  printf("----------------------------------------------------\r\n");
}

/*****************************************************************************/
/*! Function to demonstrate communication channel functionality
*   Packet Transfer and I/O Data exchange
*   \return CIFX_NO_ERROR on success                                         */
/*****************************************************************************/

#ifdef __cplusplus
  extern "C" {
#endif  /* __cplusplus */

      /* Read and write I/O data (32Bytes). Output data will be incremented each cycle */
      uint8_t  abSendData[32] = {0};
      uint8_t  abRecvData[32] = {0};
      uint8_t  fRunning       = 1;
      uint32_t ulState        = 0;        /* Actual state returned                    */
      uint8_t  bIdx           = 0;
	  int testval 			  = 0;
	  uint8_t  sendPointer    = 0;
	  uint8_t  recvPointer     = 0;
	  	  
	  //  Señales comunicacion con PLC
	  char	   	Color_Ruedas	  ='0';
	  char	   	Color_Barra	  ='0';
	  char	   	Color_Circ	  ='0';
	  char		Ctrl_Char_Ruedas = Color_Ruedas;
	  char		Ctrl_Char_Barra = Color_Barra;
	  char		Ctrl_Char_Circ = Color_Circ;
	  uint8_t PLC_ByteVida = 0;
	  uint8_t PLC_Senal_Apagado = 0;
	  uint8_t Byte6_IN = 0;
	  int16_t Word1_IN = 0;
	  int16_t Word2_IN = 0;
	  int16_t Word3_IN = 0;
	  int16_t Word4_IN = 0;
	  int16_t Word5_IN = 0;
	  int16_t Vel_MotorLH = 0;
	  int16_t Vel_MotorRH = 0;
	  int16_t DiametroRuedas = 0;
	  int16_t DistanciaRuedas = 0;


	  void send_Byte(uint8_t valor){
		  abSendData[sendPointer] = valor & 0xFF;
		  sendPointer = sendPointer + 1;
	  }
	  
	  void send_Word(int16_t valor){
		  abSendData[sendPointer    ] = (valor >> 8) & 0xFF;
		  abSendData[sendPointer + 1] =  valor & 0xFF;
		  sendPointer = sendPointer + 2;
	  }
	  void send_DWord(int32_t valor){
		  abSendData[sendPointer	] = (valor >> 24) & 0xFF;
		  abSendData[sendPointer + 1] = (valor >> 16) & 0xFF;
		  abSendData[sendPointer + 2] = (valor >> 8)  & 0xFF;
		  abSendData[sendPointer + 3] =  valor & 0xFF;
		  sendPointer = sendPointer + 4;
	  }
	  uint8_t rcv_Byte(){
		  uint8_t valor = 0;
		  valor = abRecvData[recvPointer];
		  recvPointer = recvPointer + 1;
		  return valor;
	  }
	  
	  int16_t rcv_Word(){
		  int16_t valor = 0;
		  valor = valor |= (abRecvData[recvPointer    ]) << 8;
		  valor = valor |= (abRecvData[recvPointer + 1]);
		  recvPointer = recvPointer + 2;
		  return valor;
	  }
	  
	  int32_t rcv_DWord(){
		  int32_t valor = 0;
		  valor = (valor |= (abRecvData[recvPointer    ])) << 8;
		  valor = (valor |= (abRecvData[recvPointer + 1])) << 8;
		  valor = (valor |= (abRecvData[recvPointer + 2])) << 8;
		  valor =  valor |= (abRecvData[recvPointer + 3]);
		  recvPointer = recvPointer + 4;
		  return valor;
	  }



#ifdef __cplusplus
}
#endif

#endif

#ifdef TWIST_STAMPED
/** Callback for joy topic **/
void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    twist_msg = *msg;
    vref = twist_msg.twist.linear.x;
    wref = twist_msg.twist.angular.z;
    //if(vref<0.0)
    //  wref = -wref;
}
#else
/** Callback for joy topic **/
void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    twist_msg = *msg;
    vref = twist_msg.linear.x;
    wref = twist_msg.angular.z;
    //if(vref<0.0)
    //  wref = -wref;
}
#endif
/** Callback for laser topic **/
void laserScan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_msg = *msg;
}

/** Callback for points topic **/
void pointCloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    points_msg = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "koby_node");
  ROS_INFO("Initializing KOBY Node Test");
  ros::NodeHandle nh, np("~");

  // Get parameters from server
  getParameters(nh, np, &config);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 0);
  #ifdef TWIST_STAMPED
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, twist_cb);
  #else
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, twist_cb);
  #endif
  ros::Subscriber laser_sub = nh.subscribe("scan", 10, laserScan_cb);
  ros::Subscriber points_sub = nh.subscribe("/camera/depth/color/points", 10, pointCloud_cb);

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

#ifdef NETHAT
/*
  struct CIFX_LINUX_INIT init =
  {
    .init_options        = CIFX_DRIVER_INIT_AUTOSCAN,
    .iCardNumber         = 0,
    .fEnableCardLocking  = 0,
    .base_dir            = NULL,
    .poll_interval       = 0,
    .poll_StackSize      = 0,   // set to 0 to use default
    .trace_level         = 255,
    .user_card_cnt       = 0,
    .user_cards          = NULL,
  };
*/	
  struct CIFX_LINUX_INIT init =
  {
    .init_options        = CIFX_DRIVER_INIT_AUTOSCAN,
    .base_dir            = NULL,
    .poll_interval       = 0,
    .trace_level         = 255,
    .user_card_cnt       = 0,
    .user_cards          = NULL,
    .iCardNumber         = 0,
    .fEnableCardLocking  = 0,
    .poll_StackSize      = 0,   /* set to 0 to use default */
  };
  
#ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
#endif

  /* First of all initialize toolkit */
  int32_t global_lRet = cifXDriverInit(&init);

  if(CIFX_NO_ERROR == global_lRet){

  #ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
  #endif
  //CIFXHANDLE hDriver = NULL;
  lRet    = xDriverOpen(&hDriver);

  printf("---------- Communication Channel demo ----------\r\n");

  if(CIFX_NO_ERROR == lRet)
  {
    /* Driver/Toolkit successfully opened */
    //CIFXHANDLE hChannel = NULL;
    lRet = xChannelOpen(NULL, CIFX_DEV, 0, &hChannel);

    if(CIFX_NO_ERROR != lRet)
    {
      printf("Error opening Channel!");

    } else
    {
    
      CHANNEL_INFORMATION tChannelInfo = {{0}};

      /* Channel successfully opened, so query basic information */
      if( CIFX_NO_ERROR != (lRet = xChannelInfo(hChannel, sizeof(CHANNEL_INFORMATION), &tChannelInfo)))
      {
        printf("Error querying system information block\r\n");
      } else
      {
        printf("Communication Channel Info:\r\n");
        printf("Device Number    : %lu\r\n",(long unsigned int)tChannelInfo.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n",(long unsigned int)tChannelInfo.ulSerialNumber);
        printf("Firmware         : %s\r\n", tChannelInfo.abFWName);
        printf("FW Version       : %u.%u.%u build %u\r\n", 
                tChannelInfo.usFWMajor,
                tChannelInfo.usFWMinor,
                tChannelInfo.usFWRevision,
                tChannelInfo.usFWBuild);
        printf("FW Date          : %02u/%02u/%04u\r\n", 
                tChannelInfo.bFWMonth,
                tChannelInfo.bFWDay,
                tChannelInfo.usFWYear);

        printf("Mailbox Size     : %lu\r\n",(long unsigned int)tChannelInfo.ulMailboxSize);
      }
    }
  }
  }
#endif

  //***********************************
  output_data.pwmL = 0;
  output_data.pwmR = 0;

  float rpm2pwm  = 255.0 / (2.5 * 2.0*M_PI);
  float cps2rads = 2.0*(2.0*M_PI/36.0); // rad/s -> encoder*2 [counts/s] * 2*pi [rad/rev] / 36 [counts/rev]
  float rpm2rads = (2.0*M_PI)/60.0; // converssion from rpm to rads
  //output_data.cpsL = int16_t((input_data.pwmL/rpm2pwm)/cps2rads);
  //output_data.cpsR = int16_t((input_data.pwmR/rpm2pwm)/cps2rads);

  ros::Rate loop_rate(config.publishRate); //Desired frequency in Hz
  
  time_old = ros::Time::now(); // Save current time for control period
  
  // Loop
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


    wL = float(input_data.cpsL)*cps2rads;//2.0*(2.0*M_PI/36.0); // rad/s -> encoder*2 [counts/s] * 2*pi [rad/rev] / 36 [counts/rev]
    //wL = -wL; // Opposite sign
    wR = float(input_data.cpsR)*cps2rads;//2.0*(2.0*M_PI/36.0); // rad/s -> encoder*2 [counts/s] * 2*pi [rad/rev] / 36 [counts/rev]
    //wR = -wR; // Opposite sign (already implemented in Arduino)
    
    #else
    
    new_message=1;

#endif
#ifdef NETHAT
    //PROEMISA CODE
    if(CIFX_NO_ERROR == global_lRet){

    CIFXHANDLE hDriver = NULL;
    int32_t    lRet    = xDriverOpen(&hDriver);

      /* Wait for communication to be established */
        if( CIFX_DEV_NO_COM_FLAG == ( lRet = xChannelBusState( hChannel, CIFX_BUS_STATE_ON, &ulState, 10 ) ) ){
          printf("Waiting for Bus communication!\r\n");
        }
        else{
          /* Read Data from network */
          if(CIFX_NO_ERROR != (lRet = xChannelIORead(hChannel, 0, 0, sizeof(abRecvData), abRecvData, 10))){
            printf("Error reading IO Data area!\r\n");
          } else{
            //printf("IO Read Data:");
			recvPointer  = 0;
			
			// TIPO DE DATOS INPUTS
			PLC_ByteVida 	= rcv_Byte();
			Color_Ruedas	= rcv_Byte();
			Color_Barra 	= rcv_Byte();
			Color_Circ 	= rcv_Byte();
			PLC_Senal_Apagado = rcv_Byte();
			Byte6_IN = rcv_Byte();
			Word1_IN = rcv_Word();
			Word2_IN = rcv_Word();
			Word3_IN = rcv_Word();
			Word4_IN = rcv_Word();
			Word5_IN = rcv_Word();
			Vel_MotorLH = rcv_Word();
			Vel_MotorRH = rcv_Word();
			DiametroRuedas = rcv_Word();
    			DistanciaRuedas = rcv_Word();
			
			if((Color_Ruedas != Ctrl_Char_Ruedas || Color_Barra != Ctrl_Char_Barra || Color_Circ != Ctrl_Char_Circ)){
				Ctrl_Char_Ruedas = Color_Ruedas;
				Ctrl_Char_Barra = Color_Barra;
				Ctrl_Char_Circ = Color_Circ;
				
				/*mysql_connect();
				
				if(mysql1 != NULL){
					SQL_Data[39] = Color_Ruedas;
					SQL_Data[56] = Color_Barra;
					SQL_Data[73] = Color_Circ;
					if (mysql_query(mysql1, SQL_Data)){
					fprintf(stderr, "%s\n", mysql_error(mysql1));
					}
					
				}
				
				mysql_disconnect();*/
			}
			
			/*if(PLC_Senal_Apagado){
				printf("Shutting Down...\n");
				sleep(2);
				system("sudo shutdown now");
				exit(0);
			}*/
		}
	
         wL = float((Vel_MotorLH)/10)*rpm2rads;
         wR = float((Vel_MotorRH)/10)*rpm2rads;
         
         printf("VelMot_RH: %i//VelMot_LH: %i\n",Vel_MotorRH,Vel_MotorLH);
         printf("WR: %f// WL: %f\n",wR,wL);
         
      }
    }
    new_message=1;
#endif

    if (new_message ==1 )
    {

      time_now = ros::Time::now(); // Save current time for control period
      dt = (time_now - time_old).toSec(); // Time delay is the control period
      time_old = time_now; // Update previous time with current timestamp
        
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
      printf("vref: %f// wref: %f\n",vref,wref);

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
      
      
      uL = (int16_t) (wLref + kp*(wLref-wL));
      uR = (int16_t) (wRref + kp*(wRref-wR));
      
      printf("uR: %f// uL: %f\n",uR,uL);
      
      #ifdef ARDUINO
        uL = uL * rpm2pwm;
        uR = uR * rpm2pwm;
      #endif
      

      //uL = uL+(int16_t) kp*(wLref-wL) * rpm2pwm;
      //uR = uR+(int16_t) kp*(wRref-wR) * rpm2pwm;

      //output_data.pwmL = std::min(255,std::max(-255,uL));
      //output_data.pwmR = std::min(255,std::max(-255,uR));

      //output_data.pwmL = uL;
      //output_data.pwmR = uR;

      #ifdef ARDUINO
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
     #endif

      //output_data.pwmL = (int16_t)(vref*254);
      //output_data.pwmR = (int16_t)(vref*254);

      //updateControl = false;

      //output_data.pwmL = input_data.cpsL;//+1;
      //output_data.pwmR = input_data.cpsR+1;

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
   #ifdef NETHAT
     if(CIFX_NO_ERROR == global_lRet)
  {
    //PROEMISA CODE
     /* Wait for communication to be established */
        if( CIFX_DEV_NO_COM_FLAG == ( lRet = xChannelBusState( hChannel, CIFX_BUS_STATE_ON, &ulState, 10 ) ) ){
          printf("Waiting for Bus communication!\r\n");
        }
        else{
    /* Write Data to network */
            if(CIFX_NO_ERROR != (lRet = xChannelIOWrite(hChannel, 0, 0, sizeof(abSendData), abSendData, 10))){
              printf("Error writing to IO Data area!\r\n");
            } else{
			  //printf("IO Write Data:");
			  sendPointer = 0;
			  
			  send_Byte(PLC_ByteVida);
			  send_Byte(0);
			  send_Word(0);
			  send_Word(0);
			  send_Word(0);
			  send_Word(0);
			  send_Word(0);
			  send_Word(0);
			  send_Word(0);
			  send_Byte(0);
			  send_Byte(0);
			  send_Word(uL*10);
			  send_Word(uR*10);	  
			  
			 }
	}		    
    }
   
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
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    //odom_msg.twist.twist.linear.y = (float(input_data.cpsL)/rpm2pwm)/cps2rads;
    //odom_msg.twist.twist.linear.z = (float(input_data.cpsR)/rpm2pwm)/cps2rads;
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

#ifdef NETHAT
  xChannelClose(hChannel);
  xDriverClose(hDriver);
  //cifXDriverDeinit();
#endif
  printf("Quitting...\n");
}
