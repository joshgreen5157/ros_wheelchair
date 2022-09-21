#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <thread>

 


using namespace std;
using namespace cv;
using namespace raspicam;

Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate;
Mat ROILane;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, Result;

RaspiCam_Cv Camera;

stringstream ss;
string input = "a";
bool written = false;


vector<int> histrogramLane;

Point2f Source[] = {Point2f(30,150),Point2f(330,150),Point2f(20,210),Point2f(345,210)};
Point2f Destination[] = {Point2f(80,0),Point2f(280,0),Point2f(80,240),Point2f(280,240)};

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
{
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,360 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,100));

}

void commandThread(){
	while (1){
		cin >> input;
		written = false;
	}
}
 
void Capture()
{
    Camera.grab();
    Camera.retrieve( frame);
    cvtColor(frame, frame, COLOR_BGR2RGB); 
}

void Perspective()
{
	line(frame,Source[0], Source[1], Scalar(0,0,255), 2);
	line(frame,Source[1], Source[3], Scalar(0,0,255), 2);
	line(frame,Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame,Source[2], Source[0], Scalar(0,0,255), 2);


	Matrix = getPerspectiveTransform(Source, Destination);
	warpPerspective(frame, framePers, Matrix, Size(360,240));
}

void Threshold(){
	cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
	inRange(frameGray, 100, 255, frameThresh);
	Canny(frameGray,frameEdge, 100, 500, 3, false);
	add(frameThresh, frameEdge, frameFinal);
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
}

void Histrogram()
{
    histrogramLane.resize(360);
    histrogramLane.clear();
   
    for(int i=0; i<360; i++)       //frame.size().width = 400
    {
		ROILane = frameFinalDuplicate(Rect(i,140,1,100));
		divide(255, ROILane, ROILane);
		histrogramLane.push_back((int)(sum(ROILane)[0]));
    }
}

void LaneFinder()
{
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 100);
    LeftLanePos = distance(histrogramLane.begin(), LeftPtr);
   
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histrogramLane.begin() +275, histrogramLane.end());
    RightLanePos = distance(histrogramLane.begin(), RightPtr);
   
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0, 255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);
}

void LaneCenter()
{
    laneCenter = (RightLanePos-LeftLanePos)/2 +LeftLanePos;
    frameCenter = 181.5;
   
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);

    Result = laneCenter-frameCenter;
}

int fd;


void setupComPort(const string comPort){
	
	fd = open(comPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        throw std::runtime_error("Failed to open port!");
    }
	struct termios config;
    tcgetattr(fd, &config);

    // Set baudrate
    cfsetispeed(&config, B9600);
    cfsetospeed(&config, B9600);

    // 9600 8N1
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CSIZE;
    config.c_cflag |=  CS8;

    // Disable hardware based flow control
    config.c_cflag &= ~CRTSCTS;

    // Enable receiver
    config.c_cflag |= CREAD | CLOCAL;                               

    // Disable software based flow control
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Termois Non Cannoincal Mode 
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 

    // Minimum number of characters for non cannoincal read
    config.c_cc[VMIN]  = 1;

    // Timeout in deciseconds for read
    config.c_cc[VTIME] = 0; 

    // Save config
    if (tcsetattr(fd, TCSANOW, &config) < 0)                        
    {
        close(fd);
        throw std::runtime_error("Failed to configure port!");
    }

    // Flush RX Buffer
    if (tcflush(fd, TCIFLUSH) < 0)
    {
        close(fd);
        throw std::runtime_error("Failed to flush buffer!");
    }
	
}

void write(std::string message){
		int length = message.size();
		if (length > 100)
		{
			throw std::invalid_argument("Message may not be longer than 100 bytes!");
		}
		char msg[101];
		strcpy(msg, message.c_str());
		int bytesWritten = write(fd, msg, length);
}


void stop(){
	cout <<"stop";	
}


void autonomous( ){
	
     
    
   
    while(input == "a")
    {
		
		auto start = std::chrono::system_clock::now();

		Capture();
		Perspective();
		Threshold();
		Histrogram();
		LaneFinder();
		LaneCenter();
		
    
		if (Result >= -3 && Result <= 3){
			cout<<"Forward"<<endl;
			write("0");
		}
		else if (Result > 3){
			cout<<"Right"<<endl;
			write("1");
		}
        else if (Result <-3){
			cout<<"left"<<endl;
			write("2");
		}
	else{
			cout<<"stop"<<endl;
			write("2");
		}
    
		ss.str(" ");
		ss.clear();
		ss<<"Result = "<<Result;
		putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
	   
	   
		namedWindow("orignal", WINDOW_KEEPRATIO);
		moveWindow("orignal", 0, 100);
		resizeWindow("orignal", 640, 480);
		imshow("orignal", frame);
	   
		namedWindow("Perspective", WINDOW_KEEPRATIO);
		moveWindow("Perspective", 640, 100);
		resizeWindow("Perspective", 640, 480);
		imshow("Perspective", framePers);
	   
		namedWindow("Final", WINDOW_KEEPRATIO);
		moveWindow("Final", 1280, 100);
		resizeWindow("Final", 640, 480
		);
		imshow("Final", frameFinal);
	   
	   
		waitKey(1);
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
	   
		float t = elapsed_seconds.count();
		int FPS = 1/t;
		cout<<"FPS = "<<FPS<<endl;
		
		
		sleep(.001);
    }
	
	
	
	}

void mainThread(){
	
	while(1){
		
		if(input == "s" && written == false){
			cout<<"I received a stop command"<<endl;
			write("s");
			written = true;
		}
		if(input == "b" && written == false){
			cout<<"I received a Begin command"<<endl;
			write("b");
			written = true;
		}
		if(input == "a" && written == false){
			cout<<"I received an auto command"<<endl;
			autonomous();
			written = true;
		}
		
		if(input == "i" && written == false){
			cout<<"I received an on command"<<endl;
			write("i");
			written = true;
		}
		
		if(input == "o" && written == false){
			cout<<"I received an off command"<<endl;
			write("o");
			written = true;
		}
		
		
		sleep(1);
	}
}










int main(int argc,char **argv)
{

	wiringPiSetup();
	//Setup ComPort 
	string comportString = "/dev/ttyACM0";
	setupComPort(comportString);
	
	
	
	Setup(argc, argv, Camera);
	cout<<"Connecting to camera"<<endl;
	if (!Camera.open())
	{
		cout<<"Failed to Connect"<<endl;
    }
     
     cout<<"Camera Id = "<<Camera.getId()<<endl;
     
	
	thread command(commandThread);
	thread mainProgram(mainThread);
	
	command.join();
	mainProgram.join();

	

   
    return 0;
}


