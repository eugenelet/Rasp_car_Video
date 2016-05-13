/**
 * OpenCV video streaming over TCP/IP
 * Client: Receives video from server and display it
 * by Isaac Maia
 */

#define DATAGRAM_SIZE 8


/*************************************
        OP CODE
**************************************/

#define CONTROL 0x01
#define PICTURE 0x02
#define VIDEO   0x03

/*************************************
        FUNCTION
**************************************/


/**************************
        CONTROL
***************************/
#define IDLE    0x00
#define FORWARD 0x01
#define BACK    0x02
#define LEFT    0x03
#define RIGHT   0x04


/**************************
        VIDEO
***************************/
#define START   0x00
#define VALID   0x01
#define REQUEST 0x02
#define STOP    0x03
#define SENT    0x04


#include "opencv2/opencv.hpp"
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "getChar.cpp"
#include "transmit.h"
#include "receiver.h"
#include "kbhit.cpp"
#include <termios.h>

using namespace cv;
using namespace std;

int keyIn;
int sokt;

int localSocket;
int remoteSocket;
int serverPort;
int doneFlag = 1;
int counter = 0;
static char* snd_PORT="2023";
static char* rcv_PORT="2022";
//static char* IP_ADDR="127.0.0.1";
static char* IP_ADDR="192.168.2.100";


void* getch_thread(void* data){
    unsigned char output[DATAGRAM_SIZE];
	unsigned char* receivedPacket;
    int bytes;
	transmit_init(IP_ADDR, snd_PORT);   
	receiver_init(rcv_PORT); 
    while(1){
		tcflush(STDIN_FILENO, TCIFLUSH);	
		keyIn = getchar();
        switch(keyIn){
            case 'a':{
                cout <<"left"<<endl;
                output[0] = CONTROL;
                output[1] = LEFT;
                transmit(output);
                break;
            }
            case 's':{
                cout <<"back"<<endl;
                output[0] = CONTROL;
                output[1] = BACK;
                transmit(output);
                break;
            }
            case 'd':{
                cout <<"right"<<endl;
                output[0] = CONTROL;
                output[1] = RIGHT;
                transmit(output);
                break;
            }
            case 'w':{
                cout <<"forward"<<endl;
                output[0] = CONTROL;
                output[1] = FORWARD;
                transmit(output);
                break;
            }
            case 'v':{
               //  //   transmit(output);
               // }
                break;
            }
            default:{
                cout <<"idle"<<endl;
                output[0] =  CONTROL;
                output[1] = IDLE;
                transmit(output);
                break;
            }
        
        }
		receivedPacket = receiver();
		cout << "ACK!" << endl;
    }
    pthread_exit(NULL);
}



int main(int argc, char** argv)
{

    pthread_t  getch_t, video_t;


    pthread_create(&getch_t, NULL, getch_thread, (void*)&keyIn);


    //----------------------------------------------------------
    //OpenCV Code
    //----------------------------------------------------------
    cv::VideoCapture vcap;
    cv::Mat image;

    const std::string videoStreamAddress = "http://192.168.2.100:8080/?action=stream"; 
    /* it may be an address of an mjpeg stream, 
    e.g. "http://user:pass@cam_address:8081/cgi/mjpg/mjpg.cgi?.mjpg" */

    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1; 
    }   

    //Create output window for displaying frames. 
    //It's important to create this window outside of the `for` loop
    //Otherwise this window will be created automatically each time you call
    //`imshow(...)`, which is very inefficient. 
    cv::namedWindow("Output Window");

    for(;;) {
        if(!vcap.read(image)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
        }   
        cv::imshow("Output Window", image);
//        if(cv::waitKey(500) >= 0) break;                                                                                                     }   
		cv::waitKey(1);
	}
    return 0;
}
	
