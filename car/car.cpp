/**
 * OpenCV video streaming over TCP/IP
 * Server: Captures video from a webcam and send it to a client
 * by Isaac Maia
 */

#include "opencv2/opencv.hpp"
#include <iostream>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> 
#include <string.h>
#include <stdio.h>
#include "receiver.h"
#include "transmit.h"
#include "GPIOClass.h"

using namespace cv;
using namespace std;


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


GPIOClass* frontLeft1; 
GPIOClass* frontLeft2;
GPIOClass* frontRight1;
GPIOClass* frontRight2;
GPIOClass* rearLeft1;
GPIOClass* rearLeft2;
GPIOClass* rearRight1; 
GPIOClass* rearRight2;

unsigned char output[DATAGRAM_SIZE];

void idle(){
	cout<< "IDLE" << endl;
	frontLeft1->setval_gpio("1"); 
	frontLeft2->setval_gpio("1"); 
	frontRight1->setval_gpio("1");
	frontRight2->setval_gpio("1");
	rearLeft1->setval_gpio("1"); 
	rearLeft2->setval_gpio("1"); 
	rearRight1->setval_gpio("1");
	rearRight2->setval_gpio("1");
}

void forward(){
	cout << "FORWARD" << endl;
	frontLeft1->setval_gpio("0"); 
	frontLeft2->setval_gpio("1"); 
	frontRight1->setval_gpio("0");
	frontRight2->setval_gpio("1");
	rearLeft1->setval_gpio("0"); 
	rearLeft2->setval_gpio("1"); 
	rearRight1->setval_gpio("0");
	rearRight2->setval_gpio("1");
	transmit(output);
}

void back(){
	cout << "BACK" << endl;
	frontLeft1->setval_gpio("1"); 
	frontLeft2->setval_gpio("0"); 
	frontRight1->setval_gpio("1");
	frontRight2->setval_gpio("0");
	rearLeft1->setval_gpio("1"); 
	rearLeft2->setval_gpio("0"); 
	rearRight1->setval_gpio("1");
	rearRight2->setval_gpio("0");
	transmit(output);
}

void left(){
	cout << "LEFT" << endl;
	frontLeft1->setval_gpio("1"); 
	frontLeft2->setval_gpio("0"); 
	frontRight1->setval_gpio("0");
	frontRight2->setval_gpio("1");
	rearLeft1->setval_gpio("1"); 
	rearLeft2->setval_gpio("0"); 
	rearRight1->setval_gpio("0");
	rearRight2->setval_gpio("1");
	transmit(output);
}

void right(){
	cout << "RIGHT" << endl;
	frontLeft1->setval_gpio("0"); 
	frontLeft2->setval_gpio("1"); 
	frontRight1->setval_gpio("1");
	frontRight2->setval_gpio("0");
	rearLeft1->setval_gpio("0"); 
	rearLeft2->setval_gpio("1"); 
	rearRight1->setval_gpio("1");
	rearRight2->setval_gpio("0");
	transmit(output);
}




static char* snd_PORT = "2022"; 
static char* rcv_PORT="2023";
static char* IP_ADDR="192.168.2.101";

int requestFlag = 0;
int main(int argc, char** argv){
    unsigned char* receivedPacket;
    int bytes, opcode;
    receiver_init(rcv_PORT);
	transmit_init(IP_ADDR, snd_PORT);

    frontLeft1 = new GPIOClass("19"); //create new GPIO object to be attached to  GPIO4
    frontLeft2 = new GPIOClass("26"); //create new GPIO object to be attached to  GPIO4
    frontRight1 = new GPIOClass("4"); //create new GPIO object to be attached to  GPIO17
    frontRight2 = new GPIOClass("17"); //create new GPIO object to be attached to  GPIO17
    rearLeft1 = new GPIOClass("6"); //create new GPIO object to be attached to  GPIO4
    rearLeft2 = new GPIOClass("13"); //create new GPIO object to be attached to  GPIO4
    rearRight1 = new GPIOClass("2"); //create new GPIO object to be attached to  GPIO17
    rearRight2 = new GPIOClass("3"); //create new GPIO object to be attached to  GPIO17

    frontLeft1->export_gpio(); //export GPIO4
    frontLeft2->export_gpio(); //export GPIO4
    frontRight1->export_gpio(); //export GPIO4
    frontRight2->export_gpio(); //export GPIO4
    rearLeft1->export_gpio(); //export GPIO4
    rearLeft2->export_gpio(); //export GPIO4
    rearRight1->export_gpio(); //export GPIO4
    rearRight2->export_gpio(); //export GPIO4


    frontLeft1->setdir_gpio("out");
    frontLeft2->setdir_gpio("out");
    frontRight1->setdir_gpio("out");
    frontRight2->setdir_gpio("out");
    rearLeft1->setdir_gpio("out");
    rearLeft2->setdir_gpio("out");
    rearRight1->setdir_gpio("out");
    rearRight2->setdir_gpio("out");
   int counter = 0; 

    while(1){
	cout << counter ++ << endl;
		usleep(100000);
		idle();
        receivedPacket = receiver();
        //cout << receivedPacket[0]<<endl;      
        if((receivedPacket[0] == CONTROL)){
            opcode = CONTROL;   
        }       
        else{opcode = NULL;}

        switch(opcode){
            case CONTROL:{
                    if((receivedPacket[1] == FORWARD)){
        		        forward();
                    }
                    else if((receivedPacket[1] == BACK)){
                        back();
                    }
                    else if((receivedPacket[1] == LEFT)){
                        left();
                    }
                    else if((receivedPacket[1] == RIGHT)){
                        right();
                    }
                    else {
                        idle();
                    }
                    
                    break;  
                }

            default:{}



        }
    }
	return 0;

}



//int main(int argc, char** argv)
//{	
//    pthread_t  control_t;
//
//    pthread_create(&control_t, NULL, controlThread, NULL);
//
//    while(1) {}
//
//    return 0;
//}
