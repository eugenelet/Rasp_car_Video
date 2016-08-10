#include "../include/sift.h"


/**************************
        SIFT TOOLS
***************************/
#define PI 3.14159265358979323846264338327
#define SIFT_ORI_HIST_BINS 36// default number of bins in histogram for orientation assignment
#define SIFT_ORI_SIG 1.5f// determines gaussian sigma for orientation assignment
#define SIFT_ORI_RADIUS 3 * SIFT_ORI_SIG// determines the radius of the region used in orientation assignment


using namespace cv;
using namespace std;

int keyIn;
int sokt;

int localSocket;
int remoteSocket;
int serverPort;
int doneFlag = 1;
int counter = 0;
static char* snd_PORT="10023";
static char* rcv_PORT="10024";
//static char* IP_ADDR="127.0.0.1";
static char* IP_ADDR="192.168.0.100";

/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////


bool sigIntFlag = false;

void sigint(int a){
	cout << "SIG INT Captured!" << endl;
	cout << "Closing Socket Connections" << endl;
	unsigned char output[DATAGRAM_SIZE];
	output[0] = CONTROL;
	output[1] = SIG_INT;
	transmit(output);
	close_receiver();
	close_transmit();
	sigIntFlag = true;
}

void* getch_thread(void* data){
    unsigned char output[DATAGRAM_SIZE];
	unsigned char* receivedPacket;
    int bytes;
	bool wrongKey = false;
    while(1){
		tcflush(STDIN_FILENO, TCIFLUSH);	
		keyIn = getch();
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
            case 'c':{
				pthread_exit(0);
                break;
            }
			case 'h':{
				cout <<"Shut Down Pi..."<<endl;
				output[0] = CONTROL;
				output[1] = SHUTDOWN;
				transmit(output);
				break;
			}
            default:{
				cout <<"\n\n\n\n" ;
				cout << "================================" << endl;
                cout <<"W,A,S,D for Control." << endl <<
					"C to cancel."<<endl <<
					"H to Shutdown Pi." << endl;
				cout << "================================" << endl;
				cout <<"\n\n\n\n" ;
                wrongKey = true;
                break;
            }
        
        }
		if(wrongKey){
			wrongKey = false;
		}
		else{
			receivedPacket = receiver();
			cout << "ACK!" << endl;
		}
    }
    pthread_exit(NULL);
}

std::mutex mtxCam;

void task(VideoCapture* cap, Mat* frame){
	while(1){
		//cap->grab();
		mtxCam.lock();
		*cap >> *frame;
		mtxCam.unlock();
		imshow("Video Thread", *frame);
		waitKey(1);
	}
}



int main(int argc, char* argv[])
{
    

    pthread_t  getch_t, video_t;


    transmit_init(IP_ADDR, snd_PORT);  
    receiver_init(rcv_PORT); 
    pthread_create(&getch_t, NULL, getch_thread, (void*)&keyIn);

    signal(SIGINT, sigint);

    //----------------------------------------------------------
    //OpenCV Code
    //----------------------------------------------------------
    cv::VideoCapture vcap;
    vcap.set(CV_CAP_PROP_BUFFERSIZE, 3);
    cv::Mat frame;

//    const std::string videoStreamAddress = "http://192.168.0.100:8090/?action=stream"; 
    const std::string videoStreamAddress = "http://192.168.0.100:8080/stream/video.mjpeg"; 
    /* it may be an address of an mjpeg stream, 
    e.g. "http://user:pass@cam_address:8081/cgi/mjpg/mjpg.cgi?.mjpg" */

    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1; 
    }  

    vcap>>frame;
    //vcap.grab();
    thread t(task, &vcap, &frame);

    ///////////////////////////////////////////////////////////////////////////////////////////
    srand(time(NULL));
    bool time_on = false;
    bool multi_on = false;
    if (argc > 1)
        for(int i = 1; i < argc; i++){
            if(strcmp(argv[i], "time") == 0)
                time_on = true; 
            if(strcmp(argv[i], "multi") == 0)
                multi_on = true;    
            if(strcmp(argv[i], "-h") == 0){
                cout << "time: Measure time." << endl;
                cout << "multi: Multi target." << endl;
                return 0;
            }
        }
    ///////////////////////////////////////////////////////////////////////////////////////////

    clock_t start, end;
    int target_num = 3;
    char** targetFile = new char*[target_num];//"target.jpg";
    mySIFT* sift_target = new mySIFT[target_num];
    char* str1 = "target/target";
    char* str2 = ".jpg";
    Mat* target = new Mat[target_num];
    char buffer[10];
    if(multi_on){
        for(int i = 0; i < target_num; i++){
            sift_target[i] = mySIFT(1.414, 1.414, 3);//sigma k
            sprintf(buffer, "%d", i);
            char * tmp = (char *) malloc(1 + strlen(str1) + strlen(buffer) + strlen(str2) );
            strcpy(tmp, str1);
            strcat(tmp, buffer);
            strcat(tmp, str2);
            targetFile[i] = tmp;
            target[i] = imread(targetFile[i], CV_LOAD_IMAGE_GRAYSCALE);
            computeSift(sift_target[i], target[i], imread(targetFile[i]), time_on);
        }
    }
    else{
        sift_target[0] = mySIFT(1.414, 1.414, 3);//sigma k
        targetFile[0] = "target/target.jpg";
        target[0] = imread(targetFile[0], CV_LOAD_IMAGE_GRAYSCALE);
        computeSift(sift_target[0], target[0], imread(targetFile[0]), time_on);
    }

    while (1){
        Mat img_scene;

        if(sigIntFlag){
            pthread_cancel(getch_t);
            return 0;
        }

        if(!frame.empty()){
            mtxCam.lock();
            frame.copyTo(img_scene);
            mtxCam.unlock();
        }

        clock_t s;
        s = clock();
        // Mat img_scene;
        // cap >> img_scene;
        clock_t start, end;

        mySIFT hoho(1.414, 1.414, 1);//sigma k
        //hoho.LoadImage(imageName2);
        Mat imgScene;
        cvtColor(img_scene, imgScene, CV_BGR2GRAY);
        computeSift(hoho, imgScene, img_scene, time_on);
        //start = clock();
        if(multi_on)
            match_multi(sift_target, hoho, targetFile, img_scene, target_num, 0);
        else
            match(sift_target[0], hoho, targetFile[0], img_scene, s);

        //end = clock();
        //cout << "Match : " << (double)(end - s) / CLOCKS_PER_SEC << "\n";

    }

    //system("pause");
    return 0;
}

void mySIFT::LoadImage(Mat imgOri)
{
    blurredImgs.push_back(imgOri);//­ì¹Ï¬O©ñindex0
}


	
