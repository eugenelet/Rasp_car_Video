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
#define SIG_INT 0x05
#define SHUTDOWN 0x06

/**************************
        VIDEO
***************************/
#define START   0x00
#define VALID   0x01
#define REQUEST 0x02
#define STOP    0x03
#define SENT    0x04


/**************************
        SIFT TOOLS
***************************/
#define PI 3.14159265358979323846264338327
#define SIFT_ORI_HIST_BINS 36// default number of bins in histogram for orientation assignment
#define SIFT_ORI_SIG 1.5f// determines gaussian sigma for orientation assignment
#define SIFT_ORI_RADIUS 3 * SIFT_ORI_SIG// determines the radius of the region used in orientation assignment




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

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc.hpp>
//#include <legacy/legacy.hpp>
#include <ctime>
#include <iomanip>
#include <signal.h>

#include <thread>
#include <mutex>
#include <deque>

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

class mySIFT;
typedef vector< vector<double> > double2D;
class Key_Point;
void match(mySIFT& left, mySIFT& right, string targetFile, Mat img_scene);
Mat concat2Img(Mat& i1, Mat& i2);
Point2f MatMulti(Mat matrix, Point2f point);

int perimeter = 0;
int biasX[4] = {0, 0, 0 ,0};
vector<int*> accuBias;
vector< Point2f > fixed_corners(4);
bool detectFlag = false;
deque<int*> biasQueue;


class mySIFT{
public:
    mySIFT(double sig, double kk, int nOct = 2, int numOctLay = 5) : nOctave(nOct), nLayersPerOctave(numOctLay), sigma(sig), k(kk){}
    void createDoG();
    void detectKeypoints();
    void LoadImage(Mat imgOri);
    void filterKeyPoints();
    void drawKeyPoints(string imageName);
    void computeDescriptor();
    vector< Key_Point > keyPoints;
    vector< Key_Point > filteredKeyPoints;
    vector< Mat > blurredImgs;
private:
    double2D getGaussianKernel(double sigma);
    Mat GaussianBlur(const Mat& src, double sigma);
    void computeOrientationHist(vector< Key_Point >& keyPoints);
    vector< double > DescriptorHelper(int row, int column, int layer);
    void filterKeyPointsHelper1(vector< int >& brighter, vector< int >& darker, Mat& thisMat, Key_Point& thisKpt, int value, int threshold);
    void filterKeyPointsHelper2(vector< int >& brighter, vector< int >& darker, int& nbrighter, int& ndarker);

    vector< Mat > DoGs;
    int nLayersPerOctave;
    int nOctave;
    double sigma;
    double k;
};

class Key_Point{
public:
    Key_Point(double s, int r, int c, int t, int l) : scale(s), row(r), col(c), type(t), layer(l){}
    int cornerValue;
    int row;
    int col;
    int orientation;
    int type;//0:min, 1:max
    double scale;
    int layer;//¦bDoGªº­þ¤@­ÓLayer
    vector< double > descriptor;
};


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


int main(int argc, char** argv)
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

    string targetFile = "target.jpg";

    Mat img_object = imread(targetFile, CV_LOAD_IMAGE_GRAYSCALE);
    Mat img_scene;


    mySIFT haha(1.414, 1.414);//sigma k
    haha.LoadImage(img_object);
    haha.createDoG();
    haha.detectKeypoints();
    haha.filterKeyPoints();
    haha.drawKeyPoints(targetFile);
    haha.computeDescriptor();




    int perimeter = 0;
    int biasX = 0;
    vector<int> accuPerimeter;
    vector<int> accuBias;
    vector< Point2f > fixed_corners(4);
    bool detectFlag = false;
    bool newUpdate = false;

    deque<int*> biasQueue;

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

        Mat imgScene;
        cvtColor(img_scene, imgScene, CV_BGR2GRAY);
        mySIFT hoho(1.414, 1.414);//sigma k
        
        hoho.LoadImage(imgScene);
        hoho.createDoG();
        hoho.detectKeypoints();
        hoho.filterKeyPoints();
        hoho.computeDescriptor();
        match(haha, hoho, targetFile, img_scene);
    }        


    return 0;
}


void match(mySIFT& left, mySIFT& right, string targetFile, Mat img_scene)
{
    vector< Key_Point >& a = left.keyPoints;
    vector< Key_Point >& b = right.keyPoints;
    
    Mat target = imread(targetFile);//§Ú­n±m¦âªº
    Mat find = img_scene;
    Mat result = concat2Img(target, find);

    vector< Point2f > obj;
    vector< Point2f > scene;

    for (int i = 0; i < a.size(); ++i){
        int index = -1;//index of minimum distance;
        double min = INT_MAX;
        int indexMin2 = -1;
        double min2 = INT_MAX;
        
        for (int j = 0; j < b.size(); ++j){//¼É¤O¥h±½¨C¤@­Ó¥kÃäªºKey_Point:b[j]
            double dist = 0;
            for (int k = 0; k < 32; ++k)
                dist += (a[i].descriptor[k] - b[j].descriptor[k]) * (a[i].descriptor[k] - b[j].descriptor[k]);
            if (dist < min){//³Ì¤pªº­n³Q¨ú¥N¤F
                min2 = min;
                indexMin2 = index;
                min = dist;
                index = j;//¥ªÃäi match¨ì¥kÃäindex
            }
        }

        int B = rand() % 256;
        int G = rand() % 256;
        int R = rand() % 256;

        if (min < 0.5 * min2){//good matches
            double aScaling = (a[i].layer / 5 == 0) ? 1 : 1.6;
            double bScaling = (b[index].layer / 5 == 0) ? 1 : 1.6;//ÁY¤p´X­¿ªº¡A­n©ñ¤j¦^¨Ó
            //cout << aScaling << " " << bScaling << "\n";
            line(result, Point(a[i].col * aScaling, a[i].row * aScaling), Point(target.cols + b[index].col * bScaling, b[index].row * bScaling), Scalar(B, G, R));
            obj.push_back(Point2f(a[i].col * aScaling, a[i].row * aScaling));
            scene.push_back(Point2f(b[index].col * bScaling, b[index].row * bScaling));
        }
    }
    //µ²§ômatch

    Mat H;
    if(obj.size() > 3 && scene.size() > 3){
        H = findHomography(obj, scene, CV_RANSAC);
    }

    if(!H.empty()){
        vector< Point2f > obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(left.blurredImgs[0].cols, 0);
        obj_corners[2] = cvPoint(left.blurredImgs[0].cols, left.blurredImgs[0].rows);
        obj_corners[3] = cvPoint(0, left.blurredImgs[0].rows);
        
        vector< Point2f > computed_corners(4);
        for (int i = 0; i < 4; ++i)
            computed_corners[i] = MatMulti(H, obj_corners[i]);//¦Û¤vºâ
    
    
        int edge[4];
        edge[0] = abs(computed_corners[0].y - computed_corners[1].y) + abs(computed_corners[0].x - computed_corners[1].x);
        edge[1] = abs(computed_corners[1].y - computed_corners[2].y) + abs(computed_corners[1].x - computed_corners[2].x);
        edge[2] = abs(computed_corners[2].y - computed_corners[3].y) + abs(computed_corners[2].x - computed_corners[3].x);
        edge[3] = abs(computed_corners[3].y - computed_corners[0].y) + abs(computed_corners[3].x - computed_corners[0].x);
        int currentPerimeter = edge[0] + edge[1] + edge[2] + edge[3];
        int *currentBias = new int[4];
        currentBias[0] = computed_corners[0].x;
        currentBias[1] = computed_corners[1].x;
        currentBias[2] = computed_corners[2].x;
        currentBias[3] = computed_corners[3].x;
        
        //bias pipeline
        if(biasQueue.size() != 3){
            biasQueue.push_back(currentBias);
        }
        else{
            int* toDelete = biasQueue.front();
            delete toDelete;
            biasQueue.pop_front();
            biasQueue.push_back(currentBias);
            int biasDelta[4];
            int accuBiasDelta = 0;
            //computer difference between every 2 frames
            for(int i = 0; i < biasQueue.size(); i++){
                if(i == 0){
                    continue;
                }
                else{
                    for(int j = 0 ; j < 4; j++){
                        accuBiasDelta += abs(biasQueue.at(i)[j] - biasQueue.at(i - 1)[j]);
                    }
                }
            }
            if(currentPerimeter < 100 || accuBiasDelta > 400){
                detectFlag = false;
                fixed_corners.clear();
                line(result, computed_corners[0] + Point2f(left.blurredImgs[0].cols, 0), computed_corners[1] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 0, 255), 4);
                line(result, computed_corners[1] + Point2f(left.blurredImgs[0].cols, 0), computed_corners[2] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 0, 255), 4);
                line(result, computed_corners[2] + Point2f(left.blurredImgs[0].cols, 0), computed_corners[3] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 0, 255), 4);
                line(result, computed_corners[3] + Point2f(left.blurredImgs[0].cols, 0), computed_corners[0] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 0, 255), 4);
            }
            else{//update corners
                detectFlag = true;
                fixed_corners = computed_corners;
            }
        }
        if(fixed_corners.size() != 0){
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line(result, fixed_corners[0] + Point2f(left.blurredImgs[0].cols, 0), fixed_corners[1] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 255, 0), 4);
            line(result, fixed_corners[1] + Point2f(left.blurredImgs[0].cols, 0), fixed_corners[2] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 255, 0), 4);
            line(result, fixed_corners[2] + Point2f(left.blurredImgs[0].cols, 0), fixed_corners[3] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 255, 0), 4);
            line(result, fixed_corners[3] + Point2f(left.blurredImgs[0].cols, 0), fixed_corners[0] + Point2f(left.blurredImgs[0].cols, 0), Scalar(0, 255, 0), 4);
        }
        int bias = (fixed_corners[1].x - fixed_corners[0].x)/2 + fixed_corners[0].x;
        if(detectFlag){
            int biasFromCenter = bias - (result.cols - left.blurredImgs[0].cols)/2;
            unsigned char output[DATAGRAM_SIZE];
            output[0] = CONTROL;
            if(biasFromCenter < 0){//object is at left
                output[1] = LEFT;
                cout << "LEFT!" << endl;
                transmit(output);
            }
            else if(biasFromCenter > 0){//object is at right
                output[1] = RIGHT;
                cout << "RIGHT!" << endl;
                transmit(output);
            }
            if(currentPerimeter < 1000){
                output[1] = FORWARD;
                cout << "FORWARD!" << endl;
                for(int i = 0; i < 2; i++){
                     transmit(output);
                }
            }
        }
        imshow("haha", result);
        imwrite("result.jpg", result);
        waitKey(1);
    }
}

Point2f MatMulti(Mat matrix, Point2f point)//¬Ý¬Ý¸òsource code¤@¤£¤@¼Ë
{
    double input[3] = { point.x, point.y, 1 };

    vector < double > output(3);

    for (int row = 0; row < 3; ++row){
        double sum = 0;
        for (int col = 0; col < 3; ++col){
            sum += matrix.at< double >(row, col) * input[col];
        }
        output[row] = sum;
    }
    
    if (output[2] == 0.0)
        return Point2f(0, 0);
    else
        return Point2f((int)(output[0] / output[2]), (int)(output[1] / output[2]));
}

Mat concat2Img(Mat& i1, Mat& i2)
{   
    //cout << i1.type();
    Mat i3(max(i1.rows, i2.rows), i1.cols + i2.cols, i1.type(), Scalar(0, 0, 0));

    for (int row = 0; row < i3.rows; ++row)
        for (int col = 0; col < i3.cols; ++col){
            if (col < i1.cols && row < i1.rows)//¥Î¥ªÃäªº½Æ»s
                i3.at<Vec3b>(row, col) = i1.at<Vec3b>(row, col);
            else if (col >= i1.cols)
                i3.at<Vec3b>(row, col) = i2.at<Vec3b>(row, col - i1.cols);
        }
    return i3;
}

void mySIFT::computeDescriptor()
{
    for (int i = 0; i < keyPoints.size(); ++i){//¹ï¨C­Ókeypoint­nºâ¥X¤@­Ódescriptor(¥ý¸Õ¸Õ¬Ý32ºû)
        Key_Point& kpt = keyPoints[i];
        int kptRow = kpt.row;
        int kptCol = kpt.col;
        vector< double > temp;
        
        //¥ª¤W4x4
        temp = DescriptorHelper(kptRow - 3, kptCol - 3, kpt.layer);
        for (int j = 0; j < 8; ++j)
            kpt.descriptor.push_back(temp[j]);
        //¥k¤W4x4
        temp = DescriptorHelper(kptRow - 3, kptCol, kpt.layer);
        for (int j = 0; j < 8; ++j)
            kpt.descriptor.push_back(temp[j]);
        //¥ª¤U4x4
        temp = DescriptorHelper(kptRow, kptCol - 3, kpt.layer);
        for (int j = 0; j < 8; ++j)
            kpt.descriptor.push_back(temp[j]);
        //¥k¤U4x4
        temp = DescriptorHelper(kptRow, kptCol, kpt.layer);
        for (int j = 0; j < 8; ++j)
            kpt.descriptor.push_back(temp[j]);

    }
}

vector< double > mySIFT::DescriptorHelper(int Row, int Col, int layer)//¥ª¤W¨¤row, column
{
    Mat& image = blurredImgs[layer];
    vector< double > bins(8, 0);

    for (int r = Row; r <= Row + 3; ++r)
        for (int c = Col; c <= Col + 3; ++c){
            if (r >= 1 && c >= 1 && r < image.rows - 1 && c < image.cols - 1){
                int temp1 = (int)image.at<uchar>(r, c + 1) - (int)image.at<uchar>(r, c - 1);
                int temp2 = (int)image.at<uchar>(r + 1, c) - (int)image.at<uchar>(r - 1, c);//pixel­Èªº®t
                double magnitude = sqrt(temp1 * temp1 + temp2 * temp2);
                double theta = atan2(temp2, temp1) * 180 / PI;
                if (theta < 0)
                    theta += 360;
                //double weight;
                int offset = ((int)theta % 45 < 23) ? 0 : 1;
                //cout << "bins : " << ((int)theta / 45 + offset) % 8 << "magnitude : " << magnitude << "\n";
                bins[((int)theta / 45 + offset) % 8] +=  magnitude;
            }
        }
    return bins;
}

void mySIFT::drawKeyPoints(string imageName)
{
    Mat imgOriRGB = imread(imageName);
    for (int i = 0; i < keyPoints.size(); ++i){
        Key_Point& kpt = keyPoints[i];
        circle(imgOriRGB, Point(kpt.col, kpt.row), 5, Scalar(0, 0, 255));
    }
}

void mySIFT::filterKeyPoints()
{
    for (int i = 0; i < keyPoints.size(); ++i){
        Key_Point& thisKpt = keyPoints[i];
        Mat& thisMat = blurredImgs[thisKpt.layer];
        //¦³n1­Ó³sÄòªºpixel³£¤ñthisKpt²`¡A¦³n2­Ó³sÄòªºÂI³£¤ñthisKpt²L
        if (thisKpt.row - 3 < 0 || thisKpt.row + 3 >= thisMat.rows || thisKpt.col - 3 < 0 || thisKpt.col + 3 >= thisMat.cols)//ÀË¬d½d³ò¡A¤£¯à°÷µe¾ã°éªº´Nª½±µÙT
            continue;
        vector< int > brighter(16, 0);
        vector< int > darker(16, 0);
        int value = thisMat.at<uchar>(thisKpt.row, thisKpt.col);
        int threshold = 20;
        filterKeyPointsHelper1(brighter, darker, thisMat, thisKpt, value, threshold);
        int nBrighter, nDarker;
        filterKeyPointsHelper2(brighter, darker, nBrighter, nDarker);
        thisKpt.cornerValue = max(nBrighter, nDarker);
        if (thisKpt.cornerValue > 16 || thisKpt.cornerValue < 0)
            cout << "??\n";
    }

    vector< Key_Point > temp;
    for (int i = 0; i < keyPoints.size(); ++i){
        if (keyPoints[i].cornerValue >= 8)
            temp.push_back(keyPoints[i]);
    }
    keyPoints = temp;
}

void mySIFT::filterKeyPointsHelper2(vector< int >& brighter, vector< int >& darker, int& nBrighter, int& nDarker)
{
    for (int i = 0; i < 8; ++i)
        brighter.push_back(brighter[i]);
    
    int accu1 = 0;
    int maxAccu1 = 0;
    for (int i = 0; i < 16; ++i){
        if (brighter[i]){
            ++accu1;
            if (accu1 > maxAccu1)
                ++maxAccu1;
        }
        else
            accu1 = 0;//Âk0
    }
    nBrighter = maxAccu1;

    accu1 = 0;
    maxAccu1 = 0;
    for (int i = 8; i < 24; ++i){
        if (brighter[i]){
            ++accu1;
            if (accu1 > maxAccu1)
                ++maxAccu1;
        }
        else
            accu1 = 0;
    }
    if (maxAccu1 > nBrighter)
        nBrighter = maxAccu1;

    for (int i = 0; i < 8; ++i)
        darker.push_back(darker[i]);

    accu1 = 0;
    maxAccu1 = 0;
    for (int i = 0; i < 16; ++i){
        if (darker[i]){
            ++accu1;
            if (accu1 > maxAccu1)
                ++maxAccu1;
        }
        else
            accu1 = 0;//Âk0
    }
    nDarker = maxAccu1;

    accu1 = 0;
    maxAccu1 = 0;
    for (int i = 8; i < 24; ++i){
        if (darker[i]){
            ++accu1;
            if (accu1 > maxAccu1)
                ++maxAccu1;
        }
        else
            accu1 = 0;
    }
    if (maxAccu1 > nDarker)
        nDarker = maxAccu1;
}

void mySIFT::filterKeyPointsHelper1(vector< int >& brighter, vector< int >& darker, Mat& thisMat, Key_Point& thisKpt, int value, int threshold)
{
    //§Ëbrighter
    int brighterthshod = value + threshold;
    if (thisMat.at<uchar>(thisKpt.row - 3, thisKpt.col) > brighterthshod)
        brighter[0] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 3, thisKpt.col + 1) > brighterthshod)
        brighter[1] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 2, thisKpt.col + 2) > brighterthshod)
        brighter[2] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 1, thisKpt.col + 3) > brighterthshod)
        brighter[3] = 1;
    if (thisMat.at<uchar>(thisKpt.row, thisKpt.col + 3) > brighterthshod)
        brighter[4] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 1, thisKpt.col + 3) > brighterthshod)
        brighter[5] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 2, thisKpt.col + 2) > brighterthshod)
        brighter[6] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 3, thisKpt.col + 1) > brighterthshod)
        brighter[7] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 3, thisKpt.col) > brighterthshod)
        brighter[8] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 3, thisKpt.col - 1) > brighterthshod)
        brighter[9] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 2, thisKpt.col - 2) > brighterthshod)
        brighter[10] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 1, thisKpt.col - 3) > brighterthshod)
        brighter[11] = 1;
    if (thisMat.at<uchar>(thisKpt.row, thisKpt.col - 3) > brighterthshod)
        brighter[12] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 1, thisKpt.col - 3) > brighterthshod)
        brighter[13] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 2, thisKpt.col - 2) > brighterthshod)
        brighter[14] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 3, thisKpt.col - 1) > brighterthshod)
        brighter[15] = 1;

    //§Ëdarker
    int darkererthshod = value - threshold;
    if (thisMat.at<uchar>(thisKpt.row - 3, thisKpt.col) < darkererthshod)
        brighter[0] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 3, thisKpt.col + 1) < darkererthshod)
        brighter[1] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 2, thisKpt.col + 2) < darkererthshod)
        brighter[2] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 1, thisKpt.col + 3) < darkererthshod)
        brighter[3] = 1;
    if (thisMat.at<uchar>(thisKpt.row, thisKpt.col + 3) < darkererthshod)
        brighter[4] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 1, thisKpt.col + 3) < darkererthshod)
        brighter[5] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 2, thisKpt.col + 2) < darkererthshod)
        brighter[6] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 3, thisKpt.col + 1) < darkererthshod)
        brighter[7] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 3, thisKpt.col) < darkererthshod)
        brighter[8] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 3, thisKpt.col - 1) < darkererthshod)
        brighter[9] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 2, thisKpt.col - 2) < darkererthshod)
        brighter[10] = 1;
    if (thisMat.at<uchar>(thisKpt.row + 1, thisKpt.col - 3) < darkererthshod)
        brighter[11] = 1;
    if (thisMat.at<uchar>(thisKpt.row, thisKpt.col - 3) < darkererthshod)
        brighter[12] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 1, thisKpt.col - 3) < darkererthshod)
        brighter[13] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 2, thisKpt.col - 2) < darkererthshod)
        brighter[14] = 1;
    if (thisMat.at<uchar>(thisKpt.row - 3, thisKpt.col - 1) < darkererthshod)
        brighter[15] = 1;
}

void mySIFT::computeOrientationHist(vector< Key_Point >& keyPoints)// Computes a gradient orientation histogram at a specified pixel
{

    for (int i = 0; i < keyPoints.size(); ++i){//¹ï¨C­Ókeypointºâ¤è¦V
        vector< double > bins(SIFT_ORI_HIST_BINS, 0);
        Key_Point& thiskpt = keyPoints[i];
        Mat& image = blurredImgs[thiskpt.layer];
        int kptrow = thiskpt.row;
        int kptcol = thiskpt.col;
        for (int row = kptrow - SIFT_ORI_RADIUS; row <= kptrow + SIFT_ORI_RADIUS; ++row)
            for (int col = kptcol - SIFT_ORI_RADIUS; col <= kptcol + SIFT_ORI_RADIUS; ++col){
                if (row >= 1 && row < image.rows - 1 && col >= 1 && col < image.cols - 1 && (row - kptrow) * (row - kptrow) + (col - kptcol) * (col - kptcol) < SIFT_ORI_RADIUS * SIFT_ORI_RADIUS){
                    //¦b½d³ò¤º
                    int temp1 = (int)image.at<uchar>(row, col + 1) - (int)image.at<uchar>(row, col - 1);
                    int temp2 = (int)image.at<uchar>(row + 1, col) - (int)image.at<uchar>(row - 1, col);//pixel­Èªº®t
                    double magnitude = sqrt(temp1 * temp1 + temp2 * temp2);
                    double theta = atan2(temp2, temp1) * 180 / PI;
                    if (theta < 0)
                        theta += 360;
                    double weight = exp(-0.5 * ((row - kptrow) * (row - kptrow) + (col - kptcol) * (col - kptcol)) / (SIFT_ORI_SIG * SIFT_ORI_SIG));
                    int offset = ((int)theta % 10 < 5)? 0 : 1;
                    bins[((int)theta / 10 + offset) % 36] += magnitude * weight;
                    //weightSum += weight;
                }
            }
        int index = 0;
        for (int i = 1; i < SIFT_ORI_HIST_BINS; ++i)
            if (bins[i] > bins[index])
                index = i;

        thiskpt.orientation = index * 10;
    }
}

void mySIFT::detectKeypoints()
{
    int size = (DoGs.size() - (nOctave - 1))/ nOctave;//¨C¤@­Óoctave¡ADoGªºsize¡A²{¦b¬O4
    double sigmaTemp = sigma;

    for (int layer = 1; layer <= size - 2; ++layer){
        for (int row = 1; row < DoGs[0].rows - 1; ++row){
            for (int col = 1; col < DoGs[0].cols - 1; ++col){//¬ÝDoGs[layer]¨º±iªº[row][col]¨ºÂI¬O¤£¬Omax min
                int value = DoGs[layer].at<int>(row, col);
                bool isMax = true;
                bool isMin = true;
                for (int layerOffset = -1; layerOffset <= 1 && (isMin || isMax); ++layerOffset){
                    for (int i = row - 2; i <= row + 2 && (isMin || isMax); ++i){
                        for (int j = col - 2; j <= col + 2 && (isMin || isMax); ++j){
                            if (i >= 0 && j >= 0 && i < DoGs[0].rows && j < DoGs[0].cols && i != row && j != col){//¦b½d³ò¤ºªº¸Ü
                                int tester = DoGs[layer + layerOffset].at<int>(i, j);
                                if (tester + 0 >= value)
                                    isMax = false;
                                if (tester - 0 <= value)
                                    isMin = false;
                            }
                        }
                    }
                }
                if (isMax)
                    keyPoints.push_back(Key_Point(sigmaTemp, row, col, 1, layer));
                if (isMin)
                    keyPoints.push_back(Key_Point(sigmaTemp, row, col, 0, layer));
            }
        }
        sigmaTemp *= k;
    }

    //computeOrientationHist(keyPoints);
    if (DoGs.size() != 9)
        cout << "¶ã¶ã\n";

    for (int layer = 6; layer <= 7; ++layer){
        for (int row = 1; row < DoGs[5].rows - 1; ++row)
            for (int col = 1; col < DoGs[5].cols - 1; ++col){//¬ÝDoGs[layer]¨º±iªº[row][col]¨ºÂI¬O¤£¬Omax min
                int value = DoGs[layer].at<int>(row, col);
                //cout << "value : " << value << " at " << row << ", " << col << "\n";
                bool isMax = true;
                bool isMin = true;
                for (int layerOffset = -1; layerOffset <= 1 && (isMin || isMax); ++layerOffset){
                    for (int i = row - 2; i <= row + 2 && (isMin || isMax); ++i){
                        for (int j = col - 2; j <= col + 2 && (isMin || isMax); ++j){
                            if (i >= 0 && j >= 0 && i < DoGs[5].rows && j < DoGs[5].cols && i != row && j != col){//¦b½d³ò¤ºªº¸Ü
                                int tester = DoGs[layer + layerOffset].at<int>(i, j);
                                if (tester + 0 >= value)
                                    isMax = false;
                                if (tester - 0 <= value)
                                    isMin = false;
                            }
                        }
                    }
                }
                if (isMax)
                    keyPoints.push_back(Key_Point(sigmaTemp, row, col, 1, layer));
                if (isMin)
                    keyPoints.push_back(Key_Point(sigmaTemp, row, col, 0, layer));
            }
        sigmaTemp *= k;
    }

    computeOrientationHist(keyPoints);
}

void mySIFT::createDoG()
{
    double tempSigma = sigma;

    for (int j = 0; j < nOctave; ++j){//nOctave = 2

        for (int i = 0; i < nLayersPerOctave - 1; ++i){//¥]§t­ì¹Ïªº¼Ò½k¹Ï¡A¤@­ÓOctave5±i¹Ï¡A±À4¦¸¡A¦]¬°²Ä¤@±i¬O­ì¹Ï
            blurredImgs.push_back(GaussianBlur(blurredImgs.back(), tempSigma));
            tempSigma *= k;
            Mat DoG(blurredImgs[j * nLayersPerOctave + i + 1].rows, blurredImgs[j * nLayersPerOctave + i + 1].cols, CV_32SC1, Scalar(0));
            for (int row = 0; row < blurredImgs[j * nLayersPerOctave + i].rows; ++row)
                for (int col = 0; col < blurredImgs[j * nLayersPerOctave + i].cols; ++col){
                    int diff = (int)blurredImgs[j * nLayersPerOctave + i + 1].at<uchar>(row, col) - (int)blurredImgs[j * nLayersPerOctave + i].at<uchar>(row, col);
                    DoG.at<int>(row, col) = diff;
                }
            DoGs.push_back(DoG);
        }
        //°µ§¹¤@­Óoctave
        if (j != nOctave - 1){
            tempSigma = tempSigma / k / k / k / k;
            Mat src = *(blurredImgs.end() - 4);//blurredImgs[blurredImgs.size() - 3];
            Mat firstMatInNewOctave;
            resize(src, firstMatInNewOctave, Size(src.cols / 1.6, src.rows / 1.6));
            blurredImgs.push_back(firstMatInNewOctave);//±À¶i¤U¤@¼hoctaveªº²Ä¤@±i¹Ï
            Mat empty;
            DoGs.push_back(empty);
        }
    }
}

double2D mySIFT::getGaussianKernel(double sigma)
{
    int kSize = cvRound(sigma * 1.5 + 1) | 1;//kSize¬°©_¼Æ¡A¶Ã©wªº
    
    vector< double > kernel_1D;
    
    int shift = (kSize - 1) / 2;
    double sum = 0;
    
    for (int i = 0; i < kSize; ++i){
        int x = i - shift;
        double temp = exp(-0.5*(x / sigma)*(x / sigma));
        sum += temp;
        kernel_1D.push_back( temp );
    }
    
    for (int i = 0; i < kSize; ++i)
        kernel_1D[i] = kernel_1D[i] / sum;

    double2D kernel_2D;
    for (int i = 0; i < kSize; ++i){
        vector < double > a;
        kernel_2D.push_back(a);
        for (int j = 0; j < kSize; ++j)
            kernel_2D[i].push_back( kernel_1D[j] * kernel_1D[i]);
    }
    return kernel_2D;
}

Mat mySIFT::GaussianBlur(const Mat& src, double sigma)//input (¹Ï¤ù, sigma), output (¼Ò½k¹Ï¤ù)
{
    double2D G_Kernel = getGaussianKernel(sigma);
    int kSize = G_Kernel.size();
    //kernel¸n¤W¥h¡A¶W¥X¹Ï¤ùªºpixel¸É0
    int shift = (kSize - 1) / 2;

    Mat output = src.clone();

    for (int row = 0; row < src.rows; ++row)
        for (int col = 0; col < src.cols; ++col){//­n¹ïimg[row][col]ªºpixel°µ¼Ò½k
            double sum = 0.0;
            double weightSum = 0.0;//0.0
            for (int i = row - shift; i <= row + shift; ++i)
                for (int j = col - shift; j <= col + shift; ++j){
                    if (i >= 0 && j >= 0 && i < src.rows && j < src.cols){
                        sum += src.at<uchar>(i, j) * G_Kernel[i - (row - shift)][j - (col - shift)];
                        weightSum += G_Kernel[i - (row - shift)][j - (col - shift)];
                    }
                }
            output.at<uchar>(row, col) = sum / weightSum;
            //cout << "sum : " << sum << "   " << (int)output.at<uchar>(row, col) << "\n";
        }
    //cout << "Finish Blurring with sigma : " << setprecision(2) << sigma << "\n";
    return output;
}

void mySIFT::LoadImage(Mat imgOri)
{
    blurredImgs.push_back(imgOri);//­ì¹Ï¬O©ñindex0
}


	
