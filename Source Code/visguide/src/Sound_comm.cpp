#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "iostream"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "queue"
#include "vector"
#include "string"
using namespace std;

struct Data {
  int x;
  int y;
  int area;
};

struct Object {
  vector<Data> data;
};

struct Point {
  int x;
  int y;
};

int count = 0;

Object objects[11];
int Area_thres[11] = {0,1000,0,100,0,10,0,100,10,50,500};
float distance_thresh[11] = {1.6,1.6,1.6,1.6,1.6,1.6,1.6,1.6,1.6,1.6,1.6}; //All values are in meters
void callback(const sensor_msgs::ImageConstPtr & Segmsg, const sensor_msgs::ImageConstPtr & Depthmsg);
void bfs(cv::Mat & seg);
void Detect(cv::Mat & depthImg);


int main(int argc, char **argv){
	ros::init(argc, argv, "Human_Comm");
	ros::NodeHandle n;
	message_filters::Subscriber<sensor_msgs::Image> segSub(n, "/visguide/zed_node/seg/image_rect_color", 10);
	message_filters::Subscriber<sensor_msgs::Image> DepthSub(n, "/visguide/zed_node/depth/depth_registered", 10);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(segSub, DepthSub, 30);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
	ros::spin();
	return 0;
}



void callback(const sensor_msgs::ImageConstPtr & Segmsg, const sensor_msgs::ImageConstPtr & Depthmsg){
	cv::Mat seg = cv_bridge::toCvShare(Segmsg, "mono8")->image;
	cv::Mat depth = cv_bridge::toCvShare(Depthmsg, "32FC1")->image;//each pixel contains distance in METERS
		
	if(::count%12 == 0){
		bfs(seg);
		Detect(depth);
		::count = 0;
	}
	::count++;
}

void bfs(cv::Mat & seg){
	cv::Mat tempImg(seg.rows, seg.cols, CV_8UC1, cv::Scalar(0));
	for(int x=0; x<376; x++){
		for(int y=0; y<672; y++){
			if(seg.at<uchar>(x,y) !=0 && tempImg.at<uchar>(x,y) == 0){
				queue<Point> q;
				int color_det = seg.at<uchar>(x,y);
				Data temp;
				temp.x = x;
				temp.y = y;
				temp.area = 1;
				q.push({x,y});
				tempImg.at<uchar>(x,y) = 100;
				while(!q.empty()){
					int current_x = q.front().x;
					int current_y = q.front().y;
					q.pop();
					for(int i=current_x-1; i<current_x+2; i+=1){
						for(int j=current_y-1; j<current_y+2; j+=1){
							if(seg.at<uchar>(i,j) == color_det && tempImg.at<uchar>(i,j) == 0){
								q.push({i,j});
								tempImg.at<uchar>(i,j) = 100;
								temp.x+=i;
								temp.y+=j;
								temp.area++;
							}
						}
					}
				}
				temp.x = temp.x/temp.area;
				temp.y = temp.y/temp.area;
				objects[color_det].data.push_back(temp);
				cout<<"x:"<<temp.x<<"	y:"<<temp.y<<"	area:"<<temp.area<<"	col:"<<color_det<<endl;
			}
		}
	}
}


void Detect(cv::Mat & depthImg){
	int flag_DL=0,flag_DR=0, flag_PlL=0,flag_PlR=0, flag_WbL=0,flag_WbR=0, flag_TcL=0, flag_TcR=0, flag_NBL=0,flag_NBR=0;
	for(int i=1; i<11; i++){
		int no_of_objects = objects[i].data.size();
		for(int j=0; j<no_of_objects; j++){			//Loop for iterating the elements in a class
			if(objects[i].data[j].area > Area_thres[i]){
				int x = objects[i].data[j].x;
				int y = objects[i].data[j].y;
				cout<<"Area:"<<objects[i].data[j].area<<endl;
				char dis[4];
				sprintf(dis, "%0.1f", depthImg.at<float>(x,y));
				switch(i){
					case 1:		//door, double door
							if(depthImg.at<float>(x,y) < distance_thresh[1]){
								if(y < depthImg.cols/2)
									//cout<<"THERE IS A DOOR ON YOUR LEFT"<<endl;
									system("espeak -v en 'THERE IS A DOOR ON YOUR LEFT' ");
									//q.push("THERE IS A DOOR ON YOUR LEFT");
								else
									//cout<<"THERE IS A DOOR ON YOUR RIGHT"<<endl;
									system("espeak -v en 'THERE IS A DOOR ON YOUR RIGHT' ");
									//q.push("THERE IS A DOOR ON YOUR RIGHT");
									}
							else{
								if(y < depthImg.cols/2 && !flag_DL){
									//cout<<"THERE WILL BE A DOOR COMING ON YOUR LEFT"<<endl;
									flag_DL = 1;
									string str = "espeak -v en 'DOOR COMING AT" + std::string(dis) + "METERS ON YOUR LEFT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A DOOR COMING ON YOUR LEFT");
								}
								else if(!flag_DR){
									//cout<<"THERE WILL BE A DOOR COMING ON YOUR RIGHT"<<endl;
									flag_DR = 1;
									string str = "espeak -v en 'DOOR COMING AT" + std::string(dis) + "METERS ON YOUR RIGHT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A DOOR COMING ON YOUR RIGHT");
									}
								}
							break;
					case 2:
							break;
					case 3:		//plant, flora, plant life
							if(depthImg.at<float>(x,y) < distance_thresh[3]){
								if(y < depthImg.cols/2)
									//cout<<"THERE IS A PLANT ON YOUR LEFT"<<endl;
									system("espeak -v en 'THERE IS A PLANT ON YOUR LEFT' ");
									//q.push("THERE IS A PLANT ON YOUR LEFT");
								else
									//cout<<"THERE IS A PLANT ON YOUR RIGH"<<endl;
									system("espeak -v en 'THERE IS A PLANT ON YOUR RIGHT' ");
									//q.push("THERE IS A PLANT ON YOUR RIGHT");
							}
							else{
								if(y < depthImg.cols/2 && !flag_PlL){
									//cout<<"THERE WILL BE A PLANT ON YOUR LEFT"<<endl;
									flag_PlL = 1;
									string str = "espeak -v en 'PLANT COMING AT" + std::string(dis) + "METERS ON YOUR LEFT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A PLANT ON YOUR LEFT");
								}
									
								else if(!flag_PlR){
									//cout<<"THERE WILL BE A PLANT ON YOUR RIGHT"<<endl;
									flag_PlR = 1;
									string str = "espeak -v en 'PLANT COMING AT" + std::string(dis) + "METERS ON YOUR RIGHT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A PLANT ON YOUR RIGHT");	
								}
									
							}
							break;
					case 4:
							break;
					case 5:		//light, light source
							break;
					case 6:
							break;
					case 7:		//pot, flowerpot, water bottle
							if(depthImg.at<float>(x,y) < distance_thresh[7]){
								if(y < depthImg.cols/2 && x < depthImg.rows/2)
									//cout<<"THERE IS A FLOWERPOT OR WATER BOTTLE ON YOUR LEFT"<<endl;
									system("espeak -v en 'THERE IS A FLOWERPOT OR WATER BOTTLE ON YOUR LEFT' ");
									//q.push("THERE IS A FLOWERPOT OR WATER BOTTLE ON YOUR LEFT");
								else
									//cout<<"THERE IS A FLOWERPOT OR WATER BOTTLE ON YOUR RIGHT"<<endl;
									system("espeak -v en 'THERE IS A FLOWERPOT OR WATER BOTTLE ON YOUR RIGHT' ");
									//q.push("THERE IS A FLOWERPOT OR WATER BOTTLE ON YOUR RIGHT");
							}
							else{
								if(y < depthImg.cols/2 && x < depthImg.rows/2 && !flag_WbL){
									//cout<<"THERE WILL BE A FLOWERPOT OR WATER BOTTLE ON YOUR LEFT"<<endl;
									flag_WbL = 1;
									string str = "espeak -v en 'FLOWERPOT OR A WATER BOTTLE COMING AT" + std::string(dis) + "METERS ON YOUR LEFT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A FLOWERPOT OR WATER BOTTLE ON YOUR LEFT");
								}
									
								else if(!flag_WbR){
									//cout<<"THERE WILL BE A FLOWERPOT OR WATER BOTTLE ON YOUR RIGHT"<<endl;
									flag_WbR = 1;
									string str = "espeak -v en 'FLOWERPOT OR A WATER BOTTLE COMING AT" + std::string(dis) + "METERS ON YOUR RIGHT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A FLOWERPOT OR WATER BOTTLE ON YOUR RIGHT");
								}
										
							}
							break;
					case 8:		//ashcan, trash can
							if(depthImg.at<float>(x,y) < distance_thresh[8]){
								if(y < depthImg.cols/2 && x < depthImg.rows/2)
									//cout<<"THERE IS A TRASH CAN ON YOUR LEFT"<<endl;
									system("espeak -v en 'THERE IS A TRASH CAN ON YOUR LEFT' ");
									//q.push("THERE IS A TRASH CAN ON YOUR LEFT");
								else
									//cout<<"THERE IS A TRASH CAN ON YOUR RIGHT"<<endl;
									system("espeak -v en 'THERE IS A TRASH CAN ON YOUR RIGHT' ");
									//q.push("THERE IS A TRASH CAN ON YOUR RIGHT");
							}
							else{
								if(y < depthImg.cols/2 && x < depthImg.rows/2 && !flag_TcL){
									//cout<<"THERE WILL BE A TRASH CAN ON YOUR LEFT"<<endl;
									flag_TcL = 1;
									string str = "espeak -v en 'TRASH CAN COMING AT" + std::string(dis) + "METERS ON YOUR LEFT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A TRASH CAN ON YOUR LEFT");
								}
									
								else if(!flag_TcR){
									//cout<<"THERE WILL BE A TRASH CAN ON YOUR RIGHT"<<endl;
									flag_TcR = 1;
									string str = "espeak -v en 'TRASH CAN COMING AT" + std::string(dis) + "METERS ON YOUR RIGHT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A TRASH CAN ON YOUR RIGHT");
								}
										
							}
							break;
					case 9:		//bulletin board, notice board
							if(depthImg.at<float>(x,y) < distance_thresh[9]){
								if(y < depthImg.cols/2)
									//cout<<"THERE IS A NOTICE BOARD ON YOUR LEFT"<<endl;
									system("espeak -v en 'THERE IS A NOTICE BOARD ON YOUR LEFT' ");
									//q.push("THERE IS A NOTICE BOARD ON YOUR LEFT");
								else
									//cout<<"THERE IS A NOTICE BOARD ON YOUR RIGHT"<<endl;
									system("espeak -v en 'THERE IS A NOTICE BOARD ON YOUR RIGHT' ");
									//q.push("THERE IS A NOTICE BOARD ON YOUR RIGHT");
							}
							else{
								if(y < depthImg.cols/2 && !flag_NBL){
									//cout<<"THERE WILL BE A NOTICE BOARD ON YOUR LEFT"<<endl;
									flag_NBL = 1;
									string str = "espeak -v en 'NOTICE BOARD COMING AT" + std::string(dis) + "METERS ON YOUR LEFT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A NOTICE BOARD ON YOUR LEFT");
								}
									
								else if(!flag_NBR){
									//cout<<"THERE WILL BE A NOTICE BOARD ON YOUR RIGHT"<<endl;
									flag_NBR = 1;
									string str = "espeak -v en 'NOTICE BOARD COMING AT" + std::string(dis) + "METERS ON YOUR RIGHT' \0";
									system(str.c_str());
									//q.push("THERE WILL BE A NOTICE BOARD ON YOUR RIGHT");
								}
										
							}
							break;
					case 10:	//floor
							break;
				}

			}
		}

		objects[i].data.clear();
	}
}