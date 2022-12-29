#define HAVE_STRUCT_TIMESPEC 
#define BUFFER_SIZE 1080
#define left_port  2367
#define right_port 2369 
#define HOST "192.168.1.100"
#define BUFFER_NUMBER 2000

#include <sys/socket.h>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <math.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <unistd.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

#define VPoint velodyne_pointcloud::PointXYZIR
#define Point2 pcl::PointXYZI

pthread_mutex_t buffer_lock = PTHREAD_MUTEX_INITIALIZER;

ros::Publisher pub_raw_data;

char Buffer_left[BUFFER_NUMBER][1080] = {};
char Buffer_right[BUFFER_NUMBER][1080] = {};


struct point
{
    int azimuth;
    float x, y, z, r, theta, hori, layer;
    int flag;
    int ring;

};

void* thread_parshing(void* data_1)
{

    struct sockaddr_in Server;
    struct sockaddr_in Client;

    socklen_t Client_size;

    int socket_main;
    int Recv_Size;

    char Buffer_socket[BUFFER_SIZE];

    char Buffer_state[BUFFER_NUMBER][1080];

    int iter = 0;

    Client_size = sizeof(Client);

    socket_main = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    memset(Buffer_socket, 0, BUFFER_SIZE);

    Server.sin_family = AF_INET;
    Server.sin_addr.s_addr = inet_addr(HOST);
    Server.sin_port = htons(left_port);


    if ( bind(socket_main, (struct sockaddr *)&Server,
			sizeof(Server)) <0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

    if (recvfrom(socket_main, Buffer_socket, BUFFER_SIZE,
                0, (struct sockaddr*)&Client, &Client_size) <= 0)
    {
        cout << "Left Lidar read error!" << endl;

        while (true) {

            Recv_Size = recvfrom(socket_main, Buffer_socket, BUFFER_SIZE,
                0, (struct sockaddr*)&Client, &Client_size);
            
            if (Recv_Size >= 0)
                break;
            else {

                usleep(1000000);
                 //받은 데이터 출력

            }
        }
        exit(0);
    }

    cout << "Left Lidar connect" << endl;
    //================
    while (true){

        int num = 0;    
        
        memset(Buffer_state, 0, sizeof(Buffer_state));

        while (true) {
        memset(Buffer_socket, 0, sizeof(Buffer_socket));
        // receive lidar data
        recvfrom(socket_main, Buffer_socket, BUFFER_SIZE,
                0, (struct sockaddr*)&Client, &Client_size);
    
        int azimuth = (Buffer_socket[13 ] << 8 | Buffer_socket[12]) / 100;

        if (azimuth > 0){
            
                // cout << "azimuth : " << azimuth << endl;

                // save lidar data
                for(int i = 0; i < 1080; i++){
                    Buffer_state[num][i] = Buffer_socket[i];
                }
                num = num +1;
        }
        
        // stop saving data and parshing buffer
        if( azimuth> 323){
            pthread_mutex_lock(&buffer_lock);
            memset(Buffer_left, 0, sizeof(Buffer_left));
            memcpy(Buffer_left, Buffer_state, sizeof(Buffer_left));
            pthread_mutex_unlock(&buffer_lock);
            break;            
        }

        else{
            continue;
        }        

        
        if(num == BUFFER_NUMBER){
            num = 0;
            cout << num << endl;
        }

        
         
    }

    

    }
    

    close(socket_main);
}

void* thread_parshing_2(void* data_2)
{

    struct sockaddr_in Server;
    struct sockaddr_in Client;

    socklen_t Client_size;

    int socket_main;
    int Recv_Size;

    char Buffer_socket[BUFFER_SIZE];

    char Buffer_state[BUFFER_NUMBER][1080];
    
    int iter = 0;

    Client_size = sizeof(Client);

    socket_main = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    memset(Buffer_socket, 0, BUFFER_SIZE);

    Server.sin_family = AF_INET;
    Server.sin_addr.s_addr = inet_addr(HOST);
    Server.sin_port = htons(right_port);

    if ( bind(socket_main, (struct sockaddr *)&Server,
			sizeof(Server)) <0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

    if (recvfrom(socket_main, Buffer_socket, BUFFER_SIZE,
                0, (struct sockaddr*)&Client, &Client_size) <= 0)
    {
        cout << "Right Lidar read error!" << endl;

        while (true) {

            Recv_Size = recvfrom(socket_main, Buffer_socket, BUFFER_SIZE,
                0, (struct sockaddr*)&Client, &Client_size);
            
            if (Recv_Size >= 0)
                break;
            else {

                usleep(1000000);
                 //받은 데이터 출력

            }
        }
        exit(0);
    }

    cout << "Right Lidar connect" << endl;
    //================
    while (true){

        int num = 0;    
        memset(Buffer_state, 0, sizeof(Buffer_state));

        while (true) {
        memset(Buffer_socket, 0, sizeof(Buffer_socket));
        // receive lidar data
        recvfrom(socket_main, Buffer_socket, BUFFER_SIZE,
                0, (struct sockaddr*)&Client, &Client_size);
    
        int azimuth = (Buffer_socket[13 ] << 8 | Buffer_socket[12]) / 100;

        if (azimuth > 0){
            
                // cout << "azimuth : " << azimuth << endl;

                // save lidar data
                for(int i = 0; i < 1080; i++){
                    Buffer_state[num][i] = Buffer_socket[i];
                }
                num = num +1;
        }

        // stop saving data and parshing buffer
        if( azimuth> 323){
            pthread_mutex_lock(&buffer_lock);
            memset(Buffer_right, 0, sizeof(Buffer_right));
            memcpy(Buffer_right, Buffer_state, sizeof(Buffer_right));
            pthread_mutex_unlock(&buffer_lock);
            break;            
        }

        else{
            continue;
        }        
        
        
        if(num == BUFFER_NUMBER){
            num = 0;
            
        }

      
         
    }

    

    }
    

    close(socket_main);
}


void* thread_recv(void* data_2)
{

    float vertical_angle[32] = { 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,-12,-13,-14,-15,-16};
    float z_angle = 0;

    char Buffer_left_local[BUFFER_NUMBER][1080];
    char Buffer_right_local[BUFFER_NUMBER][1080];
    
    while (true) {
        
        // init
        int iter = 0;
        struct point PCD[50000];

        memcpy(Buffer_left_local, Buffer_left, sizeof(Buffer_left_local));
        memcpy(Buffer_right_local, Buffer_right, sizeof(Buffer_right_local));

        
        int len = sizeof(Buffer_left_local) / sizeof(Buffer_left_local[0]);

        pthread_mutex_lock(&buffer_lock);
        for (int j = 0; j < len; j++){
            
            for (int i = 0; i < 8; i++) {

            int azimuth = (Buffer_left_local[j][12 + 130 * i+1 ] << 8 | Buffer_left_local[j][12+130 * i]) / 100;
            
            if (azimuth > 0){
            
                // cout << "azimuth : " << azimuth << endl;

                for (int k = 0; k < 32; k++) {

                    float dist = (float)((int)((Buffer_left_local[j][12+130 * i + 1  + 4 * k + 1] << 8 | Buffer_left_local[j][12+130 * i + 1  + 4 * k + 2])) * 4/1000);
                    z_angle = vertical_angle[k];
                    int channel = k+1;
                    
                    PCD[iter].x = dist * cos(azimuth);
                    PCD[iter].y = dist * sin(azimuth) + 0.5;
                    PCD[iter].z = dist * sin(z_angle);
                    PCD[iter].azimuth = azimuth;
                    PCD[iter].ring = channel;
                    iter = iter +1;

                    }
                 }
            }
        }

        len = sizeof(Buffer_right_local) / sizeof(Buffer_right_local[0]);
        
        for (int  j= 0; j < len; j++){

            for (int i = 0; i < 8; i++) {

            int azimuth = (Buffer_right_local[j][12 + 130 * i+1 ] << 8 | Buffer_right_local[j][12+130 * i]) / 100;

            if (azimuth > 0){
            
                // cout << "azimuth : " << azimuth << endl;

                for (int k = 0; k < 32; k++) {

                    float dist = (float)((int)((Buffer_right_local[j][12+130 * i + 1  + 4 * k + 1] << 8 | Buffer_right_local[j][12+130 * i + 1  + 4 * k + 2])) * 4/1000);
                    z_angle = vertical_angle[k];
                    int channel = k+1;
                    
                    PCD[iter].x = dist * cos(azimuth);
                    PCD[iter].y = dist * sin(azimuth) - 0.5;
                    PCD[iter].z = dist * sin(z_angle);
                    PCD[iter].azimuth = azimuth;
                    PCD[iter].ring = channel;

                    iter = iter +1;
                    }
                }
            }
        }

        pthread_mutex_unlock(&buffer_lock);

        memset(Buffer_left_local, 0, sizeof(Buffer_left_local));
        memset(Buffer_right_local, 0, sizeof(Buffer_right_local));

        pcl::PointCloud<pcl::PointXYZI> raw_data_pc;
        pcl::PCLPointCloud2 raw_data_pc2;

        raw_data_pc.resize(iter); 

        for (int i = 0; i < iter; i++) {

            
            raw_data_pc.points[i].x = PCD[i].x;
            raw_data_pc.points[i].y = PCD[i].y;
            raw_data_pc.points[i].z = PCD[i].z;
            
        }

        pcl::toPCLPointCloud2(raw_data_pc, raw_data_pc2);
        sensor_msgs::PointCloud2 raw_data_pc2_result; 	//출력할 방식인 PC2 선정 및 이름 output_v 정의
        pcl_conversions::fromPCL(raw_data_pc2, raw_data_pc2_result);
        raw_data_pc2_result.header.frame_id = "Lidar";
        pub_raw_data.publish(raw_data_pc2_result);

    }
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Point_parshing_1");         //node name 
	ros::NodeHandle nh;                                 //nodehandle
    pub_raw_data = nh.advertise<sensor_msgs::PointCloud2> ("/raw_data", 10);
    pthread_t threadID_1;
    pthread_t threadID_2;
    pthread_t threadID_3;

    int thr_id_1;
    int thr_id_2;
    int thr_id_3;

    // threadID로 TID를 받아오고, threadRoutine라는 함수 포인터로 스레드를 실행한다.
    printf("Create Thread!\n");
    
    thr_id_1 = pthread_create(&threadID_1, NULL, thread_parshing, NULL);

    thr_id_2 = pthread_create(&threadID_2, NULL, thread_recv, NULL);

    thr_id_3 = pthread_create(&threadID_3, NULL, thread_parshing_2, NULL);

    if (thr_id_1 < 0)
    {
        perror("Left Lidar Thread Problem!\n");
        exit(EXIT_FAILURE);
    }

    if (thr_id_2 < 0)
    {
        perror("Ros Parshing Thread Problem!\n");
        exit(EXIT_FAILURE);
    }

    if (thr_id_3 < 0)
    {
        perror("Right Lidar Thread Problem!\n");
        exit(EXIT_FAILURE);
    }

    // threadID를 가진 thread가 실행되는 동안 기다린다.
    printf("creating thread success!\n");
    pthread_join(threadID_3, NULL);
    pthread_join(threadID_1, NULL);
    pthread_join(threadID_2, NULL);
    printf("Code Start!\n");
    

    ros::spin();
}
