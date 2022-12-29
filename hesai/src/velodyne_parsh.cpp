#define HAVE_STRUCT_TIMESPEC 
#define BUFFER_SIZE 10000 // 버퍼 사이즈
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

ros::Publisher pub_raw_data;

char Buffer_global[BUFFER_SIZE] = {};

struct point
{
    int azimuth;
    float x, y, z, r, theta, hori, layer;
    int flag;
    int ring;

};

struct point PCD[384];



void* thread_parshing(void* data_1)
{
    int   L_ServerPort = 2367; // 서버 포트번호
    struct sockaddr_in L_ToServer;
    int L_ClientSocket;
    int client_len;
    char Buffer_parshing[BUFFER_SIZE] = {};

    cout << "1" << endl;
    if ((L_ClientSocket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        cout << "error" << endl;
        

    }
    memset(Buffer_parshing, 0, BUFFER_SIZE);

    /* 연결 요청할 서버의 주소와 포트 번호 프로토콜 등을 지정한다. */

    L_ToServer.sin_family = AF_INET;
    L_ToServer.sin_addr.s_addr = inet_addr("127.0.0.1");
    L_ToServer.sin_port = htons(L_ServerPort);

    client_len = sizeof(L_ToServer);
    cout << "2" << endl;
    /* 서버에 연결을 시도한다. */
    if (connect(L_ClientSocket, (struct sockaddr*)&L_ToServer, client_len) == -1) {
        cout << "error" << endl;

    }

    cout << "3" << endl;
    if (read(L_ClientSocket, Buffer_parshing, BUFFER_SIZE) <= 0)
    {
        cout << "read error!" << endl;

        while (true) {

            if (read(L_ClientSocket, Buffer_parshing, BUFFER_SIZE) >= 0)
                break;
            else {

                cout << "재실행" << endl;
                usleep(1000000);
            }

        }
        exit(0);
    }
    cout << "4" << endl;
    while (true) {
        cout << Buffer_parshing << endl;
        memset(Buffer_parshing, 0, sizeof(Buffer_parshing));

        read(L_ClientSocket, Buffer_parshing, BUFFER_SIZE);

        memcpy(Buffer_global, Buffer_parshing, BUFFER_SIZE);
    }

    close(L_ClientSocket);
}


void* thread_recv(void* data_2)
{
    int i;
    int k;
    float x_pcd = 0;
    float y_pcd = 0;
    float z_pcd = 0;
    int channel;


    float lidar[100] = { 0 };
    float vertical_angle[16] = { -15,1,13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15 };
    float z_angle = 0;

    while (true) {


        for (i = 0; i < 12; i++) {

            int azimuth = (Buffer_global[100 * i + 3] << 8 | Buffer_global[100 * i + 2]) / 100;

            for (k = 1; k <= 32; k++) {

                float dist = (float)((int)((Buffer_global[100 * i + 3 * k + 1] << 8 | Buffer_global[100 * i + 3 * k + 2]) * 2) / 1000);
                if (k >= 17) {
                    z_angle = vertical_angle[k - 17];
                    channel = k - 16;
                }
                else {
                    z_angle = vertical_angle[k - 1];
                    channel = k;
                }

                PCD[32 * i + k].x = dist * cos(azimuth);;
                PCD[32 * i + k].y = dist * sin(azimuth);
                PCD[32 * i + k].z = dist * sin(z_angle);
                PCD[32 * i + k].azimuth = azimuth;
                PCD[32 * i + k].ring = channel;
            }

        }

        //memset(Buffer_global, 0, sizeof(Buffer_global));

        // for (i = 0; i < 326; i++) {
        //     printf("%d %f %f %f\n", PCD[i].azimuth, PCD[i].x, PCD[i].y, PCD[i].z);
        // }

        pcl::PointCloud<pcl::PointXYZI> raw_data_pc;
        pcl::PCLPointCloud2 raw_data_pc2;
        raw_data_pc.resize(326); //cloud의 size를 3으로 설정

        for (i = 0; i < 326; i++) {
            raw_data_pc.points[i].x = PCD[i].x;
            raw_data_pc.points[i].y = PCD[i].y;
            raw_data_pc.points[i].z = PCD[i].z;
            
        }

        pcl::toPCLPointCloud2(raw_data_pc, raw_data_pc2);
        sensor_msgs::PointCloud2 raw_data_pc2_result; 	//출력할 방식인 PC2 선정 및 이름 output_v 정의
        pcl_conversions::fromPCL(raw_data_pc2, raw_data_pc2_result);
        raw_data_pc2_result.header.frame_id = "velodyne";
        pub_raw_data.publish(raw_data_pc2_result);

    }
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Velodyne_Point_parshing_1");         //node name 
	ros::NodeHandle nh;                                 //nodehandle
    pub_raw_data = nh.advertise<sensor_msgs::PointCloud2> ("/raw_data", 1);
    pthread_t threadID_1;
    pthread_t threadID_2;
    pthread_t threadID_3;

    int thr_id_1;
    int thr_id_2;

    // threadID로 TID를 받아오고, threadRoutine라는 함수 포인터로 스레드를 실행한다.
    printf("Create Thread!\n");
    
    thr_id_1 = pthread_create(&threadID_1, NULL, thread_parshing, NULL);

    thr_id_2 = pthread_create(&threadID_2, NULL, thread_recv, NULL);

    if (thr_id_1 < 0)
    {
        perror("parshing thread create error");
        exit(EXIT_FAILURE);
    }

    if (thr_id_2 < 0)
    {
        perror("receive thread create error");
        exit(EXIT_FAILURE);
    }

    // threadID를 가진 thread가 실행되는 동안 기다린다.
    printf("creating thread success!\n");

    pthread_join(threadID_1, NULL);
    pthread_join(threadID_2, NULL);
    printf("Code Start!\n");
    

    ros::spin();
}

