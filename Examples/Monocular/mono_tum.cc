#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include <stdlib.h>
#include<opencv2/core/core.hpp>
#include <Converter.h>	
#include<System.h>
#include "cv.h" 
#include "highgui.h"

#include <atomic>
#include <csignal>
#include <thread>
#include <pthread.h>
#include <semaphore.h>
#include <opencv2/core/cvstd.hpp>

// 640*480
#define WIDTH 640
#define HEIGHT 480

// RGB
#define NUMCOLORS 3

#define BUFFER_SIZE 2

void saveMap(ORB_SLAM2::System &SLAM);
void init_vars();

typedef struct threads{
	std::unique_ptr<std::thread> frame_producer;
	std::unique_ptr<std::thread> pose_producer;
}threads_t;

/*
https://stackoverflow.com/questions/33875583/call-python-from-c-opencv-3-0
*/
struct frame
{
	uint8_t data[HEIGHT] [WIDTH] [NUMCOLORS];
};

// global variables
sem_t full,empty;
pthread_mutex_t mutex1;
pthread_mutex_t mutex2;
int in=0;
int out=0; 
cv::Ptr<cv::Mat> buffer[BUFFER_SIZE];
std::string fstrPointData;
std::atomic<bool> continue_slam(true);
// Create SLAM system. It initializes all system threads and gets ready to process frames.
ORB_SLAM2::System* SLAM;
threads_t threads;

double consume_time = 1;
double produce_time = 1;


void pose_data_generator()
{
	while(continue_slam)
		std::cout << std::endl;
	// mpCurrentKeyframe->getInversePose()
}

void start_localization_SIGUSR1(int signum)
{
	printf("Ive recived signal1");
	threads.pose_producer = std::unique_ptr<std::thread>(new std::thread(pose_data_generator));
}

void stop_slam_SIGUSR2(int signum)
{
	printf("Ive recived signal2");
	continue_slam = false;
}

/*
void stop_exec_SIGABRT(int signum)
{
	exit(signum);
}
*/



/* https://github.com/codophobia/producer-consumer-problem-solution-in-c/blob/master/producer-consumer.c */
cv::Mat getframe()
{
	cv::Ptr<cv::Mat> smartPtr;	
	sem_wait(&full);
	pthread_mutex_lock(&mutex1);

	

	/* CONSUME */
	cv::Mat frame = buffer[out]->clone();
	out = (out + 1) % BUFFER_SIZE;
	/*
	bool bufferNotTooFull = is_bufferNotTooFull();
	std::cout<< "bufferNotTooFull from getframe: " << bufferNotTooFull << std::endl;
	if (bufferNotTooFull)
		pthread_cond_signal(&c);
	*/	


	pthread_mutex_unlock(&mutex1);
	sem_post(&empty);
	return frame;
} 

void frames_generator()
{
	struct frame fr = {};
	cv::Ptr<cv::Mat> smartPtr;
	int counter = 0;
	while(true)
	{
		

		sem_wait(&empty);
		pthread_mutex_lock(&mutex1);

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
		/* PRODUCE */
		
		// Reads in the raw data
		do{
			fr = {};
			std::fread(&fr, 1, sizeof(fr.data), stdin);

			// Rebuild raw data to cv::Mat
			smartPtr = new cv::Mat(HEIGHT, WIDTH, CV_8UC3, *fr.data);
		}while(smartPtr->empty() && continue_slam);
		// before passing into slam system
		cv::cvtColor(*smartPtr, *smartPtr, CV_BGR2RGB);

		buffer[in] = smartPtr;
		in = (in + 1) % BUFFER_SIZE;

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		pthread_mutex_lock(&mutex2);
        produce_time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        pthread_mutex_unlock(&mutex2);

		pthread_mutex_unlock(&mutex1);
		sem_post(&full);
		counter++;
		std::cout << "counter is " << counter << std::endl; 
		if (counter > 50)
		{
			sleep(consume_time * 0.95);
		}
		
	}
}

int main()
{
	// register signal SIGINT, SIGTERM and signal handlers
   	signal(SIGINT, start_localization_SIGUSR1);
   	signal(SIGUSR2, stop_slam_SIGUSR2);
   	init_vars();
	double time_stamp;
	int counter = 0;

    
    std::cout << std::endl << "-------" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;

	/* https://stackoverflow.com/questions/18375998/is-it-possible-to-define-an-stdthread-and-initialize-it-later */
	threads.frame_producer = std::unique_ptr<std::thread>(new std::thread(frames_generator));

	// Main loop
	while (continue_slam)
	{
		#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        cv::imshow( "press ESC on this screen to quit", getframe());
		if ( (cv::waitKey(50) & 255) == 27 ) break; 

        

		/*http://20sep1995.blogspot.com/2019/02/how-to-run-orb-slam-with-your-own-data.html*/
		time_stamp = (0.2) * (counter + 1);

		/*
		
		int empty_val;
		sem_getvalue(&empty, &empty_val);
		int full_val;
		sem_getvalue(&full, &full_val);
		std::cout << "(producer) in= "<< in << ", (producer) full= "<< full_val << std::endl << 
		"(consumer) out= " << out << ", (consumer) empty= "<< empty_val << std::endl << std::endl;


		// Pass the image to the SLAM system
        SLAM->TrackMonocular(getframe(), time_stamp);

        #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        
        pthread_mutex_lock(&mutex2);
        consume_time= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        std::cout << "(producer) produce_time= " << produce_time << std::endl <<
        "(consumer) consume_time= " << consume_time << std::endl << std::endl;
        pthread_mutex_unlock(&mutex2);
        counter++;
 		
 		*/
        /*
		// Wait to load the next frame
		double T = (0.4) * (counter + 2) - time_stamp;
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
		*/
	}

	threads.frame_producer->join();

	/*
	// Stop all threads
    SLAM->Shutdown();
    
    // save the map
    saveMap(*SLAM);

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    delete SLAM;
    */
   	return 0;
}







void saveMap(ORB_SLAM2::System &SLAM)
{
    std::cout<<"saving the map"<<std::endl;

    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open(fstrPointData);
    for(auto p : mapPoints)
    {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
}

void init_vars()
{
	std::string home_env_p(std::getenv("HOME"));
	std::cout << "the path is "<< home_env_p << std::endl;
	std::string path_to_vocabulary = home_env_p + "/ORB_SLAM2/Vocabulary/ORBvoc.txt";
	std::string path_to_settings = home_env_p + "/ORB_SLAM2/Examples/Monocular/TUM1.yaml";
	std::string fstrPointData = home_env_p + "/PointData.csv";

	sem_init(&empty,0,BUFFER_SIZE);
	sem_init(&full,0,0);
	pthread_mutex_init(&mutex1,NULL);
	pthread_mutex_init(&mutex2,NULL);

	// SLAM = new ORB_SLAM2::System(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR,true);
} 
