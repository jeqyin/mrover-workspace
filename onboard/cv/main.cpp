#include "perception.hpp"
#include <unistd.h>

#include <mutex>
#include <shared_mutex>
#include <thread>
#include <condition_variable>

using namespace cv;
using namespace std;


struct Frame{
  Mat color; 
  Mat depth;
  unsigned sequence_num;
};

//shared data
Frame frame;
shared_timed_mutex frame_lock;

Frame frameTennisball;
shared_timed_mutex frameTennisball_lock;

Frame frameObstacle;
shared_timed_mutex frameObstacle_lock;

lcm::LCM lcm_;
shared_timed_mutex lcm_lock;

condition_variable_any next_frame;

int calcFocalWidth(){   //mm
    return tan(fieldofView/2) * focalLength;
}

int calcRoverPix(float dist, float pixWidth){   //pix
    float roverWidthSensor = realWidth * 1.2  * focalLength/(dist * 1000);
    return roverWidthSensor*(pixWidth/2)/calcFocalWidth();
}

float getGroundDist(float angleOffset){  // the expected distance if no obstacles
    return zedHeight/sin(angleOffset);
}

double getAngle(float xPixel, float wPixel){
    return atan((xPixel - wPixel/2)/(wPixel/2)* tan(fieldofView/2))* 180.0 /PI;
}

float getObstacleMin(float expected){
    return expected - obstacleThreshold/sin(angleOffset);
}

bool cam_grab_succeed(Camera &cam, int & counter_fail) {
  while (!cam.grab()) {
    counter_fail++;
    if (counter_fail > 1000000) {
      cerr<<"camera failed\n";
      return false;
    }
  }
  counter_fail = 0;
  return true;
}

static string rgb_foldername, depth_foldername;
void disk_record_init() {
  if (WRITE_CURR_FRAME_TO_DISK) {
    // write images colleted to the folder
    // absolute path
    rgb_foldername = DEFAULT_ONLINE_DATA_FOLDER + "rgb/"; 
    depth_foldername = DEFAULT_ONLINE_DATA_FOLDER + "depth/" ;
    string mkdir_rgb =  std::string("mkdir -p ") + rgb_foldername;
    string mkdir_depth =  std::string("mkdir -p ") + depth_foldername;
    int dir_err_rgb = system( mkdir_rgb.c_str() );
    int dir_err_depth = system(mkdir_depth.c_str());
    if (-1 == dir_err_rgb || -1 == dir_err_depth) {
      exit(1);
    }
  }
}

void write_curr_frame_to_disk(Mat &rgb, Mat & depth, int counter ) {
    cv::imwrite(rgb_foldername +  std::to_string(counter) + std::string(".jpg"), rgb );
    //std::string file_str = std::string("depth_") + std::to_string(counter);// + std::string(".jpg");
    
    cv::imwrite(depth_foldername +  std::to_string(counter) + std::string(".exr"), depth );
}

void camera_thread(){
  cout << "camera_thread\n";

  //initialize camera variables
  Camera cam;
  int j = 0;
  // double frame_time = 0;
  int counter_fail = 0;
  
  //initialize data recording variables if enabled
  disk_record_init();

  while(true){
    get_frame(Camera &cam, j, counter_fail);
  }
}

void get_frame(Camera &cam, int j, int counter_fail){

    if (!cam_grab_succeed(cam, counter_fail)) break;
  
    // auto start = chrono::high_resolution_clock::now();
    // Mat src = cam.image();
  
    //grab color and depth images
    // unique_lock<shared_timed_mutex> write_lock(frame_lock);
    frame_lock.lock();
    cout << "locking camera camera\n";
    frame.color = cam.image();
    frame.depth = cam.depth();
    frame.sequence_num += 1;
    // write_lock.unlock();
    next_frame.notify_all();
    cout << "unlocking camera camera\n";
    frame_lock.unlock();

    //display source images
    // #ifdef PERCEPTION_DEBUG
    //   imshow("image", depth);
    // #endif

    // write to disk if permitted
    if(WRITE_CURR_FRAME_TO_DISK){
      write_curr_frame_to_disk(frame.color, frame.depth, j);
      j++;
    }
}

void obstacle_thread(){
  cout << "obstacle_thread\n";

  //initialize lcm
  rover_msgs::Obstacle obstacleMessage;
  obstacleMessage.detected = false;

  unsigned seq_tracker = 0;

  while(true){

    //TODO check if empty
    shared_lock<shared_timed_mutex> frame_read_lock(frame_lock);
      // cout << "locking camera obs\n";

    while(frame.sequence_num == seq_tracker){
      next_frame.wait(frame_read_lock);
    }

    Frame cur_frame = frame;
          // cout << "unlocking camera obs\n";

    frame_read_lock.unlock();

    seq_tracker = cur_frame.sequence_num;

    /*initialize obstacle detection*/
    float pixelWidth = cur_frame.color.cols;
    int roverPixWidth = calcRoverPix(distThreshold, pixelWidth);

    /* obstacle detection */
    //TODO pass in frame by reference
    obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(cur_frame.depth, cur_frame.color, num_sliding_windows, roverPixWidth);

    //if bearing is too small, ignore
    if(obstacle_detection.bearing > 0.05 || obstacle_detection.bearing < -0.05) {
      cout<< "bearing not zero!\n";
      obstacleMessage.detected = true;    //if an obstacle is detected in front
    } else {
      cout<<"bearing zero\n";
      obstacleMessage.detected = false;
    }

    obstacleMessage.bearing = obstacle_detection.bearing;

    // #ifdef PERCEPTION_DEBUG
    //   cout << "Turn " << obstacleMessage.bearing << ", detected " << (bool)obstacleMessage.detected<< endl;
    // #endif
    #ifdef PERCEPTION_DEBUG
      unique_lock<shared_timed_mutex> o_write_lock(frameObstacle_lock);
        frameObstacle = cur_frame;
      o_write_lock.unlock();
      // imshow("obs_image", cur_frame.src_color);
      // waitKey(FRAME_WAITKEY);
    #endif

    shared_lock<shared_timed_mutex> lcm_read_lock(lcm_lock);
    lcm_.publish("/obstacle", &obstacleMessage);
    lcm_read_lock.unlock();

  }
}

void tennisball_thread(){
  cout << "tennisball_thread\n";

  int tennisBuffer = 0;

  //initialize lcm
  rover_msgs::TennisBall tennisMessage;
  tennisMessage.found = false;

  unsigned seq_tracker = 0;

  while(true){

    shared_lock<shared_timed_mutex> frame_read_lock(frame_lock);
          // cout << "locking camera ten\n";

    while(!frame.sequence_num){
      next_frame.wait(frame_read_lock);
    }

    Frame cur_frame = frame;
          // cout << "unlocking camera ten\n";

    frame_read_lock.unlock();

    seq_tracker = cur_frame.sequence_num;

    vector<Point2f> centers = findTennisBall(cur_frame.color, cur_frame.depth);
    if(centers.size() != 0){
      float dist = cur_frame.depth.at<float>(centers[0].y, centers[0].x);
      if (dist < BALL_DETECTION_MAX_DIST) {
        tennisMessage.distance = dist;
        tennisMessage.bearing = getAngle((int)centers[0].x, cur_frame.color.cols);

        tennisMessage.found = true;
        tennisBuffer = 0;

        // #ifdef PERCEPTION_DEBUG
        // cout << centers.size() << " tennis ball(s) detected: " << tennisMessage.distance 
        //                                                 << "m, " << tennisMessage.bearing << "degrees\n";
        // #endif

        #ifdef PERCEPTION_DEBUG
          unique_lock<shared_timed_mutex> tb_write_lock(frameTennisball_lock);
          frameTennisball = cur_frame;
          tb_write_lock.unlock();
          // imshow("tennis_image", cur_frame.src_color);
          // waitKey(FRAME_WAITKEY);
        #endif

      }else if(tennisBuffer < 5){   //give 5 frames to recover if tennisball lost due to noise
        tennisBuffer++;
      }else
        tennisMessage.found = false;
    }
  }
  shared_lock<shared_timed_mutex> lcm_read_lock(lcm_lock);
  lcm_.publish("/tennis_ball", &tennisMessage);
  lcm_read_lock.unlock();
}


int main() {

  

  thread cameraThread(camera_thread);
  thread obstacleThread(obstacle_thread);
  thread tennisballThread(tennisball_thread);

  #ifdef PERCEPTION_DEBUG
    shared_lock<shared_timed_mutex> lockTb(frameTennisball_lock);
    shared_lock<shared_timed_mutex> lockO(frameObstacle_lock);
    // unique_lock<shared_timed_mutex> lockCamera(frame_lock);

    cout << "locking camera main\n";

    while(true){

      while(!frame.sequence_num){
        next_frame.wait(lockO);
      }

      if(!(frameTennisball.color.empty() || frameObstacle.color.empty())){
        imshow("TB_Detection", frameTennisball.color);
        imshow("O_Detection", frameObstacle.color);
        waitKey(FRAME_WAITKEY);
      }
      
      cout << "unlocking camera main\n";
    
      lockCamera.unlock();
      lockTb.unlock();
      lockO.unlock();

      cout << "locking camera main\n";

      lockTb.lock();
      lockO.lock();
      lockCamera.lock();
    }
  #endif

  cameraThread.join();
  obstacleThread.join();
  tennisballThread.join();

  // /*initialize camera*/
  // Camera cam;
  // int j = 0;
  // double frame_time = 0;
  // int counter_fail = 0;
  // #ifdef PERCEPTION_DEBUG
  //   namedWindow("image", 1);
  //   namedWindow("depth", 2);
  // #endif
  // disk_record_init();

  // /*initialize lcm messages*/
  // lcm::LCM lcm_;
  // rover_msgs::TennisBall tennisMessage;
  // rover_msgs::Obstacle obstacleMessage;
  // tennisMessage.found = false;
  // obstacleMessage.detected = false;

  // int tennisBuffer = 0;
  
  // while (true) {
  //   if (!cam_grab_succeed(cam, counter_fail)) break;
  
  //   auto start = chrono::high_resolution_clock::now();
  //   Mat src = cam.image();
    
  //   #ifdef PERCEPTION_DEBUG
  //         imshow("image", src);
  //   #endif
  //   Mat depth_img = cam.depth();

  //   // write to disk if permitted
  //   if(WRITE_CURR_FRAME_TO_DISK){
  //     write_curr_frame_to_disk(src, depth_img, j );
  //   }

  //   /*initialize obstacle detection*/
  //   float pixelWidth = src.cols;
  //   //float pixelHeight = src.rows;
  //   int roverPixWidth = calcRoverPix(distThreshold, pixelWidth);

  //   /* obstacle detection */
  //   obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(depth_img, src,  num_sliding_windows , roverPixWidth);
  //   if(obstacle_detection.bearing > 0.05 || obstacle_detection.bearing < -0.05) {
  //     cout<< "bearing not zero!\n";
  //     obstacleMessage.detected = true;    //if an obstacle is detected in front
  //   } else {
  //     cout<<"bearing zero\n";
  //     obstacleMessage.detected = false;
  //   }
  //   obstacleMessage.bearing = obstacle_detection.bearing;

  //   #ifdef PERCEPTION_DEBUG
  //     cout << "Turn " << obstacleMessage.bearing << ", detected " << (bool)obstacleMessage.detected<< endl;
  //   #endif

  //   /* Tennis ball detection*/
  //   vector<Point2f> centers = findTennisBall(src, depth_img);
  //   if(centers.size() != 0){
  //     float dist = depth_img.at<float>(centers[0].y, centers[0].x);
  //     if (dist < BALL_DETECTION_MAX_DIST) {
  //       tennisMessage.distance = dist;
  //       tennisMessage.bearing = getAngle((int)centers[0].x, src.cols);

  //       tennisMessage.found = true;
  //       tennisBuffer = 0;

  //       #ifdef PERCEPTION_DEBUG
  //       cout << centers.size() << " tennis ball(s) detected: " << tennisMessage.distance 
  //                                                       << "m, " << tennisMessage.bearing << "degrees\n";
  //       #endif

  //     }else if(tennisBuffer < 5){   //give 5 frames to recover if tennisball lost due to noise
  //       tennisBuffer++;
  //     }else
  //       tennisMessage.found = false;
  //   }

  //   lcm_.publish("/tennis_ball", &tennisMessage);
  //   lcm_.publish("/obstacle", &obstacleMessage);

  //   #ifdef PERCEPTION_DEBUG
  //     imshow("depth", depth_img);
  //     imshow("image", src);
  //     waitKey(FRAME_WAITKEY);
  //   #endif
  //   auto end = chrono::high_resolution_clock::now();

  //   auto delta = chrono::duration_cast<chrono::duration<double>>(end - start);
  //   frame_time += delta.count();
  //   #ifdef PERCEPTION_DEBUG
  //       if(j % 100 == 0){
  //           cout << "framerate: " << 1.0f/(frame_time/j) << endl;
  //       }
  //   #endif
  //   j++;
  // }

  return 0;
}
