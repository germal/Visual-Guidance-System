#include <sl/Camera.hpp>
#include <ctime>
#include <signal.h>

static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}


using namespace sl;
using namespace std;

void saveLeftIm(Camera *);
void saveRightIm(Camera *);
//void saveDepthIm(Camera *);
char l_image_name[40];
char r_image_name[40];
//char depth_name[40];
int main(int argc, char **argv) {

    if (argc != 2) {
        std::cout << "Only the path of the output SVO file should be passed as argument.\n";
        return 1;
    }
    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_VGA; // Use HD1080 video mode
    init_params.camera_fps = 100; // Set fps at 30
    init_params.depth_mode = DEPTH_MODE_NONE;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);
    // Enable recording with the filename specified in argument
    String path_output(argv[1]);
    err = zed.enableRecording(path_output, SVO_COMPRESSION_MODE_AVCHD);

    if (err != SUCCESS) {
        std::cout << "Recording initialization error. " << toString(err) << std::endl;
        if (err == ERROR_CODE_SVO_RECORDING_ERROR)
            std::cout << " Note : This error mostly comes from a wrong path or missing writing permissions." << std::endl;
        if (err == ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION)
            std::cout << " Note : This error mostly comes from a non-compatible graphic card. If you are using HEVC compression (H265), please note that most of the graphic card below pascal architecture will not support it. Prefer to use AVCHD compression which is supported on most of NVIDIA graphic cards" << std::endl;

        zed.close();
        return 1;
    }

    int imcount = 0;
 
    signal(SIGINT, intHandler);
    while (keepRunning) {
        clock_t begin = clock();  
        // Check that grab() is successful
        if (zed.grab() == SUCCESS) {
            sl::RecordingState state = zed.record();
            sprintf(l_image_name, "../data/l_image/l_image-%d.jpg", imcount);
            sprintf(r_image_name, "../data/r_image/r_image-%d.jpg", imcount);
            //sprintf(depth_name, "../data/depth/depth-%d.png", imcount);
            
            saveLeftIm(&zed);
            saveRightIm(&zed);
            //saveDepthIm();
            if (state.status)
                ++imcount;

            double time_t;
            time_t = double(clock() - begin) / CLOCKS_PER_SEC; 
            cout << "TIME: " << time_t << endl;
     
        }
    }

    zed.disableRecording();
        // Close the camera
    zed.close();
    return 0;
}

void saveLeftIm(Camera *zed)
{
  Mat zed_image;
  zed->retrieveImage(zed_image, VIEW_LEFT);
  zed_image.write(l_image_name);
}

void saveRightIm(Camera *zed)
{
  Mat zed_image;
  zed->retrieveImage(zed_image, VIEW_RIGHT);
  zed_image.write(r_image_name);
}

// void saveDepthIm()
// {
//   saveDepthAs(zed, DEPTH_FORMAT_PNG, depth_name);
// }