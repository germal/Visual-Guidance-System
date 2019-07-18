#include <sl/Camera.hpp>

using namespace sl;
using namespace std;

void saveDepthIm();

char depth_name[40];

Camera zed;

int main(int argc, char **argv) {
    // Create ZED objects
    InitParameters initParameters;
    initParameters.svo_input_filename.set(argv[1]);
	initParameters.depth_mode = DEPTH_MODE_QUALITY;
	initParameters.coordinate_units = UNIT_MILLIMETER;

    // Open the ZED
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    int svo_frame_rate = zed.getCameraFPS();
    int nb_frames = zed.getSVONumberOfFrames();
    cout<<"No of Frames in SVO file: "<<nb_frames<<endl; 

    //zed.setDepthMaxRangeValue(40000);
  	// Grab depth in FILL mode
  	RuntimeParameters runtime_parameters; 
  	runtime_parameters.sensing_mode = SENSING_MODE_FILL;
    // Start SVO playback
    int imcount = 0;
    while (imcount < nb_frames) {
		if (zed.grab() == SUCCESS) {

			// Get the side by side image
			sprintf(depth_name, "../data/depth/depth-%d.png", imcount);
			saveDepthIm();
			imcount++;
    	}
    	else
    		return 1;
	}
    zed.close();
    return 0;
	
}

void saveDepthIm()
{
	// Mat depth_map;
	// zed.retrieveMeasure(depth_map, MEASURE_DEPTH);
	// float depth_value=0;
	// depth_map.getValue(200,200,&depth_value);
	// cout <<depth_value<<"..........."<<endl;
	//depth_map.write(depth_name);
    saveDepthAs(zed, DEPTH_FORMAT_PNG, depth_name);// saves a 16-bit Greyscale image 
}