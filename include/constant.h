#ifndef CONSTANT_H
#define CONSTANT_H

const int		SKIN_X = 320;
const int		SKIN_Y = 240;
const float MIN_HAND_PIX = 21; // 11 pixels
const int		HAND_GRID_SIZE = 441;// 15x15
const int		HAND_SIZE = HAND_GRID_SIZE;

const int HAND_MAX_TRANSMIT_SIZE = 120;

//for file loading
#define MARKER_FILENAME "ARMM/Data/Celica.bmp"
//#define MARKER_FILENAME "Data/thai_art.jpg"
#define CAMERA_PGR_PARAMS_FILENAME "ARMM/Data/Cameras/camera_pointgrey.yml"
#define CAMERA_PARAMS_FILENAME "ARMM/Data/Cameras/camera.yml"
#define KINECT_PARAMS_FILENAME "ARMM/Data/Cameras/kinect.yml"
#define KINECT_TRANSFORM_FILENAME "ARMM/Data/Cameras/KinectTransform.yml"
#define KINECT_CONFIG_FILENAME "ARMM/Data/Cameras/KinectConfig.xml"

#define CAR1_BODY_FILENAME "ARMM/Data/Cars/GT4_body.ive"
#define CAR1_WHEEL_FILENAME "ARMM/Data/Cars/GT4_tire.ive"
#define CAR2_BODY_FILENAME "ARMM/Data/Cars/Murcielago_body.ive"
#define CAR2_WHEEL_FILENAME "ARMM/Data/Cars/Murcielago_tire.ive"

//for image processing
#define CAPTURE_SIZE cvSize(640,480)
#define REGISTRATION_SIZE cvSize(320,240)
#define MESH_SIZE cvSize(160,120)
#define OPFLOW_SIZE cvSize(SKIN_X,SKIN_X)
#define SKIN_SEGM_SIZE cvSize(SKIN_X,SKIN_Y)

//for VRPN connection
const int UDP_LIMITATION = 100;

//const float MIN_HAND_PIX = 15; // 11 pixels
//const int HAND_GRID_SIZE = 225;// 15x15

#endif
