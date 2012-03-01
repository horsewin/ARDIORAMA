//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "ARMM/ARMM_Client.h"

#ifdef POINTGREYCAMERA
#include "ARMM/PointgreyCamera.h"
#include <gvars3/GStringUtil.h>
#include <gvars3/instances.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#endif

//OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
//OPIRA
#include "OPIRAlibrary.h"
#include "OPIRALibraryMT.h"
#include "RegistrationAlgorithms/OCVSurf.h"

//Graphics calls
#include "ARMM/osg_Client.h"
#include "ARMM/ARMM_vrpn.h"
#include "ARMM/leastsquaresquat.h"

//Network using VRPN
#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"

//Keyboard controller
#include "ARMM/KeyboardControls_client.h"

#include <stdio.h>

using namespace std;
using namespace CVD;
using namespace GVars3;

namespace ARMM{
	//---------------------------------------------------------------------------
	// Global
	//---------------------------------------------------------------------------
	bool running = true;
	//these coord are considered as Bullet coord
	//so you have to correct to use in OSG coord
	float hand_coord_x[MAX_NUM_HANDS][HAND_GRID_SIZE];
	float hand_coord_y[MAX_NUM_HANDS][HAND_GRID_SIZE];
	float hand_coord_z[MAX_NUM_HANDS][HAND_GRID_SIZE];

	CvMat		*RegistrationParams;
	#ifdef USE_CLIENT_SENDER
	//send message to server
	vrpn_Connection_IP* m_Connection;
	ARMMClientSender* ARMM_client_sender;
	#endif

	//osg objects
	osg::Quat		CarsArrayQuat[NUM_CARS];
	osg::Quat		WheelsArrayQuat[NUM_CARS][4];
	osg::Vec3d		CarsArrayPos[NUM_CARS];
	osg::Vec3d		WheelsArrayPos[NUM_CARS][4];
	osg::Vec3d		ObjectsArrayPos[NUM_OBJECTS];
	osg::Quat		ObjectsArrayQuat[NUM_OBJECTS];

	void RenderScene(IplImage *arImage, Capture *capture);
	void DeleteLostObject(void);
	void InitVirtualObjectsCoordinate(void);

	//---------------------------------------------------------------------------
	// Constant/Define
	//---------------------------------------------------------------------------
	const float OSG_SCALE = 10;
	const char *ARMM_SERVER_IP = "ARMM_Comm@192.168.100.128"; //this value depends on IP address assigned computer

	//---------------------------------------------------------------------------
	// Code
	//---------------------------------------------------------------------------
	///////// Sending message to server for client //////////
	#ifdef USE_CLIENT_SENDER
	ARMMClientSender::ARMMClientSender( vrpn_Connection_IP *c )
		:vrpn_Tracker( "ARMM_Client_Server", c )
	{
		m_pass = 0;
	}

	ARMMClientSender::~ARMMClientSender()
	{
	}

	void ARMMClientSender::mainloop()
	{
		vrpn_gettimeofday(&_timestamp, NULL);
		vrpn_Tracker::timestamp = _timestamp;

		d_sensor = m_pass;
		char msgbuf[1000];
		int  len = vrpn_Tracker::encode_to(msgbuf);

		//printf("Pos, sensor %d = %f, %f, %f\n", d_sensor, (float) pos[0],(float) pos[1],(float) pos[2]);

		if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf,vrpn_CONNECTION_LOW_LATENCY)) {
			fprintf(stderr,"can't write message: tossing\n");
		}

		//update server main loop
	  server_mainloop();
	}

	void ARMMClientSender::SetPasskey(const int & pass)
	{
		m_pass = pass;
	}

	int ARMMClientSender::GetPasskey( void )const
	{
		return m_pass;
	}

	/////////////////////////////////////////////////////////
	#else
	int m_pass;
	#endif
	///////// End of ARMMClientSender definition ////////////////

	///////// Receiving message from ARMM server //////////
	ARMMClient::ARMMClient (const char * name, vrpn_Connection *cn)
		: vrpn_Tracker_Remote (name, cn)
	{
		register_types();

		if( d_connection != NULL){
			// Register a handler for the acceleration change callback.
			if( register_autodeleted_handler( hand_m_id, handle_hand_change_message, this, d_sender_id) )
			//if( register_autodeleted_handler( hand_m_id, handle_hand_change_message, this, d_sender_id))
			{
				fprintf(stderr, "ARMMClient: can't register hand handler\n");
				d_connection = NULL;
			}
		}
		else
		{
			fprintf(stderr, "ARMMClient: can't get connection!!\n");
		}

		//init parameter
		REP(i,HAND_SIZE){
			hand_coord_x[0][i] = 0.0;
			hand_coord_y[0][i] = 0.0;
			hand_coord_z[0][i] = 0.0;
		}
	}

	//virtual
	ARMMClient::~ARMMClient( void )
	{
	}

	void ARMMClient::mainloop( void )
	{
		if( d_connection ){
			d_connection->mainloop();
		}
		client_mainloop();
	}

	int ARMMClient::register_change_handler(void *userdata,
			vrpn_TRACKERHANDCHANGEHANDLER handler, vrpn_int32 whichSensor)
	{
		if (whichSensor < vrpn_ALL_SENSORS) {
			fprintf(stderr,
					"ARMMClient::register_change_handler: bad sensor index\n");
			return -1;
	  }
		// Ensure that the handler is non-NULL
		if (handler == NULL) {
			fprintf(stderr,
			   "ARMMClient::register_change_handler: NULL handler\n");
			return -1;
		}

	  // If this is the ALL_SENSORS value, put it on the all list; otherwise,
	  // put it into the normal list.
	  if (whichSensor == vrpn_ALL_SENSORS) {
			d_callback_list.register_handler(userdata, handler);
		  //return all_sensor_callbacks.d_handchange.register_handler(userdata, handler);
		} else if (ensure_enough_sensor_callbacks(whichSensor)) {
			d_callback_list.register_handler(userdata, handler);
			//return sensor_callbacks[whichSensor].d_handchange.register_handler(userdata, handler);
		} else {
			fprintf(stderr,"ARMMClient::register_change_handler: Out of memory\n");
			return -1;
		}
	}

	int ARMMClient::unregister_change_handler(void *userdata,
			vrpn_TRACKERHANDCHANGEHANDLER handler, vrpn_int32 whichSensor)
	{
	 // put it into the normal list.
	  if (whichSensor == vrpn_ALL_SENSORS) {
			d_callback_list.unregister_handler(userdata, handler);
		  //return all_sensor_callbacks.d_handchange.register_handler(userdata, handler);
		} else if (ensure_enough_sensor_callbacks(whichSensor)) {
			d_callback_list.unregister_handler(userdata, handler);
			//return sensor_callbacks[whichSensor].d_handchange.register_handler(userdata, handler);
		} else {
			fprintf(stderr,"ARMMClient::register_change_handler: Out of memory\n");
			return -1;
		}
	}

	//Atsushi
	int ARMMClient::register_types(void)
	{
		//vrpn_Tracker_Remote::register_types();

		// to handle hand state changes
		hand_m_id = d_connection->register_message_type("vrpn_Tracker Hand");
		return 0;
	}

	int VRPN_CALLBACK ARMMClient::handle_hand_change_message(
		void *userdata, vrpn_HANDLERPARAM p)
	{
		ARMMClient *me = (ARMMClient *)userdata;
		const char *params = (p.buffer);
		vrpn_TRACKERHANDCB	tp;

		// Fill in the parameters to the tracker from the message

		// this value depends on data size
		// you are supposed to send
		// In this case, you can count on the number of vrpn_float32 variablef
		int data_size = (UDP_LIMITATION*3)*sizeof(vrpn_float32) + sizeof(vrpn_int32);
		//int data_size = sizeof(vrpn_float32) + sizeof(vrpn_int32);

		if (p.payload_len !=  data_size) {
			fprintf(stderr,"ARMMClient: change message payload error\n");
			fprintf(stderr,"             (got %d, expected %lud)\n",
				p.payload_len, data_size );
			return -1;
		}

		tp.msg_time = p.msg_time;
		vrpn_unbuffer(&params, &tp.sensor);
		REP(i,UDP_LIMITATION){
			REP(j,3){
				vrpn_unbuffer(&params, &tp.hand[i][j]);
			}
		}
		//vrpn_unbuffer(&params, &tp.hand);

		// Go down the list of callbacks that have been registered.
		// Fill in the parameter and call each.
		//me->all_sensor_callbacks.d_handchange.call_handlers(tp);
		me->d_callback_list.call_handlers(tp);

	 // // Go down the list of callbacks that have been registered for this
		//// particular sensor
		//if (tp.sensor < 0) {
		//    fprintf(stderr,"ARMMClient:pos sensor index is negative!\n");
		//    return -1;
		//} else if (me->ensure_enough_sensor_callbacks(tp.sensor)) {
		//	me->sensor_callbacks[tp.sensor].d_handchange.call_handlers(tp);
		//} else {
		//    fprintf(stderr,"ARMMClient:pos sensor index too large\n");
		//    return -1;
		//}
		return 0;
	}
	///////// End of ARMM_Client definition ////////////////

	inline CvMat* scaleParams(CvMat *cParams, double scaleFactor) {
		CvMat *sParams = cvCloneMat(cParams);
		sParams->data.db[0]*= scaleFactor;	sParams->data.db[4]*= scaleFactor;
		sParams->data.db[2]*= scaleFactor;	sParams->data.db[5]*= scaleFactor;
		return sParams;
	}

	//-----> Callback function
	void VRPN_CALLBACK handle_pos (void * userData, const vrpn_TRACKERCB t)
	{
		if( t.sensor < CAR_PARAM){
			if(t.sensor % 5 == 0) {// pos and orientation of cars
				int j = (int) t.sensor / 5;
				CarsArrayPos[j] = osg::Vec3d((float) t.pos[0],(float) t.pos[1],(float) t.pos[2]);
				CarsArrayQuat[j] = osg::Quat((float) t.quat[0], (float) t.quat[1], (float) t.quat[2], (float) t.quat[3]);
			} else { //pos and orientation of car's wheels
				int j = (int) floor((float) t.sensor/5);
				int k = (t.sensor % 5) -1;
				WheelsArrayPos[j][k] = osg::Vec3d((float) t.pos[0],(float) t.pos[1],(float) t.pos[2]);
				WheelsArrayQuat[j][k]= osg::Quat((float) t.quat[0], (float) t.quat[1], (float) t.quat[2], (float) t.quat[3]);
			}
		}
		//----->Keyboard input checker
		else if( t.sensor == CAR_PARAM){
			if ( static_cast<int>(t.pos[0]) != 0){
				m_pass = static_cast<int>(t.pos[0]);
			}
		}
		//----->Receive objects info
		else{
			int index = t.sensor - (CAR_PARAM  + 1);
			ObjectsArrayPos[index].set(  osg::Vec3d((float)t.pos[0]*OSG_SCALE , (float)t.pos[1]*OSG_SCALE , (float)t.pos[2]*OSG_SCALE));
			osg::Quat quat = osg::Quat((double)t.quat[0]*OSG_SCALE, (double)t.quat[1]*OSG_SCALE, (double)t.quat[2]*OSG_SCALE, (double)t.quat[3]*OSG_SCALE);
			ObjectsArrayQuat[index].set(quat.x(), quat.y(), quat.z(), quat.w());
		}
	}
	void VRPN_CALLBACK handle_hands (void * userData, const vrpn_TRACKERHANDCB h)
	{
		REP(i,HAND_SIZE){
			assert(h.sensor <= UDP_LIMITATION);
			if(i < h.sensor){
				hand_coord_x[0][i] = h.hand[i][0];
				hand_coord_y[0][i] = h.hand[i][1];
				hand_coord_z[0][i] = h.hand[i][2];
			}else{
				hand_coord_x[0][i] = 0;
				hand_coord_y[0][i] = 0;
				hand_coord_z[0][i] = 100;
			}
		}
	}
	//<----- Callback function

	//////////////////// Entry point ////////////////////
//	ARMM::ARMM()
//	:
//	{
//
//	}

	ARMM::ARMM( ::Capture *camera)
#ifdef POINTGREYCAMERA
		: capture(static_cast<PointgreyCamera *>(camera))
#else
	   : capture = new CvCamera(0, CAPTURE_SIZE, CAMERA_PARAMS_FILENAME);

#endif
	{
		Init();
	}

	ARMM::~ARMM(){
		osg_uninit();
		cvReleaseMat(&RegistrationParams);

		delete capture;
		delete kc;
		delete armm_client;
	}

	void ARMM::Run()
	{
		IplImage *arImage = capture->getFrame();
		armm_client->mainloop();
		RenderScene(arImage, capture);
		cvReleaseImage(&arImage);
#ifdef USE_CLIENT_SENDER
		ARMM_client_sender->SetPasskey( kc->check_input() );
		if( ARMM_client_sender->GetPasskey()  != 0){
			ARMM_client_sender->mainloop();
		}
#else
		if( m_pass != 0){
			kc->set_input(m_pass);
			m_pass = 0;
		}
#endif
	}

	void ARMM::Run( ImageType imRGB)
	{
		IplImage *arImage = capture->getFrame();
		armm_client->mainloop();
		RenderScene(arImage, capture);
		cvReleaseImage(&arImage);
#ifdef USE_CLIENT_SENDER
		ARMM_client_sender->SetPasskey( kc->check_input() );
		if( ARMM_client_sender->GetPasskey()  != 0){
			ARMM_client_sender->mainloop();
		}
#else
		if( m_pass != 0){
			kc->set_input(m_pass);
			m_pass = 0;
		}
#endif
	}

	void ARMM::Init( void ){
		// initialize values of virtual objects
		for(int i =0; i < NUM_CARS; i++) {
			CarsArrayPos[i] = osg::Vec3d(0,0,0);
			CarsArrayQuat[i] = osg::Quat(0,0,0,1);
			for(int j =0; j < NUM_WHEELS; j++) {
				WheelsArrayPos[i][j] = osg::Vec3d(0,0,0);
				WheelsArrayQuat[i][j] = osg::Quat(0,0,0,1);
			}
		}

		RegistrationParams = scaleParams(capture->getParameters(), double(REGISTRATION_SIZE.width)/double(CAPTURE_SIZE.width));
		osg_init(calcProjection(RegistrationParams, capture->getDistortion(), REGISTRATION_SIZE));

		osg_inittracker(MARKER_FILENAME, 400, 400);
	//	m_Connection = new vrpn_Connection_IP();
	//	ARMM_Client = new vrpn_Tracker_Remote ("ARMM_Comm", m_Connection);

		//----->Client part
		armm_client = new ARMMClient (ARMM_SERVER_IP);
		armm_client->vrpn_Tracker_Remote::register_change_handler(NULL, handle_pos);
		armm_client->ARMMClient::register_change_handler(NULL, handle_hands);
		//<-----

		#ifdef USE_CLIENT_SENDER
		//----->Client_sender part
		m_Connection = new vrpn_Connection_IP();
		ARMM_client_sender = new ARMMClientSender(m_Connection);
	  cout << "Created VRPN server." << endl;
		//<-----
		#endif

		m_pass = 0;
	}

	void ARMM::RenderScene(IplImage *arImage, Capture *capture)
	{
	#ifdef SIM_MICROMACHINE
		if(Virtual_Objects_Count > 0) {
			DeleteLostObject();

			std::vector <osg::Quat> quat_obj_array;
			std::vector <osg::Vec3d> vect_obj_array;
			for(int i = 0; i < Virtual_Objects_Count; i++) {
				quat_obj_array.push_back(ObjectsArrayQuat[i]);
				vect_obj_array.push_back(ObjectsArrayPos[i]);
			}
			osg_UpdateHand(0, hand_coord_x[0], hand_coord_y[0], hand_coord_z[0]);
			osg_client_render(arImage, CarsArrayQuat, CarsArrayPos, WheelsArrayQuat, WheelsArrayPos, RegistrationParams, capture->getDistortion(), quat_obj_array, vect_obj_array);
		} else {
			osg_UpdateHand(0, hand_coord_x[0], hand_coord_y[0], hand_coord_z[0]);
			osg_client_render(arImage, CarsArrayQuat, CarsArrayPos, WheelsArrayQuat, WheelsArrayPos, RegistrationParams, capture->getDistortion());
		}
	#else
		osg_render(arImage, CarsArrayQuat, CarsArrayPos, WheelsArrayQuat, WheelsArrayPos, RegistrationParams, capture->getDistortion());
	#endif /*SIM_MICROMACHINE*/

	}

	void ARMM::DeleteLostObject(void)
	{
		obj_ind.clear();
		for(int i = 0; i < Virtual_Objects_Count; i++)
		{
			if( ObjectsArrayPos[i].z() < -100 ){
				obj_ind.push_back(i);
				//writeback coord
				if( Virtual_Objects_Count == 1);
				else{
					if( (i+1) == Virtual_Objects_Count );
					else{
						ObjectsArrayPos[i] = ObjectsArrayPos[i+1];
					}
				}
			}
		}
		if(!obj_ind.empty())
		{
			cout << "Lost Number = " << obj_ind.size() << "->";
			for(int i= obj_ind.size()-1; i>=0; i--){
				int index = obj_ind[i];
				cout << index << " ";
				vector<osg::PositionAttitudeTransform*>::iterator it = obj_transform_array.begin() + index;
				vector<osg::ref_ptr<osg::Node> >::iterator it2 = obj_node_array.begin() + index;
				shadowedScene->removeChild(obj_transform_array.at(index));
				obj_transform_array.erase(it);
				obj_node_array.erase(it2);
				Virtual_Objects_Count--;
				cout << "Virtual objects LOST : Remain " << Virtual_Objects_Count << endl;
			}
			obj_ind.clear();
		}
	}
}

