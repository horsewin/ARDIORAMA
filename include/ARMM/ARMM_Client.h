#ifndef ARMM_CLIENT_H
#define ARMM_CLIENT_H

#define POINTGREYCAMERA 1

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "constant.h"

#include <stdio.h>
//#include <tchar.h>

#include <math.h>

#include "vrpn_Tracker.h"
#include "vrpn_Connection.h"
#include "CaptureLibrary.h"

#include "ImageType.h"
//---------------------------------------------------------------------------
// Constant/Define
//---------------------------------------------------------------------------
//#define USE_CLIENT_SENDER


#define NUM_CARS 2
#define NUM_WHEELS 4

const int NUM_OBJECTS = 10;
const int CAR_PARAM = NUM_CARS*NUM_WHEELS+NUM_CARS;
extern bool running;

//---------------------------------------------------------------------------
// Struct
//---------------------------------------------------------------------------
typedef	struct _vrpn_TRACKERHANDCB {
	struct timeval	msg_time;	// Time of the report
	vrpn_int32	sensor;		// Which sensor is reporting
	vrpn_float32 hand[UDP_LIMITATION][3];
	//vrpn_float32 hand;
} vrpn_TRACKERHANDCB;
typedef void (VRPN_CALLBACK *vrpn_TRACKERHANDCHANGEHANDLER)(void *userdata,
					     const vrpn_TRACKERHANDCB info);

class KeyboardController_client;

namespace ARMM{
	//---------------------------------------------------------------------------
	// Class definition
	//---------------------------------------------------------------------------
	class PointgreyCamera;
	class ARMMClient : public vrpn_Tracker_Remote
	{
	public:
		ARMMClient(const char * name, vrpn_Connection *cn = NULL);
		virtual ~ARMMClient(void);
		virtual void mainloop();

		virtual int register_change_handler(void *userdata,
			  vrpn_TRACKERHANDCHANGEHANDLER handler, vrpn_int32 sensor = vrpn_ALL_SENSORS);

		virtual int unregister_change_handler(void *userdata,
			  vrpn_TRACKERHANDCHANGEHANDLER handler, vrpn_int32 sensor = vrpn_ALL_SENSORS);

	protected:
		vrpn_Callback_List<vrpn_TRACKERHANDCB> d_callback_list;
	  //ARMMSensorCallbacks   all_sensor_callbacks;
	  //ARMMSensorCallbacks   *sensor_callbacks;

		//Atsushi
		vrpn_int32 hand_m_id;	// ID of tracker hand message

	protected:
		virtual int register_types(void);
		static int VRPN_CALLBACK handle_hand_change_message(void *userdata,
			vrpn_HANDLERPARAM p);

	};

	class ARMM{
	public:
		ARMM();
		ARMM(::Capture * camera);
		~ARMM();
		void Run();
		void Run( ImageType imRGB );

	private:
		void Init( void );
		void RenderScene(IplImage *arImage, ::Capture *capture);
		void RenderScene(IplImage *arImage, CvMat * distortion);
		void DeleteLostObject( void );

	private:
		ARMMClient * armm_client;
		::KeyboardController_client *kc;
		#ifdef POINTGREYCAMERA
		PointgreyCamera *capture;
		#else
		Capture *capture;
		#endif

	};

	#ifdef USE_CLIENT_SENDER
	/////////////////////// ARMM_COMMUNICATOR /////////////////////////////

	class ARMMClientSender : public vrpn_Tracker
	{
	public:
		ARMMClientSender( vrpn_Connection_IP *c = 0);
		virtual ~ARMMClientSender();
		virtual void mainloop();
		void SetPasskey(const int & pass);
		int  GetPasskey( void )const;
	protected:
		struct timeval _timestamp;
		int m_pass;
	};
	#endif // ** USE_CLIENT_SENDER **
}
#endif
