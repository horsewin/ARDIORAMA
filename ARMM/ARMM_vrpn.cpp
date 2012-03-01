#ifndef ARMM_VRPN_CPP
#define ARMM_VRPN_CPP

#include "ARMM/ARMM_vrpn.h"
#include <cstring>

namespace ARMM{
	ARMM_Communicator::ARMM_Communicator( vrpn_Connection *c) : vrpn_Tracker( "ARMM_Comm", c )
	{
		HeightfieldData = new float[19200];
		CarsArrayPos = new osg::Vec3d[NUMBER_CAR];
		CarsArrayQuat = new osg::Quat[NUMBER_CAR];
		WheelsArrayPos = new osg::Vec3d*[NUMBER_CAR];
		WheelsArrayQuat = new osg::Quat*[NUMBER_CAR];

		for( int i = 0 ; i < NUMBER_CAR ; i++ ) {
			WheelsArrayPos[i] = new osg::Vec3d[NUM_WHEEL];
			WheelsArrayQuat[i] = new osg::Quat[NUM_WHEEL];
		}
	}

	ARMM_Communicator::~ARMM_Communicator() {

		for( int i = 0 ; i < NUMBER_CAR ; i++ ) {
			delete [] WheelsArrayPos[i];
			delete [] WheelsArrayQuat[i];
		}
		delete [] WheelsArrayPos;
		delete [] WheelsArrayQuat;
		delete [] CarsArrayPos;
		delete [] CarsArrayQuat;
		delete [] HeightfieldData;
	}

	void ARMM_Communicator::VRPN_Update_Client_Cars(osg::Quat *q,osg::Vec3d  *v, osg::Quat wq[][4], osg::Vec3d wv[][4], float *ground_grid) {
		memcpy(CarsArrayPos, v, sizeof(CarsArrayPos));
		memcpy(CarsArrayQuat, q, sizeof(CarsArrayQuat));
		memcpy(WheelsArrayPos, wv, sizeof(WheelsArrayPos));
		memcpy(WheelsArrayQuat, wq, sizeof(WheelsArrayQuat));
		memcpy(HeightfieldData, ground_grid, sizeof(HeightfieldData));
	}

	void ARMM_Communicator::mainloop() {
		vrpn_gettimeofday(&_timestamp, NULL);

		vrpn_Tracker::timestamp = _timestamp;

		char msgbuf[1000];

		d_sensor = 0;

		int  len = vrpn_Tracker::encode_to(msgbuf);

		if (d_connection->pack_message(len, _timestamp, position_m_id, d_sender_id, msgbuf,
			vrpn_CONNECTION_LOW_LATENCY))
		{
			fprintf(stderr,"can't write message: tossing\n");
		}



		server_mainloop();
	}

	/*

	////////////// MAIN ///////////////////

	int _tmain(int argc, _TCHAR* argv[])
	{
		// Creating the network server
		vrpn_Connection_IP* m_Connection = new vrpn_Connection_IP();

		// Creating the tracker
		ARMM_Communicator* ARMM_server = new ARMM_Communicator(m_Connection );

		cout << "Created VRPN server." << endl;

		while(true)
		{
			serverTracker->mainloop();

			m_Connection->mainloop();

			// Calling Sleep to let the CPU breathe.
			SleepEx(1,FALSE);
		}
	}

	*/

}
#endif

