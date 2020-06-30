/***************************************************************************

file        : racestate.cpp
created     : Sat Nov 16 12:00:42 CET 2002
copyright   : (C) 2002 by Eric Espi�
email       : eric.espie@torcs.org
version     : $Id: racestate.cpp,v 1.5 2005/08/17 20:48:39 berniw Exp $

***************************************************************************/

/***************************************************************************
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
***************************************************************************/

/** @file

@author	<a href=mailto:eric.espie@torcs.org>Eric Espie</a>
@version	$Id: racestate.cpp,v 1.5 2005/08/17 20:48:39 berniw Exp $
*/

/* The Race Engine State Automaton */

#include <stdlib.h>
#include <stdio.h>
#include <tgfclient.h>
#include <raceman.h>
#include <racescreens.h>
#include <cmath>

#include "racemain.h"
#include "raceinit.h"
#include "raceengine.h"
#include "racegl.h"
#include "raceresults.h"
#include "racemanmenu.h"

#include "racestate.h"

//dosssman
// Using time for dump file name creation
// Also folder creation dependencies
#include "time.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <tgf.h>
#include <string.h>
#include "sensors.h"
#include "SimpleParser.h"
#include "CarControl.h"
#include "ObstacleSensors.h"

// Dirty trick: Do all the race data recording here T_T
string obs, rews, acs;
// end dosssman

static void *mainMenu;

// dosssman
// Counting eisode to name the dumped json play data
static int ep_counter = 0;
// Static var for dump file name
static char filepath[1024];
char save_dir[196];

// Filename for respective data files
char obs_file[256] = {};
char acs_file[256] = {};
char rews_file[256] = {};

// Sensors holder variables ( no more init at each steps)
tSituation *s = NULL;

// tCarElt **cars = s->cars;
tCarElt *car = NULL; // Takes the first car, ala index == 0
tTrack *curTrack = NULL;

// Setup track angles !!! Doesn't reallty change in one episode
const float trackSensAngle[19] = { -45.0, -19, -12, -7, -4, -2.5, -1.7, -1, -.5, 0,
	.5, 1, 1.7, 2.5, 4, 7, 12, 19, 45};

// Dropped the table strcuture as we only focus on the first car
// TODO: Delete if useless
// Sensors *focusSens = NULL;//ML
Sensors *trackSens = NULL;
ObstacleSensors *oppSens = NULL;

// Stats
// int rs_timestep = 0;
// clock_t begin_clock;

float dist_to_middle, angle;
// This is an array because we may want to record multiple cars, but let's be honest ...
// Seems unused
// tdble prevDist, distRaced;

// Temp sensor data holder before serial and dump
float trackSensorOut[19], focusSensorOut[5], oppSensorOut[36], wheelSpinVel[4];

// File objects for respective datafiles
FILE *obs_f = NULL;
FILE *acs_f = NULL;
FILE *rews_f = NULL;
// end dosssman

/* State Automaton Init */
void ReStateInit(void *prevMenu) {
	mainMenu = prevMenu;
}

/* State Automaton Management         */
/* Called when a race menu is entered */
void ReStateManage(void) {
	int mode = RM_SYNC | RM_NEXT_STEP;

	do {
		switch (ReInfo->_reState) {
			case RE_STATE_CONFIG:
				//				printf("RE_STATE_CONFIG\n");
				if (getTextOnly()==false)
				{
					GfOut("RaceEngine: state = RE_STATE_CONFIG\n");
					/* Display the race specific menu */
					mode = ReRacemanMenu();
					if (mode & RM_NEXT_STEP) {
						ReInfo->_reState = RE_STATE_EVENT_INIT;
					}
				}
				// GIUSE - VISION HERE!!
				else if (getVision())
				{
					//					GfOut("RaceEngine: state = RE_STATE_CONFIG\n");
					/* Display the race specific menu */
					//					mode =
					ReRacemanMenu();
					//					if (mode & RM_NEXT_STEP) {
					ReInfo->_reState = RE_STATE_EVENT_INIT;
					//					}
				}
				else
				{
					ReInfo->_reState = RE_STATE_EVENT_INIT;
					ReRacemanMenu();
				}
				break;

			case RE_STATE_EVENT_INIT:
				//				printf("RE_STATE_EVENT_INIT\n");
				if (getTextOnly()==false)
					GfOut("RaceEngine: state = RE_STATE_EVENT_INIT\n");
				/* Load the event description (track and drivers list) */
				mode = ReRaceEventInit();
				if (mode & RM_NEXT_STEP) {
					ReInfo->_reState = RE_STATE_PRE_RACE;
				}

				if( getRecordHuman()) {
					printf( "#### DEBUG: Initializing save paths\n");
					init_save_paths();
				}
				break;

			case RE_STATE_PRE_RACE:
				//				printf("RE_STATE_PRE_RACE\n");
				if (getTextOnly()==false)
					GfOut("RaceEngine: state = RE_STATE_PRE_RACE\n");

				if( getRecordHuman()) {
						if( getRecEpisodeLimit() > 0)
							printf( "### DEBUG: [Recording] Current episode: %d / %d\n", ep_counter,
						 		getRecEpisodeLimit());
						else

							printf( "### DEBUG: [Recording] Current episode: %d\n", ep_counter);
					// open_save_files();
				}

				mode = RePreRace();
				if (mode & RM_NEXT_STEP) {
					ReInfo->_reState = RE_STATE_RACE_START;
				}
				break;

			case RE_STATE_RACE_START:
				//				printf("RE_STATE_RACE_START\n");
				if (getTextOnly()==false)
					GfOut("RaceEngine: state = RE_STATE_RACE_START\n");

				mode = ReRaceStart();

				if( getRecordHuman()) {
					// Used to compute sampling rate
					// rs_timestep = 0;
					// begin_clock = clock();
					// XXX Very important for data recording
					// Get the pointer for the first car before records starts
					// The cars must hence be loaded
					// TODO: If cars already loaded, declare the sensors variables here
					s = ReInfo->s;
					curTrack = ReInfo->track;
					// printf( "#### Collected track\n");

					car = *(s->cars+getRecCarIndex());

					// TODO: Delete if useless
					// focusSens = new Sensors(car, 5);//ML
					// printf( "#### Instanciate focusSens\n");

					trackSens = new Sensors(car, 19);
					// printf( "#### Instanciate trackSens\n");
					// Set default angles for track sensors
					for (int i = 0; i < 19; ++i) {
						trackSens->setSensor(i,trackSensAngle[i], 200);
					}

					oppSens = new ObstacleSensors(36, curTrack, car, s, 200.);
				}
				// end special hook for recording
				if (mode & RM_NEXT_STEP) {
					ReInfo->_reState = RE_STATE_RACE;
				}
				break;

			case RE_STATE_RACE:
				//				printf("RE_STATE_RACE\n");
				mode = ReUpdate();

				// dosssman
				if( getRecordHuman()) {
					// rs_timestep++;
					append_step_data();
				}
				// end dosssman

				// GfOut( mode);
				if (ReInfo->s->_raceState == RM_RACE_ENDED) {
					/* race finished */
					ReInfo->_reState = RE_STATE_RACE_END;
				} else if (mode & RM_END_RACE) {
					/* interrupt by player */
					ReInfo->_reState = RE_STATE_RACE_STOP;
				}
				break;

			case RE_STATE_RACE_STOP:
				//				printf("RE_STATE_RACE_STOP\n");
				if (getTextOnly()==false)
				GfOut("RaceEngine: state = RE_STATE_RACE_STOP\n");
				/* Interrupted by player */
				// dosssman
				// TODO: Dump player data

				if( getRecordHuman()) {
					// dump_play_data();
					// Insert gotoline in the save file
					append_episode_data();
					// close_save_files();
				}

				mode = ReRaceStop();
				if (mode & RM_NEXT_STEP) {
					if (RESTART==1)
					{
						RESTART=0;
						ReRaceCleanup();
						ReInfo->_reState = RE_STATE_PRE_RACE;
						if (getTextOnly()==false)
							GfuiScreenActivate(ReInfo->_reGameScreen);
						// GIUSE - VISION HERE!!
						else if (getVision())
							GfuiScreenActivate(ReInfo->_reGameScreen);
					}
					else
					{
						ReInfo->_reState = RE_STATE_RACE_END;
					}
				}
				break;

			case RE_STATE_RACE_END:
				//				printf("RE_STATE_RACE_END\n");
				if (getTextOnly()==false)
					GfOut("RaceEngine: state = RE_STATE_RACE_END\n");
				// mode = ReRaceEnd();
				// dosssman
				// dumping data
				if( getRecordHuman()) {
					// dump_play_data();
					// Insert gotoline in the save file
					printf( "##### DEBUG: Reached data appending");
					append_episode_data();
					// close_save_files();
				}

				ep_counter++;

				if( getRecordHuman() && ( getRecEpisodeLimit() > 0 && ep_counter >= getRecEpisodeLimit())) {
					printf( "### DEBUG: Episode limit reached, shutting down the game !\n" );
					ReInfo->s->_raceState = RM_RACE_ENDED;
					ReInfo->_reState=RE_STATE_EXIT;
					// mode = ReRaceEnd();
					break;
				}

				ReRaceCleanup();
				mode = ReRaceEventInit();
				ReInfo->_reState = RE_STATE_PRE_RACE;

				if(mode == RE_STATE_EXIT)
				{
					// dosssman
					// TODO: Dump player data
					if( getRecordHuman()) {
						printf( "### DEBUG: Episode ended from outside Torcs !\n" );

					}

					// Original
					ReInfo->_reState=RE_STATE_EXIT;
					// End Original
				}
				else
				{
					if (mode & RM_NEXT_STEP) {
						// Dump play data use ReInfo params rquired probably
						// dosssman
						// TODO: Dump player data
						// ReRaceCleanup();
						// end dosssman
						ReInfo->_reState = RE_STATE_PRE_RACE;
						// ReInfo->_reState = RE_STATE_POST_RACE;
					} else if (mode & RM_NEXT_RACE) {
						ReInfo->_reState = RE_STATE_RACE_START;
					}
				}
				break;

			case RE_STATE_POST_RACE:
				//				printf("RE_STATE_POST_RACE\n");
				if (getTextOnly()==false)
					GfOut("RaceEngine: state = RE_STATE_POST_RACE\n");

				mode = RePostRace();
				// ReRaceCleanup();
				// mode = ReRaceEventInit();
				if (mode & RM_NEXT_STEP) {
					ReInfo->_reState = RE_STATE_EVENT_SHUTDOWN;
				} else if (mode & RM_NEXT_RACE) {
					ReInfo->_reState = RE_STATE_PRE_RACE;
				}
				break;

			case RE_STATE_EVENT_SHUTDOWN:
				//				printf("RE_STATE_EVENT_SHUTDOWN\n");
				if (getTextOnly()==false)
				GfOut("RaceEngine: state = RE_STATE_EVENT_SHUTDOWN\n");
				mode = ReEventShutdown();
				if (mode & RM_NEXT_STEP) {
					ReInfo->_reState = RE_STATE_SHUTDOWN;
				} else if (mode & RM_NEXT_RACE) {
					ReInfo->_reState = RE_STATE_EVENT_INIT;
				}
				break;

			case RE_STATE_SHUTDOWN:
				//				printf("RE_STATE_SHUTDOWN\n");
				case RE_STATE_ERROR:
				printf("RE_STATE_ERROR\n");
				if (getTextOnly()==false)
				GfOut("RaceEngine: state = RE_STATE_SHUTDOWN\n");
				/* Back to race menu */
				ReInfo->_reState = RE_STATE_CONFIG;
				mode = RM_SYNC;
				break;

			case RE_STATE_EXIT:
				//				printf("RE_STATE_EXIT\n");
				if (getTextOnly()==false)
					GfScrShutdown();
				// GIUSE - VISION HERE!!
				else if (getVision())
					GfScrShutdown();
				exit (0);		/* brutal isn't it ? */
				break;
		}


	} while ( ((mode & (RM_SYNC | RM_QUIT)) == RM_SYNC) || getTextOnly()) ;


	if (mode & RM_QUIT) {
		GfScrShutdown();
		exit (0);		/* brutal isn't it ? */
	}

	if (mode & RM_ACTIVGAMESCR) {
		GfuiScreenActivate(ReInfo->_reGameScreen);
	}
}

/* Change and Execute a New State  */
void ReStateApply(void *vstate) {

	long state = (long)vstate;

	ReInfo->_reState = (int)state;
	ReStateManage();
}

//
void append_step_data() {
	// Appending step data

	// Userless for now
	// for (int i = 0; i < 5; ++i) {
	// 	focusSens->setSensor(i,(car->_focusCmd)+i-2,200);
	// }

	// computing distance to middle
	dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);

	// computing the car angle wrt the track axis
	angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	NORM_PI_PI( angle); // normalize the angle between -PI and + PI

	if (dist_to_middle<=1.0 && dist_to_middle >=-1.0 ) {
		trackSens->sensors_update();

		for (int i = 0; i < 19; ++i)
		{
			// Normalize for torcs
			trackSensorOut[i] = ( float) trackSens->getSensorOut(i) / 200.0;
			if (getNoisy())
				trackSensorOut[i] *= normRand(1,.1);
		}

		// TODO Do we really need this now ?
		// focusSens->sensors_update();//ML
		// if ((car->_focusCD <= car->_curLapTime + car->_curTime)//ML Only send focus sensor reading if cooldown is over
		// && (car->_focusCmd != 360))//ML Only send focus reading if requested by client
		// {//ML
		// 	for (int i = 0; i < 5; ++i)
		// 	{
		// 		focusSensorOut[i] = focusSens->getSensorOut(i);
		// 		if (getNoisy())
		// 		focusSensorOut[i] *= normRand(1, .01);
		// 	}
		// 	car->_focusCD = car->_curLapTime + car->_curTime + 1.0;//ML Add cooldown [seconds]
		// }//ML
		// else//ML
		// {//ML
		// 	for (int i = 0; i < 5; ++i)//ML
		// 		focusSensorOut[i] = -1;//ML During cooldown send invalid focus reading
		// }//ML
	}
	else {
		// If the cars is out of the track
		for (int i = 0; i < 19; ++i)
		{
			trackSensorOut[i] = -1;
		}
		for (int i = 0; i < 5; ++i)
		{
			focusSensorOut[i] = -1;
		}
	}

	oppSens->sensors_update(s);
	for (int i = 0; i < 36; ++i) {
		// XXX: Normalizing it like it is done in Gym TorcsEnv
		oppSensorOut[i] = oppSens->getObstacleSensorOut(i) / 200.0;
		if (getNoisy())
			oppSensorOut[i] *= normRand(1, .02);
	}

	for (int i=0; i<4; ++i) {
		// XXX: Normalizing it like it is done in Gym TorcsEnv
		wheelSpinVel[i] = ( float) car->_wheelSpinVel(i) / 100.0;
	}

	// Currently unuseeed observations
	// if (prevDist[0]<0) {
	// 	prevDist[0] = car->race.distFromStartLine;
	// }
	// float curDistRaced = car->race.distFromStartLine - prevDist[0];
	// prevDist[0] = car->race.distFromStartLine;
	// if (curDistRaced>100) {
	// 	curDistRaced -= curTrack->length;
	// }
	// if (curDistRaced<-100) {
	// 	curDistRaced += curTrack->length;
	// }
	// distRaced[0] += curDistRaced;
	// float totdist = curTrack->length * (car->race.laps -1) + car->race.distFromStartLine;
	// Player data to Json object in car-> play_data

	// CSV style logging with corresponding Torcs normalizing

	obs += SimpleParser::stringifym( angle / 3.1416); //angle, Reging
	obs += SimpleParser::stringifym( trackSensorOut, 19); // track, Reg in colect
	obs += SimpleParser::stringifym( dist_to_middle / 1.0); // trackPos, Reging
	obs += SimpleParser::stringifym( 3.6 * car->_speed_x / 300.0); // Reging
	obs += SimpleParser::stringifym( 3.6 * car->_speed_y / 300.0); // Reging
	obs += SimpleParser::stringifym( 3.6 * car->_speed_z / 300.0); // Reging
	obs += SimpleParser::stringifym( wheelSpinVel, 4); // wheelspinvel
	obs += SimpleParser::stringifym( car->_enginerpm / 10000.); // Funny reg to match GymTorcs
	obs += SimpleParser::stringifym( oppSensorOut, 36); // opp sensoirs

	// TODO Free Pointers of observations vars
	// They will be reinited at the new race start
	// delete( trackSens[0]);
	// delete( oppSens[0]);
	// delete( focusSens[0]);

	acs += SimpleParser::stringifym( float( car->ctrl.steer));
	short int accel_mult = 1;
	if ( car->priv.gear == -1)
		accel_mult = -1;


	acs += SimpleParser::stringifym(
		float( accel_mult * car->ctrl.accelCmd));
	// acs += SimpleParser::stringifym(
	// 	float(car->ctrl.brakeCmd));
	// acs += SimpleParser::stringifym( accel);

	// printf( "# DEBUG: Steer: %.f - Accel: %.f - Brake: %.f - Gear %d\n",
	//  	float( car->ctrl.steer), accel_mult * car->ctrl.accelCmd,
	// 	car->ctrl.brakeCmd, car->priv.gear);

	// XXX: Compute the rwrd in case of dist only, must match reward in gym torcs
	// Unnormalized angle to compute reard too -_-'
	// rews += SimpleParser::stringifym( 3.6 * car->_speed_x * cos( angle));
	rews += SimpleParser::stringifym( 3.6 * car->_speed_x * cos( angle)
                                    - fabs(3.6 * car->_speed_x * sin( angle))
                                    - 3.6 * car->_speed_x * fabs(dist_to_middle) * 0.3);
	// printf( "Rew: %.3f\n",
	//  	(3.6 * car->_speed_x * cos( angle)));

	// printf( "### DEBUG: Track sensor data output\n");
	// for( ushort i = 0; i < 19; i++) {
	// 	printf( "\tIndex %d: %0.32f\n", i, trackSensorOut[i]);
	// }
	// printf( "### DEBUG: WheelSpin sensor data output\n");
	// for( ushort i = 0; i < 4; i++) {
	// 	printf( "\tIndex %d: %0.32f\n", i, wheelSpinVel[i]);
	// }
	// open_save_files();
	// printf( "### DEBUG: Reached file writing\n");
	// printf( "### DEBUG: Obs bfore writing: %s\n", obs.c_str());
	// fprintf( obs_f, obs.c_str());
	// fprintf( acs_f, acs.c_str());
	// fprintf( rews_f, rews.c_str());
	// Checking if string filetype is variable
	// obs = ""; acs = ""; rews = "";
	// close_save_files();
}

void append_episode_data() {
	// Appends episode data to the CSV file then clears the variables
	// Basically goto newline and the data appending at the timestep level does
	// rest
	// DEBUG Show expert score
	float ret_cum = 0.0;
	for( ushort i = 0; i <= getRecTimestepLimit(); i+=12) {
		ret_cum += rews[i];
	}

	printf("\n#### DEBUG: Expert Score: %f\n", ret_cum);

	// printf( "### DEBUG: Reached file writing episode end\n");
	open_save_files();
	fprintf( obs_f, obs.c_str());
	fprintf( acs_f, acs.c_str());
	fprintf( rews_f, rews.c_str());
	fprintf( obs_f, "\n");
	fprintf( acs_f, "\n");
	fprintf( rews_f,"\n");
	obs = ""; acs = ""; rews = "";
	close_save_files();
	// This happens at the end so safe enough for now
	delete( trackSens);
	delete( oppSens);
	// delete( focusSens);
	// Sampling rate ( Game speed)
	// if( getRecordHuman()) {
	// 	printf( "#### DEBUG: Timestep %f\n", (float) rs_timestep);
	// 	printf( "#### DEBUG: Timespent %f\n", (float) _);
		// printf( "#### DEBUG: Sampling Rate( Game speed) = %f\n", rs_timestep / time_spent);
	// }
}

void init_save_paths() {
	struct stat st = {0};

	strncpy( save_dir, getenv( "TORCS_DATA_DIR"), 196);
	strncpy( save_dir, "\0", 1);

	// printf( "## DEBUG: ENvpath: %s\n", save_dir);

	if( save_dir[0] == '\0') {
		if (stat( save_dir, &st) == -1) {
			mkdir( filepath, 0777);
		}
	}
	else {
		strncpy( save_dir, "/tmp/torcs_play_data", 196);
		if (stat( save_dir, &st) == -1) {
			mkdir( filepath, 0777);
		}
	}

	strncat( strncat( obs_file, getenv( "TORCS_DATA_DIR"), sizeof( obs_file)), "/obs.csv", 8);
	strncat( strncat( acs_file, getenv( "TORCS_DATA_DIR"), sizeof( acs_file)), "/acs.csv", 8);
	strncat( strncat( rews_file, getenv( "TORCS_DATA_DIR"), sizeof( rews_file)), "/rews.csv", 9);

	GfOut( "##### DEBUG: Save path inited");
}

void open_save_files() {

	// printf( "### DEBUG: Opening file: %s\n", obs_file);
	obs_f = fopen( obs_file, "a");
	if( obs_f == NULL)
		printf( "### DEBUG: File opening failed");

	acs_f = fopen( acs_file, "a");

	rews_f = fopen( rews_file, "a");
}

void close_save_files() {
	if( obs_f != NULL) {
		fclose( obs_f);
		obs_f = NULL;
	}

	if( acs_f != NULL) {
		fclose( acs_f);
		acs_f = NULL;
	}

	if( rews_f != NULL) {
		fclose( rews_f);
		rews_f = NULL;
	}
}
// end dosssman

double normRand(double avg,double std) {
	double x1, x2, w, y1, y2;

	do {
		x1 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
		x2 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
		w = x1 * x1 + x2 * x2;
	} while ( w >= 1.0 );

	w = sqrt( (-2.0 * log( w ) ) / w );
	y1 = x1 * w;
	y2 = x2 * w;
	return y1*std + avg;
}


// dosssman
// Dumping firsts car play_data
// May be upgraded to all cars
// TODO: Delete if useless
// void dump_play_data() {
//
// 	// Get car and play data
// 	tSituation *s = ReInfo->s;
// 	tCarElt *car = *(s->cars); // Takes the first car, mind you
// 	JsonNode *play_data = car->play_data;
//
// 	struct stat st = {0};
// 	// Generate respective folder name
// 	sprintf( filepath, "/tmp/torcs_play_data");
//
// 	// Creating root folder for torcs_play_data
// 	if (stat( filepath, &st) == -1) {
//     mkdir( filepath, 0777);
// 	}
//
// 	//Creating folder for currentr game session
// 	sprintf( filepath, "%s/%s", filepath, getRecSessionStartStr());
//
// 	if (stat( filepath, &st) == -1) {
//     mkdir( filepath, 0777);
// 	}
//
// 	// Generating filename for play_data;
// 	sprintf( filepath, "%s/%d.json", filepath, ep_counter);
//
// 	printf( "### DEBUG: Creating file\n");
// 	printf( "### DEBUG: %s\n", filepath);
//
// 	FILE *f = fopen( filepath, "w");
//
// 	if( f == NULL)
// 		printf("### DEBUG: Failed to created file\n");
//
// 	fprintf( f, json_encode( play_data));
//
// 	printf( "### DEBUG: Dumped JSON play_data: %s\n\n", filepath);
//
// 	// Attempt to mitigate the memory leak
// 	json_delete( play_data);
//
// 	fclose(f);
// }
