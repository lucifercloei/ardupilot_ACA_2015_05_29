
#include <stdlib.h>
#include <math.h>

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;
#include "ACA_AI.h"

//global timing functions
/*
void
ACA_AI::update_one_hz_count()
{
    one_hz_count++;
    if (one_hz_count > ONE_HOUR_RESET_1_HZ)
    {one_hz_count = 0;}
}

void
ACA_AI::update_ten_hz_count()
{
    ten_hz_count++;
    if (ten_hz_count > ONE_HOUR_RESET_10_HZ)
    {ten_hz_count = 0;}
}
void
ACA_AI::update_fast_count()
{
	fast_count++;
    if (fast_count > ONE_HOUR_RESET_FAST)
    {fast_count = 0;}
}
*/

void
ACA_AI::init()
{
	system_start = 1;
	reset_speed = 0;//1 reset 0, don't reseet
	halt_dir = 0;	//1 halt to slow, 0 halt to avoid
	avoid_done = 0; //1 done, 0 not done
	avoid_hor = 1;  //1 force horizontal, 0 ready to quit horizontal and enter forward
	fail = 0;


	mode_curr = ACA_Sleep;
	mode_prev = ACA_Sleep;

	ACA_time_elapse =0; //time elapse in avoid mode
	ACA_time_elapse2 =0;//time elapse in other modes

	rc1_input = 0; //left(negative) right (positive)
	rc2_input = 0; //back (negative) forward (positive)
	rc3_input = 500; //elevation
	rc4_input = 0; //yaw

	dir_curr = ACA_STAY;
	dir_prev = ACA_STAY;

}

void
ACA_AI::update_ten_hz(int milliseconds)
{
	ACA_time_curr = milliseconds;
	update_state();
	if(mode_curr == ACA_Avoid)
	{
		apply_avoid();
		ACA_time_stamp2 = ACA_time_curr; //keep the unrelated stamp to be updated
	}
	else
	{
		ACA_time_stamp = ACA_time_curr;	 //keep the unrelated stamp to be updated
	}
    update_rc();
}

void
ACA_AI::update_lidar(int sonar_distance_cm)
{
	lidar_curr = sonar_distance_cm;
	if (lidar_curr == lidar_prev){lidar_stag_ct++;}
	else{lidar_stag_ct = 0;}

	//Lidar return value keeps the same for 1 sec, considered stagnant
	if(lidar_stag_ct > ACA_LIDAR_STAG_CT)
	{
		if(lidar_curr >ACA_LIDAR_STAG_THR)
		{
			lidar_state = ACA_INFINITE;
			lidar_cm = ACA_LIDAR_STAG_INIFINITE;
		}
		else
		{
			lidar_state = ACA_BLOCKED;
			lidar_cm = ACA_LIDAR_STAG_BLOCKED;
		}
	}
	//Lidar return value not considered stagnant
	else
	{
		lidar_cm = lidar_curr;
		if(lidar_curr > ACA_LIDAR_ALARM_THR)
		{		lidar_state = ACA_CLEAR;	}
		else if(lidar_curr > ACA_LIDAR_AVOID_THR)
		{		lidar_state = ACA_ALARM;	}
		else
		{		lidar_state = ACA_CLOSE;	}
	}
	lidar_prev = lidar_curr;
}

void
ACA_AI::update_state()
{
	if(mode_prev == ACA_Sleep)
	{
		ACA_time_stamp2 = ACA_time_curr;  //always update stamp2 (sleep mode no timing)
		if(lidar_cm > ACA_LIDAR_ALARM_THR)//stay in Sleep mode if no object
		{
			mode_curr = ACA_Sleep;
			reset_speed = 0;
		}
		else if(lidar_cm > ACA_LIDAR_AVOID_THR)//switch to slow, reset speed
		{
			mode_curr = ACA_Slow;
			reset_speed = 0; ///// changed from 1 -> 0
		}
		else
		{
			mode_curr = ACA_Halt;
		}
	}

	if(mode_prev == ACA_Slow)
	{
		if(lidar_cm < ACA_LIDAR_AVOID_THR)//enter halt mode immediately
		{
			ACA_time_stamp2 = ACA_time_curr;
			mode_curr = ACA_Halt;
			reset_speed = 0;
		}
		else if(lidar_cm > ACA_LIDAR_ALARM_THR)
		{
			if(ACA_time_elapse2 > ACA_SLOW2SLEEP_MIN) //after staying in slow for MIN time, return sleep, reset speed
				{
					ACA_time_stamp2 = ACA_time_curr;
					mode_curr = ACA_Sleep;
					reset_speed = 0; // Changed from 1 -> 0
				}
											//stay in slow for at least MIN time
			else{
					//don't update stamp2
					mode_curr = ACA_Slow;
					reset_speed = 0;
				}
		}
		else								//stay in slow
		{
			//don't update stamp2
			mode_curr = ACA_Slow;
			reset_speed = 0;
		}
	}

	if(mode_prev == ACA_Halt)
	{
		if(lidar_cm < ACA_LIDAR_AVOID_THR)	//ready to go into avoid
		{
			if(halt_dir)
			{
				ACA_time_stamp2 = ACA_time_curr;
				ACA_time_elapse2 =0;
			}
			if(ACA_time_elapse2 > ACA_HALT2AVOID_MIN) //after staying in halt for MIN time, go into avoid
				{
					ACA_time_stamp2 = ACA_time_curr;
					mode_curr = ACA_Avoid;
					avoid_done = 0;
					reset_speed = 0;
				}
											//stay in avoid for at least MIN time
			else{
					//don't update stamp2
					mode_curr = ACA_Halt;
					reset_speed = 0;
				}
			halt_dir = 0;
		}
		else								//ready to go back to slow
		{
			if(halt_dir){}
			else
			{
				ACA_time_stamp2 = ACA_time_curr;
				ACA_time_elapse2 =0;
			}
			if(ACA_time_elapse2 > ACA_HALT2SLOW_MIN) //after staying in halt for MIN time, go back to slow
				{
					ACA_time_stamp2 = ACA_time_curr;
					mode_curr = ACA_Slow;
					reset_speed = 1;
				}
											//stay in avoid for at least MIN time
			else{
					//don't update stamp
					mode_curr = ACA_Halt;
					reset_speed = 0;
				}
			halt_dir =1 ;
		}


	}

	if(mode_prev == ACA_Avoid)
	{
		if(avoid_done)
		{
			mode_curr = ACA_Halt;
			avoid_done = 0;
		}
		else
		{
			mode_curr = ACA_Avoid;
		}
	}

	ACA_time_elapse2 = ACA_time_curr - ACA_time_stamp2;
	mode_prev = mode_curr;
}

void
ACA_AI::update_rc()
{
    if (mode_curr != ACA_Avoid)
    {
        dir_curr = ACA_STAY;
    }

    switch( dir_curr )
    {
    	case ACA_STAY:
    		rc1_input = 0;
			rc2_input = 0;
			rc3_input = 500;
			rc4_input = 0;
            break;
        case ACA_LEFT:
        	rc1_input = -4500;
			rc2_input = 0;
			rc3_input = 500;
			rc4_input = 0;
            break;
        case ACA_RIGHT:
        	rc1_input = 4500;
			rc2_input = 0;
			rc3_input = 500;
			rc4_input = 0;
            break;
        case ACA_UP:
        	rc1_input = 0;
			rc2_input = 0;
			rc3_input = 800;
			rc4_input = 0;
            break;
        case ACA_DOWN:
        	rc1_input = 0;
			rc2_input = 0;
			rc3_input = 300;
			rc4_input = 0;
            break;
        case ACA_FORWARD:
        	rc1_input = 0;
			rc2_input = -4500;
			rc3_input = 500;
			rc4_input = 0;
            break;
        case ACA_BACKWARD:
        	rc1_input = 0;
			rc2_input = 2500;
			rc3_input = 500;
			rc4_input = 0;
            break;
        default:
            rc1_input = 0;
            rc2_input = 0;
            rc3_input = 500;
            rc4_input = 0;
    }

    if (avoid_emerg == 1)
    {
    	rc2_input = 2500;
    }


    
}

void
ACA_AI::apply_avoid()
{
//update ACA_time_stamp whenever dir_curr change state

		//either we see an object or time in horizontal movement is shorter than min,
		//apply horizontal movement
		if( lidar_cm <= ACA_LIDAR_AVOID_THR || avoid_hor)
		{
			if ( dir_prev == ACA_STAY )
			{

				if( (ACA_time_elapse > ACA_AVOID_STAY_MIN) && (fail == 0) )
				{
					dir_curr = ACA_LEFT;
					ACA_time_stamp = ACA_time_curr;
				}
				else
				{
					dir_curr = ACA_STAY;
				}

			}

			else if ( dir_prev == ACA_LEFT )
			{

				if( ACA_time_elapse > ACA_AVOID_HOR_MAX )
				{
					dir_curr = ACA_RIGHT;
					ACA_time_stamp = ACA_time_curr;
				}
			}

			else if ( dir_prev == ACA_RIGHT )
			{

				if( ACA_time_elapse > ACA_AVOID_HOR_MAX*2 )
				{
					dir_curr = ACA_UP;
					ACA_time_stamp = ACA_time_curr;
				}
			}
			else if ( dir_prev == ACA_UP )
			{

				if( ACA_time_elapse > ACA_AVOID_UP_MAX )
				{
					fail = 1; //Set to STAY forever
					dir_curr = ACA_STAY;
					ACA_time_stamp = ACA_time_curr;
				}
			}

			else if (dir_prev == ACA_FORWARD )//hit another object when dir_prev is forward
			{
				avoid_hor = 1;
				dir_curr = ACA_STAY;
				ACA_time_stamp = ACA_time_curr;

			}
			else
			{
				dir_curr = ACA_STAY;
			}

			//check if we stay in horizontal movement for min time
			if (ACA_time_elapse >ACA_AVOID_HOR_MIN)
			{
				avoid_hor = 0;
			}
			else
			{
				avoid_hor = 1;
			}
		}
		//either we don't see an object and time in horizontal movement is longer than min,
		//apply forward movement
		else
		{
			if(dir_prev == ACA_FORWARD)
			{

				if( ACA_time_elapse > ACA_AVOID_FOR_MIN )
				{
					dir_curr = ACA_STAY; //reset direction to stay for next apply_avoid
					avoid_done = 1;
				}

			}
			else
			{
				dir_curr = ACA_FORWARD;
				ACA_time_stamp = ACA_time_curr;
			}
		}


	ACA_time_elapse = ACA_time_curr - ACA_time_stamp;
	if( lidar_cm < ACA_LIDAR_EMERG_THR )
		{
			avoid_emerg = 1;
		}
	else
		{
			avoid_emerg = 0;
		}

	dir_prev = dir_curr;


}

