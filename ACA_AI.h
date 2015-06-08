// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
//reset for global timing, all reset at 1 hour
/*
#define ONE_HOUR_RESET_1_HZ 3600
#define ONE_HOUR_RESET_10_HZ 36000
#define ONE_HOUR_RESET_FAST 360000
*/
#define ACA_LIDAR_STAG_THR 100  //lidar stagnates within THR,
								//considered as blocked,
								//else considered infinite
#define ACA_LIDAR_ALARM_THR 750 //threshold distance for alarm, reduce speed if inside the range
#define ACA_LIDAR_AVOID_THR 350 //threshold distance for avoid, apply avoidance if inside the range
#define ACA_LIDAR_EMERG_THR 100 //threshold distance for emergency, go backwards if inside the range


#define ACA_WP_SPEED_SLEEP 250  //wp speed in cms for sleep mode
#define ACA_WP_SPEED_SLOW  150	//wp speed in cms for slow mode

#define ACA_AVOID_STAY_MIN 1000	//min time in ms to keep direction stay in avoid mode
#define ACA_AVOID_HOR_MAX  5000 //max time in ms to go horizontally in avoid mode
#define ACA_AVOID_HOR_MIN  3000 //min time in ms to go horizontally in avoid mode

#define ACA_AVOID_FOR_MIN  3000 //min time in ms to go forward in avoid mode

#define ACA_AVOID_UP_MAX  5000 //max time in ms to go up in avoid mode

#define ACA_SLOW2SLEEP_MIN 3000	//minimum time (in unit of 100ms, excuted in ten hz loop)
								//to stay in slow mode before go back to sleep mode
#define ACA_HALT2AVOID_MIN 1000	//minimum time (in unit of 100ms, excuted in ten hz loop)
								//to stay in halt mode before go into avoid mode
#define ACA_HALT2SLOW_MIN 4000	//minimum time (in unit of 100ms, excuted in ten hz loop)
								//to stay in halt mode before go back to slow mode

#define ACA_LIDAR_STAG_CT 10	//number of instances lidar seeing a constant value
								//that will be considered stagnant
#define ACA_LIDAR_STAG_BLOCKED 1		//return of lidar when in BLOCKED state
#define ACA_LIDAR_STAG_INIFINITE 9999	//return of lidar when in INIFINITE state



#define FPGA_LIDAR_THR_1  150
#define FPGA_LIDAR_THR_2  350
#define FPGA_LIDAR_THR_3  500
/// @class	ACA_AI
/// @brief	Object managing Artificial Intelligence regarding Automatic Collision Avoidance
class ACA_AI
{
public:
	//global timing variables
	/*
    uint16_t		one_hz_count=0;
    uint16_t        ten_hz_count=0;
    uint32_t		fast_count=0;
	*/
    //lidar variables
    uint16_t		lidar_cm=0;
    uint16_t		lidar_prev=0;
    uint16_t		lidar_curr=0;
    uint8_t			lidar_stag_ct=0;
    enum ACALIDAR {
            ACA_CLEAR,
            ACA_ALARM,
            ACA_CLOSE,
            ACA_BLOCKED,
            ACA_INFINITE
        };
    ACALIDAR 		lidar_state = ACA_CLEAR;

    //general control variables
    enum ACAMODE {
			ACA_Sleep,
			ACA_Slow,
			ACA_Halt,
			ACA_Avoid
    };
    ACAMODE 		mode_curr = ACA_Sleep;
    ACAMODE			mode_prev = ACA_Sleep;

    //local timing variables
    int ACA_time_curr = 0;
	int ACA_time_stamp	= 0;//time stamp in avoid mode
	int ACA_time_stamp2	= 0;//time stamp in other modes
	int ACA_time_elapse =0; //time elapse in avoid mode
	int ACA_time_elapse2 =0;//time elapse in other modes
	int rc1_input = 0; //left(negative) right (positive)
	int rc2_input = 0; //back (negative) forward (positive)
	int rc3_input = 500; //elevation
	int rc4_input = 0; //yaw


    //flight control variables
    enum ACADIRECTION {
    		ACA_STAY,
            ACA_LEFT,
            ACA_RIGHT,
            ACA_UP,
            ACA_DOWN,
            ACA_FORWARD,
            ACA_BACKWARD
        };
    ACADIRECTION 		dir_curr = ACA_STAY;
    ACADIRECTION		dir_prev = ACA_STAY;

    //flags
    bool			system_start = 0; //0 ai sleep, 1 ai start
    bool			reset_speed = 0;//1 reset 0, don't reseet
    bool			halt_dir = 0;	//1 halt to slow, 0 halt to avoid
    bool			avoid_done = 0; //1 done, 0 not done
    bool			avoid_hor = 1;  //1 force horizontal, 0 ready to quit horizontal and enter forward
    bool 			fail = 0; //0 is default state. 1 means it failed to avoid and stays in STAY
    bool			avoid_emerg = 0; // 0 is default. 1 means it needs to go backwards
    /*
    void            update_one_hz_count();
    void            update_ten_hz_count();
    void 			update_fast_count();
     */
    void 			init();
    void			update_ten_hz(int milliseconds);		//include all ten hz functions, except update_lidar
    void		    update_lidar(int sonar_distance_cm);
    void			update_state();
    void			apply_avoid();
    void			update_rc();
private:

protected:

};
