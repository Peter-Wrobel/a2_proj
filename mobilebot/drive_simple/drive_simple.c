/*******************************************************************************
* drive_simple.c
*
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <getopt.h>

#include <lcm/lcm.h>
#include "../lcmtypes/steer_command_t.h"
#include "../lcmtypes/turn_command_t.h"
#include "../lcmtypes/rpi_state_t.h"
#include "../lcmtypes/bbb_state_t.h"
#include "../lcmtypes/mbot_encoder_t.h"
#include "../lcmtypes/simple_motor_command_t.h"

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL                "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL   "MBOT_MOTOR_COMMAND_SIMPLE"
#define M_PI 3.14159265358979323846

//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;

//functions
void publish_encoder_msg();
void print_answers();

int mot_l_pol;
int mot_r_pol;
int enc_l_pol;
int enc_r_pol;

int8_t mode = 0;
// 0: move forward
// 1: stop
// 2: turn

// Task I: v control
float v_goal = 0.1;
float last_p_v_term = 0;
float p_v_term = 0;
float d_v_term = 0;
float l_pwm = 0;
float r_pwm = 0;

// Task II: w control
float p_w_term = 0;
float d_w_term = 0;

// Task IV: turn control
float p_turn = 0;
float d_turn = 0;

float vel = 0;
float ang = 0;

int64_t t_prev = 0;
int64_t t_prev_v = 0;
int64_t left_prev = 0;
int64_t right_prev = 0;

void simple_motor_command_handler(const lcm_recv_buf_t* rbuf,
                                  const char* channel,
                                  const simple_motor_command_t* msg,
                                  void* user);

void state_handler(const lcm_recv_buf_t* rbuf,
                   const char* channel,
                   const rpi_state_t* msg,
                   void* user);

void steer_command_handler(const lcm_recv_buf_t* rbuf,
                          const char* channel,
                          const steer_command_t* msg,
                          void* user);

void turn_command_handler(const lcm_recv_buf_t* rbuf,
                          const char* channel,
                          const turn_command_t* msg,
                          void* user);

void pd_controller();

void stop_controller();

void turn_controller();

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char *argv[]){
    //check args
    if( argc != 5 ) {
        printf("Usage: test_simple {left motor polarity} {right motor polarity} {left encoder polarity} {right encoder polarity}\n");
        printf("Example: test_simple -1 1 -1 1\n");
        return 0;
    }

    mot_l_pol = atoi(argv[1]);
    mot_r_pol = atoi(argv[2]);
    enc_l_pol = atoi(argv[3]);
    enc_r_pol = atoi(argv[4]);

    if( ((mot_l_pol != 1)&(mot_l_pol != -1)) |
        ((mot_r_pol != 1)&(mot_r_pol != -1)) |
        ((enc_l_pol != 1)&(enc_l_pol != -1)) |
        ((enc_r_pol != 1)&(enc_r_pol != -1))){
        printf("Usage: polarities must be -1 or 1\n");
        return 0;
      }

	// make sure another instance isn't running
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// itart signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

    if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }

    lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");

    // simple_motor_command_t_subscribe(lcm, "MBOT_MOTOR_COMMAND_SIMPLE", &simple_motor_command_handler, NULL);
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	// rc_make_pid_file();

	// done initializing so set state to RUNNING
    rc_encoder_eqep_init();
    rc_set_state(RUNNING);
    
    watchdog_timer = 0.0;
    printf("Running...\n");
    rpi_state_t_subscribe(lcm, "RPI_STATE", &state_handler, NULL);
    steer_command_t_subscribe(lcm, "STEER", &steer_command_handler, NULL);
    turn_command_t_subscribe(lcm, "TURN", &turn_command_handler, NULL);
  
    

    while(rc_get_state()==RUNNING){
//    for(int i = 0; i < 50; i++) {
        watchdog_timer += 0.01;
/*	if (i > 30) {
		mode = 1;
	}*/
        if(watchdog_timer >= 0.25)
        {
            rc_motor_set(1,0.0);
            rc_motor_set(2,0.0);
            printf("timeout...\r");
        }
	// define a timeout (for erroring out) and the delay time
	if (mode == 0) {
		pd_controller();
	}
	else if (mode == 1) {
		stop_controller();
		if (vel < 0.0000001) {
			mode = 2;
			rpi_state_t data = {
				.state = 2
			};
			bbb_state_t_publish(lcm, "BBB_STATE", &data);
		}
	}
	else if (mode == 2) {
		turn_controller();

	}
	rc_motor_set(1, l_pwm);
	rc_motor_set(2, r_pwm);
	lcm_handle_timeout(lcm, 1);
        publish_encoder_msg();
        rc_nanosleep(1E9 / 10); //handle at 10Hz
		
		last_p_v_term = p_v_term;
        float t = (float) (t_prev - t_prev_v) / 1E9;
		p_v_term = v_goal - vel;
		d_v_term = (p_v_term - last_p_v_term)/t;
	}
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    lcm_destroy(lcm);
	// rc_remove_pid_file();   // remove pid file LAST

	return 0;
}

/*******************************************************************************
*  simple_motor_command_handler()
*
*  sets motor PWMS from incoming lcm message
*
*******************************************************************************/
//////////////////////////////////////////////////////////////////////////////
/// TODO: Create a handler that receives lcm message simple_motor_command_t and
/// sets motor PWM according to the recieved message.
/// command the motor using the command: rc_motor_set(channel, polarity * pwm);
/// for now the pwm value should be proportional to the velocity you send, 
/// the sign of the velocity should indicate direction, and angular velocity 
//  indicates turning rate. 
//////////////////////////////////////////////////////////////////////////////

void simple_motor_command_handler(const lcm_recv_buf_t* rbuf,
                                  const char* channel,
                                  const simple_motor_command_t* msg,
                                  void* user) {
    watchdog_timer = 0.0;
    rc_motor_set(1, 0.15 * (msg->forward_velocity + msg->angular_velocity));
    rc_motor_set(2, 0.15 * (msg->forward_velocity - msg->angular_velocity));
}


void state_handler(const lcm_recv_buf_t* rbuf,
                   const char* channel,
                   const rpi_state_t* msg,
                   void* user) {
	watchdog_timer = 0.0;
	mode = msg->state;
}

void steer_command_handler(const lcm_recv_buf_t* rbuf,
                           const char* channel,
                           const steer_command_t* msg,
                           void* user) {
	watchdog_timer = 0.0;
	p_w_term = msg->p_term;
	d_w_term = msg->d_term;
}

void turn_command_handler(const lcm_recv_buf_t* rbuf,
                          const char* channel,
                          const turn_command_t* msg,
                          void* user) {
    watchdog_timer = 0.0;
	p_w_term = msg->p_term;
	d_w_term = msg->d_term;                          
}

void pd_controller() {
	watchdog_timer = 0.0;
	float k_p = 0.5;
	float k_d = 0.1;
	float k_p_w = 0.01;
	float k_d_w = 0.001;
    // printf("e: %f\n", p_v_term);
    // printf("e_dot: %f\n\n", d_v_term);
	l_pwm = l_pwm + k_p*p_v_term + k_d*d_v_term;
	r_pwm = r_pwm + k_p*p_v_term + k_d*d_v_term;
	
	l_pwm = l_pwm + k_p_w*p_w_term + k_d_w*d_w_term;
	r_pwm = r_pwm - k_p_w*p_w_term - k_d_w*d_w_term;
}

void stop_controller() {
	watchdog_timer = 0.0;
	float k = 1.0;
	l_pwm = l_pwm - k*v_goal*v_goal;
	r_pwm = r_pwm - k*v_goal*v_goal;
}

void turn_controller() {
	watchdog_timer = 0.0;
	float k_p = 0.5;
	float k_d = 0.1;
	float k_p_turn = 0.01;
	float k_d_turn = 0.001;
	l_pwm = l_pwm + k_p*p_v_term + k_d*d_v_term;
	r_pwm = r_pwm + k_p*p_v_term + k_d*d_v_term;
	
	l_pwm = l_pwm + k_p*p_turn + k_d*d_turn;
	l_pwm = l_pwm - k_p*p_turn - k_d*d_turn;
}

/*******************************************************************************
* void publish_encoder_msg()
*
* publishes LCM message of encoder reading
* 
*******************************************************************************/
void publish_encoder_msg(){
    //////////////////////////////////////////////////////////////////////////////
    /// TODO: update this fuction by calculating and printing the forward speeds(v) 
    ///     and angular speeds (w).
    //////////////////////////////////////////////////////////////////////////////
    
    mbot_encoder_t encoder_msg;
    encoder_msg.utime = rc_nanos_since_epoch();
    t_prev_v = t_prev;

    int64_t curr_left = enc_l_pol * rc_encoder_eqep_read(1);
    int64_t curr_right = enc_r_pol * rc_encoder_eqep_read(2);
   
    encoder_msg.left_delta = curr_left - left_prev;
    encoder_msg.right_delta = curr_right - right_prev;
    encoder_msg.leftticks = curr_left;
    encoder_msg.rightticks = curr_right;
    
    float t_passed = (float) (encoder_msg.utime - t_prev) / 1E9;

    vel = (encoder_msg.left_delta+encoder_msg.right_delta)*(2*M_PI*0.042)/(2.0*20*78*t_passed);
    ang = (-encoder_msg.left_delta+encoder_msg.right_delta)*(2*M_PI*0.042)/(0.11*20*78*t_passed);
    printf(" ENC: %lld | %lld  - v: %f | w: %f | p-term: %f | l-pwm: %f | - t_pass: %f | t: %lld\n", encoder_msg.leftticks, encoder_msg.rightticks, vel, ang, p_v_term, l_pwm, t_passed, encoder_msg.utime);

    t_prev = encoder_msg.utime;
    left_prev = curr_left;
    right_prev = curr_right;

    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}

