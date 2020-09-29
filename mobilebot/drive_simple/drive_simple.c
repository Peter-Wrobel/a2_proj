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

double v_goal = 0.01;
double last_p_v_term = 0;
double p_v_term = 0;
double d_v_term = 0;
double l_pwm = 0;
double r_pwm = 0;
double last_l_enc = 0;
double l_enc = 0;
double last_r_enc = 0;
double r_enc = 0;

void simple_motor_command_handler(const lcm_recv_buf_t* rbuf,
                                  const char* channel,
                                  const simple_motor_command_t* msg,
                                  void* user);

double get

void calc_pd_v();

void pd_controller();

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
	while(rc_get_state()==RUNNING){
        watchdog_timer += 0.01;
        if(watchdog_timer >= 0.25)
        {
            rc_motor_set(1,0.0);
            rc_motor_set(2,0.0);
            printf("timeout...\r");
        }
		// define a timeout (for erroring out) and the delay time
        // calc_pd_v();
		pd_controller();
		lcm_handle_timeout(lcm, 1);
        publish_encoder_msg();
        rc_nanosleep(1E9 / 100); //handle at 10Hz
		
		last_l_enc = l_enc;
		l_enc = double(enc_l_pol * rc_encoder_eqep_read(1));
		last_r_enc = r_enc;
		r_enc = double(enc_r_pol * rc_encoder_eqep_read(2));
		
		last_p_v_term = p_v_term;
		delta_L = l_enc - last_l_enc;
    	delta_R = r_enc - last_r_enc;
    	delta_s = (delta_L+delta_R)*(2*M_PI*0.042)/(2.0*20.78);
    	double t2 = 0.1;
		p_v_term = v_goal - delta_s/t2;
		d_v_term = (p_v_term - last_p_v_term)/t2;
	}
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    lcm_destroy(lcm);
	// rc_remove_pid_file();   // remove pid file LAST

    print_answers();
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

void calc_pd_v(){
	mbot_encoder_t encoder1;
    encoder1.utime = rc_nanos_since_epoch();
    encoder1.left_delta = 0;
    encoder1.right_delta = 0;
    encoder1.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder1.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    
    mbot_encoder_t encoder2;
    encoder2.utime = rc_nanos_since_epoch();
    encoder2.left_delta = 0;
    encoder2.right_delta = 0;
    encoder2.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder2.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    double delta_L = (double) (encoder2.leftticks - encoder1.leftticks);
    double delta_R = (double) (encoder2.rightticks - encoder1.rightticks);
    double delta_s = (delta_L+delta_R)*(2*M_PI*0.042)/(2.0*20.78);
    double t1 = (double) (encoder2.utime-encoder1.utime)*0.000001;
	double last_p_v_term = v_goal - delta_s/t1;

	mbot_encoder_t encoder3;
    encoder3.utime = rc_nanos_since_epoch();
    encoder3.left_delta = 0;
    encoder3.right_delta = 0;
    encoder3.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder3.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    
	mbot_encoder_t encoder4;
    encoder4.utime = rc_nanos_since_epoch();
    encoder4.left_delta = 0;
    encoder4.right_delta = 0;
    encoder4.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder4.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    delta_L = (double) (encoder4.leftticks - encoder3.leftticks);
    delta_R = (double) (encoder4.rightticks - encoder3.rightticks);
    delta_s = (delta_L+delta_R)*(2*M_PI*0.042)/(2.0*20.78);
    double t2 = (double) (encoder4.utime-encoder3.utime)*0.000001;
	p_v_term = v_goal - delta_s/t2;
	
	double t_e = (double) (encoder4.utime + encoder3.utime - encoder2.utime - encoder1.utime)*0.000001/2.0;
	d_v_term = (p_v_term-last_p_v_term)/t_e;
}

void simple_motor_command_handler(const lcm_recv_buf_t* rbuf,
                                  const char* channel,
                                  const simple_motor_command_t* msg,
                                  void* user){
    watchdog_timer = 0.0;
    rc_motor_set(1, 0.15 * (msg->forward_velocity + msg->angular_velocity));
    rc_motor_set(2, 0.15 * (msg->forward_velocity - msg->angular_velocity));
}

void pd_controller(){
	watchdog_timer = 0.0;
	double k_p = 0.1;
	double k_d = 0.0;
    printf("e: %f\n", p_v_term);
    printf("e_dot: %f\n\n", d_v_term);
	l_pwm = l_pwm + k_p*p_v_term + k_d*d_v_term;
	r_pwm = r_pwm + k_p*p_v_term + k_d*d_v_term;
	rc_motor_set(1, mot_l_pol * l_pwm);
	rc_motor_set(2, mot_r_pol * r_pwm);
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
    mbot_encoder_t last_encoder;
    last_encoder.utime = rc_nanos_since_epoch();
    last_encoder.left_delta = 0;
    last_encoder.right_delta = 0;
    last_encoder.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    last_encoder.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    
    mbot_encoder_t encoder_msg;
    encoder_msg.utime = rc_nanos_since_epoch();
    encoder_msg.left_delta = 0;
    encoder_msg.right_delta = 0;
    encoder_msg.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder_msg.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    double delta_L = (double) (encoder_msg.leftticks - last_encoder.leftticks);
    double delta_R = (double) (encoder_msg.rightticks - last_encoder.rightticks);
    double delta_s = (delta_L+delta_R)*(2*M_PI*0.042)/(2.0*20.78);
    double delta_theta = (-delta_L+delta_R)*(2*M_PI*0.042)/(0.11*20.78);
    double t = (double) (encoder_msg.utime-last_encoder.utime)*0.000001;
    printf(" ENC: %lld | %lld  - v: %f | w: %f \r", encoder_msg.leftticks, encoder_msg.rightticks, delta_s/t, delta_theta/t);
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}

void print_answers()
{
    /// Question 1:
    /// When you give your motors the same PWM command does the robot 
    /// move straight? Why not? What could be the reason for that? 
    /// How can we fix that?
    printf("Answer 1:\n Print your answer here \n");

    /// Question 2:
    /// What could be some uses of logs in our project? why would we 
    /// want to play logs at different speeds?
    printf("Answer 2:\n Print your answer here \n");

}
