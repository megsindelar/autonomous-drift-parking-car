#include "PID.h"

PID::PID(): K_p{0.0}, K_i{0.0}, K_d{0.0}{}

PID::PID(double Kp): K_p{Kp}, K_i{0.0}, K_d{0.0}{}

PID::PID(double Kp, double Ki): K_p{Kp}, K_i{Ki}, K_d{0.0}{}

PID::PID(double Kp, double Kd): K_p{Kp}, K_i{0.0}, K_d{Kd}{}

PID::PID(double Kp, double Ki, double Kd): K_p{Kp}, K_i{Ki}, K_d{Kd}{}

/*******************************************
 * pseudo-code - ref traj is vel and steer
 * 
 * note: this plan might be better so don't have to compute kinematics at each cycle (bc car moves fast)
 * *****************************************
 * 
 * 1. get next line ref traj (probs from csv)
 * 
 * read encoder data for both wheel encoder ticks and steer (convert ticks to velocity using time)
 * 
 * use IK to calculate how far wheels have gone from body twist (and put into world frame)
 * 
 * 
 *  -> maybe generate path beforehand where starts out by having a path that's in terms of world position
 *     and then do calculations to convert traj to ref traj of vel and steer and then run the car with those
 *      (use Kinematics to do this (and maybe in between orange cones is world frame (or near car starting pose is world?)))
 * 
 * 
*/


/*******************************************
 * pseudo-code - ref traj is world config
 * *****************************************
 * 
 * 1. get next line ref traj (probs from csv) (in world position where world is middle of cones)
 * 
 * find a twist to move to position & orientation
 * 
 * read encoder data for both wheel encoder ticks (convert to a vel) and steer and use as current body twist
 * 
 * use IK to calculate how far wheels have gone from body twist
 * 
 * use FK to calculate the config of the car (calc relative to world frame)
 *      -> use mecanum wheel model to estimate the slip
 * 
 * compare config to ref traj & use PID controller to find error in configs
 * 
 * find a new body twist to correct for error and send as control
 * 
*/