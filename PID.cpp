#include "PID.h"

PID::PID(): K_p{0.0}, K_i{0.0}, K_d{0.0}{}

PID::PID(double Kp): K_p{Kp}, K_i{0.0}, K_d{0.0}{}

PID::PID(double Kp, double Ki): K_p{Kp}, K_i{Ki}, K_d{0.0}{}

PID::PID(double Kp, double Kd): K_p{Kp}, K_i{0.0}, K_d{Kd}{}

PID::PID(double Kp, double Ki, double Kd): K_p{Kp}, K_i{Ki}, K_d{Kd}{}

