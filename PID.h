#ifndef PID_INCLUDE_GUARD_HPP
#define PID_INCLUDE_GUARD_HPP

class PID{
    private:
        double K_p = 0.0;
        double K_i = 0.0;
        double K_d = 0.0;
    public:
        PID();

        explicit PID(double Kp);

        explicit PID(double Kp, double Ki);

        explicit PID(double Kp, double Kd);

        explicit PID(double Kp, double Ki, double Kd);

};

#endif