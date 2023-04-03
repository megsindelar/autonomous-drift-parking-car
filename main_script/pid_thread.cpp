/*
Copyright (c) 2012-2017 Ben Croston ben@croston.org.
Copyright (c) 2019, NVIDIA CORPORATION.
Copyright (c) 2019 Jueon Park(pjueon) bluegbg@gmail.com.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/


// g++ -o pid_control pid_control.cpp -lJetsonGPIO -lpthread -lserial
// sudo ./pid_control

#include <iostream>
// for delay function.
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <mutex>

// for signal handling
#include <signal.h>

#include <JetsonGPIO.h>

#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include <fstream>
#include <sstream>

#include <cstdio>
#include <unistd.h>
#include <SerialStream.h>

#include <sys/time.h>
#include <stdexcept>

#include <future>

#include <queue>

#define LIMIT 10

using namespace std;
const map<string, int> output_pins{
    {"JETSON_XAVIER", 18},    {"JETSON_NANO", 33},   {"JETSON_NX", 33},
    {"CLARA_AGX_XAVIER", 18}, {"JETSON_TX2_NX", 32}, {"JETSON_ORIN", 18},
};

// define a mutex to synchronize access to shared data
std::mutex mtx;
bool stop_reading = false;


std::queue<double> data_queue;
std::mutex data_mutex;

inline void delay(double s) { this_thread::sleep_for(std::chrono::duration<double>(s)); }

static bool end_this_program = false;

void signalHandler(int s) { end_this_program = true; }

std::string str_throttle_thread;
std::string str_throttle_thread_prev = "";
int throttle_thread_count = 0;
double throttle_thread_prev = 0.0;

// read throttle encoder data in a separate thread
void throttle_encoder(std::ifstream& f_throttle){ 
    auto start = std::chrono::high_resolution_clock::now();
    while(f_throttle >> str_throttle_thread && !end_this_program){
        std::string s_throttle = str_throttle_thread;
        // lock shared data until done writing the data
        // std::lock_guard<std::mutex>> lock(mtx);
        if (!std::isalpha(s_throttle[0]) && str_throttle_thread_prev[0] == 'd'){
            if (std::stod(s_throttle) > 0){
                throttle_thread_count += 1;
            }
            throttle_thread_prev = std::stod(s_throttle);
        }
        str_throttle_thread_prev = s_throttle;
        auto time_check = std::chrono::high_resolution_clock::now();
        double time_diff = (std::chrono::duration_cast<std::chrono::nanoseconds>(time_check - start).count())*(10e-9);
        std::cout << "time diff: " << time_diff << std::endl;
        // return 1.0;
        if (time_diff >= 5.0){
            data_mutex.lock();
            data_queue.push(throttle_thread_count/time_diff);
            std::cout << "throttle thread vel: " << throttle_thread_count << std::endl;
            data_mutex.unlock();
            throttle_thread_count = 0;
            start = std::chrono::high_resolution_clock::now();
        }
    }
}



int get_output_pin()
{
    if (output_pins.find(GPIO::model) == output_pins.end())
    {
        cerr << "PWM not supported on this board\n";
        terminate();
    }

    return output_pins.at(GPIO::model);
}

class PI{
    private:
        double Kp = 0.0;
        double Ki = 0.0;
        double err_prev = 0.0;

    public:
        // constructor
        PI(double K_p, double K_i, double error_prev){
            Kp = K_p;
            Ki = K_i;
            err_prev = error_prev;
        }

        // function to update the steer
        double check_steer(double steer, double u_steer){
            // convert encoder reading position to command steer
            return (double)(u_steer - steer);
        }

        double update_steer(double error, double dt){
            if (error == 0){
                return 0.0;
            }
            else{
                // bound the integral error
                double int_error = (error-err_prev)*dt;
                if (int_error < -LIMIT){
                    int_error = -LIMIT;
                }
                if (int_error > LIMIT){
                    int_error = LIMIT;
                }
                err_prev = error;
                // update
                return Kp*error + Ki*int_error;
            }
        } 

        double check_throttle(double throttle, double u_throttle){
            return (double)(u_throttle - throttle);
        }

        double convert_throttle(double throttle){
            // from notes
            return (throttle/2.389441)*10.0;
        }

        double update_throttle(double error, double dt){
            if (error == 0){
                return 0.0;
            }
            else{
                // bound the integral error
                double int_error = (error-err_prev)*dt;
                if (int_error < -LIMIT){
                    int_error = -LIMIT;
                }
                if (int_error > LIMIT){
                    int_error = LIMIT;
                }
                err_prev = error;
                // update
                return Kp*error + Ki*int_error;
            }
        } 
};



int main()
{
    // read csv and store values
    std::fstream f_csv; 

    f_csv.open("path.csv", std::ios::in);

    std::cout << "open" << std::endl;
    std::vector<double> throttle;
    std::vector<double> steer;
    std::string line, word, temp;

    while (f_csv >> temp){
        throttle.clear();
        steer.clear();
        while (getline(f_csv, line)){
            std::stringstream s(line);

            int i = 0;
            // throttle first, then steer
            while(getline(s, word, ',')){
                double value = std::atof(word.c_str());
                if (i%2 == 0){
                    throttle.push_back(value);
                    std::cout << "throttle " << value << std::endl;
                }
                else{
                    steer.push_back(value);
                    std::cout << "steer " << value << std::endl;
                }
                i++;
            }
        }
    }

    // Pin Definitions
    int output_pin = get_output_pin();

    // When CTRL+C pressed, signalHandler will be called
    signal(SIGINT, signalHandler);

    // Pin Setup.
    // Board pin-numbering scheme
    GPIO::setmode(GPIO::BOARD);
    cout << "Hi." << endl;

    // set pin as an output pin with optional initial state of HIGH
    GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
    GPIO::PWM p(output_pin, 50);
    auto val = 10.0;
    auto incr = 0.1;
    p.start(val);
    p.ChangeDutyCycle(val);

    cout << "PWM running. Press CTRL+C to exit." << endl;

    bool arduino_open = false;
    // open arduino communication for servo
    LibSerial::SerialStream my_serial("/dev/ttyACM0", LibSerial::SerialStreamBuf::BAUD_9600);
    while (!arduino_open){
        try{
            if (my_serial.IsOpen()){
                sleep(2);
                int num = 65;
                my_serial << num << std::endl;
                std::cout << num << std::endl;
                arduino_open = true;
                sleep(1);
            }
            else{
                throw std::runtime_error("Arduino serial port not open!");
            }
        } catch (const std::runtime_error& e){
            arduino_open = false;
            end_this_program = true;
            std::cout << "Arduino serial port not open!" << std::endl;
            p.stop();
            return 0;
        }
    }

    // open steer encoder port
    std::string str_steer; 
    std::ifstream f_steer;
    cout << "Yar" << endl;
    bool steer_enc_port = false;
    f_steer.open("/dev/ttyACM1");
    while (!steer_enc_port){
        try{
            cout << "Please" << endl;
            if (!f_steer.is_open()){
                throw std::runtime_error("Steer encoder port not open!");
            }
            else{
                cout << "Yep" << endl;
                steer_enc_port = true;
            }
        } catch (const std::runtime_error& e){
            steer_enc_port = false;
            end_this_program = true;
            std::cout << "Steer encoder port not open!" << std::endl;
            p.stop();
            return 0;
        }
    }
    std::cout << "opened steer encoder port" << std::endl;

    // open throttle encoder port
    std::string str_throttle; 
    std::ifstream f_throttle;

    bool throttle_enc_port = false;
    f_throttle.open("/dev/ttyACM2");
    while (!throttle_enc_port){
        try{
            if (!f_throttle.is_open()){
                throw std::runtime_error("Throttle encoder port not open!");
            }
            else{
                throttle_enc_port = true;
            }
        } catch (const std::runtime_error& e){
            throttle_enc_port = false;
            end_this_program = true;
            std::cout << "Throttle encoder port not open!" << std::endl;
            p.stop();
            return 0;
        }
    }
    std::cout << "opened throttle encoder port" << std::endl;

    std::string s_steer_prev = "";
    std::string s_throttle_prev = "";


    // PI initialization
    double Kp = 0.2;
    double Ki = 0.07;
    PI correction = PI{Kp, Ki, 0.0};

    timespec ts_first;
    timespec ts;
    timespec ts_throttle;
    double ts_prev = 0;

    std::cout << ts.tv_sec << std::endl;
    
    std::cout << ts_prev << std::endl;



    // steer
    double steer_prev = 0.0;
    double steer_enc_prev = 0.0;
    double steer_enc = 0.0;
    double error_steer = 0.0;    
    int delta_steer_update = 0;
    int steer_current = 0;

    // throttle
    double throttle_prev = 0.0;
    double throttle_enc_prev = 0.0;
    double throttle_enc = 0.0;
    double error_throttle = 0.0;    
    int throttle_current = 0;
    double ts_throttle_prev = 0.0;
    // estimate
    double dt_throttle = 0.0;
    double delta_throttle_update = 0.0;

    bool steer_enc_measured = false;
    bool throttle_enc_measured = false;


    bool flag_steer = false;
    bool flag_throttle = false;

    int count = 0;
    int count_throttle = 0;
    double dt = 0.0;

    int count_th = 0;

    std::vector<double> stored_enc_throttle {};
    std::vector<double> stored_throttle {};
    std::vector<double> stored_steer {};

    double total_counts = 0.0;
    

    std::thread throttle_thread(throttle_encoder, std::ref(f_throttle));
    double throttle_vel = 0.0;

    // estimate based on experiments
    double d_time = 0.12862069;
    // while (!end_this_program)
    clock_gettime(CLOCK_REALTIME, &ts_first);
    while(!end_this_program){
        for (int i=0; i<throttle.size(); i++)
        {
            // std::future<double> future = std::async(std::launch::async, throttle_encoder, std::ref(f_throttle));
            delay(0.1);
            val = throttle.at(i);
            std::cout << "throttle command: " << throttle.at(i) << std::endl;
            p.ChangeDutyCycle(throttle.at(i));
            stored_throttle.push_back(throttle.at(i));
            my_serial << steer.at(i) << std::endl;
            std::cout << "steer command: " << steer.at(i) << std::endl;
            stored_steer.push_back(steer.at(i));
            // std::cout << count << std::endl;
            sleep(0.5);
            count ++;


            steer_enc = 0.0;
            steer_enc_prev = 0.0;
            error_steer = 0.0;
            delta_steer_update = 1;
            if (i == 0){
                steer_current = steer.at(i);

            throttle_enc = 0.0;
            error_throttle = 0.0;
            delta_throttle_update = 1;
            if (i == 0){
                throttle_current = throttle.at(i);
                std::cout << "throttle current wrong!: " << throttle_current << std::endl;
            }
            else{
                throttle_current = throttle.at(i-1);
                std::cout << "throttle current init: " << throttle_current << std::endl;
            }

            data_mutex.lock();
            while(!data_queue.empty()){
                throttle_vel = data_queue.front();
                std::cout << "AHHHHHHHHHHHHHHHHHHH" << std::endl;
                std::cout << "AHHHHHHHHHHHHHHHHHHH" << std::endl;
                std::cout << "AHHHHHHHHHHHHHHHHHHH" << std::endl;
                std::cout << "data_queue throttle vel: " << throttle_vel << std::endl;
                data_queue.pop();
            }
            data_mutex.unlock();

            // std::cout << "steer enc !!!: " << steer_enc + 65 << std::endl;
            error_steer = correction.check_steer((steer_current), steer.at(i));
            std::cout << "error: " << error_steer << std::endl;

            // std::cout << "steer current shouldn't change: " << steer_current << std::endl;

            clock_gettime(CLOCK_REALTIME, &ts);
            double dt = ts.tv_sec - ts_first.tv_sec - ts_prev;
            ts_prev = ts.tv_sec;

            if (flag_steer && steer_enc!=0.0){
                std::cout << "why not" << std::endl;
                std::cout << "error steer: " << error_steer << std::endl;
                delta_steer_update = correction.update_steer(error_steer, dt);
                steer_current += delta_steer_update;
                std::cout << "steer current: " << steer_current << std::endl;
                my_serial << steer_current << std::endl;

                
                flag_steer = false;
            }
            else if (steer_enc != steer_enc_prev) {
                // std::cout << "changed steer flag" << std::endl;
                flag_steer = true;
            }
            steer_enc_prev = steer_enc;



            sleep(0.5);
            std::cout << "done" << std::endl;
        }
        p.stop();
        end_this_program = true;
        throttle_thread.join();

        sleep(1.5);
        my_serial << 65 << std::endl;
        std::cout << 65 << std::endl;
        sleep(0.5);


        GPIO::cleanup();
        std::cout << "fully done" << std::endl;
    }
    return 0;
}

