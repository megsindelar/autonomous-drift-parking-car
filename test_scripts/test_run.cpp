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

#include <iostream>
// for delay function.
#include <chrono>
#include <map>
#include <string>
#include <thread>

// for signal handling
#include <signal.h>

#include <JetsonGPIO.h>

#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include <fstream>

#include <cstdio>
#include <unistd.h>
#include <SerialStream.h>

using namespace std;
const map<string, int> output_pins{
    {"JETSON_XAVIER", 18},    {"JETSON_NANO", 33},   {"JETSON_NX", 33},
    {"CLARA_AGX_XAVIER", 18}, {"JETSON_TX2_NX", 32}, {"JETSON_ORIN", 18},
};


int get_output_pin()
{
    if (output_pins.find(GPIO::model) == output_pins.end())
    {
        cerr << "PWM not supported on this board\n";
        terminate();
    }

    return output_pins.at(GPIO::model);
}

inline void delay(double s) { this_thread::sleep_for(std::chrono::duration<double>(s)); }

static bool end_this_program = false;

void signalHandler(int s) { end_this_program = true; }

int main()
{
    // Pin Definitions
    int output_pin = get_output_pin();

    std::string str; 
    std::ifstream f;
    std::fstream f_csv; 
    std::vector<std::string> deltas;

    // When CTRL+C pressed, signalHandler will be called
    signal(SIGINT, signalHandler);

    // Pin Setup.
    // Board pin-numbering scheme
    GPIO::setmode(GPIO::BOARD);

    // set pin as an output pin with optional initial state of HIGH
    GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
    GPIO::PWM p(output_pin, 50);
    auto val = 10.0;
    auto incr = 0.1;
    p.start(val);
    p.ChangeDutyCycle(val);

    cout << "PWM running. Press CTRL+C to exit." << endl;


    LibSerial::SerialStream my_serial("/dev/ttyACM0", LibSerial::SerialStreamBuf::BAUD_9600);
    sleep(2);
    int num = 65;
    my_serial << num << std::endl;
    std::cout << num << std::endl;
    sleep(1);

    // open throttle encoder port
    std::string str_throttle; 
    std::ifstream f_throttle;

    f_throttle.open("/dev/ttyACM2");
    std::cout << "open" << std::endl;


    // throttle
    double throttle_prev = 0.0;
    double throttle_enc_prev = 0.0;
    double throttle_enc = 0.0;
    double error_throttle = 0.0;    
    int delta_throttle_update = 0;
    int throttle_current = 0;
    double ts_throttle_prev = 0.0;

    timespec ts_first;
    timespec ts;
    double ts_prev = 0;
    double d_time = 0.0;
    int count_throttle = 0;

    //estimate
    double dt = 6;



    int count = 0;
    clock_gettime(CLOCK_REALTIME, &ts_first);
    while (!end_this_program)
    // while (count < 30)
    {
        // if (fd == -1){
        //     std::cout << "error: not open" << std::endl;
        //     close(fd);
        //     return -1;
        // }
        // std::cout << "open" << std::endl;
        // for (int i=0; i<16; i++){
        //     write(fd, &(data[i]), 1);
        //     std::cout << "working" << std::endl;
        //     write(fd, "", 1);
        //     sleep(1);
        // }
        // std::cout << "close" << std::endl;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // 6 to 11 is a really good range for testing and is perfectly in middle/slower range where incr is 0.1 and start at val of 9.0
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        delay(0.1);
        if (val <= 5){
            incr = -incr;
            // val += incr;
        }
        // // // if (val <= 6)
        // //     // incr = -incr;
        if (val >= 14){
            incr = -incr;
            // val -= incr;
        }
        val += incr;
        std::cout << val << std::endl;
        p.ChangeDutyCycle(val);
        // if (count == 27){
        //     num = 180;
        //     my_serial << num << std::endl;
        //     std::cout << num << std::endl;
        //     sleep(0.5);
        // }
        // std::cout << count << std::endl;
        count ++;

        // read throttle encoder
        // for (int k=1; k<5; k++){
        //     if(f_throttle >> str_throttle){
        //         std::string s_throttle = str_throttle;
        //         if (k%4 == 0){
        //             try{
        //                 clock_gettime(CLOCK_REALTIME, &ts);
        //                 d_time += ts.tv_nsec - ts_first.tv_nsec - ts_throttle_prev;
        //                 // std::cout << "time current: " << ts.tv_nsec << std::endl;
        //                 // std::cout << "time first: " << ts_first.tv_nsec << std::endl;
        //                 // std::cout << "time prev: " << ts_throttle_prev << std::endl;
        //                 // std::cout << "dt: " << d_time << std::endl;
        //                 ts_throttle_prev = ts.tv_nsec;
        //                 throttle_enc = -std::stod(s_throttle);
        //                 throttle_enc_prev += throttle_enc;
        //                 std::cout << "throttle enc !!: " << throttle_enc << std::endl;
        //                 std::cout << "throttle prev !!: " << throttle_enc_prev << std::endl;

        //                 if (count_throttle == 15){

        //                     // check and update here
        //                     // dt = 5;
        //                     // std::cout << "yar" << std::endl;
        //                     // std::cout << "dt: " << (-d_time/1e9) << std::endl;
        //                     // double dt_time = (-d_time/1e9);
        //                     // std::cout << "throttle prev !!: " << throttle_enc_prev << std::endl;
        //                     double throt = ((throttle_enc_prev/(2048*4))/dt);
        //                     std::cout << "throttle speed !!: " << throt << std::endl;
        //                     // std::cout << "throttle speed 2 !!: " << ((throttle_enc_prev/(2048*4))/(-d_time/1e9)) << std::endl;
        //                     // std::cout << "throttle speed 3 !!: " << ((throttle_enc_prev/(2048*4))/dt_time) << std::endl;
        //                     d_time = 0.0;
        //                     throttle_enc_prev = 0.0;
        //                     count_throttle = 0;
        //                 }
        //             } catch (const std::invalid_argument& e){
        //                 std::cout << "error throttle: " << s_throttle << std::endl;
        //                 std::cout << "Error: " << e.what() << std::endl;
        //                 end_this_program = true;
        //                 return 0;
        //             }
        //         }
        //     }
        // }
        // count_throttle++;
    }
    p.stop();

    sleep(1);
    my_serial << 65 << std::endl;
    std::cout << 65 << std::endl;
    sleep(0.5);

    GPIO::cleanup();

    return 0;
}

