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
#include <sstream>

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
    /////////////////////////////////////////////////////
    // read csv file and store throttle and steer path
    /////////////////////////////////////////////////////
    std::fstream f_csv; 
    // std::vector<std::string> deltas;

    // std::vector<double> throttle;
    // std::vector<double> steer;

    f_csv.open("path.csv", std::ios::in);

    std::cout << "open" << std::endl;
    std::vector<std::string> throttle;
    std::vector<std::string> steer;
    std::string line, word, temp;

    while (f_csv >> temp){
        while (getline(f_csv, line)){
            std::stringstream s(line);
            throttle.clear();
            steer.clear();

            int i = 0;
            // throttle first, then steer
            while(getline(s, word, ',')){
                if (i%2 == 0){
                    throttle.push_back(word);
                    std::cout << "throttle " << word << std::endl;
                }
                else{
                    steer.push_back(word);
                    std::cout << "steer " << word << std::endl;
                }
                i++;
            }
        }
    }


    /////////////////////////////////////////////////////
    // set pins and signal handler
    /////////////////////////////////////////////////////

    // // Pin Definitions
    // int output_pin = get_output_pin();

    // // When CTRL+C pressed, signalHandler will be called
    // signal(SIGINT, signalHandler);

    // // Pin Setup.
    // // Board pin-numbering scheme
    // GPIO::setmode(GPIO::BOARD);

    // // set pin as an output pin with optional initial state of HIGH
    // GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
    // GPIO::PWM p(output_pin, 50);
    // auto val = 10.0;
    // auto incr = 0.1;
    // p.start(val);
    // p.ChangeDutyCycle(val);

    // cout << "PWM running. Press CTRL+C to exit." << endl;







    /////////////////////////////////////////////////////
    // open arduino to steer car
    /////////////////////////////////////////////////////
    char data[] = {'Y', 'N'}; //, 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y'};
    char data_turn[] = {'Y'}; //, 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y'};
    char data_reset[] = {'N'}; //, 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y'};
    std::string str; 
    std::ifstream f;
    std::fstream f_csv; 
    std::vector<std::string> deltas;

    int fd;
    // f.open("/dev/ttyACM0");
    fd = open("/dev/ttyACM0", O_WRONLY);
    if (fd == -1){
        std::cout << "error: not open" << std::endl;
        close(fd);
        return -1;
    }
    std::cout << "open" << std::endl;


    /////////////////////////////////////////////////////
    // test steer
    /////////////////////////////////////////////////////
    for (int i=0; i<3; i++){
        write(fd, &(data[i]), 1);
        std::cout << "working" << std::endl;
        write(fd, "", 1);
        sleep(2);
    }


    /////////////////////////////////////////////////////
    // throttle and steer
    /////////////////////////////////////////////////////
    int count = 0;
    // while (!end_this_program)
    // while (count < 30)
    // {
    //     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //     // 6 to 11 is a really good range for testing and is perfectly in middle/slower range where incr is 0.1 and start at val of 9.0
    //     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //     delay(0.1);
    //     if (val <= 7){
    //         // incr = -incr;
    //         val = 7;
    //     }
    //     // // // if (val <= 6)
    //     // //     // incr = -incr;
    //     if (val > 7){
    //         // incr = -incr;
    //         val -= incr;
    //     }
    //     // val += incr;
    //     std::cout << val << std::endl;
    //     p.ChangeDutyCycle(val);
    //     if (count == 29){
    //         write(fd, &(data_turn[0]), 1);
    //         std::cout << "working" << std::endl;
    //         write(fd, "", 1);
    //         sleep(1);
    //     }
    //     // std::cout << count << std::endl;
    //     count ++;
    // }
    // p.stop();


    write(fd, &(data_reset[0]), 1);
    std::cout << "working" << std::endl;
    write(fd, "", 1);
    sleep(2);
    // close(fd);

    std::cout << "close" << std::endl;
    close(fd);

    // GPIO::cleanup();

    return 0;
}

