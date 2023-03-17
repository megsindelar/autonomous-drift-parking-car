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

#include <cstdio>
#include <unistd.h>
#include <SerialStream.h>

#include <sys/time.h>
#include <stdexcept>

inline void delay(double s) {std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

static bool end_this_program = false;

void signalHandler(int s) { end_this_program = true; }

int main()
{

    // Pin Definitions
    int output_pin = 33; //get_output_pin();
    int input_throttle_pin = 12;

    // When CTRL+C pressed, signalHandler will be called
    signal(SIGINT, signalHandler);

    // Pin Setup.
    // Board pin-numbering scheme
    GPIO::setmode(GPIO::BOARD);

    // reading hall effect throttle
    GPIO::setup(input_throttle_pin, GPIO::IN);

    int count_th = 0;

    while(!end_this_program){
        int state = GPIO::input(input_throttle_pin);
        if (state == GPIO::LOW){
            count_th ++;
            state = 1;
            std::cout << "count: " << count_th << std::endl;
            std::cout << "state: " << GPIO::input(input_throttle_pin) << std::endl;
        }
        delay(1.0);

    }

    GPIO::cleanup();
    end_this_program = true;
    std::cout << "done" << std::endl;
    return 0;

}