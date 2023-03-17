// /*
// Copyright (c) 2012-2017 Ben Croston ben@croston.org.
// Copyright (c) 2019, NVIDIA CORPORATION.
// Copyright (c) 2019 Jueon Park(pjueon) bluegbg@gmail.com.

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
// */

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>



// #include <iostream>
// // for delay function.
// #include <chrono>
// #include <map>
// #include <string>
// #include <thread>

// // for signal handling
// #include <signal.h>

// #include <JetsonGPIO.h>

// using namespace std;
// const map<string, int> output_pins{
//     {"JETSON_XAVIER", 18},    {"JETSON_NANO", 33},   {"JETSON_NX", 33},
//     {"CLARA_AGX_XAVIER", 18}, {"JETSON_TX2_NX", 32}, {"JETSON_ORIN", 18},
// };

// int get_output_pin()
// {
//     if (output_pins.find(GPIO::model) == output_pins.end())
//     {
//         cerr << "PWM not supported on this board\n";
//         terminate();
//     }

//     return output_pins.at(GPIO::model);
// }

// inline void delay(double s) { this_thread::sleep_for(std::chrono::duration<double>(s)); }

// static bool end_this_program = false;

// void signalHandler(int s) { end_this_program = true; }

//************************************************************************************
// NOTE: need to use sudo to run
//*************************************************************************************
int main(void){
    char data[] = {'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'N', 'Y', 'Y', 'Y', 'Y', 'Y', 'Y'};
    std::string str; 
    // std::ifstream f;
    std::fstream f_csv; 
    std::vector<std::string> deltas;

    std::vector<double> throttle;
    std::vector<double> steer;

    f_csv.open("path.csv", std::ios::in);

    std::cout << "open" << std::endl;
    std::vector<string> row;
    string line, word, temp;

    while (fin >> temp){
        row.clear();

        std::getline(f_csv, line);

        std::stringstream s(line);

        while (getline(s, word, ', ')){
            row.push_back(word);
        }
    }


    // writing to a csv
    // for (int i = 0; i<20; i++){
    //     std::cin >> throttle[i]
    //             >> steer[i];
    //     std::cout << "throttle"
    //             << throttle[i]
    //             << "steer"
    //             << steer[i] << std::endl;
    // }
    // std::cout << "done" << std::endl;
    for (int i = 0; i<20; i++){
        std::cout << "throttle"
                << throttle[i]
                << "steer"
                << steer[i] << std::endl;
    }
        std::cout << str << '\n';
    i++;

    
    f_csv.close();
    // f.close();
    return 0;
}