#include <iostream>
// for delay function.
#include <chrono>
#include <map>
#include <string>
#include <thread>

// for signal handling
#include <signal.h>

#include <vector>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include <fstream>
#include <sstream>

#include <cstring>

#include <locale>
#include <codecvt>

// #include "serial/serial.h"
// #include "serial.cc"

#include <cstdio>
#include <unistd.h>
// #include "serial/serial.h"
#include <SerialStream.h>


static bool end_this_program = false;

void signalHandler(int s) { end_this_program = true; }



int main(){

    // // int data[] = {'Y180', 'Y90', 'Y65', 'Y45', 'Y20', 'Y65'};
    // // std::vector<int> data = {180, 65, 65, 180, 20, 65};


    // std::fstream f_csv; 
    // // std::vector<std::string> deltas;

    // // std::vector<double> throttle;
    // // std::vector<double> steer;

    // f_csv.open("path.csv", std::ios::in);

    // std::cout << "open" << std::endl;
    // std::vector<std::string> throttle;
    // std::vector<std::string> steer;
    // std::string line, word, temp;

    // while (f_csv >> temp){
    //     throttle.clear();
    //     steer.clear();
    //     while (getline(f_csv, line)){
    //         std::stringstream s(line);

    //         int i = 0;
    //         // throttle first, then steer
    //         while(getline(s, word, ',')){
    //             if (i%2 == 0){
    //                 throttle.push_back(word);
    //                 std::cout << "throttle " << word << std::endl;
    //             }
    //             else{
    //                 steer.push_back(word);
    //                 std::cout << "steer " << word << std::endl;
    //             }
    //             i++;
    //         }
    //     }
    // }

    // for (int i = 0; i<sizeof(steer); i++){
    //     std::cout << "steer vec " << steer[i] << std::endl;
    // }



    // serial::Serial arduino;
    // arduino.setPort("/dev/ttyACM0");
    // arduino.setBaudrate(9600);
    // arduino.open();

    // int data = 42;
    // arduino.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));

    // arduino.close();
    LibSerial::SerialStream my_serial("/dev/ttyACM0", LibSerial::SerialStreamBuf::BAUD_9600);
    sleep(2);
    // char cstr[16];
    // int num = 20;
    // std::string temp = std::to_string(num);
    // char const *num_char = temp.c_str();
    // my_serial << num_char << std::endl;
    int num = 50;
    my_serial << num << std::endl;
    std::cout << num << std::endl;
    sleep(1);
    for (int i=0; i<130; i++){
        my_serial << num << std::endl;
        std::cout << num << std::endl;
        sleep(0.5);
        num += 1;
    }
    // my_serial << 90 << std::endl;
    // std::cout << "90" << std::endl;
    // sleep(1);
    // my_serial << 180 << std::endl;
    // std::cout << "180" << std::endl;
    // sleep(1);
    // my_serial << 20 << std::endl;
    // std::cout << "20" << std::endl;
    // sleep(1);
    // my_serial << 65 << std::endl;
    // std::cout << "65" << std::endl;
    // sleep(1);
    return 0;



    
    // std::vector<char> data = {'Y', 'N', 'Y', 'N', 'Y', 'N'};
    // std::string str; 
    // std::ifstream f;
    // std::vector<std::string> deltas;

    // std::string y = "Y";
    // std::string n = "N";

    // // When CTRL+C pressed, signalHandler will be called
    // signal(SIGINT, signalHandler);

    // int fd;
    // // f.open("/dev/ttyACM0");
    // fd = open("/dev/ttyACM0", O_WRONLY);
    // if (fd == -1){
    //     std::cout << "error: not open" << std::endl;
    //     close(fd);
    //     return -1;
    // }
    // std::cout << "open" << std::endl;

    // while (!end_this_program){
    //     // for (int a = 17; a < sizeof(steer); a++){
    //     for (int a = 16; a < 20; a++){
    //         std::cout << steer[a] << std::endl;
    //         // std::vector<char> c = {};
    //         // write(fd, &(y), 1);
    //         // sleep(2);
    //         std::string str = steer[a];
    //         std::cout << "str " << str << std::endl;
    //         std::string s = y;
    //         for(int i = 0; i < 3; i++){
    //         // for(int i = 0; i < 5; i++){
    //             s += str[i];
    //             // char c = 'M';
    //         }
    //         // s += n;
    //         std::cout << "s " << s << std::endl;
    //         char* char_array = new char[sizeof(str) + 3];
    //         std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    //         // std::wstring w_str = s;
            
    //         // std::cout << "char array "<< utf8_str << std::endl;
    //         // write(fd, &(s), 1);
    //         // write(fd, "", 1);
    //         // sleep(2);
    //         std::wstring w_str = L"Y160";
    //         std::string utf8_str = converter.to_bytes(w_str);
    //         write(fd, &utf8_str, 1);
    //         write(fd, "", 1);
    //         sleep(2);
    //         // char_array = {};
    //         // write(fd, &(data[0]), 1);
    //         // std::cout << "working" << std::endl;
    //         // write(fd, "", 1);
    //         // sleep(2);
    //     }











        // for (int i = 0; i < 6; i++){
        // std::string s = std::to_string(data[0]);
        // write(fd, &s, 1);
        // std::cout << data[0] << std::endl;
        // write(fd, "", 1);
        // sleep(2);

        // for (int i=0; i<6; i++){
        //     write(fd, &(data[i]), 1);
        //     std::cout << "working" << std::endl;
        //     write(fd, "", 1);
        //     sleep(2);
        // }

        // std::string dat = std::to_string(data[0]);
        // std::cout << dat << std::endl;
        // // std::vector<char> c = {};
        // write(fd, &(y), 1);
        // sleep(2);
        // // for(int i = 0; i < dat.length(); i++){
        // //     char c = char(dat[i]);
        // //     std::cout << c << std::endl;
        // //     write(fd, &(c), 1);
        // // }
        // write(fd, &(n), 1);
        // std::cout << data[0] << std::endl;
        // sleep(2);

        

        // dat = std::to_string(data[1]);
        // write(fd, &(y), 1);
        // for(int i = 0; i < dat.length(); i++){
        //     char c = char(dat[i]);
        //     std::cout << c << std::endl;
        //     write(fd, &(c), 1);
        // }
        // std::cout << data[1] << std::endl;
        // write(fd, &(n), 1);
        // sleep(2);

        // dat = std::to_string(data[2]);
        // write(fd, &(y), 1);
        // for(int i = 0; i < dat.length(); i++){
        //     char c = char(dat[i]);
        //     std::cout << c << std::endl;
        //     write(fd, &(c), 1);
        // }
        // std::cout << data[1] << std::endl;
        // write(fd, &(n), 1);
        // sleep(2);





        // char c = '0' + data[0];
        // write(fd, &c, 1);
        // std::cout << data[0] << std::endl;
        // write(fd, "", 1);
        // sleep(2);

        // c = '0' + data[1];
        // write(fd, &c, 1);
        // std::cout << data[1] << std::endl;
        // write(fd, "", 1);
        // sleep(2);

        // c = '0' + data[2];
        // write(fd, &c, 1);
        // std::cout << data[2] << std::endl;
        // write(fd, "", 1);
        // sleep(2);

        // c = char(data[3]);
        // write(fd, &c, 1);
        // std::cout << data[3] << std::endl;
        // write(fd, "", 1);
        // sleep(2);
        
        // }
    // }

    // std::cout << "close" << std::endl;
    // close(fd);
}