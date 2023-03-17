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




int main(){
    std::string str_steer; 
    std::ifstream f_steer;

    f_steer.open("/dev/ttyACM2");
    std::cout << "open" << std::endl;

    std::string s_steer_prev = "";

    bool steer_enc_measured = false;

    double steer_enc = 0.0;

    int m = 2;
    // for (int i=1; i<50000000; i++){
    //     if(f >> str){
    //         std::cout << str << std::endl;
    //         std::string s = str;
    //         if (i%4 == 0){
    //             double i = std::stod(s);
    //             std::cout << i << std::endl;
    //         }
    //     }
    // }
    for (int i = 0; i < 5000; i++){
        while (!steer_enc_measured){
            if(f_steer >> str_steer){
                std::string s_steer = str_steer;
                std::cout << s_steer << std::endl;
                // std::cout << "prev: " << s_steer_prev << std::endl;
                if (!std::isalpha(s_steer[0]) && s_steer_prev[0] == 'd'){
                    try{
                        if (std::isalpha(s_steer[0])){
                            throw std::invalid_argument("Not a num for steer enc");
                        }
                        else{
                            steer_enc = -std::stod(s_steer);
                            std::cout << "yar" << std::endl;
                            std::cout << "steer enc 1: " << steer_enc << std::endl;
                            // steer_enc_measured = true;
                        }
                    } catch (const std::invalid_argument& e){
                        std::cout << "error steer: " << s_steer << std::endl;
                        std::cout << "Error: " << e.what() << std::endl;
                        // end_this_program = true;
                        // p.stop();
                        return 0;
                    }
                }
                s_steer_prev = s_steer;
            }
        }
    }

    // std::string str_throttle; 
    // std::ifstream f_throttle;

    // f_throttle.open("/dev/ttyACM2");
    // std::cout << "open" << std::endl;
    // for (int i=1; i<20; i++){
    //     if(f_throttle >> str_throttle){
    //         std::string s_throttle = str_throttle;
    //         if (i%4 == 0){
    //             // double i = std::stod(s_throttle);
    //             // std::cout << i << std::endl;
    //         // }
    //             try{
    //                 double throttle_init = -std::stod(s_throttle);
    //                 std::cout << "yar" << std::endl;
    //                 std::cout << "throttle init: " << throttle_init << std::endl;
    //             } catch (const std::invalid_argument& e){
    //                 std::cout << "error throttle: " << s_throttle << std::endl;
    //                 std::cout << "Error: " << e.what() << std::endl;
    //                 // end_this_program = true;
    //                 return 0;
    //             }
    //         }
    //     }
    // }
}
    