//
// Created by oles on 02.03.16.
//

#include <string>
#include <iostream>
#include "andrzej_control/PCA9685.h"

static constexpr std::string usage = "./servo_test channel pwm";

int main(int argc, char *argv)
{
    PCA9685 pca(1,0x40);

    if(argc < 3)
    {
        std::cerr << usage << std::endl;
        return EXIT_FAILURE;
    }

    int channel = 0, pwm_to_set = 0;

    try
    {
        channel = std::stoi(argv[1]);
        pwm_to_set = std::stoi(argv[2]);
    }
    catch (std::invalid_argument ex)
    {
        std::cerr << usage << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Setting channel " << channel << " to " << pwm_to_set << " pwm..." << std::endl;

    pca.setPWM(static_cast<uint8_t>(channel), pwm_to_set);

    std::cout << "Reading pwm..." << std::endl;

    int pwm_on_board = 0;
    pwm_on_board = pca.getPWM(static_cast<uint8_t>(channel));

    std::cout << "Received " << pwm_on_board << " pwm on channel " << channel << std::endl;

    if(pwm_on_board != pwm_to_set)
    {
        std::cerr << "Written and read values differ" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}