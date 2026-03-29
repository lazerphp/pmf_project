#include "Simulation.h"
#include <iostream>

int main()
{
    std::cout << "Запуск симуляции..." << std::endl;
    try
    {
        Simulation sim;
        sim.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Ошибка: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}