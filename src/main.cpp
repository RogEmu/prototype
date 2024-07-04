/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** main
*/

#include "CPU.h"
#include "Bus.h"

#include <iostream>

int main(int ac, char**av)
{
    if (ac != 2)
    {
        std::cout << "Input ROM file" << std::endl;
        return -1;
    }

    CPU cpu;
    Bus bus;

    bus.loadFromFile(std::string(av[1]));
    bus.connectCPU(&cpu);
    cpu.connectBus(&bus);
    cpu.memoryWrite(0xFFFC, 0x00);
    cpu.memoryWrite(0xFFFD, 0xC0);
    cpu.reset();
    cpu.run();
    return 0;
}
