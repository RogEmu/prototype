/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** main
*/

#include "CPU.h"

int main(int ac, char**av)
{
    CPU cpu;
    std::vector<uint8_t> memory = {0xa5, 0x10, 0x00};

    cpu.loadMemory(memory);
    cpu.memoryWrite(0x10,0x55);
    cpu.reset();
    cpu.run();
    return 0;
}
