/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** main
*/

#include "CPU.h"

int main()
{
    CPU cpu;
    std::vector<uint8_t> memory = {0xa9, 0xc0, 0xaa, 0xe8, 0x00};

    cpu.loadMemory(memory);
    cpu.reset();
    cpu.run();
    return 0;
}
