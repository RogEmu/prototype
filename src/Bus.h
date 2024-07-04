/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** Bus
*/

#ifndef BUS_H_
#define BUS_H_

#include <cstdint>
#include <array>
#include <string>

class CPU;

class Bus
{
    public:
        Bus();
        ~Bus();

        void cpuWrite(uint16_t addr, uint8_t byte);
        uint8_t cpuRead(uint16_t addr);

        void loadFromFile(const std::string& filename);

        void connectCPU(CPU *cpu);

    private:
        CPU *m_cpu;
        uint8_t m_ram[0x10000];
};

#endif /* !BUS_H_ */
