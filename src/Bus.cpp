#include "Bus.h"

#include "CPU.h"
#include <fstream>
#include <iostream>

Bus::Bus()
{
    std::fill(m_ram, m_ram + 0x10000, 0);
}

Bus::~Bus()
{
}

void Bus::cpuWrite(uint16_t addr, uint8_t byte)
{
        m_ram[addr] = byte;
}

uint8_t Bus::cpuRead(uint16_t addr)
{
        return m_ram[addr];
}

void Bus::loadFromFile(const std::string &filename)
{
    std::ifstream fileStream;

    fileStream.open(filename);
    if (!fileStream.is_open())
    {
        std::cout << "Couldn't open file: " << filename << std::endl;
        return;
    }
    fileStream.read((char *)m_ram, 0x10);
    fileStream.read((char *)m_ram + 0xC000, 0x4000);
}

void Bus::connectCPU(CPU *cpu)
{
    m_cpu = cpu;
}
