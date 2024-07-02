/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** CPU
*/
#include "CPU.h"

CPU::CPU() :
    m_procStatus(0x20)
{
}

CPU::~CPU()
{
}

void CPU::lda(uint8_t byte)
{
    m_acc = byte;
    m_pc++;
    if (m_acc == 0)
        setFlag(Flag::FLAG_ZERO);
    else
        unsetFlag(Flag::FLAG_ZERO);
    if (m_acc >> 7)
        setFlag(Flag::FLAG_NEGATIVE);
    else
        unsetFlag(Flag::FLAG_NEGATIVE);
}

void CPU::tax()
{
    m_regX = m_acc;
    if (m_regX == 0)
        setFlag(Flag::FLAG_ZERO);
    else
        unsetFlag(Flag::FLAG_ZERO);
    if (m_regX >> 7)
        setFlag(Flag::FLAG_NEGATIVE);
    else
        unsetFlag(Flag::FLAG_NEGATIVE);
}

void CPU::inx()
{
    m_regX++;
    if (m_regX == 0)
        setFlag(Flag::FLAG_ZERO);
    else
        unsetFlag(Flag::FLAG_ZERO);
    if (m_regX >> 7)
        setFlag(Flag::FLAG_NEGATIVE);
    else
        unsetFlag(Flag::FLAG_NEGATIVE);
}

void CPU::brk()
{
    setFlag(Flag::FLAG_B);
}

uint8_t CPU::mem_read(uint16_t addr)
{
    return m_mem[addr];
}

void CPU::mem_write(uint16_t addr, uint8_t data)
{
    m_mem[addr] = data;
}

void CPU::load(std::vector<uint8_t> &program)
{
    m_mem[0x8000-(0x8000+program.size())] = program.copy();
    m_pc = 0x8000;
}

void CPU::run()
{
    int instruction = 0;

    while (true)
    {
        instruction = mem_read(m_pc);
        m_pc++;
    }
}

void CPU::setFlag(Flag flag)
{
    m_procStatus |= flag;
}

void CPU::unsetFlag(Flag flag)
{
    m_procStatus &= ~flag;
}
