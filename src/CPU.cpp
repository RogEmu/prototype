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

void CPU::setFlag(Flag flag)
{
    m_procStatus |= flag;
}

void CPU::unsetFlag(Flag flag)
{
    m_procStatus &= ~flag;
}
