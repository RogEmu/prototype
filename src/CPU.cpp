/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** CPU
*/
#include "CPU.h"

#include <iostream>

CPU::CPU() :
    m_pc(0),
    m_sp(0),
    m_acc(0),
    m_regX(0),
    m_regY(0),
    m_procStatus(0)
{
}

CPU::~CPU()
{
}

void CPU::LDA()
{
    m_acc = memoryRead(m_pc++);
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, (m_acc >> 7));
}

void CPU::NOP()
{
    m_pc++;
}

void CPU::TAX()
{
    m_regX = m_acc;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::INX()
{
    m_regX++;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::BRK()
{
    setFlag(Flag::B, true);
}

uint8_t CPU::memoryRead(uint16_t addr)
{
    return m_mem[addr];
}

uint16_t CPU::memoryReadAddress(uint16_t addr)
{
    uint8_t highAddr = m_mem[addr + 1];
    uint8_t lowAddr = m_mem[addr];
    return (highAddr << 8) | lowAddr;
}

void CPU::memoryWrite(uint16_t addr, uint8_t data)
{
    m_mem[addr] = data;
}

uint8_t CPU::fetchByte()
{
    uint8_t byte = memoryRead(m_pc++);
    return byte;
}

uint16_t CPU::addressFromMode(AddressingMode mode)
{
    switch (mode)
    {
    case AddressingMode::Immediate:
        return m_pc;
    case AddressingMode::Absolute:
        return memoryReadAddress(m_pc);
    case AddressingMode::ZeroPage:
        return memoryRead(m_pc) & 0xFF;
    case AddressingMode::Relative:
        return m_pc + memoryRead(m_pc);
    case AddressingMode::Indirect:
        return memoryReadAddress(memoryReadAddress(m_pc));
    default:
        break;
    }
    return 0;
}

void CPU::stackPush(uint8_t byte)
{
    memoryWrite(0x0100 | m_sp--, byte);
}

uint8_t CPU::stackPull()
{
    uint8_t byte = memoryRead(0x0100 | m_sp++);
    return byte;
}

uint16_t CPU::ImmediateMode()
{
    return m_pc++;
}

uint16_t CPU::AbsoluteMode()
{
    uint16_t addr = memoryReadAddress(m_pc);
    m_pc += 2;
    return addr;
}

uint16_t CPU::AbsoluteIndexedXMode()
{
    uint16_t addr = memoryReadAddress(m_pc) + m_regX;
    m_pc += 2;
    return addr;
}

uint16_t CPU::AbsoluteIndexedYMode()
{
    uint16_t addr = memoryReadAddress(m_pc) + m_regY;
    m_pc += 2;
    return addr;
}

uint16_t CPU::ZeroPageMode()
{
    uint16_t addr = memoryRead(m_pc++);
    addr &= 0x00FF;
    return addr;
}

uint16_t CPU::ZeroPageXMode()
{
    uint16_t addr = memoryRead(m_pc++) + m_regX;
    addr &= 0x00FF;
    return addr;
}

uint16_t CPU::ZeroPageYMode()
{
    uint16_t addr = memoryRead(m_pc++) + m_regY;
    addr &= 0x00FF;
    return addr;
}

uint16_t CPU::IndirectMode()
{
    uint16_t indirectAddr = memoryReadAddress(m_pc);
    m_pc += 2;
    return memoryReadAddress(indirectAddr);
}

uint16_t CPU::IndexedIndirectMode()
{
    return 0;
}

void CPU::reset()
{
    m_acc = 0;
    m_regX = 0;
    m_regY = 0;
    m_sp = 0xFF;
    m_procStatus = 0;
    m_pc = memoryReadAddress(0xFFFC);
}

void CPU::loadMemory(const std::vector<uint8_t> &program)
{
    size_t maxSize = 0xFFFF - 0x8000;

    if (program.size() > maxSize)
    {
        std::cout << "Program memory is too large" << std::endl;
        return;
    }
    std::copy(program.begin(), program.end(), m_mem + 0x8000);
    memoryWrite(0xFFFC, 0x00);
    memoryWrite(0xFFFD, 0x80);
}

void CPU::run()
{
    uint8_t opcode = 0;
    do
    {
        opcode = fetchByte();
        switch (opcode)
        {
        case OpCode::LDA:
            LDA();
            break;
        case OpCode::TAX :
            TAX();
            break;
        case OpCode::INX:
            INX();
            break;
        case OpCode::BRK:
            BRK();
            break;
        default:
            break;
        }

        printf("CPU State:\n");
        printf("    Program Counter: %X, current opcode: %02X\n", m_pc, opcode);
        printf("    Stack Pointer: %X\n", m_sp);
        printf("    Accumulator: %X\n", m_acc);
        printf("    Register X: %X\n", m_regX);
        printf("    Register Y: %X\n", m_regY);
        printf("    Status Register: %X\n\n", m_procStatus);
    } while (opcode);
}

void CPU::setFlag(Flag flag, bool on)
{
    if (on)
        m_procStatus |= flag;
    else
        m_procStatus &= ~flag;
}
