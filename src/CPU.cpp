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
    m_opcodeLookup[0xA9] = (Instruction){"LDA", AddressingMode::Immediate, &CPU::LDA, 2};
    m_opcodeLookup[0xA5] = (Instruction){"LDA", AddressingMode::ZeroPage, &CPU::LDA, 3};
    m_opcodeLookup[0xB5] = (Instruction){"LDA", AddressingMode::ZeroPageX, &CPU::LDA, 4};
    m_opcodeLookup[0xAD] = (Instruction){"LDA", AddressingMode::Absolute, &CPU::LDA, 4};
    m_opcodeLookup[0xBD] = (Instruction){"LDA", AddressingMode::AbsoluteX, &CPU::LDA, 4};
    m_opcodeLookup[0xB9] = (Instruction){"LDA", AddressingMode::AbsoluteY, &CPU::LDA, 4};
    m_opcodeLookup[0xA1] = (Instruction){"LDA", AddressingMode::IndexedIndirect, &CPU::LDA, 6};
    m_opcodeLookup[0xB1] = (Instruction){"LDA", AddressingMode::IndirectIndexed, &CPU::LDA, 5};

    m_opcodeLookup[0xEA] = (Instruction){"NOP", AddressingMode::Implicit, &CPU::NOP, 2};
    m_opcodeLookup[0xAA] = (Instruction){"TAX", AddressingMode::Implicit, &CPU::TAX, 2};
    m_opcodeLookup[0xE8] = (Instruction){"INX", AddressingMode::Implicit, &CPU::INX, 2};
    m_opcodeLookup[0x00] = (Instruction){"BRK", AddressingMode::Implicit, &CPU::BRK, 7};
}

CPU::~CPU()
{
}

void CPU::LDA(AddressingMode mode)
{
    m_acc = memoryRead(addressFromMode(mode));
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, (m_acc >> 7));
}

void CPU::NOP(AddressingMode mode)
{
    (void)mode;
    m_pc++;
}

void CPU::TAX(AddressingMode mode)
{
    (void)mode;
    m_regX = m_acc;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::INX(AddressingMode mode)
{
    (void)mode;
    m_regX++;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::BRK(AddressingMode mode)
{
    (void)mode;
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
        return ImmediateMode();
    case AddressingMode::Absolute:
        return AbsoluteMode();
    case AddressingMode::AbsoluteX:
        return AbsoluteIndexedXMode();
    case AddressingMode::AbsoluteY:
        return AbsoluteIndexedYMode();
    case AddressingMode::ZeroPage:
        return ZeroPageMode();
    case AddressingMode::Relative:
        return RelativeMode();
    case AddressingMode::Indirect:
        return IndirectMode();
    case AddressingMode::IndirectIndexed:
        return IndirectIndexedMode();
    case AddressingMode::IndexedIndirect:
        return IndexedIndirectMode();
    case AddressingMode::ZeroPageX:
        return ZeroPageXMode();
    case AddressingMode::ZeroPageY:
        return ZeroPageYMode();
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
    uint16_t addressLSB = (memoryRead(m_pc++) + m_regX) & 0xFF;
    uint16_t addressMSB = (addressLSB + 1) << 8;
    return memoryReadAddress(addressMSB | addressLSB);
}

uint16_t CPU::IndirectIndexedMode()
{
    uint16_t addrPtr = memoryRead(m_pc++);
    uint16_t lsb = memoryRead(addrPtr & 0xFF) + m_regY;
    uint16_t msb = memoryRead(addrPtr + 1) << 8;
    return msb | lsb;
}

uint16_t CPU::RelativeMode()
{
    int8_t offset = memoryRead(m_pc);
    m_pc += offset;
    return m_pc;
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

        if (!m_opcodeLookup.contains(opcode))
        {
            printf("Unknown opcode (%02x)\n", opcode);
            continue;
        }

        auto instruction = m_opcodeLookup[opcode];

        (this->*instruction.operation)(instruction.addrMode);

        printf("CPU State:\n");
        printf("    Program Counter: %04X, current instruction: %02X:%s:%d\n", m_pc, opcode, instruction.name.c_str(), instruction.cycles);
        printf("    Stack Pointer: %02X\n", m_sp);
        printf("    Accumulator: %02X\n", m_acc);
        printf("    Register X: %02X\n", m_regX);
        printf("    Register Y: %02X\n", m_regY);
        printf("    Status Register: %02X\n\n", m_procStatus);
    } while (opcode);
}

void CPU::setFlag(Flag flag, bool on)
{
    if (on)
        m_procStatus |= flag;
    else
        m_procStatus &= ~flag;
}
