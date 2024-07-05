/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** CPU
*/
#include "CPU.h"

#include <iostream>
#include <bitset>
#include <iomanip>
#include <sstream>

#include "Bus.h"

#define PAGE_CROSS(x, y) (((x) & 0xFF00) != ((y) & 0xFF00))

CPU::CPU():
    m_pc(0),
    m_sp(0),
    m_acc(0),
    m_regX(0),
    m_regY(0),
    m_procStatus(Flag::UNUSED),
    adrFromMode(0),
    m_currentCycles(0),
    m_totalCycles(0)
{
    m_opcodeLookup[0x69] = (Instruction){"ADC", AddressingMode::Immediate, &CPU::ADC, 2};
    m_opcodeLookup[0x65] = (Instruction){"ADC", AddressingMode::ZeroPage, &CPU::ADC, 3};
    m_opcodeLookup[0x75] = (Instruction){"ADC", AddressingMode::ZeroPageX, &CPU::ADC, 4};
    m_opcodeLookup[0x6D] = (Instruction){"ADC", AddressingMode::Absolute, &CPU::ADC, 4};
    m_opcodeLookup[0x7D] = (Instruction){"ADC", AddressingMode::AbsoluteX, &CPU::ADC, 4};
    m_opcodeLookup[0x79] = (Instruction){"ADC", AddressingMode::AbsoluteY, &CPU::ADC, 4};
    m_opcodeLookup[0x61] = (Instruction){"ADC", AddressingMode::IndexedIndirect, &CPU::ADC, 6};
    m_opcodeLookup[0x71] = (Instruction){"ADC", AddressingMode::IndirectIndexed, &CPU::ADC, 5};

    m_opcodeLookup[0x29] = (Instruction){"AND", AddressingMode::Immediate, &CPU::AND, 2};
    m_opcodeLookup[0x25] = (Instruction){"AND", AddressingMode::ZeroPage, &CPU::AND, 3};
    m_opcodeLookup[0x35] = (Instruction){"AND", AddressingMode::ZeroPageX, &CPU::AND, 4};
    m_opcodeLookup[0x2D] = (Instruction){"AND", AddressingMode::Absolute, &CPU::AND, 4};
    m_opcodeLookup[0x23] = (Instruction){"AND", AddressingMode::AbsoluteX, &CPU::AND, 4};
    m_opcodeLookup[0x39] = (Instruction){"AND", AddressingMode::AbsoluteY, &CPU::AND, 4};
    m_opcodeLookup[0x21] = (Instruction){"AND", AddressingMode::IndexedIndirect, &CPU::AND, 6};
    m_opcodeLookup[0x31] = (Instruction){"AND", AddressingMode::IndirectIndexed, &CPU::AND, 5};

    m_opcodeLookup[0x0A] = (Instruction){"ASL", AddressingMode::Implicit, &CPU::ASL, 2};
    m_opcodeLookup[0x06] = (Instruction){"ASL", AddressingMode::ZeroPage, &CPU::ASL, 5};
    m_opcodeLookup[0x16] = (Instruction){"ASL", AddressingMode::ZeroPageX, &CPU::ASL, 6};
    m_opcodeLookup[0x0E] = (Instruction){"ASL", AddressingMode::Absolute, &CPU::ASL, 6};
    m_opcodeLookup[0x1E] = (Instruction){"ASL", AddressingMode::AbsoluteX, &CPU::ASL, 7};

    m_opcodeLookup[0x90] = (Instruction){"BCC", AddressingMode::Relative, &CPU::BCC, 2};
    m_opcodeLookup[0xB0] = (Instruction){"BCS", AddressingMode::Relative, &CPU::BCS, 2};
    m_opcodeLookup[0xF0] = (Instruction){"BEQ", AddressingMode::Relative, &CPU::BEQ, 2};
    m_opcodeLookup[0x30] = (Instruction){"BMI", AddressingMode::Relative, &CPU::BMI, 2};
    m_opcodeLookup[0xD0] = (Instruction){"BNE", AddressingMode::Relative, &CPU::BNE, 2};
    m_opcodeLookup[0x10] = (Instruction){"BPL", AddressingMode::Relative, &CPU::BPL, 2};
    m_opcodeLookup[0x50] = (Instruction){"BVC", AddressingMode::Relative, &CPU::BVC, 2};
    m_opcodeLookup[0x70] = (Instruction){"BVS", AddressingMode::Relative, &CPU::BVS, 2};

    m_opcodeLookup[0x24] = (Instruction){"BIT", AddressingMode::ZeroPage, &CPU::BIT, 3};
    m_opcodeLookup[0x2C] = (Instruction){"BIT", AddressingMode::Absolute, &CPU::BIT, 4};

    m_opcodeLookup[0x00] = (Instruction){"BRK", AddressingMode::Implicit, &CPU::BRK, 7};

    m_opcodeLookup[0x18] = (Instruction){"CLC", AddressingMode::Implicit, &CPU::CLC, 2};
    m_opcodeLookup[0xD8] = (Instruction){"CLD", AddressingMode::Implicit, &CPU::CLD, 2};
    m_opcodeLookup[0x58] = (Instruction){"CLI", AddressingMode::Implicit, &CPU::CLI, 2};
    m_opcodeLookup[0xB8] = (Instruction){"CLV", AddressingMode::Implicit, &CPU::CLV, 2};

    m_opcodeLookup[0xC9] = (Instruction){"CMP", AddressingMode::Immediate, &CPU::CMP, 2};
    m_opcodeLookup[0xC5] = (Instruction){"CMP", AddressingMode::ZeroPage, &CPU::CMP, 3};
    m_opcodeLookup[0xD5] = (Instruction){"CMP", AddressingMode::ZeroPageX, &CPU::CMP, 4};
    m_opcodeLookup[0xCD] = (Instruction){"CMP", AddressingMode::Absolute, &CPU::CMP, 4};
    m_opcodeLookup[0xDD] = (Instruction){"CMP", AddressingMode::AbsoluteX, &CPU::CMP, 4};
    m_opcodeLookup[0xD9] = (Instruction){"CMP", AddressingMode::AbsoluteY, &CPU::CMP, 4};
    m_opcodeLookup[0xC1] = (Instruction){"CMP", AddressingMode::IndexedIndirect, &CPU::CMP, 6};
    m_opcodeLookup[0xD1] = (Instruction){"CMP", AddressingMode::IndirectIndexed, &CPU::CMP, 5};

    m_opcodeLookup[0xE0] = (Instruction){"CPX", AddressingMode::Immediate, &CPU::CPX, 2};
    m_opcodeLookup[0xE4] = (Instruction){"CPX", AddressingMode::ZeroPage, &CPU::CPX, 3};
    m_opcodeLookup[0xEC] = (Instruction){"CPX", AddressingMode::Absolute, &CPU::CPX, 4};

    m_opcodeLookup[0xC0] = (Instruction){"CPY", AddressingMode::Immediate, &CPU::CPY, 2};
    m_opcodeLookup[0xC4] = (Instruction){"CPY", AddressingMode::ZeroPage, &CPU::CPY, 3};
    m_opcodeLookup[0xCC] = (Instruction){"CPY", AddressingMode::Absolute, &CPU::CPY, 4};

    m_opcodeLookup[0xC6] = (Instruction){"DEC", AddressingMode::ZeroPage, &CPU::DEC, 5};
    m_opcodeLookup[0xD6] = (Instruction){"DEC", AddressingMode::ZeroPageX, &CPU::DEC, 6};
    m_opcodeLookup[0xCE] = (Instruction){"DEC", AddressingMode::Absolute, &CPU::DEC, 6};
    m_opcodeLookup[0xDE] = (Instruction){"DEC", AddressingMode::AbsoluteX, &CPU::DEC, 7};

    m_opcodeLookup[0xCA] = (Instruction){"DEX", AddressingMode::Implicit, &CPU::DEX, 2};
    m_opcodeLookup[0x88] = (Instruction){"DEY", AddressingMode::Implicit, &CPU::DEY, 2};

    m_opcodeLookup[0x49] = (Instruction){"EOR", AddressingMode::Immediate, &CPU::EOR, 2};
    m_opcodeLookup[0x45] = (Instruction){"EOR", AddressingMode::ZeroPage, &CPU::EOR, 3};
    m_opcodeLookup[0x55] = (Instruction){"EOR", AddressingMode::ZeroPageX, &CPU::EOR, 4};
    m_opcodeLookup[0x4D] = (Instruction){"EOR", AddressingMode::Absolute, &CPU::EOR, 4};
    m_opcodeLookup[0x5D] = (Instruction){"EOR", AddressingMode::AbsoluteX, &CPU::EOR, 4};
    m_opcodeLookup[0x59] = (Instruction){"EOR", AddressingMode::AbsoluteX, &CPU::EOR, 4};
    m_opcodeLookup[0x41] = (Instruction){"EOR", AddressingMode::IndexedIndirect, &CPU::EOR, 6};
    m_opcodeLookup[0x51] = (Instruction){"EOR", AddressingMode::IndirectIndexed, &CPU::EOR, 5};

    m_opcodeLookup[0xE6] = (Instruction){"INC", AddressingMode::ZeroPage, &CPU::INC, 5};
    m_opcodeLookup[0xF6] = (Instruction){"INC", AddressingMode::ZeroPageX, &CPU::INC, 6};
    m_opcodeLookup[0xEE] = (Instruction){"INC", AddressingMode::Absolute, &CPU::INC, 6};
    m_opcodeLookup[0xFE] = (Instruction){"INC", AddressingMode::AbsoluteX, &CPU::INC, 7};

    m_opcodeLookup[0xE8] = (Instruction){"INX", AddressingMode::Implicit, &CPU::INX, 2};
    m_opcodeLookup[0xC8] = (Instruction){"INY", AddressingMode::Implicit, &CPU::INY, 2};

    m_opcodeLookup[0x4C] = (Instruction){"JMP", AddressingMode::Absolute, &CPU::JMP, 3};
    m_opcodeLookup[0x6C] = (Instruction){"JMP", AddressingMode::Indirect, &CPU::JMP, 5};

    m_opcodeLookup[0xA9] = (Instruction){"LDA", AddressingMode::Immediate, &CPU::LDA, 2};
    m_opcodeLookup[0xA5] = (Instruction){"LDA", AddressingMode::ZeroPage, &CPU::LDA, 3};
    m_opcodeLookup[0xB5] = (Instruction){"LDA", AddressingMode::ZeroPageX, &CPU::LDA, 4};
    m_opcodeLookup[0xAD] = (Instruction){"LDA", AddressingMode::Absolute, &CPU::LDA, 4};
    m_opcodeLookup[0xBD] = (Instruction){"LDA", AddressingMode::AbsoluteX, &CPU::LDA, 4};
    m_opcodeLookup[0xB9] = (Instruction){"LDA", AddressingMode::AbsoluteY, &CPU::LDA, 4};
    m_opcodeLookup[0xA1] = (Instruction){"LDA", AddressingMode::IndexedIndirect, &CPU::LDA, 6};
    m_opcodeLookup[0xB1] = (Instruction){"LDA", AddressingMode::IndirectIndexed, &CPU::LDA, 5};

    m_opcodeLookup[0xEA] = (Instruction){"NOP", AddressingMode::Implicit, &CPU::NOP, 2};

    m_opcodeLookup[0x20] = (Instruction){"JSR", AddressingMode::Absolute, &CPU::JSR, 6};

    m_opcodeLookup[0xA2] = (Instruction){"LDX", AddressingMode::Immediate, &CPU::LDX, 2};
    m_opcodeLookup[0xA6] = (Instruction){"LDX", AddressingMode::ZeroPage, &CPU::LDX, 3};
    m_opcodeLookup[0xB6] = (Instruction){"LDX", AddressingMode::ZeroPageY, &CPU::LDX, 4};
    m_opcodeLookup[0xAE] = (Instruction){"LDX", AddressingMode::Absolute, &CPU::LDX, 4};
    m_opcodeLookup[0xBE] = (Instruction){"LDX", AddressingMode::AbsoluteY, &CPU::LDX, 4};

    m_opcodeLookup[0xA0] = (Instruction){"LDY", AddressingMode::Immediate, &CPU::LDY, 2};
    m_opcodeLookup[0xA4] = (Instruction){"LDY", AddressingMode::ZeroPage, &CPU::LDY, 3};
    m_opcodeLookup[0xB4] = (Instruction){"LDY", AddressingMode::ZeroPageX, &CPU::LDY, 4};
    m_opcodeLookup[0xAC] = (Instruction){"LDY", AddressingMode::Absolute, &CPU::LDY, 4};
    m_opcodeLookup[0xBC] = (Instruction){"LDY", AddressingMode::AbsoluteX, &CPU::LDY, 4};

    m_opcodeLookup[0x4A] = (Instruction){"LSR", AddressingMode::Implicit, &CPU::LSR, 2};
    m_opcodeLookup[0x46] = (Instruction){"LSR", AddressingMode::ZeroPage, &CPU::LSR, 5};
    m_opcodeLookup[0x56] = (Instruction){"LSR", AddressingMode::ZeroPageX, &CPU::LSR, 6};
    m_opcodeLookup[0x4E] = (Instruction){"LSR", AddressingMode::Absolute, &CPU::LSR, 6};
    m_opcodeLookup[0x5E] = (Instruction){"LSR", AddressingMode::AbsoluteX, &CPU::LSR, 7};

    m_opcodeLookup[0x09] = (Instruction){"ORA", AddressingMode::Immediate, &CPU::ORA, 2};
    m_opcodeLookup[0x05] = (Instruction){"ORA", AddressingMode::ZeroPage, &CPU::ORA, 3};
    m_opcodeLookup[0x15] = (Instruction){"ORA", AddressingMode::ZeroPageX, &CPU::ORA, 4};
    m_opcodeLookup[0x0D] = (Instruction){"ORA", AddressingMode::Absolute, &CPU::ORA, 4};
    m_opcodeLookup[0x1D] = (Instruction){"ORA", AddressingMode::AbsoluteX, &CPU::ORA, 4};
    m_opcodeLookup[0x19] = (Instruction){"ORA", AddressingMode::AbsoluteY, &CPU::ORA, 4};
    m_opcodeLookup[0x01] = (Instruction){"ORA", AddressingMode::IndexedIndirect, &CPU::ORA, 6};
    m_opcodeLookup[0x11] = (Instruction){"ORA", AddressingMode::IndirectIndexed, &CPU::ORA, 5};

    m_opcodeLookup[0x48] = (Instruction){"PHA", AddressingMode::Implicit, &CPU::PHA, 3};
    m_opcodeLookup[0x08] = (Instruction){"PHP", AddressingMode::Implicit, &CPU::PHP, 3};
    m_opcodeLookup[0x68] = (Instruction){"PLA", AddressingMode::Implicit, &CPU::PLA, 4};
    m_opcodeLookup[0x28] = (Instruction){"PLP", AddressingMode::Implicit, &CPU::PLP, 4};

    m_opcodeLookup[0x2A] = (Instruction){"ROL", AddressingMode::Implicit, &CPU::ROL, 2};
    m_opcodeLookup[0x26] = (Instruction){"ROL", AddressingMode::ZeroPage, &CPU::ROL, 5};
    m_opcodeLookup[0x36] = (Instruction){"ROL", AddressingMode::ZeroPageX, &CPU::ROL, 6};
    m_opcodeLookup[0x2E] = (Instruction){"ROL", AddressingMode::Absolute, &CPU::ROL, 6};
    m_opcodeLookup[0x2E] = (Instruction){"ROL", AddressingMode::AbsoluteX, &CPU::ROL, 7};

    m_opcodeLookup[0x6A] = (Instruction){"ROR", AddressingMode::Implicit, &CPU::ROR, 2};
    m_opcodeLookup[0x66] = (Instruction){"ROR", AddressingMode::ZeroPage, &CPU::ROR, 5};
    m_opcodeLookup[0x76] = (Instruction){"ROR", AddressingMode::ZeroPageX, &CPU::ROR, 6};
    m_opcodeLookup[0x6E] = (Instruction){"ROR", AddressingMode::Absolute, &CPU::ROR, 6};
    m_opcodeLookup[0x7E] = (Instruction){"ROR", AddressingMode::AbsoluteX, &CPU::ROR, 7};

    m_opcodeLookup[0x40] = (Instruction){"RTI", AddressingMode::Implicit, &CPU::RTI, 6};
    m_opcodeLookup[0x60] = (Instruction){"RTS", AddressingMode::Implicit, &CPU::RTS, 6};

    m_opcodeLookup[0xE9] = (Instruction){"SBC", AddressingMode::Immediate, &CPU::SBC, 2};
    m_opcodeLookup[0xE5] = (Instruction){"SBC", AddressingMode::ZeroPage, &CPU::SBC, 3};
    m_opcodeLookup[0xF5] = (Instruction){"SBC", AddressingMode::ZeroPageX, &CPU::SBC, 4};
    m_opcodeLookup[0xED] = (Instruction){"SBC", AddressingMode::Absolute, &CPU::SBC, 4};
    m_opcodeLookup[0xFD] = (Instruction){"SBC", AddressingMode::AbsoluteX, &CPU::SBC, 4};
    m_opcodeLookup[0xF9] = (Instruction){"SBC", AddressingMode::AbsoluteY, &CPU::SBC, 4};
    m_opcodeLookup[0xE1] = (Instruction){"SBC", AddressingMode::IndexedIndirect, &CPU::SBC, 6};
    m_opcodeLookup[0xF1] = (Instruction){"SBC", AddressingMode::IndirectIndexed, &CPU::SBC, 5};

    m_opcodeLookup[0x38] = (Instruction){"SEC", AddressingMode::Implicit, &CPU::SEC, 2};
    m_opcodeLookup[0xF8] = (Instruction){"SED", AddressingMode::Implicit, &CPU::SED, 2};
    m_opcodeLookup[0x78] = (Instruction){"SEI", AddressingMode::Implicit, &CPU::SEI, 2};

    m_opcodeLookup[0x85] = (Instruction){"STA", AddressingMode::ZeroPage, &CPU::STA, 3};
    m_opcodeLookup[0x95] = (Instruction){"STA", AddressingMode::ZeroPageX, &CPU::STA, 4};
    m_opcodeLookup[0x8D] = (Instruction){"STA", AddressingMode::Absolute, &CPU::STA, 4};
    m_opcodeLookup[0x9D] = (Instruction){"STA", AddressingMode::AbsoluteX, &CPU::STA, 5};
    m_opcodeLookup[0x99] = (Instruction){"STA", AddressingMode::AbsoluteY, &CPU::STA, 5};
    m_opcodeLookup[0x81] = (Instruction){"STA", AddressingMode::IndexedIndirect, &CPU::STA, 6};
    m_opcodeLookup[0x91] = (Instruction){"STA", AddressingMode::IndirectIndexed, &CPU::STA, 6};

    m_opcodeLookup[0x86] = (Instruction){"STX", AddressingMode::ZeroPage, &CPU::STX, 3};
    m_opcodeLookup[0x96] = (Instruction){"STX", AddressingMode::ZeroPageY, &CPU::STX, 4};
    m_opcodeLookup[0x8E] = (Instruction){"STX", AddressingMode::Absolute, &CPU::STX, 4};

    m_opcodeLookup[0x84] = (Instruction){"STY", AddressingMode::ZeroPage, &CPU::STY, 3};
    m_opcodeLookup[0x94] = (Instruction){"STY", AddressingMode::ZeroPageX, &CPU::STY, 4};
    m_opcodeLookup[0x8C] = (Instruction){"STY", AddressingMode::Absolute, &CPU::STY, 4};

    m_opcodeLookup[0xAA] = (Instruction){"TAX", AddressingMode::Implicit, &CPU::TAX, 2};
    m_opcodeLookup[0xA8] = (Instruction){"TAY", AddressingMode::Implicit, &CPU::TAY, 2};
    m_opcodeLookup[0xBA] = (Instruction){"TSX", AddressingMode::Implicit, &CPU::TSX, 2};
    m_opcodeLookup[0x8A] = (Instruction){"TXA", AddressingMode::Implicit, &CPU::TXA, 2};
    m_opcodeLookup[0x9A] = (Instruction){"TXS", AddressingMode::Implicit, &CPU::TXS, 2};
    m_opcodeLookup[0x98] = (Instruction){"TYA", AddressingMode::Implicit, &CPU::TYA, 2};
}

CPU::~CPU()
{
}

void CPU::ADC(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t data = memoryRead(addr);
    uint16_t result = m_acc + data + (m_procStatus & Flag::CARRY);

    setFlag(Flag::CARRY, result > 0xFF);
    setFlag(Flag::ZERO, (result & 0xFF) == 0);
    setFlag(Flag::OVERFLOW, (~(m_acc ^ data) & (m_acc ^ result)) & 0x0080);
    setFlag(Flag::NEGATIVE, (result & 0x80));
    m_acc = result & 0x00FF;
}

void CPU::AND(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    m_acc &= byte;
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, m_acc >> 7);
}

void CPU::ASL(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = (mode == AddressingMode::Implicit) ? m_acc : memoryRead(addr);

    setFlag(Flag::CARRY, byte & 0x80);
    byte <<= 1;
    setFlag(Flag::ZERO, byte == 0);
    setFlag(Flag::NEGATIVE, byte & 0x80);
    if (mode == AddressingMode::Implicit)
        m_acc = byte;
    else
        memoryWrite(addr, byte);
}

void CPU::BCC(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (!isFLagSet(Flag::CARRY))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BCS(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (isFLagSet(Flag::CARRY))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BEQ(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (isFLagSet(Flag::ZERO))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BIT(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);
    uint8_t res = m_acc & byte;

    setFlag(Flag::ZERO, res == 0);
    setFlag(Flag::OVERFLOW, byte & 0x40);
    setFlag(Flag::NEGATIVE, byte & 0x80);
}

void CPU::BMI(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (isFLagSet(Flag::NEGATIVE))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BNE(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (!isFLagSet(Flag::ZERO))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BPL(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (!isFLagSet(Flag::NEGATIVE))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BRK(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::B, true);
}

void CPU::BVC(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (!isFLagSet(Flag::OVERFLOW))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::BVS(AddressingMode mode)
{
    int8_t offset = addressFromMode(mode) & 0xFF;

    if (isFLagSet(Flag::OVERFLOW))
    {
        m_pc += offset;
        m_currentCycles++;
        if (PAGE_CROSS(m_pc + offset, m_pc))
            m_currentCycles++;
    }
}

void CPU::CLC(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::CARRY, 0);
}

void CPU::CLD(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::DECMODE_DISABLE, 0);
}

void CPU::CLI(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::INT_DISABLE, 0);
}

void CPU::CLV(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::OVERFLOW, 0);
}

void CPU::CMP(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    uint8_t res = m_acc - byte;
    setFlag(Flag::CARRY, m_acc >= byte);
    setFlag(Flag::ZERO, m_acc == byte);
    setFlag(Flag::NEGATIVE, res & 0x80);
}

void CPU::CPX(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    uint8_t res = m_regX - byte;
    setFlag(Flag::CARRY, m_regX >= byte);
    setFlag(Flag::ZERO, m_regX == byte);
    setFlag(Flag::NEGATIVE, res & 0x80);
}

void CPU::CPY(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    uint8_t res = m_regY - byte;
    setFlag(Flag::CARRY, m_regY >= byte);
    setFlag(Flag::ZERO, m_regY == byte);
    setFlag(Flag::NEGATIVE, res & 0x80);
}

void CPU::DEC(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    byte--;
    setFlag(Flag::ZERO, byte == 0);
    setFlag(Flag::NEGATIVE, byte & 0x80);
    memoryWrite(addr, byte);
}

void CPU::DEX(AddressingMode mode)
{
    (void)mode;
    m_regX--;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, m_regX & 0x80);
}

void CPU::DEY(AddressingMode mode)
{
    (void)mode;
    m_regY--;
    setFlag(Flag::ZERO, m_regY == 0);
    setFlag(Flag::NEGATIVE, m_regY & 0x80);
}

void CPU::EOR(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);
    m_acc ^= byte;

    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, m_acc & 0x80);
}

void CPU::INC(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    byte++;
    setFlag(Flag::ZERO, byte == 0);
    setFlag(Flag::NEGATIVE, byte & 0x80);
    memoryWrite(addr, byte);
}

void CPU::INX(AddressingMode mode)
{
    (void)mode;
    m_regX++;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::INY(AddressingMode mode)
{
    (void)mode;
    m_regY++;
    setFlag(Flag::ZERO, m_regY == 0);
    setFlag(Flag::NEGATIVE, (m_regY >> 7));
}

void CPU::JMP(AddressingMode mode)
{
    m_pc = addressFromMode(mode);
}

void CPU::JSR(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    m_pc--;
    stackPush(m_pc >> 8);
    stackPush(m_pc & 0xFF);
    m_pc = addr;
}

void CPU::LDA(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);

    m_acc = memoryRead(addr);
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, (m_acc >> 7));
}

void CPU::LDX(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    m_regX = byte;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, m_regX & 0x80);
}

void CPU::LDY(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    m_regY = byte;
    setFlag(Flag::ZERO, m_regY == 0);
    setFlag(Flag::NEGATIVE, m_regY >> 7);
}

void CPU::LSR(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = (mode == AddressingMode::Implicit) ? m_acc : memoryRead(addr);

    setFlag(Flag::CARRY, byte & 0x01);
    uint8_t result = byte >> 1;
    setFlag(Flag::ZERO, (result & 0x00FF) == 0x0000);
    setFlag(Flag::NEGATIVE, result & 0x80);

    if (mode == AddressingMode::Implicit)
        m_acc = result & 0x00FF;
    else
        memoryWrite(addr, result & 0x00FF);
}

void CPU::NOP(AddressingMode mode)
{
    (void)mode;
}

void CPU::ORA(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = memoryRead(addr);

    m_acc |= byte;
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, m_acc >> 7);
}

void CPU::PHA(AddressingMode mode)
{
    (void)mode;
    stackPush(m_acc);
}

void CPU::PHP(AddressingMode mode)
{
    (void)mode;
    stackPush(m_procStatus);
}

void CPU::PLA(AddressingMode mode)
{
    (void)mode;
    m_acc = stackPull();
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, m_acc >> 7);
}

void CPU::PLP(AddressingMode mode)
{
    (void)mode;
    m_procStatus = stackPull();
}

void CPU::ROL(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = (mode == AddressingMode::Implicit) ? m_acc : memoryRead(addr);
    uint8_t carry = isFLagSet(Flag::CARRY);

    setFlag(Flag::CARRY, byte & 0x80);
    byte = (byte << 1) | carry;
    setFlag(Flag::ZERO, byte == 0);
    setFlag(Flag::NEGATIVE, byte & 0x80);

    if (mode == AddressingMode::Implicit)
        m_acc = byte;
    else
        memoryWrite(addr, byte);
}

void CPU::ROR(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t byte = (mode == AddressingMode::Implicit) ? m_acc : memoryRead(addr);
    uint8_t carry = isFLagSet(Flag::CARRY);

    setFlag(Flag::CARRY, byte & 0x01);
    byte = (byte >> 1) | (carry << 7);
    setFlag(Flag::ZERO, byte == 0);
    setFlag(Flag::NEGATIVE, byte & 0x80);

    if (mode == AddressingMode::Implicit)
        m_acc = byte;
    else
        memoryWrite(addr, byte);
}

void CPU::RTI(AddressingMode mode)
{
    (void)mode;
    m_procStatus = stackPull();
    m_pc = stackPull();
    m_pc |= ((uint16_t)stackPull()) << 8;
}

void CPU::RTS(AddressingMode mode)
{
    (void)mode;
    m_pc = (uint16_t)stackPull();
    m_pc |= (uint16_t)stackPull() << 8;
    m_pc++;
}

void CPU::SBC(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint16_t data = memoryRead(addr) ^ 0x00FF;
    uint16_t result = m_acc + data + (isFLagSet(Flag::CARRY) ? 1 : 0);

    setFlag(Flag::CARRY, result & 0xFF00);
    setFlag(Flag::ZERO, (result & 0x00FF) == 0);
    setFlag(Flag::OVERFLOW, (~(m_acc ^ data) & (m_acc ^ result)) & 0x80);
    setFlag(Flag::NEGATIVE, result & 0x80);
    m_acc = result & 0x00FF;
}

void CPU::SEC(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::CARRY, true);
}

void CPU::SED(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::DECMODE_DISABLE, true);
}

void CPU::SEI(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::INT_DISABLE, true);
}

void CPU::STA(AddressingMode mode)
{
    memoryWrite(addressFromMode(mode), m_acc);
}

void CPU::STX(AddressingMode mode)
{
    memoryWrite(addressFromMode(mode), m_regX);
}

void CPU::STY(AddressingMode mode)
{
    memoryWrite(addressFromMode(mode), m_regY);
}

void CPU::TAX(AddressingMode mode)
{
    (void)mode;
    m_regX = m_acc;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::TAY(AddressingMode mode)
{
    (void)mode;
    m_regY = m_acc;
    setFlag(Flag::ZERO, m_regY == 0);
    setFlag(Flag::NEGATIVE, (m_regY >> 7));
}

void CPU::TSX(AddressingMode mode)
{
    (void)mode;
    m_regX = m_sp;
    setFlag(Flag::ZERO, m_regX == 0);
    setFlag(Flag::NEGATIVE, (m_regX >> 7));
}

void CPU::TXA(AddressingMode mode)
{
    (void)mode;
    m_acc = m_regX;
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, (m_acc >> 7));
}

void CPU::TXS(AddressingMode mode)
{
    (void)mode;
    m_sp = m_regX;
}

void CPU::TYA(AddressingMode mode)
{
    (void)mode;
    m_acc = m_regY;
    setFlag(Flag::ZERO, m_acc == 0);
    setFlag(Flag::NEGATIVE, (m_acc >> 7));
}

uint8_t CPU::memoryRead(uint16_t addr)
{
    return m_bus->cpuRead(addr);
}

uint16_t CPU::memoryReadAddress(uint16_t addr)
{
    uint8_t highAddr = memoryRead(addr + 1);
    uint8_t lowAddr = memoryRead(addr);
    return (highAddr << 8) | lowAddr;
}

void CPU::memoryWrite(uint16_t addr, uint8_t data)
{
    m_bus->cpuWrite(addr, data);
}

void CPU::logInstruction(uint8_t opcode, AddressingMode mode)
{
    printf("%02X ", opcode);
    switch (mode)
    {
        case AddressingMode::Implicit:
            printf("       ");
            break;
        case AddressingMode::Absolute:
        case AddressingMode::AbsoluteX:
        case AddressingMode::AbsoluteY:
            printf("%02X %02X  ", memoryRead(m_pc), memoryRead(m_pc + 1));
            break;
        case AddressingMode::Immediate:
        case AddressingMode::ZeroPage:
        case AddressingMode::ZeroPageX:
        case AddressingMode::ZeroPageY:
        case AddressingMode::IndirectIndexed:
        case AddressingMode::IndexedIndirect:
        case AddressingMode::Relative:
            printf("%02X     ", memoryRead(m_pc));
            break;
        default:
            break;
    }
}

void CPU::logDisassembly(uint8_t opcode, AddressingMode mode)
{
    switch (mode)
    {
        case AddressingMode::Implicit:
            printf("                             ");
            break;
        case AddressingMode::Immediate:
            printf(" #$%02X                        ", memoryRead(m_pc));
            break;
        case AddressingMode::Absolute:
        case AddressingMode::AbsoluteX:
        case AddressingMode::AbsoluteY:
            printf(" $%04X                       ", (memoryRead(m_pc) | ((uint16_t)memoryRead(m_pc + 1)) << 8));
            break;
        case AddressingMode::Relative:
            printf(" $%04X                       ", m_pc + 1 + (int8_t)memoryRead(m_pc));
            break;
        case AddressingMode::ZeroPage:
        case AddressingMode::ZeroPageX:
        case AddressingMode::ZeroPageY:
            printf(" $%02X = %02X                    ", memoryRead(m_pc), memoryRead(adrFromMode));
            break;
        case AddressingMode::IndirectIndexed:
        case AddressingMode::IndexedIndirect:
            printf(" $%02X                         ", memoryRead(m_pc));
            break;
        default:
            break;
    }
}

std::string formatRegisters(uint8_t m_acc, uint8_t m_regX, uint8_t m_regY, uint8_t m_procStatus, uint8_t m_sp)
{
    std::ostringstream oss;
    oss << "A:" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << static_cast<int>(m_acc) << " "
        << "X:" << std::setw(2) << static_cast<int>(m_regX) << " "
        << "Y:" << std::setw(2) << static_cast<int>(m_regY) << " "
        << "P:" << std::setw(2) << static_cast<int>(m_procStatus) << " "
        << "SP:" << std::setw(2) << static_cast<int>(m_sp) << "\n";
    return oss.str();
}

void CPU::tick()
{
    if (m_currentCycles <= 0)
    {
        uint8_t opcode = fetchByte();

        m_procStatus |= Flag::UNUSED | Flag::INT_DISABLE;
        if (!m_opcodeLookup.contains(opcode))
        {
            printf("Illegal opcode (%02x)\n", opcode);
            return;
        }
        auto instruction = m_opcodeLookup[opcode];
        m_currentCycles = instruction.cycles;
        printf("%04X  ", m_pc - 1);
        logInstruction(opcode, instruction.addrMode);
        printf("%s", instruction.name.c_str());
        logDisassembly(opcode, instruction.addrMode);
        std::string registerString = formatRegisters(m_acc, m_regX, m_regY, m_procStatus, m_sp);
        printf("%s", registerString.c_str());
        (this->*instruction.operation)(instruction.addrMode);
    }
    m_currentCycles--;
    m_totalCycles++;
}

void CPU::connectBus(Bus *bus)
{
    m_bus = bus;
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
    uint8_t byte = memoryRead(0x0100 | ++m_sp);
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
    if ((m_pc & 0xFF00) != (addr & 0xFF00))
        m_currentCycles++;
    m_pc += 2;
    return addr;
}

uint16_t CPU::AbsoluteIndexedYMode()
{
    uint16_t addr = memoryReadAddress(m_pc) + m_regY;
    if ((m_pc & 0xFF00) != (addr & 0xFF00))
        m_currentCycles++;
    m_pc += 2;
    return addr;
}

uint16_t CPU::ZeroPageMode()
{
    adrFromMode = memoryRead(m_pc);
    uint16_t addr = memoryRead(m_pc++);
    addr &= 0x00FF;
    return addr;
}

uint16_t CPU::ZeroPageXMode()
{
    adrFromMode = memoryRead(m_pc);
    uint16_t addr = memoryRead(m_pc++) + m_regX;
    addr &= 0x00FF;
    return addr;
}

uint16_t CPU::ZeroPageYMode()
{
    adrFromMode = memoryRead(m_pc);
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
    uint16_t addrPtr = memoryRead(m_pc);
    uint16_t lsb = memoryRead(addrPtr & 0xFF) + m_regY;
    uint16_t msb = memoryRead(addrPtr + 1) << 8;

    if ((m_pc & 0xFF00) != (addrPtr & 0xFF00))
        m_currentCycles++;
    m_pc++;
    return msb | lsb;
}

uint16_t CPU::RelativeMode()
{
    return memoryRead(m_pc++);
}

void CPU::reset()
{
    m_acc = 0;
    m_regX = 0;
    m_regY = 0;
    m_sp = 0xFD;
    m_procStatus = 0;
    m_currentCycles = 7;
    m_totalCycles = 0;
    m_pc = memoryReadAddress(0xFFFC);
}

void CPU::irq()
{
}

void CPU::nmi()
{
}

void CPU::run()
{
    do
    {
        tick();
    } while (true);
}

void CPU::setFlag(Flag flag, bool on)
{
    if (on)
        m_procStatus |= flag;
    else
        m_procStatus &= ~flag;
}

bool CPU::isFLagSet(Flag flag)
{
    return m_procStatus & flag;
}
