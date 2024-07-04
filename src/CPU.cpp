/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** CPU
*/
#include "CPU.h"

#include <iostream>

#define PAGE_CROSS(x, y) ((x & 0xFF00) != (y & 0xFF00))

CPU::CPU() :
    m_pc(0),
    m_sp(0),
    m_acc(0),
    m_regX(0),
    m_regY(0),
    m_procStatus(0)
{
    m_opcodeLookup[0x69] = (Instruction){"ADC", AddressingMode::Immediate, &CPU::ADC, 2};
    m_opcodeLookup[0x65] = (Instruction){"ADC", AddressingMode::ZeroPage, &CPU::ADC, 3};
    m_opcodeLookup[0x75] = (Instruction){"ADC", AddressingMode::ZeroPageX, &CPU::ADC, 4};
    m_opcodeLookup[0x6D] = (Instruction){"ADC", AddressingMode::Absolute, &CPU::ADC, 4};
    m_opcodeLookup[0x7D] = (Instruction){"ADC", AddressingMode::AbsoluteX, &CPU::ADC, 4};
    m_opcodeLookup[0x79] = (Instruction){"ADC", AddressingMode::AbsoluteY, &CPU::ADC, 4};
    m_opcodeLookup[0x61] = (Instruction){"ADC", AddressingMode::IndexedIndirect, &CPU::ADC, 6};
    m_opcodeLookup[0x71] = (Instruction){"ADC", AddressingMode::IndirectIndexed, &CPU::ADC, 5};

    m_opcodeLookup[0xA9] = (Instruction){"LDA", AddressingMode::Immediate, &CPU::LDA, 2};
    m_opcodeLookup[0xA5] = (Instruction){"LDA", AddressingMode::ZeroPage, &CPU::LDA, 3};
    m_opcodeLookup[0xB5] = (Instruction){"LDA", AddressingMode::ZeroPageX, &CPU::LDA, 4};
    m_opcodeLookup[0xAD] = (Instruction){"LDA", AddressingMode::Absolute, &CPU::LDA, 4};
    m_opcodeLookup[0xBD] = (Instruction){"LDA", AddressingMode::AbsoluteX, &CPU::LDA, 4};
    m_opcodeLookup[0xB9] = (Instruction){"LDA", AddressingMode::AbsoluteY, &CPU::LDA, 4};
    m_opcodeLookup[0xA1] = (Instruction){"LDA", AddressingMode::IndexedIndirect, &CPU::LDA, 6};
    m_opcodeLookup[0xB1] = (Instruction){"LDA", AddressingMode::IndirectIndexed, &CPU::LDA, 5};

    m_opcodeLookup[0xEA] = (Instruction){"NOP", AddressingMode::Implicit, &CPU::NOP, 2};
    m_opcodeLookup[0xE8] = (Instruction){"INX", AddressingMode::Implicit, &CPU::INX, 2};
    m_opcodeLookup[0x00] = (Instruction){"BRK", AddressingMode::Implicit, &CPU::BRK, 7};

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

void CPU::LDA(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);

    m_acc = memoryRead(addr);
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
    uint8_t byte = 0;

    if (mode == AddressingMode::Implicit)
        byte = m_acc;
    setFlag(Flag::CARRY, byte & 0x80);
    setFlag(Flag::ZERO, byte == 0);
    setFlag(Flag::NEGATIVE, byte & 0x80);
    byte <<= 1;
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
    setFlag(Flag::OVERFLOW, res & 0x40);
    setFlag(Flag::NEGATIVE, res & 0x80);
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

void CPU::BRK(AddressingMode mode)
{
    (void)mode;
    setFlag(Flag::B, true);
}

void CPU::SBC(AddressingMode mode)
{
    uint16_t addr = addressFromMode(mode);
    uint8_t data = memoryRead(addr);
    uint16_t result = m_acc - data - (m_procStatus & Flag::CARRY);

    setFlag(Flag::CARRY, result < 0x100);
    setFlag(Flag::ZERO, (result & 0xFF) == 0);
    setFlag(Flag::OVERFLOW, (~(m_acc ^ data) & (m_acc ^ result)) & 0x0080);
    setFlag(Flag::NEGATIVE, (result & 0x80));
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
        m_currentCycles = instruction.cycles;
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

bool CPU::isFLagSet(Flag flag)
{
    return m_procStatus & flag;
}
