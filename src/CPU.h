/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** CPU
*/

#ifndef CPU_H_
#define CPU_H_

#include <cstdint>
#include <vector>
#include <string>
#include <map>

class Bus;

enum Flag
{
    CARRY = 0x1,
    ZERO = 0x2,
    INT_DISABLE = 0x4,
    DECMODE_DISABLE = 0x8,
    B = 0x10,
    UNUSED = 0x20,
    OVERFLOW = 0x40,
    NEGATIVE = 0x80
};

enum class AddressingMode
{
    Implicit,
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Relative,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndexedIndirect,
    IndirectIndexed
};

class CPU
{
    public:
        CPU();
        ~CPU();

        void run();
        void reset();
        void irq();
        void nmi();

        uint8_t memoryRead(uint16_t addr);
        void memoryWrite(uint16_t addr, uint8_t data);

        void tick();

        void connectBus(Bus *bus);

    private:
        void setFlag(Flag flag, bool on);
        bool isFLagSet(Flag flag);

        uint16_t memoryReadAddress(uint16_t addr);
        uint8_t fetchByte();

        uint16_t addressFromMode(AddressingMode mode);

        void stackPush(uint8_t byte);
        uint8_t stackPull();

        //Addressing Modes
        uint16_t ImmediateMode();
        uint16_t AbsoluteMode();
        uint16_t AbsoluteIndexedXMode();
        uint16_t AbsoluteIndexedYMode();
        uint16_t ZeroPageMode();
        uint16_t ZeroPageXMode();
        uint16_t ZeroPageYMode();
        uint16_t IndirectMode();
        uint16_t IndexedIndirectMode();
        uint16_t IndirectIndexedMode();
        uint16_t RelativeMode();

        struct Instruction
        {
            std::string name;
            AddressingMode addrMode;
            void (CPU::*operation)(AddressingMode);
            uint8_t cycles;
        };

        // Instructions
        void ADC(AddressingMode mode);
        void AND(AddressingMode mode);
        void ASL(AddressingMode mode);
        void BCC(AddressingMode mode);
        void BCS(AddressingMode mode);
        void BEQ(AddressingMode mode);
        void BIT(AddressingMode mode);
        void BMI(AddressingMode mode);
        void BNE(AddressingMode mode);
        void BPL(AddressingMode mode);
        void BRK(AddressingMode mode);
        void BVC(AddressingMode mode);
        void BVS(AddressingMode mode);
        void CLC(AddressingMode mode);
        void CLD(AddressingMode mode);
        void CLI(AddressingMode mode);
        void CLV(AddressingMode mode);
        void CMP(AddressingMode mode);
        void CPX(AddressingMode mode);
        void CPY(AddressingMode mode);
        void DEC(AddressingMode mode);
        void DEX(AddressingMode mode);
        void DEY(AddressingMode mode);
        void EOR(AddressingMode mode);
        void INC(AddressingMode mode);
        void INX(AddressingMode mode);
        void INY(AddressingMode mode);
        void JMP(AddressingMode mode);
        void JSR(AddressingMode mode);
        void LDA(AddressingMode mode);
        void LDX(AddressingMode mode);
        void LDY(AddressingMode mode);
        void LSR(AddressingMode mode);
        void NOP(AddressingMode mode);
        void ORA(AddressingMode mode);
        void PHA(AddressingMode mode);
        void PHP(AddressingMode mode);
        void PLA(AddressingMode mode);
        void PLP(AddressingMode mode);
        void ROL(AddressingMode mode);
        void ROR(AddressingMode mode);
        void RTI(AddressingMode mode);
        void RTS(AddressingMode mode);
        void SBC(AddressingMode mode);
        void SEC(AddressingMode mode);
        void SED(AddressingMode mode);
        void SEI(AddressingMode mode);
        void STA(AddressingMode mode);
        void STX(AddressingMode mode);
        void STY(AddressingMode mode);
        void TAX(AddressingMode mode);
        void TAY(AddressingMode mode);
        void TSX(AddressingMode mode);
        void TXA(AddressingMode mode);
        void TXS(AddressingMode mode);
        void TYA(AddressingMode mode);

        void logInstruction(uint8_t opcode, AddressingMode mode);
        void logDisassembly(uint8_t opcode, AddressingMode mode);
        void disZeroPage(AddressingMode mode);
    private:
        uint16_t m_pc; // Program Counter
        uint8_t m_sp; // Stack Pointer
        uint8_t m_acc; // Accumulator
        uint8_t m_regX; // Register X
        uint8_t m_regY; // Register Y
        uint8_t m_procStatus; // Processor Status

        uint16_t adrFromMode;

        std::map<uint8_t, Instruction> m_opcodeLookup;

        uint8_t m_currentCycles;
        uint64_t m_totalCycles;

        Bus* m_bus;
};

#endif /* !CPU_H_ */
