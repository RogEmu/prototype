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

        void loadMemory(const std::vector<uint8_t> &program);
        void run();
        void reset();

        uint8_t memoryRead(uint16_t addr);
        void memoryWrite(uint16_t addr, uint8_t data);

    private:
        void setFlag(Flag flag, bool on);

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
        void ADC();
        // void AND();
        // void ASL();
        // void BCC();
        // void BCS();
        // void BEQ();
        // void BIT();
        // void BMI();
        // void BNE();
        // void BPL();
        void BRK(AddressingMode mode);
        // void BVC();
        // void BVS();
        // void CLC();
        // void CLD();
        // void CLI();
        // void CLV();
        // void CMP();
        // void CPX();
        // void CPY();
        // void DEC();
        // void DEX();
        // void DEY();
        // void EOR();
        // void INC();
        void INX(AddressingMode mode);
        // void INY();
        // void JMP();
        // void JSR();
        void LDA(AddressingMode mode);
        // void LDX();
        // void LDY();
        // void LSR();
        void NOP(AddressingMode mode);
        // void ORA();
        // void PHA();
        // void PHP();
        // void PLA();
        // void PLP();
        // void ROL();
        // void ROR();
        // void RTI();
        // void RTS();
        // void SBC();
        // void SEC();
        // void SED();
        // void SEI();
        // void STA();
        // void STX();
        // void STY();
        void TAX(AddressingMode mode);
        // void TAY();
        // void TSX();
        // void TXA();
        // void TXS();
        // void TYA();

    private:
        uint16_t m_pc; // Program Counter
        uint8_t m_sp; // Stack Pointer
        uint8_t m_acc; // Accumulator
        uint8_t m_regX; // Register X
        uint8_t m_regY; // Register Y
        uint8_t m_procStatus; // Processor Status

        uint8_t m_mem[0x10000]; // Memory

        std::map<uint8_t, Instruction> m_opcodeLookup;
};

#endif /* !CPU_H_ */
