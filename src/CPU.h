/*
** EPITECH PROJECT, 2024
** prototype
** File description:
** CPU
*/

#ifndef CPU_H_
#define CPU_H_

#include <cstdint>

class CPU
{
    public:
        CPU();
        ~CPU();

        enum Flag
        {
            FLAG_CARRY = 0x0,
            FLAG_ZERO = 0x1,
            FLAG_INT_DISABLE = 0x2,
            FLAG_DECMODE_DISABLE = 0x4,
            FLAG_B = 0x8,
            FLAG_OVERFLOW = 0x40,
            FLAG_NEGATIVE = 0x80
        };

        void load(std::vector<uint8_t> &program);
        void run();

    private:
        void lda(uint8_t byte);
        void tax();
        void inx();
        void brk();

        void setFlag(Flag flag);
        void unsetFlag(Flag flag);

        uint8_t mem_read(uint16_t addr);
        void mem_write(uint16_t addr, uint8_t data);

    private:
        uint16_t m_pc; // Program Counter
        uint8_t m_sp; // Stack Pointer
        uint8_t m_acc; // Accumulator
        uint8_t m_regX; // Register X
        uint8_t m_regY; // Register Y
        uint8_t m_procStatus; // Processor Status
        uint8_t m_mem[0x10000]; // Memory
};

#endif /* !CPU_H_ */
