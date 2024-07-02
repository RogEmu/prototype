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

    private:
        void lda(uint8_t byte);

        void setFlag(Flag flag);
        void unsetFlag(Flag flag);

    private:
        uint16_t m_pc;
        uint8_t m_sp;
        uint8_t m_acc;
        uint8_t m_regX;
        uint8_t m_regY;
        uint8_t m_procStatus;
};

#endif /* !CPU_H_ */
