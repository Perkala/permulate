#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>

enum
{
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,
    R_COND,
    R_COUNT
};

enum
{
    OP_BR = 0,  /* BRANCH */
    OP_ADD,     /* ADD */
    OP_LD,      /* LOAD */
    OP_ST,      /* STORE */
    OP_JSR,     /* JUMP REGISTER */
    OP_AND,     /* BITWISE AND */
    OP_LDR,     /* LOAD REGISTER */
    OP_STR,     /* STORE REGISTER */
    OP_RTI,     /* UNUSED */
    OP_NOT,     /* BITWISE NOT */
    OP_LDI,     /* LOAD INDIRECT */
    OP_STI,     /* STORE INDIRECT */
    OP_JMP,     /* JUMP */
    OP_RES,     /* RESERVERED (UNUSED) */
    OP_LEA,     /* LOAD EFFECTIVE ADDRESS */
    OP_TRAP     /* EXECUTE TRAP */ 
};

enum
{
    FL_POS = 1 << 0,    /* POSITIVE */
    FL_ZRO = 1 << 1,    /* ZERO */
    FL_NEG = 1 << 2     /* NEGATIVE */ 
};

enum
{
    TRAP_GETC = 0x20,  /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT = 0x21,   /* output a character */
    TRAP_PUTS = 0x22,  /* output a word string */
    TRAP_IN = 0x23,    /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT = 0x25   /* halt the program */
};

/* RESERVED MEMORY MAPPED KEYBOARD REGISTERS */
enum
{
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02  /* keyboard data */
};

struct termios original_tio;
uint16_t memory[UINT16_MAX];
uint16_t reg[R_COUNT];

void mem_write(uint16_t address, uint16_t val);
uint16_t mem_read(uint16_t address);

void ADD(uint16_t instruction);
void AND(uint16_t instruction);
void NOT(uint16_t instruction);
void BR(uint16_t instruction);
void JMP(uint16_t instruction);
void JSR(uint16_t instruction);
void LD(uint16_t instruction);
void LDI(uint16_t instruction);
void LDR(uint16_t instruction);
void LEA(uint16_t instruction);
void ST(uint16_t instruction);
void STI(uint16_t instruction);
void STR(uint16_t instruction);

void T_GETC();
void T_OUT();
void T_PUTS();
void T_IN();
void T_PUTSP();
void T_HALT();

uint16_t swap16(uint16_t x);
void read_image_file(FILE *file);
int read_image(const char *image_path);

uint16_t check_key();
void disable_input_buffering();
void restore_input_buffering();
void handle_interrupt();

int running_flag = 1;

int main(int argc, const char* argv[])
{
    if (argc < 2)
    {
        printf("./permulate [image-file1] ...\n");
        exit(2);
    }
    for(int i = 1; i < argc; ++i)
    {
        if (!read_image(argv[i]))
        {
            printf("Failed to load image: %s\n", argv[i]);
            exit(1);
        }
    }
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    /* set the PC register to starting position */
    /* 0x3000 is the default */
    enum { PC_START = 0x3000 };
    reg[R_PC] = PC_START;

    while(running_flag)
    {
        uint16_t instr = mem_read(reg[R_PC]++);
        /* left 4 bits of an instructions represents an operation code */
        uint16_t op_code = instr >> 12;
        uint16_t trap_code = instr & 0xFF;

        switch(op_code)
        {
            case OP_ADD:
                ADD(instr);
                break;
            case OP_AND:
                AND(instr);
                break;
            case OP_NOT:
                NOT(instr);
                break;
            case OP_BR:
                BR(instr);
                break;
            case OP_JMP:
                JMP(instr);
                break;
            case OP_JSR:
                JSR(instr);
                break;
            case OP_LD:
                LD(instr);
                break;
            case OP_LDI:
                LDI(instr);
                break;
            case OP_LDR:
                LDR(instr);
                break;
            case OP_LEA:
                LEA(instr);
                break;
            case OP_ST:
                ST(instr);
                break;
            case OP_STI:
                STI(instr);
                break;
            case OP_STR:
                STR(instr);
                break;
            case OP_TRAP:
                switch (trap_code)
                {
                    case TRAP_GETC:
                        T_GETC();
                        break;
                    case TRAP_OUT:
                        T_OUT();
                        break;
                    case TRAP_PUTS:
                        T_PUTS();
                        break;
                    case TRAP_IN:
                        T_IN();
                        break;
                    case TRAP_PUTSP:
                        T_PUTSP();
                        break;
                    case TRAP_HALT:
                        T_HALT();
                        break;
                }
                break;
            case OP_RES:
                /* not implemented */
            case OP_RTI:
                /* not implemented */
            default:
                abort();
                break;
        }
    }
    restore_input_buffering();
}

/* FLAGs MANIPULATION */
void update_flags(uint16_t r)
{
    if(reg[r] == 0)
    {
        reg[R_COND] = FL_ZRO;
    }
    else if(reg[r] >> 15)
    {
        reg[R_COND] = FL_NEG;
    }
    else
    {
        reg[R_COND] = FL_POS;
    }
}

/* INSTRUCTIONS IMPLEMENTATIONS */
/////////////////////////////////
////////////////////////////////
///////////////////////////////

uint16_t sign_extend(uint16_t x, int bit_count)
{
    if ((x >> (bit_count - 1)) & 1) {
        x |= (0xFFFF << bit_count);
    }
    return x;
}

/* ADD */
void ADD(uint16_t instruction)
{
    /* destination register (DR), observing bits 11-9, masking with 111 to read only 3 relevant bits after shifting */
    uint16_t r0 = (instruction >> 9) & 0x7;
    /* first operand (SR1), observing bits 8-6, masking with 1111 to read only 3 relevant bits after shifting */
    uint16_t r1 = (instruction >> 6) & 0x7;
    /* operation mode, immediate or not (1 or 0) */
    uint16_t imm_flag = (instruction >> 5) & 0x1;
    if (imm_flag)
    {
        uint16_t imm5 = sign_extend(instruction & 0x1f, 5);
        reg[r0] = reg[r1] + imm5;
    }
    else
    {
        uint16_t r2 = instruction & 0x7;
        reg[r0] = reg[r1] + reg[r2];
    }
    update_flags(r0);
}

/* BITWISE AND */
void AND(uint16_t instruction)
{
    /* destination register (DR) */
    uint16_t r0 = (instruction >> 9) & 0x7;
    /* first operand (SR1) */
    uint16_t r1 = (instruction >> 6) & 0x7;
    /* operation mode, immediate or not (1 or 0) */
    uint16_t imm_flag = (instruction >> 5) & 0x1;
    if(imm_flag)
    {
        uint16_t imm5 = sign_extend(instruction & 0x1f, 5);
        reg[r0] = reg[r1] & imm5;
    }
    else
    {
        uint16_t r2 = instruction & 0x7;
        reg[r0] = reg[r1] & reg[r2];
    }
    update_flags(r0);
}

/* BITWISE NOT */
void NOT(uint16_t instruction)
{
    /* destination register (DR) */
    uint16_t r0 = (instruction >> 9) & 0x7;
    /* operand (SR1) */
    uint16_t r1 = (instruction >> 6) & 0x7;
    reg[r0] = ~reg[r1];
    update_flags(r0);
}

/* BRANCH */
void BR(uint16_t instruction)
{
    uint16_t cond_flag = (instruction >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instruction & 0x1ff, 9);
    if(cond_flag & reg[R_COND])
    {
        reg[R_PC] += pc_offset;
    }
}

/* JUMP */
void JMP(uint16_t instruction)
{
    uint16_t r1 = (instruction >> 6) & 0x7;
    reg[R_PC] = reg[r1];
}

/* JUMP TO SUBROUTINE */
void JSR(uint16_t instruction)
{
    uint16_t r1 = (instruction >> 6) & 0x7;
    uint16_t long_pc_offset = sign_extend(instruction & 0x7ff, 11);
    uint16_t long_flag = (instruction >> 11) & 1;

    reg[R_R7] = reg[R_PC];
    if (long_flag)
    {
        reg[R_PC] += long_pc_offset;  /* JSR */
    }
    else
    {
        reg[R_PC] = reg[r1]; /* JSRR */
    }
}

/* LOAD */
void LD(uint16_t instruction)
{
    /* destination register (DR) */
    uint16_t r0 = (instruction >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instruction & 0x1ff, 9);

    reg[r0] = mem_read(reg[R_PC] + pc_offset);
    update_flags(r0);
}

/* LOAD INDIRECT */
void LDI(uint16_t instruction)
{
    /* destination register (DR) */
    uint16_t r0 = (instruction >> 9) & 0x7;
    /* PCOffset 9, sign-extending bits [8:0] to 16 bits and adding to incremented PC */
    uint16_t pc_offset = sign_extend(instruction & 0x1ff, 9);
    reg[r0] = mem_read(mem_read(reg[R_PC] + pc_offset));
    update_flags(r0);
}

/* LOAD REGISTER */
void LDR(uint16_t instruction)
{
    /* destination register (DR) */
    uint16_t r0 = (instruction >> 9) & 0x7;
    uint16_t r1 = (instruction >> 6) & 0x7;
    uint16_t offset = sign_extend(instruction & 0x3f, 6);
    reg[r0] = mem_read(reg[r1] + offset);
    update_flags(r0);
}

/* LOAD EFFECTIVE ADRESS */
void LEA(uint16_t instruction)
{
    /* destination register (DR) */
    uint16_t r0 = (instruction >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instruction & 0x1ff, 9);

    reg[r0] = reg[R_PC] + pc_offset;
    update_flags(r0);
}

/* STORE */
void ST(uint16_t instruction)
{
    uint16_t r0 = (instruction >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instruction & 0x1ff, 9);
    mem_write(reg[R_PC] + pc_offset, reg[r0]);
}

/* STORE INDIRECT */
void STI(uint16_t instruction)
{
    uint16_t r0 = (instruction >> 9) & 0x7;
    uint16_t pc_offset = sign_extend(instruction & 0x1ff, 9);
    mem_write(mem_read(reg[R_PC] + pc_offset), reg[r0]);
}

/* STORE REGISTER */
void STR(uint16_t instruction)
{
    uint16_t r0 = (instruction >> 9) & 0x7;
    uint16_t r1 = (instruction >> 6) & 0x7;
    uint16_t offset = sign_extend(instruction & 0x3f, 6);
    mem_write(reg[r1] + offset, reg[r0]);
}

/* TRAP ROUTINES */
//////////////////
/////////////////
////////////////

/* READ A SINGLE ASCII CHAR */
void T_GETC()
{
    reg[R_R0] = (uint16_t)getchar();
}

/* OUTPUT CHARACTER */
void T_OUT()
{
    putc((char)reg[R_R0], stdout);
    fflush(stdout);
}

/* OUTPUT WORD STRING */
void T_PUTS()
{
    uint16_t *c = memory + reg[R_R0];
    while(*c)
    {
        putc((char)*c,stdout);
        ++c;
    }
    fflush(stdout);
}

/* PROMPT FOR INPUT CHARACTER */
void T_IN()
{
    printf("Enter a character: ");
    char c = getchar();
    putc(c, stdout);
    reg[R_R0] = (uint16_t)c;
}

/* OUTPUT BYTE STRING */
void T_PUTSP()
{
    uint16_t *c = memory + reg[R_R0];
    while(*c)
    {
        char char1 = (*c) & 0xff;
        putc(char1, stdout);
        char char2 = (*c) >> 8;
        if (char2)
        {
            putc(char2, stdout);
        }
        ++c;
        fflush(stdout); 
    }
}

/* HALT PROGRAM */
void T_HALT()
{
    puts("HALT");
    fflush(stdout);
    running_flag = 0;
}

/* BIG-ENDIAN to LITTLE-ENDIAN */
uint16_t swap16(uint16_t x)
{
    return (x << 8) | (x >> 8);
}

/* READ LC-3 PROGRAM INTO MEMORY */
//////////////////////////////////
/////////////////////////////////
////////////////////////////////

void read_image_file(FILE *file)
{
    /* the origin tells us where in memory to place the image */
    uint16_t origin;
    fread(&origin, sizeof(origin), 1, file);
    origin = swap16(origin);

    /* we know the maximum file size so we only need one fread */
    uint16_t max_read = UINT16_MAX - origin;
    uint16_t* p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    /* swap to little endian */
    while (read-- > 0)
    {
        *p = swap16(*p);
        ++p;
    }
}

/* TAKES A PATH AS A STRING */
int read_image(const char *image_path)
{
    FILE* file = fopen(image_path, "rb");
    if (!file)
    {
        return 0;
    }
    read_image_file(file);
    fclose(file);
    return 1;
}

/* MEMORY OPERATIONS */
//////////////////////
/////////////////////
////////////////////

void mem_write(uint16_t address, uint16_t val)
{
    memory[address] = val;
}

uint16_t mem_read(uint16_t address)
{
    if(address == MR_KBSR)
    {
        if(check_key())
        {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        }
        else
        {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[address];
}

/* ACCESSING KEYBOARD AND CONFIGURING TERMINAL SETTINGS */
/////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
///////////////////////////////////////////////////////

uint16_t check_key()
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

void disable_input_buffering()
{
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

void handle_interrupt()
{
    restore_input_buffering();
    printf("\n");
    exit(-2);
}