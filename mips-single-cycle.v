`timescale 1ns / 1ps



`define CTL_WIDTH 31:0

`define CTL_ALUCONTROL 
`define CTL_ALUCONTROL_UNSIGNED
`define CTL_ALUCONTROL_RIGHT
`define CTL_ALUCONTROL_VARIABLE

`define CTL_BRANCHTYPE


`define CTL_REGDST 
`define CTL_REGWRITE

`define CTL_MEMREAD // useless? no
`define CTL_MEMWRITE
`define CTL_MEM8
`define CTL_MEM16

`define CTL_USEIMM

`define CTL_BRANCH
`define CTL_JUMP
`define CTL_JUMPREG

`define DMEM_MAX_ADDR 16'd512

`define STACK_BEGIN_ADDR 16'd256 
//IO Mapping
`define IN0_ADDR 16'd272
`define IN1_ADDR 
`define IN2_ADDR
`define IN3_ADDR
`define IN4_ADDR
`define IN5_ADDR
`define IN6_ADDR
`define IN7_ADDR

`define OUT0_ADDR 16'd256
`define OUT1_ADDR 
`define OUT2_ADDR
`define OUT3_ADDR
`define OUT4_ADDR
`define OUT5_ADDR
`define OUT6_ADDR
`define OUT7_ADDR


//ALU Control Signal
`define SLL 4'b0000
`define SRL 4'b0001
`define SRA 4'b0010

`define LUI 4'b0011
//`define SRLV 4'b0100
//`define SRAV 4'b0101
`define MUL 4'b0100
//`define MULU 4'b0101
`define ADD 4'b0110
`define ADDU 4'b0111
`define SUB 4'b1000
`define SUBU 4'b1001
`define AND 4'b1010
`define OR 4'b1011
`define XOR 4'b1100
`define NOR 4'b1101

`define SLT 4'b1110
`define SLTU 4'b1111


//7-Segment
`define A 0
`define D 1
`define e 2
`define E 3
`define F 4
`define I 5
`define L 6
`define P 7
`define r 8
`define S 9
`define U 10
`define y 11



//IPcore ROM
//register file combinational or sequential
//case default, if else

/*

Notes:

1. always @ (*) or always @ (A or B) is combinational logic, only = (blocking) assignments should be used in it.

2. reg is not always a flipflop. In combinational "always @ ()" reg is eventually a wire, while in sequential "always", reg can be a flipflop.

3. In an always @ (*) block, each value that is assigned in at least one place must be assigned to a non-trivial value during every ‘execution’ of the always @ (*) block. That's why 'else' should not be omitted after an 'if', otherwise, this will cause a latch to deal with the unassigned situation.
(http://inst.eecs.berkeley.edu/~eecs151/sp19/files/verilog/always_at_blocks.pdf)

4. Initialization

*/

//Reference: http://ee.hawaii.edu/~sasaki/EE361/Fall03/EE361-SingleMIPS.pdf


/*

Func: [31: 26]
Opcode: [5: 0]

RegDst: Select the destination reg, rt(0) or rd(1)
ALUSrc: Select either a register operand (0) or a constant operand (1), a constant operand is used in I-type such as ANDI, SW, LW
PCSrc:
    (0) PC <- PC + 4
    (1) PC <- PC + 4 + (offset * 4) (times 4 for word alignment)
*/

module top (CLK100MHZ, SW, AN, S);
    
    input CLK100MHZ;
    input [15: 0] SW;
    
    output [7: 0] AN;
    output [6: 0] S;
    
    wire clk, rst;
    reg [31: 0] data;
    reg [5:0] state;
    
    parameter login_user = 0, login_pass = 1, login_fail = 2, login_success = 3, play = 2, display_score = 3;
    
    assign rst = SW[0];
    
    clkdiv cd(.mclk(CLK100MHZ), .clk(clk));
    
    MIPSTop mt(clk, rst);
    
    
    wire [7: 0] out = mt.dmem.RAM[`OUT0_ADDR/4];
    wire [31: 0] pc = mt.PC;
    
    wire [3: 0] d0 = out % 10;
    wire [3: 0] d1 = (out / 10) % 10;
    wire [3: 0] d2 = (out / 100) % 10;
    wire [3: 0] d3 = 4'b0;
    wire [3: 0] d4 = pc % 10;
    wire [3: 0] d5 = (pc / 10) % 10;
    wire [3: 0] d6 = (pc / 100) % 10;
    wire [3: 0] d7 = (pc / 1000) % 10;
    
    display dis(CLK100MHZ, AN, S, 
        {d7, d6, d5, d4, d3, d2, d1, d0}, 0);
    
    assign mt.dmem.iwrite = SW[0] & SW[1];
    assign mt.dmem.iaddr = `IN0_ADDR;
    assign mt.dmem.idata = SW[15:4];
endmodule


//op = 0 display 0-f, op = 1 display alphabet
module display (clk100mhz, an, out, data, op);
    
    input clk100mhz;
    input [31: 0] data;
    input op;
    output reg [6:0] out;
    output reg [7:0] an;
    
    wire [3: 0] d0, d1, d2, d3, d4, d5, d6, d7;
    wire clk380hz;
    reg [3:0] current;   
    
    clkdiv cc(.mclk(clk100mhz), .clk380(clk380hz));
    
    assign {d0, d1, d2, d3, d4, d5, d6, d7} = data;
    
    //Current data to display
    always @ ( * )
        case(an)
            8'b01111111: current = d0;
            8'b10111111: current = d1;
            8'b11011111: current = d2;
            8'b11101111: current = d3;
            8'b11110111: current = d4;
            8'b11111011: current = d5;
            8'b11111101: current = d6;
            8'b11111110: current = d7; 
            default:current = 0;
        endcase
        
    //Dynamic Display
    always @ (posedge clk380hz)
        if (an == 8'b11111110) an <= 8'b01111111;
        else an <= {1'b1, an[7: 1]};
    
    //7-Segment Encoding
    always @ ( * )
        if (~op)
            case(current)
            0:out = 7'b0000001;
            1:out = 7'b1001111;
            2:out = 7'b0010010;
            3:out = 7'b0000110;
            4:out = 7'b1001100;
            5:out = 7'b0100100;
            6:out = 7'b0100000;
            7:out = 7'b0001111;
            8:out = 7'b0000000;
            9:out = 7'b0001100;
            10:out = 7'b0001000;
            11:out = 7'b1100000;
            12:out = 7'b0110001;
            13:out = 7'b1000010;
            14:out = 7'b0110000;
            15:out = 7'b0111000;
            default: out = 0;
            endcase
        else 
            case (current)
            `A: out = 7'b0001000;
            `e: out = 7'b0010000;
            `E: out = 7'b0110000;
            `F: out = 7'b0111000;
            `I: out = 7'b1001111;
            `L: out = 7'b1110001;
            `P: out = 7'b0011000;
            `r: out = 7'b0111001;
            `S: out = 7'b0100100;
            `U: out = 7'b1000001;
            `y: out = 7'b1000100;
            default: out = 0;
        endcase
endmodule


module clkdiv(
    input mclk,
    output clk380,
    //output clk6000,
    //output clk190,
    output clk
    );
    reg [27:0]q;
    always @ (posedge mclk)
        q <= q + 1;

    assign clk380 = q[17];//380hz
    //assign clk6000 = q[13];//6000hz 
    //assign clk190 = q[18];//190hz 
    //assign clk1_4hz=q[24];         
    assign clk = q[20];
    //assign clk1_1hz=q[26];         
endmodule


module MIPSTop (clk, rst);

    input clk, rst;
    
    wire [31: 0] ALUResult;
    wire [31: 0] PC, WriteData, ReadData;
    wire [31: 0] Instr;
    wire MemWrite;
    wire MemSign;
    wire [1: 0] MemWidth;
    
    MIPS mips(clk, rst, PC, Instr, MemWrite, MemSign, MemWidth, ALUResult, WriteData, ReadData);
    IMem imem(PC, Instr);
    DMem dmem(clk, ALUResult, MemWrite, WriteData, ReadData, MemSign, MemWidth);

endmodule

module IMem (addr, instr);
    input [31: 0] addr;
    output reg [31: 0] instr;
    
    parameter LH = 6'b100001, LUI = 6'b001111;
    parameter ADDI = 6'b001000, ADDIU = 6'b001001, ORI = 6'b001101, BEQ = 6'b000100, BNE = 6'b000101, BLEZ = 6'b000110, BGTZ = 6'b000111;
    parameter ADD = 6'b100000, ADDU = 6'b100001, OR = 6'b100101, SUB = 6'b100010;
    parameter J = 6'b000010, JAL = 6'b000011;
    parameter JR = 6'b001000, JALR = 6'b001001;
    parameter SB = 6'b101000, SW = 6'b101011, LW = 6'b100011, LB = 6'b100000, LBU = 6'b100100;
   
    /* Registers */
    parameter r0 = 5'b0;
    parameter v0 = 5'b00010, v1 = 5'b00011;//Return values from function
    parameter a0 = 5'b00100, a1 = 5'b00101, a2 = 5'b00110, a3 = 5'b00111;//Arguments to function
    parameter ra = 5'b11111;//Return address register
    parameter t0 = 5'b01000, t1 = 5'b01001, t2 = 5'b01010, t3 = 5'b01011;//Temporary data
    parameter s0 = 5'h10, s1 = 5'h11, s2 = 5'h12, s3 = 5'h13, s4 = 5'h14, s5 = 5'h15, s6 = 5'h16, s7 = 5'h17;//Saved Registers, preserved by subprograms
    parameter sp = 5'b11101;//Stack Pointer
    
    
    parameter HLT = {6'b000100, 10'b0, 16'hffff};
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
   
    always @ (addr[31: 2])
        
        case (addr[31: 2])
            
            /*
            def lcg(modulus, a, c, seed):
                while True:
                    seed = (a * seed + c) % modulus
                    yield seed
            
            */
            //init
            0: instr = {ADDI, r0, sp, `STACK_BEGIN_ADDR};//Stack pointer 256
            
            //1: instr = {ADDI, r0, t0, 16'd0};
            //2: instr = {SB, r0, t0, `IN0_ADDR};
            
            //Read seed from IO device
            3: instr = {LBU, r0, s0, `IN0_ADDR};
            
            //loop:
            10: instr = {ORI, s0, a1, 16'd0};// retrieve seed
            11: instr = {ADDI, r0, a0, 16'd17};// a = 17
            12: instr = {ADDI, r0, a2, 16'd3};// c = 3
            13: instr = {ADDI, r0, a3, 16'd256};// m = 256
            
            14: instr = {JAL, 26'd50};// call lcg()
            
            15: instr = {ORI, v0, s0, 16'b0};
            16: instr = {SB, r0, v0, `OUT0_ADDR};
            17: instr = {BEQ, r0, r0, 16'hfff8};//loop
            
            
            //linear congruential generator  
            //int lcg(a, seed, c, m) {return seed = (a * seed + c) % m;}
            50: instr = {6'b011100, a1, a0, t0, 5'b00000, 6'b000010};//MUL
            51: instr = {6'b0, a2, t0, t0, 5'b00000, ADDU};
            52: instr = {ORI, t0, a0, 16'b0};
            53: instr = {6'b000000, r0, a3, a1, 5'b0, OR};
            //Push
            54: instr = {ADDI, sp, sp, 16'hfffc};// sp -= 4
            55: instr = {SW, sp, ra, 16'b0};
            //Push end
            
            //Testing JARL
            56: instr = {ORI, r0, t2, 14'd80, 2'd0};
            57: instr = {6'b0, t2, r0, ra, 5'b0, JALR};
            //56: instr = {JAL, 26'd80};//call mod()
            
            //Pop
            58: instr = {LW, sp, ra, 16'b0};
            59: instr = {ADDI, sp, sp, 16'h4};// sp -= 4
            //Pop end
            60: instr = RET;
            
            // int mod(a0, a1) {return v0 = a0 % a1;} 
            80: instr = {ADDI, a0, v0, 16'b0};//ADDI $2, $4, 0 # $1 <= $2
            81: instr = {6'b0, a1, v0, t0, 5'b0, SUB};//SUB $8, $3, $2
            82: instr = {6'b0, r0, v0, t1, 5'b0, SUB};
            83: instr = {BLEZ, t0, 5'b0, 16'b10};// # if c - b <= 0, then branch 1 (b -= c)
            84: instr = {BGTZ, t1, 5'b0, 16'b11};// # else if b < 0, then branch 2 (b += c)
            85: instr = RET;// # else return 
            86: instr = {6'b0, v0, a1, v0, 5'b0, SUB};//SUB $1, $1, $3 # branch 1
            87: instr = {BEQ, r0, r0, 16'hfff9};// B -0x7 # loop
            88: instr = {6'b0, v0, a1, v0, 5'b0, ADD};//ADD $1, $1, $3 # branch 2
            89: instr = {BEQ, r0, r0, 16'hfff7};// B -0x9 # loop
            
            /*
            //Test 1
            0: instr = 32'b001000_00000_00010_00000000_00001000;// ADDI $2, $0, 0x8
            1: instr = 32'b001000_00000_00001_00000000_00000000;// ADDI $1, $0, 0x1
            2: instr = 32'b001000_00001_00001_00000000_00000001;// ADDI $1, $0, 0x1
            3: instr = 32'b000101_00010_00001_11111111_11111110;// BNE $2, $0, -0x2 # for instruction
            
            5: instr = HLT// BEQ $0, $0, -0x2 # halt
            */
            /*
            //Testing ALU slt and shift
            0: instr = 32'b001010_10000_00001_1111111111111111;// SLTI $1, $16, -0x1
            1: instr = 32'b001010_10000_00001_0000000000000000;// SLTI $1, $16, 0x0
            2: instr = 32'b001010_10000_00001_0000000000001111;// SLTI $1, $16, 0xf # $1 <= 1
            
            3: instr = 32'b001011_10000_00010_0000000000000000;// SLTIU $2, $16, -0 
            4: instr = 32'b001011_10000_00010_1111111111111111;// SLTIU $2, $16, 0xffff # $2 <= 1
            
            5: instr = 32'b001111_00000_00011_11111111_11110011;// LUI $3, -13
            
            6: instr = 32'b000000_00000_00011_00001_00010_000000;// SLL $1, $3, 0x2 
            7: instr = 32'b000000_00000_00011_00001_00010_000010;// SRL $1, $3, 0x2
            8: instr = 32'b000000_00000_00011_00001_00010_000011;// SRA $1, $3, 0x2
             
            9: instr = 32'b000000_00010_00011_00001_00010_000100;// SLLV $1, $3, $2
            10: instr = 32'b000000_00010_00011_00001_00010_000110;// SRLV $1, $3, $2
            11: instr = 32'b000000_00010_00011_00001_00010_000111;// SRAV $1, $3, $2
            */
            
            /*
            //Testing Branch
            0: instr = 32'b001111_00000_00001_11111111_11110011;// LUI $1, -13
            1: instr = 32'b001000_00000_00010_00000000_00000001;// ADDI $2, $0, 0x1
            2: instr = 32'b001000_00000_00011_00000000_00000010;// ADDI $3, $1, 0x2
            3: instr = 32'b000101_00001_00001_11111111_11111111;// BNE $1, $1, -0x1 # current instruction
            4: instr = 32'b000100_00001_00000_11111111_11111111;// BEQ $1, $0, -0x2 # current instruction
            5: instr = 32'b000100_00001_00001_00000000_00001000;// BEQ $1, $1, 0x8 
            
            14: instr = 32'b000110_00010_00000_11111111_11111111;// BLEZ $2, -0x2 # current instruction
            15: instr = 32'b000110_00001_00000_00000000_00001000;// BLEZ $1, 0x8 
            
            24: instr = 32'b000111_00000_00000_11111111_11111111;// BGTZ $0, -0x2 # current instruction
            25: instr = 32'b000111_00001_00000_11111111_11111111;// BGTZ $1, -0x2 # current instruction
            26: instr = 32'b000111_00011_00000_11111111_11111111;// BGTZ $3, -0x2 # current instruction
            */
            
            /*
            //Testing Store and Load
            0: instr = 32'b001111_00000_00001_11111111_11110011;// LUI $1, -13
            1: instr = 32'b001000_00001_00001_10000000_10000010;// ADDI $1, $1, 0x2
            2: instr = 32'b101001_00000_00001_00000000_00000010;// SH $1, 2($0) 
            3: instr = 32'b101000_00000_00001_00000000_00000001;// SB $1, 1($0)
            4: instr = 32'b101011_00000_00001_00000000_00000100;// SW $1, 4($0)
            
            5: instr = 32'b100100_00000_00001_00000000_00000001;// LBU $1, 1($0)
            6: instr = 32'b100101_00000_00010_00000000_00000010;// LHU $2, 1($0)
            7: instr = 32'b100001_00000_00011_00000000_00000010;// LH $3, 2($0)
            8: instr = 32'b100011_00000_00100_00000000_00000100;// LH $4, 4($0)
            */     
            
            //2: instr = 32'b001101_00000_00010_00000000_00000010;// ORI $2, $0, 2
            //3: instr = 32'b101011_00000_00010_00000000_00000000;// SW $2, 0($0) 
            //4: instr = 32'b100001_00010_00001_11111111_11111110;// LH $1, -2($2) $1 <- MEM[-2+2] = 2
            //4: instr = 32'b001000_00000_00000_00000000_00000010;
            //5: instr = 32'b001000_00000_00000_00000000_00000010;
            //...
            default: instr = 32'b0;
        endcase

endmodule


module DMem (clk, addr, writeEnable, writeData, readData, sign, width);

    input clk, writeEnable;
    input [31: 0] addr, writeData;
    input [1: 0] width;
    input sign;
    output reg [31: 0] readData;
    
    wire iwrite;
    wire [31:0] iaddr, idata;
    
    reg [31: 0] RAM[`DMEM_MAX_ADDR/4 - 1: 0];
    reg s;
    always @ (*)
        case ({ width, addr[1:0] })
        4'b0000: s = sign ? RAM[addr[31: 2]][7] : 0;
        4'b0001: s = sign ? RAM[addr[31: 2]][15] : 0;
        4'b0010: s = sign ? RAM[addr[31: 2]][23] : 0;
        4'b0011: s = sign ? RAM[addr[31: 2]][31] : 0;
        
        4'b0100: s = sign ? RAM[addr[31: 2]][15] : 0;
        4'b0101: s = sign ? RAM[addr[31: 2]][15] : 0;
        4'b0110: s = sign ? RAM[addr[31: 2]][31] : 0;
        4'b0111: s = sign ? RAM[addr[31: 2]][31] : 0;
        default: s = 0;
        endcase
    //reg [7: 0] RAM[63: 0];
    //assign readData = RAM[ addr[31: 2] ];//Word aligned
    
    // LB, LH, LW
    always @ (*)
        case (width)
        0: 
        begin
            case (addr[1: 0])
            0: readData = {{24{s}}, RAM[addr[31: 2]][7: 0]};
            1: readData = {{24{s}}, RAM[addr[31: 2]][15: 8]};
            2: readData = {{24{s}}, RAM[addr[31: 2]][23: 16]};
            3: readData = {{24{s}}, RAM[addr[31: 2]][31: 24]};
            endcase
        end
        1: readData = addr[1] ? 
            {{16{s}}, RAM[addr[31: 2]][31: 16]}:
            {{16{s}}, RAM[addr[31: 2]][15: 0]};
        2: readData = RAM[addr[31: 2]];
        default: readData = RAM[addr[31: 2]];
        endcase
    
    always @ (posedge clk)
        if (writeEnable)
            case (width)
            0: begin
                case (addr[1: 0])
                0: RAM[addr[31: 2]][7: 0] <= writeData[7:0];
                1: RAM[addr[31: 2]][15: 8] <= writeData[7:0];
                2: RAM[addr[31: 2]][23: 16] <= writeData[7:0];
                3: RAM[addr[31: 2]][31: 24] <= writeData[7:0];
                endcase
            end
            1: begin
                if (addr[1]) RAM[addr[31: 2]][31: 16] <= writeData[15: 0];
                else RAM[addr[31: 2]][15: 0] <= writeData[15: 0];
            end
            2: RAM[addr[31: 2]] <= writeData;
            default: RAM[addr[31: 2]] <= RAM[addr[31: 2]];
            endcase
        else if (iwrite)
            RAM[iaddr[31: 2]] <= idata;//For input 
            
endmodule


module MIPS (clk, rst, PC, Instr, MemWrite, MemSign, MemWidth, ALUResult, WriteData, ReadData);
    input clk, rst;
    input [31: 0] Instr;
    input [31: 0] ReadData;
    
    
    output MemWrite, MemSign;
    output [1: 0] MemWidth;
    output [31: 0] PC;
    output [31: 0] ALUResult, WriteData;

    wire ZF, SF, OF, RegWrite, ALUSrc, MemWrite, PCSrc, Jump;
    wire [1:0] MemToReg, RegDst;
    wire [3: 0] ALUControl;

    Datapath dp(clk, rst, Instr, MemToReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump, MemWrite, ALUControl, ALUResult, WriteData, ReadData, ZF, SF, OF, PC);

    ControlUnit cu(Instr[31: 26], Instr[5: 0], ZF, SF, OF, RegDst, RegWrite, ALUSrc, ALUControl, MemWrite, MemSign, MemWidth, MemToReg, PCSrc, Jump);
    
endmodule


module Datapath (clk, rst, Instr, MemToReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump, MemWrite, ALUControl, ALUResult, WriteData, ReadData, ZF, SF, OF, PC);
    input clk, rst;
    
    //ControlUnit
    input PCSrc, ALUSrc, RegWrite, Jump, MemWrite;
    input [1: 0] MemToReg, RegDst;//For JAL
    input [3: 0] ALUControl;
    
    input [31: 0] Instr;
    input [31: 0] ReadData;//the output of DMem

    output [31: 0] WriteData;//data to wirte DMem
    output [31: 0] ALUResult;
    output ZF, OF, SF;
    output [31: 0] PC;
    //output [31: 0] srcB;
    wire [4: 0] writeReg;
    
    wire [31: 0] pcNext, pcPlus4, pcBranch, pcNextBranch; 
    wire [31: 0] pcJump;//for jr jal
    
    wire [31: 0] signImm, signImmSh2;//shift 2

    wire [31: 0] writeData;//for regfile
   
    wire [31: 0] readData1;//output of regfile
    wire [31: 0] srcB;//, ALUResult;//ALU
    reg [31: 0] srcA;
    
    PCReg #(32) pcreg (clk, rst, pcNext, PC);
    Adder adder1 (PC, 32'b100, pcPlus4);

    ShiftLeft2 immsh (signImm, signImmSh2);//imm shift

    Adder adder2 (pcPlus4, signImmSh2, pcBranch);
    
    assign pcJump =  (Instr[31:26] == 6'b000000 && (Instr[5: 0] == 6'b001000 || Instr[5: 0] == 6'b001001)) ? readData1: {pcPlus4[31:28], Instr[25:0], 2'b00};//For JR and JALR
    
    assign pcNextBranch = PCSrc ? pcBranch:pcPlus4; 
    assign pcNext = Jump ? pcJump: pcNextBranch;
    //Mux2 #(32) pcbrmux (pcPlus4, pcBranch, PCSrc, pcNextBranch);//branch mux
    //Mux2 #(32) pcmux (pcNextBranch, pcJump, Jump, pcNext);//branch mux
    
    //JAL
    assign writeReg = RegDst == 2'b00 ? Instr[20:16] :(RegDst == 2'b01 ? Instr[15: 11] : 5'b11111 );
    //Mux2 #(5) wrmux (Instr[20: 16], Instr[15: 11], RegDst, writeReg);//for Regfile's WriteReg
    
    RegisterFile rf (clk, RegWrite, writeData, writeReg, Instr[25:21], Instr[20:16], readData1, WriteData);

    SignExtend signext (Instr[15: 0], signImm);
    
    Mux2 #(32) srcbmux (WriteData, signImm, ALUSrc, srcB);
    
    ALU alu (srcA, srcB, ALUControl, ALUResult, ZF, OF, SF);//add shamt
    
    //JAL
    assign writeData = MemToReg == 2'b00 ? ALUResult : (MemToReg == 2'b01 ? ReadData : pcPlus4);
    //Mux2 #(32) resmux(ALUResult, ReadData, MemToReg, writeData);
    
    //For alu shift
    always @ (*)
        if (Instr[31: 26] == 6'b0 && (Instr[5: 0] == 6'b0 || Instr[5: 0] == 6'b000010 || Instr[5: 0] == 6'b000011))
            srcA = {27'b0, Instr[10: 6]};
        else srcA = readData1;
        
endmodule

module ControlUnit (Func, Opcode, ZF, SF, OF, RegDst, RegWrite, ALUSrc, ALUControl, MemWrite, MemSign, MemWidth, MemToReg, PCSrc, Jump);
    input [5: 0] Func, Opcode;
    input ZF, SF, OF;
    output [3: 0] ALUControl;
    output RegWrite, ALUSrc, MemWrite, MemSign;
    output [1: 0] RegDst, MemToReg;
    output [1: 0] MemWidth;
    output Jump;
    output reg PCSrc;
    
    wire Branch;
    
    reg [11: 0] ctl;

    //parameter AND = 4'b0000, OR = 4'b0001, ADD = 4'b0010, SUB = 4'b0110, SLT = 4'b0111, XOR = 4'b1100, NOR = 4'b1101;
    
    assign { RegWrite, RegDst, ALUSrc, Branch, MemWrite, MemToReg, ALUControl } = ctl;

    assign Jump = (Func == 6'b000010 || Func == 6'b000011 || (Func == 6'b0 && (Opcode == 6'b001000 || Opcode == 6'b001001) ));//J or JAL or JR or JALR
    
    // for sh, lh, sb, lb, sw, lw
    assign MemWidth = ((Func == 6'b101001 || Func == 6'b100001 || Func == 6'b100101) ? 1: ((Func == 6'b101000 || Func == 6'b100000 || Func == 6'b100100) ? 0: 2));
    assign MemSign = (Func == 6'b100100 || Func == 6'b100101)? 0: 1;

    // Conditional Branch
    always @ (*) 
        if (Branch)
            case (Func)
            6'b000100: PCSrc = ZF;//BEQ
            6'b000101: PCSrc = ~ ZF;//BNE
            6'b000110: PCSrc = ZF | SF;//BLEZ
            6'b000111: PCSrc = ~ (ZF | SF);//BGTZ
            default:PCSrc = 0;
            endcase
        else PCSrc = 0;

    always @ (*)
        case (Func)
        6'b000000: 
        begin
            case (Opcode)
            //r
            6'b000000: ctl = {8'b10100000, `SLL};
            6'b000010: ctl = {8'b10100000, `SRL};
            6'b000011: ctl = {8'b10100000, `SRA};
            6'b000100: ctl = {8'b10100000, `SLL};
            6'b000110: ctl = {8'b10100000, `SRL};
            6'b000111: ctl = {8'b10100000, `SRA};
            
            6'b001000: ctl = {8'b10100000, `ADD};//JR
            6'b001001: ctl = {8'b10100011, `ADD};//JALR
            
            6'b100000: ctl = {8'b10100000, `ADD};
            6'b100001: ctl = {8'b10100000, `ADDU};
            6'b100010: ctl = {8'b10100000, `SUB};
            6'b100011: ctl = {8'b10100000, `SUBU};
            6'b100100: ctl = {8'b10100000, `AND};
            6'b100101: ctl = {8'b10100000, `OR};
            6'b100110: ctl = {8'b10100000, `XOR};
            6'b100111: ctl = {8'b10100000, `NOR};
            
            6'b101010: ctl = {8'b10100000, `SLT};
            6'b101011: ctl = {8'b10100000, `SLTU};
            
            default: ctl = 0;
            endcase
        end
        //SPECIAL(ADD, ADDU, AND, OR, XOR, SLT, SLTU, NOP, MOVZ, MOVN)
        //r
        6'b000100: ctl = {8'b0xx01000, `SUB};//BEQ (unconditional branch if rs rs are both zero)
        6'b000101: ctl = {8'b0xx01000, `SUB};//BNE
        6'b000110: ctl = {8'b0xx01000, `SUB};//BLEZ
        6'b000111: ctl = {8'b0xx01000, `SUB};//BGTZ
        
        //r
        6'b001000: ctl = {8'b10010000, `ADD};//ADDI
        6'b001001: ctl = {8'b10010000, `ADDU};//ADDIU
        6'b001010: ctl = {8'b10010000, `SLT};//SLTI
        6'b001011: ctl = {8'b10010000, `SLTU};//SLTIU
        6'b001100: ctl = {8'b10010000, `AND};//ANDI
        6'b001101: ctl = {8'b10010000, `OR};//ORI
        6'b001110: ctl = {8'b10010000, `XOR};//XORI
        6'b001111: ctl = {8'b10010000, `LUI};//LUI
        
        6'b011100: ctl = Opcode == 6'b000010 ? {8'b10100000, `MUL} : 0;//MUL
            
        //r
        6'b100000: ctl = {8'b10010001, `ADD};//LB 
        6'b100001: ctl = {8'b10010001, `ADD};//LH
        6'b100011: ctl = {8'b10010001, `ADD};//LW 
        6'b100100: ctl = {8'b10010001, `ADD};//LBU
        6'b100101: ctl = {8'b10010001, `ADD};//LHU
        6'b101000: ctl = {8'b0xx101xx, `ADD};//SB
        6'b101001: ctl = {8'b0xx101xx, `ADD};//SH
        6'b101011: ctl = {8'b0xx101xx, `ADD};//SW

        6'b000010: ctl = {8'b0xxxx000, `ADD};//J
        6'b000011: ctl = {8'b111xx011, `ADD};//JAL
        
        default: ctl = 0;
        endcase
endmodule

/*
module ControlUnit (Func, Opcode, ZF, SF, OF, RegDst, RegWrite, ALUSrc, ALUControl, MemWrite, MemSign, MemWidth, MemToReg, PCSrc, Jump);
    input [5: 0] Func, Opcode;
    input ZF, SF, OF;
    output [3: 0] ALUControl;
    output RegWrite, RegDst, ALUSrc, MemWrite, MemToReg, MemSign;
    output [1:0] MemWidth;
    output Jump;
    output reg PCSrc;
    
    wire Branch;
    
    reg [9: 0] ctl;

    //parameter AND = 4'b0000, OR = 4'b0001, ADD = 4'b0010, SUB = 4'b0110, SLT = 4'b0111, XOR = 4'b1100, NOR = 4'b1101;
    
    assign { RegWrite, RegDst, ALUSrc, Branch, MemWrite, MemToReg, ALUControl } = ctl;

    assign Jump = (Func == 6'b000010 || Func == 6'b000011 || (Func == 6'b0 && Opcode == 6'b001000));//J or JAL or JR
    
    // for sh, lh, sb, lb, sw, lw
    assign MemWidth = ((Func == 6'b101001 || Func == 6'b100001 || Func == 6'b100101) ? 1: ((Func == 6'b101000 || Func == 6'b100000 || Func == 6'b100100) ? 0: 2));
    assign MemSign = (Func == 6'b100100 || Func == 6'b100101)? 0: 1;

    // Conditional Branch
    always @ (*) 
        if (Branch)
            case (Func)
            6'b000100: PCSrc = ZF;//BEQ
            6'b000101: PCSrc = ~ ZF;//BNE
            6'b000110: PCSrc = ZF | SF;//BLEZ
            6'b000111: PCSrc = ~ (ZF | SF);//BGTZ
            default:PCSrc = 0;
            endcase
        else PCSrc = 0;

    always @ (*)
        case (Func)
        6'b000000: 
        begin
            case (Opcode)
            //r
            6'b000000: ctl = {6'b110000, `SLL};
            6'b000010: ctl = {6'b110000, `SRL};
            6'b000011: ctl = {6'b110000, `SRA};
            6'b000100: ctl = {6'b110000, `SLL};
            6'b000110: ctl = {6'b110000, `SRL};
            6'b000111: ctl = {6'b110000, `SRA};
            
            6'b001000: ctl = {6'b110000, `ADD};//JR
            
            6'b100000: ctl = {6'b110000, `ADD};
            6'b100001: ctl = {6'b110000, `ADDU};
            6'b100010: ctl = {6'b110000, `SUB};
            6'b100011: ctl = {6'b110000, `SUBU};
            6'b100100: ctl = {6'b110000, `AND};
            6'b100101: ctl = {6'b110000, `OR};
            6'b100110: ctl = {6'b110000, `XOR};
            6'b100111: ctl = {6'b110000, `NOR};
            
            6'b101010: ctl = {6'b110000, `SLT};
            6'b101011: ctl = {6'b110000, `SLTU};
            
            default: ctl = 0;
            endcase
        end
        //SPECIAL(ADD, ADDU, AND, OR, XOR, SLT, SLTU, NOP, MOVZ, MOVN)
        //r
        6'b000100: ctl = {6'b0x0100, `SUB};//BEQ (unconditional branch if rs rs are both zero)
        6'b000101: ctl = {6'b0x0100, `SUB};//BNE
        6'b000110: ctl = {6'b0x0100, `SUB};//BLEZ
        6'b000111: ctl = {6'b0x0100, `SUB};//BGTZ
        
        //r
        6'b001000: ctl = {6'b101000, `ADD};//ADDI
        6'b001001: ctl = {6'b101000, `ADDU};//ADDIU
        6'b001010: ctl = {6'b101000, `SLT};//SLTI
        6'b001011: ctl = {6'b101000, `SLTU};//SLTIU
        6'b001100: ctl = {6'b101000, `AND};//ANDI
        6'b001101: ctl = {6'b101000, `OR};//ORI
        6'b001110: ctl = {6'b101000, `XOR};//XORI
        6'b001111: ctl = {6'b101000, `LUI};//LUI
        
        //r
        6'b100000: ctl = {6'b101001, `ADD};//LB 
        6'b100001: ctl = {6'b101001, `ADD};//LH
        6'b100011: ctl = {6'b101001, `ADD};//LW 
        6'b100100: ctl = {6'b101001, `ADD};//LBU
        6'b100101: ctl = {6'b101001, `ADD};//LHU
        6'b101000: ctl = {6'b0x101x, `ADD};//SB
        6'b101001: ctl = {6'b0x101x, `ADD};//SH
        6'b101011: ctl = {6'b0x101x, `ADD};//SW

        6'b000010: ctl = {6'b0xx000, `ADD};//J
        //6'b000011: ctl = {6'b1xx000, `ADD};//J
        
        default: ctl = 0;
        endcase
endmodule
*/

module Adder (a, b, y);
    input [31: 0] a, b;
    output [31: 0] y;
    assign y = a + b;
endmodule

module RegisterFile (clk, writeEnable, writeData, writeAddr, readAddr1, readAddr2, readData1, readData2);

    input clk, writeEnable;
    input [4: 0] readAddr1, readAddr2, writeAddr;
    input [31: 0] writeData;

    output [31: 0] readData1, readData2;

    reg [31: 0] RegCell [0: 31];
    
    assign readData1 = (readAddr1 != 0) ? RegCell[readAddr1] : 0;
    
    assign readData2 = (readAddr2 != 0) ? RegCell[readAddr2] : 0;

    always @ (posedge clk)
        if (writeEnable) RegCell[writeAddr] <= writeData;
    
    integer i;
    //initialization
    initial
    begin
    for (i = 0; i < 32; i = i + 1) RegCell[i] = 0;
    end
    
endmodule

module Mux2 #(parameter WIDTH = 8) (d0, d1, s, y);
    input [WIDTH-1: 0] d0, d1;
    input s;
    output [WIDTH-1: 0] y;
    assign y = s ? d1: d0;
endmodule

//PC
module PCReg #(parameter WIDTH = 8) (clk, rst, d, q);

    input clk, rst;
    input [WIDTH - 1: 0] d;
    output reg [WIDTH - 1: 0] q = 0;

    always @ (posedge clk or posedge rst)
        if (rst) q <= 0;
        else q <= d;
        
endmodule

module ShiftLeft2 (a, y);

    input [31: 0] a;
    output [31: 0] y;
    assign y = {a[29: 0], 2'b00};

endmodule

module SignExtend (a, y);
    input [15: 0] a;
    output [31: 0] y;
    assign y = { {16{a[15]}}, a };
endmodule

/*
module Control(Instr);


endmodule


module Execute (CLK, Rst, Instr, SrcA, SrcB, Control, Result);
    input CLK, Rst;
    input [31: 0] Instr, SrcA, SrcB;
    input [4: 0] Control;
    output reg [31:0] Result;
    
    wire [31: 0] ALUResult;
    wire [31: 0] ShiftResult;
    wire [31: 0] MultiplyResult;
    
    wire ZF, OF, SF;
    
    wire Direction = ;
    
    wire Signed = ;
    
    Shifter (SrcB, SrcA, Direction, Signed, ShiftResult);
    Multiplier (SrcA, SrcB, Signed, Result);
    ALU (SrcA, SrcB, ALUControl, ALUResult, ZF, OF, SF);
    
endmodule 




module Shifter (Src, Shamt, Direction, Signed, Result);
    input [31: 0] Src;
    input [4:0] Shamt;
    input Direction;//0 = left, 1 = right
    input Signed;
    output reg [31: 0] Result;

    always @ (*)
        case ({Direction, Signed})
        2'b00: Result = Src << Shamt;
        2'b01: Result = Src << Shamt;
        2'b10: Result = Src >> Shamt;
        2'b11: Result = $signed(Src) >>> Shamt;
        endcase
    
endmodule 



module Multiplier (SrcA, SrcB, Signed, Result);
    input [31: 0] SrcA, SrcB;
    input Signed;
    output [31: 0] Result;
    
    reg [63: 0] temp;
    assign Result = temp[31: 0];
        
    always @ (*)
        if (Signed) temp = $signed(SrcA) * $signed(SrcB);
        else temp = SrcA * SrcB;
        
endmodule

*/
module ALU (SrcA, SrcB, ALUControl, ALUResult, ZF, OF, SF);
    input [31: 0] SrcA, SrcB;
    //input [4: 0] Shamt;// shift amount
    input [3: 0] ALUControl;
    output ZF, OF, SF;
    output reg [31: 0] ALUResult;
    //output reg [31: 0] HO, LO;
    wire [4:0] shamt;
    
    assign shamt = SrcA[4: 0];
    
    assign ZF = (ALUResult == 0)? 1: 0;
    assign SF = ALUResult[31];
    assign OF = (~(SrcA[31] ^ SrcB[31])) & (SrcA[31] ^ ALUResult[31]);
    
    always @ ( * )
        case (ALUControl)
            `SLL: ALUResult = SrcB << shamt;//shift left
            `SRL: ALUResult = SrcB >> shamt;//shift right logic
            `SRA: ALUResult = $signed(SrcB) >>> shamt;//shift right arithmetic
            `LUI: ALUResult = {SrcB[15: 0], 16'b0};//shift right arithmetic
            
            //`SLLV: ALUResult = SrcB << SrcA;//shift left
            //`SRLV: ALUResult = SrcB >> SrcA;//shift right logic
            //`SRAV: ALUResult = SrcB >>> SrcA;//shift right arithmetic
            `MUL: ALUResult = $signed(SrcA) * $signed(SrcB);
            
            `AND: ALUResult = SrcA & SrcB;
            `OR: ALUResult = SrcA | SrcB;
            `ADD: ALUResult = SrcA + SrcB;
            `ADDU: ALUResult = SrcA + SrcB;
            `SUB: ALUResult = SrcA - SrcB;
            `SUBU: ALUResult = SrcA - SrcB;
            `XOR: ALUResult = SrcA ^ SrcB;//XOR
            `NOR: ALUResult = ~(SrcA | SrcB);//NOR
            //4: ALUResult = SrcA & ~SrcB;
            //5: ALUResult = SrcA | ~SrcB;
            `SLT: ALUResult = ($signed(SrcA) < $signed(SrcB));//SLT: set if less than (signed)
            `SLTU: ALUResult = (SrcA < SrcB);//SLT: set if less than (unsigned)
            default: ALUResult = 32'hxxxx_xxxx;
        endcase
endmodule
