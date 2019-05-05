`timescale 1ns / 1ps

`define CTL_WIDTH 31:0

`define CTL_ALUCONTROL 31:28//4bits

`define CTL_ALUSRCA 27
`define ALUSRCA_RS 1'b0
`define ALUSRCA_SHAMT 1'b1

`define CTL_ALUSRCB 26:25
`define ALUSRCB_RT 2'b0
`define ALUSRCB_IMM 2'b1
`define ALUSRCB_IMMU 2'b10//zero-extended immediate


`define CTL_REGDST 1:0
`define REGDST_RTYPE 2'd0
`define REGDST_ITYPE 2'd1
`define REGDST_RA 2'd2

`define CTL_REGWRITE 2

`define CTL_REGWRITE_DATA 4: 3
`define REGWRITE_DATA_FROM_ALU 2'd0
`define REGWRITE_DATA_FROM_MEMORY 2'd1
`define REGWRITE_DATA_FROM_PCPLUS4 2'd2

`define CTL_MEMREAD 5// useless? no

`define CTL_MEMWRITE 6
`define CTL_MEMSIGNED 7
`define CTL_MEMWIDTH 9: 8
`define MEMWIDTH_8 2'd0
`define MEMWIDTH_16 2'd1
`define MEMWIDTH_32 2'd2

`define CTL_PCSRC 11: 10
`define PCSRC_PCPLUS4 2'b0
`define PCSRC_BRANCH 2'b1
`define PCSRC_JUMPIMM 2'd2
`define PCSRC_JUMPREG 2'd3


`define DMEM_MAX_ADDR 16'd512

`define STACK_BEGIN_ADDR 16'd256 


//IO Mapping
`define IN0_ADDR 16'd272
`define OUT0_ADDR 16'd256


//ALU Control Signal
`define ALU_SLL 4'b0000
`define ALU_SRL 4'b0001
`define ALU_SRA 4'b0010

`define ALU_LUI 4'b0011
`define ALU_MUL 4'b0100
`define ALU_ADD 4'b0110
`define ALU_ADDU 4'b0111
`define ALU_SUB 4'b1000
`define ALU_SUBU 4'b1001
`define ALU_AND 4'b1010
`define ALU_OR 4'b1011
`define ALU_XOR 4'b1100
`define ALU_NOR 4'b1101

`define ALU_SLT 4'b1110
`define ALU_SLTU 4'b1111

/*

Notes:

1. always @ (*) or always @ (A or B) is combinational logic, only = (blocking) assignments should be used in it.

2. reg is not always a flipflop. In combinational "always @ ()" reg is eventually a wire, while in sequential "always", reg can be a flipflop.

3. In an always @ (*) block, each value that is assigned in at least one place must be assigned to a non-trivial value during every ‘execution’ of the always @ (*) block. That's why 'else' should not be omitted after an 'if', otherwise, this will cause a latch to deal with the unassigned situation.
(http://inst.eecs.berkeley.edu/~eecs151/sp19/files/verilog/always_at_blocks.pdf)

4. Initialization

*/


module top (CLK100MHZ, SW, AN, CA, CB, CC, CD, CE, CF, CG);
    
    input CLK100MHZ;
    input [15: 0] SW;
    
    output [7: 0] AN;
    output CA, CB, CC, CD, CE, CF, CG;
    
    reg [31: 0] data;
    wire [6: 0] C;
    wire clk, rst;
    
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
    
    clkdiv cd(.mclk(CLK100MHZ), .clk(clk));
    
    MIPSTop mt(clk, rst);
    
    Display dis(CLK100MHZ, AN, C, 
        {d7, d6, d5, d4, d3, d2, d1, d0});
        
    assign {CA, CB, CC, CD, CE, CF, CG} = C;
    assign rst = SW[0];
    
    // Input Mapping 
    assign mt.dmem.iwrite = SW[0] & SW[1];
    assign mt.dmem.iaddr = `IN0_ADDR;
    assign mt.dmem.idata = SW[15:4];
    
endmodule

//For 7 segments display
module Display (CLK100MHZ, AN, C, Data);
    
    input CLK100MHZ;
    input [31: 0] Data;
    output reg [6:0] C;
    output [7: 0] AN;
    
    wire [3: 0] d0, d1, d2, d3, d4, d5, d6, d7;
    wire clk380hz;
    reg [3:0] current;
    reg [7: 0] an;
    
    parameter mask = 8'b0000_1000;
    
    clkdiv cc(.mclk(CLK100MHZ), .clk380(clk380hz));
    
    assign AN = an | mask;
    assign {d0, d1, d2, d3, d4, d5, d6, d7} = Data;
    
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
        case(current)
            0:C = 7'b0000001;
            1:C = 7'b1001111;
            2:C = 7'b0010010;
            3:C = 7'b0000110;
            4:C = 7'b1001100;
            5:C = 7'b0100100;
            6:C = 7'b0100000;
            7:C = 7'b0001111;
            8:C = 7'b0000000;
            9:C = 7'b0001100;
            10:C = 7'b0001000;
            11:C = 7'b1100000;
            12:C = 7'b0110001;
            13:C = 7'b1000010;
            14:C = 7'b0110000;
            15:C = 7'b0111000;
            default: C = 0;
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


module MIPSTop (CLK, RST);

    input CLK, RST;
    
    wire [31: 0] ALUResult;
    wire [31: 0] PC, WriteData, ReadData;
    wire [31: 0] Instr;
    wire MemWrite;
    wire MemSign;
    wire [1: 0] MemWidth;
    
    MIPS mips(CLK, RST, PC, Instr, MemWrite, MemSign, MemWidth, ALUResult, WriteData, ReadData);
    IMem imem(PC, Instr);
    DMem dmem(CLK, ALUResult, MemWrite, WriteData, ReadData, MemSign, MemWidth);

endmodule

module IMem (Addr, Instr);
    input [31: 0] Addr;
    output [31: 0] Instr;
    
    parameter SIZE = 200;
    
    reg [31: 0] ROM[SIZE - 1: 0];
    assign Instr = ROM[Addr[31:2]];
    
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
    
    
    integer i;
    initial
    begin
        for (i = 0; i < SIZE; i = i + 1) ROM[i] = 0;
        
        ROM[0] = {ADDI, r0, sp, `STACK_BEGIN_ADDR};//Stack pointer 256
        
            
        //ROM[1] = {ADDI, r0, t0, 16'd0};
        //ROM[2] = {SB, r0, t0, `IN0_ADDR};
            
        //Read seed from IO device
        ROM[3] = {LBU, r0, s0, `IN0_ADDR};
            
        //loop:
        ROM[10] = {ORI, s0, a1, 16'd0};// retrieve seed
        ROM[11] = {ADDI, r0, a0, 16'd17};// a = 17
        ROM[12] = {ADDI, r0, a2, 16'd3};// c = 3
        ROM[13] = {ADDI, r0, a3, 16'd256};// m = 256
            
        ROM[14] = {JAL, 26'd50};// call lcg()
            
        ROM[15] = {ORI, v0, s0, 16'b0};
        ROM[16] = {SB, r0, v0, `OUT0_ADDR};
        ROM[17] = {BEQ, r0, r0, 16'hfff8};//loop
            
            
        //linear congruential generator  
        //int lcg(a, seed, c, m) {return seed = (a * seed + c) % m;}
        ROM[50] = {6'b011100, a1, a0, t0, 5'b00000, 6'b000010};//MUL
        ROM[51] = {6'b0, a2, t0, t0, 5'b00000, ADDU};
        ROM[52] = {ORI, t0, a0, 16'b0};
        ROM[53] = {6'b000000, r0, a3, a1, 5'b0, OR};
        //Push
        ROM[54] = {ADDI, sp, sp, 16'hfffc};// sp -= 4
        ROM[55] = {SW, sp, ra, 16'b0};
        //Push end
            
        //Testing JARL
        ROM[56] = {ORI, r0, t2, 14'd80, 2'd0};
        ROM[57] = {6'b0, t2, r0, ra, 5'b0, JALR};
        //ROM[56] = {JAL, 26'd80};//call mod()
            
        //Pop
        ROM[58] = {LW, sp, ra, 16'b0};
        ROM[59] = {ADDI, sp, sp, 16'h4};// sp -= 4
        //Pop end
        ROM[60] = RET;
            
        // int mod(a0, a1) {return v0 = a0 % a1;} 
        ROM[80] = {ADDI, a0, v0, 16'b0};//ADDI $2, $4, 0 # $1 <= $2
        ROM[81] = {6'b0, a1, v0, t0, 5'b0, SUB};//SUB $8, $3, $2
        ROM[82] = {6'b0, r0, v0, t1, 5'b0, SUB};
        ROM[83] = {BLEZ, t0, 5'b0, 16'b10};// # if c - b <= 0, then branch 1 (b -= c)
        ROM[84] = {BGTZ, t1, 5'b0, 16'b11};// # else if b < 0, then branch 2 (b += c)
        ROM[85] = RET;// # else return 
        ROM[86] = {6'b0, v0, a1, v0, 5'b0, SUB};//SUB $1, $1, $3 # branch 1
        ROM[87] = {BEQ, r0, r0, 16'hfff9};// B -0x7 # loop
        ROM[88] = {6'b0, v0, a1, v0, 5'b0, ADD};//ADD $1, $1, $3 # branch 2
        ROM[89] = {BEQ, r0, r0, 16'hfff7};// B -0x9 # loop           
    end
    
endmodule

/*
module AddressDecoder(MemWrite, Addr, MemWriteEnable, IOWriteEnable, ReadDataSel);
    input MemWrite;
    input [31: 0] Addr;
    output MemWriteEnable, IOWriteEnable;
    output [`READ_IO_SELECT_WIDTH] ReadDataSel;
    
    wire isIO = Addr == `IN0_ADDR;
    
    assign MemWriteEnable = isIO ? 0 : MemWrite;
    assign IOWriteEnable = isIO ? MemWrite : 0;
    assign ReadDataSel = isIO ? `READ_IO: `READ_MEM;
    
endmodule
*/

module DMem (CLK, Addr, WriteEnable, WriteData, ReadData, Sign, Width);

    input CLK, WriteEnable;
    input [31: 0] Addr, WriteData;
    input [1: 0] Width;
    input Sign;
    output reg [31: 0] ReadData;
    
    wire iwrite;
    wire [31:0] iaddr, idata;
    
    reg [31: 0] RAM[`DMEM_MAX_ADDR/4 - 1: 0];

    always @ (*)
    
        case (Width)
        `MEMWIDTH_8: 
        begin
            case (Addr[1: 0])
            0: ReadData = {{24{Sign & RAM[Addr[31: 2]][7]}}, RAM[Addr[31: 2]][7: 0]};
            1: ReadData = {{24{Sign & RAM[Addr[31: 2]][15]}}, RAM[Addr[31: 2]][15: 8]};
            2: ReadData = {{24{Sign & RAM[Addr[31: 2]][23]}}, RAM[Addr[31: 2]][23: 16]};
            3: ReadData = {{24{Sign & RAM[Addr[31: 2]][31]}}, RAM[Addr[31: 2]][31: 24]};
            endcase
        end
        
        `MEMWIDTH_16: ReadData = Addr[1] ? 
            {{16{Sign & RAM[Addr[31: 2]][31]}}, RAM[Addr[31: 2]][31: 16]}:
            {{16{Sign & RAM[Addr[31: 2]][15]}}, RAM[Addr[31: 2]][15: 0]};
        
        `MEMWIDTH_32: ReadData = RAM[Addr[31: 2]];
        
        default: ReadData = RAM[Addr[31: 2]];
        endcase
    
    always @ (posedge CLK) 
    
        if (WriteEnable)
        
            case (Width)
            `MEMWIDTH_8: 
            begin
                case (Addr[1: 0])
                0: RAM[Addr[31: 2]][7: 0] <= WriteData[7:0];
                1: RAM[Addr[31: 2]][15: 8] <= WriteData[7:0];
                2: RAM[Addr[31: 2]][23: 16] <= WriteData[7:0];
                3: RAM[Addr[31: 2]][31: 24] <= WriteData[7:0];
                endcase
            end
            
            `MEMWIDTH_16:
            begin
                if (Addr[1]) RAM[Addr[31: 2]][31: 16] <= WriteData[15: 0];
                else RAM[Addr[31: 2]][15: 0] <= WriteData[15: 0];
            end
            
            `MEMWIDTH_32: RAM[Addr[31: 2]] <= WriteData;
            
            default: RAM[Addr[31: 2]] <= RAM[Addr[31: 2]];
            endcase
            
        else if (iwrite)
            RAM[iaddr[31: 2]] <= idata;//For input 
            
endmodule


module MIPS (CLK, RST, PC, Instr, MemWrite, MemSigned, MemWidth, ALUResult, MemWriteData, MemReadData);
    input CLK, RST;
    input [31: 0] Instr;
    input [31: 0] MemReadData;
    
    output MemWrite, MemSigned;
    output [1: 0] MemWidth;
    output [31: 0] PC;
    output [31: 0] ALUResult, MemWriteData;

    wire ZF, SF, OF;
    wire [1:0] MemToReg, RegDst;
    wire [3: 0] ALUControl;
    
    wire [`CTL_WIDTH] ctl;
    wire branchSrc;
    
    assign MemWidth = ctl[`CTL_MEMWIDTH];
    assign MemSigned = ctl[`CTL_MEMSIGNED];
    assign MemWrite = ctl[`CTL_MEMWRITE];

    Datapath dp(CLK, RST, Instr, ctl, branchSrc, MemReadData, MemWriteData, ALUResult, ZF, SF, OF, PC);

    ControlUnit cu(Instr[31: 26], Instr[5: 0], ZF, SF, OF, ctl, branchSrc);
    
endmodule


module Datapath (CLK, RST, Instr, CTL, BranchSrc, MemReadData, MemWriteData, ALUResult, ZF, SF, OF, PC);
    input CLK, RST;
    input [31: 0] Instr;
    input [`CTL_WIDTH] CTL;
    input BranchSrc;//taken or not taken
    
    input [31: 0] MemReadData;//the output of DMem

    output [31: 0] MemWriteData;//data to wirte DMem
    output [31: 0] ALUResult;
    output ZF, OF, SF;
    output [31: 0] PC;
    
    wire [31: 0] pcNext, pcPlus4, pcBranch, pcNextBranch; 
    //wire [31: 0] pcJump;//for jr jal
    
    wire [31: 0] imm, signImm, signImmSh2;//shift 2

    wire [4: 0] writeReg;// regdst
    wire [31: 0] regWriteData;//for regfile
    wire [31: 0] readData1, readData2;//output of regfile
    
    wire [31: 0] srcA, srcB;//ALU
    
    PCReg #(32) pcreg (CLK, RST, pcNext, PC);
    
    //assign pcPlus4 = PC + 32'b100;
    Adder adder1 (PC, 32'b100, pcPlus4);

    ShiftLeft2 immsh (signImm, signImmSh2);//imm shift
    
    //Some instr should not sign extend !    
    SignExtend signext (Instr[15: 0], signImm);
    
    Adder adder2 (pcPlus4, signImmSh2, pcBranch);
    
    assign pcNextBranch = BranchSrc ? pcBranch: pcPlus4; 
    
    assign pcNext = CTL[`CTL_PCSRC] == `PCSRC_BRANCH ? pcNextBranch
        : CTL[`CTL_PCSRC] == `PCSRC_JUMPIMM ? {pcPlus4[31:28], Instr[25:0], 2'b00}
        : CTL[`CTL_PCSRC] == `PCSRC_JUMPREG ? readData1
        : pcPlus4 ;

    assign writeReg = CTL[`CTL_REGDST] == `REGDST_RTYPE ? Instr[15: 11]
        : CTL[`CTL_REGDST] == `REGDST_ITYPE ? Instr[20: 16]: 5'b11111;
    
    assign regWriteData = CTL[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_ALU ? ALUResult
        : CTL[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_MEMORY ? MemReadData
        : pcPlus4;
    
    RegisterFile rf (CLK, CTL[`CTL_REGWRITE], regWriteData, writeReg, Instr[25:21], Instr[20:16], readData1, readData2);

    assign srcA = CTL[`CTL_ALUSRCA] == `ALUSRCA_SHAMT ? {27'b0, Instr[10: 6]}: readData1;
    assign srcB = CTL[`CTL_ALUSRCB] == `ALUSRCB_IMM ? signImm 
    : CTL[`CTL_ALUSRCB] == `ALUSRCB_IMMU ? {16'b0, Instr[15: 0]}
    : readData2;
    
    ALU alu (srcA, srcB, CTL[`CTL_ALUCONTROL], ALUResult, ZF, OF, SF);//add shamt
    
    assign MemWriteData = readData2;
        
endmodule



module ControlUnit (Func, Opcode, ZF, SF, OF, CTL, BranchSrc);
    input [5: 0] Func, Opcode;
    input ZF, SF, OF;
    
    output [`CTL_WIDTH] CTL;
    output reg BranchSrc;
    
    reg [`CTL_WIDTH] ctl;
    assign CTL = ctl;
    
    
    // Conditional Branch
    always @ (*) 
        if (ctl[`CTL_PCSRC] == `PCSRC_BRANCH)
            case (Func)
            6'b000100: BranchSrc = ZF;//BEQ
            6'b000101: BranchSrc = ~ ZF;//BNE
            6'b000110: BranchSrc = ZF | SF;//BLEZ
            6'b000111: BranchSrc = ~ (ZF | SF);//BGTZ
            default:BranchSrc = 0;
            endcase
        else BranchSrc = 0;

    always @ (*)
        case (Func)
        6'b000000: 
        begin
            case (Opcode)
            //r
            6'b000000://SLL
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SLL;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_SHAMT;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
                
            end
            6'b000010://SRL
            begin   
                ctl[`CTL_ALUCONTROL] = `ALU_SRL;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_SHAMT;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            
            
            end
            6'b000011://SRA
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SRA;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_SHAMT;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end

            6'b000100://SLLV
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SLL;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b000110://SRLV
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SRL;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b000111://SRAV
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SRA;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b001000://JR
            begin
                ctl[`CTL_REGWRITE] = 0;
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                ctl[`CTL_PCSRC] = `PCSRC_JUMPREG;
            end
            6'b001001://JALR
            begin
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_PCPLUS4;
                ctl[`CTL_REGDST] = `REGDST_RA;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                ctl[`CTL_PCSRC] = `PCSRC_JUMPREG;
            end
            6'b100000://ADD
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_ADD;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            6'b100001://ADDU
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_ADDU;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b100010://SUB
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SUB;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b100011://SUBU
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SUBU;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b100100://AND
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_AND;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            6'b100101://OR
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_OR;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b100110://XOR
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_XOR;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            
            6'b100111://NOR
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_NOR;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            6'b101010:
            begin
                //ctl = {8'b10100000, `SLT};
                ctl[`CTL_ALUCONTROL] = `ALU_SLT;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
                
            end
            6'b101011://SLTU
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_SLTU;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
                
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
            default: ctl = 0;
            endcase
        end
        
        //I-TYPE
        6'b000100://BEQ
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_SUB;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
            ctl[`CTL_REGWRITE] = 0;    
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_BRANCH;
        end
        6'b000101://BNE
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_SUB;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
            ctl[`CTL_REGWRITE] = 0;    
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_BRANCH;
        end
        6'b000110://BLEZ
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_SUB;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
            ctl[`CTL_REGWRITE] = 0;    
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_BRANCH;
        end
        6'b000111://BGTZ
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_SUB;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
            ctl[`CTL_REGWRITE] = 0;    
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_BRANCH;
        end

        6'b001000://ADDI
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001001://ADDIU
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADDU;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001010://SLTI
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_SLT;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001011://SLTIU
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_SLTU;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001100://ANDI
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_AND;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMMU;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001101://ORI
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_OR;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMMU;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001110://XORI
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_XOR;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMMU;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b001111://LUI
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_LUI;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        
        6'b011100:
        begin
            if (Opcode == 6'b000010)//MUL
            begin
                ctl[`CTL_ALUCONTROL] = `ALU_MUL;
                ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
                ctl[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                ctl[`CTL_REGWRITE] = 1;
                ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                ctl[`CTL_REGDST] = `REGDST_RTYPE;
               
                //ctl[`CTL_MEMREAD] = 0;
                ctl[`CTL_MEMWRITE] = 0;
                
                ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
            end
        end
        
        6'b100000://LB
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 1;
            ctl[`CTL_MEMWRITE] = 0;
            ctl[`CTL_MEMSIGNED] = 1;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b100001://LH
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 1;
            ctl[`CTL_MEMWRITE] = 0;
            ctl[`CTL_MEMSIGNED] = 1;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b100011://LW
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 1;
            ctl[`CTL_MEMWRITE] = 0;
            ctl[`CTL_MEMSIGNED] = 1;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_32;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b100100://LBU
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 1;
            ctl[`CTL_MEMWRITE] = 0;
            ctl[`CTL_MEMSIGNED] = 0;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b100101://LHU
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            ctl[`CTL_REGDST] = `REGDST_ITYPE;
               
            //ctl[`CTL_MEMREAD] = 1;
            ctl[`CTL_MEMWRITE] = 0;
            ctl[`CTL_MEMSIGNED] = 0;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b101000://SB
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 0;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 1;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b101001://SH
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 0;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 1;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b101011://SW
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 0;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 1;
            ctl[`CTL_MEMWIDTH] = `MEMWIDTH_32;
                
            ctl[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end
        6'b000010://J
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 0;
               
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_JUMPIMM;
        end
        
        6'b000011://JAL
        begin
            ctl[`CTL_ALUCONTROL] = `ALU_ADD;
            ctl[`CTL_ALUSRCA] = `ALUSRCA_RS;
            ctl[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            ctl[`CTL_REGWRITE] = 1;
            ctl[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_PCPLUS4;
            ctl[`CTL_REGDST] = `REGDST_RA;
            
            //ctl[`CTL_MEMREAD] = 0;
            ctl[`CTL_MEMWRITE] = 0;
                
            ctl[`CTL_PCSRC] = `PCSRC_JUMPIMM;
        end
        default: ctl = 0;
        endcase
endmodule


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
    input [3: 0] ALUControl;
    output ZF, OF, SF;
    output reg [31: 0] ALUResult;
    
    wire [4:0] shamt;
    wire exception;
    
    assign shamt = SrcA[4: 0];
    
    assign ZF = (ALUResult == 0)? 1: 0;
    assign SF = ALUResult[31];
    assign OF = ALUControl == `ALU_ADD ? (~(SrcA[31] ^ SrcB[31])) & (SrcA[31] ^ ALUResult[31])
        : ALUControl == `ALU_SUB ? (~SrcA[31] & SrcB[31] & ALUResult[31]) | (SrcA[31] & ~SrcB[31] & ~ALUResult[31])
        : 0;
    assign exception = (ALUControl == `ALU_ADD || ALUControl == `ALU_SUB) ? OF : 0;
    
    always @ ( * )
        case (ALUControl)
            `ALU_SLL: ALUResult = SrcB << shamt;//Shift Left
            `ALU_SRL: ALUResult = SrcB >> shamt;//Shift Right Logic
            `ALU_SRA: ALUResult = $signed(SrcB) >>> shamt;//Shift Right Arithmetic
            `ALU_LUI: ALUResult = {SrcB[15: 0], 16'b0};//Load Upper Immediate
            
            `ALU_MUL: ALUResult = $signed(SrcA) * $signed(SrcB);
            
            `ALU_AND: ALUResult = SrcA & SrcB;
            `ALU_OR: ALUResult = SrcA | SrcB;
            `ALU_ADD: ALUResult = SrcA + SrcB;
            `ALU_ADDU: ALUResult = SrcA + SrcB;
            `ALU_SUB: ALUResult = SrcA - SrcB;
            `ALU_SUBU: ALUResult = SrcA - SrcB;
            `ALU_XOR: ALUResult = SrcA ^ SrcB;//XOR
            `ALU_NOR: ALUResult = ~(SrcA | SrcB);//NOR

            `ALU_SLT: ALUResult = ($signed(SrcA) < $signed(SrcB));//Set if less than (signed)
            `ALU_SLTU: ALUResult = (SrcA < SrcB);//Set if less than (unsigned)
            default: ALUResult = 32'hxxxx_xxxx;
        endcase
endmodule
