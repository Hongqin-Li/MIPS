`timescale 1ns / 1ps
`define ROM t.imem.ROM

//IO Mapping
`define IN0_ADDR 16'd272
`define OUT0_ADDR 16'd256


`define INSTRS parameter SPECIAL = 6'b0;parameter SLL = 6'b000000, SRL = 6'b000010, SRA = 6'b000011, SLLV = 6'b000100, SRLV = 6'b000110, SRAV = 6'b000111, JR = 6'b001000, JALR = 6'b001001, ADD = 6'b100000, ADDU = 6'b100001, SUB = 6'b100010, SUBU = 6'b100011, AND = 6'b100100, OR = 6'b100101, XOR = 6'b100110, NOR = 6'b100111, SLT = 6'b101010, SLTU = 6'b101011, MUL = 6'b000010;parameter BEQ = 6'b000100, BNE = 6'b000101, BLEZ = 6'b000110, BGTZ = 6'b000111, ADDI = 6'b001000, ADDIU = 6'b001001, SLTI = 6'b001010, SLTIU = 6'b001011, ANDI = 6'b001100, ORI = 6'b001101, XORI = 6'b001110, LUI = 6'b001111, LB = 6'b100000, LH = 6'b100001, LW = 6'b100011, LBU = 6'b100100, LHU = 6'b100101, SB = 6'b101000, SH = 6'b101001, SW = 6'b101011;parameter J = 6'b000010, JAL = 6'b000011;
`define REGS parameter r0 = 5'b0, v0 = 5'b00010, v1 = 5'b00011, a0 = 5'b00100, a1 = 5'b00101, a2 = 5'b00110, a3 = 5'b00111, ra = 5'b11111, t0 = 5'b01000, t1 = 5'b01001, t2 = 5'b01010, t3 = 5'b01011, s0 = 5'h10, s1 = 5'h11, s2 = 5'h12, s3 = 5'h13, s4 = 5'h14, s5 = 5'h15, s6 = 5'h16, s7 = 5'h17, sp = 5'b11101;
    


module Hazard_tb;

    `INSTRS
    `REGS
    parameter HLT = {BEQ, v0, v0, 16'hffff};
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
    
    reg clk = 0;
    reg rst = 0;
    
    MIPSTop t(clk, rst);
    
    wire [31: 0] ALUResult = t.ALUResult, Instr = t.Instr;
    wire [31: 0] pc = t.PC[31:2];
    
    wire [31: 0] rf [0: 31];
    wire [31: 0] dmem = t.dmem.RAM[0];
    
    //wire [31: 0] test = rf[v0];
    wire [31: 0] reg_v0 = rf[v0];
    wire [31: 0] reg_v1 = rf[v1];
    
    
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.mips.dp.rf.RegCell[i];end
    endgenerate
    
    integer j;
    initial
    begin
       
        for (j = 0; j < 100; j = j + 1) `ROM[j] = 0;
        
        //Forward from Writeback
        `ROM[0] = {LUI, r0, v0, 16'hffff};//v0 = 0xffff0000
        `ROM[1] = {ORI, v0, v0, 16'hffff};//v0 = 0xffffffff
        `ROM[2] = {SPECIAL, v0, v0, v0, 5'b0, ADD};// v0 = 0xfffffffe
        
        //Forward from Memory, Stall
        `ROM[3] = {SW, r0, v0, 16'b0};
        `ROM[4] = {LW, r0, v1, 16'b0};//v1 = 0xfffffffe = -2
        `ROM[5] = {BLEZ, v1, r0, 16'd3};//Not taken
        
        //Control Hazard
        `ROM[9] = {BEQ, r0, r0, 16'hffff};//HLT
        `ROM[10] = {BEQ, r0, r0, 16'd0};//NOP
        
        
        rst = ~rst;
        #1 ;
        rst = ~rst;
        repeat (300) #1 clk = ~ clk;
    end




endmodule



module Instr_tb;
    
    `INSTRS
    `REGS
    parameter HLT = {BEQ, v0, v0, 16'hffff};
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
    
    reg clk = 0;
    reg rst = 0;
    
    MIPSTop t(clk, rst);
    
    wire [31: 0] ALUResult = t.ALUResult, Instr = t.Instr;
    wire [31: 0] pc = t.PC[31:2];
    
    wire [31: 0] rf [0: 31];
    wire [31: 0] dmem = t.dmem.RAM[0];
    
    wire [31: 0] test;

    
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.mips.dp.rf.RegCell[i];end
    endgenerate
    
    
    assign test = rf[v0];
    
    integer j;
    initial
    begin
        //Testing ALU slt and shift
        //for (j = 0; j < 32; j = j + 1) t.mips.dp.rf.RegCell[j] = j;
        for (j = 0; j < 100; j = j + 1) `ROM[j] = 0;
        
        
        //R-Type
        `ROM[0] = {ORI, r0, v0, 16'h0};// v0 = 0
        `ROM[1] = {LUI, r0, a0, 16'hfff3};// a0 = 0xfff30000
        `ROM[2] = {ORI, r0, a1, 16'h1};// a1 = 1
            
        `ROM[3] = {SPECIAL, r0, a0, v0, 5'd2, SLL};// v0 = a0 << 2 = 0xffcc0000
        `ROM[4] = {SPECIAL, r0, a0, v0, 5'd2, SRL};// v0 = a0 >> 2 = 0x3ffcc000
        `ROM[5] = {SPECIAL, r0, a0, v0, 5'd2, SRA};// v0 = a0 >>> 2 = 0xfffcc000
        
        `ROM[6] = {SPECIAL, a1, a0, v0, 5'd0, SLLV};// v0 = a0 << a1 = 0xffe60000
        `ROM[7] = {SPECIAL, a1, a0, v0, 5'd0, SRLV};// v0 = a0 >> a1 = 0x7ff98000
        `ROM[8] = {SPECIAL, a1, a0, v0, 5'd0, SRAV};// v0 = a0 >>> a1 = 0xfff98000
        
        
        
        
        `ROM[9] = {ORI, r0, v0, 16'h0};// v0 = 0
        `ROM[10] = {LUI, r0, a0, 16'hffff};// a0 = 0xffff0000
        `ROM[11] = {ORI, r0, a1, 16'h1};// a1 = 1
        `ROM[12] = {ORI, r0, a2, 16'h2};// a2 = 2
        `ROM[13] = {ORI, r0, a3, 16'h3};// a3 = 3
        
        `ROM[14] = {SPECIAL, a0, a0, v0, 5'd0, ADD};// v0 = a0 + a0 = 0xfffe0000 
        `ROM[15] = {SPECIAL, a1, a0, v0, 5'd0, ADD};// v0 = a1 + a0 = 0xffff0001
        `ROM[16] = {SPECIAL, a2, a0, v0, 5'd0, ADD};// v0 = a2 + a0 = 0xffff0002
        
        `ROM[17] = {SPECIAL, a0, a0, v0, 5'd0, ADDU};// v0 = a0 + a0 = 0xfffe0000
        `ROM[18] = {SPECIAL, a1, a0, v0, 5'd0, ADDU};// v0 = a1 + a0 = 0xffff0001
        `ROM[19] = {SPECIAL, a2, a0, v0, 5'd0, ADDU};// v0 = a2 + a0 = 0xffff0002
        
        `ROM[20] = {SPECIAL, a0, a0, v0, 5'd0, SUB};// v0 = a0 - a0 = 0
        `ROM[21] = {SPECIAL, a1, a0, v0, 5'd0, SUB};// v0 = a1 - a0 = 0x10001
        `ROM[22] = {SPECIAL, a2, a0, v0, 5'd0, SUB};// v0 = a2 - a0 = 0x10002
        
        `ROM[23] = {SPECIAL, a0, a0, v0, 5'd0, SUBU};// v0 = a0 - a0 = 0
        `ROM[24] = {SPECIAL, a1, a0, v0, 5'd0, SUBU};// v0 = a1 - a0 = 0x10001
        `ROM[25] = {SPECIAL, a2, a0, v0, 5'd0, SUBU};// v0 = a2 - a0 = 0x10002

        `ROM[26] = {SPECIAL, a0, a0, v0, 5'd0, AND};// v0 = 0xffff0000
        `ROM[27] = {SPECIAL, a1, a0, v0, 5'd0, AND};// v0 = 0
        `ROM[28] = {SPECIAL, a2, a0, v0, 5'd0, AND};// v0 = 0
        
        `ROM[29] = {SPECIAL, a0, a0, v0, 5'd0, OR};// v0 = 0xffff0000
        `ROM[30] = {SPECIAL, a1, a0, v0, 5'd0, OR};// v0 = 0xffff0001
        `ROM[31] = {SPECIAL, a2, a0, v0, 5'd0, OR};// v0 = 0xffff0002
        
        `ROM[32] = {SPECIAL, a0, a0, v0, 5'd0, XOR};// v0 = 0
        `ROM[33] = {SPECIAL, a1, a0, v0, 5'd0, XOR};// v0 = 0xffff0001
        `ROM[34] = {SPECIAL, a2, a0, v0, 5'd0, XOR};// v0 = 0xffff0002
        

        `ROM[35] = {ORI, r0, v0, 16'd0};// v0 = 0

        `ROM[36] = {SLTI, r0, v0, 16'hffff};// v0 = 0
        `ROM[37] = {SLTI, r0, v0, 16'h0};// v0 = 0
        `ROM[38] = {SLTI, r0, v0, 16'hff00};// v0 = 0
        `ROM[39] = {SLTI, r0, v0, 16'h1};// v0 = 1
        
        `ROM[40] = {ORI, r0, v0, 16'd0};// # v0 = 0
        `ROM[41] = {ORI, r0, a1, 16'd1};// # a1 = 1
        
        `ROM[42] = {SLTIU, a1, v0, 16'h0};// v0 = 0
        `ROM[43] = {SLTIU, a1, v0, 16'h1};// v0 = 0
        `ROM[44] = {SLTIU, a1, v0, 16'hff00};// v0 = 1
        
        
        
        //Branch
        `ROM[60] = {ORI, r0, v0, 16'd0};// # $v0 = 0
        `ROM[61] = {ORI, r0, a0, 16'd0};// # $a0 = 0
        `ROM[62] = {ORI, r0, a1, 16'd1};// # $a1 = 1
        `ROM[63] = {ORI, r0, a2, 16'd2};// # $a2 = 2
        `ROM[64] = {ADDI, r0, a3, 16'hffff};// # $a3 = -1
        
        `ROM[65] = {BEQ, r0, a1, 16'h1};// should not branch
        `ROM[66] = {BEQ, r0, a0, 16'h1};// should branch 
        `ROM[67] = {BEQ, r0, r0, 16'hffff};// # HLT
        
        `ROM[68] = {BNE, r0, a0, 16'h1};// # should not branch
        `ROM[69] = {BNE, r0, a1, 16'h1};// # should branch
        `ROM[70] = {BEQ, r0, r0, 16'hffff};// # HLT
        
        `ROM[71] = {BLEZ, a1, r0, 16'h2};// # greater than 0, should not branch
        `ROM[72] = {BLEZ, a2, r0, 16'h1};// # greater than 0, should not branch
        `ROM[73] = {BLEZ, a0, r0, 16'h1};// # equal to zero, should branch
        `ROM[74] = {BEQ, r0, r0, 16'hffff};// # HLT 
        `ROM[75] = {BLEZ, a3, r0, 16'h1};// # less than zero, should branch
        `ROM[76] = {BEQ, r0, r0, 16'hffff};// # HLT 
        
        `ROM[77] = {BGTZ, a0, r0, 16'h2};// # equal to 0, should not branch
        `ROM[78] = {BGTZ, a3, r0, 16'h1};// # less than 0, should not branch
        `ROM[79] = {BGTZ, a1, r0, 16'h1};// # greater than 0, should branch
        `ROM[80] = {BEQ, r0, r0, 16'hffff};// # HLT 
        
        
        //I-Type
        `ROM[81] = {ADDI, a0, v0, 16'h0};// # v0 = 0
        `ROM[82] = {ADDI, a1, v0, 16'hffff};// # v0 = 0
        `ROM[83] = {ADDI, a3, v0, 16'h1};// # v0 = 0
        
        `ROM[84] = {ADDIU, a0, v0, 16'h0};// # v0 = 0
        `ROM[85] = {ADDIU, a1, v0, 16'hffff};// # v0 = 0
        `ROM[86] = {ADDIU, a3, v0, 16'h1};// # v0 = 0
        
        `ROM[87] = {SLTI, a1, v0, 16'h0};// # v0 = 0
        `ROM[88] = {SLTI, a2, v0, 16'hffff};// # v0 = 0
        `ROM[89] = {SLTI, a3, v0, 16'h0};// # v0 = 1
        
        `ROM[90] = {SLTIU, a1, v0, 16'h0};// # v0 = 0
        `ROM[91] = {SLTIU, a2, v0, 16'hffff};// # v0 = 1
        `ROM[92] = {SLTIU, a3, v0, 16'h0};// # v0 = 0
        
        `ROM[93] = {ANDI, a0, v0, 16'h0};// # v0 = 0
        `ROM[94] = {ANDI, a1, v0, 16'hffff};// # v0 = 1
        `ROM[95] = {ANDI, a2, v0, 16'h1};// # v0 = 0
        
        `ROM[96] = {ORI, a0, v0, 16'h0};// # v0 = 0
        `ROM[97] = {ORI, a1, v0, 16'hffff};// # v0 = 0x0000ffff
        `ROM[98] = {ORI, a2, v0, 16'h1};// # v0 = 3
        
        `ROM[99] = {XORI, a0, v0, 16'h0};// # v0 = 0
        `ROM[100] = {XORI, a1, v0, 16'hffff};// # v0 = 0xfffe
        `ROM[101] = {XORI, a2, v0, 16'h1};// # v0 = 3
        
        
        
        `ROM[102] = {ORI, r0, v0, 16'h0};// # 0
        `ROM[103] = {ORI, r0, a1, 16'h1};// # 0xffff
        `ROM[104] = {ORI, r0, a2, 16'hf0f0};// # 0x0000f0f0
        `ROM[105] = {LUI, r0, a3, 16'habcd};// # 0xabcd0000
        `ROM[106] = {ORI, a3, a3, 16'h7faf};// # 0xabcd7faf
        
        
        `ROM[107] = {SB, r0, a3, 16'h0};// # 
        `ROM[108] = {SH, r0, a2, 16'h0};// # 
        `ROM[109] = {SW, r0, a3, 16'h0};// # 
        
        `ROM[110] = {LB, r0, v0, 16'h0};// # 0xffffffaf
        `ROM[111] = {LH, r0, v0, 16'h0};// # 0x00007faf
        `ROM[112] = {LBU, r0, v0, 16'h0};// # 0x00000af
        `ROM[113] = {LHU, r0, v0, 16'h0};// # 0x00007faf
        
        `ROM[114] = {LW, r0, v0, 16'h0};// # 0xabcdafaf
        
        `ROM[115] = {BEQ, r0, r0, 16'hffff};// # HLT
        

        //t.mips.dp.rf.RegCell[16] = 64;
        rst = ~rst;
        #20 ;
        rst = ~rst;
        repeat (300) #1 clk = ~ clk;
    end


endmodule



module MIPS_tb;
    `INSTRS
    `REGS
    parameter HLT = {BEQ, v0, v0, 16'hffff};
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
    
    reg clk = 0;
    reg rst = 0;
    MIPSTop t(clk, rst);
    
    wire [31: 0] ALUResult = t.ALUResult, Instr = t.Instr;
    wire [31: 0] pc = t.PC[31:2];
    wire [31: 0] pc_target = t.mips.dp.PCTarget[31:2];
    wire [31: 0] RD1 = t.mips.dp.ReadData1D;
    wire [31: 0] RD2 = t.mips.dp.ReadData2D;
    
    wire [31: 0] writeData = t.dmem.WriteData;
    wire [31: 0] writeAddr = t.dmem.Addr;
    
    wire [31: 0] rf [0: 31];
    
    wire [31: 0] test_in;
    wire [31: 0] test_out;
    wire taken = t.mips.dp.TakenD;
    wire BranchTaken = t.mips.dp.BranchTaken;
    
    assign test_in = t.dmem.RAM[`IN0_ADDR/4];
    assign test_out = t.dmem.RAM[`OUT0_ADDR/4];
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.mips.dp.rf.RegCell[i];end
    endgenerate
    
    initial
    begin
        t.mips.dp.rf.RegCell[16] = 64;
            
        `ROM[1] = {ADDI, r0, t0, 16'd0};
        `ROM[2] = {SB, r0, t0, `IN0_ADDR};
        
        rst = ~rst;
        #20 ;
        rst = ~rst;
        repeat (300) #2 clk = ~ clk;
    end
    
endmodule


