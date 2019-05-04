`timescale 1ns / 1ps


`define CTL_WIDTH 31:0
`define CTL_UNDEFINED 32'hxxxxxxxx//This will not happen!

`define CTL_ALUCONTROL 31:28//4bits
//ALU Decoder
`define ALUCONTROL_WIDTH 3:0
`define ALUOP_WIDTH 1:0
`define ALUOP_ADDU 2'd0
`define ALUOP_SUBU 2'd1
`define ALUOP_ALU 2'd2

`define CTL_ALUSRCA 27:26
`define ALUSRCA_RS 2'd0
`define ALUSRCA_SHAMT 2'd1
`define ALUSRCA_PC 2'd2//multi-cycle pc increment

`define CTL_ALUSRCB 25:23
`define ALUSRCB_RT 3'd0
`define ALUSRCB_IMM 3'd1
`define ALUSRCB_IMMU 3'd2//zero-extended immediate
`define ALUSRCB_IMM4 3'd3//multi-cycle pc increment
`define ALUSRCB_IMMSH2 3'd4//multi-cycle beq


`define CTL_REGDST 1:0
`define REGDST_RTYPE 2'd0
`define REGDST_ITYPE 2'd1
`define REGDST_RA 2'd2

`define CTL_REGWRITE 2

`define CTL_REGWRITE_DATA 4: 3
`define REGWRITE_DATA_FROM_ALU 2'd0
`define REGWRITE_DATA_FROM_MEMORY 2'd1
`define REGWRITE_DATA_FROM_PCPLUS4 2'd2

`define CTL_MEMREAD 5//Useless in single-cycle, but used in multi-cycle

`define CTL_MEMWRITE 6
`define CTL_MEMSIGNED 7
`define CTL_MEMWIDTH 9: 8
`define MEMWIDTH_32 2'd0
`define MEMWIDTH_16 2'd1
`define MEMWIDTH_8 2'd2

`define CTL_PCSRC 11: 10
`define PCSRC_PCPLUS4 2'b0
`define PCSRC_ALURESULT 2'b0
`define PCSRC_BRANCH 2'b1
`define PCSRC_JUMPIMM 2'd2
`define PCSRC_JUMPREG 2'd3

//For multicycle
`define CTL_PCWRITE 20
`define CTL_IRWRITE 22
`define CTL_IORD 21
`define IORD_IMEM 1'd0
`define IORD_DMEM 1'd1

`define STATE_FETCH 5'd0
`define STATE_DECODE 5'd1
`define STATE_EXECUTE 5'd2
`define STATE_MEMORY 5'd3
`define STATE_WRITEBACK 5'd4

`define STACK_BEGIN_ADDR 16'd256 

`define ROM t.mem.RAM




module Instr_tb;
    
    reg clk = 0;
    reg rst = 0;
    
    MIPSTop t(clk, rst);
    
    wire [31: 0] ALUResult = t.dp.alu.ALUResult;
    wire [31: 0] Instr = t.Instr;
    wire [31: 0] pc = t.dp.PCReg.q[31:2];
    
    wire [31: 0] rf [0: 31];
    wire [31: 0] mem = t.mem.RAM[125];
    wire [31: 0] mem2 = t.mem.RAM[126];
    
    
    wire [5: 0] state = t.cu.state;
    wire [31: 0] test;
    
    wire [`CTL_WIDTH] ctl = t.cu.CTL;
    //wire pc_write = ctl[`CTL_PCWRITE];
    //wire ir_write = ctl[`CTL_IRWRITE];
    wire mem_write = ctl[`CTL_MEMWRITE];
    wire [5:0] mem_signed = ctl[`CTL_MEMSIGNED];
    wire [5:0] mem_width = t.mem.Width;
    wire [5:0] ctl_mem_width = ctl[`CTL_MEMWIDTH];
    wire [31: 0] mem_write_data = t.mem.WriteData;
    wire [31: 0] mem_addr = t.mem.Addr;
    
    wire i_or_d = ctl[`CTL_IORD];
    //wire taken = t.dp.BranchSrc;
    wire [5:0] alu_srcA = ctl[`CTL_ALUSRCA];
    wire [5:0] alu_srcB = ctl[`CTL_ALUSRCB];
    wire [31: 0] alu_a = t.dp.srcA;
    wire [31: 0] alu_b = t.dp.srcB;
    wire [31: 0] ppc = t.dp.PC;
    //wire [5:0] reg_dst = ctl[`CTL_REGDST];
    //wire reg_write = ctl[`CTL_REGWRITE];
    wire pc_write = ctl[`CTL_PCWRITE];
    wire [5:0] ctl_op = t.cu.Opcode;
    wire [5:0] pc_src = ctl[`CTL_PCSRC];
    
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.dp.rf.RegCell[i];end
    endgenerate
    
    
    //R-Type
    parameter SPECIAL = 6'b0;
    parameter SLL = 6'b000000, SRL = 6'b000010, SRA = 6'b000011, SLLV = 6'b000100, SRLV = 6'b000110, SRAV = 6'b000111, JR = 6'b001000, JALR = 6'b001001, ADD = 6'b100000, ADDU = 6'b100001, SUB = 6'b100010, SUBU = 6'b100011, AND = 6'b100100, OR = 6'b100101, XOR = 6'b100110, NOR = 6'b100111, SLT = 6'b101010, SLTU = 6'b101011, MUL = 6'b000010;

    //I-Type
    parameter BEQ = 6'b000100, BNE = 6'b000101, BLEZ = 6'b000110, BGTZ = 6'b000111, ADDI = 6'b001000, ADDIU = 6'b001001, SLTI = 6'b001010, SLTIU = 6'b001011, ANDI = 6'b001100, ORI = 6'b001101, XORI = 6'b001110, LUI = 6'b001111, LB = 6'b100000, LH = 6'b100001, LW = 6'b100011, LBU = 6'b100100, LHU = 6'b100101, SB = 6'b101000, SH = 6'b101001, SW = 6'b101011;
    
    parameter J = 6'b000010, JAL = 6'b000011;
    
    // Registers 
    parameter r0 = 5'b0;
    parameter v0 = 5'b00010, v1 = 5'b00011;//Return values from function
    parameter a0 = 5'b00100, a1 = 5'b00101, a2 = 5'b00110, a3 = 5'b00111;//Arguments to function
    parameter ra = 5'b11111;//Return address register
    parameter t0 = 5'b01000, t1 = 5'b01001, t2 = 5'b01010, t3 = 5'b01011;//Temporary data
    parameter s0 = 5'h10, s1 = 5'h11, s2 = 5'h12, s3 = 5'h13, s4 = 5'h14, s5 = 5'h15, s6 = 5'h16, s7 = 5'h17;//Saved Registers, preserved by subprograms
    parameter sp = 5'b11101;//Stack Pointer
    
    
    parameter HLT = {BEQ, r0, r0, 16'hffff};
    
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
    
    assign test = rf[v0];
    
    integer j;
    initial
    begin
        //Testing ALU slt and shift
        //for (j = 0; j < 32; j = j + 1) t.mips.dp.rf.RegCell[j] = j;
        for (j = 0; j < 140; j = j + 1) `ROM[j] = 0;
        
        
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
        
        
        //Jump
        `ROM[45] = {ORI, r0, a0, 14'd51, 2'd0};
        `ROM[46] = {ORI, r0, a1, 14'd52, 2'd0};
        
        `ROM[47] = {SPECIAL, a0, 15'b0 ,JR};// # Jump to ROM[130]
        `ROM[48] = {JAL, 26'd52};//# Jump to ROM[131]
        `ROM[49] = {SPECIAL, a1, r0, ra, 5'd0, JALR};//# Jump to ROM[131]
        `ROM[50] = {J, 26'd60};//# Jump out
        
        `ROM[51] = {J, 26'd48};//# For non-link jump
        `ROM[52] = RET;//# For link jump
        
        
        
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
        
        
        `ROM[107] = {SB, r0, a3, 16'd500};// # 
        `ROM[108] = {SH, r0, a2, 16'd500};// # 
        `ROM[109] = {SW, r0, a3, 16'd500};// # 
        
        `ROM[110] = {LB, r0, v0, 16'd500};// # 0xffffffaf
        `ROM[111] = {LH, r0, v0, 16'd500};// # 0x00007faf
        `ROM[112] = {LBU, r0, v0, 16'd500};// # 0x00000af
        `ROM[113] = {LHU, r0, v0, 16'd500};// # 0x00007faf
        
        `ROM[114] = {LW, r0, v0, 16'd500};// # 0xabcd7faf
        
        //`ROM[115] = HLT;
        
        //Jump
        `ROM[116] = {ORI, r0, a0, 14'd130, 2'd0};
        `ROM[117] = {ORI, r0, a1, 14'd131, 2'd0};
        
        `ROM[118] = {SPECIAL, a0, 15'b0 ,JR};// # Jump to ROM[130]
        `ROM[119] = {JAL, 26'd131};//# Jump to ROM[131]
        `ROM[120] = {SPECIAL, a1, r0, ra, 5'd0, JALR};//# Jump to ROM[131]
        `ROM[121] = HLT;
        
        `ROM[130] = {J, 26'd119};//# For non-link jump
        `ROM[131] = RET;//# For link jump
        
        
        //t.mips.dp.rf.RegCell[16] = 64;
        rst = ~rst;
        #20 ;
        rst = ~rst;
        repeat (600) #1 clk = ~ clk;
    end


endmodule


module MIPS_tb;
    
    reg clk = 0;
    reg rst = 0;
    
    MIPSTop t(clk, rst);
    
    wire [31: 0] ALUResult = t.dp.alu.ALUResult;
    wire [31: 0] Instr = t.Instr;
    wire [31: 0] pc = t.dp.PCReg.q[31:2];
    
    wire [31: 0] rf [0: 31];
    wire [31: 0] r_v0 = rf[2];
    wire [31: 0] r_v1 = rf[3];
    wire [31: 0] r_a0 = rf[4];
    wire [31: 0] r_a1 = rf[5];
    wire [31: 0] r_a2 = rf[6];
    wire [31: 0] r_a3 = rf[7];
    wire [31: 0] r_t0 = rf[8];
    wire [31: 0] r_t1 = rf[9];
    wire [31: 0] r_s0 = rf[16];
    
    wire [5: 0] state = t.cu.state;
    wire [31: 0] test = t.mem.RAM[`OUT0_ADDR / 4 ];
    
    wire [`CTL_WIDTH] ctl = t.cu.CTL;
    //wire pc_write = ctl[`CTL_PCWRITE];
    //wire ir_write = ctl[`CTL_IRWRITE];
    wire [5: 0] reg_write = ctl[`CTL_REGWRITE];
    wire [5: 0] reg_src = ctl[`CTL_REGWRITE_DATA];
    wire mem_write = ctl[`CTL_MEMWRITE];
    wire [5:0] mem_signed = ctl[`CTL_MEMSIGNED];
    wire [5:0] mem_width = t.mem.Width;
    wire [5:0] ctl_mem_width = ctl[`CTL_MEMWIDTH];
    wire [31: 0] mem_write_data = t.mem.WriteData;
    wire [31: 0] mem_addr = t.mem.Addr;
    wire [31: 0] mem_out = t.mem.ReadData;
    wire i_or_d = ctl[`CTL_IORD];
    //wire taken = t.dp.BranchSrc;
    wire [31: 0] alu_a = t.dp.srcA;
    wire [31: 0] alu_b = t.dp.srcB;
    wire [31: 0] ppc = t.dp.PC[31: 2];
    //wire [5:0] reg_dst = ctl[`CTL_REGDST];
    //wire reg_write = ctl[`CTL_REGWRITE];
    
    wire [5:0] pc_src = ctl[`CTL_PCSRC];
    
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.dp.rf.RegCell[i];end
    endgenerate
    
    
    //R-Type
    parameter SPECIAL = 6'b0;
    parameter SLL = 6'b000000, SRL = 6'b000010, SRA = 6'b000011, SLLV = 6'b000100, SRLV = 6'b000110, SRAV = 6'b000111, JR = 6'b001000, JALR = 6'b001001, ADD = 6'b100000, ADDU = 6'b100001, SUB = 6'b100010, SUBU = 6'b100011, AND = 6'b100100, OR = 6'b100101, XOR = 6'b100110, NOR = 6'b100111, SLT = 6'b101010, SLTU = 6'b101011, MUL = 6'b000010;

    //I-Type
    parameter BEQ = 6'b000100, BNE = 6'b000101, BLEZ = 6'b000110, BGTZ = 6'b000111, ADDI = 6'b001000, ADDIU = 6'b001001, SLTI = 6'b001010, SLTIU = 6'b001011, ANDI = 6'b001100, ORI = 6'b001101, XORI = 6'b001110, LUI = 6'b001111, LB = 6'b100000, LH = 6'b100001, LW = 6'b100011, LBU = 6'b100100, LHU = 6'b100101, SB = 6'b101000, SH = 6'b101001, SW = 6'b101011;
    
    parameter J = 6'b000010, JAL = 6'b000011;
    
    // Registers 
    parameter r0 = 5'b0;
    parameter v0 = 5'b00010, v1 = 5'b00011;//Return values from function
    parameter a0 = 5'b00100, a1 = 5'b00101, a2 = 5'b00110, a3 = 5'b00111;//Arguments to function
    parameter ra = 5'b11111;//Return address register
    parameter t0 = 5'b01000, t1 = 5'b01001, t2 = 5'b01010, t3 = 5'b01011;//Temporary data
    parameter s0 = 5'h10, s1 = 5'h11, s2 = 5'h12, s3 = 5'h13, s4 = 5'h14, s5 = 5'h15, s6 = 5'h16, s7 = 5'h17;//Saved Registers, preserved by subprograms
    parameter sp = 5'b11101;//Stack Pointer
    
    
    parameter HLT = {BEQ, v0, v0, 16'hffff};
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
    
    
    integer j;
    initial
    begin
        //Testing ALU slt and shift
        //for (j = 0; j < 32; j = j + 1) t.mips.dp.rf.RegCell[j] = j;
        for (j = 0; j < 100; j = j + 1) `ROM[j] = 0;

        `ROM[0] = {ADDI, r0, sp, `STACK_BEGIN_ADDR};//Stack pointer 256
        
            
        `ROM[1] = {ADDI, r0, t0, 16'd0};
        `ROM[2] = {SB, r0, t0, `IN0_ADDR};
            
        //Read seed from IO device
        `ROM[3] = {LBU, r0, s0, `IN0_ADDR};
            
        //loop:
        `ROM[10] = {ORI, s0, a1, 16'd0};// retrieve seed
        `ROM[11] = {ADDI, r0, a0, 16'd17};// a = 17
        `ROM[12] = {ADDI, r0, a2, 16'd3};// c = 3
        `ROM[13] = {ADDI, r0, a3, 16'd256};// m = 256
            
        `ROM[14] = {JAL, 26'd50};// call lcg()
            
        `ROM[15] = {ORI, v0, s0, 16'b0};
        `ROM[16] = {SB, r0, v0, `OUT0_ADDR};
        `ROM[17] = {BEQ, r0, r0, 16'hfff8};//loop
            
            
        //linear congruential generator  
        //int lcg(a, seed, c, m) {return seed = (a * seed + c) % m;}
        `ROM[50] = {6'b011100, a1, a0, t0, 5'b00000, 6'b000010};//MUL
        `ROM[51] = {6'b0, a2, t0, t0, 5'b00000, ADDU};
        `ROM[52] = {ORI, t0, a0, 16'b0};
        `ROM[53] = {6'b000000, r0, a3, a1, 5'b0, OR};
        //Push
        `ROM[54] = {ADDI, sp, sp, 16'hfffc};// sp -= 4
        `ROM[55] = {SW, sp, ra, 16'b0};
        //Push end
            
        //Testing JALR
        `ROM[56] = {ORI, r0, t2, 14'd80, 2'd0};
        `ROM[57] = {6'b0, t2, r0, ra, 5'b0, JALR};
        //`ROM[56] = {JAL, 26'd80};//call mod()
            
        //Pop
        `ROM[58] = {LW, sp, ra, 16'b0};
        `ROM[59] = {ADDI, sp, sp, 16'h4};// sp -= 4
        //Pop end
        `ROM[60] = RET;
            
        // int mod(a0, a1) {return v0 = a0 % a1;} 
        `ROM[80] = {ADDI, a0, v0, 16'b0};//ADDI $2, $4, 0 # $1 <= $2
        `ROM[81] = {6'b0, a1, v0, t0, 5'b0, SUB};//SUB $8, $3, $2
        `ROM[82] = {6'b0, r0, v0, t1, 5'b0, SUB};
        `ROM[83] = {BLEZ, t0, 5'b0, 16'b10};// # if c - b <= 0, then branch 1 (b -= c)
        `ROM[84] = {BGTZ, t1, 5'b0, 16'b11};// # else if b < 0, then branch 2 (b += c)
        `ROM[85] = RET;// # else return 
        `ROM[86] = {6'b0, v0, a1, v0, 5'b0, SUB};//SUB $1, $1, $3 # branch 1
        `ROM[87] = {BEQ, r0, r0, 16'hfff9};// B -0x7 # loop
        `ROM[88] = {6'b0, v0, a1, v0, 5'b0, ADD};//ADD $1, $1, $3 # branch 2
        `ROM[89] = {BEQ, r0, r0, 16'hfff7};// B -0x9 # loop           

        //t.mips.dp.rf.RegCell[16] = 64;
        rst = ~rst;
        #20 ;
        rst = ~rst;
        repeat (800) #1 clk = ~ clk;
    end


endmodule


