/*
1. write Register in posedge and read in negtive edge

2. interrupt


*/

`timescale 1ns / 1ps


`include "cache.v"

//For writing instruction code
`define INSTRS parameter SPECIAL = 6'b0;parameter SLL = 6'b000000, SRL = 6'b000010, SRA = 6'b000011, SLLV = 6'b000100, SRLV = 6'b000110, SRAV = 6'b000111, JR = 6'b001000, JALR = 6'b001001, ADD = 6'b100000, ADDU = 6'b100001, SUB = 6'b100010, SUBU = 6'b100011, AND = 6'b100100, OR = 6'b100101, XOR = 6'b100110, NOR = 6'b100111, SLT = 6'b101010, SLTU = 6'b101011, MUL = 6'b000010;parameter BEQ = 6'b000100, BNE = 6'b000101, BLEZ = 6'b000110, BGTZ = 6'b000111, ADDI = 6'b001000, ADDIU = 6'b001001, SLTI = 6'b001010, SLTIU = 6'b001011, ANDI = 6'b001100, ORI = 6'b001101, XORI = 6'b001110, LUI = 6'b001111, LB = 6'b100000, LH = 6'b100001, LW = 6'b100011, LBU = 6'b100100, LHU = 6'b100101, SB = 6'b101000, SH = 6'b101001, SW = 6'b101011;parameter J = 6'b000010, JAL = 6'b000011;
`define REGS parameter r0 = 5'b0, v0 = 5'b00010, v1 = 5'b00011, a0 = 5'b00100, a1 = 5'b00101, a2 = 5'b00110, a3 = 5'b00111, ra = 5'b11111, t0 = 5'b01000, t1 = 5'b01001, t2 = 5'b01010, t3 = 5'b01011, s0 = 5'h10, s1 = 5'h11, s2 = 5'h12, s3 = 5'h13, s4 = 5'h14, s5 = 5'h15, s6 = 5'h16, s7 = 5'h17, sp = 5'b11101;



`define OP_ALU 6'b0
`define OP_SPECIAL 6'b0
`define OP_MUL 6'b011100


`define FUNC_SLL 6'b000000
`define FUNC_SRL 6'b000010
`define FUNC_SRA 6'b000011
`define FUNC_SLLV 6'b000100
`define FUNC_SRLV 6'b000110
`define FUNC_SRAV 6'b000111
`define FUNC_JR 6'b001000
`define FUNC_JALR 6'b001001
`define FUNC_ADD 6'b100000
`define FUNC_ADDU 6'b100001
`define FUNC_SUB 6'b100010
`define FUNC_SUBU 6'b100011
`define FUNC_AND 6'b100100
`define FUNC_OR 6'b100101
`define FUNC_XOR 6'b100110
`define FUNC_NOR 6'b100111
`define FUNC_SLT 6'b101010
`define FUNC_SLTU 6'b101011
`define FUNC_MUL 6'b000010

//I-Type
`define OP_BEQ 6'b000100
`define OP_BNE 6'b000101
`define OP_BLEZ 6'b000110
`define OP_BGTZ 6'b000111
`define OP_ADDI 6'b001000
`define OP_ADDIU 6'b001001
`define OP_SLTI 6'b001010
`define OP_SLTIU 6'b001011
`define OP_ANDI 6'b001100
`define OP_ORI 6'b001101
`define OP_XORI 6'b001110
`define OP_LUI 6'b001111
`define OP_LB 6'b100000
`define OP_LH 6'b100001
`define OP_LW 6'b100011
`define OP_LBU 6'b100100
`define OP_LHU 6'b100101
`define OP_SB 6'b101000
`define OP_SH 6'b101001
`define OP_SW 6'b101011

`define OP_J 6'b000010
`define OP_JAL 6'b000011

`define IS_JUMP(Opcode, Func) (Opcode == `OP_J || Opcode == `OP_JAL || (Opcode == `OP_SPECIAL && (Func == `FUNC_JR|| Func == `FUNC_JALR)) )

`define CTL_WIDTH 31:0
`define CTL_WIDTH_NUM 32

`define CTL_ALUCONTROL 31:28//4bits
`define ALUCONTROL_WIDTH 3:0

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
`define CTL_MEMWIDTH 9: 8//Identical to cache!!
`define MEMWIDTH_8 `REQUEST_WRITE_BYTE
`define MEMWIDTH_16 `REQUEST_WRITE_HALF
`define MEMWIDTH_32 `REQUEST_WRITE_WORD

/*
`define MEMWIDTH_8 2'd2
`define MEMWIDTH_16 2'd1
`define MEMWIDTH_32 2'd0
*/


`define CTL_PCSRC 11: 10
`define PCSRC_PCPLUS4 2'b0
`define PCSRC_BRANCH 2'b1
`define PCSRC_JUMPIMM 2'd2
`define PCSRC_JUMPREG 2'd3



//For Branch comparison in Decode
`define CTL_BRANCH 14: 12
`define BRANCH_NONE 3'd0
`define BRANCH_BEQ 3'd1
`define BRANCH_BNE 3'd2
`define BRANCH_BLEZ 3'd3
`define BRANCH_BGTZ 3'd4



//For Pipeline
`define HCTL_WIDTH 31: 0

`define HCTL_STALLF 0
`define HCTL_STALLD 1
`define HCTL_STALLE 2
`define HCTL_STALLM 3
`define HCTL_STALLW 4

`define HCTL_FLUSHF 5
`define HCTL_FLUSHD 6
`define HCTL_FLUSHE 7
`define HCTL_FLUSHM 8
`define HCTL_FLUSHW 9

// Forward from Execute to Decode
`define HCTL_FORWARDAD 10
`define HCTL_FORWARDBD 11

// Forward from Memory or Writeback to Execute
`define HCTL_FORWARDAE 13: 12
`define HCTL_FORWARDBE 15: 14
`define FORWARDE_WIDTH 1: 0

`define FORWARDE_NONE 2'd0
`define FORWARDE_FROM_MEMORY 2'd1
`define FORWARDE_FROM_WRITEBACK 2'd2



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



`define DMEM_MAX_ADDR 16'd512
`define IMEM_MAX_ADDR 16'd512

`define STACK_BEGIN_ADDR 16'd128 


//IO Mapping
`define IN0_ADDR 16'd272
`define OUT0_ADDR 16'd160



module top (CLK100MHZ, SW, AN, CA, CB, CC, CD, CE, CF, CG);
    
    input CLK100MHZ;
    input [15: 0] SW;
    
    output [7: 0] AN;
    output CA, CB, CC, CD, CE, CF, CG;
    
    
    
    reg [31: 0] data;
    wire [6: 0] C;
    wire clk, clk1_4hz, rst;
    
    parameter blocki = `OUT0_ADDR / (1<<`BLOCK_OFFSET_WIDTH_NUM);
    parameter block_offset = `OUT0_ADDR % (1<<`BLOCK_OFFSET_WIDTH_NUM);
    
    clkdiv cd(.mclk(CLK100MHZ), .clk(clk),.clk1_4hz(clk1_4hz));
    
    MIPSTop mt(SW[3] ? clk1_4hz|SW[2] : clk|SW[2], rst);//Stopping, running or running in low speed
    
    
    wire [`BLOCK_DATA_WIDTH] out_block;
    
    wire [7:0] out = out_block[block_offset+7:block_offset];
    //wire [7: 0] out = mt.dmem.mem.RAM[`OUT0_ADDR/4];
    
    wire [31: 0] pc = mt.PC;
    
    wire [3: 0] d0 = out % 10;
    wire [3: 0] d1 = (out / 10) % 10;
    wire [3: 0] d2 = (out / 100) % 10;
    wire [3: 0] d3 = 4'b0;
    wire [3: 0] d4 = pc % 10;
    wire [3: 0] d5 = (pc / 10) % 10;
    wire [3: 0] d6 = (pc / 100) % 10;
    wire [3: 0] d7 = (pc / 1000) % 10;
    
    
    Display dis(CLK100MHZ, AN, C, 
        {d7, d6, d5, d4, d3, d2, d1, d0});
        
    assign {CA, CB, CC, CD, CE, CF, CG} = C;
    assign rst = SW[0];
    
    // Input Mapping 
    assign mt.in0_data = SW[15:4];
    assign out_block = mt.dmem.mem.RAM[blocki];
    //assign mt.dmem.iwrite = SW[0] & SW[1];
    //assign mt.dmem.iaddr = `IN0_ADDR;
    //assign mt.dmem.idata = SW[15:4];
    
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
    output clk1_4hz,
    output clk
    );
    reg [27:0]q;
    always @ (posedge mclk)
        q <= q + 1;

    assign clk380 = q[17];//380hz
    //assign clk6000 = q[13];//6000hz 
    //assign clk190 = q[18];//190hz 
    assign clk1_4hz=q[24];         
    assign clk = q[20];
    //assign clk1_1hz=q[26];         
endmodule


module MIPSTop (CLK, RST);

    input CLK, RST;
    
    wire [31: 0] PC, MemAddr, MemWriteData, MemReadData, ReadData;
    wire [31: 0] Instr;
    wire MemWrite, MemSigned;
    wire [1: 0] MemWidth;
    wire [31: 0] ALUResult = MemAddr;
    
    MIPS mips(
        CLK, RST,
        PC, Instr,
        MemWrite, MemSigned, MemWidth, MemAddr, MemWriteData, ReadData);
    
    IMem imem(PC, Instr);
    
    //DMem dmem(CLK, MemAddr, MemWrite, MemWriteData, MemReadData, MemSigned, MemWidth);
    
    //Memory Mapped IO, in0 is read-only 
    wire isIn0 = MemAddr == `IN0_ADDR;
    wire [31: 0] in0_data;
    assign ReadData = isIn0 ? in0_data : MemReadData;
    
    wire dmemReady;
    assign mips.dp.dmem_ready = isIn0 | dmemReady;

    RAMTop dmem(CLK, mips.dp.MemValid & ~isIn0, MemAddr, MemWrite, MemWriteData, MemReadData, MemSigned, MemWidth, dmemReady);

endmodule

module IMem (
    input [31: 0] Addr,
    output [31: 0] Instr);
    
    `INSTRS
    `REGS
    parameter HLT = {6'b000100, 10'b0, 16'hffff};
    parameter RET = {6'b0, ra, 10'b0, 5'b0, JR};
    
    
    reg [31: 0] ROM[`IMEM_MAX_ADDR/4 - 1: 0];
    assign Instr = ROM[Addr[31:2]];
    
    integer i;
    initial
    begin
        for (i = 0; i < `IMEM_MAX_ADDR/4; i = i + 1) ROM[i] = 0;
        
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


module MIPS (
    input CLK, RST,
    //IMEM
    output [31:0] PCF,
    input [31: 0] InstrF,
    //DMEM
    output MemWrite, MemSigned,
    output [1: 0] MemWidth,
    output [31: 0] MemAddr, MemWriteData,
    input [31: 0] MemReadData);

    
    wire [`HCTL_WIDTH] HCTL;
    wire [`CTL_WIDTH] CTL_D, CTL_E, CTL_W, CTL_M;
    wire [5: 0] OpcodeD, FuncD;
    
    assign MemWidth = CTL_M[`CTL_MEMWIDTH];
    assign MemSigned = CTL_M[`CTL_MEMSIGNED];
    assign MemWrite = CTL_M[`CTL_MEMWRITE];

    Controller cu(CLK, RST, HCTL, OpcodeD, FuncD, CTL_D, CTL_E, CTL_M, CTL_W);
    Datapath dp(CLK, RST, OpcodeD, FuncD, HCTL, CTL_D, CTL_E, CTL_M, CTL_W, PCF, InstrF, MemAddr, MemWriteData, MemReadData);
    
    
endmodule



module ALUDecoder(Opcode, Func, ALUControl);

    input [5: 0] Opcode, Func;
    output [`ALUCONTROL_WIDTH] ALUControl;

    reg [`ALUCONTROL_WIDTH] ctl;
    assign ALUControl = ctl;

    always @ (*)
    begin
        ctl = `ALU_ADDU;
        case (Opcode)
        `OP_SPECIAL:
            case (Func)
			`FUNC_SLL: ctl = `ALU_SLL;
			`FUNC_SRL: ctl = `ALU_SRL;
			`FUNC_SRA: ctl = `ALU_SRA;
			`FUNC_SLLV: ctl = `ALU_SLL;
			`FUNC_SRLV: ctl = `ALU_SRL;
			`FUNC_SRAV: ctl = `ALU_SRA;
			`FUNC_JR: ctl = `ALU_ADDU;
			`FUNC_JALR: ctl = `ALU_ADDU;
			`FUNC_ADD: ctl = `ALU_ADD;
			`FUNC_ADDU: ctl = `ALU_ADDU;
			`FUNC_SUB: ctl = `ALU_SUB;
			`FUNC_SUBU: ctl = `ALU_SUBU;
			`FUNC_AND: ctl = `ALU_AND;
			`FUNC_OR: ctl = `ALU_OR;
			`FUNC_XOR: ctl = `ALU_XOR;
			`FUNC_NOR: ctl = `ALU_NOR;
			`FUNC_SLT: ctl = `ALU_SLT;
			`FUNC_SLTU: ctl = `ALU_SLTU;
            endcase

        `OP_MUL: 
            if (Func == 6'b000010) ctl = `ALU_MUL;
            else ctl = `ALU_ADDU;

        `OP_BEQ, `OP_BNE, `OP_BLEZ, `OP_BGTZ: ctl = `ALU_SUBU;
        `OP_ADDI: ctl = `ALU_ADD;
        `OP_ADDIU: ctl = `ALU_ADDU;
        `OP_SLTI: ctl = `ALU_SLT;
        `OP_SLTIU: ctl = `ALU_SLTU;
        `OP_ANDI: ctl = `ALU_AND;
        `OP_ORI: ctl = `ALU_OR;
        `OP_XORI: ctl = `ALU_XOR;
        `OP_LUI: ctl = `ALU_LUI;

        default: ctl = `ALU_ADDU;
        endcase
    end
endmodule



module ControlUnit (Opcode, Func, CTL);

    input [5: 0] Func, Opcode;
    output reg [`CTL_WIDTH] CTL;
    
    wire [`ALUCONTROL_WIDTH] ALUCtl;

    ALUDecoder alu_decoder(Opcode, Func, ALUCtl);
    
    always @ (*)
    begin

        CTL = 0;
        CTL[`CTL_ALUCONTROL] = ALUCtl;
        CTL[`CTL_PCSRC] = `PCSRC_PCPLUS4;

        case (Opcode)
        
        `OP_SPECIAL: 
        begin

            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            CTL[`CTL_REGDST] = `REGDST_RTYPE;

            case (Func)

            `FUNC_SLL, `FUNC_SRL, `FUNC_SRA:
                CTL[`CTL_ALUSRCA] = `ALUSRCA_SHAMT;
            
            `FUNC_JR:
            begin
                CTL[`CTL_REGWRITE] = 0;
                CTL[`CTL_PCSRC] = `PCSRC_JUMPREG;
            end

            `FUNC_JALR:
            begin
                CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_PCPLUS4;
                CTL[`CTL_REGDST] = `REGDST_RA;
                CTL[`CTL_PCSRC] = `PCSRC_JUMPREG;
            end

            default: ;

            endcase

        end
        
        `OP_BEQ:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
            CTL[`CTL_BRANCH] = `BRANCH_BEQ;
            CTL[`CTL_PCSRC] = `PCSRC_BRANCH;
        end
        `OP_BNE:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
            CTL[`CTL_BRANCH] = `BRANCH_BNE;
            CTL[`CTL_PCSRC] = `PCSRC_BRANCH;
        end
        `OP_BLEZ:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
            CTL[`CTL_BRANCH] = `BRANCH_BLEZ;
            CTL[`CTL_PCSRC] = `PCSRC_BRANCH;
        end
        `OP_BGTZ:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
            CTL[`CTL_BRANCH] = `BRANCH_BGTZ;
            CTL[`CTL_PCSRC] = `PCSRC_BRANCH;
        end

        `OP_ADDI, `OP_ADDIU, `OP_SLTI, `OP_SLTIU, `OP_LUI:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
        end

        `OP_ANDI, `OP_ORI, `OP_XORI: 
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMMU;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
        end
        
        `OP_MUL:
        begin
            if (Func == `FUNC_MUL)//MUL
            begin
                CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
                CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
                
                CTL[`CTL_REGWRITE] = 1;
                CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
                CTL[`CTL_REGDST] = `REGDST_RTYPE;
            end
        end
        
        `OP_LB:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
            
            CTL[`CTL_MEMREAD] = 1;
            CTL[`CTL_MEMSIGNED] = 1;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_8;
        end
        `OP_LH:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
            
            CTL[`CTL_MEMREAD] = 1;   
            CTL[`CTL_MEMSIGNED] = 1;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_16;
        end
        `OP_LW:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
            
            CTL[`CTL_MEMREAD] = 1;
            CTL[`CTL_MEMSIGNED] = 1;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_32;
        end
        `OP_LBU:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
               
            CTL[`CTL_MEMREAD] = 1;
            CTL[`CTL_MEMSIGNED] = 0;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                
        end
        `OP_LHU:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;
            
            
            CTL[`CTL_MEMREAD] = 1;   
            CTL[`CTL_MEMSIGNED] = 0;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                
            CTL[`CTL_PCSRC] = `PCSRC_PCPLUS4;
        end

        `OP_SB:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_MEMWRITE] = 1;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_8;
        end
        `OP_SH:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_MEMWRITE] = 1;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_16;
        end
        `OP_SW:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
            CTL[`CTL_MEMWRITE] = 1;
            CTL[`CTL_MEMWIDTH] = `MEMWIDTH_32;
        end
        `OP_J:
        begin
            CTL[`CTL_PCSRC] = `PCSRC_JUMPIMM;
        end
        
        `OP_JAL:
        begin
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_PCPLUS4;//PC + 4
            CTL[`CTL_REGDST] = `REGDST_RA;
            
            CTL[`CTL_PCSRC] = `PCSRC_JUMPIMM;
        end
        default: ;
        endcase
    end
endmodule



module RegisterFile (CLK, WriteEnable, WriteData, WriteAddr, ReadAddr1, ReadAddr2, ReadData1, ReadData2);

    input CLK, WriteEnable;
    input [4: 0] ReadAddr1, ReadAddr2, WriteAddr;
    input [31: 0] WriteData;

    output [31: 0] ReadData1, ReadData2;

    reg [31: 0] RegCell [0: 31];
    
    assign ReadData1 = (ReadAddr1 != 0) ? RegCell[ReadAddr1] : 0;
    assign ReadData2 = (ReadAddr2 != 0) ? RegCell[ReadAddr2] : 0;

    always @ (negedge CLK)
        if (WriteEnable) RegCell[WriteAddr] <= WriteData;
    
    //initialization
    integer i;
    initial
    begin
    for (i = 0; i < 32; i = i + 1) RegCell[i] = 0;
    end
    
endmodule



module Controller(
    input CLK, RST, 
    input [`HCTL_WIDTH] HCTL, 
    input [5: 0] OpcodeD, FuncD,
    output [`CTL_WIDTH] CTL_D, CTL_E, CTL_M, CTL_W);

    ControlUnit cu(OpcodeD, FuncD, CTL_D);
    //assign PCSrc = Taken;

    //Pipeline Registers
    //floprc #(`CTL_WIDTH_NUM) regE(CLK, RST, HCTL[`HCTL_FLUSHE], CTL_D, CTL_E);
    //flopr #(`CTL_WIDTH_NUM) regM(CLK, RST, CTL_E, CTL_M);
    //flopr #(`CTL_WIDTH_NUM) regW(CLK, RST, CTL_M, CTL_W);
    flopenrc #(`CTL_WIDTH_NUM) regE(CLK, RST, ~HCTL[`HCTL_STALLE], HCTL[`HCTL_FLUSHE], CTL_D, CTL_E);
    flopenr #(`CTL_WIDTH_NUM) regM(CLK, RST, ~HCTL[`HCTL_STALLM], CTL_E, CTL_M);
    floprc #(`CTL_WIDTH_NUM) regW(CLK, RST, HCTL[`HCTL_FLUSHW], CTL_M, CTL_W);

endmodule 


module Datapath(
    input CLK, RST,
    //Control Unit
    output [5: 0] OpcodeD, FuncD,
    output [`HCTL_WIDTH] HCTL,
    input [`CTL_WIDTH] CTL_D, CTL_E, CTL_M, CTL_W, 

    //IMEM
    output [31: 0] PCF,
    input [31: 0] InstrF,

    //DMEM
    output [31: 0] MemAddrM, MemWriteDataM,
    input [31: 0] MemReadDataM);

    //Hazard Control
    wire StallF = HCTL[`HCTL_STALLF];
    wire StallD = HCTL[`HCTL_STALLD];
    wire StallE = HCTL[`HCTL_STALLE];
    wire StallM = HCTL[`HCTL_STALLM];
    wire FlushE = HCTL[`HCTL_FLUSHE];
    wire FlushD = HCTL[`HCTL_FLUSHD];
    wire FlushW = HCTL[`HCTL_FLUSHW];
    wire ForwardAD = HCTL[`HCTL_FORWARDAD];
    wire ForwardBD = HCTL[`HCTL_FORWARDBD];
    wire [`FORWARDE_WIDTH] ForwardAE = HCTL[`HCTL_FORWARDAE];
    wire [`FORWARDE_WIDTH] ForwardBE = HCTL[`HCTL_FORWARDBE];


    wire [31: 0] PCPlus4F, PCPlus4D, PCPlus4E, PCPlus4W;//For JAL and JALR
    wire [31: 0] ALUOutM;
    
    //Fetch Stage
    wire [31: 0] PCNextF;
    assign PCPlus4F = PCF + 4;
    flopenr #(32) pcreg(CLK, RST, ~StallF, PCNextF, PCF);

    /* Decode Stage */
    wire [31: 0] InstrD;
    wire [31: 0] ReadData1, ReadData2;//Raw ReadData from Register File
    wire [31: 0] ImmD;//Selected from signImm and unsignImm
    wire [4: 0] RsD, RtD, RdD, ShamtD;
    wire [31: 0] PCTarget;

    //FIXME flopenrc?
    flopenrc #(32) r1D(CLK, RST, ~StallD, FlushD, PCPlus4F, PCPlus4D);
    flopenrc #(32) r2D(CLK, RST, ~StallD, FlushD, InstrF, InstrD);

    wire [4: 0] WriteRegW;
    wire [31:0] ResultW;
    //RegisterFile rf(CLK, CTL_W[`CTL_REGWRITE], RsD, RtD, WriteRegW, ResultW, ReadData1, ReadData2);
    RegisterFile rf(CLK, CTL_W[`CTL_REGWRITE], ResultW, WriteRegW, RsD, RtD, ReadData1, ReadData2);

    assign OpcodeD = InstrD[31: 26];
    assign FuncD = InstrD[5: 0];


    wire [31: 0] signImmD = {{16{InstrD[15]}}, InstrD[15: 0]};
    wire [31: 0] unsignImmD = {16'b0, InstrD[15: 0]};

    //ForwardD
    wire [31: 0] ReadData1D = ForwardAD ? ALUOutM : ReadData1;
    wire [31: 0] ReadData2D = ForwardBD ? ALUOutM : ReadData2;
    
    wire ZF = ReadData1D == 0;
    wire SF = ReadData1D[31] == 1;
    wire LEZ = SF | ZF;
    
    wire BranchTaken
        = CTL_D[`CTL_BRANCH] == `BRANCH_BEQ ? ReadData1D == ReadData2D
        : CTL_D[`CTL_BRANCH] == `BRANCH_BNE ? ReadData1D != ReadData2D
        : CTL_D[`CTL_BRANCH] == `BRANCH_BGTZ ? ~ LEZ
        : CTL_D[`CTL_BRANCH] == `BRANCH_BLEZ ? LEZ
        : 1'b0;

    wire TakenD
        = CTL_D[`CTL_PCSRC] == `PCSRC_JUMPIMM ? 1'b1
        : CTL_D[`CTL_PCSRC] == `PCSRC_JUMPREG ? 1'b1
        : CTL_D[`CTL_PCSRC] == `PCSRC_BRANCH ? BranchTaken 
        : 1'b0;

    assign PCTarget
        = CTL_D[`CTL_PCSRC] == `PCSRC_JUMPIMM ? {PCPlus4D[31:28], InstrD[25: 0], 2'b0} 
        : CTL_D[`CTL_PCSRC] == `PCSRC_JUMPREG ? ReadData1D
        : CTL_D[`CTL_PCSRC] == `PCSRC_BRANCH ? PCPlus4D + {signImmD[29:0], 2'b0} 
        : PCPlus4D;

    assign PCNextF = TakenD ? PCTarget : PCPlus4F;
    
    assign RsD = InstrD[25: 21];
    assign RtD = InstrD[20: 16];
    assign RdD = InstrD[15: 11];
    assign ShamtD = InstrD[10: 6];
    assign ImmD 
        = CTL_D[`CTL_ALUSRCB] == `ALUSRCB_IMMU ? unsignImmD
        : signImmD;

    /* Execute Stage */
    wire [31: 0] ReadData1E, ReadData2E, ImmE;
    wire [4: 0] RsE, RtE, RdE, ShamtE;
    wire [31: 0] ALUOutE, MemWriteDataE;
    wire [4: 0] WriteRegE;

    
    flopenrc #(32) r1E(CLK, RST, ~StallE, FlushE, ReadData1, ReadData1E);
    flopenrc #(32) r2E(CLK, RST, ~StallE, FlushE, ReadData2, ReadData2E);
    flopenrc #(32) r3E(CLK, RST, ~StallE, FlushE, ImmD, ImmE);
    flopenrc #(5)  r4E(CLK, RST, ~StallE, FlushE, RsD, RsE);
    flopenrc #(5)  r5E(CLK, RST, ~StallE, FlushE, RtD, RtE);
    flopenrc #(5)  r6E(CLK, RST, ~StallE, FlushE, RdD, RdE);
    flopenrc #(5)  r7E(CLK, RST, ~StallE, FlushE, ShamtD, ShamtE);
    flopenrc #(32) r8E(CLK, RST, ~StallE, FlushE, PCPlus4D, PCPlus4E);

    wire [31: 0] ReadData1FWE 
        = ForwardAE == `FORWARDE_FROM_MEMORY ? ALUOutM 
        : ForwardAE == `FORWARDE_FROM_WRITEBACK ? ResultW
        : ReadData1E;

    wire [31: 0] ReadData2FWE 
        = ForwardBE == `FORWARDE_FROM_MEMORY ? ALUOutM 
        : ForwardBE == `FORWARDE_FROM_WRITEBACK ? ResultW
        : ReadData2E;

    wire [31: 0] ALUSrcAE = CTL_E[`CTL_ALUSRCA] == `ALUSRCA_RS ? ReadData1FWE : ShamtE;
    wire [31: 0] ALUSrcBE = CTL_E[`CTL_ALUSRCB] == `ALUSRCB_RT ? ReadData2FWE : ImmE;

    ALU alu(ALUSrcAE, ALUSrcBE, CTL_E[`CTL_ALUCONTROL], ALUOutE);
    
    assign MemWriteDataE = ReadData2FWE;//Since only Store can write memory
    assign WriteRegE
        = CTL_E[`CTL_REGDST] == `REGDST_RTYPE ? RdE
        : CTL_E[`CTL_REGDST] == `REGDST_ITYPE ? RtE
        : CTL_E[`CTL_REGDST] == `REGDST_RA ? 5'b11111
        : 5'b0;

    /* Memory Stage */
    wire [31: 0] PCPlus4M;
    wire [4: 0] WriteRegM;
    
    flopenr #(32) r1M(CLK, RST, ~StallM, MemWriteDataE, MemWriteDataM);
    flopenr #(32) r2M(CLK, RST, ~StallM, ALUOutE, ALUOutM);
    flopenr #(32) r3M(CLK, RST, ~StallM, WriteRegE, WriteRegM);
    flopenr #(32) r4M(CLK, RST, ~StallM, PCPlus4E, PCPlus4M);
    assign MemAddrM = ALUOutM;
    
    /* Writeback Stage */
    wire [31: 0] ALUOutW, MemReadDataW;
    floprc #(32) r1W(CLK, RST, FlushW, ALUOutM, ALUOutW);
    floprc #(32) r2W(CLK, RST, FlushW, MemReadDataM, MemReadDataW);
    floprc #(5)  r3W(CLK, RST, FlushW, WriteRegM, WriteRegW);
    floprc #(32) r4W(CLK, RST, FlushW, PCPlus4M, PCPlus4W);

    assign ResultW
        = CTL_W[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_ALU ? ALUOutW
        : CTL_W[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_MEMORY ? MemReadDataW
        : PCPlus4W;

    
    /* Hazard Control */
    wire MemToRegE = CTL_E[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_MEMORY;
    wire MemToRegM = CTL_M[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_MEMORY;
    wire BranchOrJumpRegD = CTL_D[`CTL_PCSRC] == `PCSRC_BRANCH | CTL_D[`CTL_PCSRC] == `PCSRC_JUMPREG;

    /* Cache control */
    wire MemValid = CTL_M[`CTL_MEMREAD] | CTL_M[`CTL_MEMWRITE];
    wire dmem_ready;
    wire MemStall = MemValid & ~dmem_ready;
    Hazard hz(
        RsD, RtD, RsE, RtE, 
        WriteRegE, WriteRegM, WriteRegW,
        CTL_E[`CTL_REGWRITE], CTL_M[`CTL_REGWRITE], CTL_W[`CTL_REGWRITE],
        MemToRegE, MemToRegM, BranchOrJumpRegD, TakenD, MemStall, 
        HCTL);

endmodule 


module Hazard(
    input [4:0] RsD, RtD, RsE, RtE, 
    input [4:0] WriteRegE, WriteRegM, WriteRegW,
    input RegWriteE, RegWriteM, RegWriteW,
    input MemToRegE, MemToRegM, BranchOrJumpRegD,//If Decode is executing Branch or JR or JALR
    input TakenD, MemStall,
    output [`HCTL_WIDTH] HCTL);

    wire RsD_ne0 = RsD != 0;
    wire RtD_ne0 = RtD != 0;
    wire RsE_ne0 = RsE != 0;
    wire RtE_ne0 = RtE != 0;
    

    //Forwarding sources to D stage (For Branch)
    assign HCTL[`HCTL_FORWARDAD] = (RsD_ne0 & RsD == WriteRegM & RegWriteM);
    assign HCTL[`HCTL_FORWARDBD] = (RtD_ne0 & RtD == WriteRegM & RegWriteM);


    //Forwarding sources to E stage (ALU)
    assign HCTL[`HCTL_FORWARDAE]
        = RsE_ne0 & RsE == WriteRegM & RegWriteM ? `FORWARDE_FROM_MEMORY
        : RsE_ne0 & RsE == WriteRegW & RegWriteW ? `FORWARDE_FROM_WRITEBACK
        : `FORWARDE_NONE;
    assign HCTL[`HCTL_FORWARDBE]
        = RtE_ne0 & RtE == WriteRegM & RegWriteM ? `FORWARDE_FROM_MEMORY
        : RtE_ne0 & RtE == WriteRegW & RegWriteW ? `FORWARDE_FROM_WRITEBACK
        : `FORWARDE_NONE;

    wire lwStallD = MemToRegE & (RtE == RsD | RtE == RtD);
    //TODO WriteReg is R0?
    wire branchStallD = BranchOrJumpRegD &
        (RegWriteE & ((RsD_ne0 & WriteRegE == RsD) | (RtD_ne0 & WriteRegE == RtD))
        |MemToRegM & ((RsD_ne0 & WriteRegM == RsD) | (RtD_ne0 & WriteRegM == RtD)));
    
    wire stallD = lwStallD | branchStallD;
    assign HCTL[`HCTL_STALLF] = stallD | MemStall;
    assign HCTL[`HCTL_STALLD] = stallD | MemStall;
    assign HCTL[`HCTL_STALLE] = MemStall;
    assign HCTL[`HCTL_STALLM] = MemStall;
    
    assign HCTL[`HCTL_FLUSHE] = ~MemStall & stallD;
    assign HCTL[`HCTL_FLUSHW] = MemStall;
    assign HCTL[`HCTL_FLUSHD] = ~MemStall & ~stallD & TakenD;

endmodule



module flopr #(parameter WIDTH = 8) (
    input CLK, RST, 
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q);

    always @ (posedge CLK, posedge RST)
        if (RST) q <= 0;
        else q <= d;
endmodule

module floprc #(parameter WIDTH = 8) (
    input CLK, RST, CLR, 
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q);

    always @ (posedge CLK, posedge RST)
        if (RST) q <= 0;
        else if (CLR) q <= 0;
        else q <= d;
endmodule


module flopenr #(parameter WIDTH = 8) (
    input CLK, RST, 
    input EN,
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q);

    always @ (posedge CLK, posedge RST)
        if (RST) q <= 0;
        else if(EN) q <= d;
endmodule


module flopenrc #(parameter WIDTH = 8) (
    input CLK, RST, 
    input EN, CLR,
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q);

    always @ (posedge CLK, posedge RST)
        if (RST) q <= 0;
        else if (CLR) q <= 0;
        else if (EN) q <= d;
endmodule

/*
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
//module ALU (SrcA, SrcB, ALUControl, ALUResult, ZF, OF, SF);
module ALU (SrcA, SrcB, ALUControl, ALUResult);
    input [31: 0] SrcA, SrcB;
    input [3: 0] ALUControl;
    output reg [31: 0] ALUResult;
    
    wire [4:0] shamt;
    wire ZF, OF, SF;
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
