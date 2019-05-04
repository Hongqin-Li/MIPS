`timescale 1ns / 1ps

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

`define OP_IS_ITYPE(op) (op == `OP_BEQ || op == `OP_BNE || op == `OP_BLEZ || op == `OP_BGTZ || op == `OP_ADDI || op == `OP_ADDIU || op == `OP_SLTI || op == `OP_SLTIU || op == `OP_ANDI || op == `OP_ORI || op == `OP_XORI || op == `OP_LUI || op == `OP_LB || op == `OP_LH || op == `OP_LW || op == `OP_LBU || op == `OP_LHU || op == `OP_SB || op == `OP_SH || op == `OP_SW) 


`define OP_IS_LOAD(Opcode) (Opcode == `OP_LB || Opcode == `OP_LH || Opcode == `OP_LW || Opcode == `OP_LBU || Opcode == `OP_LHU )
`define OP_IS_STORE(Opcode) (Opcode == `OP_SB || Opcode == `OP_SH || Opcode == `OP_SW)

`define OP_IS_JUMP(Opcode, Func) (Opcode == `OP_J || Opcode == `OP_JAL || (Opcode == `OP_SPECIAL && (Func == `FUNC_JR|| Func == `FUNC_JALR)) )

`define OP_IS_BRANCH(Opcode) (Opcode == `OP_BEQ || Opcode == `OP_BNE || Opcode == `OP_BLEZ || Opcode == `OP_BGTZ)

`define OP_NEED_MEM(Opcode) (Opcode == `OP_LB || Opcode == `OP_LH || Opcode == `OP_LW || Opcode == `OP_LBU || Opcode == `OP_LHU || Opcode == `OP_SW || Opcode == `OP_SH || Opcode == `OP_SB)


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
`define ALUSRCB_IMM8 3'd4//multi-cycle pc increment
`define ALUSRCB_IMMSH2 3'd5//multi-cycle beq


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

`define CTL_BRANCH_TAKEN 12
`define BRANCH_TAKEN 1'b1
`define BRANCH_NOT_TAKEN 1'b0

//For multicycle
`define CTL_PCWRITE 20
`define CTL_IRWRITE 22
`define CTL_IORD 21
`define IORD_IMEM 1'd0
`define IORD_DMEM 1'd1

// Use MEMIR cause IR will be the next instr in MEMORY stage
`define CTL_IRSRC 13
`define IRSRC_IR 1'b0
`define IRSRC_MEM 1'b1

`define STATE_WIDTH 4:0

`define STATE_FETCH 5'd0
`define STATE_DECODE 5'd1
`define STATE_EXECUTE 5'd2
`define STATE_EXECUTE_BRANCH 5'd5// Compute next branch pc when taken
`define STATE_MEMORY 5'd3
`define STATE_WRITEBACK 5'd4

`define STATE_UNDEFINED 5'dxxxxx



`define DMEM_MAX_ADDR 16'd600

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





module top (CLK100MHZ, SW, AN, CA, CB, CC, CD, CE, CF, CG);
    
    input CLK100MHZ;
    input [15: 0] SW;
    
    output [7: 0] AN;
    output CA, CB, CC, CD, CE, CF, CG;
    
    reg [31: 0] data;
    wire [6: 0] C;
    wire clk, clk1_4hz, rst;
    
    wire [7: 0] out = mt.mem.RAM[`OUT0_ADDR/4];
    wire [31: 0] pc = mt.dp.PC;
    
    wire [3: 0] d0 = out % 10;
    wire [3: 0] d1 = (out / 10) % 10;
    wire [3: 0] d2 = (out / 100) % 10;
    wire [3: 0] d3 = 4'b0;
    wire [3: 0] d4 = pc % 10;
    wire [3: 0] d5 = (pc / 10) % 10;
    wire [3: 0] d6 = (pc / 100) % 10;
    wire [3: 0] d7 = (pc / 1000) % 10;
    
    clkdiv cd(.mclk(CLK100MHZ), .clk(clk),.clk1_4hz(clk1_4hz));
    
    MIPSTop mt(SW[3] ? clk1_4hz|SW[2] : clk|SW[2], rst);//Stopping, running or running in low speed
    
    Display dis(CLK100MHZ, AN, C, 
        {d7, d6, d5, d4, d3, d2, d1, d0});
        
    assign {CA, CB, CC, CD, CE, CF, CG} = C;
    assign rst = SW[0];
    
    // Input Mapping 
    assign mt.mem.iwrite = SW[0] & SW[1];
    assign mt.mem.iaddr = `IN0_ADDR;
    assign mt.mem.idata = SW[15:4];
    
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

    wire [31: 0] Instr;

    wire [31: 0] ALUResult;
    wire [31: 0] MemReadData, MemWriteData, MemAddr;
    wire BranchSrc;

    wire ZF, SF, OF;
    wire [`CTL_WIDTH] ctl;
    wire branchSrc;
    
    Datapath dp(CLK, RST, ctl, BranchSrc, MemReadData, MemWriteData, MemAddr, ZF, SF, OF, Instr);

    ControlUnit cu(CLK, RST, Instr[31: 26], Instr[5: 0], ZF, SF, OF, ctl, BranchSrc);

    Memory mem(CLK, MemAddr, ctl[`CTL_MEMWRITE], MemWriteData, MemReadData, ctl[`CTL_MEMSIGNED], ctl[`CTL_MEMWIDTH]);
    
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

module Memory (CLK, Addr, WriteEnable, WriteData, ReadData, Sign, Width);

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
    
    
    
    initial
    begin
    RAM[0] = {ADDI, r0, sp, `STACK_BEGIN_ADDR};//Stack pointer 256
            
    //RAM[1] = {ADDI, r0, t0, 16'd0};
    //RAM[2] = {SB, r0, t0, `IN0_ADDR};
            
    //Read seed from IO device
    RAM[3] = {LBU, r0, s0, `IN0_ADDR};
            
    //loop:
    RAM[10] = {ORI, s0, a1, 16'd0};// retrieve seed
    RAM[11] = {ADDI, r0, a0, 16'd17};// a = 17
    RAM[12] = {ADDI, r0, a2, 16'd3};// c = 3
    RAM[13] = {ADDI, r0, a3, 16'd256};// m = 256
            
    RAM[14] = {JAL, 26'd50};// call lcg()
            
    RAM[15] = {ORI, v0, s0, 16'b0};
    RAM[16] = {SB, r0, v0, `OUT0_ADDR};
    RAM[17] = {BEQ, r0, r0, 16'hfff8};//loop
            
            
    //linear congruential generator  
    //int lcg(a, seed, c, m) {return seed = (a * seed + c) % m;}
    RAM[50] = {6'b011100, a1, a0, t0, 5'b00000, 6'b000010};//MUL
    RAM[51] = {6'b0, a2, t0, t0, 5'b00000, ADDU};
    RAM[52] = {ORI, t0, a0, 16'b0};
    RAM[53] = {6'b000000, r0, a3, a1, 5'b0, OR};
    //Push
    RAM[54] = {ADDI, sp, sp, 16'hfffc};// sp -= 4
    RAM[55] = {SW, sp, ra, 16'b0};
    //Push end
            
    //Testing JARL
    RAM[56] = {ORI, r0, t2, 14'd80, 2'd0};
    RAM[57] = {6'b0, t2, r0, ra, 5'b0, JALR};
    //`ROM[56] = {JAL, 26'd80};//call mod()
            
    //Pop
    RAM[58] = {LW, sp, ra, 16'b0};
    RAM[59] = {ADDI, sp, sp, 16'h4};// sp -= 4
    //Pop end
    RAM[60] = RET;
            
    // int mod(a0, a1) {return v0 = a0 % a1;} 
    RAM[80] = {ADDI, a0, v0, 16'b0};//ADDI $2, $4, 0 # $1 <= $2
    RAM[81] = {6'b0, a1, v0, t0, 5'b0, SUB};//SUB $8, $3, $2
    RAM[82] = {6'b0, r0, v0, t1, 5'b0, SUB};
    RAM[83] = {BLEZ, t0, 5'b0, 16'b10};// # if c - b <= 0, then branch 1 (b -= c)
    RAM[84] = {BGTZ, t1, 5'b0, 16'b11};// # else if b < 0, then branch 2 (b += c)
    RAM[85] = RET;// # else return 
    RAM[86] = {6'b0, v0, a1, v0, 5'b0, SUB};//SUB $1, $1, $3 # branch 1
    RAM[87] = {BEQ, r0, r0, 16'hfff9};// B -0x7 # loop
    RAM[88] = {6'b0, v0, a1, v0, 5'b0, ADD};//ADD $1, $1, $3 # branch 2
    RAM[89] = {BEQ, r0, r0, 16'hfff7};// B -0x9 # loop           

    
    
    end        
endmodule



//module Datapath (CLK, RST, Instr, CTL, BranchSrc, MemReadData, MemWriteData, ALUResult, ZF, SF, OF, PC);
module Datapath (CLK, RST, CTL, BranchSrc, MemReadData, MemWriteData, MemAddr, ZF, SF, OF, Instr);
    input CLK, RST;
    input [`CTL_WIDTH] CTL;
    input BranchSrc;//taken or not taken
    
    input [31: 0] MemReadData;//the output of DMem

    output [31: 0] MemAddr;
    output [31: 0] MemWriteData;//data to wirte DMem
    output ZF, OF, SF;
    output [31: 0] Instr;//The current instr
   
    wire [31: 0] PC, pcNext, pcBranch, pcNextBranch; 
    
    wire [31: 0] imm, signImm, signImmSh2;//shift 2

    wire [4: 0] writeReg;// regdst
    wire [31: 0] regWriteData;//for regfile
    wire [31: 0] readData1, readData2, A, B, ALUResult;
    
    wire [31: 0] srcA, srcB;//ALU
    wire [31: 0] aluout;//aluout register

    wire [31: 0] instrMem, instr;//In Memory stage, we use instr since instr has been overidden by next instr
    
    assign MemAddr = CTL[`CTL_IORD] == `IORD_IMEM ? PC : aluout;
    
    Register #(32) PCReg (CLK, RST, CTL[`CTL_PCWRITE], pcNext, PC);
    Register #(32) IR (CLK, RST, CTL[`CTL_IRWRITE], MemReadData, instr);
    
    //To be modify
    Register #(32) MemIR (CLK, RST, 1, instr, instrMem);//Add 
    assign Instr = CTL[`CTL_IRSRC] == `IRSRC_MEM ? instrMem : instr;
    
    Register #(32) RA (CLK, RST, 1, readData1, A);
    Register #(32) RB (CLK, RST, 1, readData2, B);
    Register #(32) ALUOut (CLK, RST, 1, ALUResult, aluout);
    

    ShiftLeft2 immsh (signImm, signImmSh2);//imm shift
    SignExtend signext (Instr[15: 0], signImm);

    //assign signImm = {{16{Instr[15]}}, Instr[15: 0]};
    //assign signImmSh2 = {signImm[29:0], 2'b00};

    assign pcNextBranch = BranchSrc ? ALUResult: PC;//PC is now PC + 4 
    assign pcNext
        = CTL[`CTL_PCSRC] == `PCSRC_BRANCH ? pcNextBranch
        : CTL[`CTL_PCSRC] == `PCSRC_JUMPIMM ? {PC[31:28], Instr[25:0], 2'b00}
        : CTL[`CTL_PCSRC] == `PCSRC_JUMPREG ? readData1
        : CTL[`CTL_PCSRC] == `PCSRC_ALURESULT ? ALUResult
        : PC;

    //Register File
    assign writeReg
        = CTL[`CTL_REGDST] == `REGDST_RTYPE ? Instr[15: 11]
        : CTL[`CTL_REGDST] == `REGDST_ITYPE ? Instr[20: 16]
        : 5'b11111;
    
    assign regWriteData
        = CTL[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_ALU ? ALUResult
        : CTL[`CTL_REGWRITE_DATA] == `REGWRITE_DATA_FROM_MEMORY ? MemReadData
        : ALUResult;//Used in jal and jalr
    
    RegisterFile rf (CLK, CTL[`CTL_REGWRITE], regWriteData, writeReg, Instr[25:21], Instr[20:16], readData1, readData2);



    //ALU
    assign srcA
        = CTL[`CTL_ALUSRCA] == `ALUSRCA_SHAMT ? {27'b0, Instr[10: 6]}
        : CTL[`CTL_ALUSRCA] == `ALUSRCA_PC ? PC
        : A;

    assign srcB
        = CTL[`CTL_ALUSRCB] == `ALUSRCB_IMM ? signImm
        : CTL[`CTL_ALUSRCB] == `ALUSRCB_IMMU ? {16'b0, Instr[15: 0]}
        : CTL[`CTL_ALUSRCB] == `ALUSRCB_IMM4 ? 32'd4
        : CTL[`CTL_ALUSRCB] == `ALUSRCB_IMM8 ? 32'd8
        : CTL[`CTL_ALUSRCB] == `ALUSRCB_IMMSH2 ? signImmSh2
        : B;
    
    ALU alu (srcA, srcB, CTL[`CTL_ALUCONTROL], ALUResult, ZF, OF, SF);//add shamt
    
    assign MemWriteData = B;
        
endmodule



module ALUDecoder(Opcode, Func, ALUOp, ALUControl);

    input [5: 0] Opcode, Func;
    input [`ALUOP_WIDTH] ALUOp;
    output reg [`ALUCONTROL_WIDTH] ALUControl;

    reg [`ALUCONTROL_WIDTH] ctl;


    always @ (*)
        case (ALUOp)
        `ALUOP_ADDU: ALUControl = `ALU_ADDU;
        `ALUOP_SUBU: ALUControl = `ALU_SUBU;
        default: ALUControl = ctl;
        endcase

    always @ (*)
        case (Opcode)
        `OP_SPECIAL:
            case (Func)
			`FUNC_SLL: ctl = `ALU_SLL;
			`FUNC_SRL: ctl = `ALU_SRL;
			`FUNC_SRA: ctl = `ALU_SRA;
			`FUNC_SLLV: ctl = `ALU_SLL;
			`FUNC_SRLV: ctl = `ALU_SRL;
			`FUNC_SRAV: ctl = `ALU_SRA;
			//`FUNC_JR: ctl = `ALU_JR;
			//`FUNC_JALR: ctl = `ALU_JALR;
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
endmodule


// Multi-cycle Control Unit: Mealy FSM
module ControlUnit(CLK, RST, Opcode, Func, ZF, SF, OF, CTL, BranchSrc);
    input CLK, RST;
    input [5: 0] Func, Opcode;
    input ZF, SF, OF;
    output reg [`CTL_WIDTH] CTL;
    output BranchSrc;
    
    reg [`STATE_WIDTH] state, nextState;

    reg [`ALUOP_WIDTH] ALUOp;
    wire [`ALUCONTROL_WIDTH ] aluctl;
    wire branchTaken;
    ALUDecoder aludecoder(Opcode, Func, ALUOp, aluctl);

    assign BranchSrc
        = (Opcode == `OP_BEQ) ? ZF 
        : (Opcode == `OP_BNE) ? ~ ZF
        : (Opcode == `OP_BLEZ) ? ZF | SF
        : (Opcode == `OP_BGTZ) ? ~ (ZF | SF)
        : 0;
    assign branchTaken = BranchSrc;
    
    always @ (posedge CLK) 
        if (RST) state <= `STATE_FETCH;
        else state <= nextState;

    //Next State
    always @ (*)
    begin
        nextState = `STATE_FETCH;
        case(state)
            `STATE_FETCH:
                nextState = `STATE_DECODE;

            `STATE_DECODE:
                if (Opcode == `OP_SPECIAL && (Func == `FUNC_JR || Func == `FUNC_JALR))
                    nextState = `STATE_FETCH;
                else if (Opcode == `OP_J || Opcode == `OP_JAL)
                    nextState = `STATE_FETCH;
                    //nextState = `STATE_WRITEBACK;
                else nextState = `STATE_EXECUTE;

            `STATE_EXECUTE:
                if (`OP_IS_BRANCH(Opcode))
                begin
                    if (BranchSrc) nextState = `STATE_EXECUTE_BRANCH;
                    else nextState = `STATE_DECODE;
                end
                else if (`OP_NEED_MEM(Opcode))
                    nextState = `STATE_MEMORY;
                else
                    nextState = `STATE_DECODE;
                    //nextState = `STATE_WRITEBACK;
            `STATE_EXECUTE_BRANCH:
                nextState = `STATE_FETCH;
            `STATE_MEMORY:
                nextState = `STATE_DECODE;
            default: nextState = `STATE_FETCH;
        endcase
    end

    //output 
    always @ (*)
    begin
        //default
        CTL = 0;
        ALUOp = `ALUOP_ALU;
        CTL[`CTL_ALUCONTROL] = aluctl;
        case (state)
            /*
            * 1. Fetch instruction from Memory (MEM)
            * Next CLK: update IR
            */
            `STATE_FETCH: 
            begin
                CTL[`CTL_IORD] = `IORD_IMEM;
                CTL[`CTL_PCSRC] = `PCSRC_ALURESULT;
                CTL[`CTL_IRWRITE] = 1;
            end

        /*
        * 1. Decode and Read Register File (RF)
        * 2. Compute next PC (ALU)
        * Next CLK: update A, B, PC
        * Jump-type slightly different
        * Branch is different, also
        */
        `STATE_DECODE:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_PC;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM4;
            ALUOp = `ALUOP_ADDU;
            CTL[`CTL_PCWRITE] = 1;
            CTL[`CTL_PCSRC] = `PCSRC_ALURESULT;
            
            
            /*if (`OP_IS_BRANCH(Opcode))
            begin
                CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
            end*/
            
            if (Opcode == `OP_J)
                CTL[`CTL_PCSRC] = `PCSRC_JUMPIMM;

            else if (Opcode == `OP_JAL)
            begin 
                CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM4;
                CTL[`CTL_PCSRC] = `PCSRC_JUMPIMM;
                CTL[`CTL_REGWRITE] = 1;
                CTL[`CTL_REGDST] = `REGDST_RA;
                CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_PCPLUS4;
            end

            else if (Opcode == `OP_SPECIAL && Func == `FUNC_JR)
                CTL[`CTL_PCSRC] = `PCSRC_JUMPREG;

            else if (Opcode == `OP_SPECIAL && Func == `FUNC_JALR)
            begin
                CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM4;
                CTL[`CTL_PCSRC] = `PCSRC_JUMPREG;
                CTL[`CTL_REGWRITE] = 1;
                CTL[`CTL_REGDST] = `REGDST_RTYPE;
                CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_PCPLUS4;
            end
            else ;
        end



        /*
        * 1. Perform computation (ALU)
        * 2. Fetch next instruction (MEM)
        * Next CLK: update RF, ALU, IR 
        */
        `STATE_EXECUTE:
        begin
            CTL[`CTL_ALUSRCA] = `ALUSRCA_RS;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_RT;
            ALUOp = `ALUOP_ALU;

            CTL[`CTL_IORD] = `IORD_IMEM;
            CTL[`CTL_IRWRITE] = 1;

            case(Opcode)
                `OP_SPECIAL:
                begin
                    CTL[`CTL_REGWRITE] = 1;
                    CTL[`CTL_REGDST] = `REGDST_RTYPE;
                    if (Func == `FUNC_SLL || Func == `FUNC_SRL || Func == `FUNC_SRA)
                        CTL[`CTL_ALUSRCA] = `ALUSRCA_SHAMT;
                end
                `OP_MUL:
                    if (Func == `FUNC_MUL) 
                    begin
                        CTL[`CTL_REGWRITE] = 1;
                        CTL[`CTL_REGDST] = `REGDST_RTYPE;
                    end
                    else
                        CTL = `CTL_UNDEFINED;

                `OP_BEQ, `OP_BNE, `OP_BLEZ, `OP_BGTZ:
                begin
                    //When branch not taken, it also need to update IR by next instruction 
                    ALUOp = `ALUOP_SUBU;
                    CTL[`CTL_IRWRITE] = !BranchSrc;
                end

                //Unsigned IMM, zero-extended
                `OP_SLTIU, `OP_ANDI, `OP_ORI, `OP_XORI, `OP_LUI:
                begin
                    CTL[`CTL_REGWRITE] = 1;
                    CTL[`CTL_REGDST] = `REGDST_ITYPE;
                    CTL[`CTL_ALUSRCB] = `ALUSRCB_IMMU;
                end
                `OP_ADDI, `OP_ADDIU, `OP_SLTI:
                begin
                    CTL[`CTL_REGWRITE] = 1;
                    CTL[`CTL_REGDST] = `REGDST_ITYPE;
                    CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                end
                `OP_LB, `OP_LH, `OP_LW, `OP_LBU, `OP_LHU, `OP_SB, `OP_SH, `OP_SW:
                    CTL[`CTL_ALUSRCB] = `ALUSRCB_IMM;
                
                default: CTL = `CTL_UNDEFINED;
            endcase
        end
        /*
        * Only when branch taken
        * 1. Compute the target address
        */
        `STATE_EXECUTE_BRANCH:
        begin
            ALUOp = `ALUOP_ADDU;
            CTL[`CTL_ALUSRCA] = `ALUSRCA_PC;
            CTL[`CTL_ALUSRCB] = `ALUSRCB_IMMSH2;
            
            CTL[`CTL_PCWRITE] = 1;//Branch Not taken
            CTL[`CTL_PCSRC] = `PCSRC_ALURESULT;
            
        end
        //Only LOAD and STORE get to this
        /*
        * 1. Read or Write(MEM)
        *
        */
        `STATE_MEMORY:
        begin
            CTL[`CTL_IORD] = `IORD_DMEM;
            CTL[`CTL_IRSRC] = `IRSRC_MEM;
            CTL[`CTL_MEMWRITE] = 0;
            CTL[`CTL_MEMSIGNED] = 1;

            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
            CTL[`CTL_REGDST] = `REGDST_ITYPE;

            case (Opcode)
                `OP_LB: CTL[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                `OP_LH: CTL[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                `OP_LW: CTL[`CTL_MEMWIDTH] = `MEMWIDTH_32;
                
                `OP_LBU:
                begin
                    CTL[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                    CTL[`CTL_MEMSIGNED] = 0;
                end
                `OP_LHU:
                begin
                    CTL[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                    CTL[`CTL_MEMSIGNED] = 0;
                end

                `OP_SB:
                begin
                    CTL[`CTL_REGWRITE] = 0;
                    CTL[`CTL_MEMWRITE] = 1;
                    CTL[`CTL_MEMWIDTH] = `MEMWIDTH_8;
                end
                `OP_SH:
                begin
                    CTL[`CTL_REGWRITE] = 0;
                    CTL[`CTL_MEMWRITE] = 1;
                    CTL[`CTL_MEMWIDTH] = `MEMWIDTH_16;
                end
                `OP_SW:
                begin
                    CTL[`CTL_REGWRITE] = 0;
                    CTL[`CTL_MEMWRITE] = 1;
                    CTL[`CTL_MEMWIDTH] = `MEMWIDTH_32;
                end
                default: CTL[`CTL_WIDTH] = `CTL_UNDEFINED;
            endcase
        end
        
        default:;
        /*
        //Only R/I-Type ALU and LOAD can get to this 
        `STATE_WRITEBACK:
        begin
            CTL[`CTL_REGWRITE] = 1;
            CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_ALU;
            CTL[`CTL_PCWRITE] = 0;//To Decode cause no branch or jump
            case (Opcode)
                //R-Type ALU
                `OP_SPECIAL:
                    CTL[`CTL_REGDST] = `REGDST_RTYPE;
                `OP_MUL:
                    if (Func == `FUNC_MUL)
                        CTL[`CTL_REGDST] = `REGDST_RTYPE;
                    else CTL = `CTL_UNDEFINED;

                //I-Type ALU
                `OP_SLTIU, `OP_ANDI, `OP_ORI, `OP_XORI, `OP_LUI, `OP_ADDI, `OP_ADDIU, `OP_SLTI:
                    CTL[`CTL_REGDST] = `REGDST_ITYPE;
                //LOAD
                `OP_LB, `OP_LH, `OP_LW, `OP_LBU, `OP_LHU:
                begin
                    CTL[`CTL_REGDST] = `REGDST_ITYPE;
                    CTL[`CTL_REGWRITE_DATA] = `REGWRITE_DATA_FROM_MEMORY;
                end
                default: CTL = `CTL_UNDEFINED;
            endcase
        end
        */
        endcase
    end
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

module Register #(parameter WIDTH = 8) (clk, rst, en, d, q);

    input clk, rst, en;
    input [WIDTH - 1: 0] d;
    output reg [WIDTH - 1: 0] q = 0;

    always @ (posedge clk or posedge rst)
        if (rst) q <= 0;
        else if (en) q <= d;
        else q <= q;
    
    initial
    begin
    q = 0;
    end
    
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
