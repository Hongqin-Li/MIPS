`timescale 1ns / 1ps

module MIPS_tb;
    reg clk = 0;
    reg rst = 0;
    
    wire [31: 0] ALUResult, Instr;
    wire [31: 0] pc;
    
    wire [31: 0] rf [0: 31];
    wire [31: 0] dm [0: 31];
    wire [31: 0] srca;
    wire [1: 0] wid;
    wire sig;
    wire [31: 0]test;
    wire zf, sf, of;
    wire pcsrc;
    wire [31: 0] pcbranch;
    Top t(clk, rst, ALUResult, Instr);
    
    assign srca = t.mips.dp.srcA;
    
    assign zf = t.mips.dp.ZF;
    assign sf = t.mips.dp.SF;
    assign of = t.mips.dp.OF;
    assign pcsrc = t.mips.dp.PCSrc;
    
    assign pcbranch = t.mips.dp.pcBranch;
    
    assign wid = t.MemWidth;
    assign sig = t.MemSign;
    assign pc = t.PC;
    assign test = t.mips.dp.pcJump;
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.mips.dp.rf.RegCell[i];end
    
    for (i = 0; i < 32; i = i + 1) begin assign dm[i] = t.dmem.RAM[i];end
    
    endgenerate
    
    initial
    begin
        rst = ~rst;
        #20 ;
        rst = ~rst;
        repeat (100) #20 clk = ~ clk;
    end
    
endmodule


/*
module RegisterFile_tb;
    reg clk = 0;
    reg RegWrite;
    reg [31: 0] writeData;
    reg [4: 0] writeReg;
    
    parameter [4:0] readReg1 = 1, readReg2 = 2;
    reg [31: 0] Instr = {32'b001000, readReg1, readReg2, 16'b1010101010101010 };
    
    wire [31: 0] readData1, readData2;
    
    RegisterFile rf (clk, RegWrite, writeData, writeReg, Instr[25:21], Instr[20:16], readData1, readData2);
    initial
    begin
        rf.RegCell[1] = 1;
        rf.RegCell[2] = 2;
    end
endmodule

*/
/*
module Datapath_tb;

    reg clk, rst, MemToReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump, MemWrite;
    reg [3: 0] ALUControl;
    reg [31: 0] Instr;
    reg [31: 0] ReadData;//the output of DMem

    wire [31: 0] WriteData;//for DMem
    wire [31: 0] ALUResult;
    wire ZF, OF, SF;
    wire [31: 0] PC;
    
    wire [31: 0] addr1, data1, data2, srcb, writeReg, writeData;
    wire [31: 0] signi, signish;
    wire [4: 0] readReg1, readReg2;
    
    Datapath dp(clk, rst, Instr, MemToReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump, MemWrite, ALUControl, ALUResult, WriteData, ReadData, ZF, SF, OF, PC);

    assign {readReg1, readReg2} = {dp.rf.readAddr1, dp.rf.readAddr2 }; 
    assign data1 = dp.readData1;  
    assign data2 = dp.rf.readData2;
    assign srcb = dp.srcB;
    assign writeReg = dp.writeReg;
    assign writeData = dp.writeData;
    assign signi = dp.signImm;
    assign signish = dp.signImmSh2;
    
    assign addr1 = dp.rf.readAddr1;
    
    initial
    begin    
        clk = 0;
        rst = 0; #20 ;
        rst = 1; #20 ;
        rst = 0; #20 ;
        {MemToReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump, MemWrite} = 6'b0111000;
        ALUControl = 2;
        Instr = 32'b001000_00001_00010_1010101010101010;//ADDI
        ReadData = 8;
        repeat (10) #20 clk = ~ clk;
    end
    
endmodule
*/

/*
module ControlUnit_tb;
    reg [5: 0] Func, Opcode;
    reg ZF, SF, OF;
    wire [3: 0] ALUControl;
    wire RegDst, RegWrite, ALUSrc, MemWrite, MemToReg, PCSrc, Jump;
    
    ControlUnit cu(Func, Opcode, ZF, SF, OF, RegDst, RegWrite, ALUSrc, ALUControl, MemWrite, MemToReg, PCSrc, Jump);
    initial
    begin
        Func = 6'b0;
        Opcode = 6'b100000;
        ZF = 1;    
    end
endmodule
*/
/*
module SignExtend_tb();
    reg [15: 0] in;
    wire [31: 0] out;
    SignExtend signext (in, out);
    initial
    begin
        in = 16'b10000000_00000011;
    end    
endmodule
*/

/*
module IMem_tb;
    
    reg [31: 0] addr = 0;
    wire [31: 0] instr;
    
    IMem imem (addr, instr);
    
    reg [31: 0] i = 0;
    initial
    begin
        for (i = 0; i < 10; i = i + 1)
        begin
            #10 ;
            addr <= addr + 4;
        end
    end
endmodule
*/


/*
module ALU_tb();
    reg [31: 0] a, b;
    reg [3: 0] ctl;
    
    wire zf, of, sf;
    wire [31: 0] out;
    
    ALU aluu(a, b, ctl, out, zf, of, sf);
    
    reg [31: 0] i = 0;
    initial 
    begin
        a = 32'b100;
        b = 32'b110;
        ctl = 0;
        for (i = 0; i < 10; i = i + 1)
        begin
            # 10;
            ctl = ctl + 1;
        end
    end
endmodule
*/


