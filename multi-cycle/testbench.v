`timescale 1ns / 1ps

module MIPS_tb;
    reg clk = 0;
    reg rst = 0;
    MIPSTop t(clk, rst);
    
    wire [31: 0] ALUResult = t.ALUResult, Instr = t.Instr;
    wire [31: 0] pc = t.PC;
    wire [31: 0] writeData = t.dmem.writeData;
    wire [31: 0 ] writeAddr = t.dmem.addr;
    
    wire [31: 0] rf [0: 31];
    
    wire [31: 0]test;

    assign test = t.dmem.RAM[16'h100/4];
    generate 
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin assign rf[i] = t.mips.dp.rf.RegCell[i];end
    endgenerate
    
    initial
    begin
        t.mips.dp.rf.RegCell[16] = 64;
        rst = ~rst;
        #20 ;
        rst = ~rst;
        repeat (300) #2 clk = ~ clk;
    end
    
endmodule

/*
module Multiplier_tb;


    reg s = 0;
    reg signed [31: 0] a = -10000;
    reg signed [31: 0] b = 10000;
    wire [31 : 0] result;
    
    Multiplier mlt(a, b, s, result);
    wire [63: 0] t = mlt.temp;
    
    initial 
    begin
        #20 s = ~s;
    end
    
    
endmodule

*/


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


