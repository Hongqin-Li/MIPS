`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/07/2019 11:54:29 AM
// Design Name: 
// Module Name: test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module test(SW, LED);
    input [3: 0] SW;
    output [3: 0] LED;
    
    reg [2:0] ctl;
    assign LED = ctl;
    always @ (*)
    begin
        ctl = 0;
        case(SW[1:0])
            2'd0: begin
            case(SW[3:2])
                0:ctl[2] = 0;
                1:ctl[2] = 1;
                2:ctl[1] = 0;
                3:ctl[1] = 1;
            endcase
            end
            2'd1: ctl[0] = 1;
            2'd2: ctl[2] = 1;

        endcase
    end
endmodule
