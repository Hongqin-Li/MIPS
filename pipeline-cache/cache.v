
`ifndef CACHE
`define CACHE

`define DELAY 6//The delay time, i.e. number of cycle (1<<DELAY) to wait for memory
//Only need to modify this
`define BLOCK_OFFSET_WIDTH_NUM 4 //e, no aligned by word
`define BLOCK_OFFSET_WIDTH `BLOCK_OFFSET_WIDTH_NUM-1: 0

`define BLOCK_DATA_WIDTH_NUM ((1<<`BLOCK_OFFSET_WIDTH_NUM)*8)  //by bit 
`define BLOCK_DATA_WIDTH `BLOCK_DATA_WIDTH_NUM-1: 0

`define BLOCK_WIDTH `BLOCK_DATA_WIDTH_NUM + 1 + t: 0
`define BLOCK_VALID 0
`define BLOCK_DIRTY 1
`define BLOCK_DATA `BLOCK_DATA_WIDTH_NUM + 1: 2
`define BLOCK_TAG `BLOCK_DATA_WIDTH_NUM + 1 + t:`BLOCK_DATA_WIDTH_NUM + 2

`define REQUEST_WIDTH 35 + `BLOCK_DATA_WIDTH_NUM: 0
`define REQUEST_ADDR 31: 0
`define REQUEST_VALID 32

`define REQUEST_TYPE 33
    `define REQUEST_READ 1'b0
    `define REQUEST_WRITE 1'b1
    
//TODO: only used in write, should be identical to CTL_MEMWIDTH define !!
`define REQUEST_WRITE_WIDTH 35:34
`define REQUEST_WRITE_WORD 2'd0
`define REQUEST_WRITE_HALF 2'd1
`define REQUEST_WRITE_BYTE 2'd2

    
`define REQUEST_DATA 35 + `BLOCK_DATA_WIDTH_NUM: 36

`define RESPONSE_WIDTH `BLOCK_DATA_WIDTH_NUM:0
`define RESPONSE_READY 0
`define RESPONSE_DATA `BLOCK_DATA_WIDTH_NUM:1




/*
module ROMTop(
    input CLK, 
    input Valid,
    input [31:0] Addr,
    output[31:0] Data
    );

    wire [`REQUEST_WIDTH] request_from_cpu;
    wire [`RESPONSE_WIDTH] response_to_cpu;
    wire [`REQUEST_WIDTH] request_to_mem;
    wire [`RESPONSE_WIDTH] response_from_mem;
    
    assign request_from_cpu[`REQUEST_VALID] = Valid;
    assign request_from_cpu[`REQUEST_ADDR] = Addr;
    assign request_from_cpu[`REQUEST_DATA] = 128'hfffbaefcdab;//no used
    assign request_from_cpu[`REQUEST_TYPE] = `REQUEST_WRITE;//Read only
    assign request_from_cpu[`REQUEST_WRITE_WIDTH] = `REQUEST_WRITE_BYTE;//no used
    Cache c(CLK, 0, request_from_cpu, request_to_mem, response_from_mem, response_to_cpu);
    ROM mem(CLK, 0, request_to_mem, response_from_mem);

    parameter t = c.t;
    wire [`BLOCK_DATA_WIDTH] response_data = response_to_cpu[`RESPONSE_DATA];
    //wire offset = Addr[`BLOCK_OFFSET_WIDTH]
    wire [`BLOCK_OFFSET_WIDTH_NUM-3:0] word_offset = Addr[`BLOCK_OFFSET_WIDTH_NUM-1:2];
    assign Data = response_data[(word_offset<<3) +: 32];
    
    
endmodule
*/

module Memory #(parameter s=2) (
    input CLK, 
    input Valid,
    input [31:0] Addr,
    //TODO
    input WriteEN,
    input [31:0] WriteData,
    output[31:0] Data,
    input Signed,
    input [1:0] Width,
    output Ready
    );
    

    wire [`REQUEST_WIDTH] request_from_cpu;
    wire [`RESPONSE_WIDTH] response_to_cpu;
    wire [`REQUEST_WIDTH] request_to_mem;
    wire [`RESPONSE_WIDTH] response_from_mem;
   
    /* Create a new Request */
    reg [`BLOCK_DATA_WIDTH] writeData;
    
    assign request_from_cpu[`REQUEST_VALID] = Valid;
    assign request_from_cpu[`REQUEST_ADDR] = Addr;
    assign request_from_cpu[`REQUEST_DATA] = writeData;
    assign request_from_cpu[`REQUEST_TYPE] = WriteEN ? `REQUEST_WRITE :`REQUEST_READ;
    assign request_from_cpu[`REQUEST_WRITE_WIDTH] = Width;//TODO
    
        
    Cache #(s) c(CLK, 0, request_from_cpu, request_to_mem, response_from_mem, response_to_cpu);
    RAM mem(CLK, 0, request_to_mem, response_from_mem);

    //parameter s = 2, e = `BLOCK_OFFSET_WIDTH_NUM;
    //parameter t = 32 - s - e;
    
    wire [`BLOCK_DATA_WIDTH] response_data = response_to_cpu[`RESPONSE_DATA];
    //wire offset = Addr[`BLOCK_OFFSET_WIDTH]
    wire [`BLOCK_OFFSET_WIDTH_NUM+2: 0] offset = {Addr[`BLOCK_OFFSET_WIDTH], 3'b0};
    wire [31:0] rawData = response_data[offset +: 32];
    assign Data = Width == `REQUEST_WRITE_BYTE ? { {24{Signed & rawData[7]}}, rawData[7:0]}
                : Width == `REQUEST_WRITE_HALF ? { {16{Signed & rawData[7]}}, rawData[15:0]}
                : rawData;
    
    
    assign Ready = response_to_cpu[`RESPONSE_READY];
    always @ (*) begin
        writeData = 0;
        writeData[offset +: 32] = WriteData;
    end
    
    
endmodule



module RAM(
    input CLK, RST,
    input [`REQUEST_WIDTH] Request,
    output reg [`RESPONSE_WIDTH] Response);

    reg [`BLOCK_DATA_WIDTH] RAM[256:0];
    reg [4:0] cnt = 0;
    
    wire [31: 0] addr = Request[`REQUEST_ADDR];
    wire [`BLOCK_DATA_WIDTH] requestData = Request[`REQUEST_DATA];
    wire [31-`BLOCK_OFFSET_WIDTH_NUM: 0] a = addr[31:`BLOCK_OFFSET_WIDTH_NUM];
    
    wire [`BLOCK_DATA_WIDTH] readBlock = RAM[a];
   
    /* Write */
    wire [`BLOCK_OFFSET_WIDTH_NUM+2: 0] offset = {addr[`BLOCK_OFFSET_WIDTH], 3'b0};
    reg [`BLOCK_DATA_WIDTH] dataMask, nmask;
    wire [`BLOCK_DATA_WIDTH] mask = ~nmask;
    
    always @(*) begin
    
        dataMask = 0;
        nmask = 0;
        
        if (Request[`REQUEST_WRITE_WIDTH] == `REQUEST_WRITE_BYTE) begin
            dataMask[offset +: 8] = requestData[offset +: 8];
            nmask[offset +:8] = ~0;
        end       
        
        else if (Request[`REQUEST_WRITE_WIDTH] == `REQUEST_WRITE_HALF) begin
            dataMask[offset +:16] = requestData[offset +: 16];
            nmask[offset+:16] = ~0;
        end
        
        else begin
            dataMask[offset+:32] = requestData[offset +: 32];
            nmask[offset+:32] = ~0;
        end
    end
   
   
   
    always @ (posedge CLK)
        if (Request[`REQUEST_VALID]) begin
            if (cnt[`DELAY]) begin
                cnt <= 0;
                Response[`RESPONSE_READY] <= 1;
                if (Request[`REQUEST_TYPE] == `REQUEST_READ)
                    Response[`RESPONSE_DATA] <= readBlock;
                else 
                    RAM[a] <= (RAM[a] & mask) | dataMask;
            end
            else cnt <= cnt + 1;
        end
        else begin
            cnt <= 0;
            Response[`RESPONSE_READY] <= 0;
        end

    integer i;
    initial 
    begin
        for (i = 0; i < 100; i = i + 1) RAM[i] = 0;
        //RAM[0] = 128'hffffffffeeeeeeeefefefefe12345678;
        //RAM[1] = 128'haaaabbbbccccccccddddddddabcdef01;
    end


endmodule

module Cache #(parameter s=2) (
    input CLK, RST,
    /* Request from higher level */
    input [`REQUEST_WIDTH] Request,
    /* Request to lower level */
    output [`REQUEST_WIDTH] RequestToNext,
    /* Response from lower level */
    input [`RESPONSE_WIDTH] ResponseFromNext,
    /* Response to higher level */
    output reg [`RESPONSE_WIDTH] Response
);

    /* 
        t: Tag width
        s: Set width
        e: Block offset width 
    */
    localparam e = `BLOCK_OFFSET_WIDTH_NUM;
    localparam t = 32 - s - e;

    localparam TAG_MSB = 31, TAG_LSB = 32 - t;
    localparam SET_MSB = TAG_LSB - 1, SET_LSB = TAG_LSB - s;
    //parameter OFFSET_MSB = SET_LSB - 1, OFFSET_LSB = 2;//2 for word align 

    localparam NUM_SETS = 1 << s;

    reg [`BLOCK_WIDTH] cache[NUM_SETS-1: 0][1: 0];//Two-way
    reg LRU [NUM_SETS-1: 0];//2-way pseudo-LRU
    
    /* Initialization */
    integer i;
    initial begin
        for (i = 0; i < NUM_SETS; i = i + 1) begin
            cache[i][0] = 0;
            cache[i][1] = 0;
            LRU[i] = 0;
        end
    end
    

    reg [2: 0] state =  3'd0;
    localparam idle = 3'd0, compareTag = 3'd1, allocate = 3'd2, writeback = 3'd3, writearound = 3'd4;


    /* Begin: Only dependent of Request */
    wire [`BLOCK_DATA_WIDTH] requestData = Request[`REQUEST_DATA];
    wire [31: 0] requestAddr = Request[`REQUEST_ADDR];
    wire [t-1: 0] requestTag = requestAddr[TAG_MSB: TAG_LSB];
    wire [s-1: 0] requestSet = requestAddr[SET_MSB: SET_LSB];
    
    //Only used in write
    wire [`BLOCK_OFFSET_WIDTH_NUM+2: 0] offset = {requestAddr[`BLOCK_OFFSET_WIDTH], 3'b0};
    reg [`BLOCK_DATA_WIDTH] dataMask, nmask;
    wire [`BLOCK_DATA_WIDTH] mask = ~nmask, writeMask = dataMask;
    always @(*) begin
        dataMask = 0;
        nmask = 0;
        if (Request[`REQUEST_WRITE_WIDTH] == `REQUEST_WRITE_BYTE) begin
            dataMask[offset +: 8] = requestData[offset +: 8];
            nmask[offset +:8] = ~0;
        end       
        else if (Request[`REQUEST_WRITE_WIDTH] == `REQUEST_WRITE_HALF) begin
            dataMask[offset +:16] = requestData[offset +: 16];
            nmask[offset+:16] = ~0;
        end
        else begin
            dataMask[offset+:32] = requestData[offset +: 32];
            nmask[offset+:32] = ~0;
        end
    end
   
    //Since we have two ways
    wire [`BLOCK_WIDTH] targetBlock1 = cache[requestSet][0];
    wire [`BLOCK_WIDTH] targetBlock2 = cache[requestSet][1];

    wire [1:0] used = requestTag == targetBlock1[`BLOCK_TAG] && targetBlock1[`BLOCK_VALID] ? 2'b10
                    : requestTag == targetBlock2[`BLOCK_TAG] && targetBlock2[`BLOCK_VALID] ? 2'b11
                    : 2'b00;//not valid
    wire hit = used[1];
    wire targetBlockIndex = used[0];
    /* End: Only dependent of Request */

    //Selected block by request
    wire [`BLOCK_WIDTH] targetBlock = cache[requestSet][targetBlockIndex];
    wire [31: 0] targetBlockAddr = {targetBlock[`BLOCK_TAG], requestSet, requestAddr[SET_MSB:0]};
    wire [`BLOCK_DATA_WIDTH] targetBlockData = targetBlock[`BLOCK_DATA];

    //When miss
    wire [`BLOCK_WIDTH] replaceBlock = cache[requestSet][LRU[requestSet]];

    wire [`REQUEST_WIDTH] replaceBlockRequest;
    assign replaceBlockRequest[`REQUEST_VALID] = 1;
    assign replaceBlockRequest[`REQUEST_ADDR] = {replaceBlock[`BLOCK_TAG], requestSet, requestAddr[SET_MSB: 0]};
    assign replaceBlockRequest[`REQUEST_TYPE] = `REQUEST_WRITE;
    assign replaceBlockRequest[`REQUEST_DATA] = replaceBlock[`BLOCK_DATA];

    assign RequestToNext
        = state == writearound ? Request
        : state == allocate    ? Request
        : state == writeback   ? replaceBlockRequest
        : 0;

    reg [`REQUEST_WIDTH] previousRequest = 0;
    //negedge
    always @ (negedge CLK) begin

        case (state) 

            idle:
                //Only run with request changes
                if (Request[`REQUEST_VALID]) begin
                    if (Request != previousRequest) begin
                        previousRequest <= Request;
                        Response[`RESPONSE_READY] <= 0;
                        state <= compareTag;
                    end
                    else Response[`RESPONSE_READY] <= 1;
                end
                else
                    Response[`RESPONSE_READY] <= 0;
                
            compareTag: begin

                /* Hit */
                if (hit) begin

                    state <= idle;
                    
                    Response[`RESPONSE_READY] <= 1;
                    LRU[requestSet] <= ~targetBlockIndex;

                    /* Write Hit */
                    if (Request[`REQUEST_TYPE] == `REQUEST_WRITE) begin

                        cache[requestSet][targetBlockIndex][`BLOCK_DIRTY] <= 1;
                        //FIXME variant width
                        cache[requestSet][targetBlockIndex][`BLOCK_DATA] <= (Request[`REQUEST_DATA] & mask) | writeMask;

                    end

                end
                /* Miss */
                else begin

                    /* Read miss with read-allocate */
                    if (Request[`REQUEST_TYPE] == `REQUEST_READ)

                        if (replaceBlock[`BLOCK_DIRTY]) 
                            state <= writeback;
                        else 
                            state <= allocate;

                    /* Write miss with no write-allocate */
                    else 
                        state <= writearound;

                end
            end

            allocate:
                if (ResponseFromNext[`RESPONSE_READY]) begin
                    /* Read into cache */
                    cache[requestSet][LRU[requestSet]][`BLOCK_DATA] <= ResponseFromNext[`RESPONSE_DATA];
                    cache[requestSet][LRU[requestSet]][`BLOCK_VALID] <= 1;
                    cache[requestSet][LRU[requestSet]][`BLOCK_DIRTY] <= 0;
                    
                    LRU[requestSet] <= ~LRU[requestSet];

                    Response[`RESPONSE_READY] <= 1;
                    Response[`RESPONSE_DATA] <= ResponseFromNext[`RESPONSE_DATA];

                    state <= idle;
                end
                else 
                    state <= allocate;

            //Read miss with dirty block: write-back the dirty block
            writeback:

                if (ResponseFromNext[`RESPONSE_READY]) begin
                    cache[requestSet][LRU[requestSet]][`BLOCK_DIRTY] <= 0;//Clear dirty
                    state <= allocate;
                end

                else 
                    state <= writeback;

            //Write miss: write-around
            writearound:
                if (ResponseFromNext[`RESPONSE_READY]) begin
                    state <= idle;
                    Response[`RESPONSE_READY] <= 1;
                end
                else state <= writearound;

        endcase
    end


endmodule

`endif
