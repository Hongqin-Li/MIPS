`define CPU_REQUEST_WIDTH 66: 0
`define CPU_REQUEST_ADDR 31: 0
`define CPU_REQUEST_DATA 63: 32

`define CPU_REQUEST_VALID 64

`define CPU_REQUEST_TYPE 66
`define REQUEST_READ 1'b0
`define REQUEST_WRITE 1'b1

`define RESPONSE_TO_CPU_WIDTH 32:0
`define RESPONSE_TO_CPU_DATA 31:0
`define RESPONSE_TO_CPU_READY 32//only needed by read


//TODO be more flexibility
`define BLOCK_SIZE_LENGTH 4
`define BLOCK_SIZE_WIDTH `BLOCK_SIZE_LENGTH-1:0
`define BLOCK_SIZE_ALIGN_WORD_LENGTH (`BLOCK_SIZE_LENGTH - 2)
`define BLOCK_SIZE_BY_WORD 4//number of word in a block
`define BLOCK_SIZE_BY_BYTE 16
`define BLOCK_SIZE_BY_BIT 128

`define BLOCK_WIDTH `BLOCK_SIZE_BY_BIT+1+t:0
`define BLOCK_VALID 0
`define BLOCK_DIRTY 1
`define BLOCK_DATA `BLOCK_SIZE_BY_BIT + 1: 2
`define BLOCK_TAG `BLOCK_SIZE_BY_BIT + 1 + t:`BLOCK_SIZE_BY_BIT + 2

`define REQUEST_WIDTH 33 + BLOCK_SIZE_BY_BIT: 0
`define REQUEST_ADDR 31: 0
`define REQUEST_VALID 32
`define REQUEST_TYPE 33
    `define REQUEST_READ 1'b0
    `define REQUEST_WRITE 1'b1
`define REQUEST_DATA 33 + BLOCK_SIZE_BY_BIT: 34

`define RESPONSE_WIDTH BLOCK_SIZE_BY_BIT:0
`define RESPONSE_READY 0
`define RESPONSE_DATA BLOCK_SIZE_BY_BIT:1


module Memory(
    input CLK, RST,
    input [`REQUEST_WIDTH] Request,
    output reg [`RESPONSE_WIDTH] Response);

    reg [BLOCK_SIZE_WIDTH] ROM[256:0];
    reg [4:0] cnt = 0;
    wire [BLOCK_SIZE_WIDTH] readData = ROM[Request[`REQUEST_ADDR][31:`BLOCK_SIZE_LENGTH]];
   
    always @ (posedge CLK)
        if (Request[`REQUEST_VALID]) begin
            if (cnt[4]) begin
                cnt <= 0;
                Response[`RESPONSE_DATA] <= readData;
                Response[`RESPONSE_READY] <= 1;
            end
            else cnt <= cnt + 1;
        end
        else begin
            cnt <= 0;
            Response[`RESPONSE_READY] <= 0;
        end


    initial 
    begin

        ROM[0] = 64'hfedcba987654321;
        ROM[1] = 64'hfffffffffffffff;

    end


endmodule

module Cache(
    input CLK, RST,
    /* Request from higher level */
    input [`REQUEST_WIDTH] CPURequest,
    /* Request to lower level */
    output reg [`REQUEST_WIDTH] RequestToNext,
    /* Response from lower level */
    input [`RESPONSE_WIDTH] ResponseFromNext,
    /* Response to higher level */
    output reg [`RESPONSE_WIDTH] Response
);

    /* 
        T: Tag length
        S: Set length
        E: Number of blocks per set 
        T + S <= 30 to reserve 2 bits for word align
    */
    parameter s = 2, e = `BLOCK_SIZE_LENGTH;
    parameter t = 32 - s - e;

    parameter TAG_MSB = 31, TAG_LSB = 32 - t;
    parameter SET_MSB = TAG_LSB - 1, SET_LSB = TAG_LSB - s;
    parameter OFFSET_MSB = SET_LSB - 1, OFFSET_LSB = 2;//2 for word align 

    parameter NUM_SETS = 1 << s, NUM_BLOCKS = 1 << e;


    reg [`BLOCK_WIDTH] cache[NUM_SETS-1: 0][NUM_BLOCKS-1: 0];
    reg LRU [NUM_SETS-1: 0];
    //TODO: initialization
    //reg [e-1: 0] LRU [NUM_SETS-1: 0];

    reg [2: 0] state =  3'b0, nextState;
    parameter idle = 3'b0, compareTag = 3'd2, allocate = 3'b3, writeback = 3'd4, writearound = 3'd5;


    /* Only dependent of Request */
    wire [31: 0] requestAddr = Request[`REQUEST_ADDR];
    wire [t-1: 0] requestTag = requestAddr[TAG_MSB: TAG_LSB];
    wire [s-1: 0] requestSet = requestAddr[SET_MSB: SET_LSB];

    //Since we have two ways
    wire [`BLOCK_WIDTH] targetBlock1 = cache[requestSet][0];
    wire [`BLOCK_WIDTH] targetBlock2 = cache[requestSet][1];

    wire [1:0] used = requestTag == targetBlock1[`BLOCK_TAG] && targetBlock1[`BLOCK_VALID] ? 2'b10
                    : requestTag == targetBlock2[`BLOCK_TAG] && targetBlock2[`BLOCK_VALID] ? 2'b11
                    : 2'b00;//not valid
    wire hit = used[1];
    wire targetBlockIndex = used[0];
    /* Only dependent of Request */

    //Selected block by request
    wire targetBlock = cache[requestSet][targetBlockIndex];
    wire targetBlockAddr = {targetBlock[`BLOCK_TAG], requestSet, requestAddr[SET_MSB:0]};
    wire targetBlockData = targetBlock[`BLOCK_DATA];

    //When miss
    wire replaceBlock = cache[requestSet][LRU[requestSet]];

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


    always @ (posedge CLK) begin

        case (state) 

            idle: begin

                Response[`RESPONSE_READY] <= 0;

                if (Request[`REQUEST_VALID])
                    state <= compareTag;

            end

            compareTag: begin

                /* Hit */
                if (hit) begin

                    state <= idle;
                    
                    Response[`RESPONSE_READY] <= 1;
                    LRU[requestSet] <= ~blockIndex;

                    /* Write Hit with write-back */
                    if (request[`REQUEST_TYPE] == `REQUEST_WRITE) begin

                        cache[requestSet][blockIndex][`BLOCK_DIRTY] <= 1;
                        cache[requestSet][blockIndex][`BLOCK_DATA] <= Request[`REQUEST_DATA];

                    end

                end
                /* Miss */
                else begin

                    /* Read miss with read-allocate */
                    if (request[`REQUEST_TYPE] == `REQUEST_READ)

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
                    state <= idle
                    Response[`RESPONSE_READY] <= 1;
                end
                else state <= writearound;

        endcase
    end


endmodule
