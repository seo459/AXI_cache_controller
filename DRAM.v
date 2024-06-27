`timescale 1ns / 1ps

module DRAM #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 32,
    parameter ROW_SIZE = 128,
    parameter COL_SIZE = 8,
    parameter MAX_BURST_LENGTH = 256,
    parameter READ_DELAY = 10,
    parameter WEIGHT_FILE = "weight.txt"
) (
    input  wire                                  clk,
    input  wire                                  reset,

    input  wire [ADDR_WIDTH-1 : 0]               s_axi_ARADDR,     // slave side address (from cache).
    input  wire [$clog2(MAX_BURST_LENGTH)-1 : 0] s_axi_ARLEN,      // slave side burst length (from cache).
    output wire                                  s_axi_ARREADY,    // slave side ready (to cache).
    input  wire                                  s_axi_ARVALID,    // slave side address validation (from cache).

    input  wire                                  s_axi_RREADY,     // cache's ready to receive signal (from cache).
    output wire                                  s_axi_RVALID,     // slave side data validation (from DRAM).
    output wire                                  s_axi_RLAST,      // slave side data LAST (from DRAM).
    output wire [DATA_WIDTH-1 : 0]               s_axi_RDATA       // slave side data value (from DRAM).
);

    localparam MEM_SIZE = ROW_SIZE * COL_SIZE; // Local parameter for total memory size

    parameter IDLE       = 1'b0;
    parameter BURST_READ = 1'b1;

    reg [DATA_WIDTH-1 : 0] memory [0 : ROW_SIZE-1][0 : COL_SIZE-1]; // 2D DRAM memory array
    
    integer i, j;
    reg [DATA_WIDTH-1 : 0] flat_memory [0 : MEM_SIZE-1];

    // Initialize memory
    initial begin
        $readmemh(WEIGHT_FILE, flat_memory);
        for (i = 0; i < ROW_SIZE; i = i + 1) begin
            for (j = 0; j < COL_SIZE; j = j + 1) begin
                if (i * COL_SIZE + j < MEM_SIZE) begin
                    memory[i][j] = flat_memory[i * COL_SIZE + j];
                end else begin
                    memory[i][j] = {DATA_WIDTH{1'b0}}; // Initialize unassigned memory cells to 0
                end
            end
        end
    end

    reg [DATA_WIDTH-1 : 0]                r_s_axi_RDATA;
    reg [DATA_WIDTH-1 : 0]                next_s_axi_RDATA;

    reg [ADDR_WIDTH-1 : 0]                r_s_axi_ARADDR;
    reg [ADDR_WIDTH-1 : 0]                next_s_axi_ARADDR;
    
    reg [$clog2(MAX_BURST_LENGTH)-1 : 0]  r_s_axi_ARLEN;
    reg [$clog2(MAX_BURST_LENGTH)-1 : 0]  next_s_axi_ARLEN;

    reg [$clog2(MAX_BURST_LENGTH)-1 : 0]  burst_counter;
    reg [$clog2(MAX_BURST_LENGTH)-1 : 0]  next_burst_counter;
    
    reg [$clog2(ROW_SIZE)-1 : 0]          r_col;
    reg [$clog2(ROW_SIZE)-1 : 0]          next_col;

    reg [$clog2(COL_SIZE)-1 : 0]          r_row;
    reg [$clog2(COL_SIZE)-1 : 0]          next_row;
    
    reg [$clog2(READ_DELAY)-1: 0]         cnt;
    reg [$clog2(READ_DELAY)-1: 0]         next_cnt;

    reg  r_s_axi_RVALID;
    reg  next_s_axi_RVALID;

    reg  current_state;
    reg  next_state;

    assign s_axi_RVALID   =  r_s_axi_RVALID;
    assign s_axi_RDATA    =  r_s_axi_RDATA;
    assign s_axi_RLAST    = (burst_counter == (r_s_axi_ARLEN - 1)) ? 1'b1 : 1'b0;
    assign s_axi_ARREADY  = (current_state == IDLE)                ? 1'b1 : 1'b0;

    // combination logic
    always @(*) begin
        // Default assignments (default case)
        next_state          = current_state;
        next_s_axi_ARADDR   = r_s_axi_ARADDR;
        next_s_axi_ARLEN    = r_s_axi_ARLEN;
        next_s_axi_RVALID   = r_s_axi_RVALID;
        next_burst_counter  = burst_counter;
        next_cnt            = cnt;
        next_s_axi_RDATA    = r_s_axi_RDATA;
        next_col            = r_col;
        next_row            = r_row;

        case (current_state)
            IDLE: begin
                if (s_axi_ARVALID & s_axi_ARREADY) begin // cache request to DRMA: s_axi_ARVALID, DRAM ready : s_axi_ARREADY 
                    next_state          = BURST_READ;
                    next_s_axi_ARADDR = s_axi_ARADDR;
                    next_s_axi_ARLEN  = s_axi_ARLEN;
                    next_s_axi_RVALID = 0;
                    next_burst_counter  = 0;
                    next_cnt            = 0;
                    next_s_axi_RDATA  = 0;
                    next_col          = s_axi_ARADDR / COL_SIZE; // ARADDR / COL_SIZE : col ADDR
                    next_row          = s_axi_ARADDR % COL_SIZE; // ARADDR % COL_SIZE : row ADDR
                end
            end

            BURST_READ: begin
                if(cnt < (READ_DELAY -1)) begin // First Access Delay
                    next_cnt = cnt + 1;
                    next_s_axi_RVALID = 0;
                end
                else if (cnt >= (READ_DELAY -1)) begin // First Access start
                    if (burst_counter >= (r_s_axi_ARLEN - 1)) begin // Finish condition
                        next_state = IDLE;
                        next_burst_counter = 0;
                        next_s_axi_RVALID = 0;
                    end
                    else if(r_row >= (COL_SIZE - 1)) begin // go to next col.
                        next_cnt = 0;           // DRAM Row buffer miss penalty
                        next_row = 0;
                        next_col = r_col + 1; // Go to next col

                        next_burst_counter = burst_counter + 1; // revised.. PM.18:07
                        next_s_axi_RVALID = 0;
                    end
                    else if(s_axi_RREADY & s_axi_RVALID) begin // increase count
                        // slave(cache) ready to receive. And slave(DRAM) can send data to cache.
                        next_row = r_row + 1;
                        next_burst_counter  = burst_counter + 1;
                        next_s_axi_RVALID = 1;
                        next_s_axi_RDATA  = memory[r_col][r_row + 1];
                    end
                    else begin // Non-handshake
                        next_row = r_row;
                        next_burst_counter  = burst_counter;
                        next_s_axi_RVALID = 1;
                        next_s_axi_RDATA  = memory[r_col][r_row];
                    end
                end
            end
        endcase
    end

    // FF
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state  <= IDLE;
            burst_counter  <= 0;
            cnt            <= 0;
            r_s_axi_ARADDR <= 0;
            r_s_axi_ARLEN  <= 0;
            r_s_axi_RVALID <= 0;
            r_s_axi_RDATA  <= 0;
            r_row          <= 0;
            r_col          <= 0;
        end
        else begin
            current_state  <= next_state;
            burst_counter  <= next_burst_counter;
            cnt            <= next_cnt;
            r_s_axi_ARADDR <= next_s_axi_ARADDR;
            r_s_axi_ARLEN  <= next_s_axi_ARLEN;
            r_s_axi_RDATA  <= next_s_axi_RDATA; // registered output
            r_s_axi_RVALID <= next_s_axi_RVALID;
            r_row          <= next_row;
            r_col          <= next_col;
        end
    end

endmodule
