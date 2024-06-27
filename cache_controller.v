`timescale 1ns / 1ps
// DRAM is modeled as a non-modifiable data storage, similar to ROM

module cache_controller #( 
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 8,
    parameter INDEX_WIDTH = 10,  // 2^10 = 1024 sets
    parameter OFFSET_WIDTH = 4,  // 2^4 = 16 // one cache line contain 16-block (8(DATA_WIDTH) * 16 bits)
    parameter TAG_WIDTH = ADDR_WIDTH - INDEX_WIDTH - OFFSET_WIDTH  // remained bits => TAG
) (
    input  wire clk,
    input  wire reset,
    output wire hit,   
    
    /*-------- HOST to SRAM --------*/
    
    //s_AR channel
    input  wire s_axi_ARVALID, 
    output wire s_axi_ARREADY, 
    input  wire [ADDR_WIDTH-1:0] s_axi_ARADDR, 

    //s_R channel
    output wire s_axi_RVALID,
    input  wire s_axi_RREADY,
    output wire [DATA_WIDTH-1:0] s_axi_RDATA,

    //s_AW channel
    input  wire s_axi_AWVALID, 
    output wire s_axi_AWREADY, 
    input  wire [ADDR_WIDTH-1:0] s_axi_AWADDR,

    //s_W channel
    input  wire s_axi_WVALID,
    output wire s_axi_WREADY,
    input  wire [DATA_WIDTH-1:0] s_axi_WDATA,

    /*-------- SRAM to DRAM --------*/

    //m_AR channel
    output wire m_axi_ARVALID, 
    input  wire m_axi_ARREADY,
    output wire [ADDR_WIDTH-1:0] m_axi_ARADDR, 

    //m_R channel
    input  wire m_axi_RVALID,
    output wire m_axi_RREADY, 
    input  wire [DATA_WIDTH-1:0] m_axi_RDATA,
    
    //m_AW channel
    output wire m_axi_AWVALID,
    input  wire m_axi_AWREADY,
    output wire [ADDR_WIDTH-1:0] m_axi_AWADDR,

    //m_W channel
    output wire m_axi_WVALID,
    input  wire m_axi_WREADY,
    output wire [DATA_WIDTH-1:0] m_axi_WDATA
);

    localparam NUM_WAYS = 2;

    //SET, BLOCK_SIZE(LINE_SIZE)
    localparam NUM_SETS   = (1 << INDEX_WIDTH);
    localparam BLOCK_SIZE = (1 << OFFSET_WIDTH); 

    parameter IDLE       = 2'b00;
    parameter DRAM_READ  = 2'b10;
    parameter SRAM_READ  = 2'b01;
    parameter SRAM_WRITE = 2'b11;

    reg [1:0] current_state;
    reg [1:0] next_state;

    // Cache line structure
    reg cache_valid [0:NUM_WAYS-1][0:NUM_SETS-1];
    reg [TAG_WIDTH-1:0] cache_tag [0:NUM_WAYS-1][0:NUM_SETS-1];
    reg [DATA_WIDTH-1:0] cache_data [0:NUM_WAYS-1][0:NUM_SETS-1][0:BLOCK_SIZE-1];

    // LRU bit
    reg lru [0:NUM_SETS-1];

    // determine set (in all)
    reg [TAG_WIDTH-1:0]    r_input_tag;
    reg [TAG_WIDTH-1:0]    next_input_tag;
    // determine cache line (in set)
    reg [INDEX_WIDTH-1:0]  r_index;
    reg [INDEX_WIDTH-1:0]  next_index;
    // determine block (in cache line)
    reg [OFFSET_WIDTH-1:0] r_offset;
    reg [OFFSET_WIDTH-1:0] next_offset;

    integer i, j;

    reg next_hit;
    reg r_hit;

    reg [OFFSET_WIDTH-1:0] next_rd_cnt;
    reg [OFFSET_WIDTH-1:0] rd_cnt;

    reg next_write_way;
    reg write_way;

    // s_AR channel
    assign s_axi_ARREADY = (current_state == IDLE) ? 1'b1 : 1'b0;

    // s_R channel
    reg next_s_axi_RVALID;
    reg r_s_axi_RVALID;
    reg [DATA_WIDTH-1:0] next_s_axi_RDATA;
    reg [DATA_WIDTH-1:0] r_s_axi_RDATA;
    assign s_axi_RVALID = r_s_axi_RVALID;
    assign s_axi_RDATA  = r_s_axi_RDATA;

    reg next_s_axi_R_HANDSHAKE;
    reg r_s_axi_R_HANDSHAKE;

    // s_AW channel
    reg [ADDR_WIDTH-1:0] next_s_axi_AWADDR;
    reg [ADDR_WIDTH-1:0] r_s_axi_AWADDR;
    assign s_axi_AWREADY = (current_state == IDLE) ? 1'b1 : 1'b0;

    // s_W channel
    reg next_s_axi_WREADY;
    reg r_s_axi_WREADY;
    reg [DATA_WIDTH-1:0] next_s_axi_WDATA;
    reg [DATA_WIDTH-1:0] r_s_axi_WDATA;
    assign s_axi_WREADY = r_s_axi_WREADY;

    reg next_s_axi_W_HANDSHAKE;
    reg r_s_axi_W_HANDSHAKE;
     
    // m_AR channel
    reg next_m_axi_ARVALID;
    reg r_m_axi_ARVALID;
    reg [ADDR_WIDTH-1:0] next_m_axi_ARADDR;
    reg [ADDR_WIDTH-1:0] r_m_axi_ARADDR;
    assign m_axi_ARVALID = r_m_axi_ARVALID;
    assign m_axi_ARADDR  = r_m_axi_ARADDR;

    reg next_m_axi_AR_HANDSHAKE;
    reg r_m_axi_AR_HANDSHAKE;

    // m_R channel
    reg next_m_axi_RREADY;
    reg r_m_axi_RREADY;
    assign m_axi_RREADY = r_m_axi_RREADY;

    // m_AW channel
    reg next_m_axi_AWVALID;
    reg r_m_axi_AWVALID;
    reg [ADDR_WIDTH-1:0] next_m_axi_AWADDR;
    reg [ADDR_WIDTH-1:0] r_m_axi_AWADDR;
    assign m_axi_AWVALID = r_m_axi_AWVALID;
    assign m_axi_AWADDR  = r_m_axi_AWADDR;

    // m_W channel
    reg next_m_axi_WVALID;
    reg r_m_axi_WVALID;
    reg [DATA_WIDTH-1:0] next_m_axi_WDATA;
    reg [DATA_WIDTH-1:0] r_m_axi_WDATA;
    assign m_axi_WVALID = r_m_axi_WVALID;
    assign m_axi_WDATA  = r_m_axi_WDATA;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= IDLE;
            r_hit         <= 0;
            rd_cnt        <= 0;
            write_way     <= 0;

            r_input_tag <= 0;
            r_index     <= 0;
            r_offset    <= 0;

            r_s_axi_RVALID      <= 0;
            r_s_axi_RDATA       <= 0;
            r_s_axi_R_HANDSHAKE <= 0;

            r_s_axi_AWADDR <= 0;

            r_s_axi_WREADY      <= 0;
            r_s_axi_WDATA       <= 0;
            r_s_axi_W_HANDSHAKE <= 0;

            r_m_axi_ARVALID      <= 0;
            r_m_axi_ARADDR       <= 0;
            r_m_axi_AR_HANDSHAKE <= 0;

            r_m_axi_RREADY  <= 0;

            r_m_axi_AWVALID <= 0;
            r_m_axi_AWADDR  <= 0;

            r_m_axi_WVALID  <= 0;
            r_m_axi_WDATA   <= 0;

            for (i = 0; i < NUM_WAYS; i = i + 1) begin
                for (j = 0; j < NUM_SETS; j = j + 1) begin
                    cache_valid[i][j] <= 0;
                end
            end

            for (i = 0; i < NUM_SETS; i = i + 1) begin
                lru[i] <= 1;
            end
        end 
        else begin
            current_state <= next_state;
            r_hit     <= next_hit;
            rd_cnt    <= next_rd_cnt;
            write_way <= next_write_way;

            r_input_tag <= next_input_tag;
            r_index     <= next_index;
            r_offset    <= next_offset;

            r_s_axi_RVALID      <= next_s_axi_RVALID;
            r_s_axi_RDATA       <= next_s_axi_RDATA;
            r_s_axi_R_HANDSHAKE <= next_s_axi_R_HANDSHAKE;

            r_s_axi_AWADDR      <= next_s_axi_AWADDR;

            r_s_axi_WREADY      <= next_s_axi_WREADY;
            r_s_axi_WDATA       <= next_s_axi_WDATA;
            r_s_axi_W_HANDSHAKE <= next_s_axi_W_HANDSHAKE;

            r_m_axi_ARVALID      <= next_m_axi_ARVALID;
            r_m_axi_ARADDR       <= next_m_axi_ARADDR;
            r_m_axi_AR_HANDSHAKE <= next_m_axi_AR_HANDSHAKE;

            r_m_axi_RREADY  <= next_m_axi_RREADY;

            r_m_axi_AWVALID <= next_m_axi_AWVALID;
            r_m_axi_AWADDR  <= next_m_axi_AWADDR;

            r_m_axi_WVALID <= next_m_axi_WVALID;
            r_m_axi_WDATA  <= next_m_axi_WDATA;
        end
    end
    
    always @(*) begin
        // Default assignments
        next_state          = current_state;
        next_hit            = r_hit;
        next_rd_cnt         = rd_cnt;
        next_write_way      = write_way;

        next_input_tag      = r_input_tag;
        next_index          = r_index;
        next_offset         = r_offset;

        // s_R
        next_s_axi_RVALID   = r_s_axi_RVALID;
        next_s_axi_RDATA    = r_s_axi_RDATA;
        next_s_axi_R_HANDSHAKE = r_s_axi_R_HANDSHAKE;

        // s_AW
        next_s_axi_AWADDR   = r_s_axi_AWADDR;

        // s_W
        next_s_axi_WREADY      = r_s_axi_WREADY;
        next_s_axi_WDATA       = r_s_axi_WDATA;
        next_s_axi_W_HANDSHAKE = r_s_axi_W_HANDSHAKE;

        // m_AR
        next_m_axi_ARVALID      = r_m_axi_ARVALID;
        next_m_axi_ARADDR       = r_m_axi_ARADDR;
        next_m_axi_AR_HANDSHAKE = r_m_axi_AR_HANDSHAKE;

        // m_R
        next_m_axi_RREADY    = r_m_axi_RREADY;

        // m_AW
        next_m_axi_AWVALID  = r_m_axi_AWVALID;
        next_m_axi_AWADDR   = r_m_axi_AWADDR;

        // m_W
        next_m_axi_WDATA    = r_m_axi_WDATA;
        next_m_axi_WVALID   = r_m_axi_WVALID;
    
        case (current_state)
            IDLE: begin
                if (s_axi_ARVALID & s_axi_ARREADY) begin // HOST READ TO SRAM
                    next_input_tag = s_axi_ARADDR[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
                    next_index     = s_axi_ARADDR[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH];
                    next_offset    = s_axi_ARADDR[OFFSET_WIDTH-1:0];

                    for (i = 0; i < NUM_WAYS; i = i + 1) begin
                        if (cache_valid[i][s_axi_ARADDR[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH]] && (cache_tag[i][s_axi_ARADDR[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH]] == s_axi_ARADDR[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH])) begin
                            next_hit = 1;
                            lru[s_axi_ARADDR[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH]] = (i == 0) ? 1 : 0;
                        end    
                    end

                    if (!next_hit) begin
                        next_state              = DRAM_READ;
                        next_m_axi_ARVALID      = 1;
                        next_m_axi_AR_HANDSHAKE = 0;
                        next_m_axi_ARADDR = s_axi_ARADDR;

                        next_s_axi_RVALID       = 0;
                        next_rd_cnt             = 0;
                    end
                    else begin // if(next_hit)
                        next_state             = SRAM_READ;
                        next_s_axi_R_HANDSHAKE = 0;
                        next_s_axi_RVALID      = 0;

                    end
                end

                else if (s_axi_AWVALID && s_axi_AWREADY) begin // HOST WRITE TO SRAM
                    next_state             = SRAM_WRITE;
                    next_s_axi_WREADY      = 0;
                    next_s_axi_W_HANDSHAKE = 0;

                    next_input_tag   = s_axi_AWADDR[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];
                    next_index       = s_axi_AWADDR[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH];
                    next_offset      = s_axi_AWADDR[OFFSET_WIDTH-1:0];
                    

                    if(lru[s_axi_AWADDR[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH]] == 0) begin
                        next_write_way = 1'b0;
                    end
                    else begin
                        next_write_way = 1'b1;
                    end
                end
                else begin
                    next_state = IDLE;
                end
            end

            DRAM_READ: begin 
                if(!r_m_axi_AR_HANDSHAKE) begin
                    if(m_axi_ARVALID & m_axi_ARREADY) begin
                        next_state              = DRAM_READ;
                        next_m_axi_AR_HANDSHAKE = 1;
                        next_m_axi_RREADY       = 1; 
                        // next_m_axi_ARADDR (X)
                    end
                end

                else begin // if(r_m_axi_AR_HANDSHAKE)
                    if(m_axi_RREADY & m_axi_RVALID) begin
                        cache_data[lru[r_index]][r_index][rd_cnt] = m_axi_RDATA;
                        
                        if(rd_cnt >= ((1 << OFFSET_WIDTH) - 1)) begin
                            cache_tag[lru[r_index]][r_index]   = r_input_tag;
                            cache_valid[lru[r_index]][r_index] = 1;
                            
                            next_state              = SRAM_READ; // GO TO SRAM READ
                            next_rd_cnt             = 0;
                            next_m_axi_AR_HANDSHAKE = 0;

                            next_input_tag          = r_input_tag; // HOST가 맨처음 요청했던 주소의 정보이다.
                            next_index              = r_index;
                            next_offset             = r_offset;

                            next_s_axi_R_HANDSHAKE  = 0;
                            next_s_axi_RVALID       = 0;
                            next_m_axi_RREADY       = 0;
                            
                        end

                        else begin // if(rd_cnt < (1 << OFFSET_WIDTH) -1 )
                            next_rd_cnt = rd_cnt + 1;
                            next_state  = DRAM_READ;
                        end
                    end
                    else begin // if(!m_axi_RVALID)
                        next_state = DRAM_READ;
                        // waitting for DRAM's RVALID... (Caution DEADLOCK)
                    end
                end

            end

            SRAM_READ: begin
                if(!r_s_axi_R_HANDSHAKE) begin // initial, r_s_axi_R_HANDSHAKE = 0 
                    next_state        = SRAM_READ;
                    next_s_axi_RDATA  = cache_data[~lru[r_index]][r_index][r_offset];
                    next_s_axi_RVALID = 1; // 1-cycle READY to READ

                    if(s_axi_RREADY & s_axi_RVALID) begin
                        next_s_axi_R_HANDSHAKE = 1;
                    end
                    else begin // Waitting for HOST
                        next_s_axi_R_HANDSHAKE = 0;
                    end
                end

                else if(r_s_axi_R_HANDSHAKE) begin
                    next_state             = IDLE;
                    next_s_axi_RVALID      = 0;
                    next_s_axi_R_HANDSHAKE = 0;
                    // next_s_axi_RDATA    = 8'd0;
                end
            end

            SRAM_WRITE: begin
                if(!r_s_axi_W_HANDSHAKE) begin
                    next_state             = SRAM_WRITE;
                    next_s_axi_WREADY      = 1;

                    if(s_axi_WVALID & s_axi_WREADY) begin
                        cache_data[lru[r_index]][r_index][r_offset] = s_axi_WDATA;
                        cache_tag[lru[r_index]][r_index]            = r_input_tag;
                        cache_valid[lru[r_index]][r_index]          = 1;
                        next_s_axi_W_HANDSHAKE = 1;
                    end
                    else begin
                        next_s_axi_W_HANDSHAKE = 0;
                    end
                end

                else if(r_s_axi_W_HANDSHAKE) begin
                    next_state             = IDLE;
                    
                    next_s_axi_WREADY      = 0;
                    next_s_axi_W_HANDSHAKE = 0;
                    // next_s_axi_WDATA    = 8'd0;
                end
            end
        endcase
    end

    assign hit = r_hit;

endmodule
