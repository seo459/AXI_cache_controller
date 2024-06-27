`timescale 1ns/1ps

module tb_RAM_integration;

    // Parameter definitions
    parameter ADDR_WIDTH = 32;
    parameter DATA_WIDTH = 8;
    parameter INDEX_WIDTH = 10;  // 2^10 = 1024 sets
    parameter OFFSET_WIDTH = 3; // 2^3 = 8 // one cache line contain 28-block (8(DATA_WIDTH) * 8 bits)
    parameter TAG_WIDTH = ADDR_WIDTH - INDEX_WIDTH - OFFSET_WIDTH;
    parameter ROW_SIZE = 128;
    parameter COL_SIZE = 8;
    parameter MAX_BURST_LENGTH = 256;
    parameter READ_DELAY = 10;
    parameter WEIGHT_FILE = "weight.txt";

    // Clock and reset
    reg clk;
    reg reset;

    // Host to SRAM (cache) signals
    reg  host_m_axi_ARVALID;
    wire host_m_axi_ARREADY;
    reg  [ADDR_WIDTH-1:0] host_m_axi_ARADDR;
    wire host_m_axi_RVALID;
    reg  host_m_axi_RREADY;
    wire [DATA_WIDTH-1:0] host_m_axi_RDATA;
    reg  host_m_axi_AWVALID;
    wire host_m_axi_AWREADY;
    reg  [ADDR_WIDTH-1:0] host_m_axi_AWADDR;
    reg  host_m_axi_WVALID;
    wire host_m_axi_WREADY;
    reg  [DATA_WIDTH-1:0] host_m_axi_WDATA;

    // SRAM (cache) to DRAM signals
    wire cache_m_axi_ARVALID;
    wire cache_m_axi_ARREADY;
    wire [ADDR_WIDTH-1:0] cache_m_axi_ARADDR;
    wire cache_m_axi_RVALID;
    wire cache_m_axi_RREADY;
    wire [DATA_WIDTH-1:0] cache_m_axi_RDATA;
    wire cache_m_axi_RLAST;

    wire cache_m_axi_AWVALID;
    wire cache_m_axi_AWREADY;
    wire [ADDR_WIDTH-1:0] cache_m_axi_AWADDR;
    wire cache_m_axi_WVALID;
    wire cache_m_axi_WREADY;
    wire [DATA_WIDTH-1:0] cache_m_axi_WDATA;

    // Hit signal
    wire hit;

    // Instantiate cache controller
    cache_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .INDEX_WIDTH(INDEX_WIDTH),
        .OFFSET_WIDTH(OFFSET_WIDTH),
        .TAG_WIDTH(TAG_WIDTH)
    ) uut_cache_controller (
        .clk(clk),
        .reset(reset),
        .hit(hit),
        .s_axi_ARVALID(host_m_axi_ARVALID),
        .s_axi_ARREADY(host_m_axi_ARREADY),
        .s_axi_ARADDR(host_m_axi_ARADDR),
        .s_axi_RVALID(host_m_axi_RVALID),
        .s_axi_RREADY(host_m_axi_RREADY),
        .s_axi_RDATA(host_m_axi_RDATA),
        .s_axi_AWVALID(host_m_axi_AWVALID),
        .s_axi_AWREADY(host_m_axi_AWREADY),
        .s_axi_AWADDR(host_m_axi_AWADDR),
        .s_axi_WVALID(host_m_axi_WVALID),
        .s_axi_WREADY(host_m_axi_WREADY),
        .s_axi_WDATA(host_m_axi_WDATA),
        .m_axi_ARVALID(cache_m_axi_ARVALID),
        .m_axi_ARREADY(cache_m_axi_ARREADY),
        .m_axi_ARADDR(cache_m_axi_ARADDR),
        .m_axi_RVALID(cache_m_axi_RVALID),
        .m_axi_RREADY(cache_m_axi_RREADY),
        .m_axi_RDATA(cache_m_axi_RDATA),
        .m_axi_AWVALID(cache_m_axi_AWVALID),
        .m_axi_AWREADY(cache_m_axi_AWREADY),
        .m_axi_AWADDR(cache_m_axi_AWADDR),
        .m_axi_WVALID(cache_m_axi_WVALID),
        .m_axi_WREADY(cache_m_axi_WREADY),
        .m_axi_WDATA(cache_m_axi_WDATA)
    );

    // Instantiate DRAM
    DRAM #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ROW_SIZE(ROW_SIZE),
        .COL_SIZE(COL_SIZE),
        .MAX_BURST_LENGTH(MAX_BURST_LENGTH),
        .READ_DELAY(READ_DELAY),
        .WEIGHT_FILE(WEIGHT_FILE)
    ) uut_DRAM (
        .clk(clk),
        .reset(reset),
        .s_axi_ARADDR(cache_m_axi_ARADDR),
        .s_axi_ARVALID(cache_m_axi_ARVALID),
        .s_axi_ARREADY(cache_m_axi_ARREADY),
        .s_axi_ARLEN(1 << OFFSET_WIDTH),
        .s_axi_RREADY(cache_m_axi_RREADY),
        .s_axi_RVALID(cache_m_axi_RVALID),
        .s_axi_RLAST(cache_m_axi_RLAST), // Not connected to cache_controller in this example
        .s_axi_RDATA(cache_m_axi_RDATA)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end

    // Test sequence
    initial begin
        // Initialize testbench signals
        reset = 1;
        host_m_axi_ARVALID = 0;
        host_m_axi_ARADDR = 0;
        host_m_axi_RREADY = 0;
        host_m_axi_AWVALID = 0;
        host_m_axi_AWADDR = 0;
        host_m_axi_WVALID = 0;
        host_m_axi_WDATA = 0;
        #20;
        reset = 0;
        #56;

        // Test 1: Read from cache (expect miss and fetch from DRAM)
        host_m_axi_ARADDR  = 32'd100; // Address to read
        host_m_axi_ARVALID = 1;
        host_m_axi_RREADY  = 1;
        #10;

        host_m_axi_ARADDR  = 0;
        host_m_axi_ARVALID = 0;

        // Wait for read to complete
        wait (host_m_axi_RVALID);
        #61;

        // Test 2: Write to cache
        
        host_m_axi_AWADDR = 32'd100; // Address to write
        host_m_axi_AWVALID = 1;
        #10;
        host_m_axi_AWADDR = 0;
        host_m_axi_AWVALID = 0;
        #20;
        host_m_axi_WDATA = 8'hCD; // Data to write
        host_m_axi_WVALID = 1;
        #10;
        host_m_axi_WDATA = 0;
        host_m_axi_WVALID = 0;

        #300;

        // Test 3: Read from cache
        host_m_axi_ARADDR  = 32'd100; // Address to read
        host_m_axi_ARVALID = 1;
        host_m_axi_RREADY  = 1;
        #10;

        host_m_axi_ARADDR  = 0;
        host_m_axi_ARVALID = 0;

        // Wait for read to complete
        wait (host_m_axi_RVALID);
        #200;


        // Finish simulation
        $stop;
    end

    // Dump waveforms for debugging
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_RAM_integration);
    end

endmodule
