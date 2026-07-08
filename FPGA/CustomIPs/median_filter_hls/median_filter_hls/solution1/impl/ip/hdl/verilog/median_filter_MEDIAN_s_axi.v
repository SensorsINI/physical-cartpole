// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
`timescale 1ns/1ps
module median_filter_MEDIAN_s_axi
#(parameter
    C_S_AXI_ADDR_WIDTH = 8,
    C_S_AXI_DATA_WIDTH = 32
)(
    input  wire                          ACLK,
    input  wire                          ARESET,
    input  wire                          ACLK_EN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] AWADDR,
    input  wire                          AWVALID,
    output wire                          AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0] WDATA,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0] WSTRB,
    input  wire                          WVALID,
    output wire                          WREADY,
    output wire [1:0]                    BRESP,
    output wire                          BVALID,
    input  wire                          BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] ARADDR,
    input  wire                          ARVALID,
    output wire                          ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0] RDATA,
    output wire [1:0]                    RRESP,
    output wire                          RVALID,
    input  wire                          RREADY,
    output wire [31:0]                   filtered_i,
    input  wire [31:0]                   filtered_o,
    input  wire                          filtered_o_ap_vld,
    output wire [31:0]                   raw_i,
    input  wire [31:0]                   raw_o,
    input  wire                          raw_o_ap_vld,
    output wire [31:0]                   window_size,
    output wire [31:0]                   trim_count,
    output wire [31:0]                   filter_mode,
    output wire [31:0]                   rail_low,
    output wire [31:0]                   rail_high,
    output wire [31:0]                   dz_status_i,
    input  wire [31:0]                   dz_status_o,
    input  wire                          dz_status_o_ap_vld,
    output wire [31:0]                   dz_window_i,
    input  wire [31:0]                   dz_window_o,
    input  wire                          dz_window_o_ap_vld,
    output wire [31:0]                   dz_age_i,
    input  wire [31:0]                   dz_age_o,
    input  wire                          dz_age_o_ap_vld,
    input  wire [31:0]                   dz_low_count,
    input  wire                          dz_low_count_ap_vld,
    input  wire [31:0]                   dz_high_count,
    input  wire                          dz_high_count_ap_vld
);
//------------------------Address Info-------------------
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of filtered_i
//        bit 31~0 - filtered_i[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of filtered_o
//        bit 31~0 - filtered_o[31:0] (Read)
// 0x1c : Control signal of filtered_o
//        bit 0  - filtered_o_ap_vld (Read/COR)
//        others - reserved
// 0x20 : Data signal of raw_i
//        bit 31~0 - raw_i[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of raw_o
//        bit 31~0 - raw_o[31:0] (Read)
// 0x2c : Control signal of raw_o
//        bit 0  - raw_o_ap_vld (Read/COR)
//        others - reserved
// 0x30 : Data signal of window_size
//        bit 31~0 - window_size[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of trim_count
//        bit 31~0 - trim_count[31:0] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of filter_mode
//        bit 31~0 - filter_mode[31:0] (Read/Write)
// 0x44 : reserved
// 0x48 : Data signal of rail_low
//        bit 31~0 - rail_low[31:0] (Read/Write)
// 0x4c : reserved
// 0x50 : Data signal of rail_high
//        bit 31~0 - rail_high[31:0] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of dz_status_i
//        bit 31~0 - dz_status_i[31:0] (Read/Write)
// 0x5c : reserved
// 0x60 : Data signal of dz_status_o
//        bit 31~0 - dz_status_o[31:0] (Read)
// 0x64 : Control signal of dz_status_o
//        bit 0  - dz_status_o_ap_vld (Read/COR)
//        others - reserved
// 0x68 : Data signal of dz_window_i
//        bit 31~0 - dz_window_i[31:0] (Read/Write)
// 0x6c : reserved
// 0x70 : Data signal of dz_window_o
//        bit 31~0 - dz_window_o[31:0] (Read)
// 0x74 : Control signal of dz_window_o
//        bit 0  - dz_window_o_ap_vld (Read/COR)
//        others - reserved
// 0x78 : Data signal of dz_age_i
//        bit 31~0 - dz_age_i[31:0] (Read/Write)
// 0x7c : reserved
// 0x80 : Data signal of dz_age_o
//        bit 31~0 - dz_age_o[31:0] (Read)
// 0x84 : Control signal of dz_age_o
//        bit 0  - dz_age_o_ap_vld (Read/COR)
//        others - reserved
// 0x88 : Data signal of dz_low_count
//        bit 31~0 - dz_low_count[31:0] (Read)
// 0x8c : Control signal of dz_low_count
//        bit 0  - dz_low_count_ap_vld (Read/COR)
//        others - reserved
// 0x98 : Data signal of dz_high_count
//        bit 31~0 - dz_high_count[31:0] (Read)
// 0x9c : Control signal of dz_high_count
//        bit 0  - dz_high_count_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

//------------------------Parameter----------------------
localparam
    ADDR_FILTERED_I_DATA_0    = 8'h10,
    ADDR_FILTERED_I_CTRL      = 8'h14,
    ADDR_FILTERED_O_DATA_0    = 8'h18,
    ADDR_FILTERED_O_CTRL      = 8'h1c,
    ADDR_RAW_I_DATA_0         = 8'h20,
    ADDR_RAW_I_CTRL           = 8'h24,
    ADDR_RAW_O_DATA_0         = 8'h28,
    ADDR_RAW_O_CTRL           = 8'h2c,
    ADDR_WINDOW_SIZE_DATA_0   = 8'h30,
    ADDR_WINDOW_SIZE_CTRL     = 8'h34,
    ADDR_TRIM_COUNT_DATA_0    = 8'h38,
    ADDR_TRIM_COUNT_CTRL      = 8'h3c,
    ADDR_FILTER_MODE_DATA_0   = 8'h40,
    ADDR_FILTER_MODE_CTRL     = 8'h44,
    ADDR_RAIL_LOW_DATA_0      = 8'h48,
    ADDR_RAIL_LOW_CTRL        = 8'h4c,
    ADDR_RAIL_HIGH_DATA_0     = 8'h50,
    ADDR_RAIL_HIGH_CTRL       = 8'h54,
    ADDR_DZ_STATUS_I_DATA_0   = 8'h58,
    ADDR_DZ_STATUS_I_CTRL     = 8'h5c,
    ADDR_DZ_STATUS_O_DATA_0   = 8'h60,
    ADDR_DZ_STATUS_O_CTRL     = 8'h64,
    ADDR_DZ_WINDOW_I_DATA_0   = 8'h68,
    ADDR_DZ_WINDOW_I_CTRL     = 8'h6c,
    ADDR_DZ_WINDOW_O_DATA_0   = 8'h70,
    ADDR_DZ_WINDOW_O_CTRL     = 8'h74,
    ADDR_DZ_AGE_I_DATA_0      = 8'h78,
    ADDR_DZ_AGE_I_CTRL        = 8'h7c,
    ADDR_DZ_AGE_O_DATA_0      = 8'h80,
    ADDR_DZ_AGE_O_CTRL        = 8'h84,
    ADDR_DZ_LOW_COUNT_DATA_0  = 8'h88,
    ADDR_DZ_LOW_COUNT_CTRL    = 8'h8c,
    ADDR_DZ_HIGH_COUNT_DATA_0 = 8'h98,
    ADDR_DZ_HIGH_COUNT_CTRL   = 8'h9c,
    WRIDLE                    = 2'd0,
    WRDATA                    = 2'd1,
    WRRESP                    = 2'd2,
    WRRESET                   = 2'd3,
    RDIDLE                    = 2'd0,
    RDDATA                    = 2'd1,
    RDRESET                   = 2'd2,
    ADDR_BITS                = 8;

//------------------------Local signal-------------------
    reg  [1:0]                    wstate = WRRESET;
    reg  [1:0]                    wnext;
    reg  [ADDR_BITS-1:0]          waddr;
    wire [C_S_AXI_DATA_WIDTH-1:0] wmask;
    wire                          aw_hs;
    wire                          w_hs;
    reg  [1:0]                    rstate = RDRESET;
    reg  [1:0]                    rnext;
    reg  [C_S_AXI_DATA_WIDTH-1:0] rdata;
    wire                          ar_hs;
    wire [ADDR_BITS-1:0]          raddr;
    // internal registers
    reg  [31:0]                   int_filtered_i = 'b0;
    reg  [31:0]                   int_filtered_o = 'b0;
    reg                           int_filtered_o_ap_vld;
    reg  [31:0]                   int_raw_i = 'b0;
    reg  [31:0]                   int_raw_o = 'b0;
    reg                           int_raw_o_ap_vld;
    reg  [31:0]                   int_window_size = 'b0;
    reg  [31:0]                   int_trim_count = 'b0;
    reg  [31:0]                   int_filter_mode = 'b0;
    reg  [31:0]                   int_rail_low = 'b0;
    reg  [31:0]                   int_rail_high = 'b0;
    reg  [31:0]                   int_dz_status_i = 'b0;
    reg  [31:0]                   int_dz_status_o = 'b0;
    reg                           int_dz_status_o_ap_vld;
    reg  [31:0]                   int_dz_window_i = 'b0;
    reg  [31:0]                   int_dz_window_o = 'b0;
    reg                           int_dz_window_o_ap_vld;
    reg  [31:0]                   int_dz_age_i = 'b0;
    reg  [31:0]                   int_dz_age_o = 'b0;
    reg                           int_dz_age_o_ap_vld;
    reg  [31:0]                   int_dz_low_count = 'b0;
    reg                           int_dz_low_count_ap_vld;
    reg  [31:0]                   int_dz_high_count = 'b0;
    reg                           int_dz_high_count_ap_vld;

//------------------------Instantiation------------------


//------------------------AXI write fsm------------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY  = (wstate == WRDATA);
assign BRESP   = 2'b00;  // OKAY
assign BVALID  = (wstate == WRRESP);
assign wmask   = { {8{WSTRB[3]}}, {8{WSTRB[2]}}, {8{WSTRB[1]}}, {8{WSTRB[0]}} };
assign aw_hs   = AWVALID & AWREADY;
assign w_hs    = WVALID & WREADY;

// wstate
always @(posedge ACLK) begin
    if (ARESET)
        wstate <= WRRESET;
    else if (ACLK_EN)
        wstate <= wnext;
end

// wnext
always @(*) begin
    case (wstate)
        WRIDLE:
            if (AWVALID)
                wnext = WRDATA;
            else
                wnext = WRIDLE;
        WRDATA:
            if (WVALID)
                wnext = WRRESP;
            else
                wnext = WRDATA;
        WRRESP:
            if (BREADY)
                wnext = WRIDLE;
            else
                wnext = WRRESP;
        default:
            wnext = WRIDLE;
    endcase
end

// waddr
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (aw_hs)
            waddr <= AWADDR[ADDR_BITS-1:0];
    end
end

//------------------------AXI read fsm-------------------
assign ARREADY = (rstate == RDIDLE);
assign RDATA   = rdata;
assign RRESP   = 2'b00;  // OKAY
assign RVALID  = (rstate == RDDATA);
assign ar_hs   = ARVALID & ARREADY;
assign raddr   = ARADDR[ADDR_BITS-1:0];

// rstate
always @(posedge ACLK) begin
    if (ARESET)
        rstate <= RDRESET;
    else if (ACLK_EN)
        rstate <= rnext;
end

// rnext
always @(*) begin
    case (rstate)
        RDIDLE:
            if (ARVALID)
                rnext = RDDATA;
            else
                rnext = RDIDLE;
        RDDATA:
            if (RREADY & RVALID)
                rnext = RDIDLE;
            else
                rnext = RDDATA;
        default:
            rnext = RDIDLE;
    endcase
end

// rdata
always @(posedge ACLK) begin
    if (ACLK_EN) begin
        if (ar_hs) begin
            rdata <= 'b0;
            case (raddr)
                ADDR_FILTERED_I_DATA_0: begin
                    rdata <= int_filtered_i[31:0];
                end
                ADDR_FILTERED_O_DATA_0: begin
                    rdata <= int_filtered_o[31:0];
                end
                ADDR_FILTERED_O_CTRL: begin
                    rdata[0] <= int_filtered_o_ap_vld;
                end
                ADDR_RAW_I_DATA_0: begin
                    rdata <= int_raw_i[31:0];
                end
                ADDR_RAW_O_DATA_0: begin
                    rdata <= int_raw_o[31:0];
                end
                ADDR_RAW_O_CTRL: begin
                    rdata[0] <= int_raw_o_ap_vld;
                end
                ADDR_WINDOW_SIZE_DATA_0: begin
                    rdata <= int_window_size[31:0];
                end
                ADDR_TRIM_COUNT_DATA_0: begin
                    rdata <= int_trim_count[31:0];
                end
                ADDR_FILTER_MODE_DATA_0: begin
                    rdata <= int_filter_mode[31:0];
                end
                ADDR_RAIL_LOW_DATA_0: begin
                    rdata <= int_rail_low[31:0];
                end
                ADDR_RAIL_HIGH_DATA_0: begin
                    rdata <= int_rail_high[31:0];
                end
                ADDR_DZ_STATUS_I_DATA_0: begin
                    rdata <= int_dz_status_i[31:0];
                end
                ADDR_DZ_STATUS_O_DATA_0: begin
                    rdata <= int_dz_status_o[31:0];
                end
                ADDR_DZ_STATUS_O_CTRL: begin
                    rdata[0] <= int_dz_status_o_ap_vld;
                end
                ADDR_DZ_WINDOW_I_DATA_0: begin
                    rdata <= int_dz_window_i[31:0];
                end
                ADDR_DZ_WINDOW_O_DATA_0: begin
                    rdata <= int_dz_window_o[31:0];
                end
                ADDR_DZ_WINDOW_O_CTRL: begin
                    rdata[0] <= int_dz_window_o_ap_vld;
                end
                ADDR_DZ_AGE_I_DATA_0: begin
                    rdata <= int_dz_age_i[31:0];
                end
                ADDR_DZ_AGE_O_DATA_0: begin
                    rdata <= int_dz_age_o[31:0];
                end
                ADDR_DZ_AGE_O_CTRL: begin
                    rdata[0] <= int_dz_age_o_ap_vld;
                end
                ADDR_DZ_LOW_COUNT_DATA_0: begin
                    rdata <= int_dz_low_count[31:0];
                end
                ADDR_DZ_LOW_COUNT_CTRL: begin
                    rdata[0] <= int_dz_low_count_ap_vld;
                end
                ADDR_DZ_HIGH_COUNT_DATA_0: begin
                    rdata <= int_dz_high_count[31:0];
                end
                ADDR_DZ_HIGH_COUNT_CTRL: begin
                    rdata[0] <= int_dz_high_count_ap_vld;
                end
            endcase
        end
    end
end


//------------------------Register logic-----------------
assign filtered_i  = int_filtered_i;
assign raw_i       = int_raw_i;
assign window_size = int_window_size;
assign trim_count  = int_trim_count;
assign filter_mode = int_filter_mode;
assign rail_low    = int_rail_low;
assign rail_high   = int_rail_high;
assign dz_status_i = int_dz_status_i;
assign dz_window_i = int_dz_window_i;
assign dz_age_i    = int_dz_age_i;
// int_filtered_i[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_filtered_i[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_FILTERED_I_DATA_0)
            int_filtered_i[31:0] <= (WDATA[31:0] & wmask) | (int_filtered_i[31:0] & ~wmask);
    end
end

// int_filtered_o
always @(posedge ACLK) begin
    if (ARESET)
        int_filtered_o <= 0;
    else if (ACLK_EN) begin
        if (filtered_o_ap_vld)
            int_filtered_o <= filtered_o;
    end
end

// int_filtered_o_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_filtered_o_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (filtered_o_ap_vld)
            int_filtered_o_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_FILTERED_O_CTRL)
            int_filtered_o_ap_vld <= 1'b0; // clear on read
    end
end

// int_raw_i[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_raw_i[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RAW_I_DATA_0)
            int_raw_i[31:0] <= (WDATA[31:0] & wmask) | (int_raw_i[31:0] & ~wmask);
    end
end

// int_raw_o
always @(posedge ACLK) begin
    if (ARESET)
        int_raw_o <= 0;
    else if (ACLK_EN) begin
        if (raw_o_ap_vld)
            int_raw_o <= raw_o;
    end
end

// int_raw_o_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_raw_o_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (raw_o_ap_vld)
            int_raw_o_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_RAW_O_CTRL)
            int_raw_o_ap_vld <= 1'b0; // clear on read
    end
end

// int_window_size[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_window_size[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_WINDOW_SIZE_DATA_0)
            int_window_size[31:0] <= (WDATA[31:0] & wmask) | (int_window_size[31:0] & ~wmask);
    end
end

// int_trim_count[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_trim_count[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_TRIM_COUNT_DATA_0)
            int_trim_count[31:0] <= (WDATA[31:0] & wmask) | (int_trim_count[31:0] & ~wmask);
    end
end

// int_filter_mode[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_filter_mode[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_FILTER_MODE_DATA_0)
            int_filter_mode[31:0] <= (WDATA[31:0] & wmask) | (int_filter_mode[31:0] & ~wmask);
    end
end

// int_rail_low[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_rail_low[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RAIL_LOW_DATA_0)
            int_rail_low[31:0] <= (WDATA[31:0] & wmask) | (int_rail_low[31:0] & ~wmask);
    end
end

// int_rail_high[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_rail_high[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RAIL_HIGH_DATA_0)
            int_rail_high[31:0] <= (WDATA[31:0] & wmask) | (int_rail_high[31:0] & ~wmask);
    end
end

// int_dz_status_i[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_status_i[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DZ_STATUS_I_DATA_0)
            int_dz_status_i[31:0] <= (WDATA[31:0] & wmask) | (int_dz_status_i[31:0] & ~wmask);
    end
end

// int_dz_status_o
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_status_o <= 0;
    else if (ACLK_EN) begin
        if (dz_status_o_ap_vld)
            int_dz_status_o <= dz_status_o;
    end
end

// int_dz_status_o_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_status_o_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (dz_status_o_ap_vld)
            int_dz_status_o_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_DZ_STATUS_O_CTRL)
            int_dz_status_o_ap_vld <= 1'b0; // clear on read
    end
end

// int_dz_window_i[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_window_i[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DZ_WINDOW_I_DATA_0)
            int_dz_window_i[31:0] <= (WDATA[31:0] & wmask) | (int_dz_window_i[31:0] & ~wmask);
    end
end

// int_dz_window_o
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_window_o <= 0;
    else if (ACLK_EN) begin
        if (dz_window_o_ap_vld)
            int_dz_window_o <= dz_window_o;
    end
end

// int_dz_window_o_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_window_o_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (dz_window_o_ap_vld)
            int_dz_window_o_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_DZ_WINDOW_O_CTRL)
            int_dz_window_o_ap_vld <= 1'b0; // clear on read
    end
end

// int_dz_age_i[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_age_i[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DZ_AGE_I_DATA_0)
            int_dz_age_i[31:0] <= (WDATA[31:0] & wmask) | (int_dz_age_i[31:0] & ~wmask);
    end
end

// int_dz_age_o
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_age_o <= 0;
    else if (ACLK_EN) begin
        if (dz_age_o_ap_vld)
            int_dz_age_o <= dz_age_o;
    end
end

// int_dz_age_o_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_age_o_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (dz_age_o_ap_vld)
            int_dz_age_o_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_DZ_AGE_O_CTRL)
            int_dz_age_o_ap_vld <= 1'b0; // clear on read
    end
end

// int_dz_low_count
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_low_count <= 0;
    else if (ACLK_EN) begin
        if (dz_low_count_ap_vld)
            int_dz_low_count <= dz_low_count;
    end
end

// int_dz_low_count_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_low_count_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (dz_low_count_ap_vld)
            int_dz_low_count_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_DZ_LOW_COUNT_CTRL)
            int_dz_low_count_ap_vld <= 1'b0; // clear on read
    end
end

// int_dz_high_count
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_high_count <= 0;
    else if (ACLK_EN) begin
        if (dz_high_count_ap_vld)
            int_dz_high_count <= dz_high_count;
    end
end

// int_dz_high_count_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_dz_high_count_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (dz_high_count_ap_vld)
            int_dz_high_count_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_DZ_HIGH_COUNT_CTRL)
            int_dz_high_count_ap_vld <= 1'b0; // clear on read
    end
end


//------------------------Memory logic-------------------

endmodule
