// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
`timescale 1ns/1ps
module secloc_shell_CTRL_s_axi
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
    output wire                          interrupt,
    output wire [31:0]                   angleD,
    output wire [31:0]                   angle_cos,
    output wire [31:0]                   angle_sin,
    output wire [31:0]                   position,
    output wire [31:0]                   positionD,
    output wire [31:0]                   target_equilibrium,
    output wire [31:0]                   target_position,
    output wire [31:0]                   angle,
    output wire [31:0]                   tick,
    output wire [31:0]                   log_base,
    output wire [31:0]                   ref_period_ticks,
    output wire [31:0]                   dead_ang,
    output wire [31:0]                   dead_pos,
    output wire [31:0]                   control_flags,
    input  wire [31:0]                   Q,
    input  wire                          Q_ap_vld,
    input  wire [31:0]                   status,
    input  wire                          status_ap_vld,
    input  wire [31:0]                   update_count,
    input  wire                          update_count_ap_vld,
    input  wire [31:0]                   nn_wait_cycles,
    input  wire                          nn_wait_cycles_ap_vld,
    output wire                          ap_start,
    input  wire                          ap_done,
    input  wire                          ap_ready,
    input  wire                          ap_idle
);
//------------------------Address Info-------------------
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - enable ap_done interrupt (Read/Write)
//        bit 1  - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - ap_done (COR/TOW)
//        bit 1  - ap_ready (COR/TOW)
//        others - reserved
// 0x10 : Data signal of angleD
//        bit 31~0 - angleD[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of angle_cos
//        bit 31~0 - angle_cos[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of angle_sin
//        bit 31~0 - angle_sin[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of position
//        bit 31~0 - position[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of positionD
//        bit 31~0 - positionD[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of target_equilibrium
//        bit 31~0 - target_equilibrium[31:0] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of target_position
//        bit 31~0 - target_position[31:0] (Read/Write)
// 0x44 : reserved
// 0x48 : Data signal of angle
//        bit 31~0 - angle[31:0] (Read/Write)
// 0x4c : reserved
// 0x50 : Data signal of tick
//        bit 31~0 - tick[31:0] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of log_base
//        bit 31~0 - log_base[31:0] (Read/Write)
// 0x5c : reserved
// 0x60 : Data signal of ref_period_ticks
//        bit 31~0 - ref_period_ticks[31:0] (Read/Write)
// 0x64 : reserved
// 0x68 : Data signal of dead_ang
//        bit 31~0 - dead_ang[31:0] (Read/Write)
// 0x6c : reserved
// 0x70 : Data signal of dead_pos
//        bit 31~0 - dead_pos[31:0] (Read/Write)
// 0x74 : reserved
// 0x78 : Data signal of control_flags
//        bit 31~0 - control_flags[31:0] (Read/Write)
// 0x7c : reserved
// 0x80 : Data signal of Q
//        bit 31~0 - Q[31:0] (Read)
// 0x84 : Control signal of Q
//        bit 0  - Q_ap_vld (Read/COR)
//        others - reserved
// 0x90 : Data signal of status
//        bit 31~0 - status[31:0] (Read)
// 0x94 : Control signal of status
//        bit 0  - status_ap_vld (Read/COR)
//        others - reserved
// 0xa0 : Data signal of update_count
//        bit 31~0 - update_count[31:0] (Read)
// 0xa4 : Control signal of update_count
//        bit 0  - update_count_ap_vld (Read/COR)
//        others - reserved
// 0xb0 : Data signal of nn_wait_cycles
//        bit 31~0 - nn_wait_cycles[31:0] (Read)
// 0xb4 : Control signal of nn_wait_cycles
//        bit 0  - nn_wait_cycles_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

//------------------------Parameter----------------------
localparam
    ADDR_AP_CTRL                   = 8'h00,
    ADDR_GIE                       = 8'h04,
    ADDR_IER                       = 8'h08,
    ADDR_ISR                       = 8'h0c,
    ADDR_ANGLED_DATA_0             = 8'h10,
    ADDR_ANGLED_CTRL               = 8'h14,
    ADDR_ANGLE_COS_DATA_0          = 8'h18,
    ADDR_ANGLE_COS_CTRL            = 8'h1c,
    ADDR_ANGLE_SIN_DATA_0          = 8'h20,
    ADDR_ANGLE_SIN_CTRL            = 8'h24,
    ADDR_POSITION_DATA_0           = 8'h28,
    ADDR_POSITION_CTRL             = 8'h2c,
    ADDR_POSITIOND_DATA_0          = 8'h30,
    ADDR_POSITIOND_CTRL            = 8'h34,
    ADDR_TARGET_EQUILIBRIUM_DATA_0 = 8'h38,
    ADDR_TARGET_EQUILIBRIUM_CTRL   = 8'h3c,
    ADDR_TARGET_POSITION_DATA_0    = 8'h40,
    ADDR_TARGET_POSITION_CTRL      = 8'h44,
    ADDR_ANGLE_DATA_0              = 8'h48,
    ADDR_ANGLE_CTRL                = 8'h4c,
    ADDR_TICK_DATA_0               = 8'h50,
    ADDR_TICK_CTRL                 = 8'h54,
    ADDR_LOG_BASE_DATA_0           = 8'h58,
    ADDR_LOG_BASE_CTRL             = 8'h5c,
    ADDR_REF_PERIOD_TICKS_DATA_0   = 8'h60,
    ADDR_REF_PERIOD_TICKS_CTRL     = 8'h64,
    ADDR_DEAD_ANG_DATA_0           = 8'h68,
    ADDR_DEAD_ANG_CTRL             = 8'h6c,
    ADDR_DEAD_POS_DATA_0           = 8'h70,
    ADDR_DEAD_POS_CTRL             = 8'h74,
    ADDR_CONTROL_FLAGS_DATA_0      = 8'h78,
    ADDR_CONTROL_FLAGS_CTRL        = 8'h7c,
    ADDR_Q_DATA_0                  = 8'h80,
    ADDR_Q_CTRL                    = 8'h84,
    ADDR_STATUS_DATA_0             = 8'h90,
    ADDR_STATUS_CTRL               = 8'h94,
    ADDR_UPDATE_COUNT_DATA_0       = 8'ha0,
    ADDR_UPDATE_COUNT_CTRL         = 8'ha4,
    ADDR_NN_WAIT_CYCLES_DATA_0     = 8'hb0,
    ADDR_NN_WAIT_CYCLES_CTRL       = 8'hb4,
    WRIDLE                         = 2'd0,
    WRDATA                         = 2'd1,
    WRRESP                         = 2'd2,
    WRRESET                        = 2'd3,
    RDIDLE                         = 2'd0,
    RDDATA                         = 2'd1,
    RDRESET                        = 2'd2,
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
    reg                           int_ap_idle;
    reg                           int_ap_ready;
    reg                           int_ap_done = 1'b0;
    reg                           int_ap_start = 1'b0;
    reg                           int_auto_restart = 1'b0;
    reg                           int_gie = 1'b0;
    reg  [1:0]                    int_ier = 2'b0;
    reg  [1:0]                    int_isr = 2'b0;
    reg  [31:0]                   int_angleD = 'b0;
    reg  [31:0]                   int_angle_cos = 'b0;
    reg  [31:0]                   int_angle_sin = 'b0;
    reg  [31:0]                   int_position = 'b0;
    reg  [31:0]                   int_positionD = 'b0;
    reg  [31:0]                   int_target_equilibrium = 'b0;
    reg  [31:0]                   int_target_position = 'b0;
    reg  [31:0]                   int_angle = 'b0;
    reg  [31:0]                   int_tick = 'b0;
    reg  [31:0]                   int_log_base = 'b0;
    reg  [31:0]                   int_ref_period_ticks = 'b0;
    reg  [31:0]                   int_dead_ang = 'b0;
    reg  [31:0]                   int_dead_pos = 'b0;
    reg  [31:0]                   int_control_flags = 'b0;
    reg  [31:0]                   int_Q = 'b0;
    reg                           int_Q_ap_vld;
    reg  [31:0]                   int_status = 'b0;
    reg                           int_status_ap_vld;
    reg  [31:0]                   int_update_count = 'b0;
    reg                           int_update_count_ap_vld;
    reg  [31:0]                   int_nn_wait_cycles = 'b0;
    reg                           int_nn_wait_cycles_ap_vld;

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
                ADDR_AP_CTRL: begin
                    rdata[0] <= int_ap_start;
                    rdata[1] <= int_ap_done;
                    rdata[2] <= int_ap_idle;
                    rdata[3] <= int_ap_ready;
                    rdata[7] <= int_auto_restart;
                end
                ADDR_GIE: begin
                    rdata <= int_gie;
                end
                ADDR_IER: begin
                    rdata <= int_ier;
                end
                ADDR_ISR: begin
                    rdata <= int_isr;
                end
                ADDR_ANGLED_DATA_0: begin
                    rdata <= int_angleD[31:0];
                end
                ADDR_ANGLE_COS_DATA_0: begin
                    rdata <= int_angle_cos[31:0];
                end
                ADDR_ANGLE_SIN_DATA_0: begin
                    rdata <= int_angle_sin[31:0];
                end
                ADDR_POSITION_DATA_0: begin
                    rdata <= int_position[31:0];
                end
                ADDR_POSITIOND_DATA_0: begin
                    rdata <= int_positionD[31:0];
                end
                ADDR_TARGET_EQUILIBRIUM_DATA_0: begin
                    rdata <= int_target_equilibrium[31:0];
                end
                ADDR_TARGET_POSITION_DATA_0: begin
                    rdata <= int_target_position[31:0];
                end
                ADDR_ANGLE_DATA_0: begin
                    rdata <= int_angle[31:0];
                end
                ADDR_TICK_DATA_0: begin
                    rdata <= int_tick[31:0];
                end
                ADDR_LOG_BASE_DATA_0: begin
                    rdata <= int_log_base[31:0];
                end
                ADDR_REF_PERIOD_TICKS_DATA_0: begin
                    rdata <= int_ref_period_ticks[31:0];
                end
                ADDR_DEAD_ANG_DATA_0: begin
                    rdata <= int_dead_ang[31:0];
                end
                ADDR_DEAD_POS_DATA_0: begin
                    rdata <= int_dead_pos[31:0];
                end
                ADDR_CONTROL_FLAGS_DATA_0: begin
                    rdata <= int_control_flags[31:0];
                end
                ADDR_Q_DATA_0: begin
                    rdata <= int_Q[31:0];
                end
                ADDR_Q_CTRL: begin
                    rdata[0] <= int_Q_ap_vld;
                end
                ADDR_STATUS_DATA_0: begin
                    rdata <= int_status[31:0];
                end
                ADDR_STATUS_CTRL: begin
                    rdata[0] <= int_status_ap_vld;
                end
                ADDR_UPDATE_COUNT_DATA_0: begin
                    rdata <= int_update_count[31:0];
                end
                ADDR_UPDATE_COUNT_CTRL: begin
                    rdata[0] <= int_update_count_ap_vld;
                end
                ADDR_NN_WAIT_CYCLES_DATA_0: begin
                    rdata <= int_nn_wait_cycles[31:0];
                end
                ADDR_NN_WAIT_CYCLES_CTRL: begin
                    rdata[0] <= int_nn_wait_cycles_ap_vld;
                end
            endcase
        end
    end
end


//------------------------Register logic-----------------
assign interrupt          = int_gie & (|int_isr);
assign ap_start           = int_ap_start;
assign angleD             = int_angleD;
assign angle_cos          = int_angle_cos;
assign angle_sin          = int_angle_sin;
assign position           = int_position;
assign positionD          = int_positionD;
assign target_equilibrium = int_target_equilibrium;
assign target_position    = int_target_position;
assign angle              = int_angle;
assign tick               = int_tick;
assign log_base           = int_log_base;
assign ref_period_ticks   = int_ref_period_ticks;
assign dead_ang           = int_dead_ang;
assign dead_pos           = int_dead_pos;
assign control_flags      = int_control_flags;
// int_ap_start
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_start <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0] && WDATA[0])
            int_ap_start <= 1'b1;
        else if (ap_ready)
            int_ap_start <= int_auto_restart; // clear on handshake/auto restart
    end
end

// int_ap_done
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_done <= 1'b0;
    else if (ACLK_EN) begin
        if (ap_done)
            int_ap_done <= 1'b1;
        else if (ar_hs && raddr == ADDR_AP_CTRL)
            int_ap_done <= 1'b0; // clear on read
    end
end

// int_ap_idle
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_idle <= 1'b0;
    else if (ACLK_EN) begin
            int_ap_idle <= ap_idle;
    end
end

// int_ap_ready
always @(posedge ACLK) begin
    if (ARESET)
        int_ap_ready <= 1'b0;
    else if (ACLK_EN) begin
            int_ap_ready <= ap_ready;
    end
end

// int_auto_restart
always @(posedge ACLK) begin
    if (ARESET)
        int_auto_restart <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0])
            int_auto_restart <=  WDATA[7];
    end
end

// int_gie
always @(posedge ACLK) begin
    if (ARESET)
        int_gie <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_GIE && WSTRB[0])
            int_gie <= WDATA[0];
    end
end

// int_ier
always @(posedge ACLK) begin
    if (ARESET)
        int_ier <= 1'b0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_IER && WSTRB[0])
            int_ier <= WDATA[1:0];
    end
end

// int_isr[0]
always @(posedge ACLK) begin
    if (ARESET)
        int_isr[0] <= 1'b0;
    else if (ACLK_EN) begin
        if (int_ier[0] & ap_done)
            int_isr[0] <= 1'b1;
        else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
            int_isr[0] <= int_isr[0] ^ WDATA[0]; // toggle on write
    end
end

// int_isr[1]
always @(posedge ACLK) begin
    if (ARESET)
        int_isr[1] <= 1'b0;
    else if (ACLK_EN) begin
        if (int_ier[1] & ap_ready)
            int_isr[1] <= 1'b1;
        else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
            int_isr[1] <= int_isr[1] ^ WDATA[1]; // toggle on write
    end
end

// int_angleD[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_angleD[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ANGLED_DATA_0)
            int_angleD[31:0] <= (WDATA[31:0] & wmask) | (int_angleD[31:0] & ~wmask);
    end
end

// int_angle_cos[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_angle_cos[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ANGLE_COS_DATA_0)
            int_angle_cos[31:0] <= (WDATA[31:0] & wmask) | (int_angle_cos[31:0] & ~wmask);
    end
end

// int_angle_sin[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_angle_sin[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ANGLE_SIN_DATA_0)
            int_angle_sin[31:0] <= (WDATA[31:0] & wmask) | (int_angle_sin[31:0] & ~wmask);
    end
end

// int_position[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_position[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_POSITION_DATA_0)
            int_position[31:0] <= (WDATA[31:0] & wmask) | (int_position[31:0] & ~wmask);
    end
end

// int_positionD[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_positionD[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_POSITIOND_DATA_0)
            int_positionD[31:0] <= (WDATA[31:0] & wmask) | (int_positionD[31:0] & ~wmask);
    end
end

// int_target_equilibrium[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_target_equilibrium[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_TARGET_EQUILIBRIUM_DATA_0)
            int_target_equilibrium[31:0] <= (WDATA[31:0] & wmask) | (int_target_equilibrium[31:0] & ~wmask);
    end
end

// int_target_position[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_target_position[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_TARGET_POSITION_DATA_0)
            int_target_position[31:0] <= (WDATA[31:0] & wmask) | (int_target_position[31:0] & ~wmask);
    end
end

// int_angle[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_angle[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ANGLE_DATA_0)
            int_angle[31:0] <= (WDATA[31:0] & wmask) | (int_angle[31:0] & ~wmask);
    end
end

// int_tick[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_tick[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_TICK_DATA_0)
            int_tick[31:0] <= (WDATA[31:0] & wmask) | (int_tick[31:0] & ~wmask);
    end
end

// int_log_base[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_log_base[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_LOG_BASE_DATA_0)
            int_log_base[31:0] <= (WDATA[31:0] & wmask) | (int_log_base[31:0] & ~wmask);
    end
end

// int_ref_period_ticks[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_ref_period_ticks[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_REF_PERIOD_TICKS_DATA_0)
            int_ref_period_ticks[31:0] <= (WDATA[31:0] & wmask) | (int_ref_period_ticks[31:0] & ~wmask);
    end
end

// int_dead_ang[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_dead_ang[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DEAD_ANG_DATA_0)
            int_dead_ang[31:0] <= (WDATA[31:0] & wmask) | (int_dead_ang[31:0] & ~wmask);
    end
end

// int_dead_pos[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_dead_pos[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DEAD_POS_DATA_0)
            int_dead_pos[31:0] <= (WDATA[31:0] & wmask) | (int_dead_pos[31:0] & ~wmask);
    end
end

// int_control_flags[31:0]
always @(posedge ACLK) begin
    if (ARESET)
        int_control_flags[31:0] <= 0;
    else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CONTROL_FLAGS_DATA_0)
            int_control_flags[31:0] <= (WDATA[31:0] & wmask) | (int_control_flags[31:0] & ~wmask);
    end
end

// int_Q
always @(posedge ACLK) begin
    if (ARESET)
        int_Q <= 0;
    else if (ACLK_EN) begin
        if (Q_ap_vld)
            int_Q <= Q;
    end
end

// int_Q_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_Q_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (Q_ap_vld)
            int_Q_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_Q_CTRL)
            int_Q_ap_vld <= 1'b0; // clear on read
    end
end

// int_status
always @(posedge ACLK) begin
    if (ARESET)
        int_status <= 0;
    else if (ACLK_EN) begin
        if (status_ap_vld)
            int_status <= status;
    end
end

// int_status_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_status_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (status_ap_vld)
            int_status_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_STATUS_CTRL)
            int_status_ap_vld <= 1'b0; // clear on read
    end
end

// int_update_count
always @(posedge ACLK) begin
    if (ARESET)
        int_update_count <= 0;
    else if (ACLK_EN) begin
        if (update_count_ap_vld)
            int_update_count <= update_count;
    end
end

// int_update_count_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_update_count_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (update_count_ap_vld)
            int_update_count_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_UPDATE_COUNT_CTRL)
            int_update_count_ap_vld <= 1'b0; // clear on read
    end
end

// int_nn_wait_cycles
always @(posedge ACLK) begin
    if (ARESET)
        int_nn_wait_cycles <= 0;
    else if (ACLK_EN) begin
        if (nn_wait_cycles_ap_vld)
            int_nn_wait_cycles <= nn_wait_cycles;
    end
end

// int_nn_wait_cycles_ap_vld
always @(posedge ACLK) begin
    if (ARESET)
        int_nn_wait_cycles_ap_vld <= 1'b0;
    else if (ACLK_EN) begin
        if (nn_wait_cycles_ap_vld)
            int_nn_wait_cycles_ap_vld <= 1'b1;
        else if (ar_hs && raddr == ADDR_NN_WAIT_CYCLES_CTRL)
            int_nn_wait_cycles_ap_vld <= 1'b0; // clear on read
    end
end


//------------------------Memory logic-------------------

endmodule
