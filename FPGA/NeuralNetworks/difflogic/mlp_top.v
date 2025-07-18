// mlp_top.v  - AXI-Stream façade for the differentiable-logic MLP
// Vivado-synthesizable Verilog-2001
`timescale 1ns/1ps
module mlp_top
#(
    parameter integer AXI_DATA_WIDTH  = 32,
    parameter integer NET_INPUT_BITS  = 700,
    parameter integer NET_OUTPUT_BITS = 1000
)
(
    // ── Global clock & reset ─────────────────────────────────────────
    input  wire                       AXIS_ACLK,
    input  wire                       AXI_ARESETN,

    // ── Slave (host → FPGA) ──────────────────────────────────────────
    output wire                       S_AXIS_TREADY,
    input  wire [AXI_DATA_WIDTH-1:0]  S_AXIS_TDATA,
    input  wire                       S_AXIS_TLAST,
    input  wire                       S_AXIS_TVALID,

    // ── Master (FPGA → host) ─────────────────────────────────────────
    output reg                        M_AXIS_TVALID,
    output reg  [AXI_DATA_WIDTH-1:0]  M_AXIS_TDATA,
    output reg                        M_AXIS_TLAST,
    input  wire                       M_AXIS_TREADY
);

    //-----------------------------------------------------------------
    // Compile-time helpers
    //-----------------------------------------------------------------
    localparam integer IN_WORDS  = 28;  // **kept on purpose**
    localparam integer OUT_WORDS = (NET_OUTPUT_BITS + AXI_DATA_WIDTH - 1) / AXI_DATA_WIDTH;

    localparam integer RDW = (IN_WORDS  > 1) ? $clog2(IN_WORDS ) : 1;
    localparam integer WRW = (OUT_WORDS > 1) ? $clog2(OUT_WORDS) : 1;

    //-----------------------------------------------------------------
    // FSM - three states are enough
    //-----------------------------------------------------------------
    localparam [1:0] ST_IDLE      = 2'd0,
                     ST_READ_IN   = 2'd1,
                     ST_WRITE_OUT = 2'd2;

    reg  [1:0] state, next_state;

    //-----------------------------------------------------------------
    // Payload buffers & counters
    //-----------------------------------------------------------------
    reg [AXI_DATA_WIDTH * 28 - 1:0] in_buf_words;
    reg [NET_INPUT_BITS -1:0]       in_buf;

    // New pipeline registers (break long combinational path)
    reg [NET_INPUT_BITS  -1:0] net_in_reg;   // frozen input → u_net
    reg [NET_OUTPUT_BITS -1:0] net_out_reg;  // u_net result, 1-cycle later
    reg                         out_ready;   // asserted after net_out_reg valid

    reg [NET_OUTPUT_BITS -1:0]  out_buf;

    reg [RDW-1:0] rd_idx;
    reg [WRW-1:0] wr_idx;

    integer i;

    //-----------------------------------------------------------------
    // Unpack 28×32-bit words → flat 700-bit vector
    //-----------------------------------------------------------------
    always @* begin
        /* Explicit slices keep synthesis simple and reproducible. */
        in_buf[0 * 100+: 100] = in_buf_words[0 * 128 +: 100];
        in_buf[1 * 100+: 100] = in_buf_words[1 * 128 +: 100];
        in_buf[2 * 100+: 100] = in_buf_words[2 * 128 +: 100];
        in_buf[3 * 100+: 100] = in_buf_words[3 * 128 +: 100];
        in_buf[4 * 100+: 100] = in_buf_words[4 * 128 +: 100];
        in_buf[5 * 100+: 100] = in_buf_words[5 * 128 +: 100];
        in_buf[6 * 100+: 100] = in_buf_words[6 * 128 +: 100];
    end

    //-----------------------------------------------------------------
    // Differentiable-logic network (pure combinational)
    //-----------------------------------------------------------------
    wire [NET_OUTPUT_BITS-1:0] net_out_wire;

    net u_net (
        .in         (net_in_reg),   // ← registered input
        .out        (net_out_wire), // combinational result
        .categories (/* unused */)
    );

    //-----------------------------------------------------------------
    // Next-state logic
    //-----------------------------------------------------------------
    always @* begin
        next_state = state;  // hold by default
        case (state)
            ST_IDLE:
                if (S_AXIS_TVALID) next_state = ST_READ_IN;

            ST_READ_IN:  // last word accepted?
                if (S_AXIS_TVALID && S_AXIS_TREADY &&
                    rd_idx == IN_WORDS-1 && S_AXIS_TLAST)
                    next_state = ST_WRITE_OUT;

            ST_WRITE_OUT: // finished streaming?
                if (M_AXIS_TREADY && wr_idx == OUT_WORDS-1 && out_ready)
                    next_state = ST_IDLE;
        endcase
    end

    assign S_AXIS_TREADY = (state != ST_WRITE_OUT);  // back-pressure only while sending

    //-----------------------------------------------------------------
    // Sequential logic
    //-----------------------------------------------------------------
    always @(posedge AXIS_ACLK or negedge AXI_ARESETN) begin
        if (!AXI_ARESETN) begin
            //-----------------------------------------------------------------
            // Safe reset values
            //-----------------------------------------------------------------
            state         <= ST_IDLE;
            rd_idx        <= {RDW{1'b0}};
            wr_idx        <= {WRW{1'b0}};
            M_AXIS_TVALID <= 1'b0;
            M_AXIS_TLAST  <= 1'b0;
            M_AXIS_TDATA  <= {AXI_DATA_WIDTH{1'b0}};
            net_in_reg    <= {NET_INPUT_BITS{1'b0}};
            net_out_reg   <= {NET_OUTPUT_BITS{1'b0}};
            out_buf       <= {NET_OUTPUT_BITS{1'b0}};
            out_ready     <= 1'b0;
        end
        else begin
            //-----------------------------------------------------------------
            // State register
            //-----------------------------------------------------------------
            state <= next_state;

            //-----------------------------------------------------------------
            // READ side - gather input words
            //-----------------------------------------------------------------
            if (state == ST_READ_IN && S_AXIS_TVALID && S_AXIS_TREADY) begin
                in_buf_words[rd_idx*AXI_DATA_WIDTH +: AXI_DATA_WIDTH] <= S_AXIS_TDATA;
                rd_idx <= rd_idx + 1;
            end
            if (state == ST_IDLE)
                rd_idx <= {RDW{1'b0}};

            //-----------------------------------------------------------------
            // Start network evaluation (freeze input)
            //-----------------------------------------------------------------
            if (state == ST_READ_IN && next_state == ST_WRITE_OUT) begin
                net_in_reg <= in_buf;   // capture freshly completed vector
                out_ready  <= 1'b0;     // clear flag: result not yet ready
            end

            //-----------------------------------------------------------------
            // Pipeline stage: capture network output every cycle
            //-----------------------------------------------------------------
            net_out_reg <= net_out_wire;

            //-----------------------------------------------------------------
            // Mark output valid exactly one cycle after input freeze
            //-----------------------------------------------------------------
            if (state == ST_WRITE_OUT && !out_ready) begin
                /* First cycle inside ST_WRITE_OUT:
                   - net_out_reg now contains the settled result.
                   - move it into out_buf for easy slicing.             */
                out_buf   <= net_out_reg;
                out_ready <= 1'b1;
                wr_idx    <= {WRW{1'b0}};
            end
            else if (state == ST_IDLE)
                out_ready <= 1'b0;

            //-----------------------------------------------------------------
            // WRITE side - stream result when ready
            //-----------------------------------------------------------------
            if (state == ST_WRITE_OUT && out_ready) begin
                if (M_AXIS_TREADY) begin
                    M_AXIS_TDATA <= out_buf[wr_idx*AXI_DATA_WIDTH +: AXI_DATA_WIDTH];
                    wr_idx       <= wr_idx + 1;
                end
                M_AXIS_TVALID <= 1'b1;
                M_AXIS_TLAST  <= (wr_idx == OUT_WORDS-1);
            end
            else begin
                M_AXIS_TVALID <= 1'b0;
                M_AXIS_TLAST  <= 1'b0;
            end
        end
    end

endmodule
