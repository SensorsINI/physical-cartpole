-- =====================================================================
-- AXI4-Lite interface for controller via adapter
-- Auto-increment registers for back-to-back input/output transfers
-- Address map:
--   0x00 : CTRL  [bit0=start (W), bit1=done (R)]
--   0x04 : IN    (write inputs sequentially, auto-increments)
--   0x08 : OUT   (read outputs sequentially, auto-increments)
-- =====================================================================
--
-- =====================================================================
-- *** CONFIGURATION: Select HLS Core Adapter ***
-- =====================================================================
-- Change the entity name below to match your HLS core:
--   - controller_adapter_cp   : For 1-output CartPole cores (layer9_out_0_V)
--   - controller_adapter_f1t  : For 2-output F1T cores (layer8_out_0_V, layer8_out_1_V)
-- 
-- Edit the entity name on this line ONLY:
-- >>> CONTROLLER_ADAPTER_ENTITY: controller_adapter_cp <<<
--
-- Then update line ~420 in the architecture to match.
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.controller_io_parameters.all;

entity controller_axilite is
    generic(
        C_S_AXI_DATA_WIDTH : integer := 32;
        C_S_AXI_ADDR_WIDTH : integer := 5
    );
    port(
        -- AXI4-Lite Slave Interface
        S_AXI_ACLK    : in  std_logic;
        S_AXI_ARESETN : in  std_logic;

        -- Write Address
        S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_AWVALID : in  std_logic;
        S_AXI_AWREADY : out std_logic;

        -- Write Data
        S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
        S_AXI_WVALID  : in  std_logic;
        S_AXI_WREADY  : out std_logic;

        -- Write Response
        S_AXI_BRESP   : out std_logic_vector(1 downto 0);
        S_AXI_BVALID  : out std_logic;
        S_AXI_BREADY  : in  std_logic;

        -- Read Address
        S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_ARVALID : in  std_logic;
        S_AXI_ARREADY : out std_logic;

        -- Read Data
        S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_RRESP   : out std_logic_vector(1 downto 0);
        S_AXI_RVALID  : out std_logic;
        S_AXI_RREADY  : in  std_logic;

        -- Optional observability
        ap_start_out  : out std_logic;
        ap_done_out   : out std_logic
    );
end entity;

architecture RTL of controller_axilite is
    -----------------------------------------------------------------------------
    -- Controller I/O parameters from package
    -----------------------------------------------------------------------------
    constant NUM_INPUTS        : integer := CONTROLLER_INPUTS;
    constant BITS_PER_INPUT    : integer := BITS_PER_CONTROLLER_INPUT;
    constant NUM_OUTPUTS       : integer := CONTROLLER_OUTPUTS;
    constant BITS_PER_OUTPUT   : integer := BITS_PER_CONTROLLER_OUTPUT;
    constant TOTAL_INPUT_BITS  : integer := NUM_INPUTS  * BITS_PER_INPUT;
    constant TOTAL_OUTPUT_BITS : integer := NUM_OUTPUTS * BITS_PER_OUTPUT;

    -----------------------------------------------------------------------------
    -- AXI4-Lite internal
    -----------------------------------------------------------------------------
    signal awready   : std_logic := '0';
    signal wready    : std_logic := '0';
    signal bvalid    : std_logic := '0';
    signal bresp     : std_logic_vector(1 downto 0) := "00";
    signal arready   : std_logic := '0';
    signal rvalid    : std_logic := '0';
    signal rresp     : std_logic_vector(1 downto 0) := "00";
    signal rdata     : std_logic_vector(31 downto 0) := (others => '0');

    signal awaddr_q  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
    signal have_aw   : std_logic := '0';
    signal have_w    : std_logic := '0';

    -----------------------------------------------------------------------------
    -- Software-visible registers
    -----------------------------------------------------------------------------
    -- reg_ctrl_sw: bit0=start (SW). reg_ctrl presented to SW has bit1=done (HW).
    signal reg_ctrl_sw : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_ctrl    : std_logic_vector(31 downto 0) := (others => '0');
    signal done_bit    : std_logic := '0';

    -- Input/Output scratch (each entry is a full 32b word; only LSBs used)
    type word_array_t is array(natural range <>) of std_logic_vector(31 downto 0);
    signal input_items  : word_array_t(0 to NUM_INPUTS-1)  := (others => (others => '0'));
    signal output_items : word_array_t(0 to NUM_OUTPUTS-1) := (others => (others => '0'));

    -- Auto-increment indices
    signal input_count  : integer range 0 to NUM_INPUTS  := 0;
    signal output_count : integer range 0 to NUM_OUTPUTS := 0;

    -- Control pulses (single-clock enables synthesized from writes/reads)
    signal reset_counters   : std_logic := '0';
    signal inc_input_count  : std_logic := '0';
    signal inc_output_count : std_logic := '0';

    -----------------------------------------------------------------------------
    -- Core adapter ports
    -----------------------------------------------------------------------------
    signal ap_rst          : std_logic;
    signal ap_start        : std_logic := '0';
    signal ap_done         : std_logic;
    signal ap_ready        : std_logic;
    signal core_in_data    : std_logic_vector(TOTAL_INPUT_BITS-1 downto 0) := (others => '0');
    signal core_in_valid   : std_logic := '0';
    signal core_out_data   : std_logic_vector(TOTAL_OUTPUT_BITS-1 downto 0);
    signal core_out_valid  : std_logic;

    -----------------------------------------------------------------------------
    -- FSM
    -----------------------------------------------------------------------------
    type fsm_state is (S_IDLE, S_START, S_WAIT_READY, S_WAIT_DONE, S_PACK_OUT, S_DONE);
    signal cs, ns : fsm_state := S_IDLE;

    -- Track if read was for OUT register
    signal araddr_q            : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
    signal reading_output_reg  : std_logic := '0';
begin
    -----------------------------------------------------------------------------
    -- Top-level assigns
    -----------------------------------------------------------------------------
    S_AXI_AWREADY <= awready;
    S_AXI_WREADY  <= wready;
    S_AXI_BVALID  <= bvalid;
    S_AXI_BRESP   <= bresp;

    S_AXI_ARREADY <= arready;
    S_AXI_RVALID  <= rvalid;
    S_AXI_RDATA   <= rdata;
    S_AXI_RRESP   <= rresp;

    ap_start_out <= ap_start;
    ap_done_out  <= ap_done;

    ap_rst <= not S_AXI_ARESETN;

    -----------------------------------------------------------------------------
    -- AXI4-Lite WRITE channel
    -----------------------------------------------------------------------------
    write_p : process(S_AXI_ACLK)
        variable waddr : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                awready          <= '0';
                wready           <= '0';
                bvalid           <= '0';
                bresp            <= "00";
                have_aw          <= '0';
                have_w           <= '0';
                awaddr_q         <= (others => '0');
                reg_ctrl_sw      <= (others => '0');
                -- pulses
                reset_counters   <= '0';
                inc_input_count  <= '0';
            else
                -- default pulses low
                reset_counters  <= '0';
                inc_input_count <= '0';

                -- if we're returning a BRESP, wait for BREADY
                if bvalid = '1' then
                    if S_AXI_BREADY = '1' then
                        bvalid <= '0';
                    end if;
                    awready <= '0';
                    wready  <= '0';
                else
                    -- accept AW if we don't have one latched
                    if have_aw = '0' then awready <= '1'; else awready <= '0'; end if;
                    if (S_AXI_AWVALID = '1' and awready = '1') then
                        awaddr_q <= S_AXI_AWADDR;
                        have_aw  <= '1';
                    end if;

                    -- accept W if we don't have it latched
                    if have_w = '0' then wready <= '1'; else wready <= '0'; end if;
                    if (S_AXI_WVALID = '1' and wready = '1') then
                        have_w <= '1';
                    end if;

                    -- perform write when both AW and W are latched
                    if (have_aw = '1' and have_w = '1') then
                        waddr := to_integer(unsigned(awaddr_q(C_S_AXI_ADDR_WIDTH-1 downto 2)));
                        case waddr is
                            when 0 =>
                                -- CTRL
                                reg_ctrl_sw <= S_AXI_WDATA;
                                if S_AXI_WDATA(0) = '0' then
                                    -- start cleared => reset counters for a new transaction
                                    reset_counters <= '1';
                                end if;

                            when 1 =>
                                -- IN: back-to-back input words
                                if input_count < NUM_INPUTS then
                                    -- honor WSTRB minimally: only update enabled bytes
                                    for b in 0 to 3 loop
                                        if S_AXI_WSTRB(b) = '1' then
                                            input_items(input_count)(8*b+7 downto 8*b) <= S_AXI_WDATA(8*b+7 downto 8*b);
                                        end if;
                                    end loop;
                                    inc_input_count <= '1';
                                end if;

                            when others =>
                                null;
                        end case;

                        -- generate single-cycle OKAY response
                        bresp   <= "00";
                        bvalid  <= '1';
                        have_aw <= '0';
                        have_w  <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------
    -- AXI4-Lite READ channel
    -----------------------------------------------------------------------------
    read_p : process(S_AXI_ACLK)
        variable raddr : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                arready           <= '0';
                rvalid            <= '0';
                rresp             <= "00";
                rdata             <= (others => '0');
                araddr_q          <= (others => '0');
                reading_output_reg<= '0';
                inc_output_count  <= '0';
            else
                inc_output_count <= '0';

                if rvalid = '0' then
                    arready <= '1';
                else
                    arready <= '0';
                end if;

                -- Latch address and drive data
                if (S_AXI_ARVALID = '1' and arready = '1') then
                    araddr_q <= S_AXI_ARADDR;
                    rvalid   <= '1';
                    rresp    <= "00";
                    raddr := to_integer(unsigned(S_AXI_ARADDR(C_S_AXI_ADDR_WIDTH-1 downto 2)));

                    case raddr is
                        when 0 =>
                            -- CTRL readback: SW bits plus HW done in bit[1]
                            rdata <= reg_ctrl;
                            reading_output_reg <= '0';

                        when 2 =>
                            -- OUT: next output word (auto-increments after RREADY)
                            if output_count < NUM_OUTPUTS then
                                rdata <= output_items(output_count);
                            else
                                rdata <= (others => '0');
                            end if;
                            reading_output_reg <= '1';

                        when others =>
                            rdata <= (others => '0');
                            reading_output_reg <= '0';
                    end case;

                elsif (rvalid = '1' and S_AXI_RREADY = '1') then
                    -- complete read
                    rvalid <= '0';
                    if reading_output_reg = '1' then
                        inc_output_count <= '1';
                    end if;
                    reading_output_reg <= '0';
                end if;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------
    -- Indices management
    -----------------------------------------------------------------------------
    idx_p : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                input_count  <= 0;
                output_count <= 0;
            else
                if reset_counters = '1' then
                    input_count  <= 0;
                    output_count <= 0;
                else
                    if (inc_input_count = '1') and (input_count < NUM_INPUTS) then
                        input_count <= input_count + 1;
                    end if;
                    if (inc_output_count = '1') and (output_count < NUM_OUTPUTS) then
                        output_count <= output_count + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------
    -- Controller FSM
    -----------------------------------------------------------------------------
    fsm_seq : process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                cs            <= S_IDLE;
                ap_start      <= '0';
                core_in_valid <= '0';
                done_bit      <= '0';
                output_items  <= (others => (others => '0'));
            else
                cs <= ns;

                case cs is
                    when S_IDLE =>
                        ap_start      <= '0';
                        core_in_valid <= '0';
                        done_bit      <= '0';

                    when S_START =>
                        -- pack inputs (same order & bit slicing as other interfaces)
                        for i in 0 to NUM_INPUTS-1 loop
                            core_in_data(i*BITS_PER_INPUT + BITS_PER_INPUT-1 downto
                                         i*BITS_PER_INPUT)
                                <= input_items(i)(BITS_PER_INPUT-1 downto 0);
                        end loop;
                        ap_start      <= '1';
                        core_in_valid <= '1';

                    when S_WAIT_READY =>
                        ap_start      <= '1';
                        core_in_valid <= '1';

                    when S_WAIT_DONE =>
                        ap_start      <= '0';
                        core_in_valid <= '0';

                    when S_PACK_OUT =>
                        -- unpack outputs (LSBs hold the fixed-point result)
                        for i in 0 to NUM_OUTPUTS-1 loop
                            output_items(i)(BITS_PER_OUTPUT-1 downto 0) <=
                                core_out_data((i+1)*BITS_PER_OUTPUT - 1 downto
                                              i*BITS_PER_OUTPUT);
                            output_items(i)(31 downto BITS_PER_OUTPUT) <= (others => '0');
                        end loop;

                    when S_DONE =>
                        done_bit <= '1';

                    when others => null;
                end case;
            end if;
        end if;
    end process;

    fsm_comb : process(cs, reg_ctrl_sw, ap_ready, ap_done, core_out_valid, input_count)
    begin
        ns <= cs;
        case cs is
            when S_IDLE =>
                if (reg_ctrl_sw(0) = '1') and (input_count = NUM_INPUTS) then
                    ns <= S_START;
                end if;

            when S_START =>
                if ap_ready = '1' then
                    ns <= S_WAIT_DONE;
                else
                    ns <= S_WAIT_READY;
                end if;

            when S_WAIT_READY =>
                if ap_ready = '1' then
                    ns <= S_WAIT_DONE;
                end if;

            when S_WAIT_DONE =>
                if (ap_done = '1') or (core_out_valid = '1') then
                    ns <= S_PACK_OUT;
                end if;

            when S_PACK_OUT =>
                ns <= S_DONE;

            when S_DONE =>
                -- host must clear start=0 to arm next cycle
                if reg_ctrl_sw(0) = '0' then
                    ns <= S_IDLE;
                end if;

            when others =>
                ns <= S_IDLE;
        end case;
    end process;

    -- Present SW ctrl bits + HW done flag
    process(reg_ctrl_sw, done_bit)
    begin
        reg_ctrl       <= reg_ctrl_sw;
        reg_ctrl(1)    <= done_bit;  -- bit1 = done (RO)
    end process;

    -----------------------------------------------------------------------------
    -- Controller adapter instance
    -----------------------------------------------------------------------------
    -- *** TO CHANGE ADAPTER: Edit the entity name on the line below ***
    -- Current: controller_adapter_cp (1-output CartPole core)
    -- Options: controller_adapter_cp | controller_adapter_f1t
    -- See configuration section at top of file for details.
    -----------------------------------------------------------------------------
    inst_adapter: entity work.controller_adapter_cp
        port map(
            core_clk       => S_AXI_ACLK,
            core_rst       => ap_rst,
            core_start     => ap_start,
            core_ready     => ap_ready,
            core_done      => ap_done,
            core_in        => core_in_data,
            core_in_valid  => core_in_valid,
            core_out       => core_out_data,
            core_out_valid => core_out_valid
        );
end RTL;
