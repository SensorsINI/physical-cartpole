library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.mlp_top_pkg.all;

entity mlp_axi_lite_interface is
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

        -- Exposed signals for ILA or external monitoring
        ap_start_out  : out std_logic;
        ap_done_out   : out std_logic
    );
end entity;

architecture RTL of mlp_axi_lite_interface is

    -----------------------------------------------------------------------------
    -- Constants from your package
    -----------------------------------------------------------------------------
    constant INPUT_NEURONS         : integer := MLP_INPUT_NEURONS;
    constant INPUT_BITS_PER_NEURON : integer := MLP_INPUT_DATA_BITS;

    constant OUTPUT_NEURONS         : integer := MLP_OUTPUT_NEURONS;
    constant OUTPUT_BITS_PER_NEURON : integer := MLP_OUTPUT_DATA_BITS;

    constant TOTAL_INPUT_BITS  : integer := INPUT_NEURONS  * INPUT_BITS_PER_NEURON;
    constant TOTAL_OUTPUT_BITS : integer := OUTPUT_NEURONS * OUTPUT_BITS_PER_NEURON;

    -----------------------------------------------------------------------------
    -- AXI4-Lite internal signals
    -----------------------------------------------------------------------------
    signal slv_awready : std_logic := '0';
    signal slv_wready  : std_logic := '0';
    signal slv_bvalid  : std_logic := '0';
    signal slv_bresp   : std_logic_vector(1 downto 0) := (others => '0');

    signal slv_arready : std_logic := '0';
    signal slv_rvalid  : std_logic := '0';
    signal slv_rresp   : std_logic_vector(1 downto 0) := (others => '0');
    signal slv_rdata   : std_logic_vector(31 downto 0) := (others => '0');

    -----------------------------------------------------------------------------
    -- Register map
    -----------------------------------------------------------------------------
    signal awaddr_reg : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
    signal write_addr_latched : std_logic := '0';
    signal wdata_latched      : std_logic := '0';

    -- The CPU writes into reg_ctrl_sw; we combine done_bit => reg_ctrl(1).
    signal reg_ctrl_sw : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_ctrl    : std_logic_vector(31 downto 0) := (others => '0');
    signal done_bit    : std_logic := '0';

    -- For inputs
    type input_array_t is array(0 to INPUT_NEURONS-1) of std_logic_vector(31 downto 0);
    signal input_items : input_array_t := (others => (others => '0'));

    -- For outputs
    type output_array_t is array(0 to OUTPUT_NEURONS-1) of std_logic_vector(31 downto 0);
    signal output_items : output_array_t := (others => (others => '0'));

    -----------------------------------------------------------------------------
    -- We store the number of input writes and output reads in a single process
    -----------------------------------------------------------------------------
    signal input_count  : integer range 0 to INPUT_NEURONS := 0;
    signal output_count : integer range 0 to OUTPUT_NEURONS := 0;

    -- Control signals to avoid multiple drivers
    signal reset_counters   : std_logic := '0';
    signal inc_input_count  : std_logic := '0';
    signal inc_output_count : std_logic := '0';

    -----------------------------------------------------------------------------
    -- Signals to/from the HLS MLP block
    -----------------------------------------------------------------------------
    signal ap_rst    : std_logic;
    signal ap_start  : std_logic := '0';
    signal ap_done   : std_logic;
    signal ap_idle   : std_logic;
    signal ap_ready  : std_logic;

    signal input_1_v        : std_logic_vector(TOTAL_INPUT_BITS-1 downto 0) := (others => '0');
    signal input_1_v_ap_vld : std_logic := '0';

    signal layer9_out_0_v        : std_logic_vector(TOTAL_OUTPUT_BITS-1 downto 0);
    signal layer9_out_0_v_ap_vld : std_logic;

    -----------------------------------------------------------------------------
    -- FSM to control HLS block
    -----------------------------------------------------------------------------
    type fsm_state is (
       S_IDLE,
       S_START,
       S_WAIT_READY,
       S_WAIT_DONE,
       S_PACK_OUTPUTS,
       S_DONE
    );
    signal cs, ns : fsm_state := S_IDLE;

    -----------------------------------------------------------------------------
    -- Track AR address to see if we should increment output_count
    -----------------------------------------------------------------------------
    signal latched_ar         : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
    signal reading_output_reg : std_logic := '0';

begin

    -----------------------------------------------------------------------------
    -- Expose ap_start and ap_done to top-level ports
    -----------------------------------------------------------------------------
    ap_start_out <= ap_start;
    ap_done_out  <= ap_done;

    -----------------------------------------------------------------------------
    -- Create active-high reset for the HLS IP
    -----------------------------------------------------------------------------
    ap_rst <= not S_AXI_ARESETN;

    -----------------------------------------------------------------------------
    -- Tie top-level AXI signals
    -----------------------------------------------------------------------------
    S_AXI_AWREADY <= slv_awready;
    S_AXI_WREADY  <= slv_wready;
    S_AXI_BVALID  <= slv_bvalid;
    S_AXI_BRESP   <= slv_bresp;

    S_AXI_ARREADY <= slv_arready;
    S_AXI_RVALID  <= slv_rvalid;
    S_AXI_RDATA   <= slv_rdata;
    S_AXI_RRESP   <= slv_rresp;

    -----------------------------------------------------------------------------
    -- AXI4-Lite Write Channel (Fully-Compliant Handshake)
    -----------------------------------------------------------------------------
    process(S_AXI_ACLK)
        variable awaddr_int : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                slv_awready         <= '0';
                slv_wready          <= '0';
                slv_bvalid          <= '0';
                slv_bresp           <= "00";
                awaddr_reg          <= (others => '0');
                write_addr_latched  <= '0';
                wdata_latched       <= '0';
                reg_ctrl_sw         <= (others => '0');

                -- We only set signals here; actual counters are updated
                -- in the single synchronous process below.
                reset_counters      <= '0';
                inc_input_count     <= '0';

            else
                -- Default signals each cycle
                inc_input_count <= '0';
                reset_counters  <= '0';

                if (slv_bvalid = '1') then
                    -- Wait for BREADY from master
                    if (S_AXI_BREADY = '1') then
                        slv_bvalid <= '0';
                    end if;
                    -- Keep AWREADY/WREADY=0 while BVALID=1
                    slv_awready <= '0';
                    slv_wready  <= '0';

                else
                    -- We are not currently sending a response,
                    -- so we can accept AW or W if needed.

                    -- Latch AW if AWVALID & AWREADY
                    if (write_addr_latched = '0') then
                        slv_awready <= '1';
                    else
                        slv_awready <= '0';
                    end if;

                    if (S_AXI_AWVALID='1' and slv_awready='1') then
                        awaddr_reg         <= S_AXI_AWADDR;
                        write_addr_latched <= '1';
                    end if;

                    -- Latch W if WVALID & WREADY
                    if (wdata_latched = '0') then
                        slv_wready <= '1';
                    else
                        slv_wready <= '0';
                    end if;

                    if (S_AXI_WVALID='1' and slv_wready='1') then
                        wdata_latched <= '1';
                    end if;

                    -- If we have latched AW and W, perform the actual write
                    if (write_addr_latched='1' and wdata_latched='1') then
                        awaddr_int := to_integer(unsigned(awaddr_reg(C_S_AXI_ADDR_WIDTH-1 downto 2)));

                        case awaddr_int is
                            when 0 =>
                                -- CPU wrote to the control register (bit0= start)
                                reg_ctrl_sw(31 downto 0) <= S_AXI_WDATA;

                                -- If start=0 => we want to reset counters.
                                if S_AXI_WDATA(0) = '0' then
                                    reset_counters <= '1';
                                end if;

                            when 1 =>
                                -- Next input neuron => increment input_count
                                inc_input_count <= '1';
                                -- Latch the actual input data
                                if input_count < INPUT_NEURONS then
                                    input_items(input_count) <= S_AXI_WDATA;
                                end if;

                            when others =>
                                null;
                        end case;

                        -- Generate a single-cycle BVALID response
                        slv_bvalid         <= '1';
                        slv_bresp          <= "00";  -- OKAY
                        write_addr_latched <= '0';
                        wdata_latched      <= '0';
                    end if;
                end if;
            end if; -- reset
        end if; -- rising_edge
    end process;

    -----------------------------------------------------------------------------
    -- AXI4-Lite Read Channel
    -----------------------------------------------------------------------------
    process(S_AXI_ACLK)
        variable araddr_int : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                slv_arready <= '0';
                slv_rvalid  <= '0';
                slv_rdata   <= (others => '0');
                slv_rresp   <= "00";
                latched_ar  <= (others => '0');
                reading_output_reg <= '0';

                inc_output_count <= '0';

            else
                -- Defaults
                inc_output_count <= '0';

                -- If not currently sending data (RVALID=1), we are ready for AR
                if (slv_rvalid='0') then
                    slv_arready <= '1';
                else
                    slv_arready <= '0';
                end if;

                -- AR handshake
                if (S_AXI_ARVALID='1' and slv_arready='1') then
                    latched_ar  <= S_AXI_ARADDR;
                    slv_rvalid  <= '1';  -- data valid
                elsif (slv_rvalid='1' and S_AXI_RREADY='1') then
                    -- Master accepted data => end read transaction
                    slv_rvalid <= '0';

                    -- If we were reading the output reg, increment output_count
                    if reading_output_reg = '1' then
                        inc_output_count <= '1';
                    end if;
                    reading_output_reg <= '0';
                end if;

                -- Produce RDATA when we first assert RVALID
                if (S_AXI_ARVALID='1' and slv_arready='1') then
                    araddr_int := to_integer(unsigned(S_AXI_ARADDR(C_S_AXI_ADDR_WIDTH-1 downto 2)));
                    case araddr_int is
                        when 0 =>
                            -- Return reg_ctrl (SW bits plus done bit)
                            slv_rdata <= reg_ctrl;
                            reading_output_reg <= '0';

                        when 2 =>
                            -- Return next output neuron => increment after read
                            if output_count < OUTPUT_NEURONS then
                                slv_rdata <= output_items(output_count);
                            else
                                slv_rdata <= (others => '0');
                            end if;
                            reading_output_reg <= '1';

                        when others =>
                            slv_rdata <= (others => '0');
                            reading_output_reg <= '0';
                    end case;
                end if;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------
    -- Single synchronous process for input_count & output_count
    -----------------------------------------------------------------------------
    process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                input_count  <= 0;
                output_count <= 0;
            else
                if reset_counters = '1' then
                    input_count  <= 0;
                    output_count <= 0;
                else
                    if inc_input_count = '1' and input_count < INPUT_NEURONS then
                        input_count <= input_count + 1;
                    end if;
                    if inc_output_count = '1' and output_count < OUTPUT_NEURONS then
                        output_count <= output_count + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------
    -- Main FSM for controlling the HLS block
    -----------------------------------------------------------------------------
    process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                cs <= S_IDLE;
                ap_start <= '0';
                input_1_v_ap_vld <= '0';
                done_bit <= '0';
                output_items <= (others => (others => '0'));
            else
                cs <= ns;

                case cs is
                    when S_IDLE =>
                        -- De-assert everything
                        ap_start <= '0';
                        input_1_v_ap_vld <= '0';
                        done_bit <= '0';

                    when S_START =>
                        -- Move all input data into input_1_v in one cycle
                        for i in 0 to INPUT_NEURONS-1 loop
                            input_1_v(i*INPUT_BITS_PER_NEURON + (INPUT_BITS_PER_NEURON-1)
                                      downto i*INPUT_BITS_PER_NEURON)
                                <= input_items(i)(INPUT_BITS_PER_NEURON-1 downto 0);
                        end loop;
                        ap_start <= '1';
                        input_1_v_ap_vld <= '1';

                    when S_WAIT_READY =>
                        -- Keep ap_start=1 until ap_ready=1
                        ap_start <= '1';
                        input_1_v_ap_vld <= '1';

                    when S_WAIT_DONE =>
                        -- Drop ap_start=0, wait for ap_done
                        ap_start <= '0';
                        input_1_v_ap_vld <= '0';

                    when S_PACK_OUTPUTS =>
                        -- Copy layer9_out_0_v to output_items
                        for i in 0 to OUTPUT_NEURONS-1 loop
                            output_items(i)(OUTPUT_BITS_PER_NEURON-1 downto 0) <=
                                layer9_out_0_v((i+1)*OUTPUT_BITS_PER_NEURON - 1
                                               downto i*OUTPUT_BITS_PER_NEURON);
                            output_items(i)(31 downto OUTPUT_BITS_PER_NEURON) <= (others => '0');
                        end loop;

                    when S_DONE =>
                        done_bit <= '1';

                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------
    -- Next-state logic
    -----------------------------------------------------------------------------
    process(cs, reg_ctrl_sw, ap_ready, ap_done, layer9_out_0_v_ap_vld, input_count)
    begin
        ns <= cs;
        case cs is
            when S_IDLE =>
                -- Wait for CPU to set bit0=1 (start) AND for all inputs
                if (reg_ctrl_sw(0)='1') and (input_count = INPUT_NEURONS) then
                    ns <= S_START;
                end if;

            when S_START =>
                if ap_ready='1' then
                    ns <= S_WAIT_DONE;
                else
                    ns <= S_WAIT_READY;
                end if;

            when S_WAIT_READY =>
                if ap_ready='1' then
                    ns <= S_WAIT_DONE;
                end if;

            when S_WAIT_DONE =>
                -- Wait for ap_done or out_vld
                if ap_done='1' or layer9_out_0_v_ap_vld='1' then
                    ns <= S_PACK_OUTPUTS;
                end if;

            when S_PACK_OUTPUTS =>
                ns <= S_DONE;

            when S_DONE =>
                -- CPU must clear bit0=0 => then go back to IDLE
                if reg_ctrl_sw(0)='0' then
                    ns <= S_IDLE;
                end if;

            when others =>
                ns <= S_IDLE;
        end case;
    end process;

    -----------------------------------------------------------------------------
    -- Combine SW bits + done bit => reg_ctrl
    -----------------------------------------------------------------------------
    process(reg_ctrl_sw, done_bit)
    begin
        reg_ctrl <= reg_ctrl_sw;
        reg_ctrl(1) <= done_bit;  -- bit1 = done
    end process;

    -----------------------------------------------------------------------------
    -- Instantiate the HLS MLP block
    -----------------------------------------------------------------------------
    inst_mlp : entity work.myproject
        port map(
            ap_clk  => S_AXI_ACLK,
            ap_rst  => ap_rst,

            ap_start => ap_start,
            ap_done  => ap_done,
            ap_idle  => ap_idle,
            ap_ready => ap_ready,

            input_1_V        => input_1_v,
            input_1_V_ap_vld => input_1_v_ap_vld,

            layer9_out_0_V        => layer9_out_0_v,
            layer9_out_0_V_ap_vld => layer9_out_0_v_ap_vld
        );

end RTL;
