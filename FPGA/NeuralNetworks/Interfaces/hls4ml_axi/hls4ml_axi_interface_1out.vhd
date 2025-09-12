library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.mlp_top_pkg.all;  -- => your package with MLP_* constants

entity mlp_axi_interface is
    generic(
        C_S_AXI_DATA_WIDTH : integer := 32;  -- Must be 32
        C_S_AXI_ADDR_WIDTH : integer := 12   -- 4KB region
    );
    port(
        ------------------------------------------------------------------------------
        -- Global Clock/Reset
        ------------------------------------------------------------------------------
        S_AXI_ACLK    : in  std_logic;
        S_AXI_ARESETN : in  std_logic;

        ------------------------------------------------------------------------------
        -- AXI4 Slave Interface (Write Address)
        ------------------------------------------------------------------------------
        S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_AWLEN   : in  std_logic_vector(7 downto 0);
        S_AXI_AWSIZE  : in  std_logic_vector(2 downto 0);
        S_AXI_AWBURST : in  std_logic_vector(1 downto 0);
        S_AXI_AWVALID : in  std_logic;
        S_AXI_AWREADY : out std_logic;

        -- Write Data
        S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
        S_AXI_WLAST   : in  std_logic;
        S_AXI_WVALID  : in  std_logic;
        S_AXI_WREADY  : out std_logic;

        -- Write Response
        S_AXI_BRESP   : out std_logic_vector(1 downto 0);
        S_AXI_BVALID  : out std_logic;
        S_AXI_BREADY  : in  std_logic;

        ------------------------------------------------------------------------------
        -- AXI4 Slave Interface (Read Address)
        ------------------------------------------------------------------------------
        S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_ARLEN   : in  std_logic_vector(7 downto 0);
        S_AXI_ARSIZE  : in  std_logic_vector(2 downto 0);
        S_AXI_ARBURST : in  std_logic_vector(1 downto 0);
        S_AXI_ARVALID : in  std_logic;
        S_AXI_ARREADY : out std_logic;

        -- Read Data
        S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_RRESP   : out std_logic_vector(1 downto 0);
        S_AXI_RLAST   : out std_logic;
        S_AXI_RVALID  : out std_logic;
        S_AXI_RREADY  : in  std_logic;

        ------------------------------------------------------------------------------
        -- Optional Debug Outputs
        ------------------------------------------------------------------------------
        ap_start_out  : out std_logic;  -- for debugging
        ap_done_out   : out std_logic
    );
end entity;

architecture RTL of mlp_axi_interface is

    ------------------------------------------------------------------------------
    -- 1) Neural Network Parameters (from mlp_top_pkg)
    ------------------------------------------------------------------------------
    constant INPUT_NEURONS          : integer := MLP_INPUT_NEURONS;
    constant INPUT_BITS_PER_NEURON  : integer := MLP_INPUT_DATA_BITS;
    constant OUTPUT_NEURONS         : integer := MLP_OUTPUT_NEURONS;
    constant OUTPUT_BITS_PER_NEURON : integer := MLP_OUTPUT_DATA_BITS;

    constant TOTAL_INPUT_BITS  : integer := INPUT_NEURONS  * INPUT_BITS_PER_NEURON;
    constant TOTAL_OUTPUT_BITS : integer := OUTPUT_NEURONS * OUTPUT_BITS_PER_NEURON;

    ------------------------------------------------------------------------------
    -- 2) Example Address Map
    --    0x000 => control reg
    --    0x010 => input region
    --    0x200 => output region
    ------------------------------------------------------------------------------
    constant CTRL_BASE_ADDR   : integer := 16#000#;  -- 0x000
    constant INPUT_BASE_ADDR  : integer := 16#010#;  -- 0x010 => decimal 16
    constant OUTPUT_BASE_ADDR : integer := 16#200#;  -- 0x200 => decimal 512

    ------------------------------------------------------------------------------
    -- 3) Local Memories for Input & Output
    ------------------------------------------------------------------------------
    constant NUM_INPUT_WORDS  : integer := INPUT_NEURONS;   -- 1 word per input neuron
    constant NUM_OUTPUT_WORDS : integer := OUTPUT_NEURONS;  -- 1 word per output neuron

    type mem_in_t is array(0 to NUM_INPUT_WORDS-1) of std_logic_vector(31 downto 0);
    signal input_mem  : mem_in_t := (others => (others => '0'));

    type mem_out_t is array(0 to NUM_OUTPUT_WORDS-1) of std_logic_vector(31 downto 0);
    signal output_mem : mem_out_t := (others => (others => '0'));

    ------------------------------------------------------------------------------
    -- 4) Control Register
    ------------------------------------------------------------------------------
    signal reg_ctrl_sw : std_logic_vector(31 downto 0) := (others => '0'); -- CPU writes
    signal done_bit    : std_logic := '0';
    signal reg_ctrl    : std_logic_vector(31 downto 0) := (others => '0'); -- for readback

    ------------------------------------------------------------------------------
    -- 5) AXI Handshake Signals
    ------------------------------------------------------------------------------
    -- Write side
    signal awready_int : std_logic := '0';
    signal wready_int  : std_logic := '0';
    signal bvalid_int  : std_logic := '0';
    signal bresp_int   : std_logic_vector(1 downto 0) := "00";

    -- Read side
    signal arready_int : std_logic := '0';
    signal rvalid_int  : std_logic := '0';
    signal rresp_int   : std_logic_vector(1 downto 0) := "00";
    signal rlast_int   : std_logic := '0';
    signal rdata_int   : std_logic_vector(31 downto 0) := (others => '0');

    ------------------------------------------------------------------------------
    -- 6) Write Address/Burst Tracking
    ------------------------------------------------------------------------------
    signal awaddr_word_reg : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0) := (others => '0');
    signal awlen_reg       : unsigned(7 downto 0) := (others => '0');
    signal awsize_reg      : std_logic_vector(2 downto 0) := (others => '0');
    signal awburst_reg     : std_logic_vector(1 downto 0) := (others => '0');
    signal write_active    : std_logic := '0';

    signal waddr_word_reg  : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0) := (others => '0');
    signal burst_wcount    : unsigned(7 downto 0) := (others => '0');
    signal write_slverr    : std_logic := '0';

    ------------------------------------------------------------------------------
    -- 7) Read Address/Burst Tracking
    ------------------------------------------------------------------------------
    signal araddr_word_reg : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0) := (others => '0');
    signal arlen_reg       : unsigned(7 downto 0) := (others => '0');
    signal arsize_reg      : std_logic_vector(2 downto 0) := (others => '0');
    signal arburst_reg     : std_logic_vector(1 downto 0) := (others => '0');
    signal read_active     : std_logic := '0';

    signal raddr_word_reg  : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0) := (others => '0');
    signal burst_rcount    : unsigned(7 downto 0) := (others => '0');
    signal read_slverr     : std_logic := '0';
    signal load_raddr      : std_logic := '0';

    ------------------------------------------------------------------------------
    -- 8) Signals to/from the HLS MLP block
    ------------------------------------------------------------------------------
    signal ap_rst           : std_logic;
    signal ap_start         : std_logic := '0';
    signal ap_done          : std_logic;
    signal ap_idle          : std_logic;
    signal ap_ready         : std_logic;

    signal input_1_v        : std_logic_vector(TOTAL_INPUT_BITS-1 downto 0) := (others => '0');
    signal input_1_v_ap_vld : std_logic := '0';

    signal layer9_out_0_v        : std_logic_vector(TOTAL_OUTPUT_BITS-1 downto 0) := (others => '0');
    signal layer9_out_0_v_ap_vld : std_logic := '0';

    ------------------------------------------------------------------------------
    -- 9) Small FSM for controlling the HLS IP
    ------------------------------------------------------------------------------
    type fsm_state is (
       S_IDLE,
       S_START,
       S_WAIT_READY,
       S_WAIT_DONE,
       S_PACK_OUTPUTS,
       S_DONE
    );
    signal cs, ns : fsm_state := S_IDLE;

    ------------------------------------------------------------------------------
    -- AXI Address Increment Helper (treat WRAP as INCR for simplicity)
    ------------------------------------------------------------------------------
    function burst_word_next(
        curr_addr  : unsigned;
        burst_type : std_logic_vector(1 downto 0)
    ) return unsigned is
        variable result : unsigned(curr_addr'range) := curr_addr;
    begin
        case burst_type is
            when "01" =>  -- INCR
                result := curr_addr + 1;
            when "00" =>  -- FIXED
                null;  -- do not increment
            when "10" =>  -- WRAP
                -- For simplicity treat WRAP same as INCR
                result := curr_addr + 1;
            when others =>
                result := curr_addr + 1;
        end case;
        return result;
    end function;

begin
    ------------------------------------------------------------------------------
    -- Tie top-level AXI signals
    ------------------------------------------------------------------------------
    S_AXI_AWREADY <= awready_int;
    S_AXI_WREADY  <= wready_int;
    S_AXI_BRESP   <= bresp_int;
    S_AXI_BVALID  <= bvalid_int;

    S_AXI_ARREADY <= arready_int;
    S_AXI_RDATA   <= rdata_int;
    S_AXI_RRESP   <= rresp_int;
    S_AXI_RLAST   <= rlast_int;
    S_AXI_RVALID  <= rvalid_int;

    ------------------------------------------------------------------------------
    -- Debug signals
    ------------------------------------------------------------------------------
    ap_start_out <= ap_start;
    ap_done_out  <= ap_done;

    ------------------------------------------------------------------------------
    -- HLS Reset (active-high)
    ------------------------------------------------------------------------------
    ap_rst <= not S_AXI_ARESETN;

    ------------------------------------------------------------------------------
    -- Combine SW bits + done_bit => reg_ctrl (bit1 => done)
    ------------------------------------------------------------------------------
    process(reg_ctrl_sw, done_bit)
        variable tmp : std_logic_vector(31 downto 0);
    begin
        tmp := reg_ctrl_sw;
        tmp(1) := done_bit;  -- bit1 => done
        reg_ctrl <= tmp;
    end process;

    ------------------------------------------------------------------------------
    -- A) Single Process for Write (AW + W + B)
    ------------------------------------------------------------------------------
    axi_write_process : process(S_AXI_ACLK)
        variable current_addr : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0);
        variable addr_int     : integer;
        variable idx          : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN='0' then
                -- Reset all write-related signals
                awready_int     <= '0';
                wready_int      <= '0';
                bvalid_int      <= '0';
                bresp_int       <= "00";
                write_active    <= '0';

                awaddr_word_reg <= (others => '0');
                awlen_reg       <= (others => '0');
                awsize_reg      <= (others => '0');
                awburst_reg     <= (others => '0');

                waddr_word_reg  <= (others => '0');
                burst_wcount    <= (others => '0');
                write_slverr    <= '0';

            else
                ----------------------------------------------------------------------
                -- Default handshake logic
                ----------------------------------------------------------------------
                if write_active='0' then
                    -- Idle => can accept AW
                    awready_int <= '1';
                    wready_int  <= '0';
                else
                    -- Active => already have AW, now accept W
                    awready_int <= '0';
                    wready_int  <= '1';
                end if;

                -- Keep BVALID asserted until Master sees it
                if bvalid_int='1' then
                    if S_AXI_BREADY='1' then
                        bvalid_int <= '0';
                    end if;
                end if;

                ----------------------------------------------------------------------
                -- Latch AW on handshake (only when idle)
                ----------------------------------------------------------------------
                if (write_active='0' and S_AXI_AWVALID='1' and awready_int='1') then
                    awaddr_word_reg <= unsigned(S_AXI_AWADDR(C_S_AXI_ADDR_WIDTH-1 downto 2));
                    awlen_reg       <= unsigned(S_AXI_AWLEN);
                    awsize_reg      <= S_AXI_AWSIZE;
                    awburst_reg     <= S_AXI_AWBURST;

                    waddr_word_reg  <= unsigned(S_AXI_AWADDR(C_S_AXI_ADDR_WIDTH-1 downto 2));
                    burst_wcount    <= (others => '0');
                    write_slverr    <= '0';
                    write_active    <= '1';
                end if;

                ----------------------------------------------------------------------
                -- Handle WDATA when write_active
                ----------------------------------------------------------------------
                if (write_active='1' and S_AXI_WVALID='1' and wready_int='1') then
                    -- Compute current_addr => either AW base on first beat or next
                    if burst_wcount=0 then
                        current_addr := awaddr_word_reg;
                    else
                        current_addr := burst_word_next(waddr_word_reg, awburst_reg);
                    end if;

                    addr_int := to_integer(current_addr)*4;

                    -- Check AWSIZE => must be 2 => 4 bytes
                    if awsize_reg /= "010" then
                        write_slverr <= '1';
                    end if;

                    ------------------------------------------------------------------
                    -- Write to correct region
                    ------------------------------------------------------------------
                    if addr_int = CTRL_BASE_ADDR then
                        -- CPU writes control register
                        for i in 0 to 3 loop
                            if S_AXI_WSTRB(i)='1' then
                                reg_ctrl_sw(8*i+7 downto 8*i)
                                    <= S_AXI_WDATA(8*i+7 downto 8*i);
                            end if;
                        end loop;

                    elsif (addr_int >= INPUT_BASE_ADDR) and (addr_int < OUTPUT_BASE_ADDR) then
                        idx := (addr_int - INPUT_BASE_ADDR) / 4;
                        if (idx >= 0) and (idx < NUM_INPUT_WORDS) then
                            for i in 0 to 3 loop
                                if S_AXI_WSTRB(i)='1' then
                                    input_mem(idx)(8*i+7 downto 8*i)
                                        <= S_AXI_WDATA(8*i+7 downto 8*i);
                                end if;
                            end loop;
                        else
                            write_slverr <= '1';
                        end if;

                    else
                        -- Possibly writing to output or out of range => SLVERR
                        write_slverr <= '1';
                    end if;

                    -- Update address counters
                    waddr_word_reg <= current_addr;
                    burst_wcount   <= burst_wcount + 1;

                    ------------------------------------------------------------------
                    -- If WLAST => we must generate BVALID
                    ------------------------------------------------------------------
                    if S_AXI_WLAST='1' then
                        write_active <= '0';
                        if write_slverr='1' then
                            bresp_int <= "10"; -- SLVERR
                        else
                            bresp_int <= "00"; -- OKAY
                        end if;
                        bvalid_int <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------------------
    -- B) READ ADDRESS CHANNEL (AR)
    ------------------------------------------------------------------------------
    process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                arready_int     <= '0';
                read_active     <= '0';
                araddr_word_reg <= (others => '0');
                arlen_reg       <= (others => '0');
                arsize_reg      <= (others => '0');
                arburst_reg     <= (others => '0');
                load_raddr      <= '0';

            else
                load_raddr <= '0';

                if read_active='0' then
                    arready_int <= '1';  -- can accept AR
                else
                    arready_int <= '0';
                end if;

                if (S_AXI_ARVALID='1' and arready_int='1') then
                    araddr_word_reg <= unsigned(S_AXI_ARADDR(C_S_AXI_ADDR_WIDTH-1 downto 2));
                    arlen_reg       <= unsigned(S_AXI_ARLEN);
                    arsize_reg      <= S_AXI_ARSIZE;
                    arburst_reg     <= S_AXI_ARBURST;
                    read_active     <= '1';
                    load_raddr      <= '1';

                elsif (read_active='1'
                       and rvalid_int='1'
                       and S_AXI_RREADY='1'
                       and rlast_int='1') then
                    -- done with the burst
                    read_active <= '0';
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------------------
    -- C) READ DATA CHANNEL (R)
    ------------------------------------------------------------------------------
    process(S_AXI_ACLK)
        variable current_addr : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0);
        variable addr_int     : integer;
        variable idx_in       : integer;
        variable idx_out      : integer;
        variable tmp          : std_logic_vector(31 downto 0);
        variable rcount_next  : unsigned(7 downto 0);
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                rvalid_int     <= '0';
                rlast_int      <= '0';
                rresp_int      <= "00";
                rdata_int      <= (others => '0');
                raddr_word_reg <= (others => '0');
                burst_rcount   <= (others => '0');
                read_slverr    <= '0';

            else
                rlast_int <= '0';

                if read_active='1' then
                    if load_raddr='1' then
                        -- Just latched AR => start fresh
                        raddr_word_reg <= araddr_word_reg;
                        burst_rcount   <= (others => '0');
                        rvalid_int     <= '0';
                        read_slverr    <= '0';

                    elsif (rvalid_int='0') then
                        -- Drive first read beat
                        current_addr := raddr_word_reg;
                        addr_int     := to_integer(current_addr)*4;
                        tmp          := (others => '0');

                        if arsize_reg /= "010" then
                            read_slverr <= '1';
                        end if;

                        -- fetch data
                        if addr_int = CTRL_BASE_ADDR then
                            tmp := reg_ctrl;
                        elsif (addr_int >= INPUT_BASE_ADDR) and (addr_int < OUTPUT_BASE_ADDR) then
                            idx_in := (addr_int - INPUT_BASE_ADDR) / 4;
                            if (idx_in >= 0) and (idx_in < NUM_INPUT_WORDS) then
                                tmp := input_mem(idx_in);
                            else
                                read_slverr <= '1';
                            end if;
                        else
                            -- Possibly output region
                            idx_out := (addr_int - OUTPUT_BASE_ADDR) / 4;
                            if (idx_out >= 0) and (idx_out < NUM_OUTPUT_WORDS) then
                                tmp := output_mem(idx_out);
                            else
                                read_slverr <= '1';
                            end if;
                        end if;

                        rdata_int  <= tmp;
                        rvalid_int <= '1';

                        -- Single-beat if ARLEN=0
                        if arlen_reg = 0 then
                            rlast_int <= '1';
                        end if;

                    elsif (rvalid_int='1' and S_AXI_RREADY='1') then
                        -- Master accepted previous beat => next
                        rcount_next := burst_rcount + 1;
                        burst_rcount <= rcount_next;

                        if rlast_int='1' then
                            rvalid_int <= '0';
                        else
                            current_addr := burst_word_next(raddr_word_reg, arburst_reg);
                            raddr_word_reg <= current_addr;

                            addr_int := to_integer(current_addr)*4;
                            tmp := (others => '0');

                            if addr_int = CTRL_BASE_ADDR then
                                tmp := reg_ctrl;
                            elsif (addr_int >= INPUT_BASE_ADDR) and (addr_int < OUTPUT_BASE_ADDR) then
                                idx_in := (addr_int - INPUT_BASE_ADDR) / 4;
                                if (idx_in >= 0) and (idx_in < NUM_INPUT_WORDS) then
                                    tmp := input_mem(idx_in);
                                else
                                    read_slverr <= '1';
                                end if;
                            else
                                idx_out := (addr_int - OUTPUT_BASE_ADDR) / 4;
                                if (idx_out >= 0) and (idx_out < NUM_OUTPUT_WORDS) then
                                    tmp := output_mem(idx_out);
                                else
                                    read_slverr <= '1';
                                end if;
                            end if;

                            rdata_int <= tmp;

                            -- Check if this is last beat
                            if rcount_next = arlen_reg then
                                rlast_int <= '1';
                            end if;
                        end if;
                    end if;

                    -- RRESP
                    if read_slverr='1' then
                        rresp_int <= "10"; -- SLVERR
                    else
                        rresp_int <= "00"; -- OKAY
                    end if;
                else
                    rvalid_int <= '0';
                end if;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------------------
    -- D) FSM for the HLS MLP design
    ------------------------------------------------------------------------------
    process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if (S_AXI_ARESETN='0') then
                cs <= S_IDLE;
                ap_start <= '0';
                input_1_v_ap_vld <= '0';
                done_bit <= '0';
                output_mem <= (others => (others => '0'));
            else
                cs <= ns;

                case cs is
                    when S_IDLE =>
                        ap_start <= '0';
                        input_1_v_ap_vld <= '0';
                        done_bit <= '0';

                    when S_START =>
                        -- Pack inputs => input_1_v
                        for i in 0 to INPUT_NEURONS-1 loop
                            input_1_v(i*INPUT_BITS_PER_NEURON + (INPUT_BITS_PER_NEURON-1)
                                      downto i*INPUT_BITS_PER_NEURON)
                                <= input_mem(i)(INPUT_BITS_PER_NEURON-1 downto 0);
                        end loop;
                        ap_start <= '1';
                        input_1_v_ap_vld <= '1';

                    when S_WAIT_READY =>
                        ap_start <= '1';
                        input_1_v_ap_vld <= '1';

                    when S_WAIT_DONE =>
                        ap_start <= '0';
                        input_1_v_ap_vld <= '0';

                    when S_PACK_OUTPUTS =>
                        -- Copy outputs => output_mem
                        for j in 0 to OUTPUT_NEURONS-1 loop
                            output_mem(j)(OUTPUT_BITS_PER_NEURON-1 downto 0) <=
                                layer9_out_0_v((j+1)*OUTPUT_BITS_PER_NEURON-1
                                               downto j*OUTPUT_BITS_PER_NEURON);
                            output_mem(j)(31 downto OUTPUT_BITS_PER_NEURON) <= (others => '0');
                        end loop;

                    when S_DONE =>
                        done_bit <= '1';

                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process;

    ------------------------------------------------------------------------------
    -- Next-state combinational logic
    ------------------------------------------------------------------------------
    process(cs, reg_ctrl_sw, ap_ready, ap_done, layer9_out_0_v_ap_vld)
    begin
        ns <= cs;
        case cs is
            when S_IDLE =>
                if reg_ctrl_sw(0)='1' then
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
                if (ap_done='1') or (layer9_out_0_v_ap_vld='1') then
                    ns <= S_PACK_OUTPUTS;
                end if;

            when S_PACK_OUTPUTS =>
                ns <= S_DONE;

            when S_DONE =>
                if reg_ctrl_sw(0)='0' then
                    ns <= S_IDLE;
                end if;

            when others =>
                ns <= S_IDLE;
        end case;
    end process;

    ------------------------------------------------------------------------------
    -- E) Instantiate the HLS MLP design
    ------------------------------------------------------------------------------
    inst_myproject : entity work.myproject
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
