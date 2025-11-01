-- =====================================================================
-- AXI4 (memory-mapped) bridge for 'myproject' HLS MLP
-- Matches AXIS packing: each 32-bit input word contributes the low
-- MLP_INPUT_DATA_BITS bits, concatenated LSB-first across inputs.
-- Address map:
--   0x000 : CTRL    [bit0 start (W), bit1 done (R), bit2 busy (R)]
--   0x010 : INPUTS  (NUM_INPUT_WORDS x 4B)
--   0x200 : OUTPUTS (NUM_OUTPUT_WORDS x 4B)
-- =====================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.mlp_top_pkg.all;

entity mlp_axi_interface is
  generic(
    C_S_AXI_DATA_WIDTH : integer := 32;
    C_S_AXI_ADDR_WIDTH : integer := 12
  );
  port(
    -- Global
    S_AXI_ACLK    : in  std_logic;
    S_AXI_ARESETN : in  std_logic;

    -- AXI4 Write Address
    S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    S_AXI_AWLEN   : in  std_logic_vector(7 downto 0);
    S_AXI_AWSIZE  : in  std_logic_vector(2 downto 0);
    S_AXI_AWBURST : in  std_logic_vector(1 downto 0);
    S_AXI_AWVALID : in  std_logic;
    S_AXI_AWREADY : out std_logic;

    -- AXI4 Write Data
    S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    S_AXI_WLAST   : in  std_logic;
    S_AXI_WVALID  : in  std_logic;
    S_AXI_WREADY  : out std_logic;

    -- AXI4 Write Response
    S_AXI_BRESP   : out std_logic_vector(1 downto 0);
    S_AXI_BVALID  : out std_logic;
    S_AXI_BREADY  : in  std_logic;

    -- AXI4 Read Address
    S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    S_AXI_ARLEN   : in  std_logic_vector(7 downto 0);
    S_AXI_ARSIZE  : in  std_logic_vector(2 downto 0);
    S_AXI_ARBURST : in  std_logic_vector(1 downto 0);
    S_AXI_ARVALID : in  std_logic;
    S_AXI_ARREADY : out std_logic;

    -- AXI4 Read Data
    S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_AXI_RRESP   : out std_logic_vector(1 downto 0);
    S_AXI_RLAST   : out std_logic;
    S_AXI_RVALID  : out std_logic;
    S_AXI_RREADY  : in  std_logic;

    -- Optional debug
    ap_start_out  : out std_logic;
    ap_done_out   : out std_logic
  );
end entity;

architecture RTL of mlp_axi_interface is
  -- NN parameters from package
  constant INPUT_NEURONS          : integer := MLP_INPUT_NEURONS;
  constant INPUT_BITS_PER_NEURON  : integer := MLP_INPUT_DATA_BITS;
  constant OUTPUT_NEURONS         : integer := MLP_OUTPUT_NEURONS;
  constant OUTPUT_BITS_PER_NEURON : integer := MLP_OUTPUT_DATA_BITS;

  constant TOTAL_INPUT_BITS  : integer := INPUT_NEURONS  * INPUT_BITS_PER_NEURON;
  constant TOTAL_OUTPUT_BITS : integer := OUTPUT_NEURONS * OUTPUT_BITS_PER_NEURON;

  -- Address map
  constant CTRL_BASE_ADDR   : integer := 16#000#;  -- 0x000
  constant INPUT_BASE_ADDR  : integer := 16#010#;  -- 0x010
  constant OUTPUT_BASE_ADDR : integer := 16#200#;  -- 0x200

  -- AXI response encodings
  constant RESP_OKAY   : std_logic_vector(1 downto 0) := "00";
  constant RESP_SLVERR : std_logic_vector(1 downto 0) := "10";

  -- Local memories (32-bit words)
  constant NUM_INPUT_WORDS  : integer := INPUT_NEURONS;
  constant NUM_OUTPUT_WORDS : integer := OUTPUT_NEURONS;

  type mem_in_t  is array(0 to NUM_INPUT_WORDS -1) of std_logic_vector(31 downto 0);
  type mem_out_t is array(0 to NUM_OUTPUT_WORDS-1) of std_logic_vector(31 downto 0);

  signal input_mem  : mem_in_t  := (others => (others => '0'));
  signal output_mem : mem_out_t := (others => (others => '0'));

  -- CTRL bits (SW -> bit0 start; HW -> bit1 done; HW -> bit2 busy)
  signal reg_ctrl_sw : std_logic_vector(31 downto 0) := (others => '0');
  signal reg_ctrl    : std_logic_vector(31 downto 0) := (others => '0');
  signal done_bit    : std_logic := '0';  -- sticky DONE (set by HW, cleared on new start)
  signal busy_bit    : std_logic := '0';

  -- AXI plumbing
  signal awready_int, wready_int, bvalid_int : std_logic := '0';
  signal bresp_int  : std_logic_vector(1 downto 0) := RESP_OKAY;
  signal arready_int, rvalid_int, rlast_int  : std_logic := '0';
  signal rresp_int  : std_logic_vector(1 downto 0) := RESP_OKAY;
  signal rdata_int  : std_logic_vector(31 downto 0) := (others => '0');

  signal write_active, read_active : std_logic := '0';
  signal write_slverr, read_slverr : std_logic := '0';

  signal awaddr_word_reg, waddr_word_reg : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0) := (others => '0');
  signal awlen_reg    : unsigned(7 downto 0) := (others => '0');
  signal awsize_reg   : std_logic_vector(2 downto 0) := (others => '0');
  signal awburst_reg  : std_logic_vector(1 downto 0) := (others => '0');
  signal burst_wcount : unsigned(7 downto 0) := (others => '0');

  signal araddr_word_reg, raddr_word_reg : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0) := (others => '0');
  signal arlen_reg    : unsigned(7 downto 0) := (others => '0');
  signal arsize_reg   : std_logic_vector(2 downto 0) := (others => '0');
  signal arburst_reg  : std_logic_vector(1 downto 0) := (others => '0');
  signal burst_rcount : unsigned(7 downto 0) := (others => '0');
  signal load_raddr   : std_logic := '0';

  -- Track low address bits for alignment checking
  signal awaddr_lsb : std_logic_vector(1 downto 0) := (others => '0');
  signal araddr_lsb : std_logic_vector(1 downto 0) := (others => '0');

  -- HLS ports
  signal ap_rst           : std_logic;
  signal ap_start         : std_logic := '0';
  signal ap_done          : std_logic;
  signal ap_idle          : std_logic;
  signal ap_ready         : std_logic;
  signal input_1_v        : std_logic_vector(TOTAL_INPUT_BITS -1 downto 0) := (others => '0');
  signal input_1_v_ap_vld : std_logic := '0';
  signal layer9_out_0_v        : std_logic_vector(TOTAL_OUTPUT_BITS-1 downto 0);
  signal layer9_out_0_v_ap_vld : std_logic;

  -- Simple control FSM
  type fsm_state is (S_IDLE, S_START, S_WAIT_READY, S_WAIT_DONE, S_PACK_OUTPUTS, S_DONE);
  signal cs, ns : fsm_state := S_IDLE;

  -- Start edge detection for clean pulse to FSM
  signal reg_ctrl_sw_d0 : std_logic := '0';
  signal start_pulse    : std_logic := '0';

  function burst_word_next(
    curr_addr  : unsigned;
    burst_type : std_logic_vector(1 downto 0))
    return unsigned is
    variable r : unsigned(curr_addr'range) := curr_addr;
  begin
    -- Only FIXED and INCR are functionally supported; WRAP is pre-flagged as SLVERR.
    case burst_type is
      when "01" => r := curr_addr + 1;  -- INCR
      when "00" => null;                -- FIXED
      when others => r := curr_addr + 1;
    end case;
    return r;
  end function;
begin
  -- Tie-offs
  S_AXI_AWREADY <= awready_int;
  S_AXI_WREADY  <= wready_int;
  S_AXI_BVALID  <= bvalid_int;
  S_AXI_BRESP   <= bresp_int;

  S_AXI_ARREADY <= arready_int;
  S_AXI_RVALID  <= rvalid_int;
  S_AXI_RLAST   <= rlast_int;
  S_AXI_RRESP   <= rresp_int;
  S_AXI_RDATA   <= rdata_int;

  ap_rst       <= not S_AXI_ARESETN;
  ap_start_out <= ap_start;
  ap_done_out  <= ap_done;

  -- CTRL readback (bit1=done, bit2=busy)
  process(reg_ctrl_sw, done_bit, busy_bit)
    variable t : std_logic_vector(31 downto 0);
  begin
    t := reg_ctrl_sw;
    t(1) := done_bit;
    t(2) := busy_bit;
    reg_ctrl <= t;
  end process;

  -- ===========================
  -- AXI Write Channel (AW/W/B)
  -- ===========================
  process(S_AXI_ACLK)
    variable curr_addr         : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0);
    variable addr_b            : integer;
    variable idx               : integer;
    variable beat_slverr       : std_logic;
    variable write_slverr_next : std_logic;
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN='0' then
        awready_int     <= '0';
        wready_int      <= '0';
        bvalid_int      <= '0';
        bresp_int       <= RESP_OKAY;
        write_active    <= '0';
        write_slverr    <= '0';
        awaddr_word_reg <= (others => '0');
        awlen_reg       <= (others => '0');
        awsize_reg      <= (others => '0');
        awburst_reg     <= (others => '0');
        waddr_word_reg  <= (others => '0');
        burst_wcount    <= (others => '0');
        awaddr_lsb      <= (others => '0');
      else
        -- default ready behavior:
        --  - accept new AW only when no write in progress AND no BRESP pending
        --  - then accept W beats for that transaction
        if (write_active='0') and (bvalid_int='0') then
          awready_int <= '1';
          wready_int  <= '0';
        elsif write_active='1' then
          awready_int <= '0';
          wready_int  <= '1';
        else
          awready_int <= '0';
          wready_int  <= '0';
        end if;

        if (bvalid_int='1') and (S_AXI_BREADY='1') then
          bvalid_int <= '0';
        end if;

        -- latch AW
        if (write_active='0' and bvalid_int='0' and S_AXI_AWVALID='1' and awready_int='1') then
          awaddr_word_reg <= unsigned(S_AXI_AWADDR(C_S_AXI_ADDR_WIDTH-1 downto 2));
          awlen_reg       <= unsigned(S_AXI_AWLEN);
          awsize_reg      <= S_AXI_AWSIZE;
          awburst_reg     <= S_AXI_AWBURST;
          waddr_word_reg  <= unsigned(S_AXI_AWADDR(C_S_AXI_ADDR_WIDTH-1 downto 2));
          awaddr_lsb      <= S_AXI_AWADDR(1 downto 0);
          burst_wcount    <= (others => '0');
          write_slverr    <= '0';
          bresp_int       <= RESP_OKAY;
          write_active    <= '1';
        end if;

        -- accept W
        if (write_active='1' and S_AXI_WVALID='1' and wready_int='1') then
          if burst_wcount=0 then
            curr_addr := awaddr_word_reg;
          else
            curr_addr := burst_word_next(waddr_word_reg, awburst_reg);
          end if;

          addr_b            := to_integer(curr_addr)*4;
          beat_slverr       := '0';
          write_slverr_next := write_slverr;

          -- enforce 32-bit access size
          if awsize_reg/="010" then
            beat_slverr := '1';
          end if;

          -- first beat: alignment and supported burst types
          if burst_wcount=0 then
            if awaddr_lsb /= "00" then
              beat_slverr := '1';
            end if;
            if (awburst_reg /= "00") and (awburst_reg /= "01") then
              beat_slverr := '1';
            end if;
          end if;

          if addr_b = CTRL_BASE_ADDR then
            for i in 0 to 3 loop
              if S_AXI_WSTRB(i)='1' then
                reg_ctrl_sw(8*i+7 downto 8*i) <= S_AXI_WDATA(8*i+7 downto 8*i);
              end if;
            end loop;

          elsif (addr_b >= INPUT_BASE_ADDR) and (addr_b < OUTPUT_BASE_ADDR) then
            idx := (addr_b - INPUT_BASE_ADDR) / 4;
            if (idx>=0) and (idx<NUM_INPUT_WORDS) then
              for i in 0 to 3 loop
                if S_AXI_WSTRB(i)='1' then
                  input_mem(idx)(8*i+7 downto 8*i) <= S_AXI_WDATA(8*i+7 downto 8*i);
                end if;
              end loop;
            else
              beat_slverr := '1';
            end if;
          else
            -- writes to output region or unmapped => SLVERR
            beat_slverr := '1';
          end if;

          if beat_slverr='1' then
            write_slverr_next := '1';
          end if;

          -- aggregate per-beat error indication
          write_slverr <= write_slverr_next;

          waddr_word_reg <= curr_addr;
          burst_wcount   <= burst_wcount + 1;

          if S_AXI_WLAST='1' then
            -- AWLEN is "number of beats minus one". If WLAST does not line up,
            -- the whole transaction is flagged SLVERR.
            if burst_wcount /= awlen_reg then
              write_slverr_next := '1';
            end if;

            write_active <= '0';

            if write_slverr_next='1' then
              bresp_int <= RESP_SLVERR;
            else
              bresp_int <= RESP_OKAY;
            end if;

            -- keep internal flag coherent with final BRESP
            write_slverr <= write_slverr_next;
            bvalid_int   <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;

  -- ===========================
  -- AXI Read Address (AR)
  -- ===========================
  process(S_AXI_ACLK)
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN='0' then
        arready_int     <= '0';
        read_active     <= '0';
        araddr_word_reg <= (others => '0');
        arlen_reg       <= (others => '0');
        arsize_reg      <= (others => '0');
        arburst_reg     <= (others => '0');
        burst_rcount    <= (others => '0');
        load_raddr      <= '0';
        araddr_lsb      <= (others => '0');
      else
        load_raddr <= '0';
        if read_active='0' then
          arready_int <= '1';
        else
          arready_int <= '0';
        end if;

        if (S_AXI_ARVALID='1' and arready_int='1') then
          araddr_word_reg <= unsigned(S_AXI_ARADDR(C_S_AXI_ADDR_WIDTH-1 downto 2));
          arlen_reg       <= unsigned(S_AXI_ARLEN);
          arsize_reg      <= S_AXI_ARSIZE;
          arburst_reg     <= S_AXI_ARBURST;
          araddr_lsb      <= S_AXI_ARADDR(1 downto 0);
          read_active     <= '1';
          load_raddr      <= '1';
        elsif (read_active='1' and rvalid_int='1' and S_AXI_RREADY='1' and rlast_int='1') then
          read_active <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ===========================
  -- AXI Read Data (R)
  -- ===========================
  process(S_AXI_ACLK)
    variable curr_addr        : unsigned((C_S_AXI_ADDR_WIDTH-1)-2 downto 0);
    variable addr_b           : integer;
    variable idx_in           : integer;
    variable idx_out          : integer;
    variable d                : std_logic_vector(31 downto 0);
    variable next_cnt         : unsigned(7 downto 0);
    variable beat_slverr      : std_logic;
    variable read_slverr_next : std_logic;
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN='0' then
        rvalid_int     <= '0';
        rlast_int      <= '0';
        rresp_int      <= RESP_OKAY;
        rdata_int      <= (others => '0');
        raddr_word_reg <= (others => '0');
        burst_rcount   <= (others => '0');
        read_slverr    <= '0';
      else
        -- default: deassert RLAST; asserted explicitly on the cycle
        -- a last beat is driven.
        rlast_int <= '0';

        if read_active='1' then
          if load_raddr='1' then
            -- New burst: initialize address and error tracking.
            raddr_word_reg <= araddr_word_reg;
            burst_rcount   <= (others => '0');
            rvalid_int     <= '0';
            rresp_int      <= RESP_OKAY;
            read_slverr    <= '0';

          elsif (rvalid_int='0') then
            -- First data beat of a burst.
            curr_addr        := raddr_word_reg;
            addr_b           := to_integer(curr_addr)*4;
            d                := (others => '0');
            beat_slverr      := '0';
            read_slverr_next := read_slverr;

            if arsize_reg/="010" then
              beat_slverr := '1';
            end if;

            -- First beat: alignment and burst-type checks.
            if burst_rcount=0 then
              if araddr_lsb /= "00" then
                beat_slverr := '1';
              end if;
              if (arburst_reg /= "00") and (arburst_reg /= "01") then
                beat_slverr := '1';
              end if;
            end if;

            if addr_b = CTRL_BASE_ADDR then
              d := reg_ctrl;
            elsif (addr_b >= INPUT_BASE_ADDR) and (addr_b < OUTPUT_BASE_ADDR) then
              idx_in := (addr_b - INPUT_BASE_ADDR)/4;
              if (idx_in>=0) and (idx_in<NUM_INPUT_WORDS) then
                d := input_mem(idx_in);
              else
                beat_slverr := '1';
              end if;
            else
              idx_out := (addr_b - OUTPUT_BASE_ADDR)/4;
              if (idx_out>=0) and (idx_out<NUM_OUTPUT_WORDS) then
                d := output_mem(idx_out);
              else
                beat_slverr := '1';
              end if;
            end if;

            if beat_slverr='1' then
              read_slverr_next := '1';
            end if;
            read_slverr <= read_slverr_next;

            rdata_int  <= d;
            rvalid_int <= '1';
            if arlen_reg=0 then
              rlast_int <= '1';
            end if;

            if read_slverr_next='1' then
              rresp_int <= RESP_SLVERR;
            else
              rresp_int <= RESP_OKAY;
            end if;

          elsif (rvalid_int='1' and S_AXI_RREADY='1') then
            -- Handshake on current beat; prepare next (if any).
            next_cnt     := burst_rcount + 1;
            burst_rcount <= next_cnt;

            if rlast_int='1' then
              -- Last beat just accepted; RVALID will be cleared and
              -- AR side will drop read_active.
              rvalid_int <= '0';
            else
              -- Subsequent beats are generated in a single-cycle pipeline:
              curr_addr      := burst_word_next(raddr_word_reg, arburst_reg);
              raddr_word_reg <= curr_addr;
              addr_b         := to_integer(curr_addr)*4;
              d              := (others => '0');
              beat_slverr      := '0';
              read_slverr_next := read_slverr;

              if arsize_reg/="010" then
                beat_slverr := '1';
              end if;

              if addr_b = CTRL_BASE_ADDR then
                d := reg_ctrl;
              elsif (addr_b >= INPUT_BASE_ADDR) and (addr_b < OUTPUT_BASE_ADDR) then
                idx_in := (addr_b - INPUT_BASE_ADDR)/4;
                if (idx_in>=0) and (idx_in<NUM_INPUT_WORDS) then
                  d := input_mem(idx_in);
                else
                  beat_slverr := '1';
                end if;
              else
                idx_out := (addr_b - OUTPUT_BASE_ADDR)/4;
                if (idx_out>=0) and (idx_out<NUM_OUTPUT_WORDS) then
                  d := output_mem(idx_out);
                else
                  beat_slverr := '1';
                end if;
              end if;

              if beat_slverr='1' then
                read_slverr_next := '1';
              end if;
              read_slverr <= read_slverr_next;

              rdata_int <= d;
              if next_cnt = arlen_reg then
                rlast_int <= '1';
              end if;

              if read_slverr_next='1' then
                rresp_int <= RESP_SLVERR;
              else
                rresp_int <= RESP_OKAY;
              end if;
            end if;
          end if;
        else
          rvalid_int <= '0';
        end if;
      end if;
    end if;
  end process;

  -- ===========================
  -- Control FSM (Start/Done)
  -- ===========================
  process(S_AXI_ACLK)
    variable start_pulse_i : std_logic;
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN='0' then
        cs               <= S_IDLE;
        ap_start         <= '0';
        input_1_v_ap_vld <= '0';
        done_bit         <= '0';
        busy_bit         <= '0';
        output_mem       <= (others => (others => '0'));
        reg_ctrl_sw_d0   <= '0';
        start_pulse      <= '0';
      else
        -- CTRL[0] rising edge -> one-cycle start pulse
        start_pulse_i := '0';
        if (reg_ctrl_sw(0)='1' and reg_ctrl_sw_d0='0') then
          start_pulse_i := '1';
        end if;
        reg_ctrl_sw_d0 <= reg_ctrl_sw(0);
        start_pulse    <= start_pulse_i;

        cs <= ns;

        case cs is
          when S_IDLE =>
            ap_start         <= '0';
            input_1_v_ap_vld <= '0';
            busy_bit         <= '0';
            -- clear sticky DONE whenever SW has deasserted start
            if reg_ctrl_sw(0) = '0' then
              done_bit <= '0';
            end if;

          when S_START =>
            -- LSB-first packing, identical to AXIS bridge
            for i in 0 to INPUT_NEURONS-1 loop
              input_1_v(i*INPUT_BITS_PER_NEURON + INPUT_BITS_PER_NEURON-1 downto
                        i*INPUT_BITS_PER_NEURON)
                <= input_mem(i)(INPUT_BITS_PER_NEURON-1 downto 0);
            end loop;
            ap_start         <= '1';
            input_1_v_ap_vld <= '1';
            busy_bit         <= '1';

          when S_WAIT_READY =>
            ap_start         <= '1';
            input_1_v_ap_vld <= '1';
            busy_bit         <= '1';

          when S_WAIT_DONE =>
            ap_start         <= '0';
            input_1_v_ap_vld <= '0';
            busy_bit         <= '1';

          when S_PACK_OUTPUTS =>
            -- Capture HLS outputs atomically once valid/done is seen.
            for j in 0 to OUTPUT_NEURONS-1 loop
              output_mem(j)(OUTPUT_BITS_PER_NEURON-1 downto 0) <=
                layer9_out_0_v((j+1)*OUTPUT_BITS_PER_NEURON-1 downto j*OUTPUT_BITS_PER_NEURON);
              output_mem(j)(31 downto OUTPUT_BITS_PER_NEURON) <= (others => '0');
            end loop;
            busy_bit <= '1';

          when S_DONE =>
            done_bit <= '1';   -- latch DONE
            busy_bit <= '0';

          when others =>
            null;
        end case;
      end if;
    end if;
  end process;

  -- Next state
  process(cs, start_pulse, ap_ready, ap_done, layer9_out_0_v_ap_vld)
  begin
    ns <= cs;
    case cs is
      when S_IDLE =>
        if start_pulse='1' then
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
        ns <= S_IDLE;

      when others =>
        ns <= S_IDLE;
    end case;
  end process;

  -- HLS instance
  inst_myproject: entity work.myproject
    port map(
      ap_clk                => S_AXI_ACLK,
      ap_rst                => ap_rst,
      ap_start              => ap_start,
      ap_done               => ap_done,
      ap_idle               => ap_idle,
      ap_ready              => ap_ready,
      input_1_V             => input_1_v,
      input_1_V_ap_vld      => input_1_v_ap_vld,
      layer9_out_0_V        => layer9_out_0_v,
      layer9_out_0_V_ap_vld => layer9_out_0_v_ap_vld
    );
end RTL;
