-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
-- Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

entity secloc_shell_CTRL_s_axi is
generic (
    C_S_AXI_ADDR_WIDTH    : INTEGER := 8;
    C_S_AXI_DATA_WIDTH    : INTEGER := 32);
port (
    ACLK                  :in   STD_LOGIC;
    ARESET                :in   STD_LOGIC;
    ACLK_EN               :in   STD_LOGIC;
    AWADDR                :in   STD_LOGIC_VECTOR(C_S_AXI_ADDR_WIDTH-1 downto 0);
    AWVALID               :in   STD_LOGIC;
    AWREADY               :out  STD_LOGIC;
    WDATA                 :in   STD_LOGIC_VECTOR(C_S_AXI_DATA_WIDTH-1 downto 0);
    WSTRB                 :in   STD_LOGIC_VECTOR(C_S_AXI_DATA_WIDTH/8-1 downto 0);
    WVALID                :in   STD_LOGIC;
    WREADY                :out  STD_LOGIC;
    BRESP                 :out  STD_LOGIC_VECTOR(1 downto 0);
    BVALID                :out  STD_LOGIC;
    BREADY                :in   STD_LOGIC;
    ARADDR                :in   STD_LOGIC_VECTOR(C_S_AXI_ADDR_WIDTH-1 downto 0);
    ARVALID               :in   STD_LOGIC;
    ARREADY               :out  STD_LOGIC;
    RDATA                 :out  STD_LOGIC_VECTOR(C_S_AXI_DATA_WIDTH-1 downto 0);
    RRESP                 :out  STD_LOGIC_VECTOR(1 downto 0);
    RVALID                :out  STD_LOGIC;
    RREADY                :in   STD_LOGIC;
    interrupt             :out  STD_LOGIC;
    angleD                :out  STD_LOGIC_VECTOR(31 downto 0);
    angle_cos             :out  STD_LOGIC_VECTOR(31 downto 0);
    angle_sin             :out  STD_LOGIC_VECTOR(31 downto 0);
    position              :out  STD_LOGIC_VECTOR(31 downto 0);
    positionD             :out  STD_LOGIC_VECTOR(31 downto 0);
    target_equilibrium    :out  STD_LOGIC_VECTOR(31 downto 0);
    target_position       :out  STD_LOGIC_VECTOR(31 downto 0);
    angle                 :out  STD_LOGIC_VECTOR(31 downto 0);
    tick                  :out  STD_LOGIC_VECTOR(31 downto 0);
    log_base              :out  STD_LOGIC_VECTOR(31 downto 0);
    ref_period_ticks      :out  STD_LOGIC_VECTOR(31 downto 0);
    dead_ang              :out  STD_LOGIC_VECTOR(31 downto 0);
    dead_pos              :out  STD_LOGIC_VECTOR(31 downto 0);
    control_flags         :out  STD_LOGIC_VECTOR(31 downto 0);
    Q                     :in   STD_LOGIC_VECTOR(31 downto 0);
    Q_ap_vld              :in   STD_LOGIC;
    status                :in   STD_LOGIC_VECTOR(31 downto 0);
    status_ap_vld         :in   STD_LOGIC;
    update_count          :in   STD_LOGIC_VECTOR(31 downto 0);
    update_count_ap_vld   :in   STD_LOGIC;
    nn_wait_cycles        :in   STD_LOGIC_VECTOR(31 downto 0);
    nn_wait_cycles_ap_vld :in   STD_LOGIC;
    ap_start              :out  STD_LOGIC;
    ap_done               :in   STD_LOGIC;
    ap_ready              :in   STD_LOGIC;
    ap_idle               :in   STD_LOGIC
);
end entity secloc_shell_CTRL_s_axi;

-- ------------------------Address Info-------------------
-- 0x00 : Control signals
--        bit 0  - ap_start (Read/Write/COH)
--        bit 1  - ap_done (Read/COR)
--        bit 2  - ap_idle (Read)
--        bit 3  - ap_ready (Read)
--        bit 7  - auto_restart (Read/Write)
--        others - reserved
-- 0x04 : Global Interrupt Enable Register
--        bit 0  - Global Interrupt Enable (Read/Write)
--        others - reserved
-- 0x08 : IP Interrupt Enable Register (Read/Write)
--        bit 0  - enable ap_done interrupt (Read/Write)
--        bit 1  - enable ap_ready interrupt (Read/Write)
--        others - reserved
-- 0x0c : IP Interrupt Status Register (Read/TOW)
--        bit 0  - ap_done (COR/TOW)
--        bit 1  - ap_ready (COR/TOW)
--        others - reserved
-- 0x10 : Data signal of angleD
--        bit 31~0 - angleD[31:0] (Read/Write)
-- 0x14 : reserved
-- 0x18 : Data signal of angle_cos
--        bit 31~0 - angle_cos[31:0] (Read/Write)
-- 0x1c : reserved
-- 0x20 : Data signal of angle_sin
--        bit 31~0 - angle_sin[31:0] (Read/Write)
-- 0x24 : reserved
-- 0x28 : Data signal of position
--        bit 31~0 - position[31:0] (Read/Write)
-- 0x2c : reserved
-- 0x30 : Data signal of positionD
--        bit 31~0 - positionD[31:0] (Read/Write)
-- 0x34 : reserved
-- 0x38 : Data signal of target_equilibrium
--        bit 31~0 - target_equilibrium[31:0] (Read/Write)
-- 0x3c : reserved
-- 0x40 : Data signal of target_position
--        bit 31~0 - target_position[31:0] (Read/Write)
-- 0x44 : reserved
-- 0x48 : Data signal of angle
--        bit 31~0 - angle[31:0] (Read/Write)
-- 0x4c : reserved
-- 0x50 : Data signal of tick
--        bit 31~0 - tick[31:0] (Read/Write)
-- 0x54 : reserved
-- 0x58 : Data signal of log_base
--        bit 31~0 - log_base[31:0] (Read/Write)
-- 0x5c : reserved
-- 0x60 : Data signal of ref_period_ticks
--        bit 31~0 - ref_period_ticks[31:0] (Read/Write)
-- 0x64 : reserved
-- 0x68 : Data signal of dead_ang
--        bit 31~0 - dead_ang[31:0] (Read/Write)
-- 0x6c : reserved
-- 0x70 : Data signal of dead_pos
--        bit 31~0 - dead_pos[31:0] (Read/Write)
-- 0x74 : reserved
-- 0x78 : Data signal of control_flags
--        bit 31~0 - control_flags[31:0] (Read/Write)
-- 0x7c : reserved
-- 0x80 : Data signal of Q
--        bit 31~0 - Q[31:0] (Read)
-- 0x84 : Control signal of Q
--        bit 0  - Q_ap_vld (Read/COR)
--        others - reserved
-- 0x90 : Data signal of status
--        bit 31~0 - status[31:0] (Read)
-- 0x94 : Control signal of status
--        bit 0  - status_ap_vld (Read/COR)
--        others - reserved
-- 0xa0 : Data signal of update_count
--        bit 31~0 - update_count[31:0] (Read)
-- 0xa4 : Control signal of update_count
--        bit 0  - update_count_ap_vld (Read/COR)
--        others - reserved
-- 0xb0 : Data signal of nn_wait_cycles
--        bit 31~0 - nn_wait_cycles[31:0] (Read)
-- 0xb4 : Control signal of nn_wait_cycles
--        bit 0  - nn_wait_cycles_ap_vld (Read/COR)
--        others - reserved
-- (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

architecture behave of secloc_shell_CTRL_s_axi is
    type states is (wridle, wrdata, wrresp, wrreset, rdidle, rddata, rdreset);  -- read and write fsm states
    signal wstate  : states := wrreset;
    signal rstate  : states := rdreset;
    signal wnext, rnext: states;
    constant ADDR_AP_CTRL                   : INTEGER := 16#00#;
    constant ADDR_GIE                       : INTEGER := 16#04#;
    constant ADDR_IER                       : INTEGER := 16#08#;
    constant ADDR_ISR                       : INTEGER := 16#0c#;
    constant ADDR_ANGLED_DATA_0             : INTEGER := 16#10#;
    constant ADDR_ANGLED_CTRL               : INTEGER := 16#14#;
    constant ADDR_ANGLE_COS_DATA_0          : INTEGER := 16#18#;
    constant ADDR_ANGLE_COS_CTRL            : INTEGER := 16#1c#;
    constant ADDR_ANGLE_SIN_DATA_0          : INTEGER := 16#20#;
    constant ADDR_ANGLE_SIN_CTRL            : INTEGER := 16#24#;
    constant ADDR_POSITION_DATA_0           : INTEGER := 16#28#;
    constant ADDR_POSITION_CTRL             : INTEGER := 16#2c#;
    constant ADDR_POSITIOND_DATA_0          : INTEGER := 16#30#;
    constant ADDR_POSITIOND_CTRL            : INTEGER := 16#34#;
    constant ADDR_TARGET_EQUILIBRIUM_DATA_0 : INTEGER := 16#38#;
    constant ADDR_TARGET_EQUILIBRIUM_CTRL   : INTEGER := 16#3c#;
    constant ADDR_TARGET_POSITION_DATA_0    : INTEGER := 16#40#;
    constant ADDR_TARGET_POSITION_CTRL      : INTEGER := 16#44#;
    constant ADDR_ANGLE_DATA_0              : INTEGER := 16#48#;
    constant ADDR_ANGLE_CTRL                : INTEGER := 16#4c#;
    constant ADDR_TICK_DATA_0               : INTEGER := 16#50#;
    constant ADDR_TICK_CTRL                 : INTEGER := 16#54#;
    constant ADDR_LOG_BASE_DATA_0           : INTEGER := 16#58#;
    constant ADDR_LOG_BASE_CTRL             : INTEGER := 16#5c#;
    constant ADDR_REF_PERIOD_TICKS_DATA_0   : INTEGER := 16#60#;
    constant ADDR_REF_PERIOD_TICKS_CTRL     : INTEGER := 16#64#;
    constant ADDR_DEAD_ANG_DATA_0           : INTEGER := 16#68#;
    constant ADDR_DEAD_ANG_CTRL             : INTEGER := 16#6c#;
    constant ADDR_DEAD_POS_DATA_0           : INTEGER := 16#70#;
    constant ADDR_DEAD_POS_CTRL             : INTEGER := 16#74#;
    constant ADDR_CONTROL_FLAGS_DATA_0      : INTEGER := 16#78#;
    constant ADDR_CONTROL_FLAGS_CTRL        : INTEGER := 16#7c#;
    constant ADDR_Q_DATA_0                  : INTEGER := 16#80#;
    constant ADDR_Q_CTRL                    : INTEGER := 16#84#;
    constant ADDR_STATUS_DATA_0             : INTEGER := 16#90#;
    constant ADDR_STATUS_CTRL               : INTEGER := 16#94#;
    constant ADDR_UPDATE_COUNT_DATA_0       : INTEGER := 16#a0#;
    constant ADDR_UPDATE_COUNT_CTRL         : INTEGER := 16#a4#;
    constant ADDR_NN_WAIT_CYCLES_DATA_0     : INTEGER := 16#b0#;
    constant ADDR_NN_WAIT_CYCLES_CTRL       : INTEGER := 16#b4#;
    constant ADDR_BITS         : INTEGER := 8;

    signal waddr               : UNSIGNED(ADDR_BITS-1 downto 0);
    signal wmask               : UNSIGNED(C_S_AXI_DATA_WIDTH-1 downto 0);
    signal aw_hs               : STD_LOGIC;
    signal w_hs                : STD_LOGIC;
    signal rdata_data          : UNSIGNED(C_S_AXI_DATA_WIDTH-1 downto 0);
    signal ar_hs               : STD_LOGIC;
    signal raddr               : UNSIGNED(ADDR_BITS-1 downto 0);
    signal AWREADY_t           : STD_LOGIC;
    signal WREADY_t            : STD_LOGIC;
    signal ARREADY_t           : STD_LOGIC;
    signal RVALID_t            : STD_LOGIC;
    -- internal registers
    signal int_ap_idle         : STD_LOGIC;
    signal int_ap_ready        : STD_LOGIC;
    signal int_ap_done         : STD_LOGIC := '0';
    signal int_ap_start        : STD_LOGIC := '0';
    signal int_auto_restart    : STD_LOGIC := '0';
    signal int_gie             : STD_LOGIC := '0';
    signal int_ier             : UNSIGNED(1 downto 0) := (others => '0');
    signal int_isr             : UNSIGNED(1 downto 0) := (others => '0');
    signal int_angleD          : UNSIGNED(31 downto 0) := (others => '0');
    signal int_angle_cos       : UNSIGNED(31 downto 0) := (others => '0');
    signal int_angle_sin       : UNSIGNED(31 downto 0) := (others => '0');
    signal int_position        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_positionD       : UNSIGNED(31 downto 0) := (others => '0');
    signal int_target_equilibrium : UNSIGNED(31 downto 0) := (others => '0');
    signal int_target_position : UNSIGNED(31 downto 0) := (others => '0');
    signal int_angle           : UNSIGNED(31 downto 0) := (others => '0');
    signal int_tick            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_log_base        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_ref_period_ticks : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dead_ang        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dead_pos        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_control_flags   : UNSIGNED(31 downto 0) := (others => '0');
    signal int_Q               : UNSIGNED(31 downto 0) := (others => '0');
    signal int_Q_ap_vld        : STD_LOGIC;
    signal int_status          : UNSIGNED(31 downto 0) := (others => '0');
    signal int_status_ap_vld   : STD_LOGIC;
    signal int_update_count    : UNSIGNED(31 downto 0) := (others => '0');
    signal int_update_count_ap_vld : STD_LOGIC;
    signal int_nn_wait_cycles  : UNSIGNED(31 downto 0) := (others => '0');
    signal int_nn_wait_cycles_ap_vld : STD_LOGIC;


begin
-- ----------------------- Instantiation------------------


-- ----------------------- AXI WRITE ---------------------
    AWREADY_t <=  '1' when wstate = wridle else '0';
    AWREADY   <=  AWREADY_t;
    WREADY_t  <=  '1' when wstate = wrdata else '0';
    WREADY    <=  WREADY_t;
    BRESP     <=  "00";  -- OKAY
    BVALID    <=  '1' when wstate = wrresp else '0';
    wmask     <=  (31 downto 24 => WSTRB(3), 23 downto 16 => WSTRB(2), 15 downto 8 => WSTRB(1), 7 downto 0 => WSTRB(0));
    aw_hs     <=  AWVALID and AWREADY_t;
    w_hs      <=  WVALID and WREADY_t;

    -- write FSM
    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                wstate <= wrreset;
            elsif (ACLK_EN = '1') then
                wstate <= wnext;
            end if;
        end if;
    end process;

    process (wstate, AWVALID, WVALID, BREADY)
    begin
        case (wstate) is
        when wridle =>
            if (AWVALID = '1') then
                wnext <= wrdata;
            else
                wnext <= wridle;
            end if;
        when wrdata =>
            if (WVALID = '1') then
                wnext <= wrresp;
            else
                wnext <= wrdata;
            end if;
        when wrresp =>
            if (BREADY = '1') then
                wnext <= wridle;
            else
                wnext <= wrresp;
            end if;
        when others =>
            wnext <= wridle;
        end case;
    end process;

    waddr_proc : process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (aw_hs = '1') then
                    waddr <= UNSIGNED(AWADDR(ADDR_BITS-1 downto 0));
                end if;
            end if;
        end if;
    end process;

-- ----------------------- AXI READ ----------------------
    ARREADY_t <= '1' when (rstate = rdidle) else '0';
    ARREADY <= ARREADY_t;
    RDATA   <= STD_LOGIC_VECTOR(rdata_data);
    RRESP   <= "00";  -- OKAY
    RVALID_t  <= '1' when (rstate = rddata) else '0';
    RVALID    <= RVALID_t;
    ar_hs   <= ARVALID and ARREADY_t;
    raddr   <= UNSIGNED(ARADDR(ADDR_BITS-1 downto 0));

    -- read FSM
    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                rstate <= rdreset;
            elsif (ACLK_EN = '1') then
                rstate <= rnext;
            end if;
        end if;
    end process;

    process (rstate, ARVALID, RREADY, RVALID_t)
    begin
        case (rstate) is
        when rdidle =>
            if (ARVALID = '1') then
                rnext <= rddata;
            else
                rnext <= rdidle;
            end if;
        when rddata =>
            if (RREADY = '1' and RVALID_t = '1') then
                rnext <= rdidle;
            else
                rnext <= rddata;
            end if;
        when others =>
            rnext <= rdidle;
        end case;
    end process;

    rdata_proc : process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (ar_hs = '1') then
                    rdata_data <= (others => '0');
                    case (TO_INTEGER(raddr)) is
                    when ADDR_AP_CTRL =>
                        rdata_data(7) <= int_auto_restart;
                        rdata_data(3) <= int_ap_ready;
                        rdata_data(2) <= int_ap_idle;
                        rdata_data(1) <= int_ap_done;
                        rdata_data(0) <= int_ap_start;
                    when ADDR_GIE =>
                        rdata_data(0) <= int_gie;
                    when ADDR_IER =>
                        rdata_data(1 downto 0) <= int_ier;
                    when ADDR_ISR =>
                        rdata_data(1 downto 0) <= int_isr;
                    when ADDR_ANGLED_DATA_0 =>
                        rdata_data <= RESIZE(int_angleD(31 downto 0), 32);
                    when ADDR_ANGLE_COS_DATA_0 =>
                        rdata_data <= RESIZE(int_angle_cos(31 downto 0), 32);
                    when ADDR_ANGLE_SIN_DATA_0 =>
                        rdata_data <= RESIZE(int_angle_sin(31 downto 0), 32);
                    when ADDR_POSITION_DATA_0 =>
                        rdata_data <= RESIZE(int_position(31 downto 0), 32);
                    when ADDR_POSITIOND_DATA_0 =>
                        rdata_data <= RESIZE(int_positionD(31 downto 0), 32);
                    when ADDR_TARGET_EQUILIBRIUM_DATA_0 =>
                        rdata_data <= RESIZE(int_target_equilibrium(31 downto 0), 32);
                    when ADDR_TARGET_POSITION_DATA_0 =>
                        rdata_data <= RESIZE(int_target_position(31 downto 0), 32);
                    when ADDR_ANGLE_DATA_0 =>
                        rdata_data <= RESIZE(int_angle(31 downto 0), 32);
                    when ADDR_TICK_DATA_0 =>
                        rdata_data <= RESIZE(int_tick(31 downto 0), 32);
                    when ADDR_LOG_BASE_DATA_0 =>
                        rdata_data <= RESIZE(int_log_base(31 downto 0), 32);
                    when ADDR_REF_PERIOD_TICKS_DATA_0 =>
                        rdata_data <= RESIZE(int_ref_period_ticks(31 downto 0), 32);
                    when ADDR_DEAD_ANG_DATA_0 =>
                        rdata_data <= RESIZE(int_dead_ang(31 downto 0), 32);
                    when ADDR_DEAD_POS_DATA_0 =>
                        rdata_data <= RESIZE(int_dead_pos(31 downto 0), 32);
                    when ADDR_CONTROL_FLAGS_DATA_0 =>
                        rdata_data <= RESIZE(int_control_flags(31 downto 0), 32);
                    when ADDR_Q_DATA_0 =>
                        rdata_data <= RESIZE(int_Q(31 downto 0), 32);
                    when ADDR_Q_CTRL =>
                        rdata_data(0) <= int_Q_ap_vld;
                    when ADDR_STATUS_DATA_0 =>
                        rdata_data <= RESIZE(int_status(31 downto 0), 32);
                    when ADDR_STATUS_CTRL =>
                        rdata_data(0) <= int_status_ap_vld;
                    when ADDR_UPDATE_COUNT_DATA_0 =>
                        rdata_data <= RESIZE(int_update_count(31 downto 0), 32);
                    when ADDR_UPDATE_COUNT_CTRL =>
                        rdata_data(0) <= int_update_count_ap_vld;
                    when ADDR_NN_WAIT_CYCLES_DATA_0 =>
                        rdata_data <= RESIZE(int_nn_wait_cycles(31 downto 0), 32);
                    when ADDR_NN_WAIT_CYCLES_CTRL =>
                        rdata_data(0) <= int_nn_wait_cycles_ap_vld;
                    when others =>
                        NULL;
                    end case;
                end if;
            end if;
        end if;
    end process;

-- ----------------------- Register logic ----------------
    interrupt            <= int_gie and (int_isr(0) or int_isr(1));
    ap_start             <= int_ap_start;
    angleD               <= STD_LOGIC_VECTOR(int_angleD);
    angle_cos            <= STD_LOGIC_VECTOR(int_angle_cos);
    angle_sin            <= STD_LOGIC_VECTOR(int_angle_sin);
    position             <= STD_LOGIC_VECTOR(int_position);
    positionD            <= STD_LOGIC_VECTOR(int_positionD);
    target_equilibrium   <= STD_LOGIC_VECTOR(int_target_equilibrium);
    target_position      <= STD_LOGIC_VECTOR(int_target_position);
    angle                <= STD_LOGIC_VECTOR(int_angle);
    tick                 <= STD_LOGIC_VECTOR(int_tick);
    log_base             <= STD_LOGIC_VECTOR(int_log_base);
    ref_period_ticks     <= STD_LOGIC_VECTOR(int_ref_period_ticks);
    dead_ang             <= STD_LOGIC_VECTOR(int_dead_ang);
    dead_pos             <= STD_LOGIC_VECTOR(int_dead_pos);
    control_flags        <= STD_LOGIC_VECTOR(int_control_flags);

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_start <= '0';
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_AP_CTRL and WSTRB(0) = '1' and WDATA(0) = '1') then
                    int_ap_start <= '1';
                elsif (ap_ready = '1') then
                    int_ap_start <= int_auto_restart; -- clear on handshake/auto restart
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_done <= '0';
            elsif (ACLK_EN = '1') then
                if (ap_done = '1') then
                    int_ap_done <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_AP_CTRL) then
                    int_ap_done <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_idle <= '0';
            elsif (ACLK_EN = '1') then
                if (true) then
                    int_ap_idle <= ap_idle;
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_ready <= '0';
            elsif (ACLK_EN = '1') then
                if (true) then
                    int_ap_ready <= ap_ready;
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_auto_restart <= '0';
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_AP_CTRL and WSTRB(0) = '1') then
                    int_auto_restart <= WDATA(7);
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_gie <= '0';
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_GIE and WSTRB(0) = '1') then
                    int_gie <= WDATA(0);
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ier <= "00";
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_IER and WSTRB(0) = '1') then
                    int_ier <= UNSIGNED(WDATA(1 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_isr(0) <= '0';
            elsif (ACLK_EN = '1') then
                if (int_ier(0) = '1' and ap_done = '1') then
                    int_isr(0) <= '1';
                elsif (w_hs = '1' and waddr = ADDR_ISR and WSTRB(0) = '1') then
                    int_isr(0) <= int_isr(0) xor WDATA(0); -- toggle on write
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_isr(1) <= '0';
            elsif (ACLK_EN = '1') then
                if (int_ier(1) = '1' and ap_ready = '1') then
                    int_isr(1) <= '1';
                elsif (w_hs = '1' and waddr = ADDR_ISR and WSTRB(0) = '1') then
                    int_isr(1) <= int_isr(1) xor WDATA(1); -- toggle on write
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ANGLED_DATA_0) then
                    int_angleD(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_angleD(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ANGLE_COS_DATA_0) then
                    int_angle_cos(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_angle_cos(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ANGLE_SIN_DATA_0) then
                    int_angle_sin(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_angle_sin(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_POSITION_DATA_0) then
                    int_position(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_position(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_POSITIOND_DATA_0) then
                    int_positionD(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_positionD(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_TARGET_EQUILIBRIUM_DATA_0) then
                    int_target_equilibrium(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_target_equilibrium(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_TARGET_POSITION_DATA_0) then
                    int_target_position(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_target_position(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ANGLE_DATA_0) then
                    int_angle(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_angle(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_TICK_DATA_0) then
                    int_tick(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_tick(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_LOG_BASE_DATA_0) then
                    int_log_base(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_log_base(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_REF_PERIOD_TICKS_DATA_0) then
                    int_ref_period_ticks(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_ref_period_ticks(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DEAD_ANG_DATA_0) then
                    int_dead_ang(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dead_ang(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DEAD_POS_DATA_0) then
                    int_dead_pos(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dead_pos(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_CONTROL_FLAGS_DATA_0) then
                    int_control_flags(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_control_flags(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_Q <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (Q_ap_vld = '1') then
                    int_Q <= UNSIGNED(Q); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_Q_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (Q_ap_vld = '1') then
                    int_Q_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_Q_CTRL) then
                    int_Q_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_status <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (status_ap_vld = '1') then
                    int_status <= UNSIGNED(status); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_status_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (status_ap_vld = '1') then
                    int_status_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_STATUS_CTRL) then
                    int_status_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_update_count <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (update_count_ap_vld = '1') then
                    int_update_count <= UNSIGNED(update_count); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_update_count_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (update_count_ap_vld = '1') then
                    int_update_count_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_UPDATE_COUNT_CTRL) then
                    int_update_count_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_nn_wait_cycles <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (nn_wait_cycles_ap_vld = '1') then
                    int_nn_wait_cycles <= UNSIGNED(nn_wait_cycles); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_nn_wait_cycles_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (nn_wait_cycles_ap_vld = '1') then
                    int_nn_wait_cycles_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_NN_WAIT_CYCLES_CTRL) then
                    int_nn_wait_cycles_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;


-- ----------------------- Memory logic ------------------

end architecture behave;
