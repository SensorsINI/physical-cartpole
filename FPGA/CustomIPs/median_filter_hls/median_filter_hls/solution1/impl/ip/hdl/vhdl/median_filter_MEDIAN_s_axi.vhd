-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.1 (64-bit)
-- Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
-- ==============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

entity median_filter_MEDIAN_s_axi is
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
    filtered_i            :out  STD_LOGIC_VECTOR(31 downto 0);
    filtered_o            :in   STD_LOGIC_VECTOR(31 downto 0);
    filtered_o_ap_vld     :in   STD_LOGIC;
    raw_i                 :out  STD_LOGIC_VECTOR(31 downto 0);
    raw_o                 :in   STD_LOGIC_VECTOR(31 downto 0);
    raw_o_ap_vld          :in   STD_LOGIC;
    window_size           :out  STD_LOGIC_VECTOR(31 downto 0);
    trim_count            :out  STD_LOGIC_VECTOR(31 downto 0);
    filter_mode           :out  STD_LOGIC_VECTOR(31 downto 0);
    rail_low              :out  STD_LOGIC_VECTOR(31 downto 0);
    rail_high             :out  STD_LOGIC_VECTOR(31 downto 0);
    dz_status_i           :out  STD_LOGIC_VECTOR(31 downto 0);
    dz_status_o           :in   STD_LOGIC_VECTOR(31 downto 0);
    dz_status_o_ap_vld    :in   STD_LOGIC;
    dz_window_i           :out  STD_LOGIC_VECTOR(31 downto 0);
    dz_window_o           :in   STD_LOGIC_VECTOR(31 downto 0);
    dz_window_o_ap_vld    :in   STD_LOGIC;
    dz_age_i              :out  STD_LOGIC_VECTOR(31 downto 0);
    dz_age_o              :in   STD_LOGIC_VECTOR(31 downto 0);
    dz_age_o_ap_vld       :in   STD_LOGIC;
    dz_low_count          :in   STD_LOGIC_VECTOR(31 downto 0);
    dz_low_count_ap_vld   :in   STD_LOGIC;
    dz_high_count         :in   STD_LOGIC_VECTOR(31 downto 0);
    dz_high_count_ap_vld  :in   STD_LOGIC
);
end entity median_filter_MEDIAN_s_axi;

-- ------------------------Address Info-------------------
-- 0x00 : reserved
-- 0x04 : reserved
-- 0x08 : reserved
-- 0x0c : reserved
-- 0x10 : Data signal of filtered_i
--        bit 31~0 - filtered_i[31:0] (Read/Write)
-- 0x14 : reserved
-- 0x18 : Data signal of filtered_o
--        bit 31~0 - filtered_o[31:0] (Read)
-- 0x1c : Control signal of filtered_o
--        bit 0  - filtered_o_ap_vld (Read/COR)
--        others - reserved
-- 0x20 : Data signal of raw_i
--        bit 31~0 - raw_i[31:0] (Read/Write)
-- 0x24 : reserved
-- 0x28 : Data signal of raw_o
--        bit 31~0 - raw_o[31:0] (Read)
-- 0x2c : Control signal of raw_o
--        bit 0  - raw_o_ap_vld (Read/COR)
--        others - reserved
-- 0x30 : Data signal of window_size
--        bit 31~0 - window_size[31:0] (Read/Write)
-- 0x34 : reserved
-- 0x38 : Data signal of trim_count
--        bit 31~0 - trim_count[31:0] (Read/Write)
-- 0x3c : reserved
-- 0x40 : Data signal of filter_mode
--        bit 31~0 - filter_mode[31:0] (Read/Write)
-- 0x44 : reserved
-- 0x48 : Data signal of rail_low
--        bit 31~0 - rail_low[31:0] (Read/Write)
-- 0x4c : reserved
-- 0x50 : Data signal of rail_high
--        bit 31~0 - rail_high[31:0] (Read/Write)
-- 0x54 : reserved
-- 0x58 : Data signal of dz_status_i
--        bit 31~0 - dz_status_i[31:0] (Read/Write)
-- 0x5c : reserved
-- 0x60 : Data signal of dz_status_o
--        bit 31~0 - dz_status_o[31:0] (Read)
-- 0x64 : Control signal of dz_status_o
--        bit 0  - dz_status_o_ap_vld (Read/COR)
--        others - reserved
-- 0x68 : Data signal of dz_window_i
--        bit 31~0 - dz_window_i[31:0] (Read/Write)
-- 0x6c : reserved
-- 0x70 : Data signal of dz_window_o
--        bit 31~0 - dz_window_o[31:0] (Read)
-- 0x74 : Control signal of dz_window_o
--        bit 0  - dz_window_o_ap_vld (Read/COR)
--        others - reserved
-- 0x78 : Data signal of dz_age_i
--        bit 31~0 - dz_age_i[31:0] (Read/Write)
-- 0x7c : reserved
-- 0x80 : Data signal of dz_age_o
--        bit 31~0 - dz_age_o[31:0] (Read)
-- 0x84 : Control signal of dz_age_o
--        bit 0  - dz_age_o_ap_vld (Read/COR)
--        others - reserved
-- 0x88 : Data signal of dz_low_count
--        bit 31~0 - dz_low_count[31:0] (Read)
-- 0x8c : Control signal of dz_low_count
--        bit 0  - dz_low_count_ap_vld (Read/COR)
--        others - reserved
-- 0x98 : Data signal of dz_high_count
--        bit 31~0 - dz_high_count[31:0] (Read)
-- 0x9c : Control signal of dz_high_count
--        bit 0  - dz_high_count_ap_vld (Read/COR)
--        others - reserved
-- (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

architecture behave of median_filter_MEDIAN_s_axi is
    type states is (wridle, wrdata, wrresp, wrreset, rdidle, rddata, rdreset);  -- read and write fsm states
    signal wstate  : states := wrreset;
    signal rstate  : states := rdreset;
    signal wnext, rnext: states;
    constant ADDR_FILTERED_I_DATA_0    : INTEGER := 16#10#;
    constant ADDR_FILTERED_I_CTRL      : INTEGER := 16#14#;
    constant ADDR_FILTERED_O_DATA_0    : INTEGER := 16#18#;
    constant ADDR_FILTERED_O_CTRL      : INTEGER := 16#1c#;
    constant ADDR_RAW_I_DATA_0         : INTEGER := 16#20#;
    constant ADDR_RAW_I_CTRL           : INTEGER := 16#24#;
    constant ADDR_RAW_O_DATA_0         : INTEGER := 16#28#;
    constant ADDR_RAW_O_CTRL           : INTEGER := 16#2c#;
    constant ADDR_WINDOW_SIZE_DATA_0   : INTEGER := 16#30#;
    constant ADDR_WINDOW_SIZE_CTRL     : INTEGER := 16#34#;
    constant ADDR_TRIM_COUNT_DATA_0    : INTEGER := 16#38#;
    constant ADDR_TRIM_COUNT_CTRL      : INTEGER := 16#3c#;
    constant ADDR_FILTER_MODE_DATA_0   : INTEGER := 16#40#;
    constant ADDR_FILTER_MODE_CTRL     : INTEGER := 16#44#;
    constant ADDR_RAIL_LOW_DATA_0      : INTEGER := 16#48#;
    constant ADDR_RAIL_LOW_CTRL        : INTEGER := 16#4c#;
    constant ADDR_RAIL_HIGH_DATA_0     : INTEGER := 16#50#;
    constant ADDR_RAIL_HIGH_CTRL       : INTEGER := 16#54#;
    constant ADDR_DZ_STATUS_I_DATA_0   : INTEGER := 16#58#;
    constant ADDR_DZ_STATUS_I_CTRL     : INTEGER := 16#5c#;
    constant ADDR_DZ_STATUS_O_DATA_0   : INTEGER := 16#60#;
    constant ADDR_DZ_STATUS_O_CTRL     : INTEGER := 16#64#;
    constant ADDR_DZ_WINDOW_I_DATA_0   : INTEGER := 16#68#;
    constant ADDR_DZ_WINDOW_I_CTRL     : INTEGER := 16#6c#;
    constant ADDR_DZ_WINDOW_O_DATA_0   : INTEGER := 16#70#;
    constant ADDR_DZ_WINDOW_O_CTRL     : INTEGER := 16#74#;
    constant ADDR_DZ_AGE_I_DATA_0      : INTEGER := 16#78#;
    constant ADDR_DZ_AGE_I_CTRL        : INTEGER := 16#7c#;
    constant ADDR_DZ_AGE_O_DATA_0      : INTEGER := 16#80#;
    constant ADDR_DZ_AGE_O_CTRL        : INTEGER := 16#84#;
    constant ADDR_DZ_LOW_COUNT_DATA_0  : INTEGER := 16#88#;
    constant ADDR_DZ_LOW_COUNT_CTRL    : INTEGER := 16#8c#;
    constant ADDR_DZ_HIGH_COUNT_DATA_0 : INTEGER := 16#98#;
    constant ADDR_DZ_HIGH_COUNT_CTRL   : INTEGER := 16#9c#;
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
    signal int_filtered_i      : UNSIGNED(31 downto 0) := (others => '0');
    signal int_filtered_o      : UNSIGNED(31 downto 0) := (others => '0');
    signal int_filtered_o_ap_vld : STD_LOGIC;
    signal int_raw_i           : UNSIGNED(31 downto 0) := (others => '0');
    signal int_raw_o           : UNSIGNED(31 downto 0) := (others => '0');
    signal int_raw_o_ap_vld    : STD_LOGIC;
    signal int_window_size     : UNSIGNED(31 downto 0) := (others => '0');
    signal int_trim_count      : UNSIGNED(31 downto 0) := (others => '0');
    signal int_filter_mode     : UNSIGNED(31 downto 0) := (others => '0');
    signal int_rail_low        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_rail_high       : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_status_i     : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_status_o     : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_status_o_ap_vld : STD_LOGIC;
    signal int_dz_window_i     : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_window_o     : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_window_o_ap_vld : STD_LOGIC;
    signal int_dz_age_i        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_age_o        : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_age_o_ap_vld : STD_LOGIC;
    signal int_dz_low_count    : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_low_count_ap_vld : STD_LOGIC;
    signal int_dz_high_count   : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dz_high_count_ap_vld : STD_LOGIC;


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
                    when ADDR_FILTERED_I_DATA_0 =>
                        rdata_data <= RESIZE(int_filtered_i(31 downto 0), 32);
                    when ADDR_FILTERED_O_DATA_0 =>
                        rdata_data <= RESIZE(int_filtered_o(31 downto 0), 32);
                    when ADDR_FILTERED_O_CTRL =>
                        rdata_data(0) <= int_filtered_o_ap_vld;
                    when ADDR_RAW_I_DATA_0 =>
                        rdata_data <= RESIZE(int_raw_i(31 downto 0), 32);
                    when ADDR_RAW_O_DATA_0 =>
                        rdata_data <= RESIZE(int_raw_o(31 downto 0), 32);
                    when ADDR_RAW_O_CTRL =>
                        rdata_data(0) <= int_raw_o_ap_vld;
                    when ADDR_WINDOW_SIZE_DATA_0 =>
                        rdata_data <= RESIZE(int_window_size(31 downto 0), 32);
                    when ADDR_TRIM_COUNT_DATA_0 =>
                        rdata_data <= RESIZE(int_trim_count(31 downto 0), 32);
                    when ADDR_FILTER_MODE_DATA_0 =>
                        rdata_data <= RESIZE(int_filter_mode(31 downto 0), 32);
                    when ADDR_RAIL_LOW_DATA_0 =>
                        rdata_data <= RESIZE(int_rail_low(31 downto 0), 32);
                    when ADDR_RAIL_HIGH_DATA_0 =>
                        rdata_data <= RESIZE(int_rail_high(31 downto 0), 32);
                    when ADDR_DZ_STATUS_I_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_status_i(31 downto 0), 32);
                    when ADDR_DZ_STATUS_O_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_status_o(31 downto 0), 32);
                    when ADDR_DZ_STATUS_O_CTRL =>
                        rdata_data(0) <= int_dz_status_o_ap_vld;
                    when ADDR_DZ_WINDOW_I_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_window_i(31 downto 0), 32);
                    when ADDR_DZ_WINDOW_O_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_window_o(31 downto 0), 32);
                    when ADDR_DZ_WINDOW_O_CTRL =>
                        rdata_data(0) <= int_dz_window_o_ap_vld;
                    when ADDR_DZ_AGE_I_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_age_i(31 downto 0), 32);
                    when ADDR_DZ_AGE_O_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_age_o(31 downto 0), 32);
                    when ADDR_DZ_AGE_O_CTRL =>
                        rdata_data(0) <= int_dz_age_o_ap_vld;
                    when ADDR_DZ_LOW_COUNT_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_low_count(31 downto 0), 32);
                    when ADDR_DZ_LOW_COUNT_CTRL =>
                        rdata_data(0) <= int_dz_low_count_ap_vld;
                    when ADDR_DZ_HIGH_COUNT_DATA_0 =>
                        rdata_data <= RESIZE(int_dz_high_count(31 downto 0), 32);
                    when ADDR_DZ_HIGH_COUNT_CTRL =>
                        rdata_data(0) <= int_dz_high_count_ap_vld;
                    when others =>
                        NULL;
                    end case;
                end if;
            end if;
        end if;
    end process;

-- ----------------------- Register logic ----------------
    filtered_i           <= STD_LOGIC_VECTOR(int_filtered_i);
    raw_i                <= STD_LOGIC_VECTOR(int_raw_i);
    window_size          <= STD_LOGIC_VECTOR(int_window_size);
    trim_count           <= STD_LOGIC_VECTOR(int_trim_count);
    filter_mode          <= STD_LOGIC_VECTOR(int_filter_mode);
    rail_low             <= STD_LOGIC_VECTOR(int_rail_low);
    rail_high            <= STD_LOGIC_VECTOR(int_rail_high);
    dz_status_i          <= STD_LOGIC_VECTOR(int_dz_status_i);
    dz_window_i          <= STD_LOGIC_VECTOR(int_dz_window_i);
    dz_age_i             <= STD_LOGIC_VECTOR(int_dz_age_i);

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_FILTERED_I_DATA_0) then
                    int_filtered_i(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_filtered_i(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_filtered_o <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (filtered_o_ap_vld = '1') then
                    int_filtered_o <= UNSIGNED(filtered_o); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_filtered_o_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (filtered_o_ap_vld = '1') then
                    int_filtered_o_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_FILTERED_O_CTRL) then
                    int_filtered_o_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_RAW_I_DATA_0) then
                    int_raw_i(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_raw_i(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_raw_o <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (raw_o_ap_vld = '1') then
                    int_raw_o <= UNSIGNED(raw_o); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_raw_o_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (raw_o_ap_vld = '1') then
                    int_raw_o_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_RAW_O_CTRL) then
                    int_raw_o_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_WINDOW_SIZE_DATA_0) then
                    int_window_size(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_window_size(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_TRIM_COUNT_DATA_0) then
                    int_trim_count(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_trim_count(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_FILTER_MODE_DATA_0) then
                    int_filter_mode(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_filter_mode(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_RAIL_LOW_DATA_0) then
                    int_rail_low(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_rail_low(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_RAIL_HIGH_DATA_0) then
                    int_rail_high(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_rail_high(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DZ_STATUS_I_DATA_0) then
                    int_dz_status_i(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dz_status_i(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_status_o <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (dz_status_o_ap_vld = '1') then
                    int_dz_status_o <= UNSIGNED(dz_status_o); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_status_o_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (dz_status_o_ap_vld = '1') then
                    int_dz_status_o_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_DZ_STATUS_O_CTRL) then
                    int_dz_status_o_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DZ_WINDOW_I_DATA_0) then
                    int_dz_window_i(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dz_window_i(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_window_o <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (dz_window_o_ap_vld = '1') then
                    int_dz_window_o <= UNSIGNED(dz_window_o); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_window_o_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (dz_window_o_ap_vld = '1') then
                    int_dz_window_o_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_DZ_WINDOW_O_CTRL) then
                    int_dz_window_o_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DZ_AGE_I_DATA_0) then
                    int_dz_age_i(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dz_age_i(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_age_o <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (dz_age_o_ap_vld = '1') then
                    int_dz_age_o <= UNSIGNED(dz_age_o); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_age_o_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (dz_age_o_ap_vld = '1') then
                    int_dz_age_o_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_DZ_AGE_O_CTRL) then
                    int_dz_age_o_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_low_count <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (dz_low_count_ap_vld = '1') then
                    int_dz_low_count <= UNSIGNED(dz_low_count); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_low_count_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (dz_low_count_ap_vld = '1') then
                    int_dz_low_count_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_DZ_LOW_COUNT_CTRL) then
                    int_dz_low_count_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_high_count <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (dz_high_count_ap_vld = '1') then
                    int_dz_high_count <= UNSIGNED(dz_high_count); -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dz_high_count_ap_vld <= '0';
            elsif (ACLK_EN = '1') then
                if (dz_high_count_ap_vld = '1') then
                    int_dz_high_count_ap_vld <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_DZ_HIGH_COUNT_CTRL) then
                    int_dz_high_count_ap_vld <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;


-- ----------------------- Memory logic ------------------

end architecture behave;
