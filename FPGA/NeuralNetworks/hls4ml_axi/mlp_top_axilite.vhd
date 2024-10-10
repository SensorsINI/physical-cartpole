library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mlp is
    generic(
        AXI_DATA_WIDTH       : integer := 32;
        AXI_ADDR_WIDTH       : integer := 32;
        MLP_INPUT_NEURONS    : integer := 7;
        MLP_INPUT_DATA_BITS  : integer := 12;
        MLP_OUTPUT_NEURONS   : integer := 1;
        MLP_OUTPUT_DATA_BITS : integer := 12
    );
    port(
        -- AXI clock and reset
        AXI_ACLK     : in  std_logic;
        AXI_ARESETN  : in  std_logic;
        -- AXI Lite interface
        S_AXI_AWADDR : in  std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0);
        S_AXI_AWVALID: in  std_logic;
        S_AXI_WDATA  : in  std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
        S_AXI_WVALID : in  std_logic;
        S_AXI_WREADY : out std_logic;
        S_AXI_BREADY : in  std_logic;
        S_AXI_BVALID : out std_logic;
        S_AXI_ARADDR : in  std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0);
        S_AXI_ARVALID: in  std_logic;
        S_AXI_RDATA  : out std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
        S_AXI_RVALID : out std_logic;
        S_AXI_RREADY : in  std_logic;
        S_AXI_RRESP  : out std_logic_vector(1 downto 0);
        S_AXI_BRESP  : out std_logic_vector(1 downto 0);
        S_AXI_ARREADY: out std_logic;
        S_AXI_AWREADY: out std_logic
    );
end entity mlp;

architecture RTL of mlp is
    -- Constants
    constant MLP_INPUT_BIT_WIDTH  : integer := MLP_INPUT_NEURONS * MLP_INPUT_DATA_BITS;
    constant MLP_OUTPUT_BIT_WIDTH : integer := MLP_OUTPUT_NEURONS * MLP_OUTPUT_DATA_BITS;
    -- Now we have one word per input neuron
    constant INPUT_WORDS          : integer := MLP_INPUT_NEURONS;
    constant OUTPUT_WORDS         : integer := MLP_OUTPUT_NEURONS; -- Assuming one word per output neuron

    -- Address mapping
    constant INPUT_ADDR_BASE      : std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0) := (others => '0'); -- Base address for input registers
    constant OUTPUT_ADDR_BASE     : std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0) := x"00001000"; -- Base address for output registers
    constant CONTROL_ADDR         : std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0) := x"00002000"; -- Address for control register
    constant STATUS_ADDR          : std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0) := x"00002004"; -- Address for status register

    -- Types
    type reg_array_t is array(0 to INPUT_WORDS - 1) of std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);
    type output_reg_array_t is array(0 to OUTPUT_WORDS - 1) of std_logic_vector(AXI_DATA_WIDTH - 1 downto 0);

    -- Registers
    signal input_regs  : reg_array_t := (others => (others => '0'));
    signal output_regs : output_reg_array_t := (others => (others => '0'));

    -- Internal control signals
    signal ap_start_int         : std_logic := '0';
    signal ap_done_int          : std_logic := '0';
    signal ap_idle_int          : std_logic := '0';
    signal ap_ready_int         : std_logic := '0';
    signal ap_rst_int           : std_logic;  -- Internal reset signal
    signal ap_done_latched_reset: std_logic := '0';

    -- AXI Lite signals
    signal awready_int       : std_logic := '0';
    signal wready_int        : std_logic := '0';
    signal bvalid_int        : std_logic := '0';
    signal arready_int       : std_logic := '0';
    signal rvalid_int        : std_logic := '0';
    signal rdata_int         : std_logic_vector(AXI_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal write_addr_int    : std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal read_addr_int     : std_logic_vector(AXI_ADDR_WIDTH - 1 downto 0) := (others => '0');

    -- Flags to track input readiness
    signal input_write_flags : std_logic_vector(INPUT_WORDS - 1 downto 0) := (others => '0');
    signal all_inputs_ready  : std_logic := '0';
    signal output_valid      : std_logic := '0';

    -- MLP interface signals
    signal reg_input         : std_logic_vector(MLP_INPUT_BIT_WIDTH - 1 downto 0) := (others => '0');
    signal reg_output        : std_logic_vector(MLP_OUTPUT_BIT_WIDTH - 1 downto 0) := (others => '0');
    signal layer_out_int     : std_logic_vector(MLP_OUTPUT_BIT_WIDTH - 1 downto 0) := (others => '0');

    -- Helper signals
    signal aw_en             : std_logic := '0';
    signal ar_en             : std_logic := '0';

    -- Latching ap_done
    signal ap_done_latched   : std_logic := '0';

    -- Constant to represent all ones for input_write_flags
    constant ALL_ONES_FLAGS : std_logic_vector(INPUT_WORDS - 1 downto 0) := (others => '1');

    -- Dummy signal for unconnected output port
    signal layer9_out_0_V_ap_vld_int : std_logic;

begin
    -- AXI Lite outputs
    S_AXI_AWREADY <= awready_int;
    S_AXI_WREADY  <= wready_int;
    S_AXI_BVALID  <= bvalid_int;
    S_AXI_BRESP   <= "00"; -- OKAY response

    S_AXI_ARREADY <= arready_int;
    S_AXI_RVALID  <= rvalid_int;
    S_AXI_RDATA   <= rdata_int;
    S_AXI_RRESP   <= "00"; -- OKAY response

    -- Create a static internal reset signal
    ap_rst_int <= not AXI_ARESETN;

    -- Write channel process
    write_channel: process(AXI_ACLK)
        variable input_reg_index : integer range 0 to INPUT_WORDS - 1;
    begin
        if rising_edge(AXI_ACLK) then
            if AXI_ARESETN = '0' then
                awready_int       <= '0';
                wready_int        <= '0';
                bvalid_int        <= '0';
                aw_en             <= '0';
                write_addr_int    <= (others => '0');
                input_write_flags <= (others => '0');
                all_inputs_ready  <= '0';
                ap_start_int      <= '0';
                ap_done_latched_reset <= '0';
            else
                -- Default assignments
                ap_done_latched_reset <= '0';

                -- Compute all_inputs_ready
                if input_write_flags = ALL_ONES_FLAGS then
                    all_inputs_ready <= '1';
                else
                    all_inputs_ready <= '0';
                end if;

                -- Write address handshake
                if (S_AXI_AWVALID = '1' and awready_int = '0') then
                    awready_int    <= '1';
                    write_addr_int <= S_AXI_AWADDR;
                    aw_en          <= '1';
                elsif (awready_int = '1' and S_AXI_AWVALID = '0') then
                    awready_int <= '0';
                end if;

                -- Write data handshake
                if (S_AXI_WVALID = '1' and wready_int = '0') then
                    wready_int <= '1';
                    if (aw_en = '1') then
                        -- Decode address and write data
                        if (write_addr_int >= INPUT_ADDR_BASE and write_addr_int < std_logic_vector(unsigned(INPUT_ADDR_BASE) + (INPUT_WORDS * 4))) then
                            -- Write to input registers
                            input_reg_index := to_integer(unsigned(write_addr_int(5 downto 2)));
                            if (input_reg_index >= 0 and input_reg_index < INPUT_WORDS) then
                                input_regs(input_reg_index)        <= S_AXI_WDATA;
                                input_write_flags(input_reg_index) <= '1';
                            end if;
                        elsif (write_addr_int = CONTROL_ADDR) then
                            -- Control register (start signal and clear done flag)
                            if S_AXI_WDATA(0) = '1' then
                                ap_start_int <= '1'; -- Start the MLP
                                if all_inputs_ready = '1' then
                                    input_write_flags <= (others => '0'); -- Reset flags
                                end if;
                            end if;
                            if S_AXI_WDATA(1) = '1' then
                                ap_done_latched_reset <= '1'; -- Request to clear done flag
                            end if;
                        end if;
                    end if;
                elsif (wready_int = '1' and S_AXI_WVALID = '0') then
                    wready_int <= '0';
                end if;

                -- Write response
                if (aw_en = '1' and wready_int = '1' and bvalid_int = '0') then
                    bvalid_int <= '1';
                    aw_en      <= '0';
                elsif (bvalid_int = '1' and S_AXI_BREADY = '1') then
                    bvalid_int <= '0';
                end if;

                -- Reset ap_start_int when MLP is ready
                if ap_ready_int = '1' then
                    ap_start_int <= '0';
                end if;
            end if;
        end if;
    end process write_channel;

    -- Read channel process
    read_channel: process(AXI_ACLK)
        variable output_reg_index : integer range 0 to OUTPUT_WORDS - 1;
    begin
        if rising_edge(AXI_ACLK) then
            if AXI_ARESETN = '0' then
                arready_int   <= '0';
                rvalid_int    <= '0';
                ar_en         <= '0';
                read_addr_int <= (others => '0');
                rdata_int     <= (others => '0');
            else
                -- Read address handshake
                if (S_AXI_ARVALID = '1' and arready_int = '0') then
                    arready_int   <= '1';
                    read_addr_int <= S_AXI_ARADDR;
                    ar_en         <= '1';
                elsif (arready_int = '1' and S_AXI_ARVALID = '0') then
                    arready_int <= '0';
                end if;

                -- Read data
                if (ar_en = '1' and rvalid_int = '0') then
                    if (read_addr_int >= OUTPUT_ADDR_BASE and read_addr_int < std_logic_vector(unsigned(OUTPUT_ADDR_BASE) + (OUTPUT_WORDS * 4))) then
                        -- Read from output registers
                        output_reg_index := to_integer(unsigned(read_addr_int(5 downto 2)));
                        if (output_reg_index >= 0 and output_reg_index < OUTPUT_WORDS) then
                            rdata_int <= output_regs(output_reg_index);
                        end if;
                    elsif (read_addr_int = STATUS_ADDR) then
                        -- Read status register
                        rdata_int     <= (others => '0');
                        rdata_int(0)  <= ap_done_latched; -- Indicate if MLP is done
                    else
                        rdata_int <= (others => '0');
                    end if;
                    rvalid_int <= '1';
                    ar_en      <= '0';
                elsif (rvalid_int = '1' and S_AXI_RREADY = '1') then
                    rvalid_int <= '0';
                end if;
            end if;
        end if;
    end process read_channel;

    -- Assemble input data
    input_assembly: process(AXI_ACLK)
    begin
        if rising_edge(AXI_ACLK) then
            if AXI_ARESETN = '0' then
                reg_input <= (others => '0');
            else
                if all_inputs_ready = '1' then
                    -- Concatenate input registers into reg_input
                    for i in 0 to INPUT_WORDS - 1 loop
                        reg_input((i+1)*MLP_INPUT_DATA_BITS - 1 downto i*MLP_INPUT_DATA_BITS) <=
                            input_regs(i)(MLP_INPUT_DATA_BITS - 1 downto 0);
                    end loop;
                    -- Do not reset input_write_flags here
                end if;
            end if;
        end if;
    end process input_assembly;

    -- Capture MLP output data when done
    output_capture: process(AXI_ACLK)
    begin
        if rising_edge(AXI_ACLK) then
            if AXI_ARESETN = '0' then
                reg_output     <= (others => '0');
                output_valid   <= '0';
                ap_done_latched <= '0';
            else
                if ap_done_latched_reset = '1' then
                    ap_done_latched <= '0';
                    output_valid    <= '0';
                elsif ap_done_int = '1' and output_valid = '0' then
                    reg_output     <= layer_out_int;
                    output_valid   <= '1';
                    ap_done_latched <= '1';

                    -- Assign reg_output to output_regs
                    for i in 0 to OUTPUT_WORDS - 1 loop
                        output_regs(i) <= std_logic_vector(resize(unsigned(reg_output((i+1)*MLP_OUTPUT_DATA_BITS - 1 downto i*MLP_OUTPUT_DATA_BITS)), AXI_DATA_WIDTH));
                    end loop;
                end if;
            end if;
        end if;
    end process output_capture;

    -- Instantiate the MLP module
    inst_mlp : entity work.myproject
        port map(
            ap_clk                  => AXI_ACLK,
            ap_rst                  => ap_rst_int,
            ap_start                => ap_start_int,
            ap_done                 => ap_done_int,
            ap_idle                 => ap_idle_int,
            ap_ready                => ap_ready_int,
            input_1_V               => reg_input,
            input_1_V_ap_vld        => '1',
            layer9_out_0_V          => layer_out_int,
            layer9_out_0_V_ap_vld   => layer9_out_0_V_ap_vld_int  -- Connect to signal instead of 'open'
        );

    -- Ensure unused signals are properly handled
    layer9_out_0_V_ap_vld_int <= '0';

end architecture RTL;
