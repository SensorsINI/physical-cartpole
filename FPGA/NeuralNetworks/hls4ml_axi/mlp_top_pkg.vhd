package mlp_top_pkg is
    constant MLP_INPUT_NEURONS    : integer := 7;
    constant MLP_INPUT_DATA_BITS  : integer := 14;
    constant MLP_INPUT_BIT_WIDTH  : integer := MLP_INPUT_NEURONS * MLP_INPUT_DATA_BITS;

    constant MLP_OUTPUT_NEURONS   : integer := 1;
    constant MLP_OUTPUT_DATA_BITS : integer := 14;
    constant MLP_OUTPUT_BIT_WIDTH : integer := MLP_OUTPUT_NEURONS * MLP_OUTPUT_DATA_BITS;
end package mlp_top_pkg;
