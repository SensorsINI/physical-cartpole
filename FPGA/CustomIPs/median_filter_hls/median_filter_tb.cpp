#include "median_filter.h"
#include <iostream>
#include <fstream>

int main() {
    // Creating a stream for testing
    hls::stream<stream_type> test_stream;
    stream_type input_sample;

    // Variable to store the output from the AXI Lite interface
    volatile unsigned short axi_lite_output;

    // Open a file to write the output
    std::ofstream outfile("test_output.txt");

    if (!outfile.is_open()) {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    // Counter to keep track of clock cycles
    int clock_cycles = 0;

    int test_length = 2000;
    int data_period = 10;
    short window_size = 64;

    // Run the simulation for a certain number of cycles
    for (int cycle = 0; cycle < test_length; cycle++) {
        // Write new data to the stream every 10 clock cycles
        if (clock_cycles == 0) {
            input_sample.data = cycle; // Example data
            test_stream.write(input_sample);
        }
//        input_sample.data = cycle; // Example data
//        test_stream.write(input_sample);
        // Call the median filter function
        if (cycle == 1000){
        	--window_size;
        }

        if (cycle == 1200){
        	window_size=1;
        }

        median_filter(test_stream, &axi_lite_output, window_size);

        // Write the result to the file
        outfile << "Cycle: " << cycle << ", Input: ";
        if (clock_cycles == 0) {
            outfile << input_sample.data;
        } else {
            outfile << "No new data";
        }
        outfile << ", Output: " << axi_lite_output << std::endl;

        // Increment and reset the clock cycle counter
        clock_cycles = (clock_cycles + 1) % data_period;
    }

    outfile << "Testbench completed successfully." << std::endl;
    outfile.close();
    return 0;
}
