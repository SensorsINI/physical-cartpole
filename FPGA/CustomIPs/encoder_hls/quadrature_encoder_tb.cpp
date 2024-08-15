#include <iostream>
#include "quadrature_encoder.h"

// Test function
void test_encoder(bool A, bool B, bool reset, int &count) {
    quadrature_encoder(A, B, reset, &count);
    std::cout << "A=" << A << ", B=" << B << ", Count=" << count << std::endl;
}

int main() {
    int count = 0; // This variable will hold the count value
    
    // Test case 1: Simulating clockwise rotation
    std::cout << "Simulating clockwise rotation:" << std::endl;
    test_encoder(0, 0, 0, count);
    test_encoder(0, 1, 0, count);
    test_encoder(1, 1, 0, count);
    test_encoder(1, 0, 0, count);
    test_encoder(0, 0, 0, count);

    std::cout << "\nResetting counter:" << std::endl;
    test_encoder(0, 0, 1, count);

    // Resetting count
    count = 0;
    std::cout << "\nSimulating counter-clockwise rotation:" << std::endl;
    // Test case 2: Simulating counter-clockwise rotation
    test_encoder(0, 0, 0, count);
    test_encoder(1, 0, 0, count);
    test_encoder(1, 1, 0, count);
    test_encoder(0, 1, 0, count);
    test_encoder(0, 0, 0, count);

    
    return 0;
}

