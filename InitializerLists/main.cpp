// INCLUDES
#include <OpenSim/Common/Array.h>

#include <clocale>
#include <chrono> // for std::chrono functions
#include <iostream>
#include <string>

int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // This works 
    OpenSim::Array<double> timeRange(0, 2, 2);
    timeRange[0] = -1.0;
    timeRange[1] = 2.0;
    std::cout << timeRange << std::endl;

    // And this works but doesn't give me what I expect
    OpenSim::Array<int> timeRange2{-1, 2};
    std::cout << timeRange2 << std::endl;

    // But this doesn't even compile
    // OpenSim::Array<double> timeRange3{1.0, 2.0};

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    return 0;
}
