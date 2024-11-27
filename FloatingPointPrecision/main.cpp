#include <algorithm>
#include <chrono>
#include <clocale>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

template <typename T>
std::vector<T> uniformTimeSamples(const T offset, const T step_size, const int numEl) {
    std::vector<int> ivec(numEl);
    std::iota(ivec.begin(), ivec.end(), 0); // ivec will become: [0..99]
    std::vector<T> output(ivec.size());
    std::transform(ivec.begin(), ivec.end(), output.begin(),
                   [step_size, offset](int value) {
                       return std::fma(value, step_size, offset);
                   });
    return output;
}

int main() {
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();

  const int startVal = 0;
  const int numEl = 100000;

  // 40 FPS Sampling Rate
  const double rate = 1.0 / 40.0;
  // Last value expected in sampled array
  const double last_num = 2510.65;

  const double offset = 10.675;

  std::vector<int> integers(numEl);

  for (int i = startVal; i < numEl; ++i) {
    integers[i] = i;
  }

  std::vector<double> decimalValues(numEl);
  decimalValues[0] = offset;
  double time = decimalValues[0];
  for (int i = 1; i < numEl; ++i) {
    time += rate;
    decimalValues[i] = time;
  }

  std::vector<double> decimalValues2(numEl);
  for (int i = startVal; i < numEl; ++i) {
    decimalValues2[i] = integers[i] * rate + offset;
  }

  std::vector<double> decimalValues3(numEl);
  for (int i = startVal; i < numEl; ++i) {
    decimalValues3[i] = std::fma(integers[i], rate, offset);
  }

  std::vector<double> decimalValues4 = uniformTimeSamples(offset, rate, numEl);

  for (int i = startVal; i < numEl; ++i) {
    std::cout << std::fixed << std::setprecision(32)
              << "Add: " << decimalValues[i]
              << " Multiply: " << decimalValues2[i]
              << " FMA: " << decimalValues3[i]
              << " CUSTOM: " << decimalValues4[i] << std::endl;
  }

  const double err1 = std::abs(last_num - decimalValues[numEl - 1]);
  const double err2 = std::abs(last_num - decimalValues2[numEl - 1]);
  const double err3 = std::abs(last_num - decimalValues3[numEl - 1]);
  const double err4 = std::abs(last_num - decimalValues3[numEl - 1]);

  std::cout << "\nError of last value: " << std::endl;
  std::cout << std::fixed << std::setprecision(32) << "Add: " << err1
            << " Multiply: " << err2 << " FMA: " << err3 << " CUSTOM: " << err4 << std::endl;

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}
