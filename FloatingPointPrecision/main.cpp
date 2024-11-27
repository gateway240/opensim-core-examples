#include <algorithm>
#include <chrono>
#include <clocale>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

template <typename T>
std::pair<bool, double> isUniform(const std::vector<T>& x) {
    // Check if the vector is empty or has only one element
    if (x.size() < 2) {
        return {false, std::numeric_limits<double>::quiet_NaN()};
    }

    // Check if the vector is a real number (not complex)
    if (!std::is_floating_point<T>::value && !std::is_integral<T>::value) {
        throw std::invalid_argument("Input must be a real numeric vector.");
    }

    // Initialize step as NaN
    double step = std::numeric_limits<double>::quiet_NaN();
    bool tf = false;

    // Handle integer types
    if (std::is_integral<T>::value) {
        std::vector<unsigned long long> unsigned_x(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            unsigned_x[i] = static_cast<unsigned long long>(x[i]);
        }

        // Check if the vector is sorted in descending order
        bool xWasFlipped = false;
        if (unsigned_x[1] < unsigned_x[0]) {
            std::reverse(unsigned_x.begin(), unsigned_x.end());
            xWasFlipped = true;
        }

        unsigned long long integerStep = unsigned_x[1] - unsigned_x[0];
        tf = std::all_of(unsigned_x.begin() + 1, unsigned_x.end(), [&](unsigned long long val) {
            return val - unsigned_x[&val - &unsigned_x[0] - 1] == integerStep;
        });

        if (integerStep == 0 && tf) {
            tf = std::all_of(unsigned_x.begin() + 1, unsigned_x.end(), [&](unsigned long long val) {
                return val == unsigned_x[0];
            });
        }

        if (tf) {
            step = (xWasFlipped ? -1.0 : 1.0) * static_cast<double>(integerStep);
        }
    } else { // Handle floating-point types
        double maxElement = std::max(std::abs(x.front()), std::abs(x.back()));
        double tol = 4 * std::numeric_limits<double>::epsilon() * maxElement;
        size_t numSpaces = x.size() - 1;
        double span = x.back() - x.front();
        const double mean_step = (std::isfinite(span)) ? span / numSpaces : (x.back() / numSpaces - x.front() / numSpaces);

        double stepAbs = std::abs(mean_step);
        if (stepAbs < tol) {
            tol = (stepAbs < std::numeric_limits<double>::epsilon() * maxElement) ? 
                    std::numeric_limits<double>::epsilon() * maxElement : 
                    stepAbs;
        }
        std::vector<T> results(x.size());
        std::adjacent_difference(x.begin(), x.end(),results.begin());
        // The first value from std::adjacent_difference is the first input so it is skipped 
        tf = std::all_of(results.begin() + 1, results.end(), [&mean_step, &tol](T val) {
            return std::abs(val - mean_step) <= tol;
        });

        if (!tf && x.size() == 2) {
            tf = true; // Handle special case for two elements
        }
        // std::cout << "TF: " << tf << " Mean step: " << mean_step << " Tol: " << tol << std::endl;
        if (tf) {
            step = mean_step;
        }
    }

    return {tf, step};
}

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
  std::vector<double> vec2 = {0.0,0.5,0.11,0.16,0.19,0.24};
  auto [tf3, step3] = isUniform(vec2);
  // Should be non-uniform
  std::cout << "Sanity check test vector is " << (tf3 ? "uniformly spaced." : "not uniformly spaced.") << std::endl; // Not uniformly spaced

  auto [tf1, step1] = isUniform(decimalValues);
  std::cout << "Addition Uniformly Spaced: " << tf1 << " Step: " << step1 << std::endl;

  auto [tf2, step2] = isUniform(decimalValues4);
  std::cout << "FMA Uniformly Spaced: " << tf2 << " Step: " << step2 << std::endl;

  const double err1 = std::abs(last_num - decimalValues[numEl - 1]);
  const double err2 = std::abs(last_num - decimalValues2[numEl - 1]);
  const double err3 = std::abs(last_num - decimalValues3[numEl - 1]);
  const double err4 = std::abs(last_num - decimalValues4[numEl - 1]);

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
