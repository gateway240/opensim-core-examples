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
std::pair<bool, double> isUniform(const std::vector<T> &x) {

  // Initialize step as NaN
  T step = std::numeric_limits<T>::quiet_NaN();
  bool tf = false;

  T maxElement = std::max(std::abs(x.front()), std::abs(x.back()));
  T tol = 4 * std::numeric_limits<T>::epsilon() * maxElement;
  size_t numSpaces = x.size() - 1;
  T span = x.back() - x.front();
  const T mean_step = (std::isfinite(span))
                          ? span / numSpaces
                          : (x.back() / numSpaces - x.front() / numSpaces);

  T stepAbs = std::abs(mean_step);
  if (stepAbs < tol) {
    tol = (stepAbs < std::numeric_limits<T>::epsilon() * maxElement)
              ? std::numeric_limits<T>::epsilon() * maxElement
              : stepAbs;
  }
  std::vector<T> results(x.size());
  std::adjacent_difference(x.begin(), x.end(), results.begin());
  // First value from adjacent_difference is the first input so it is skipped
  tf = std::all_of(
      results.begin() + 1, results.end(),
      [&mean_step, &tol](T val) { return std::abs(val - mean_step) <= tol; });

  if (!tf && x.size() == 2) {
    tf = true; // Handle special case for two elements
  }
  if (tf) {
    step = mean_step;
  }

  return {tf, step};
}

template <typename T>
std::vector<T> createVectorLinspace(
        int length, T start, T end) {
    std::vector<T> v(length);
    for (int i = 0; i < length; ++i) {
        v[i] = start + i * (end - start) / (length - 1);
    }
    return v;
}

template <typename T>
std::vector<T> uniformTimeSamples(const T offset, const T step_size,
                                  const int numEl) {
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

  // std::vector<double> decimalValues4 = uniformTimeSamples(offset, rate, numEl);

  std::vector<double> decimalValues4 = createVectorLinspace(numEl,offset,last_num);

  for (int i = startVal; i < numEl; ++i) {
    std::cout << std::fixed << std::setprecision(32)
              << "Add: " << decimalValues[i]
              << " Multiply: " << decimalValues2[i]
              << " FMA: " << decimalValues3[i]
              << " CUSTOM: " << decimalValues4[i] << std::endl;
  }
  std::vector<double> vec2 = {0.0, 0.5, 0.11, 0.16, 0.19, 0.24};
  auto [tf3, step3] = isUniform(vec2);
  // Should be non-uniform
  std::cout << "Sanity check test vector is "
            << (tf3 ? "uniformly spaced." : "not uniformly spaced.")
            << std::endl; // Not uniformly spaced

  auto [tf1, step1] = isUniform(decimalValues);
  std::cout << "Addition Uniformly Spaced: " << tf1 << " Step: " << step1
            << std::endl;

  auto [tf2, step2] = isUniform(decimalValues3);
  std::cout << "FMA Uniformly Spaced: " << tf2 << " Step: " << step2
            << std::endl;

  auto [tf4, step4] = isUniform(decimalValues4);
  std::cout << "FMA Uniformly Spaced: " << tf4 << " Step: " << step4
            << std::endl;

  const double err1 = std::abs(last_num - decimalValues[numEl - 1]);
  const double err2 = std::abs(last_num - decimalValues2[numEl - 1]);
  const double err3 = std::abs(last_num - decimalValues3[numEl - 1]);
  const double err4 = std::abs(last_num - decimalValues4[numEl - 1]);

  std::cout << "\nError of last value: " << std::endl;
  std::cout << std::fixed << std::setprecision(32) << "Add: " << err1
            << " Multiply: " << err2 << " FMA: " << err3 << " CUSTOM: " << err4
            << std::endl;

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}
