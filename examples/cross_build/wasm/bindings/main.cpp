#include <Eigen/Core>
#include <cstdint>
#include <emscripten/emscripten.h>
#include <iostream>
#include <string>

#ifdef __cplusplus
#define EXTERN extern "C"
#else
#define EXTERN
#endif

EXTERN EMSCRIPTEN_KEEPALIVE uint32_t fib(uint32_t n) {
  std::cout << "Calculating Fibonacci for n = " << n << std::endl;
  if (n <= 1)
    return n;
  uint32_t a = 0, b = 1, c;
  for (uint32_t i = 2; i <= n; ++i) {
    c = a + b;
    a = b;
    b = c;
  }
  return c;
}

EXTERN EMSCRIPTEN_KEEPALIVE void printMessage(const char *message) {
  std::cout << "Message from C: " << message << std::endl;
  std::string script =
      "alert('Message from C++: " + std::string(message) + "')";
  std::cout << "Executing script: " << script << std::endl;
  emscripten_run_script(script.c_str());
}

EXTERN EMSCRIPTEN_KEEPALIVE void addOne(int32_t *input, int32_t *output) {
  *output = *input + 1;
}

EXTERN EMSCRIPTEN_KEEPALIVE float sumArray(const float *data, int32_t size) {
  // print data input
  std::cout << "Data input: ";
  for (int i = 0; i < size; ++i) {
    std::cout << data[i] << " ";
  }
  std::cout << std::endl;
  Eigen::Map<const Eigen::ArrayXf> vec(data, size);
  return vec.sum();
}

int main() {
  std::cout << "Hello World!" << std::endl;
  auto data = new float[5]{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
  std::cout << sumArray(data, 5) << std::endl;
}
