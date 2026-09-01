#include <sycl/sycl.hpp>
#include <zlib.h>
#include <fmt/core.h>
#include <vector>

int main() {
    sycl::queue q;
    fmt::print("SYCL Device: {}\n", 
               q.get_device().get_info<sycl::info::device::name>());

    // Compute squares using SYCL parallel_for
    constexpr size_t N = 10000;
    std::vector<float> data(N);
    {
        sycl::buffer buf(data);
        q.submit([&](sycl::handler& h) {
            auto acc = buf.get_access<sycl::access::mode::write>(h);
            h.parallel_for(N, [=](sycl::id<1> i) {
                acc[i] = static_cast<float>(i[0] * i[0]);
            });
        });
    }
    fmt::print("Computed {} squares. First: {}, Last: {}\n",
               N, data[0], data.back());

    // Compress results with zlib
    std::vector<Bytef> compressed(compressBound(data.size() * sizeof(float)));
    uLongf comp_size = compressed.size();
    compress(compressed.data(), &comp_size,
             reinterpret_cast<const Bytef*>(data.data()),
             data.size() * sizeof(float));

    fmt::print("Compressed {} bytes -> {} bytes ({:.1f}%)\n",
               data.size() * sizeof(float), comp_size,
               100.0 * comp_size / (data.size() * sizeof(float)));
}
