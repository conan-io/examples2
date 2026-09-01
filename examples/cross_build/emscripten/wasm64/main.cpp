#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

int main() {
  const size_t GIGABYTE = 1024 * 1024 * 1024;
  const size_t CHUNK_SIZE = 32 * 1024 * 1024; // 32 MiB
  std::vector<char *> allocations;
  size_t total_allocated = 0;

  while (true) {
    char *block = static_cast<char *>(malloc(CHUNK_SIZE));
    if (!block) {
      std::cerr << "Failed after allocating " << total_allocated / (1024 * 1024)
                << " MiB ~ " << total_allocated / GIGABYTE << " GiB"
                << std::endl;
      break;
    }

    // Touch memory to ensure it's actually committed (not lazy-allocated)
    std::memset(block, 0xAB, CHUNK_SIZE);
    allocations.push_back(block);
    total_allocated += CHUNK_SIZE;
    if (total_allocated % GIGABYTE == 0) {
      std::cout << "Current allocated: " << total_allocated / GIGABYTE << " GiB"
                << std::endl;
    }
  }

  // Free the memory afterward
  for (char *ptr : allocations) {
    free(ptr);
  }

  return 0;
}
