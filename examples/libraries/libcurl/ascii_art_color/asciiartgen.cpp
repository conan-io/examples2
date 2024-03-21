#include <iostream>
#include <curl/curl.h>
#include <vector>
#include <string>
#include <fmt/core.h>
#include <fmt/color.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Size of ASCII art
static const int new_width = 80;

// Ascii gradient
static const std::string ASCII_CHARS = " .:-=+#%@@@";

// Function to scale the luminance into an ASCII character
char map_luminance_to_ascii(float luminance) {
    size_t position = luminance * (ASCII_CHARS.size() - 1);
    return ASCII_CHARS[position];
}

// Function to write data from curl call
static size_t write_data(void* ptr, size_t size, size_t nmemb, void* stream) {
    ((std::string*)stream)->append((char*)ptr, size * nmemb);
    return size * nmemb;
}

std::string download_image(const std::string& url) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "curl init failed" << std::endl;
        return "";
    }

    std::string response_string;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Enable following redirection

    // Deactivate SSL verification, just for development!
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
    }

    curl_easy_cleanup(curl);
    return response_string;
}

// Function to convert image to ASCII art
void image_to_ascii(const std::string& image_data) {
    int width, height, channels;
    unsigned char* data = stbi_load_from_memory(
        reinterpret_cast<const stbi_uc*>(image_data.data()),
        image_data.size(),
        &width,
        &height,
        &channels,
        0
    );

    if (data == nullptr) {
        std::cerr << "Error loading image" << std::endl;
        return;
    }

    // Adjust aspect ratio for ASCII art
    int new_height = static_cast<int>(static_cast<double>(height) / width * new_width * 0.45);

    for (int i = 0; i < new_height; ++i) {
        for (int j = 0; j < new_width; ++j) {
            int old_i = i * height / new_height;
            int old_j = j * width / new_width;

            float r = data[(old_i * width + old_j) * channels + 0] / 255.0f;
            float g = data[(old_i * width + old_j) * channels + 1] / 255.0f;
            float b = data[(old_i * width + old_j) * channels + 2] / 255.0f;
            float luminance = (0.2126f * r + 0.7152f * g + 0.0722f * b);

            char ascii_char = map_luminance_to_ascii(luminance);

            // Use fmt to print ASCII character with color
            fmt::print(fmt::fg(fmt::rgb(uint8_t(r * 255), uint8_t(g * 255), uint8_t(b * 255))), "{}", ascii_char);
        }
        fmt::print("\n");
    }

    stbi_image_free(data);
}

int main(int argc, char** argv) {
    // Picture by Roshan Kamath from pexels.com
    std::string url = "https://images.pexels.com/photos/1661179/pexels-photo-1661179.jpeg";

    if (argc > 1) {
        url = argv[1]; // Use provided URL if available
    }

    std::string image_data = download_image(url);
    image_to_ascii(image_data);

    return 0;
}
