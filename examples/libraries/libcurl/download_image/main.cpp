#include <iostream>
#include <curl/curl.h>
#include <vector>
#include <string>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

// Size of ASCII art
static const int new_width = 100;

// Ascii gradient
static const std::string ASCII_CHARS = " .:-=+#%@@"; // Inverted colors

// Function to scale the luminance into an ASCII character
char map_luminance_to_ascii(float luminance) {
    size_t position = luminance * (ASCII_CHARS.size() - 1);
    return ASCII_CHARS[position];
}

// Function to download image
static size_t write_data(void* ptr, size_t size, size_t nmemb, void* stream) {
    ((std::string*)stream)->append((char*)ptr, size * nmemb);
    return size * nmemb;
}

std::string download_image(const std::string& url) {
    CURL* curl = curl_easy_init();
    std::string response_string;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
    curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    return response_string;
}

// Function to convert image to ASCII
std::string image_to_ascii(const std::string& image_data) {
    int width, height, channels;
    unsigned char* data = stbi_load_from_memory(
            reinterpret_cast<const stbi_uc*>(image_data.data()),
            image_data.size(),
            &width,
            &height,
            &channels,
            0
    );

    int new_height = static_cast<int>(static_cast<double>(height) / width * new_width * 0.45); // 0.55 is the aspect ratio adjustment

    std::string ascii_image;
    for (int i = 0; i < new_height; ++i) {
        for (int j = 0; j < new_width; ++j) {
            int old_i = i * height / new_height;
            int old_j = j * width / new_width;

            float r = data[(old_i * width + old_j) * channels + 0] / 255.0f;
            float g = data[(old_i * width + old_j) * channels + 1] / 255.0f;
            float b = data[(old_i * width + old_j) * channels + 2] / 255.0f;
            float luminance = (0.2126f * r + 0.7152f * g + 0.0722f * b); // Subtract the luminance from 1 to invert

            ascii_image += map_luminance_to_ascii(luminance);
        }
        ascii_image += '\n';
    }

    stbi_image_free(data);
    return ascii_image;
}

int main(int argc, char** argv) {
    // Picture by Katarzyna Modrzejewska from pexels.com
    std::string url = "https://images.pexels.com/photos/1314550/pexels-photo-1314550.jpeg";

    if (argc > 1) {
        url = argv[1];
    }

    std::string image_data = download_image(url);
    std::cout << image_to_ascii(image_data);
}
