#include <cpr/cpr.h>
#include <fmt/core.h>
#include <rapidjson/document.h>


int main() {
    cpr::Response r = cpr::Get(cpr::Url{"https://api.github.com/repos/conan-io/conan"});
    if (r.status_code != 200) {
        fmt::print("Failed to fetch data: {}\n", r.status_code);
        return 1;
    }

    rapidjson::Document doc;
    if (doc.Parse(r.text.c_str()).HasParseError()) {
        fmt::print("Error parsing JSON response\n");
        return 1;
    }

    if (doc.HasMember("full_name") && doc["full_name"].IsString()) {
        fmt::print("Repository: {}\n", doc["full_name"].GetString());
    }
    if (doc.HasMember("stargazers_count") && doc["stargazers_count"].IsInt()) {
        fmt::print("Stars: {}\n", doc["stargazers_count"].GetInt());
    }
    if (doc.HasMember("forks_count") && doc["forks_count"].IsInt()) {
        fmt::print("Forks: {}\n", doc["forks_count"].GetInt());
    }

    return 0;
}
