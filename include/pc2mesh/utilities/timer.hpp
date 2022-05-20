#pragma once
#include <chrono>
#include <iostream>
#include <string_view>
namespace pc2mesh::utilities {
    template <typename Body>
    decltype(std::declval<Body>()()) time(std::string_view name, Body body) {
        using namespace std::chrono;
        auto start = high_resolution_clock::now();
        auto val = body();
        auto end = high_resolution_clock::now();
        std::cout << name << " finished in " << duration_cast<milliseconds>(end - start).count() << "ms" << std::endl;
        return val;
    }
}