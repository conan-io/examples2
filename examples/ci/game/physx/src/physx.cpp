#include <iostream>
#include "physx.h"

void physx(){
    #ifdef NDEBUG
    std::cout << "physx/" << PKG_VERSION << ": Checking if things collide (Release)!\n";
    #else
    std::cout << "physx/" << PKG_VERSION << ": Checking if things collide (Debug)!\n";
    #endif

    // ARCHITECTURES
    #ifdef _M_X64
    std::cout << "  physx/" << PKG_VERSION << ": _M_X64 defined\n";
    #endif

    #ifdef _M_IX86
    std::cout << "  physx/" << PKG_VERSION << ": _M_IX86 defined\n";
    #endif

    #if __i386__
    std::cout << "  physx/" << PKG_VERSION << ": __i386__ defined\n";
    #endif

    #if __x86_64__
    std::cout << "  physx/" << PKG_VERSION << ": __x86_64__ defined\n";
    #endif

    // Libstdc++
    #if defined _GLIBCXX_USE_CXX11_ABI
    std::cout << "  physx/" << PKG_VERSION << ": _GLIBCXX_USE_CXX11_ABI "<< _GLIBCXX_USE_CXX11_ABI << "\n";
    #endif

    // COMPILER VERSIONS
    #if _MSC_VER
    std::cout << "  physx/" << PKG_VERSION << ": _MSC_VER" << _MSC_VER<< "\n";
    #endif

    #if _MSVC_LANG
    std::cout << "  physx/" << PKG_VERSION << ": _MSVC_LANG" << _MSVC_LANG<< "\n";
    #endif

    #if __cplusplus
    std::cout << "  physx/" << PKG_VERSION << ": __cplusplus" << __cplusplus<< "\n";
    #endif

    #if __INTEL_COMPILER
    std::cout << "  physx/" << PKG_VERSION << ": __INTEL_COMPILER" << __INTEL_COMPILER<< "\n";
    #endif

    #if __GNUC__
    std::cout << "  physx/" << PKG_VERSION << ": __GNUC__" << __GNUC__<< "\n";
    #endif

    #if __GNUC_MINOR__
    std::cout << "  physx/" << PKG_VERSION << ": __GNUC_MINOR__" << __GNUC_MINOR__<< "\n";
    #endif

    #if __clang_major__
    std::cout << "  physx/" << PKG_VERSION << ": __clang_major__" << __clang_major__<< "\n";
    #endif

    #if __clang_minor__
    std::cout << "  physx/" << PKG_VERSION << ": __clang_minor__" << __clang_minor__<< "\n";
    #endif

    #if __apple_build_version__
    std::cout << "  physx/" << PKG_VERSION << ": __apple_build_version__" << __apple_build_version__<< "\n";
    #endif

    // SUBSYSTEMS

    #if __MSYS__
    std::cout << "  physx/" << PKG_VERSION << ": __MSYS__" << __MSYS__<< "\n";
    #endif

    #if __MINGW32__
    std::cout << "  physx/" << PKG_VERSION << ": __MINGW32__" << __MINGW32__<< "\n";
    #endif

    #if __MINGW64__
    std::cout << "  physx/" << PKG_VERSION << ": __MINGW64__" << __MINGW64__<< "\n";
    #endif

    #if __CYGWIN__
    std::cout << "  physx/" << PKG_VERSION << ": __CYGWIN__" << __CYGWIN__<< "\n";
    #endif
}
