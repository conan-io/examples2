#include <cstdlib>
#include <fstream>
#include <iostream>

#include "Poco/MD5Engine.h"
#include "Poco/DigestStream.h"

int main(void) {
    Poco::MD5Engine md5;
    Poco::DigestOutputStream dos(md5);
    const std::string quote ("To be a Cimmerian warrior, you must have both cunning and balance as well as speed and strength.");

    dos << quote << std::endl;
    dos.flush();

    const Poco::DigestEngine::Digest& digest = md5.digest();

    std::cout << "[" << Poco::DigestEngine::digestToHex(digest) << "] " << quote << std::endl;

    return EXIT_SUCCESS;
}
