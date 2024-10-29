#include "str_printer/str_printer.h"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  str_printer("Hi there! I am using fmt library fetched with Conan C/C++ Package Manager");
  return 0;
}
