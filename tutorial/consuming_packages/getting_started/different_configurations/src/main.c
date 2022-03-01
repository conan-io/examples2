#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <zlib.h>

int main(void) {
    #ifdef NDEBUG
    printf("Release configuration!\n");
    #else
    printf("Debug configuration!\n");
    #endif
    printf("ZLIB VERSION: %s\n", zlibVersion());
    return EXIT_SUCCESS;
}
