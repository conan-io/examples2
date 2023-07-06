#include <SDL2/SDL.h>
#include <SDL2/SDL_timer.h>

int main(int argc, char *argv[])
{
    // returns zero on success else non-zero
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
        printf("error initializing SDL: %s\n", SDL_GetError());
    }
    SDL_Window* win = SDL_CreateWindow("GAME", // creates a window
                                    SDL_WINDOWPOS_CENTERED,
                                    SDL_WINDOWPOS_CENTERED,
                                    1000, 1000, 0);

    // triggers the program that controls
    // your graphics hardware and sets flags
    Uint32 render_flags = SDL_RENDERER_ACCELERATED;

    // creates a renderer to render our images
    SDL_Renderer* rend = SDL_CreateRenderer(win, -1, render_flags);

    ///
    /// Section 2: SDL image loader
    ///

    ///
    /// Section 4: SDL ttf and rendering text
    ///

    ///
    /// Section 3: Game Loop and Basic Controlls
    ///


    // We add a delay in order to see that our window
    // has successfully popped up.
    SDL_Delay(3000);

    ///
    /// Section 5: Freeing resources
    ///


    // We destroy our window. We are passing in the pointer
    // that points to the memory allocated by the 
    // 'SDL_CreateWindow' function. Remember, this is
    // a 'C-style' API, we don't have destructors.
    SDL_DestroyWindow(win);
    
    // We safely uninitialize SDL2, that is, we are
    // taking down the subsystems here before we exit
    // our program.
    SDL_Quit();
}
