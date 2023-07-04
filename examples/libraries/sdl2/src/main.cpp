#include <SDL2/SDL.h>
#include <SDL2/SDL_timer.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>

#include <memory>
#include <exception>
#include <string>

void render_text(
    SDL_Renderer *renderer,
    int x,
    int y,
    const char *text,
    TTF_Font *font,
    SDL_Rect *rect,
    SDL_Color *color)
{
    SDL_Surface *surface;
    SDL_Texture *texture;

    surface = TTF_RenderText_Solid(font, text, *color);
    texture = SDL_CreateTextureFromSurface(renderer, surface);
    rect->x = x;
    rect->y = y;
    rect->w = surface->w;
    rect->h = surface->h;
    SDL_FreeSurface(surface);
    SDL_RenderCopy(renderer, texture, NULL, rect);
    SDL_DestroyTexture(texture);
}

// This will initilaze the SDL libary and handle closing to cleanup
struct sdl
{
    sdl()
    {
        // returns zero on success else non-zero
        if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
        {
            throw std::runtime_error(std::string("error initializing SDL: ") + std::string(SDL_GetError()));
        }
    }
    ~sdl()
    {
        // We safely uninitialize SDL2, that is, we are
        // taking down the subsystems here before we exit
        // our program.
        SDL_Quit();
    }
};

// Remember, this is a 'C-style' API, we don't have destructors.
// So we'll just make a warpper to call the cleanup functiosn for us
using sdl_window_ptr = std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)>;
sdl_window_ptr make_sdl_window(const char *title, int x, int y, int w, int h, Uint32 flags)
{
    SDL_Window *win = SDL_CreateWindow(title, x, y, w, h, flags);
    return sdl_window_ptr(win, SDL_DestroyWindow);
}

using sdl_render_ptr = std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)>;
sdl_render_ptr make_sdl_render(sdl_window_ptr &win)
{
    // triggers the program that controls
    // your graphics hardware and sets flags
    Uint32 render_flags = SDL_RENDERER_ACCELERATED;

    // creates a renderer to render our images
    SDL_Renderer *rend = SDL_CreateRenderer(win.get(), -1, render_flags);
    return sdl_render_ptr(rend, SDL_DestroyRenderer);
}

using sdl_surface_ptr = std::unique_ptr<SDL_Surface, decltype(&SDL_FreeSurface)>;
sdl_surface_ptr make_sdl_surface(SDL_Surface *surface)
{
    return sdl_surface_ptr(surface, SDL_FreeSurface);
}

using sdl_texture_ptr = std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)>;
sdl_texture_ptr make_sdl_texture(SDL_Texture *texture)
{
    return sdl_texture_ptr(texture, SDL_DestroyTexture);
}

int main(int argc, char *argv[])
{
    const sdl init;

    // creates a window
    auto win = make_sdl_window("GAME", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               1000, 1000, SDL_WINDOW_SHOWN);

    auto rend = make_sdl_render(win);

    // creates a surface to load an image into the main memory
    auto surface = make_sdl_surface(
        IMG_Load("conan-logo.png") // please provide a path for your image
    );

    // loads image to our graphics hardware memory.
    auto tex = make_sdl_texture(
        SDL_CreateTextureFromSurface(rend.get(), surface.get()));

    /* Init TTF. */
    TTF_Init();
    TTF_Font *font = TTF_OpenFont("Roboto-Regular.ttf", 24);
    if (font == NULL)
    {
        fprintf(stderr, "error: font not found\n");
        exit(EXIT_FAILURE);
    }

    // let us control our image position
    // so that we can move it with our keyboard.
    SDL_Rect dest;

    // connects our texture with dest to control position
    SDL_QueryTexture(tex.get(), NULL, NULL, &dest.w, &dest.h);

    // adjust height and width of our image box.
    dest.w /= 6;
    dest.h /= 6;

    // sets initial x-position of object
    dest.x = (1000 - dest.w) / 2;

    // sets initial y-position of object
    dest.y = (1000 - dest.h) / 2;

    // controls animation loop
    int close = 0;

    // speed of box
    int speed = 300;

    // animation loop
    while (!close)
    {
        SDL_Event event;

        // Events management
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {

            case SDL_QUIT:
                // handling of close button
                close = 1;
                break;

            case SDL_KEYDOWN:
                // keyboard API for key pressed
                switch (event.key.keysym.scancode)
                {
                case SDL_SCANCODE_ESCAPE:
                    close = 1;
                    break;
                case SDL_SCANCODE_W:
                case SDL_SCANCODE_UP:
                    dest.y -= speed / 30;
                    break;
                case SDL_SCANCODE_A:
                case SDL_SCANCODE_LEFT:
                    dest.x -= speed / 30;
                    break;
                case SDL_SCANCODE_S:
                case SDL_SCANCODE_DOWN:
                    dest.y += speed / 30;
                    break;
                case SDL_SCANCODE_D:
                case SDL_SCANCODE_RIGHT:
                    dest.x += speed / 30;
                    break;
                default:
                    break;
                }
            }
        }

        // right boundary
        if (dest.x + dest.w > 1000)
            dest.x = 1000 - dest.w;

        // left boundary
        if (dest.x < 0)
            dest.x = 0;

        // bottom boundary
        if (dest.y + dest.h > 1000)
            dest.y = 1000 - dest.h;

        // upper boundary
        if (dest.y < 0)
            dest.y = 0;

        // clears the screen
        SDL_RenderClear(rend.get());
        SDL_RenderCopy(rend.get(), tex.get(), NULL, &dest);

        SDL_Rect text_rect;
        // The color for the text we will be displaying
        SDL_Color white = {255, 255, 255, 0};
        // so we can have nice text, two lines one above the next
        render_text(rend.get(), 10, 10, "Hello World!", font, &text_rect, &white);
        render_text(rend.get(), 10, text_rect.y + text_rect.h, "Conan demo by JFrog", font, &text_rect, &white);

        // triggers the double buffers
        // for multiple rendering
        SDL_RenderPresent(rend.get());

        // calculates to 60 fps
        SDL_Delay(1000 / 60);
    }

    // close font handle
    TTF_CloseFont(font);

    // close TTF
    TTF_Quit();
}
