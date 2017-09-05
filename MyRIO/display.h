/* display.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 * 			https://www.facebook.com/krylov.georgii
 *
 */

#ifndef _DISPLAY_H_
#define _DISPLAY_H_ "1.1"

#include <SDL2/SDL.h>

#include "log.h"

namespace robot {
    namespace vision {
      class Display {
      private:
        SDL_Window * window = nullptr;
        SDL_Renderer * renderer = nullptr;
        int winX, winY;
        int W, H;

      public:
        Display(int w, int h, int setWinX = 100, int setWinY = 200) :
            W(w), H(h), winX(setWinX), winY(setWinY) {
          if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
            message(" Failed to initialize SDL : " << SDL_GetError());
            return;
          }

          window = SDL_CreateWindow("Server", winX, winY, W, H, 0);

          if (window == nullptr) {
            message("Failed to create window : " << SDL_GetError());
            return;
          }

          renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

          if (renderer == nullptr) {
              message("Failed to create renderer : " << SDL_GetError());
              return;
          }

          SDL_RenderSetLogicalSize(renderer, W, H);
        }

        ~Display() {
          SDL_DestroyWindow(window);
        }

        void displayWindow(uint8_t * rgb);

      private:

        inline void putPixelRGB(SDL_Renderer * renderer, int x, int y, uint8_t r, uint8_t g, uint8_t b){
            SDL_SetRenderDrawColor(renderer, r, g, b, 255);
            SDL_RenderDrawPoint(renderer, x, y);
        }
      };
  } /* END namespace vision */
} /* END namespace robot */

#endif /* _DISPLAY_H_ */
