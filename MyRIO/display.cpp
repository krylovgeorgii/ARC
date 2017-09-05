/* display.cpp
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 * 			https://www.facebook.com/krylov.georgii
 *
 */

#ifndef _VISION_CPP_
#define _VISION_CPP_ "1.1"

#include "display.h"

namespace robot {
  namespace vision {
    void Display::displayWindow(uint8_t * rgb) {
        //SDL_RenderClear(renderer);

        for (int x = 0; x < W; ++x) {
            for (int y = 0; y < H; ++y) {
                putPixelRGB(renderer, x, y, rgb[(y * W + x) * 3 ], rgb[(y * W + x) * 3 + 1], rgb[(y * W + x) * 3 + 2]);
            }
        }

        //SDL_RenderClear(renderer);
        SDL_RenderPresent(renderer);
        //SDL_Delay( 5 );
        //SDL_Quit();
    }
  } /* END namespace vision */
} /* END namespace robot */

#endif /* _DISPLAY_CPP_ */
