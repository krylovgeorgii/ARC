/* display.cpp
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 */

#ifndef _VISION_CPP_
#define _VISION_CPP_ "1.0"

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

    void jpeg(FILE * dest, uint8_t * rgb, uint32_t width, uint32_t height, int quality) {
          JSAMPARRAY image;
          image = static_cast<JSAMPARRAY>(calloc(height, sizeof (JSAMPROW)));
          for (size_t i = 0; i < height; ++i) {
            image[i] = static_cast<JSAMPROW>(calloc(width * 3, sizeof (JSAMPLE)));
            for (size_t j = 0; j < width; j++) {
              image[i][j * 3 + 0] = rgb[(i * width + j) * 3 + 0];
              image[i][j * 3 + 1] = rgb[(i * width + j) * 3 + 1];
              image[i][j * 3 + 2] = rgb[(i * width + j) * 3 + 2];
            }
         }

      struct jpeg_compress_struct compress;
      struct jpeg_error_mgr error;
      compress.err = jpeg_std_error(&error);
      jpeg_create_compress(&compress);
      jpeg_stdio_dest(&compress, dest);

      compress.image_width = width;
      compress.image_height = height;
      compress.input_components = 3;
      compress.in_color_space = JCS_RGB;
      jpeg_set_defaults(&compress);
      jpeg_set_quality(&compress, quality, TRUE);
      jpeg_start_compress(&compress, TRUE);
      jpeg_write_scanlines(&compress, image, height);
      jpeg_finish_compress(&compress);
      jpeg_destroy_compress(&compress);

      for (size_t i = 0; i < height; ++i) {
        free(image[i]);
      }

      free(image);
    }
  } /* END namespace vision */
} /* END namespace robot */

#endif /* _DISPLAY_CPP_ */
