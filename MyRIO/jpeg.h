/* jpeg.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *      https://www.facebook.com/krylov.georgii
 *
 */

#ifndef _JPEG_H_
#define _JPEG_H_ "1.0"

#include <jpeglib.h>

namespace robot {
  namespace vision {
    void jpeg(FILE * dest, uint8_t * rgb, uint32_t width, uint32_t height, int quality);
  } /* END namespace vision */
} /* END namespace robot */

#endif /* _JPEG_H_ */
