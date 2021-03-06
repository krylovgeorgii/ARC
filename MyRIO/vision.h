/* vision.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 * 			https://www.facebook.com/krylov.georgii
 *
 */

#ifndef _VISION_H_
#define _VISION_H_ "1.3"

#include "log.h"

#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <unistd.h>
#include <math.h>

//#include <jpeglib.h>
//#include <vector>
#include <memory>

namespace robot {
    namespace vision {
        typedef struct {
            int32_t FOCUS_ABSOLUTE = 0;
            int32_t EXPOSURE = 20;
            int32_t EXPOSURE_TYPE = V4L2_EXPOSURE_APERTURE_PRIORITY;
            int32_t BRIGHTNESS = 20;
            int32_t CONTRAST = 200;
        } settings_t;

        class Camera {
        private:
            typedef struct {
                uint8_t * start;
                size_t length;
            } buffer_t;

            typedef struct {
                int fd;
                uint32_t width;
                uint32_t height;
                size_t buffer_count;
                buffer_t * buffers;
                buffer_t head;
            } camera_t;


            int xioctl(const int fd, const int request, const void * arg) const;
            bool camera_open();
            bool camera_init();
            bool camera_start();

            void camera_stop();
            void camera_finish();
            void camera_close();

            int camera_capture();

        public:
           settings_t settings;
           int camera_frame();

           Camera(const int w, const int h,
                    std::string device = "/dev/video1", settings_t newSettings = settings_t(), timeval t = timeval { 1, 0 })
                    : W(w), H(h), DEVICE(device) {
                timeout = t;
                settings = newSettings;

                if (!camera_open()) {
                    message("error open camera");
                    return;
                }

                if (!camera_init()) {
                    message("error init camera");
                    return;
                }

                if (!camera_start()) {
                    message("error start camera");
                    return;
                }

                usleep(1 * 1000000);

                for(int i = 0; i < 1000; ++i) {
                    if(camera_frame()) {
                         break;
                    }
                }
           }

           ~Camera();

           int getW() {
             return W;
           }

           int getH() {
             return H;
           }

        private:
            std::string DEVICE;
            timeval timeout;

        protected:
            int W, H;
            camera_t * camera = nullptr;

        }; /* END class camera */

        template <class T>
        struct CvPoint {
          T x;
          T y;
        };

        class CV_UV : public Camera {
        private:
          uint8_t blueAngle_N = 0;
          uint8_t orangeAngle_N = 1;
          uint8_t greenS_N = 2;
          uint8_t redS_N = 3;
          uint8_t yellowSquare_N = 4;
          uint8_t blueBeam_N = 5;

          CvPoint<int8_t> blueAngle { 60, -40 };
          CvPoint<int8_t> orangeAngle { -40, 70 };
          CvPoint<int8_t> greenS { -11, -12 };
          CvPoint<int8_t> redS { -20, 53 };
          CvPoint<int8_t> yellowSquare { -83, 15 };
          CvPoint<int8_t> blueBeam { 55, -47 };

          int blueAngle_Y = 32;
          int blueBeam_Y = 0;

          int orangeAngle_Y = 0;
          int redS_Y = 0;

          static const uint8_t NUM_KIND_TARGETS = 6;

          CvPoint<int8_t> targetUV[6]  { blueAngle, orangeAngle, greenS, redS, yellowSquare, blueBeam };
          int Y_MAX_DIFF = 200;
          const double ANGL = 0.94;
          const int Y_MAX = 500;//80;
          const int Y_MIN = 0;

          bool DRAW;
          int8_t * uv = nullptr;

          int Y_BEGIN = 0;
          int Y_END = H;
          int X_BEGIN = 0;
          int X_END = W / 2;

          uint8_t ** color = nullptr;
          int ** comp = nullptr;

          int X_CLB_CLR = X_BEGIN + (X_END - X_BEGIN) / 2;
          int Y_CLB_CLR = Y_BEGIN + (Y_END - Y_BEGIN) / 2;
          int NUM_PIX = (X_END - X_BEGIN) * (Y_END - Y_BEGIN);

          CvPoint<int> * Q = nullptr;

          int * numINComp = nullptr;
          int * colorOfComp = nullptr;
          uint8_t * rgb = nullptr;

          uint8_t numBiggestColor;

        public:
            CV_UV(const int w, const int h,
                    std::string device = "/dev/video1", bool draw = true, settings_t newSettings = settings_t(), timeval t = timeval { 1, 0 })
                    : Camera(w, h, device, newSettings, t), DRAW(draw) {

                  Y_BEGIN = 0;
                  Y_END = H;
                  X_BEGIN = 0;
                  X_END = W / 2;

                  X_CLB_CLR = X_BEGIN + (X_END - X_BEGIN) / 2;
                  Y_CLB_CLR = Y_BEGIN + (Y_END - Y_BEGIN) / 2;
                  NUM_PIX = (X_END - X_BEGIN) * (Y_END - Y_BEGIN);

                  color = new uint8_t * [X_END - X_BEGIN];
                  if (color == nullptr) {
                      message("can`t create massive");
                  } else {
                    for (int i = 0; i <= Y_END - Y_BEGIN; ++i) {
                      color[i] = new uint8_t[Y_END - Y_BEGIN + 1];
                    }
                  }

                  comp = new int * [X_END - X_BEGIN];
                  if (comp == nullptr) {
                      message("can`t create massive");
                  } else {
                    for (int i = 0; i <= Y_END - Y_BEGIN; ++i) {
                      comp[i] = new int[Y_END - Y_BEGIN + 1];
                    }
                  }

                  Q = new CvPoint<int>[NUM_PIX];
                  if (Q == nullptr) {
                      message("can`t create massive");
                  }

                  numINComp = new int[NUM_PIX];
                  if (numINComp == nullptr) {
                      message("can`t create massive");
                  }

                  colorOfComp = new int[NUM_PIX];
                  if (colorOfComp == nullptr) {
                      message("can`t create massive");
                  }

                  if (DRAW) {
                    rgb = static_cast<uint8_t *>(calloc(W * H * 3, sizeof(uint8_t)));
                    if (rgb == nullptr) {
                        message("can`t create massive");
                    }
                  }
            }

            ~CV_UV() {
                free(uv);

                for (int i = 0; i <  X_END - X_BEGIN; ++i) {
                  //delete [] &(color[i]);
                  //delete [] comp;
                }

              //  delete [] color;
              //  delete [] comp;
            }

            uint8_t * getRGB () {
              return rgb;
            }

            uint8_t getNumBiggestColor() {
              return numBiggestColor;
            }

            CvPoint<int> handle_frame();
            void yuyv2uv();
            void calibUV( const bool onlyCalibOrOnlyDisplay, const int RAD_CLB_CLR = 10);
            void find_color();
            CvPoint<int> find_components();
            void uv2yuyv();
            uint8_t * yuyv2rgb();
            int minmax(int min, int v, int max);

        private:
            inline int8_t getColorUV(int8_t * img, int x, int y, uint8_t channel) {
                //return img[y * W + x * 2 + channel];
                return camera->head.start[(y * W + x * 2) * 2 + 1 + (channel ? 2 : 0)] - 128;
            }

            inline void setColorUV(int8_t * img, int x, int y, int channel, int8_t color) {
                //img[y * W + x * 2 + channel] = color;
                camera->head.start[(y * W + x * 2) * 2 + 1 + (channel ? 2 : 0)]  = color + 128;
            }

            inline int getColorY(int x, int y) {
                return camera->head.start[(y * W + x * 2) * 2] + camera->head.start[(y * W + x * 2) * 2 + 2];
            }

            inline void setColorY(int x, int y, uint8_t value) {
                camera->head.start[(y * W + x * 2) * 2] = value;
                camera->head.start[(y * W + x * 2) * 2 + 2] = value;
            }

            /////////////////

        };
    } /* END namespace vision */
} /* END namespace robot */

#endif /* _VISION_H_ */
