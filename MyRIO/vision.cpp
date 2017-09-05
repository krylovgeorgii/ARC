/* vision.cpp
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
#define _VISION_CPP_ "1.3"

#include "vision.h"

namespace robot {
    namespace vision {
        bool Camera::camera_open() {
            int fd = open(&(DEVICE[0]), O_RDWR | O_NONBLOCK, 0); //&(DEVICE[0])

            if (fd == -1) {
              message("error open camera");
              return false;
            }

            camera = new camera_t;

            if (camera == nullptr) {
                message("error create camera_t");
                return false;
            }

            camera->fd = fd;
            camera->width = W;
            camera->height = H;
            camera->buffer_count = 0;
            camera->buffers = nullptr;
            camera->head.length = 0;
            camera->head.start = nullptr;
            return true;
        }

         int Camera::xioctl(const int fd, const int request, const void * arg) const {
            for (int i = 0; i < 100; ++i) {
                int r = ioctl(fd, request, arg);

                if (r != -1 || errno != EINTR) {
                    return r;
                }
            }

            return -1;
        }

        bool Camera::camera_init() {
            struct v4l2_capability cap;
            if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) == -1) {
                message("VIDIOC_QUERYCAP");
                return false;
            }

            if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                message("no capture");
                return false;
            }

            if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                message("no streaming");
                return false;
            }

            struct v4l2_cropcap cropcap;
            memset(&cropcap, 0, sizeof cropcap);
            cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (xioctl(camera->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
                struct v4l2_crop crop;
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect;

                if (xioctl(camera->fd, VIDIOC_S_CROP, &crop) == -1) {
                   //cropping not supported
                }
            }

            v4l2_control ctrl;

            /*ctrl.id = V4L2_CID_FOCUS_AUTO;
            ctrl.value = 0;
            if (-1 == xioctl(camera->fd, VIDIOC_S_CTRL, &ctrl)) {
                message("can't set focus");
            }*/

            ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
            ctrl.value = 5;
            if (-1 == ioctl(camera->fd, VIDIOC_S_CTRL, &ctrl)) {
                message("can't set focus");
            }

            /*ctrl.id = V4L2_CID_EXPOSURE_AUTO;
            ctrl.value = settings.EXPOSURE_TYPE;
            if (-1 == ioctl(camera->fd, VIDIOC_S_CTRL, &ctrl)) {
                message("can't set cam settings");
            }*/

            if (settings.EXPOSURE_TYPE == V4L2_EXPOSURE_MANUAL) {
                ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
                ctrl.value = settings.EXPOSURE;
                if (-1 == ioctl(camera->fd, VIDIOC_S_CTRL, &ctrl)) {
                    message("can't set cam settings");
                }
            }

            ctrl.id = V4L2_CID_BRIGHTNESS;
            ctrl.value = settings.BRIGHTNESS;
            if (-1 == ioctl(camera->fd, VIDIOC_S_CTRL, &ctrl)) {
                message("can't set cam settings");
            }

            ctrl.id = V4L2_CID_CONTRAST;
            ctrl.value = settings.CONTRAST;
            if (-1 == ioctl(camera->fd, VIDIOC_S_CTRL, &ctrl)) {
                message("can't set cam settings");
            }

            struct v4l2_format format;
            memset(&format, 0, sizeof format);
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.width = camera->width;
            format.fmt.pix.height = camera->height;
            format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            format.fmt.pix.field = V4L2_FIELD_NONE;
            if (xioctl(camera->fd, VIDIOC_S_FMT, &format) == -1) {
                message("VIDIOC_S_FMT");
                return false;
            }

            struct v4l2_requestbuffers req;
            memset(&req, 0, sizeof req);
            req.count = 4;
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            req.memory = V4L2_MEMORY_MMAP;
            if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) == -1) {
                message("VIDIOC_REQBUFS");
                return false;
            }
            camera->buffer_count = req.count;
            camera->buffers = static_cast<buffer_t *>(calloc(req.count, sizeof(buffer_t)));

            size_t buf_max = 0;
            for (size_t i = 0; i < camera->buffer_count; ++i) {
                struct v4l2_buffer buf;
                memset(&buf, 0, sizeof buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
                if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) == -1) {
                    message("VIDIOC_QUERYBUF");
                    return false;
                }
                if (buf.length > buf_max) {
                    buf_max = buf.length;
                }
                camera->buffers[i].length = buf.length;
                camera->buffers[i].start = static_cast<uint8_t *>(mmap(NULL, buf.length,
                                PROT_READ | PROT_WRITE, MAP_SHARED, camera->fd, buf.m.offset));
                if (camera->buffers[i].start == MAP_FAILED) {
                    message("mmap");
                    return false;
                }
            }

            camera->head.start = static_cast<uint8_t *>(malloc(buf_max));
            return true;
        }

        bool Camera::camera_start() {
            for (size_t i = 0; i < camera->buffer_count; ++i) {
                struct v4l2_buffer buf;
                memset(&buf, 0, sizeof buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) {
                    message("VIDIOC_QBUF");
                    return false;
                }
            }

            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (xioctl(camera->fd, VIDIOC_STREAMON, &type) == -1) {
                message("VIDIOC_STREAMON");
                return false;
            }

            return true;
        }

        void Camera::camera_stop() {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (xioctl(camera->fd, VIDIOC_STREAMOFF, &type) == -1) {
                message("error VIDIOC_STREAMOFF");
            }
        }

        void Camera::camera_finish() {
            for (size_t i = 0; i < camera->buffer_count; ++i) {
                munmap(camera->buffers[i].start, camera->buffers[i].length);
            }

            free(camera->buffers);
            camera->buffer_count = 0;
            camera->buffers = nullptr;
            free(camera->head.start);
            camera->head.length = 0;
            camera->head.start = nullptr;
        }

        void Camera::camera_close() {
            if (close(camera->fd) == -1) {
                message("error close");
            }

            delete camera;
        }

       Camera::~Camera() {
            camera_stop();
            camera_finish();
            camera_close();
       }

        int Camera::camera_capture() {
         v4l2_buffer buf;
         memset(&buf, 0, sizeof buf);
         buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         buf.memory = V4L2_MEMORY_MMAP;

         if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) == -1) {
             return false;
         }

         memcpy(camera->head.start, camera->buffers[buf.index].start, buf.bytesused);
         camera->head.length = buf.bytesused;

         if (xioctl(camera->fd, VIDIOC_QBUF, &buf) == -1) {
             return false;
         }

         return true;
       }

        int Camera::camera_frame() {
         fd_set fds;
         FD_ZERO(&fds);
         FD_SET(camera->fd, &fds);

         int r = 0;
        // do {
             r = select(camera->fd + 1, &fds, 0, 0, &timeout);
             if (r == -1) {
                message("select is -1");
                return false;
              }

              if (r == 0) {
                message("select is 0");
                usleep(0.01 * 1000000);
                //return false;
              }
        //  } while (r == 0);

         return camera_capture();
       }

       void CV_UV::yuyv2uv() {
         uv = static_cast<int8_t *>(calloc(W * H, sizeof (int8_t)));
         for (size_t i = 0; i < H; ++i) {
           for (size_t j = 0; j < W; j += 2) {
             size_t index = i * W + j;
             uv[index + 0] = camera->head.start[index * 2 + 1] - 128;
             uv[index + 1] = camera->head.start[index * 2 + 3] - 128;
           }
         }
       }

       void CV_UV::calibUV(const bool onlyCalibOrOnlyDisplay, const int RAD_CLB_CLR) {
          if (onlyCalibOrOnlyDisplay == 1) {
            for(int x = -RAD_CLB_CLR; x < RAD_CLB_CLR; ++x) {
              for(int y = -RAD_CLB_CLR; y < RAD_CLB_CLR; ++y) {
                if (DRAW) {
                  setColorUV(uv, x + X_CLB_CLR, y + Y_CLB_CLR, 0, 128);
                  setColorUV(uv, x + X_CLB_CLR, y + Y_CLB_CLR, 1, 128);
                }
              }
            }

            return;
          }

           double u = 0, v = 0, Y = 0;
           int numColors = 0;



           for(int x = -RAD_CLB_CLR; x < RAD_CLB_CLR; ++x) {
              for(int y = -RAD_CLB_CLR; y < RAD_CLB_CLR; ++y) {

                int y_t = getColorY(x + X_CLB_CLR, y + Y_CLB_CLR);

                /*if (y_t < Y_MIN || y_t > Y_MAX) {
                  continue;
                }*/

                int u_t = getColorUV(uv, x + X_CLB_CLR, y + Y_CLB_CLR, 0);
                int v_t = getColorUV(uv, x + X_CLB_CLR, y + Y_CLB_CLR, 1);


                u += u_t;
                v += v_t;
                Y += y_t;

                ++numColors;
              }
           }

           if (numColors < RAD_CLB_CLR * 2) {
             message("numColors less than half: " << numColors << " in " << RAD_CLB_CLR * 4);
           }

           if (numColors == 0) {
             return;
           }

           u /= numColors;
           v /= numColors;
           Y /= numColors;

           message("\nU = " << u << "\nV = " << v << "\ny = " << Y);
       }

        void CV_UV::uv2yuyv() {
         for (size_t i = 0; i < H; ++i) {
           for (size_t j = 0; j < W; j += 2) {
             size_t index = i * W + j;
             camera->head.start[index * 2 + 1] = uv[index + 0] + 128;
             camera->head.start[index * 2 + 3] = uv[index + 1] + 128;
           }
         }
       }

      void CV_UV::find_color() {
        double cosVal[NUM_KIND_TARGETS];
        double t_val[NUM_KIND_TARGETS];

        for (int i = 0; i < NUM_KIND_TARGETS; ++i) {
           t_val[i] = sqrt(targetUV[i].y * targetUV[i].y + targetUV[i].x * targetUV[i].x);
        }

        for (int y = Y_BEGIN; y < Y_END; ++y) {
          for (int x = X_BEGIN; x < X_END; ++x) {
            int8_t t_u = getColorUV(uv, x, y, 0);
            int8_t t_v = getColorUV(uv, x, y, 1);

            color[x - X_BEGIN][y - Y_BEGIN] = 0;

            if (getColorY(x, y) < Y_MIN || getColorY(x, y) > Y_MAX || (abs(getColorUV(uv, x, y, 0)) < 5 && abs(getColorUV(uv, x, y, 1)) < 5)) {
              if (DRAW) {
                setColorUV(uv, x, y, 0, -100);
                setColorUV(uv, x, y, 1, 128);
                setColorY(x, y, 255);
              }

              continue;
            }

            for (uint8_t i = 0; i < NUM_KIND_TARGETS; ++i) {
              cosVal[i] = (t_u * targetUV[i].x + t_v * targetUV[i].y) / (t_val[i] * sqrt(t_u * t_u + t_v * t_v));
            }

            double maxCosVal = cosVal[0];
            uint8_t numMaxCos = 0;

            for (uint8_t i = 1; i < NUM_KIND_TARGETS; ++i) {
              if (cosVal[i] > maxCosVal) {
                maxCosVal = cosVal[i];
                numMaxCos = i;
              }
            }

            if (maxCosVal <= ANGL) {
              continue;
            }

            if (numMaxCos == blueAngle_N || numMaxCos == blueBeam_N) {
              if ((static_cast<int>(blueAngle.x) - t_u) * (static_cast<int>(blueAngle.x) - t_u) +
                  (static_cast<int>(blueAngle.y) - t_v) * (static_cast<int>(blueAngle.y) - t_v) <
                      (static_cast<int>(blueBeam.x) - t_u) * (static_cast<int>(blueBeam.x) - t_u) +
                      (static_cast<int>(blueBeam.y) - t_v) * (static_cast<int>(blueBeam.y) - t_v)) {
                numMaxCos = blueAngle_N;
              } else {
                numMaxCos = blueBeam_N;
              }
            }

            if (numMaxCos == orangeAngle_N || numMaxCos == redS_N) {
              if ((static_cast<int>(orangeAngle.x) - t_u) * (static_cast<int>(orangeAngle.x) - t_u) +
                  (static_cast<int>(orangeAngle.y) - t_v) * (static_cast<int>(orangeAngle.y) - t_v) <
                      (static_cast<int>(redS.x) - t_u) * (static_cast<int>(redS.x) - t_u) +
                      (static_cast<int>(redS.y) - t_v) * (static_cast<int>(redS.y) - t_v)) {
                numMaxCos = orangeAngle_N;
              } else {
                numMaxCos = redS_N;
              }
            }

            color[x - X_BEGIN][y - Y_BEGIN] = numMaxCos + 1;

            if (DRAW) {
              setColorUV(uv, x, y, 0, -targetUV[numMaxCos].x);
              setColorUV(uv, x, y, 1, targetUV[numMaxCos].y);
            }
          }
        }
      }

        CvPoint<int> CV_UV::find_components() {
           for (int i = 0; i < NUM_PIX; ++i) {
              numINComp[i] = 0;
              colorOfComp[i] = 0;
           }

           int head = 0, tail = 0;
           int n = 0;

           for (int y = Y_BEGIN; y < Y_END; ++y) {
               for (int x = X_BEGIN; x < X_END; ++x) {
                 comp[x - X_BEGIN][y - Y_BEGIN] = -1;
              }
           }

           for (int y = Y_BEGIN; y < Y_END; ++y) {
               for (int x = X_BEGIN; x < X_END; ++x) {
                   if (comp[x - X_BEGIN][y - Y_BEGIN] != -1) {
                     continue;
                   }

                   if (color[x - X_BEGIN][y - Y_BEGIN] == 0) {
                     continue;
                   }

                   comp[x - X_BEGIN][y - Y_BEGIN] = n;

                   ++numINComp[n];
                   colorOfComp[n] = color[x - X_BEGIN][y - Y_BEGIN];

                   head = tail = 0;
                   Q[head++] = CvPoint<int> { x, y };


                   while (head > tail) {
                      CvPoint<int> cur = Q[tail++];

                      for (int dx = -1; dx <= 1; ++dx) {
                          for (int dy = -1; dy <= 1; ++dy) {
                              if (cur.x + dx < X_BEGIN || cur.x + dx >= X_END || cur.y + dy >= Y_END || cur.y + dy < Y_BEGIN) {
                                continue;
                              }

                              if (comp[cur.x + dx - X_BEGIN][cur.y + dy - Y_BEGIN] != -1) {
                                continue;
                              }

                              if (color[cur.x + dx - X_BEGIN][cur.y + dy - Y_BEGIN] != color[cur.x - X_BEGIN][cur.y - Y_BEGIN]) {
                                  continue;
                              }

                              if(abs(getColorY(cur.x, cur.y) - getColorY(cur.x + dx, cur.y + dy)) > Y_MAX_DIFF) {
                                  continue;
                              }

                              comp[cur.x + dx - X_BEGIN][cur.y + dy - Y_BEGIN] = n;
                              ++numINComp[n];

                              Q[head++] = CvPoint<int> { cur.x + dx , cur.y + dy };
                          }
                      }
                   }
                   ++n;
               }
           }

           //message("n = " << n);

           int maxComp[NUM_KIND_TARGETS];
           int maxValue[NUM_KIND_TARGETS];
           CvPoint<int> c_mass[NUM_KIND_TARGETS];

           for (uint8_t i = 0; i < NUM_KIND_TARGETS; ++i) {
              maxComp[i] = -1;
              maxValue[i] = 1;
              c_mass[i] = CvPoint<int> { 0, 0 };

           }

           //std::unique_ptr<CvPoint<int>> c_mass(new CvPoint<int>[targetUV.size()](CvPoint<int> { 0, 0 }));
          for (uint8_t j = 0; j < NUM_KIND_TARGETS; ++j) {
              for (int i = 0; i < n; ++i) {
                  if (colorOfComp[i] != j + 1) {
                    continue;
                  }

                  if (maxValue[j] < numINComp[i]) {
                      maxValue[j]  = numINComp[i];
                      maxComp[j] = i;
                  }
              }
            }

            for (int y = Y_BEGIN; y < Y_END; ++y) {
                for (int x = X_BEGIN; x < X_END; ++x) {
                   for (uint8_t i = 0; i < NUM_KIND_TARGETS; ++i) {
                       if (comp[x - X_BEGIN][y - Y_BEGIN] == maxComp[i]) {
                            if (DRAW) {
                              setColorUV(uv, x, y, 0, -targetUV[i].x);
                              setColorUV(uv, x, y, 1, -targetUV[i].y);
                            }

                            c_mass[i].x += x;
                            c_mass[i].y += y;
                       }
                    }
                }
            }

            for (uint8_t i = 0; i < NUM_KIND_TARGETS; ++i) {
                if (maxValue[i] != 0) {
                  c_mass[i].x /=  maxValue[i];
                  c_mass[i].y /=  maxValue[i];
                }

                //message("\nc_mass.x = " << c_mass[i].x
                //     << "\nc_mass.y = " << c_mass[i].y);

                //message("Y = " << getColorY(c_mass[i].x, c_mass[i].y));

                if (DRAW) {
                  const int RADIUS_CENTR = 5;

                  for (int dx = -RADIUS_CENTR / 2; dx <= RADIUS_CENTR / 2; ++dx) {
                      for (int dy = -RADIUS_CENTR; dy <= RADIUS_CENTR; ++dy) {
                          if (c_mass[i].x + dx < X_BEGIN || c_mass[i].x + dx >= X_END ||
                            c_mass[i].y + dy >= Y_END || c_mass[i].y + dy < Y_BEGIN) {
                              continue;
                          }

                          setColorUV(uv, c_mass[i].x + dx, c_mass[i].y + dy, 0, 100);
                          setColorUV(uv, c_mass[i].x + dx, c_mass[i].y + dy, 1, 100);
                      }
                   }
                }
            }

            int maxCompOfAll = 0;
            int maxValueOfAllComp = maxValue[0];

            for (uint8_t i = 1; i < NUM_KIND_TARGETS; ++i) {
              if (maxValue[i] > maxValueOfAllComp) {
                maxValueOfAllComp = maxValue[i];
                maxCompOfAll = i;
              }
            }

            numBiggestColor = maxCompOfAll;

            if (DRAW) {
              const int RADIUS_CENTR = 15;

              for (int dx = -RADIUS_CENTR / 2; dx <= RADIUS_CENTR / 2; ++dx) {
                  for (int dy = -RADIUS_CENTR; dy <= RADIUS_CENTR; ++dy) {
                      if (c_mass[maxCompOfAll].x + dx < X_BEGIN || c_mass[maxCompOfAll].x + dx >= X_END ||
                        c_mass[maxCompOfAll].y + dy >= Y_END || c_mass[maxCompOfAll].y + dy < Y_BEGIN) {
                          continue;
                      }

                      setColorUV(uv, c_mass[maxCompOfAll].x + dx, c_mass[maxCompOfAll].y + dy, 0, 80);
                      setColorUV(uv, c_mass[maxCompOfAll].x + dx, c_mass[maxCompOfAll].y + dy, 1, 80);
                  }
               }
            }

          /*  if (maxCompOfAll < 0) {
                return CvPoint<int> { 0, 0 };
            } */

            return c_mass[maxCompOfAll];
        }


      CvPoint<int> CV_UV::handle_frame() {
            if (!camera_frame()) {
                message("error camera_frame");
                return CvPoint<int> { 0, 0 };
            }

            calibUV(0);
            find_color();
            CvPoint<int> value = find_components();

            if (DRAW) {
              calibUV(1);
              yuyv2rgb();
            }

            return value;
      }

       int CV_UV::minmax(int min, int v, int max) {
          return (v < min) ? min : (max < v) ? max : v;
       }

       uint8_t * CV_UV::yuyv2rgb() {
         for (size_t i = 0; i < H; ++i) {
           for (size_t j = 0; j < W; j += 2) {
             size_t index = i * W + j;
             int y0 = camera->head.start[index * 2 + 0] << 8;
             int u = camera->head.start[index * 2 + 1] - 128;
             int y1 = camera->head.start[index * 2 + 2] << 8;
             int v = camera->head.start[index * 2 + 3] - 128;
             rgb[index * 3 + 0] = minmax(0, (y0 + 359 * v) >> 8, 255);
             rgb[index * 3 + 1] = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);
             rgb[index * 3 + 2] = minmax(0, (y0 + 454 * u) >> 8, 255);
             rgb[index * 3 + 3] = minmax(0, (y1 + 359 * v) >> 8, 255);
             rgb[index * 3 + 4] = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
             rgb[index * 3 + 5] = minmax(0, (y1 + 454 * u) >> 8, 255);
           }
         }

         return rgb;
       }

       /*void CV_UV::display() {
            //uv2yuyv();
            yuyv2rgb();

            FILE * out = fopen("result.jpg", "w");
            jpeg(out, rgb, W, H, 100);

            fclose(out);
            free(rgb);
       }*/

    } /* END namespace vision */
} /* END namespace robot */

#endif /* _VISION_CPP_ */
