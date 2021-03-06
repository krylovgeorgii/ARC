/* test_vision.cpp
 *
 * recognize tetracube 
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 * 			https://www.facebook.com/krylov.georgii
 *

#include "MyRIO/vision.h"

#define DISPLAY

#ifdef DISPLAY
  #include "MyRIO/display.h"
#endif /* DISPLAY */


bool DRAW = 1;

robot::vision::CV_UV cam1(640, 420, "/dev/video1", DRAW);  //(1920, 1080); //1280, 720 (640, 420)

#ifdef DISPLAY
  robot::vision::Display window1(cam1.getW(), cam1.getH());
#endif /* DISPLAY */

void vision() {
  int quit = 0;
  int count = 0;
  double t = time();

  #ifdef DISPLAY
    SDL_Event event;
  #endif /* DISPLAY */

  while (!quit) {
    double startOneFTIme = time();

    cam1.handle_frame();

    switch (cam1.getNumBiggestColor()) {
      case 0:
        message("blueAngle");
        break;
      case 1:
        message("orangeAngle");
        break;
      case 2:
        message("greenS");
        break;
      case 3:
        message("redS");
        break;
      case 4:
        message("yellowSquare");
        break;
      case 5:
        message("blueBeam");
        break;
    }

    #ifdef DISPLAY
      if (DRAW) {
        window1.displayWindow(cam1.getRGB());
      }
    #endif /* DISPLAY */

    if (++count > 30) {
        message("fps = " << count / (time() - t));
        count = 0;
        t = time();
    }

    double timeToFrame = 0.034 - (time() - startOneFTIme); //0.034
    //message(timeToFrame);
    //message("deltaT = " << timeToFrame);
    wait((timeToFrame > 0) ? timeToFrame : 0.0001);

    #ifdef DISPLAY
      while (SDL_PollEvent(&event)) {
        switch(event.type) {
          case SDL_QUIT:
              quit = 1;
              break;
        }
      }
    #endif /* DISPLAY */
  }

  message("fps = " << count / (time() - t));
}

int main()
{
    vision();
}
