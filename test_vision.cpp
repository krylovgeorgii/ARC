#include "MyRIO/vision.h"

//#define DISPLAY 1

#ifdef DISPLAY
  #include "MyRIO/display.h"
#endif /* DISPLAY */


bool DRAW = 0;

robot::vision::CV_UV cam1(640, 420, "/dev/video1", DRAW);  //(1920, 1080); //1280, 720

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


    /*while(1) {
      cam1.handle_frame();
      wait(0.001);
    }*/
    //640, 420 -- 30 fps
}
