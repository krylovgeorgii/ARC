/* main.cpp
 *
 * example using button 
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 * 			https://www.facebook.com/krylov.georgii
 *
 */

#include "MyRIO/ButtonIRQ.h"
#include <thread>
#include "MyRIO/button.h"
#include "MyRIO/type.h"
#include "MyRIO/pins.h"
#include "MyRIO/time.h"
#include "MyRIO/log.h"
#include "MyRIO/robot.h"

void constructor(int argc, char **argv);
NiFpga_Status distructor();
void buttonIrqThread();

void buttonIrqThread() {
	robot::button::waitButtonPush();
	message("exit from button");
	exit(distructor());
}

int main(int argc, char **argv){
	constructor(argc, argv);

	robot::button::waitButtonPush();
	message("start doing");
	wait(1);

	std::thread buttonThread(buttonIrqThread);
	buttonThread.detach();
	wait(5);
	message("END");


	return distructor();
}

void constructor(int argc, char **argv){
	robot::start(argc, argv);
}

NiFpga_Status distructor(){
	return robot::finish();
}

