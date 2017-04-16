/* time.cpp
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 */

#ifndef _TIME_CPP_
#define _TIME_CPP_ "1.2"

#include "time_1.2.h"

namespace robot{
	void Timing::resetTime(){
		TIME_AT_START = time();
	}

	Timing timing;
}

double time(bool update_time, timeval *timer){
	if(update_time){
		gettimeofday(timer, NULL);
		robot::timing.oldTime = timer->tv_sec + timer->tv_usec * 0.000001;
	}

	return robot::timing.oldTime;
}

#endif /* _TIME_CPP_ */


