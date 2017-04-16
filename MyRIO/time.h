/* time.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 * updates in version 1.1:
 * 		add namespace timing in namespace robot
 * 		add TIME_AT_START in namespace timing
 *		add double timeAtPowerUp() in namespace timing
 *		add resetTime() in namespace timing
 *
 * updates in version 1.2:
 * 		restruct namespace timing to struct Timing
 */

#ifndef _TIME_H_
#define _TIME_H_ "1.2"

#include <sys/time.h>
#include <stdio.h>

namespace robot{
	class Timing{
	public:
		Timing(){
			resetTime();
		}

		inline double timeAtPowerUp() const {
			return TIME_AT_START;
		}

		void resetTime();
		inline timeval *getTimer() {
			return &timer;
		}

		double oldTime;

	private:
		timeval timer;
		double TIME_AT_START;
	};

	extern Timing timing;
}

double time(bool update_time = true, timeval *timer = robot::timing.getTimer());

#endif /* _TIME_H_ */
