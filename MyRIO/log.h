/* log.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 * updates in version 1.2:
 * 		add namespace log in namespace robot
 * 		add use_time_from_powerUp
 *		move _USE_LOG_ to this file from config
 *
 * updates in version 1.3:
 * 		restruct namespace log to struct Log
 * 		add Log.cleanLogFile
 * 		change Log members names register
 * 		add __FILE__ __LINE__ __func__
 */

#ifndef _LOG_H_
#define _LOG_H_ "1.4"

#include <iostream>
#include <fstream>
#include "time.h"

namespace robot{
	struct Log{
		std::string file = "LOG.log";
		bool useTerminal = true;
		bool useFile = true;
		bool useTime = true;
		bool useTimeFromPowerup = false;
		bool cleanLogFile = false;
		bool printFileName = true;
		bool printLine = true;
		bool printFuctionName = true;
	};

	extern Log log;
}

#define LOG_format(_out, _x)\
do {\
	_out << "[ ";\
	if (robot::log.useTime) {\
		_out.precision(9);\
		_out.width(12);\
		if (robot::log.useTimeFromPowerup) {\
			_out << timeMessage;\
		} else {\
			_out << timeMessage - robot::timing.timeAtPowerUp();\
		}\
	}\
	\
	if (robot::log.printFileName) {\
		_out << " " << __FILE__;\
	}\
	\
	if (robot::log.printLine) {\
		_out << " line "  << __LINE__;\
	}\
	\
	if (robot::log.printFuctionName) {\
		_out << " " << __func__ << "()";\
	}\
	\
	_out << " ] : " << _x << std::endl;\
} while(0) /* END MACROS LOG_format(out, x) */

#define message(x)\
do {\
	double timeMessage = 0;\
	\
	if (robot::log.useTime) {\
		timeMessage = time();\
	}\
	\
	if (robot::log.useFile) {\
		std::ofstream out(robot::log.file, std::ios::app);\
		if (!out && robot::log.useTerminal) {\
			LOG_format(std::cout, "Cannot open file " << robot::log.useFile);\
		} else {\
			LOG_format(out, x);\
			out.close();\
		}\
	}\
	\
	if(robot::log.useTerminal) {\
		LOG_format(std::cout, x);\
	}\
} while (0) /* END MACROS message(x) */

#endif /* _LOG_H_ */
