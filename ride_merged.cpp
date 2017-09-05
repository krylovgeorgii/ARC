/* ride.cpp
 *
 * Authors: Krylov Georgii & Ilyasov Alexander
 *
 * Contacts:
 * 		krylov.georgii@gmail.com
 * 		https://vk.com/krylov.georgii
 * 		https://www.facebook.com/krylov.georgii
 *
 */

#include <thread>
#include <mutex>
//#include <atomic>

#include "MyRIO/button.h"
#include "MyRIO/type.h"
#include "MyRIO/pins.h"
#include "MyRIO/time.h"
#include "MyRIO/log.h"
#include "MyRIO/robot.h"

#include "MyRIO/vision.h"

void constructor(int argc, char **argv);
NiFpga_Status distructor();
void shiftHandler(double dist);

int shiftPixToGrip = 109; //58
constexpr int shiftYObjIsGrip = 337 - 25; //-47
int pixInCM = 100.0 / 14 + 0.5;

constexpr double pixInCm = (154 - 110) / 4.8;

robot::WheelBazeMore *W = nullptr;
robot::MotorMore *motorTurn = nullptr, *motorShift = nullptr, *motorCube = nullptr;
robot::Aio *sen[6];
robot::Servo *servoBlock = nullptr, *servoGrab = nullptr, *servoPlate = nullptr;
//robot::Aio *RSen = nullptr;

robot::vision::CV_UV cam1(640, 420, "/dev/video0", false);

uint8_t isObj = 1;
bool useCam = false;

const int servoGrabCatchFloorVal = 115; //176
const int servoGrabStartVal = 172;
const int servoGrabCatchChuteVal = 159;
const int servoGrabStartCatchVal = 150;
const int servoGrabCatchedVal = 0;
const int servoGrabCatchedManipUp = 100;
const int servoGrabPushVal = 120;
const int servoGrabClosedPerpVal = 110;
const int servoGrabMinVal = 0;
const int servoGrabMaxVal = 172;

const int servoBlockUpVal = 123;
const int servoBlockPushVal = 25;
const int servoBlockCatchChuteVal = 48;
const int servoBlockDownVal = 12;

const double motorShiftLeftEdge = 16.5;
const double motorShiftZeroXPos = 16.3;
const double motorShiftRightEdge = -12.5;

int numBricksOnLines[2] = {3, 3};
int lastBricksOnLines[2] = {-1, -1};
int nowLine = 1;
bool everTookFromChute = false;

enum BRICK_T : const int { SQUARE, STRAIGHT, ZIGZAG, ANGLE, TFORM };

class Brick {
	public:
		Brick(const int brickType, int countOfThisType = 0) {
			Type = brickType;
			numOfThisType = countOfThisType;
			calcPushValues();
		};

		void setShiftPos () {
			shiftPos = motorShiftZeroXPos - xLevel * 5;
		};

		void setManipAng () {
			double ang;
			switch (yLevel) {
				case 0: {
					ang = -30;
					break;
				}

				case 1: {
					ang = -40;
					break;
				}

				case 2: {
					ang = -50;
					break;
				}

				case 3: {
					ang = -60;
					break;
				}

				case 4: {
					ang = -70;
					break;
				}

				case 5: {
					ang = -80;
					break;
				}

				default: {
					ang = -90;
					break;
				}
			}
			maxManipAng = ang;
		};

		void setDist () {
			if (yLevel < 4) {
				moveFromLineDist = 31.2;
			} else if (yLevel == 4) {
				moveFromLineDist = 31.2 + 2;
			} else if (yLevel == 5) {
				moveFromLineDist = 31.2 + 5;
			}
		};

		void calcPushValues() {
			switch (Type) {
				case BRICK_T::SQUARE: {
					if (numOfThisType == 0) {
						yLevel = 0;
						xLevel = 3;
						//maxManipAng = -30;
					}

					else if (numOfThisType == 1) {
						yLevel = 3;
						xLevel = 3;
						//maxManipAng = -58;
					}

					else {
						message("Too many squares");
						yLevel = 3;
						xLevel = 3;
					}
					break;
				}

				case BRICK_T::STRAIGHT: {
					if (numOfThisType == 0) {
						yLevel = 2;
						xLevel = 2;
					}

					else if (numOfThisType == 1) {
						yLevel = 5;
						xLevel = 2;
					}

					else {
						message("Too many straights");
						yLevel = 5;
						xLevel = 2;
					}
					break;
				}

				case BRICK_T::ANGLE: {
					if (numOfThisType == 0) {
						yLevel = 0;
						xLevel = 0;
					}

					else if (numOfThisType == 1) {
						yLevel = 0;
						xLevel = 6;
					}
					else if (numOfThisType == 2) {
						yLevel = 3;
						xLevel = 0;
					}

					else if (numOfThisType == 3) {
						yLevel = 3;
						xLevel = 6;
					}

					else {
						message("Too many angles");
						yLevel = 6;
						xLevel = 3;
					}
					break;
				}

				case BRICK_T::ZIGZAG: {
					if (numOfThisType == 0) {
						yLevel = 1;
						xLevel = 1;
					}

					else if (numOfThisType == 1) {
						yLevel = 1;
						xLevel = 5;
					}
					else if (numOfThisType == 2) {
						yLevel = 4;
						xLevel = 1;
					}

					else if (numOfThisType == 3) {
						yLevel = 4;
						xLevel = 5;
					}

					else {
						message("Too many angles");
						yLevel = 4;
						xLevel = 5;
					}
					break;
				};
			};

			setManipAng();
			setShiftPos();
			setDist();
		};

		double getManipAng() {
			return maxManipAng;
		}

		double getmoveDist() {
			return moveFromLineDist;
		}

		double getShiftPos() {
			return shiftPos;
		}

		int getType() {
			return Type;
		}

	private:
		int Type;
		int numOfThisType = 0;
		int xLevel, yLevel;
		double maxManipAng;
		double shiftPos;
		double moveFromLineDist;
};

Brick *nowBrick;
int numOfBricks[5] = { 0 };
bool needHoldHandler = true;

void constructor(int argc, char **argv)  {
 	robot::start(argc, argv);

 	W = new robot::WheelBazeMore (robot::Pin::PWM_A1, robot::Pin::PWM_A0, robot::Pin::DIO_A10, robot::Pin::DIO_A7,
 			robot::Pin::ENC_B, robot::Pin::ENC_A, 280. / 360 * 2, 10, 40);
 	if (W == nullptr) {
  		message("Can't create whleelBaze !");
  		exit(distructor());
 	}

 	motorShift = new robot::MotorMore (robot::Pin::PWM_B1, robot::Pin::DIO_B10, robot::Pin::ENC_C0,
 			280. / 360 * 1.5 * 2, 1.9);
 	if (motorShift == nullptr) {
 		message ("Can't create motorShift");
 		exit(distructor());
 	}

 	motorTurn = new robot::MotorMore (robot::Pin::PWM_B0, robot::Pin::DIO_B7, robot::Pin::ENC_C1,
 	 			280. / 360 * 1.5 * 2);
 	if (motorTurn == nullptr) {
 		message ("Can't create motorTurn");
 	 	exit(distructor());
 	}
 	motorTurn->reverseMotor(); // was made when gear was changed to sprocket

 	motorCube = new robot::MotorMore (robot::Pin::PWM_B1, robot::Pin::DIO_B10, robot::Pin::ENC_C0,
 			280. / 360 * 1.5);
 	if (motorCube == nullptr) {
 		message ("Can't create motorCube");
 		exit(distructor());
 	}

 	servoBlock = new robot::Servo(robot::Pin::PWM_C1);
 	if (servoBlock == nullptr) {
 		message("Can't create servoBlock");
 		exit(distructor());
 	}

 	servoBlock->relax();

 	servoGrab = new robot::Servo(robot::Pin::PWM_C0);
 	if (servoGrab == nullptr) {
 		message("Can't create servoGrab");
 	 	exit(distructor());
 	}

 	servoGrab->relax();


 	/*
	SEN 0  1.94470195  2.85290498  FR_L
	SEN 1  4.57311965  4.60192824  FR_R
	SEN 2  2.57177708  3.3238522   MID_L
	SEN 3  4.57849074  4.60632277  MID_R
	SEN 4  2.29724098  3.10827605  B_L
	SEN 5  2.38207983  3.13244597  B_R
 	 */

 	sen[SEN::MID_L] = new robot::Aio(robot::Pin::AI_B0, 2.8, 2.1);
 	sen[SEN::B_L] = new robot::Aio(robot::Pin::AI_B1, 2.8, 2.1);
 	sen[SEN::FR_L] = new robot::Aio(robot::Pin::AI_B2, 2.8, 2.1);
 	sen[SEN::FR_R] = new robot::Aio(robot::Pin::AI_B3, 2.8, 2.1);
 	sen[SEN::B_R] = new robot::Aio(robot::Pin::AI_A0, 2.8, 2.1);
 	sen[SEN::MID_R] = new robot::Aio(robot::Pin::AI_A1, 2.8, 2.1);
}

NiFpga_Status distructor() {
	W->stop();
 	delete W;
 	//servoBlock->setAngle(servoBlockUpVal);
 	servoGrab->setAngle(servoGrabStartVal);
 	motorShift->moveStr(0, 4, []()->uint8_t {return 0;});
 	motorShift->setPower(0);
 	motorTurn->setPower(0);
 	delete motorShift;
 	delete motorTurn;
 	for (int i = static_cast<int>(SEN::FR_L); i <= static_cast<int>(SEN::B_R); i++) {
 		delete sen[i];
 	}
 	servoBlock->relax();
 	servoGrab->relax();
 	delete servoBlock;
 	delete servoGrab;
 	return robot::finish();
}

int sureLine(uint8_t sensor) {
	double senVal = sen[sensor]->aioReadNorm();

	if (senVal > 0.7)
		return 1;
	if (senVal < 0.3)
		return 0;
	return -1;
}

int onLine(uint8_t sensor, double edge = 0.5) {
	return sen[sensor]->aioReadNorm() < edge ? 0 : 1;
}

int offLine(uint8_t sensor, double edge = 0.5) {
	return sen[sensor]->aioReadNorm() > edge ? 0 : 1;
}

int onLine(double senVal, double edge = 0.5) {
	return senVal < edge ? 0 : 1;
}

int offLine(double senVal, double edge = 0.5) {
	return senVal > edge ? 0 : 1;
}
uint8_t getSenPos (uint8_t oldPos, int onLine, double speed) {
	if (onLine == 1) {
		switch (oldPos) {
		case SEN_POS::BEF_OFF :
			return SEN_POS::BEF_ON;
		case SEN_POS::AFT_OFF :
			return SEN_POS::AFT_ON;
		}
	}

	else if (onLine == 0){
		if (oldPos == SEN_POS::BEF_ON || oldPos == SEN_POS::AFT_ON) {
			message("Moved from line");
			if (speed < 0.)
				return SEN_POS::BEF_OFF;
			else if (speed > 0.)
				return SEN_POS::AFT_OFF;
		}
	}

	return oldPos;
}

int setRideToLine(uint8_t senPos) {
	switch (senPos) {
	case SEN_POS::BEF_OFF :
		return 1;
	case SEN_POS::AFT_OFF :
		return -1;
	default :
		return 0;
	}
}

void printPos(uint8_t sen, uint8_t senPos) {
	std::string s;
	switch (sen) {
		case SEN::MID_L : {
			s += "MID_L - ";
			break;
		}
		case SEN::MID_R : {
			s += "MID_R - ";
			break;
		}
		default : {
			s += "Other sen - ";
			break;
		}
	}

	switch(senPos) {
		case SEN_POS::BEF_OFF : {
			s += "BEF_OFF";
			break;
		}
		case SEN_POS::BEF_ON : {
			s += "BEF_ON";
			break;
		}
		case SEN_POS::AFT_OFF : {
			s += "AFT_OFF";
			break;
		}
		case SEN_POS::AFT_ON : {
			s += "AFT_ON";
			break;
		}
	}
	message(s);
}

// @param: dir +1 Clockwise, -1 antiClockwise
//		   num number of needed line
//		   side +1 front -1 back

void turnToLine(int dir, int num, int side, double speed, bool calBefore = true, bool calAfter = true, double speedReturn = 10) {
	uint8_t contrSen;
	if (side > 0) {
		if (dir > 0)	{ contrSen = SEN::FR_L; }
		else	{ contrSen = SEN::FR_R; }
	}
	else {
		if (dir > 0)	{ contrSen = SEN::B_R; }
		else	{ contrSen = SEN::B_L; }
	}

	double t = time();
	if (calBefore) {
		if (side > 0) {
			while (time() - t < 1) {
				W->followLine(sen[SEN::FR_L]->aioReadNorm(),
					sen[SEN::FR_R]->aioReadNorm(), 250, 0);
				wait(0.001);
			}
		}
		else {
			while (time() - t < 1) {
				W->followLine(sen[SEN::B_L]->aioReadNorm(),
					sen[SEN::B_R]->aioReadNorm(), -250, 0);
				wait(0.001);
			}
		}
	}
	W->stop();
	W->encRes();

	std::function<uint8_t()> funcOn = [&]()->int { /*message("Off senVal - " << sen[contrSen]->aioReadNorm());*/
		return onLine(contrSen, 0.7); };
	std::function<uint8_t()> funcOff = [&]()->int { /*message("On senVal - " << sen[contrSen]->aioReadNorm());*/
		return offLine(contrSen, 0.3); };
	W->turnAng(130 * dir, speed, funcOn);//[]()->int { return /*onLine(SEN::FR_L)*/ 0; });
	message("---On line---");
	for (int i = 1; i < num; i++) {
		W->encRes();
		W->turnAng(45 * dir, speed, funcOff);
		message("---Off line---");
		W->encRes();
		W->turnAng(130 * dir, speed, funcOn);//[]()->int { return onLine(SEN::FR_L); });
		message("---On line---");
	}

	t = time();
	if (calAfter) {
		if (side > 0) {
			while (time() - t < 1) {
				W->followLine(sen[SEN::FR_L]->aioReadNorm(),
					sen[SEN::FR_R]->aioReadNorm(), 250, 0);
				wait(0.001);
			}
		}
		else {
			while (time() - t < 1) {
				W->followLine(sen[SEN::B_L]->aioReadNorm(),
					sen[SEN::B_R]->aioReadNorm(), -250, 0);
				wait(0.001);
			}
		}
	}

	W->stop();
	W->encRes();

	/*if (side > 0) {
		if (dir > 0) {
			W->encRes();
			W->turnAng(130, speed, []()->int { return onLine(SEN::FR_L); });

			for (int i = 1; i < num; i++) {
				W->encRes();
				W->turnAng(30, speed, []()->int { return offLine(SEN::FR_L); });
				W->encRes();
				W->turnAng(130, speed, []()->int { return onLine(SEN::FR_L); });
			}

			W->encRes();
			W->turnAng(-40, speedReturn, []()->int { return onLine(SEN::FR_R); });
		}
		else {
			W->encRes();
			W->turnAng(-130, speed, []()->int { return onLine(SEN::FR_R); } );
			for (int i = 1; i < num; i++) {
				W->encRes();
				W->turnAng(-30, speed, []()->int { return offLine(SEN::FR_R); });
				W->encRes();
				W->turnAng(-130, speed, []()->int { return onLine(SEN::FR_R); });
			}

			W->encRes();
			W->turnAng(40, speedReturn, []()->int { return onLine(SEN::FR_L); });
		}
	}
	else {
		if (dir > 0) {
			W->encRes();
			W->turnAng(130, speed, []()->int { return onLine(SEN::B_L); });

			for (int i = 1; i < num; i++) {
				W->encRes();
				W->turnAng(30, speed, []()->int { return offLine(SEN::B_L); });
				W->encRes();
				W->turnAng(130, speed, []()->int { return onLine(SEN::B_L); });
			}

			W->encRes();
			W->turnAng(-40, speedReturn, []()->int { return onLine(SEN::B_R); });
		}
		else {
			W->encRes();
			W->turnAng(-130, speed, []()->int { return onLine(SEN::B_R); } );
			for (int i = 1; i < num; i++) {
				W->encRes();
				W->turnAng(-30, speed, []()->int { return offLine(SEN::B_R); });
				W->encRes();
				W->turnAng(-130, speed, []()->int { return onLine(SEN::B_R); });
			}
		}

		W->encRes();
		W->turnAng(40, speedReturn, []()->int { return onLine(SEN::B_L); });
	}

	W->stop();*/
}

void moveToLine(uint8_t posLeft, uint8_t posRight, uint8_t lSen, uint8_t rSen, double dopPower, double speed, int align = 0) {
	double lEnc, rEnc, lOldEnc, rOldEnc;
	double nowTime, oldTime, oldMesTime = time();
	double lSpeed = 0, rSpeed = 0;
	double lPow = 0, rPow = 0;
	uint8_t lPos = posLeft, rPos = posRight;
	uint8_t lOldPos = lPos, rOldPos = rPos;
	double k = 15;
	bool lCrossed = false, rCrossed = false;


	oldTime = nowTime = time();
	lEnc = lOldEnc = W->L_enc();
	rEnc = rOldEnc = W->R_enc();

	//W->ride(power, power);

	double startTimeOut = time();

	do {
		wait(0.05);
		oldTime = nowTime;
		lOldEnc = lEnc;
		rOldEnc = rEnc;
		lOldPos = lPos;
		rOldPos = rPos;
		lPos = getSenPos(lOldPos, onLine(lSen, 0.5), lSpeed);
		rPos = getSenPos(rOldPos, onLine(rSen, 0.5), rSpeed);
		if(!lCrossed && lPos != lOldPos)
			lCrossed = true;
		if(!rCrossed && rPos != rOldPos)
			rCrossed = true;
		//lPow = setRideToLine(lPos) * power;
		//rPow = setRideToLine(rPos) * power;
		if (!lCrossed && !rCrossed)
		{
			W->rideStr(((speed - (fabs(lSpeed) + fabs(rSpeed)) / 2) * k + dopPower) * setRideToLine(lPos));
		}
		else {
			lPow = setRideToLine(lPos) * ((speed - fabs(lSpeed)) * k + dopPower);
			rPow = setRideToLine(rPos) * ((speed - fabs(rSpeed)) * k + dopPower);
			W->ride(lPow, rPow);
		}
		nowTime = time();
		lEnc = W->L_enc();
		rEnc = W->R_enc();
		lSpeed = W->L()->getSpeed(oldTime, nowTime, lOldEnc, lEnc);
		rSpeed = W->R()->getSpeed(oldTime, nowTime, rOldEnc, rEnc);
		if ( nowTime - oldMesTime > 0.25) {
			/*message("LSpeed = " << lSpeed << ", RSpeed = " << rSpeed);
			message("LPow = " << lPow << ", RPow = " << rPow);
			message("LSen = " << sen[lSen]->aioReadNorm()
					<< ", RSen = " << sen[rSen]->aioReadNorm());*/
			printPos(lSen, lPos);
			printPos(rSen, rPos);
			message("lCrossed - " << lCrossed << ", rCrossed - " << rCrossed);
			oldMesTime = nowTime;
		}

		if (time() - startTimeOut > 5) {
			return;
		}
	} while ( offLine(lSen) || offLine(rSen) ||	fabs(lSpeed) > 2 || fabs(rSpeed) > 2);
	/*message("Finished. LSpeed = " << lSpeed << ", RSpeed = " << rSpeed);
	message("LPow = " << lPow << ", RPow = " << rPow);
	message("LSen = " << sen[lSen]->aioReadNorm()
			<< ", RSen = " << sen[rSen]->aioReadNorm());*/
	/*printPos(lSen, lPos);
	printPos(rSen, rPos);*/

	W->stop();
	W->encRes();
	lEnc = 0;
	rEnc = 0;
	nowTime = time();

	if (align != 0) {
		while(time() - nowTime < 2) {
			wait(0.05);
			oldTime = nowTime;
			nowTime = time();
			lOldEnc = lEnc;
			lEnc = W->L()->getEncVal();
			rOldEnc = rEnc;
			rEnc = W->R()->getEncVal();
			W->L()->holdTargetAngle(W->L()->getAngleVal() + 5 * align * (0.5 - sen[lSen]->aioReadNorm()));
			W->R()->holdTargetAngle(W->R()->getAngleVal() + 5 * align * (0.5 - sen[rSen]->aioReadNorm()));
			lSpeed = W->L()->getSpeed(oldTime, nowTime, lOldEnc, lEnc);
			rSpeed = W->R()->getSpeed(oldTime, nowTime, rOldEnc, rEnc);
			message("l - " << 0.5 - sen[lSen]->aioReadNorm() << ", l speed - " << lSpeed);
			message("r - " << 0.5 - sen[rSen]->aioReadNorm() << ", r speed - " << rSpeed);
			//W->ride(dopPower * (0.5 - sen[lSen]->aioReadNorm()) * align, dopPower * (0.5 - sen[rSen]->aioReadNorm()) * align);
			if (fabs(0.5 - sen[lSen]->aioReadNorm()) < 0.3 && fabs(0.5 - sen[rSen]->aioReadNorm()) < 0.3 &&
					fabs(lSpeed) < 2.5 && fabs(rSpeed) < 2.5) {
				W->stop();
				message("move to line stopped");
				return;
			}
		}
	}

}

void moveToLine2(uint8_t posLeft, uint8_t posRight, uint8_t lSen = SEN::MID_L, uint8_t rSen = SEN::MID_R,
		double dopPower = 400, double speed = 30, int align = 0) { // align = 1 - front side of line, -1 - back, 0 - no
	double lEnc, rEnc, lOldEnc, rOldEnc;
	double nowTime, oldTime, oldMesTime = time();
	double lSpeed = 0, rSpeed = 0;
	double lPow = 0, rPow = 0;
	uint8_t lPos = posLeft, rPos = posRight;
	double k = 20;
	bool lCrossed = false, rCrossed = false;


	oldTime = nowTime = time();
	lEnc = lOldEnc = W->L_enc();
	rEnc = rOldEnc = W->R_enc();

	//W->ride(power, power);

	double startTimeOut = time();

	do {
		wait(0.05);
		oldTime = nowTime;

		if (!lCrossed) {
			lPos = getSenPos(posLeft, onLine(lSen), lSpeed);
			lOldEnc = lEnc;
			lEnc = W->L_enc();
			lPow = setRideToLine(lPos) * ((speed - fabs(lSpeed)) * k + dopPower);
		}

		if (!rCrossed) {
			rPos = getSenPos(posRight, onLine(rSen), rSpeed);
			rOldEnc = rEnc;
			rEnc = W->R_enc();
			rPow = setRideToLine(rPos) * ((speed - fabs(rSpeed)) * k + dopPower);
		}

		if (!lCrossed && (lPos == SEN_POS::AFT_ON || lPos == SEN_POS::BEF_ON)) {
			lCrossed = true;
			lPow = 0;
			lOldEnc = W->L_enc();
		}
		if (!rCrossed && (rPos == SEN_POS::AFT_ON || rPos == SEN_POS::BEF_ON)){
			rCrossed = true;
			rPow = 0;
			rOldEnc = W->R_enc();
		}
		if (!lCrossed && !rCrossed)
			W->rideStr(((speed - (fabs(lSpeed) + fabs(rSpeed)) / 2) * k + dopPower) * (rPos == SEN_POS::AFT_OFF ? -1 : 1));

		else
			W->ride(lPow, rPow);

		nowTime = time();


		lSpeed = W->L()->getSpeed(oldTime, nowTime, lOldEnc, lEnc);
		rSpeed = W->R()->getSpeed(oldTime, nowTime, rOldEnc, rEnc);
		if ( nowTime - oldMesTime > 0.25) {
			/*message("LSpeed = " << lSpeed << ", RSpeed = " << rSpeed);
			message("LPow = " << lPow << ", RPow = " << rPow);
			message("LSen = " << sen[lSen]->aioReadNorm()
					<< ", RSen = " << sen[rSen]->aioReadNorm());*/
			printPos(lSen, lPos);
			printPos(rSen, rPos);
			oldMesTime = nowTime;
		}

		if (time() - startTimeOut > 10) {
			message("moveToLine timeout");
			message("l speed - " << lSpeed << ", r Speed - " << rSpeed);
			return;
		}
	} while ( !lCrossed || !rCrossed || fabs(lSpeed) > 3 || fabs(rSpeed) > 3);
	lEnc = W->L_enc();
	rEnc = W->R_enc();
	W->encRes();
	W->move(W->L()->enc_degr(lOldEnc-lEnc), W->R()->enc_degr(rOldEnc-rEnc), speed, []()->int {return 0;});
	W->stop();
	W->encRes();
	lEnc = 0;
	rEnc = 0;
	nowTime = time();
	if (align != 0) {
		while(time() - nowTime < 2) {
			wait(0.05);
			oldTime = nowTime;
			nowTime = time();

			lOldEnc = lEnc;
			lEnc = W->L()->getEncVal();
			rOldEnc = rEnc;
			rEnc = W->R()->getEncVal();

			//W->L()->holdTargetAngle(W->L()->getAngleVal() + 7 * align * (0.5 - sen[lSen]->aioReadNorm()));
			//W->R()->holdTargetAngle(W->R()->getAngleVal() + 7 * align * (0.5 - sen[rSen]->aioReadNorm()));
			lSpeed = W->L()->getSpeed(oldTime, nowTime, lOldEnc, lEnc);
			rSpeed = W->R()->getSpeed(oldTime, nowTime, rOldEnc, rEnc);
			message("l - " << 0.5 - sen[lSen]->aioReadNorm() << ", l speed - " << lSpeed);
			message("r - " << 0.5 - sen[rSen]->aioReadNorm() << ", r speed - " << rSpeed);
			W->ride(dopPower * (0.5 - sen[lSen]->aioReadNorm()) * align, dopPower * (0.5 - sen[rSen]->aioReadNorm()) * align);
			if (fabs(0.5 - sen[lSen]->aioReadNorm()) < 0.15 && fabs(0.5 - sen[rSen]->aioReadNorm()) < 0.15 &&
					fabs(lSpeed) < 2.5 && fabs(rSpeed) < 2.5) {
				W->stop();
				message("move to line stopped");
				return;
			}
		}
	}
	W->stop();
	message("move to line stopped 2");
	/*message("Finished. LSpeed = " << lSpeed << ", RSpeed = " << rSpeed);
	message("LPow = " << lPow << ", RPow = " << rPow);
	message("LSen = " << sen[lSen]->aioReadNorm()
			<< ", RSen = " << sen[rSen]->aioReadNorm());*/
	/*printPos(lSen, lPos);
	printPos(rSen, rPos);*/

}

void moveToLine3(uint8_t posLeft, uint8_t posRight, bool followLine, int align = 0, uint8_t lSen = SEN::MID_L, uint8_t rSen = SEN::MID_R,
		double dopPower = 450, double speed = 25) { // align = 1 - before line, -1 - after, 0 - no
	double lEnc, rEnc, lOldEnc, rOldEnc;
	double nowTime, oldTime, oldMesTime = time();
	double lSpeed = 0, rSpeed = 0;
	double lGoal = 0, rGoal = 0;
	uint8_t lPos = posLeft, rPos = posRight;
	double lFirstBlack, lFirstWhite;
	double rFirstBlack, rFirstWhite;
	double k = 20;
	double lSenVal, rSenVal;
	int lineSign = posRight == SEN_POS::BEF_OFF ? 1 : -1;
	bool lWasBlack = false, rWasBlack = false;
	bool lNowBlack = false, rNowBlack = false;


	SEN lLineSen = lPos == SEN_POS::BEF_OFF ? SEN::FR_L : SEN::B_L;
	SEN rLineSen = rPos == SEN_POS::BEF_OFF ? SEN::FR_R : SEN::B_R;
	oldTime = nowTime = time();
	lEnc = lOldEnc = W->L_enc();
	rEnc = rOldEnc = W->R_enc();

	//W->ride(power, power);

	double startTimeOut = time();

	do {
		wait(0.01);
		oldTime = nowTime;
		nowTime = time();
		lOldEnc = lEnc;
		lEnc = W->L_enc();
		lSenVal = sen[lSen]->aioReadNorm();
		lNowBlack = lWasBlack || (onLine(lSenVal, 0.5));
		rOldEnc = rEnc;
		rEnc = W->R_enc();
		rSenVal = sen[rSen]->aioReadNorm();
		rNowBlack = rWasBlack || (onLine(rSenVal, 0.5));

		if (!lWasBlack && !rWasBlack) {
			startTimeOut = time();
			if (followLine){
				W->followLineSpeed2(sen[lLineSen]->aioReadNorm(), sen[rLineSen]->aioReadNorm(),
						0.03, speed * lineSign, nowTime - oldTime);
			}
			else {
				W->rideStr(((speed - (fabs(lSpeed) + fabs(rSpeed)) / 2) * k + dopPower) * (posRight == SEN_POS::AFT_OFF ? -1 : 1));
			}
		}

		if (!rWasBlack){
			if (lWasBlack) {
				W->R()->setPower(setRideToLine(posRight) * ((speed - fabs(rSpeed)) * k + dopPower));
			}
			rWasBlack = rNowBlack;// == 1 ? 1 : 0);
			if (rWasBlack) {
				//message("---------------R on black---------------");
				rFirstBlack = W->R()->enc_length(rEnc);//getLengthVal();
				//message("r black length - " << rFirstBlack);
				message("r sen - " << rSenVal);
				//W->L()->setPower(0);
				rGoal = rFirstBlack + (posRight == SEN_POS::AFT_OFF ? -2. : 2.) - align * 2.;
			}
		}

		else {
			W->R()->holdTargetLength(rGoal);
		}

		if (!lWasBlack) {
			if (rWasBlack) {
				W->L()->setPower(setRideToLine(posLeft) * ((speed - fabs(lSpeed)) * k + dopPower));
			}

			lWasBlack = lNowBlack;//lWasBlack || (onLine(lSen, 0.5));// == 1 ? 1 : 0);
			if (lWasBlack) {
				//message("---------------L on black---------------");
				lFirstBlack = W->L()->enc_length(lEnc);//getLengthVal();
				//message("l black length - " << lFirstBlack);
				message("l sen - " << lSenVal);
				//W->L()->setPower(0);
				lGoal = lFirstBlack + (posLeft == SEN_POS::AFT_OFF ? -2. : 2.) - align * 2.;
			}
		}

		else {
			W->L()->holdTargetLength(lGoal);
		}
		lSpeed = W->L()->getSpeed(oldTime, nowTime, lOldEnc, lEnc);
		rSpeed = W->R()->getSpeed(oldTime, nowTime, rOldEnc, rEnc);
		if ( nowTime - oldMesTime > 0.1) {
			/*message("LSpeed = " << lSpeed << ", RSpeed = " << rSpeed);
			message("LPow = " << lPow << ", RPow = " << rPow);
			message("LSen = " << sen[lSen]->aioReadNorm()
					<< ", RSen = " << sen[rSen]->aioReadNorm());*/
			//printPos(lSen, lPos);
			//printPos(rSen, rPos);
			/*message("lWasBlack - " << lWasBlack << ", rWasBlack - " << rWasBlack);
			message("lSen - " << sen[lSen]->aioReadNorm() << ", rSen - " << sen[rSen]->aioReadNorm());
			if (lWasBlack && rWasBlack){
				message("lGoal - " << lGoal << ", lLength - " << W->L()->getLengthVal());
				message("rGoal - " << rGoal << ", rLength - " << W->R()->getLengthVal());
				message("lSen - " << sen[lSen]->aioReadNorm() << ", rSen - " << sen[rSen]->aioReadNorm());
			}*/
			oldMesTime = nowTime;
		}

		if (time() - startTimeOut > 3) {
			message("moveToLine timeout");
			//message("l speed - " << lSpeed << ", r Speed - " << rSpeed);
			break;
		}
	} while ( !lWasBlack || !rWasBlack || fabs(lSpeed) > 2 || fabs(rSpeed) > 2 || (fabs(W->R()->getLengthVal() - rGoal) > 0.5
			|| fabs(W->L()->getLengthVal() - lGoal) > 0.5) && (align == 0 || fabs(sen[lSen]->aioReadNorm() - 0.5) > 0.1
					|| fabs(sen[rSen]->aioReadNorm() - 0.5) > 0.1));

	W->stop();
	message("---Moved to line");
	message("lGoal - " << lGoal << ", lLength - " << W->L()->getLengthVal());
	message("rGoal - " << rGoal << ", rLength - " << W->R()->getLengthVal());
	message("lSen - " << sen[lSen]->aioReadNorm());
	message("rSen - " << sen[rSen]->aioReadNorm());

	/*if (align != 0) {
		for (oldTime = time(); time() - oldTime < 1.0; wait(0.01)) {
			W->ride(dopPower * (0.5 - sen[lSen]->aioReadNorm()) * align * 5,
					dopPower * (0.5 - sen[rSen]->aioReadNorm()) * align * 5);
			if (fabs(0.5 - sen[lSen]->aioReadNorm()) < 0.1 && fabs(0.5 - sen[rSen]->aioReadNorm()) < 0.1 &&
					fabs(lSpeed) < 0.5 && fabs(rSpeed) < 0.5) {
				W->stop();
				message("move to line stopped");
				return;
			}
		}
	}*/
}

void followLineEnc(int dir, double k, double speed, double dist) {
	double oldT = time();
	double distAng = W->L()->enc_degr(W->length_enc(dist));
	SEN lSen, rSen;
	if (dir == 1) {
		lSen = SEN::FR_L;
		rSen = SEN::FR_R;
	}
	if (dir == -1) {
		lSen = SEN::B_L;
		rSen = SEN::B_R;
	}
	W->L()->setLastTarget(0);
	W->R()->setLastTarget(0);
	while ((fabs(W->L_ang()) + fabs(W->R_ang())) < distAng * 2 ) {
		W->followLineSpeed2(sen[lSen]->aioReadNorm(),
				sen[rSen]->aioReadNorm(), k, dir * speed, time() - oldT);
		oldT = time();
		wait(0.01);
	}

	W->stop();

	//W->L()->setPID_angle(20, 30, 5);
	//W->R()->setPID_angle(20, 30, 5);

}


/*
void button_Irq_Thread(void* resource) {
	ThreadResource* threadResource = static_cast<ThreadResource *>(resource);

	while (true) {
    	uint32_t irqAssert = 0;
    	static uint32_t irqCount = 0;

    	Irq_Wait(threadResource->irqContext,
    	static_cast<NiFpga_Irq>(threadResource->irqNumber),&irqAssert,
    	static_cast<NiFpga_Bool*>( &(threadResource->irqThreadRdy)));

    	if (irqAssert & (1 << threadResource->irqNumber)) {
    		message("exit from button\n");
    		exit(distructor());
  		}
	}
}
*/

void upServoBlock() {
	servoBlock->setAngle(servoBlockUpVal);
}

void downServoBlock() {
	for (int i = servoBlockUpVal; i > servoBlockDownVal; --i) {
		servoBlock->setAngle(i);
		wait(0.01);
	}
}

void findObj() {
	robot::vision::CvPoint<int> coord { 0, 0 };
	//double t = time();

	//while (true) {
		double startOneFTIme = time();
		coord = cam1.handle_frame();

		if (coord.y != 0) {
			isObj = 0;
			message("find obj coord.y = " << coord.y);
		}
		else {
			isObj = 1;
			message("Coord is 0");
		}

		//double timeToFrame = 0.034 - (time() - startOneFTIme);
		//wait((timeToFrame > 0) ? timeToFrame : 0.001);
}
int checkBrickType(int numOfObj) {
	int type = BRICK_T::ANGLE;
	switch (numOfObj) {
	case 0: {
		type = BRICK_T::ANGLE;
		message("Angle");
		break;
	}

	case 1: {
		type = BRICK_T::ZIGZAG;
		message("Zigzag");
		break;
	}

	case 2: {
		type = BRICK_T::SQUARE;
		message("Square");
		break;
	}
	}
	return type;
	//return numOfBricks[BRICK_T::ANGLE] == 0 ? BRICK_T::ANGLE : BRICK_T::ZIGZAG;
}

int checkBrickTypeVision(bool isFromChute = false) {
	int typeFinal;
	const int ITER_NUM = 8;
	int types[ITER_NUM] = {-1};
	double startOneFTime;

	for (int i = 0; i < ITER_NUM; ++i) {
		startOneFTime = time();
		cam1.handle_frame();

		switch (cam1.getNumBiggestColor()) {
		case 0:
		case 1: {
			types[i] = BRICK_T::ANGLE;
			//message("Angle");
			break;
		}

		case 2:
		case 3: {
			types[i] = BRICK_T::ZIGZAG;
			//message("Zigzag");
			break;
		}

		case 4: {
			types[i] = BRICK_T::SQUARE;
			//message("Square");
			break;
		}

		case 5: {
			types[i] = BRICK_T::STRAIGHT;
			//message("Straight");
			break;
		}
		}
		double timeToFrame = 0.034 - (time() - startOneFTime);
		wait((timeToFrame > 0) ? timeToFrame : 1e-4 );
	}
	typeFinal = types[ITER_NUM / 2];
	switch (typeFinal) {
	case 0: {
		message("Square");
		break;
	}
	case 1: {
		message("Straight");
		break;
	}
	case 2: {
		message("Zigzag");
		break;
	}
	case 3: {
		message("Angle");
		break;
	}
	}

	return typeFinal;
	//return numOfBricks[BRICK_T::ANGLE] == 0 ? BRICK_T::ANGLE : BRICK_T::ZIGZAG;
}

bool checkBrickIsPossible(int brickType) { // 1 - possible, 0 - possible later, -1 - never possible
	switch (brickType) {
	case BRICK_T::SQUARE:
		return numOfBricks[BRICK_T::SQUARE] == 0;
	case BRICK_T::ANGLE:
		return numOfBricks[BRICK_T::ANGLE] < 2;

	case BRICK_T::ZIGZAG:
		return numOfBricks[BRICK_T::ZIGZAG] < numOfBricks[BRICK_T::ANGLE];

	case BRICK_T::STRAIGHT:
		return (numOfBricks[BRICK_T::ZIGZAG] == 2 && numOfBricks[BRICK_T::ANGLE] == 2 && numOfBricks[BRICK_T::SQUARE] == 1);
	}
}

void checkObj() {
	int brickType = checkBrickTypeVision();
	nowBrick = new Brick(brickType, numOfBricks[brickType]);
	lastBricksOnLines[nowLine] = brickType;
}

//int targetHandlerToHold;

void turnHandler(double targetAngle, double timeToChange) {
	double angle = motorTurn->enc_degr(motorTurn->getEncVal());
	double t = time();

	bool needUpHandler;

	if (angle < targetAngle) {
		needUpHandler = true;
	} else {
		needUpHandler = false;
	}

	while (angle != targetAngle) {
		wait(0.01);
		motorTurn->holdTargetAngle(angle);

		if (time() - t >= timeToChange) {
			if(needUpHandler && angle < targetAngle) {
				++angle;

				if (angle > targetAngle) {
					angle = targetAngle;
				}
			}

			if(!needUpHandler && angle > targetAngle) {
				--angle;

				if (angle < targetAngle) {
					angle = targetAngle;
				}
			}
		}
	}

	while (needHoldHandler) {
		wait(0.01);
		motorTurn->holdTargetAngle(targetAngle);
	}

	motorTurn->setPower(0);
}


void rideToPush() {
	motorTurn->setPower(-10);
	wait(0.1, 0);
	motorTurn->encRes();
	motorTurn->setPower(0);

	servoGrab->setAngle(servoGrabCatchedVal);
	wait(0.2, 0);

	needHoldHandler = true;
	std::thread holdHandlerAngl(turnHandler, 100, 0.02);

	wait(0.5, 0);

	double t = time();
	while (time() - t < 2) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
									sen[SEN::B_R]->aioReadNorm(), -1200, 0);
		wait(0.001);
	}

	while (!onLine((uint8_t)SEN::MID_R) || !onLine((uint8_t)SEN::MID_L)) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
				sen[SEN::B_R]->aioReadNorm(), -520, -700);
		wait(0.01);
	}

	moveToLine(SEN_POS::AFT_ON, SEN_POS::AFT_ON, SEN::MID_L, SEN::MID_R, 250, 25);
	W->stop(0.1);

	turnToLine(-1, 2, 1, 50);
	W->stop();

	//shiftHandler(-9);

	needHoldHandler = false;
	if(holdHandlerAngl.joinable()) {
		holdHandlerAngl.join();
	}

	needHoldHandler = true;

	std::thread holdHandlerToPush(turnHandler, 60, 0.02);

	servoGrab->setAngle(servoGrabPushVal);
	wait(1, 0);
	W->encRes();

	int distInEnc = W->length_enc(20); //25
	while(W->enc() <  distInEnc) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
						sen[SEN::FR_R]->aioReadNorm(), 520, 400);

		wait(0.01);
	}

	t = time();
	while(time() - t < 2) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
						sen[SEN::FR_R]->aioReadNorm(), 520, 400);

		wait(0.01);
	}

	W->stop(0.5);

	needHoldHandler = false;
	if(holdHandlerToPush.joinable()) {
		holdHandlerToPush.join();
	}

	motorTurn->setPower(0);
}

bool needMove = true;

int err_length = 0;
int coordX;
uint32_t encInFrame;
int takeObj = 0;

int calcGrabVal(double start, double finish, double now, int servoStart, int servoFinish) {
	int res = (int)std::min( std::max( std::max((now - start) / (finish - start), 0.) * (servoFinish - servoStart) + servoStart,
			servoGrabMinVal * 1.0),
			servoGrabMaxVal * 1.0);
	res = std::max(std::min(servoStart, servoFinish), res);
	res = std::min(std::max(servoStart, servoFinish), res);
	return res;
}

int calcGrabPerpAng(double manipAng, int grabAng){
	return std::max(std::min((int)(servoGrabClosedPerpVal + manipAng + grabAng), servoGrabMaxVal), servoGrabMinVal);
}

void turnManip(double manipAng, int grabAng, double turnSpeed) {
	motorTurn->turnAng(manipAng, turnSpeed, [&]()->uint8_t {
		servoGrab->setAngle(calcGrabPerpAng(fabs(motorTurn->getAngleVal()), grabAng));
		wait(0.01);
		return fabs(motorTurn->getAngleVal() - manipAng) < 2 ? 1 : 0; });
}

void takeObjFromFloor() {
	int count_fps = 0;
	double time_fps = time();

	servoGrab->setAngle(servoGrabCatchFloorVal);

	robot::vision::CvPoint<int> coord { 0, 0 };
	//motorShift->setPower(

	//useCam = true;

	//std::thread thFindObj(findObj);

	/*while(isObj == 0) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
							sen[SEN::FR_R]->aioReadNorm(), 250, 300);
		wait(0.01);

		/*if (++count_fps > 10) {
			message("fps = " << count_fps / (time() - time_fps));
			count_fps = 0;
			time_fps = time();
		}*/
	//}*/

	W->encRes();

	W->stop(0.1);
	/*if(thFindObj.joinable()) {
		thFindObj.join();
	}*/

	message("==================================\nfind obj");

	double t;
	int distEnc;
	/*while (time() - t < 0.5) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
									sen[SEN::FR_R]->aioReadNorm(), 500, 0);
		wait(0.001);
	}*/

	distEnc = static_cast<int>(W->length_enc(7));
	while (fabs(W->enc()) < distEnc) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
							sen[SEN::FR_R]->aioReadNorm(), 250, 400);
		wait(0.01);
	}

	t = time();
	while (time() - t < 0.5) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
									sen[SEN::FR_R]->aioReadNorm(), 600, 0);
		wait(0.001);
	}

	W->stop(0.1);

	int pos;
	bool readPos = false;

	int err_pix = 0;
	do {
		coord = cam1.handle_frame();

		if (coord.x == 0) {
			break;
		}

		err_pix = coord.x - shiftPixToGrip;

		if (!readPos) {
			readPos = true;
			if (err_pix > 0) {
				pos = 1;
			} else {
				pos = -1;
			}
		}

		motorShift->setPower(-err_pix * 1.5);
		wait(0.0001);

		if (++count_fps > 10) {
			message("fps = " << count_fps / (time() - time_fps) << "\nerr_pix = " << err_pix);
			//message("coord.x = " << coord.x);
			count_fps = 0;
			time_fps = time();
		}
	} while (fabs(err_pix) > 15);

	motorShift->setPower(0);
	wait(0.1, 0);

	message("err_pix = " << err_pix << "\ncatch moved");

	W->stop(0.1);

	W->encRes();
	uint32_t oldEncL = W->L_enc(), oldEncR = W->R_enc();
	uint32_t encL, encR;

	int minPowerL = 250, minPowerR = 250;
	W->ride(minPowerL, minPowerR);
	int newVal;

	/*std::thread moveToCatch([](double a, double b, int (* c)()) { W->moveStr(a, b, c); }, 50, 10,
			[]()->int { return (wait(0.1), !needMove); } );*/

	distEnc = abs(static_cast<int>(W->length_enc(2)));
	//W->moveStr(10, 10, []() {

	int dopEnc = 0;

	bool changeL = false, changeR = false;

	uint32_t encLimit = 1;

	/*do {
		wait(0.1);

		encL = W->L_enc();
		encR = W->R_enc();

		changeL = false;
		changeR = false;

		if (encL - oldEncL < encLimit) {
			minPowerL += 5;
			changeL = true;
		}

		if (encR - oldEncR < encLimit) {
			minPowerR += 5;
			changeR = true;
		}

		if (!changeL && !changeR) {
			++encLimit;
		}

		W->ride(minPowerL, minPowerR);
	} while (encLimit < 10);

	message("minPower L = " << minPowerL << " R = " << minPowerR);

	minPowerL += 50;
	minPowerR += 50;*/

	//W->ride(minPowerL, minPowerR);

	uint32_t oldEnc = W->enc();
	double oldTime = time();

	auto  oldCoord  = cam1.handle_frame();

	W->ride(300);

	std::thread coordXByCam([]() {
		robot::vision::CvPoint<int> coord;

		do {
			wait(0.0001);
			encInFrame = motorShift->getEncVal();
			coord = cam1.handle_frame();

			if (coord.x == 0) {
				message("coord is NULL!!");
				//coord = oldCoord;
				continue;
			} else {
				coordX = coord.x;
			}

			if (!takeObj && coord.y >= shiftYObjIsGrip) {
			//dopEnc = abs(static_cast<int>(W->enc()));
				message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\ny = " << coord.y << "\nx = " << coord.x);

				takeObj = true;
			}
		} while (!takeObj);
	} );

	do {
		wait(0.015);

		double target = motorShift->enc_length(encInFrame) - (coordX - shiftPixToGrip) / pixInCm;
		if (target < -8) {
			target = -8;
		} else if (target > 16) {
			target = 16;
		}

		motorShift->holdTargetLength(motorShift->enc_length(encInFrame) - (coordX - shiftPixToGrip) / pixInCm);

		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
								sen[SEN::FR_R]->aioReadNorm(), 500, 300);


		//double err_length = coord.x - shiftPixToGrip;
		//motorShift->setPower(-err_length * 4);

	} while(!takeObj); //|| abs(static_cast<int>(W->enc())) < distEnc + dopEnc);

	motorShift->setPower(0);
	W->stop();
	needMove = false;

	/*if (moveToCatch.joinable()) {
		moveToCatch.join();
	}*/

	if (coordXByCam.joinable()) {
		coordXByCam.join();
	}

	wait(0.1, 0);
	message("x = " << coord.x);
	message("coord.y = " << coord.y);
}

void rideFromBaze() {
	//servoBlock->setAngle(servoBlockUpVal);
	//servoGrab->setAngle(servoGrabCatchFloorVal);
	/*int c = 0;
	while(1) {
		std::cin >> c;
		message("R ang - " << W->R_ang() << "L ang - " << W->L_ang());
		wait(0.1);
	}*/
	needHoldHandler = true;
	//std::thread turnOutHandler(turnHandler, -200, 0.05);

	W->moveStr(10, 10, []()->uint8_t {return (W->L()->getLengthVal() + W->L()->getLengthVal()) / 2 > 9.5 ? 1 : 0;});
	W->stop();
	W->encRes();
	W->turnAng(15, 10, []()->uint8_t {return fabs(W->enc_robotDegr(W->L_enc())) > 14.5? 1 : 0;});
	W->encRes();

	W->initLight(80, 40, sen, 50); // < 90
	message("Initied light");
	/*while(!onLine(SEN::MID_L) && !onLine(SEN::MID_R)) {
		W->rideStr(230, 10000);
		wait(0.01);
	}*/

	//W->stop();

	//message("Inited");

	W->L()->setPID_angle(40, 45, 12);
	W->R()->setPID_angle(40, 45, 12);
	moveToLine(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, SEN::MID_L, SEN::MID_R, 300, 25, 1);
	W->encRes();
	turnToLine(1, 1, 1, 70);

	W->encRes();
	double t = time();
	while (time() - t < 0) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
									sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}

	W->encRes();
	/*W->L()->setPID_angle(10, 0, 0);
	W->R()->setPID_angle(10, 0, 0);
	*/
	double oldT;
	t = time();
	oldT = t;
	//message(W->L_ang() << W->R_ang());
	while (!onLine((uint8_t)SEN::MID_R) || !onLine((uint8_t)SEN::MID_L)) {
		W->followLineSpeed(sen[SEN::FR_L]->aioReadNorm(),
				sen[SEN::FR_R]->aioReadNorm(), 25, 15, time() - oldT);
		oldT = time();
		wait(0.01);
	}

	W->L()->setPID_angle(35, 35, 10);
	W->R()->setPID_angle(35, 35, 10);
	W->encRes();

	moveToLine(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, SEN::MID_L, SEN::MID_R, 300, 25, 1);

	W->encRes();
	//followLineEnc(-1, 15, 10, 25);

	W->stop();
	needHoldHandler = false;
	/*if (turnOutHandler.joinable()) {
		turnOutHandler.join();
	}*/

	motorTurn->setPower(0);
	wait(0.1);
}

void rideFromBazeFor1Video(){
	W->initLight(50, 45, sen, 50); // < 90
	message("Inited light");

	W->encRes();
	moveToLine2(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, SEN::MID_L, SEN::MID_R, 350, 45);
	W->encRes();

	double t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();


	turnToLine(-1, 1, 1, 80);
	W->encRes();


	double oldT = time();
	/*W->encRes();
	while (!onLine(SEN::MID_R) || !onLine(SEN::MID_L)) {
		W->followLineSpeed(sen[SEN::FR_L]->aioReadNorm(),
				sen[SEN::FR_R]->aioReadNorm(), 15, 30, time() - oldT);
		oldT = time();
		wait(0.01);
	}
	W->stop();
	W->encRes();*/

	followLineEnc(1, 20, 30, 25);
	W->stop();
	W->encRes();

	moveToLine2(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, SEN::MID_L, SEN::MID_R, 350, 45);
	W->encRes();

	t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();

	turnToLine(1, 2, 1, 80);
	W->encRes();

	followLineEnc(1, 20, 40, 50);
	W->stop();
	W->encRes();
	t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
			sen[SEN::B_R]->aioReadNorm(), -250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();

	wait(3); // Up to 5!!

	oldT = time();
	while (!onLine((uint8_t)SEN::MID_R) || !onLine((uint8_t)SEN::MID_L)) {
		W->followLineSpeed(sen[SEN::B_L]->aioReadNorm(),
				sen[SEN::B_R]->aioReadNorm(), 15, -40, time() - oldT);
		oldT = time();
		wait(0.01);
	}
	W->stop();
	W->encRes();

	W->L()->setPID_angle(20, 30, 5);
	W->R()->setPID_angle(20, 30, 5);

	moveToLine2(SEN_POS::AFT_ON, SEN_POS::AFT_ON, SEN::MID_L, SEN::MID_R, 350, 45);

	t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();

	turnToLine(-1, 2, 1, 80);
	W->encRes();

	followLineEnc(1, 20, 40, 100);
	W->stop();
	W->encRes();

	/*while (!onLine(SEN::MID_R) || !onLine(SEN::MID_L)) {
		W->followLineSpeed(sen[SEN::FR_L]->aioReadNorm(),
				sen[SEN::FR_R]->aioReadNorm(), 5, 40, time() - oldT);
		oldT = time();
		wait(0.01);
	}
	W->stop();
	W->encRes();
	moveToLine(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, SEN::MID_L, SEN::MID_R, 230, 45);
	W->stop();
	W->encRes();

	followLineEnc(1, 20, 30, 60);
	W->stop();
	W->encRes();*/

	wait(1); // up to 5!!

	W->encRes();
	std::function<uint8_t()> funcOn = [&]()->int { return onLine((uint8_t)SEN::MID_R) || onLine((uint8_t)SEN::MID_L); };
	W->turnAng(130, 60, funcOn);//[]()->int { return /*onLine(SEN::FR_L)*/ 0; });
	W->stop();
	W->encRes();
	double nowTime = time();
	while(time() - nowTime < 1) {
		W->ride(400 * (0.5 - sen[SEN::MID_L]->aioReadNorm()), 400 * (0.5 - sen[SEN::MID_R]->aioReadNorm()));
		wait(0.01);
	}
	W->stop();
	W->encRes();

	W->moveStr(-50, 20, []()->int {return 0;});
	W->encRes();

	W->turnAng(-90, 40, []()->int {return 0;});
	W->encRes();

	W->moveStr(-50, 30, []()->int {return 0;});
	W->encRes();

	W->ride(-600, -700);
	wait(5);
	W->ride(-400, -500);
	wait(2);
	W->stop();
	W->encRes();
}

void rideFromBazeToSecondLine(){
	W->moveStr(10, 30, []()->uint8_t {return (W->L()->getLengthVal() + W->L()->getLengthVal()) / 2 > 9.5 ? 1 : 0;});
	W->stop();
	W->encRes();
	W->turnAng(15, 30, []()->uint8_t {return fabs(W->enc_robotDegr(W->L_enc())) > 14.5? 1 : 0;});
	W->stop();
	W->encRes();

	W->initLight(60, 60, sen, 80); // < 90
	message("Inited light");

	W->encRes();
	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, false, 1);
	W->encRes();

	double t = time();

	turnToLine(1, 1, 1, 80, false);
	W->encRes();

	/*t = time();
	double oldT = t, nowT = t;
	while (nowT - t < 20) {
		W->followLineSpeed2(sen[SEN::FR_L]->aioReadNorm(), sen[SEN::FR_R]->aioReadNorm(), 0.03, 20, nowT - oldT);
		wait(0.01);
		oldT = nowT;
		nowT = time();
	}*/
	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, true, -1);
	W->stop();
	W->encRes();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
}

void rideFromBazeFor3Video() {
	W->moveStr(10, 30, []()->uint8_t {return (W->L()->getLengthVal() + W->L()->getLengthVal()) / 2 > 9.5 ? 1 : 0;});
	W->stop();
	W->encRes();
	W->turnAng(15, 30, []()->uint8_t {return fabs(W->enc_robotDegr(W->L_enc())) > 14.5? 1 : 0;});
	W->stop();
	W->encRes();

	W->initLight(70, 60, sen, 80); // < 90
	message("Inited light");

	/*while (1) {
		message("l - " << sen[SEN::FR_L]->aioReadNorm());
		message("r - " << sen[SEN::FR_R]->aioReadNorm());
		wait(0.5);
	}*/

	W->encRes();
	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, true, 1);
	W->encRes();

	double t = time();

	turnToLine(1, 1, 1, 80, false);
	W->encRes();

	/*t = time();
	double oldT = t, nowT = t;
	while (nowT - t < 20) {
		W->followLineSpeed2(sen[SEN::FR_L]->aioReadNorm(), sen[SEN::FR_R]->aioReadNorm(), 0.03, 20, nowT - oldT);
		wait(0.01);
		oldT = nowT;
		nowT = time();
	}*/
	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, true, -1);
	W->stop();
	W->encRes();
	followLineEnc(1, 0.03, 25, 30); //72.5
	W->encRes();
	/*while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 300, 0);
		wait(0.001);
	}*/
	W->stop();
}

void rideToPushFor2Video(int line) { // 0 - first line, 1 - second line
	servoBlock->setAngle(servoBlockDownVal);
	wait(0.5);
	std::thread upManip([]() {
		double ang = -120;
		motorTurn->turnAng(ang, 60, [&]()->uint8_t {servoGrab->setAngle(
					calcGrabVal(0, ang, (motorTurn->getAngleVal()), servoGrabCatchedVal, servoGrabCatchedManipUp));
				return 0;});
			//servoBlock->setAngle(servoBlockUpVal);
		//wait(1);
		//motorShift->moveStr(0, 10, []()->uint8_t {return 0;});
	});
	double t = time();
	/*while (time() - t < 1) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
			sen[SEN::B_R]->aioReadNorm(), -250, 0);
		wait(0.001);
	}*/
	W->stop();
	W->encRes();

	double oldT = time();
	/*while (!onLine(SEN::MID_R) || !onLine(SEN::MID_L)) {
		W->followLineSpeed(sen[SEN::B_L]->aioReadNorm(),
				sen[SEN::B_R]->aioReadNorm(), 20, -30, time() - oldT);
		oldT = time();
		wait(0.01);
	}
	W->stop();
	W->encRes();*/

	//moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, 1);


	/*W->L()->setPID_angle(35, 35, 10);
	W->R()->setPID_angle(35, 35, 10);

	moveToLine(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, SEN::MID_L, SEN::MID_R, 300, 20);
	W->stop();
	W->encRes();*/

	/*t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();*/

	turnToLine(-1, 2, 1, 70, true, true);
	W->stop();
	W->encRes();

	if (upManip.joinable()) {
		upManip.join();
	}
	W->stop();
	W->encRes();

	followLineEnc(1, 0.03, 15 + line == 0 ? 10 : 0, 13 + line == 0 ? 80 : 0);
	W->stop();
	W->encRes();

	/*oldT = time();
	while (!onLine(SEN::MID_R) || !onLine(SEN::MID_L)) {
		W->followLineSpeed(sen[SEN::B_L]->aioReadNorm(),
				sen[SEN::B_R]->aioReadNorm(), 20, -30, time() - oldT);
		oldT = time();
		wait(0.01);
	}
	W->stop();
	W->encRes();*/

	//W->L()->setPID_angle(35, 35, 10);
	//W->R()->setPID_angle(35, 35, 10);

	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, 1);
	W->encRes();

	t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(),
			sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();
}

void moveAfterPush(int line) { // 0 - first line, 1 - second line
	std::thread returnShift ([]() { motorShift->moveStr(4, 6, []()->uint8_t {return 0;});});
	double t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
									sen[SEN::B_R]->aioReadNorm(), -250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();
	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, 1);
	if (line == 0) {
		W->stop();
		W->encRes();
		followLineEnc(-1, 0.03, 20, 10);
		W->stop();
		W->encRes();
		moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, 1);
	}

	W->encRes();
	turnToLine(1, 2, 1, 30);
	W->stop();
	W->encRes();
	followLineEnc(1, 0.03, 20, 8);
	W->stop();
	W->encRes();
	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, -1);
 	if (returnShift.joinable()) {
 		returnShift.join();
 	}

}


void pushObj () {
	std::thread shiftBrick([]() {motorShift->moveStr(nowBrick->getShiftPos(), 5, []()->uint8_t {return 0;});});

	servoBlock->setAngle(servoBlockDownVal);
	motorTurn->setPID_angle(120, 190, 40);
	double ang = -120;
	/*motorTurn->encRes();
	motorTurn->setPower(0);
	motorTurn->turnAng(ang, 20, [&]()->uint8_t {servoGrab->setAngle(
			calcGrabVal(0, ang, (motorTurn->getAngleVal()), servoGrabCatchedVal, servoGrabCatchedManipUp));
		return 0;});
	//motorTurn->turnAng(ang, 50, [&]()->uint8_t {servoGrab->setAngle( calcGrabPerpAng(motorTurn->getAngleVal(), -90);
	//	return 0;});
	*/
	//W->L()->setPID_angle(30, 40, 8);
	//W->R()->setPID_angle(30, 40, 8);
	followLineEnc(1, 0.03, 15, nowBrick->getmoveDist());
	ang = -30;
	for (int grabAng = servoGrabCatchedManipUp;
			fabs(grabAng - calcGrabPerpAng(fabs(motorTurn->getAngleVal()), -95)) > 1; grabAng++) {
		servoGrab->setAngle(grabAng);
		wait(0.04);
	}

	if (shiftBrick.joinable()) {
		shiftBrick.join();
	}
	ang = -90;
	turnManip(ang, -94, 15);
	ang = nowBrick->getManipAng();
	turnManip(ang, -80, 25);
	servoGrab->setAngle(calcGrabPerpAng(fabs(ang), -85));
		wait(0.5);
	motorTurn->setPower(100);
	wait(0.02);
	motorTurn->setPower(0);
	wait(0.1);
	W->encRes();
	followLineEnc(1, 0.03, 2, 3);

	W->stop();
	W->encRes();

	followLineEnc(-1, 0.03, 4, 8);
	W->stop();
	W->encRes();
	//wait(0.5);

	/*motorShift->moveStr(2, 2, [&]()->uint8_t {motorTurn->holdTargetAngle(ang);
		return 0;});
	motorTurn->setPower(0);

	followLineEnc(1, 8, 2, 3);
	W->stop();
	W->encRes();

	W->L()->setPID_angle(30, 40, 8);
	W->R()->setPID_angle(30, 40, 8);
	double startTime = time();
	while (time() - startTime < 1.5) {
		motorTurn->holdTargetAngle(ang);
		wait(0.01);
	}
	W->moveStr(10, 5, [&]()->uint8_t {motorTurn->holdTargetAngle(ang);
		return time() - startTime > 3 ? 1 : 0;});
	W->stop();
	W->encRes();

	followLineEnc(-1, 8, 2, 6);
	W->stop();
	W->encRes();*/

	//W->L()->setPID_angle(37, 50, 10);
	//W->R()->setPID_angle(37, 50, 10);
	motorTurn->setPower(100);
	wait(0.01);
	motorTurn->setPower(0);
	wait(0.5);
	motorTurn->encRes();
	numOfBricks[nowBrick->getType()]++;
	delete nowBrick;
}

void rideToBazeFor2Video() {
	std::thread closeManip ([]() {
		motorShift->moveStr(0, 4, []()->uint8_t {return 0;});
		motorShift->setPower(0);
		motorTurn->turnAng(-210, 60, []()->uint8_t {return 0;});
		wait(0.5);
		servoBlock->setAngle(servoBlockUpVal);
		wait(0.5);
		servoGrab->setAngle(servoGrabStartVal);
		wait(1);
	});

	W->encRes();
	std::function<uint8_t()> funcOn = [&]()->int { return onLine((uint8_t)SEN::MID_R) || onLine((uint8_t)SEN::MID_L); };
	W->turnAng(130, 60, funcOn);//[]()->int { return /*onLine(SEN::FR_L)*/ 0; });
	W->stop();
	W->encRes();
	double nowTime = time();
	while(time() - nowTime < 1) {
		W->ride(400 * (0.5 - sen[SEN::MID_L]->aioReadNorm()), 400 * (0.5 - sen[SEN::MID_R]->aioReadNorm()));
		wait(0.01);
	}
	W->stop();
	W->encRes();

	W->moveStr(-50, 20, []()->int {return 0;});
	W->encRes();

	W->turnAng(-90, 40, []()->int {return 0;});
	W->encRes();

	if (closeManip.joinable()) {
		closeManip.join();
	}

	W->moveStr(-50, 30, []()->int {return fabs(W->L()->getLengthVal() + W->R()->getLengthVal()) / 2 > 49 ? 1 : 0;});
	W->encRes();

	W->ride(-650, -700);
	wait(5);
	W->ride(-450, -500);
	wait(2);
	W->stop();
	W->encRes();
}

void tryCatchAgain() {
	int grabPos = servoGrabCatchFloorVal;
	double ang = -15;
	for (;grabPos > servoGrabCatchFloorVal - 15; grabPos --){
		servoGrab->setAngle(grabPos);
		//motorTurn->holdTargetAngle(ang);
		wait(0.01);
	}

	for (;grabPos < servoGrabCatchFloorVal + 15; grabPos ++){
		servoGrab->setAngle(grabPos);
		//motorTurn->holdTargetAngle(ang);
		wait(0.01);
	}

	motorTurn->setPower(0);
	servoGrab->setAngle(servoGrabCatchFloorVal);
	wait(0.5);
}

void turnDownManip(double ang = -40) {
	motorTurn->turnAng(219, 40, [&]()->uint8_t {return motorTurn->getAngleVal() > 205 ? 1 : 0;});

	motorTurn->setPower(100);
	wait(0.5);
	motorTurn->setPower(0);
	motorTurn->encRes();

	motorTurn->turnAng(ang, 30, []()->uint8_t {return 0;});
	motorTurn->holdTargetAngle(ang);
	motorTurn->PIDRes();
}

std::mutex t_lock;

void holdingManip (double ang, bool & stopHolding, bool mess = false) {
	while (1) {
		motorTurn->holdTargetAngle(ang);
		wait(0.01);
		if (stopHolding) {
			break;
		}
		if (mess) {
			message("Manip ang - " << motorTurn->getAngleVal());
		}
	}
	motorTurn->setPower(0);
	return;
}

bool takeObjFromChute( double  dist = 99, int numOfObj = 0 ) {
	// return indicates can we put brick that we took (true) or we left it (false)
	double ang = -54;
	motorTurn->turnAng(ang, 10, [&]()->uint8_t {return fabs(ang - motorTurn->getAngleVal()) < 0.4 ? 1 : 0;});
	servoBlock->setAngle(servoBlockCatchChuteVal);
	bool stopHolding = false;
	//servoBlock->setAngle(servoBlockCatchChuteVal);
	//std::ref stopHoldRef = new std::ref(stopHolding);
	std::thread holdManip(holdingManip, ang, std::ref(stopHolding), false);
	W->encRes();
	followLineEnc(1, 0.03, 20, dist);
	wait(1.5);
	motorShift->moveStr(-8, 4, []()->uint8_t {return 0;});
	//followLineEnc(-1, 20, 5, 1);
	/*W->encRes();
	W->moveStr(-4.6, 1.5, []()->uint8_t {return fabs(W->L()->getLengthVal()) > 1.5 ? 1 : 0;});
	W->stop();*/
	W->encRes();
	motorShift->moveStr(4.8 + 4.2 + motorShift->getLengthVal(), 4, []()->uint8_t {return 0;});//-1.6
	W->stop();

	t_lock.lock();
	stopHolding = true;
	t_lock.unlock();

	if (holdManip.joinable()) {
		holdManip.join();
	}
	/*double T = time();
	message("Start turn down");
	motorTurn->setPID_angle(3, 5, 3);
	motorTurn->turnAng(0, 6, [&]()->uint8_t {servoGrab->setAngle(
			calcGrabVal(ang, 0, motorTurn->getLastTarget() + 3, servoGrabStartCatchVal, servoGrabCatchFloorVal));
		wait(0.01);
		return time() - T > 10 ? 1 : 0;});//motorTurn->getAngleVal() > -ang + 2 ? 0 : 1;});
	motorTurn->encRes();
	motorTurn->PIDRes();*/
	ang = -35;
	motorTurn->turnAng(ang, 10, []()->uint8_t {return 0;});
	servoGrab->setAngle(servoGrabCatchChuteVal);
	wait(0.5);


	motorTurn->setPower(200);
	wait(0.3);
	motorTurn->setPower(0);

	motorTurn->setPID_angle(100, 100, 40);
	tryCatchAgain();

	/*motorTurn->setPower(100);
	wait(0.2);
	motorTurn->setPower(0);
	*/
	servoBlock->setAngle(servoBlockCatchChuteVal - 10);
	wait(0.5);
	double moveDist = -12;
	W->encRes();
	W->moveStr(moveDist, 6, [&]()->uint8_t { double dist = W->L()->getLengthVal() + W->R()->getLengthVal() / 2;
		servoGrab->setAngle( calcGrabVal(0, moveDist, dist, servoGrabCatchFloorVal, servoGrabCatchedVal));
		/*if (fabs (dist) > fabs(moveDist) / 2) {
			servoBlock->setAngle((servoBlockDownVal + servoBlockCatchChuteVal) / 2);
		}*/

		return 0;});
	motorTurn->setPID_angle(200, 250, 50);
	motorTurn->PIDRes();
	checkObj();
	if (!checkBrickIsPossible(nowBrick->getType())) {
		message("Impossible to put brick");
		std::thread upManipToLeave ([&]() { motorTurn->turnAng(-30, 30, []()->uint8_t {
				return 0;} );});
		followLineEnc(-1, 0.03, 25, 95);
		if (upManipToLeave.joinable()) {
			upManipToLeave.join();
		}

		servoGrab->setAngle(servoGrabClosedPerpVal + 10);
		wait(0.5);
		servoGrab->setAngle(servoGrabStartVal);

		std::thread downManip ([&]() { wait (0.5);
			motorTurn->setPower(200);
			wait(0.2);
			motorTurn->setPower(0);	} );

		turnToLine(-1, 1, 1, 80, false, true);

		if (downManip.joinable()) {
			downManip.join();
		}

		numBricksOnLines[1] = 1;
		lastBricksOnLines[1] = nowBrick->getType();
	}

	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, -1);
	return checkBrickIsPossible(nowBrick->getType());
}

bool freeLine (int numBr) {
	message("Sarting freeing line");
	servoGrab->setAngle(servoGrabStartVal);
	motorShift->moveStr(2, 6, []()->uint8_t {return 0;});
	for (int i = 0; i < numBr; i++) {
		W->turnAng(90, 30, []()->uint8_t {return fabs(W->enc_robotDegr(W->L_enc())) > 89? 1 : 0;});
		W->stop();
		W->encRes();
		turnToLine(-1, 1, 1, 80, false, true);
		followLineEnc(1, 0.03, 20, 27);
		W->encRes();
	}

	numBricksOnLines[1] = 0;
	return takeObjFromChute( 99 - (27 * 3 + numBr == 3 ? 0 : 3) );
}

void calibrateObjFromChute() {
	motorShift->encRes();
	motorShift->setICompToZero(false);
	motorTurn->setICompToZero(true);
	motorTurn->setPID_angle(200, 220, 20);
	motorShift->setPID_angle(20, 50, 20);

	servoGrab->setAngle(servoGrabStartVal);
	downServoBlock();
	motorTurn->turnAng(219, 40, [&]()->uint8_t {return motorTurn->getAngleVal() > 200 ? 1 : 0;});

	motorTurn->setPower(100);
	wait(0.1);
	motorTurn->setPower(0);
	wait(0.5);
	//wait(0.2);

	/*std::thread moveShift([]() {	motorShift->moveStr(2, 2, []()->uint8_t {return 0;});
		motorShift->setPower(0);} );*/
	motorShift->moveStr(4, 2, []()->uint8_t {return 0;});
	motorShift->setPower(0);
	motorTurn->encRes();
	//servoBlock->setAngle(servoBlockCatchChuteVal);

}

bool calibrateObj(int numOfObj = 0) { // return indicates can we put brick that we took (true) or we left it (false)

	bool stopHolding = false;

	motorShift->setICompToZero(false);
	motorTurn->setICompToZero(false);
	//motorTurn->setPID_angle(200, 220, 20);
	motorShift->setPID_angle(60, 70, 10);

    //std::thread downManip(turnDownManip, -40);
	double ang = -30;

	std::thread turnManip([&]() {motorTurn->turnAng(ang, 30, [&]()->uint8_t {
		/*if (numOfObj == 1) {
			message("Ang - " << motorTurn->getAngleVal());
		}*/
		return fabs(motorTurn->getAngleVal() - ang) < 2 ? 1 : 0;});
		holdingManip(ang, std::ref(stopHolding), false );
		message("ended holding");});
	W->stop();
	W->encRes();
	double nowTime = time();
	while(time() - nowTime < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(), sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.01);
	}
	W->stop();
	W->encRes();
	followLineEnc(1, 0.03, 20, 27 * numOfObj);
	W->stop();
	W->encRes();
	//std::thread holdManip(holdingManip, ang, std::ref(stopHolding));
	double startT = time(), t = time();
	/*motorShift->moveStr(-1, 5, []()->uint8_t {return 0;});
	W->encRes(); */

	//followLineEnc(1, 7, 8, 15);
	W->encRes();
	double startShift = -2, finishShift = -8;
	motorShift->moveStr(startShift, 8, [&]()->uint8_t {return fabs(startShift - motorShift->getLengthVal()) > 0.1 ? 0 : 1;});
	//motorShift->moveStr(-8, 5, []()->uint8_t {return 0;});
	double dist = 15;
	W->moveStr(dist, 7, [&]()->uint8_t { double shiftPos = std::max(std::min(((W->L()->getLengthVal() + W->R()->getLengthVal())
			/ 2 / dist * (finishShift - startShift) + startShift), motorShiftLeftEdge), motorShiftRightEdge);
		shiftPos = shiftPos * 2 / motorShift->wheelDiam / 3.1415 * 180;
		motorShift->holdTargetAngle(shiftPos, 0, -1);
		message("Pos - " << shiftPos / 2 * motorShift->wheelDiam * 3.1415 / 180
				<< ", Dist - " << (W->L()->getLengthVal() + W->R()->getLengthVal()) / 2 << ", Shift - " <<
				motorShift->getLengthVal());
		wait(0.02);
		return (W->L()->getLengthVal() + W->R()->getLengthVal()) > dist * 2 - 0.5 ? 1 : 0;});
	W->encRes();
	motorShift->setPower(0);

	//findObj();
	//wait(3.5);
	message("Shift finish - " << motorShift->getLengthVal());
	motorShift->moveStr(4.8 + 4.0 + motorShift->getLengthVal(), 4, []()->uint8_t {return 0;});
	message("Shift finish 2 - " << motorShift->getLengthVal());

	t_lock.lock();
	stopHolding = true;
	t_lock.unlock();
	/*if (holdManip.joinable()) {
		holdManip.join();
	}*/
	if (turnManip.joinable()) {
		turnManip.join();
	}
	//double ang = -40;
	double T = time();
	/*servoGrab->setAngle(servoGrabCatchFloorVal + 1);
	wait(0.5);

	motorTurn->setPower(200);
	wait(0.5);
	motorTurn->setPower(600);
	wait(0.5);
	motorTurn->setPower(0);*/

	ang = motorTurn->getAngleVal();
	message("Start turn down");
	motorTurn->turnAng(0, 12, [&]()->uint8_t {servoGrab->setAngle(
			calcGrabVal(ang, 0, motorTurn->getLastTarget(), servoGrabStartVal, servoGrabCatchFloorVal));
		wait(0.01);
		//message("Manip ang - " << motorTurn->getAngleVal());
		/*message("P - " << (motorTurn->oldErrDegr * motorTurn->P_angle) << ", I - " << motorTurn->sumErrAng
				<< ", D - " << ((motorTurn->pwm_power->getPwmWrite() - 0.5 - motorTurn->minPwmToSpin) * 1000 /
				(1000 - motorTurn->minPwmToSpin)
				- motorTurn->oldErrDegr * motorTurn->P_angle -
				motorTurn->sumErrAng) * motorTurn->D_angle << ", Angle - " << motorTurn->getAngleVal() << ", Goal - " << motorTurn->getLastTarget());
		*/
		return time() - T > 4 ? 1 : 0;});//motorTurn->getAngleVal() > -ang + 2 ? 0 : 1;});
	//motorTurn->encRes();
	motorTurn->setPID_angle(100, 100, 40);
	tryCatchAgain();

	/*followLineEnc(1, 6, 8, 15);
	W->stop();
	W->encRes();*/
	//W->moveStr(9, 15, []()->uint8_t{return 0;});
	/*servoGrab->setAngle(servoGrabCatchFloorVal);
	wait(1);
	motorTurn->setPower(500);
	wait(0.5);
	motorTurn->encRes();*/
	double moveDist = -13;
	W->moveStr(moveDist, 6, [&]()->uint8_t {servoGrab->setAngle(
			calcGrabVal(0, moveDist, (W->L()->getLengthVal() + W->R()->getLengthVal() / 2), servoGrabCatchFloorVal, servoGrabCatchedVal));
	return 0;});
	motorTurn->setPID_angle(200, 250, 50);
	motorTurn->PIDRes();

	motorTurn->setPower(0);

 	checkObj();

 	numBricksOnLines[nowLine]--;
 	bool canPushObj = true;
	if (!checkBrickIsPossible(nowBrick->getType())) {

		if (nowLine == 0) {
			std::thread shiftToLeave ([&]() {motorShift->moveStr(motorShift->getLengthVal() + 7, 6, []()->uint8_t {return 0;});});
			followLineEnc(-1, 0.03, 20, 27 * numOfObj + 3);
			W->encRes();
			double ang = -30;
			motorTurn->turnAng(ang, 30, [&]()->uint8_t {return 0;});
			if (shiftToLeave.joinable()) {
				shiftToLeave.join();
			}
			servoGrab->setAngle(servoGrabClosedPerpVal + 10);
			wait(0.5);
			servoGrab->setAngle(servoGrabStartVal);
			return false;
		}


		else {
			std::thread upManipToLeave ([&]() { motorTurn->turnAng(-30, 30, []()->uint8_t {
					return fabs(-30 - motorTurn->getAngleVal()) < 1 ? 1 : 0;} );});
			W->turnAng(90, 30, []()->uint8_t {return fabs(W->enc_robotDegr(W->L_enc())) > 89? 1 : 0;});
			W->stop();
			W->encRes();
			if (upManipToLeave.joinable()) {
				upManipToLeave.join();
			}

			servoGrab->setAngle(servoGrabClosedPerpVal + 10);
			wait(0.5);
			servoGrab->setAngle(servoGrabStartVal);

			std::thread downManip ([&]() { wait (0.5);
				motorTurn->setPower(200);
				wait(0.2);
				motorTurn->setPower(0);	} );

			turnToLine(-1, 1, 1, 80, false, true);

			if (downManip.joinable()) {
				downManip.join();
			}

			followLineEnc(1, 0.03, 20, 27);
			canPushObj = freeLine(2 - numOfObj);
		}
	}

	W->encRes();
	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, 1);
	return canPushObj;
}


void displayCoord() {
	while(1) {
 		wait(0.01);
 		/*robot::vision::CvPoint<int>*/
 		auto coord = cam1.handle_frame();
 		message("x = " << coord.x);
 		message("y = " << coord.y);
 	}
}

void rideAfterPush() {
	double t = time();
	while (time() - t < 2) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
									sen[SEN::B_R]->aioReadNorm(), -1200, 0);
		wait(0.001);
	}

	while (!onLine((uint8_t)SEN::MID_R) || !onLine((uint8_t)SEN::MID_L)) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
				sen[SEN::B_R]->aioReadNorm(), -520, -700);
		wait(0.01);
	}

	moveToLine(SEN_POS::AFT_ON, SEN_POS::AFT_ON, SEN::MID_L, SEN::MID_R, 250, 25);
	W->stop(0.1);
}

void takeInHandler() {
	message("takeInHandler");
	shiftHandler(0);
}

void shiftHandler(double dist) {
	int distInEnc = motorShift->length_enc(dist);
	int limit = abs(motorShift->length_enc(0.3));

	while(abs(static_cast<int>(motorShift->getEncVal()) - distInEnc) > limit) {
		motorShift->holdTargetLength(dist);
		wait(0.01);
	}

	double t = time();
	while(time() - t < 1) {
		motorShift->holdTargetLength(dist);
		wait(0.01);
	}

	motorShift->setPower(0);
	wait(0.1, 0);
}

void moveHandlerRightToPush() {
	needHoldHandler = true;
	std::thread holdHandlerAngl(turnHandler, 100, 0.02);

	shiftHandler(-9);

	needHoldHandler = false;
	if(holdHandlerAngl.joinable()) {
		holdHandlerAngl.join();
	}
}

void push() {
	motorTurn->setPower(0);
	servoGrab->relax();
	turnHandler(5, 0.02);
	motorTurn->setPower(0);
	wait(0.1, 0);
}

void displaySensors() {
	while(1) {
		wait(0.25);
		message("\nFR_L = " << sen[SEN::FR_L]->aioRead() << "   FR_R = " << sen[SEN::FR_R]->aioRead() << "  |  MID_L = " << sen[SEN::MID_L]->aioRead() << "   MID_R = " << sen[SEN::MID_R]->aioRead() << "  |  B_L = " << sen[SEN::B_L]->aioRead() << "   B_R = " << sen[SEN::B_R]->aioRead());
	}
}

//W->turnAng(90, 50);
 	//W->stop(10);

 		/*servoBlock->setAngle(2);
 		wait(2);*/

 		/*message("Mot Shift +");
 		wait(1);
 		motorShift->setPower(1000);
 		wait(1.5);
 		message("motorShift length " << motorShift->getLengthVal());
 		message("motorShift enc " << motorShift->getEncVal());
 		motorShift->setPower(0);
 		wait(2);
 		message("Mot Shift -");
 		wait(1);
 		motorShift->setPower(-500);
 		wait(1.5);
 		motorShift->setPower(0);*/

 		/*while(fabs(motorTurn->getAngleVal()) < 15) {
 			motorTurn->holdTargetAngle(15, 0);
 		}
 		double timeStart = time();
 		while(time() - timeStart < 2) {
 			motorTurn->holdTargetAngle(15, 0);
 		}*/

 	//W->moveStr(75, power, []()->int { return onLine(SEN::MID_L) || onLine(SEN::MID_R); } );
 	//message("MID R " << sen[MID_R]->aioReadNorm() << " MID L " << sen[MID_L]->aioReadNorm());
 	//message("L sensor -- " << LVal << ", R sensor -- " << RVal);

 	//auto s = []() { return LSen->aioReadNorm() < 0.5 ? 0 : 1; };
 	//W->turn(-300);
 	//W->stop(3);

 	/*message("start_turn");
 	W->encRes();
 	W->turnAng(130, 50, []() { return onLine(SEN::FR_L); } );//[]() { return LSen->aioReadNorm() < 0.5 ? 0 : 1; });
 	message(sen[static_cast<int>(SEN::FR_L)]->aioReadNorm());
 	message("finish_turn");*/

 	//W->turnAng(-130, 50, []() { return onLine(SEN::FR_L); } );

 	/*int t = time();

 	while (time() - t < 0.7) {
		W->followLine(sen[static_cast<int>(SEN::FR_L)]->aioReadNorm(), sen[static_cast<int>(SEN::FR_R)]->aioReadNorm(),
					500, 0);
	}*/


void loopForPushingObjects () {
	for (int numOfPutBricks = 1; numOfPutBricks < 6; ++numOfPutBricks) {
		lastBricksOnLines[nowLine] = -1;
		bool takeFromChute = false;
		bool takeFromFloor = false;

		message("New brick start. num of put bricks - " << numOfPutBricks);
		message("First line - " << numBricksOnLines[0] << "bricks, last - " << lastBricksOnLines[0]);
		message("Second line - " << numBricksOnLines[1] << "bricks, last - " << lastBricksOnLines[1]);
		if (numBricksOnLines[1] != 0) {
			if (lastBricksOnLines[1] == -1 || checkBrickIsPossible(lastBricksOnLines[1])) {
				moveAfterPush(1);
				nowLine = 1;
				takeFromFloor = true;
				message("Go to second line and take from floor");
				message("---1 var");
			}

			else if (!everTookFromChute){
				moveAfterPush(1);
				freeLine(3);
				everTookFromChute = true;
				message("Go to second line, free it and take from chute");
				message("---2 var");
			}

			else {
				moveAfterPush(0);
				takeFromFloor = true;
				nowLine = 0;
				message("Go to first line");
				message("---3 var");
			}
		}

		else {
			if (lastBricksOnLines[0] == -1 || !checkBrickIsPossible(lastBricksOnLines[0])) {
				moveAfterPush(1);
				takeFromChute = true;
				nowLine = 1;
				message("Go to second line and take from chute");
				message("---4 var");
			}

			else {
				moveAfterPush(0);
				takeFromFloor = true;
				nowLine = 0;
				message("Go to first line");
				message("---5 var");
			}
		}
		/*if (numBricksOnLines[1] != 0) {
			if (lastBricksOnLines[1] == -1 || checkBrickIsPossible(lastBricksOnLines[1])) {
				moveAfterPush(1);
				nowLine = 1;
			}
			else {
				moveAfterPush(0);
				nowLine = 0;
			}
		}
		else if (numBricksOnLines[0] != 0 && (lastBricksOnLines[0] == -1 || checkBrickIsPossible(lastBricksOnLines[0]) )) {
			moveAfterPush(0);
			nowLine = 0;
		}
		else {
			moveAfterPush(1);
			takeFromChute = true;
		}*/

		message("Chosen line " << nowLine);
		bool isTakenOk;
		if (takeFromChute) {
			message("Take from chute");
			message("---.---1 var");
			isTakenOk = takeObjFromChute();
		}
		else if (takeFromFloor){
			int brickToTakeNum = lastBricksOnLines[nowLine] == -1 ? 3 - numBricksOnLines[nowLine] : 0;
			message("Take from floor, line - " << nowLine);
			message("---.---2 var, brick num - " << brickToTakeNum);
			isTakenOk = calibrateObj(brickToTakeNum);
		}

		if (isTakenOk) {
			message("Taken ok, going to push");
			message("---.---.---1 var");
			rideToPushFor2Video(nowLine);
			pushObj();
		}

		else if (nowLine == 0) {
			message("Bad brick on first line - program stopped");
			message("---.---.---2 var");
			delete nowBrick;
			return;
		}

		else {
			message("Taken brick that cannot be put now, going to another(first) line");
			message("---.---.---3 var");
			turnToLine(1, 2, 1, 80, true, true);
			followLineEnc(1, 0.03, 20, 10);
			W->encRes();
			moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, true, -1);
			turnToLine(-1, 2, 1, 80);
			followLineEnc(1, 0.03, 20, 8);
			W->encRes();
			moveToLine3(SEN_POS::AFT_OFF, SEN_POS::BEF_OFF, true, -1);
			nowLine = 0;
			int brickToTakeNum = lastBricksOnLines[nowLine] == -1 ? 3 - numBricksOnLines[nowLine] : 0;
			calibrateObj(brickToTakeNum);
			rideToPushFor2Video(nowLine);
			pushObj();
		}
	}

}

void rideFromBazeToSecondLineAlt() {
	W->moveStr(10, 30, []()->uint8_t {return (W->L()->getLengthVal() + W->L()->getLengthVal()) / 2 > 9.5 ? 1 : 0;});
	W->stop();
	W->encRes();
	W->turnAng(-15, 30, []()->uint8_t {return fabs(W->enc_robotDegr(W->L_enc())) > 14.5? 1 : 0;});
	W->stop();
	W->encRes();

	W->initLight(40, 60, sen, 80); // < 90
	message("Inited light");

	W->encRes();
	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, false, 1);
	W->encRes();

	double t = time();

	turnToLine(1, 1, -1, 80, false);
	W->encRes();

	/*t = time();
	double oldT = t, nowT = t;
	while (nowT - t < 20) {
		W->followLineSpeed2(sen[SEN::FR_L]->aioReadNorm(), sen[SEN::FR_R]->aioReadNorm(), 0.03, 20, nowT - oldT);
		wait(0.01);
		oldT = nowT;
		nowT = time();
	}*/
	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, -1);
	W->stop();
	W->encRes();
	turnToLine(-1, 2, 1, 80);
	W->stop();
	W->encRes();
	followLineEnc(-1, 0.03, 20, 10);
	W->stop();
	W->encRes();
	moveToLine3(SEN_POS::AFT_OFF, SEN_POS::AFT_OFF, true, -1);
	W->stop();
	W->encRes();
}

void takeSliceFromCube() {
	followLineEnc(-1, 0.03, 15, 10);
	W->stop();
	W->encRes();

	double ang = 270;
	motorCube->turnAng(ang, 30, [ang]()->uint8_t {return fabs(motorCube->getAngleVal() - ang) < 1 ? 1 : 0;});
	motorCube->setPower(0);

	double t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::FR_L]->aioReadNorm(), sen[SEN::FR_R]->aioReadNorm(), 250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();

	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, true, -1);
	W->stop();
	W->encRes();
}

void rideToPushSlice() {
	turnToLine(1, 2, -1, 80, true, true);
	W->stop();
	W->encRes();

	followLineEnc(-1, 0.03, 15, 13);
	W->stop();
	W->encRes();

	moveToLine3(SEN_POS::BEF_OFF, SEN_POS::BEF_OFF, true, -1);
	W->encRes();

	double t = time();
	while (time() - t < 1) {
		W->followLine(sen[SEN::B_L]->aioReadNorm(),
			sen[SEN::B_R]->aioReadNorm(), -250, 0);
		wait(0.001);
	}
	W->stop();
	W->encRes();
}

void pushSlice() {
	double ang = -200;

	motorCube->turnAng(ang, 40, [ang]()->uint8_t {return fabs(motorCube->getAngleVal() - ang) < 1 ? 1 : 0;});

	followLineEnc(-1, 0.03, 15, 55);
	W->stop();
	W->encRes();

}

int main(int argc, char **argv)
{
	constructor(argc, argv);
 	message("Constructed");
 	//findObj();
 	/*int c = 100;
 	while(1) {
 		//std::cin >> c;
 		//servoGrab->setAngle(c);
 		message("l Motor - " << W->L_ang());
 		message("r Motor - " << W->R_ang());
 		wait(2);
 	}*/
 	nowLine = 1;
 	std::thread catchPrepareAfterStart (calibrateObjFromChute);
 	rideFromBazeToSecondLineAlt();
 	if (catchPrepareAfterStart.joinable()) {
 		catchPrepareAfterStart.join();
 	}

 	takeSliceFromCube();
 	rideToPushSlice();
 	pushSlice();
 	//calibrateObj(0);

 	//rideToPushFor2Video(1);
 	//pushObj();
 	//numBricksOnLines[nowLine]--;
 	//loopForPushingObjects();

 	//rideToBazeFor2Video();

 	//W->moveStr(1, 2, [](){return 0;});

 	//W->L->holdTargetAngle();

 	//cam1.handle_frame();
 	//wait(2);

 	//robot::button::waitButtonPush();
 	//wait(2, 0);

 	//displaySensors();
 	//displayCoord();

 	//rideFromBaze();


 	//rideFromBazeFor1Video();


 	//calibrateObj();
 	//turnManip(45, -120, 25);
 	//rideToPush();
 	  //moveHandlerRightToPush();
 	/*message("start push");
 	push();
 	message("pushed");
 	rideAfterPush();
 	takeInHandler();
	*/
 	return distructor();
}
