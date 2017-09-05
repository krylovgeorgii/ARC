/* robot.h
 *
 * Author: Krylov Georgii (without class WheelBazeMore)
 * class WheelBazeMore autors: Krylov Georgii & Ilyasov Alexander
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 * updates in version 1.3:
 * 		add support of all DIO pins
 * 		add message finish
 * 		add time in start and finish messages
 *		add resetTime() in start()
 *		remove changing flag LOG_time in start()
 *		remove Motor::ride()
 *
 * updates in version 1.4:
 * 		add P_angle, I_angle, D_angle to class motor
 * 		add setPID_angle(cdouble & P, cdouble & I, cdouble & D) to class motor
 * 		remove timer from class motor
 * 		add setTimeout_PID(cdouble & timeout) in class motor
 * 		add void setMinTime_D(cdouble & minTime) in class motor
 *		change ate to app
 */

#ifndef _ROBOT_H_
#define _ROBOT_H_ "1.8"

#include <sys/types.h>
#include <pthread.h>
#include <cmath>
#include <functional>

#include "PWM.h"
#include "DIO.h"
#include "AIO.h"
#include "Encoder.h"
#include "DIIRQ.h"
#include "pins.h"

enum SEN : const uint8_t { FR_L, FR_R, MID_L, MID_R, B_L, B_R };
enum SEN_POS : const uint8_t { BEF_OFF, BEF_ON, AFT_ON, AFT_OFF };

namespace robot {
	NiFpga_Status finish();
	NiFpga_Status start(int argc = 0, char **argv = nullptr, char *argh = nullptr);

	class Pwm {
	public:
		Pwm(const Pin pwm_pin, int startPwm = 0, const uint16_t frequency_divider = 1000);
		void pwmWrite(int newPwmValue = 0);
		int getPwmWrite() const;
		bool getInitStatus() const;

	private:
		MyRio_Pwm pwm;
		int writeValue;
		bool initSucsess;
		uint16_t frequencyDivider;
	};

	class Dio {
	public:
		Dio(const Pin dio_pin, const bool startValue = 0);
		void dioWrite(const bool newValue = 0);
		bool getDioWrite() const;
		bool dioRead(const byte num_of_iter = 5);
		bool dioReadRaw();
		bool getDioRead() const;
		bool getInitStatus() const;

	private:
		MyRio_Dio dio;
		bool writeValue;
		bool readValue;
		bool initSucsess;
		const double waitTime = 0.00001;
	};

	class Aio {
	public:
		Aio(const Pin aio_pin, const double max_val, const double min_val, const double startValue = 0);
		void aioWrite(const double newValue = 0);
		double getAioWrite() const;
		double aioRead(const unsigned int ITER_READ_SEN = 10, const byte NUM_READ_SEN = 5);
		double aioReadNorm(const double max_val, const double min_val,
							const unsigned int ITER_READ_SEN = 10, const byte NUM_READ_SEN = 5);
		double aioReadNorm(const bool useRAW = true,
				const unsigned int ITER_READ_SEN = 10, const byte NUM_READ_SEN = 5);
		double aioReadRaw();
		double getAioRead() const;
		bool getInitStatus() const;
		void setMinMaxVal(const double min, const double max);

	private:
		MyRio_Aio aio;
		double writeValue;
		double readValue;
		bool initSucsess;
		bool canRead;
		bool canWrite;
		double maxVal;
		double minVal;
		const double waitTime = 0.00001;
	};

	class lightSensor : public Aio {
	public:
		lightSensor(const Pin aio_pin, const double max_val,
				    const double min_val, const double startValue = 0) :
			Aio (aio_pin, max_val, min_val, startValue) {senPos = SEN_POS::BEF_OFF;}

		int setRideToLine(uint8_t senPos);
		void printPos(uint8_t sen, uint8_t senPos);

		int sureLine();
		int onLine();
		int offLine();
		uint8_t getSenPos (double speed);
		int setRideToLine();
		void setSenPos(uint8_t newPos);
	private:
		uint8_t senPos;
	};

	class Enc {
	public:
		Enc(const Pin enc_pin);
		int32_t encRead();
		bool getInitStatus() const;

	private:
		MyRio_Encoder enc;
		bool initSucsess;
	};

	class Motor {
	public:
		Motor(const Pin pwm_pin, const Pin dio_pin, const Pin enc_pin = Pin::NO_PIN,
				const double enc_in_degr = 140. * 3 / 360, const double wheel_diam = 7.2,
				const int initMinPwmToSpin = 200,
				const bool pwm_reversValue = 1, const bool enc_reversValue = 0, int startPower = 0);
		~Motor();

		void setPower(double newPower = 0) const;
		void setRawPower(int newPower = 0) const;
		int getSetRawPower() const;
		int32_t getEncRawVal() const;
		int32_t getEncVal() const;
		double getAngleVal() const;
		double getLengthVal() const;
		double getSpeed(double oldTime, double nowTime, double oldEnc, double nowEnc) const;
		void reverseMotorPower();
		void reverseEncoder();
		void reverseMotor();
		int32_t degr_enc(const double degr) const;
		double enc_degr(const int32_t enc_val) const;
		int32_t length_enc(const double length, const double whl_diam) const;
		int32_t length_enc(const double length) const;
		double enc_length(const int32_t enc_val) const;
		bool isEnc() const;
		void encRes();
		float holdTargetAngle(const double target_angle, const double dopPower = 0, const char sign = 0);
		float holdTargetAngle(const double target_angle, const double P, const double I, const double D,
				const double dopPower = 0, const char sign = 0);
		float holdTargetLength(const double target_length);
		float holdTargetLength(const double target_length, const double P, const double I, const double D);
		void turnAng(double finishTarget, double turnSpeed, std::function<uint8_t()> && checkFunc = []()->uint8_t {return 0;});
		void turnAng(double finishTarget, double turnSpeed, std::function<uint8_t()> & checkFunc);
		void moveStr(double length, double speed, std::function<uint8_t()> && checkFunc = []()->uint8_t {return 0;});
		void moveStr(double length, double speed, std::function<uint8_t()> & checkFunc);
		float holdTarget(const double target, const double now_val, const double P, const double I, const double D);
		float holdTarget(const double target, const double now_val);
		/*
		float holdTarget(const double target, const double now_val, const double P, const double I, const double D);
		float holdTarget(const double target, const double now_val);
		*/
		void setMinPwmToSpin(const int newPwm);
		void setPID_angle(const double P, const double I, const double D);
		void setTimeout_PID(const double timeout);
		void setMinTime_D(const double minTime);
		void setPID_length(const double P, const double I, const double D);
		double getLastTarget();
		void setLastTarget(const double newTarget);
		void setICompToZero(const bool setToZero);
		void PIDRes();

	public:
		Pwm *pwm_power;
		Dio *dio_direction;
		Enc *encoder;
		int minPwmToSpin;
		bool pwm_isReversed;
		bool enc_isReversed;
		bool isEncoder;
		double encInDegr;
		double wheelDiam;
		int32_t old_enc;
		double lastTarget;
		double oldErrDegr;
		double oldErrLength;
		double sumErr;
		double oldTime;
		double sumErrAng;
		double oldTimeAng;
		bool iCompToZero = true;
		double P_angle = 40, I_angle = 50, D_angle = 15;
		double Timeout_PID = 2, MinTime_D = 0.03;
		double P_length = 130, I_length = 60, D_length = 5;
	};

	class MotorMore : public Motor{
	public:
		MotorMore(const Pin pwm_pin, const Pin dio_pin, const Pin enc_pin = Pin::NO_PIN,
				  const double enc_in_degr = 140. * 3 / 360, const double wheel_diam = 7.2,
				  const int initMinPwmToSpin = 200,
				  const bool pwm_reversValue = 1, const bool enc_reversValue = 0, int startPower = 0) :
			  Motor(pwm_pin, dio_pin, enc_pin,
	  				enc_in_degr, wheel_diam, initMinPwmToSpin,
	  				pwm_reversValue, enc_reversValue, startPower) {}
	};

	class WheelBaze {
	public:
		WheelBaze(const Pin L_pwm_pin, const Pin R_pwm_pin,
				const Pin L_dio_pin, const Pin R_dio_pin,
				const Pin L_enc_pin = Pin::NO_PIN, const Pin R_enc_pin = Pin::NO_PIN,
				const double enc_in_degr = 136.7 * 3. * 4 / 360, const double wheel_diam = 7.5, //136.7
				const double robot_diam = 39.2, const int initMinPwmToSpin = 100);
		~WheelBaze();

		int32_t robotDegr_enc(const double rbt_degr, const double rbt_diam) const;
		int32_t robotDegr_enc(const double rbt_degr) const;
		double enc_robotDegr(const int32_t enc_value) const;
		int32_t length_enc(const double length) const;
		double enc_length(const int32_t enc_value) const;

		void stop(const double waitTime = 0) const;
		void ride(const double power = 0) const;
		void ride(const double L_power, const double R_power) const;
		void rideStr(int32_t L_enc, int32_t R_enc, const double power, const double P) const;
		void rideStr(const double power, const double P)  const;
		void rideStr(const double power = 0) const;
		void setP_rideStr(const double P);

		/*
		 * void initLight(const double dist, const double power, Aio * sen[]) const;


		void move(const double lFinishTarget, const double rFinishTarget, const double turnSpeed, int (* checkFunc)() = 0) const;
		void turnAng(const double angle, const double speed, int (* checkFunc)() = 0);
		void moveStr(const double dist, const double speed, int (* checkFunc)() = 0);

		void followLine(const double lSenValue, const double rSenValue, const double k, const double power);
		*/
		int32_t L_encRaw() const;
		int32_t R_encRaw() const;
		int32_t L_enc() const;
		int32_t R_enc() const;
		double L_ang() const;
		double R_ang() const;
		void L_encRes() const;
		void R_encRes() const;
		void encRes() const;
		int enc(const bool L_enc_inv = false, const bool R_enc_inv = false) const;
		void turn(const double L_power, const double R_power) const;
		void turn(const double power) const;
		Motor *L(), *R();

	protected:
		Motor *R_mtr, *L_mtr;

		double robotDiam;
		double wheelDiam;
		double P_rideStr = 12;

		int enc(Motor *mtr) const;
		int32_t encRaw(Motor *mtr) const;
		int ang(Motor *mtr) const;
		void encRes(Motor *mtr) const;

		//const double waitTime = 0.001;
	};

	class WheelBazeMore : public WheelBaze	{
	public:
		WheelBazeMore (const Pin L_pwm_pin, const Pin R_pwm_pin,
				const Pin L_dio_pin, const Pin R_dio_pin,
				const Pin L_enc_pin = Pin::NO_PIN, const Pin R_enc_pin = Pin::NO_PIN,
				const double enc_in_degr = 136.7 * 3. * 4 / 360, const double wheel_diam = 7.5, //136.7
				const double robot_diam = 39.2, const int initMinPwmToSpin = 100) :
					WheelBaze (L_pwm_pin, R_pwm_pin,
					L_dio_pin, R_dio_pin,
					L_enc_pin, R_enc_pin,
					enc_in_degr, wheel_diam,
					robot_diam, initMinPwmToSpin) {}
		void initLight(const double dist, const double power, Aio * sen[], const int kp_str);

		void move(const double lFinishTarget, const double rFinishTarget, const double turnSpeed, std::function<uint8_t()> & checkFunc) const;
		void move(const double lFinishTarget, const double rFinishTarget, const double turnSpeed, std::function<uint8_t()> && checkFunc) const;
		void turnAng(const double angle, const double speed, std::function<uint8_t()> & checkFunc);
		void turnAng(const double angle, const double speed, std::function<uint8_t()> && checkFunc);
		void moveStr(const double dist, const double speed, std::function<uint8_t()> & checkFunc); //int (* checkFunc)() = 0);
		void moveStr(const double dist, const double speed, std::function<uint8_t()> && checkFunc);
		void followLine(const double lSenValue, const double rSenValue, const double k, const double power);
		void followLineSpeed(double lSenValue, double rSenValue, double k, double speed, double deltaTime);
		void followLineSpeed2(double lSenValue, double rSenValue, double k, double speed, double deltaTime);
	};

	class Servo{
	public:
		Servo(const Pin pwm_pin, const double start_pos = 0,
				const int min_pwm = 313, const int max_pwm = 1500, const double avail_angle = 180);
		~Servo();
		void setRawPwm(const int pwm_value = 0) const;
		int getSetRawPwm() const;
		void setAngle(const double angle) const;
		void relax() const;

	private:
		Pwm *pwm;
		int minPwm;
		int maxPwm;
		double availAngle;
	};
} /* END namespace robot */

#endif /* ROBOT_H_ */
