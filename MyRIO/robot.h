/* robot.h
 *
 * Author: Krylov Georgii
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

#include "PWM.h"
#include "DIO.h"
#include "AIO.h"
#include "Encoder.h"
#include "DIIRQ.h"
#include "pins.h"

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

	class Enc {
	public:
		Enc(const Pin enc_pin);
		uint32_t encRead();
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
		uint32_t getEncRawVal() const;
		uint32_t getEncVal() const;
		double getAngleVal() const;
		double getLengthVal() const;
		void reverseMotorPower();
		void reverseEncoder();
		void reverseMotor();
		uint32_t degr_enc(const double degr) const;
		double enc_degr(const uint32_t enc_val) const;
		uint32_t length_enc(const double length, const double whl_diam) const;
		uint32_t length_enc(const double length) const;
		double enc_length(const uint32_t enc_val) const;
		bool isEnc() const;
		void encRes();
		float holdTargetAngle(const double target_angle, const double dopPower = 0, const char sign = 0);
		float holdTargetAngle(const double target_angle, const double P, const double I, const double D,
				const double dopPower = 0, const char sign = 0);
		float holdTargetLength(const double target_length);
		float holdTargetLength(const double target_length, const double P, const double I, const double D);
		void setMinPwmToSpin(const int newPwm);
		void setPID_angle(const double P, const double I, const double D);
		void setTimeout_PID(const double timeout);
		void setMinTime_D(const double minTime);
		void setPID_length(const double P, const double I, const double D);

	private:
		Pwm *pwm_power;
		Dio *dio_direction;
		Enc *encoder;
		int minPwmToSpin;
		bool pwm_isReversed;
		bool enc_isReversed;
		bool isEncoder;
		double encInDegr;
		double wheelDiam;
		uint32_t old_enc;
		double oldErrDegr;
		double oldErrLength;
		double sumErr;
		double oldTime;
		double P_angle, I_angle, D_angle;
		double Timeout_PID = 2, MinTime_D = 0.03;
		double P_length = 100, I_length = 15, D_length = 0;
	};

	class WheelBaze {
	public:
		WheelBaze(const Pin L_pwm_pin, const Pin R_pwm_pin,
				const Pin L_dio_pin, const Pin R_dio_pin,
				const Pin L_enc_pin = Pin::NO_PIN, const Pin R_enc_pin = Pin::NO_PIN,
				const double enc_in_degr = 136.7 * 3. * 4 / 360, const double wheel_diam = 7.5, //136.7
				const double robot_diam = 39.2, const int initMinPwmToSpin = 100);
		~WheelBaze();

		uint32_t robotDegr_enc(const double rbt_degr, const double rbt_diam) const;
		uint32_t robotDegr_enc(const double rbt_degr) const;
		double enc_robotDegr(const uint32_t enc_value) const;
		uint32_t length_enc(const double length) const;
		double enc_length(const uint32_t enc_value) const;

		void stop(const double waitTime = 0) const;
		void ride(const double power = 0) const;
		void ride(const double L_power, const double R_power) const;
		void rideStr(uint32_t L_enc, uint32_t R_enc, const double power, const double P) const;
		void rideStr(const double power, const double P)  const;
		void rideStr(const double power = 0) const;
		void setP_rideStr(const double P);

		uint32_t L_encRaw() const;
		uint32_t R_encRaw() const;
		uint32_t L_enc() const;
		uint32_t R_enc() const;
		void L_encRes() const;
		void R_encRes() const;
		void encRes() const;
		int enc(const bool L_enc_inv = false, const bool R_enc_inv = false) const;
		void turn(const double L_power, const double R_power) const;
		void turn(const double power) const;
		Motor *L(), *R();

	private:
		Motor *R_mtr, *L_mtr;

		double robotDiam;
		double P_rideStr = 12;

		int32_t enc(Motor *mtr) const;
		uint32_t encRaw(Motor *mtr) const;
		void encRes(Motor *mtr) const;

		//const double waitTime = 0.001;
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
