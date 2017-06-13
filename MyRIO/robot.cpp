/* robot.cpp
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 */

#ifndef _ROBOT_CPP_
#define _ROBOT_CPP_ "1.8"

#include "robot.h"
#include "button.h"
#include "log.h"
#include "time.h"

static constexpr double PI = 3.1415926535;

extern NiFpga_Session myrio_session;

NiFpga_Status robot::start(int argc, char **argv, char *argh) {
	if(robot::log.cleanLogFile) {
		std::ofstream out(robot::log.file, std::ios::ate);
		out.close();
	}

	std::string fileName;

	if (argc > 0 && argv != nullptr) {
		if (argv[0] != nullptr) {
			fileName = argv[0];
		}
	}

	message("start " << fileName << " at " << time());

	NiFpga_Status status = MyRio_Open();

	if (MyRio_IsNotSuccess(status)) {
		message("Can't open MyRio: " << status);
	}

	if (argc > 1) {
		for (int i = 1; i < argc; ++i) {
			std::string val = argv[i];

			if (val == "-h" || val == "--help") {
				std::cout << "	-h --help\n" << "	-v --version" << std::endl;
				if (argh != nullptr) std::cout << "	" << argh << std::endl;
			}else if (val == "-v" || val == "--version") {
				std::cout << "\n		LIB versions:\n" << std::endl;

				#ifdef _TYPE_H_
					std::cout << "		 type: " << _TYPE_H_ << std::endl;
				#endif

				#ifdef _PINS_H_
					std::cout << "		 pins: " << _PINS_H_ << std::endl;
				#endif

				#ifdef _TIME_H_
					std::cout << "		 time: " << _TIME_H_ << std::endl;
				#endif

				#ifdef _LOG_H_
					std::cout << "		  log: " << _LOG_H_ << std::endl;
				#endif

				#ifdef _ROBOT_H_
					std::cout << "		robot: " << _ROBOT_H_ << std::endl;
				#endif
			} else {
				message("incorrect parameter " << val << " use -h for help");
			}

			std::cout << std::endl;
			wait(0.001);
		}
	}

	robot::button::setupButton();

	return status;
}

NiFpga_Status robot::finish() {
	NiFpga_Status status = MyRio_Close();

	if (MyRio_IsNotSuccess(status)) {
		message("Can't close MyRio: " << status);
	}

	message("finish at " << time() << "\n///////////////////////////////////////////////////////////////////");

	return status;
}

robot::Pwm::Pwm(const Pin pwm_pin, int startPwm, const uint16_t frequency_divider) {
	initSucsess = false;
	frequencyDivider = frequency_divider;
	NiFpga_Status status;
	uint8_t selectReg;
	int reg;

	switch(pwm_pin) {
		case Pin::PWM_A0:
			pwm.cnfg = PWMA_0CNFG;
			pwm.cs = PWMA_0CS;
			pwm.max = PWMA_0MAX;
			pwm.cmp = PWMA_0CMP;
			pwm.cntr = PWMA_0CNTR;
			reg = 2;
			break;

		case Pin::PWM_A1:
			pwm.cnfg = PWMA_1CNFG;
			pwm.cs = PWMA_1CS;
			pwm.max = PWMA_1MAX;
			pwm.cmp = PWMA_1CMP;
			pwm.cntr = PWMA_1CNTR;
			reg = 3;
			break;

		case Pin::PWM_A2:
			pwm.cnfg = PWMA_2CNFG;
			pwm.cs = PWMA_2CS;
			pwm.max = PWMA_2MAX;
			pwm.cmp = PWMA_2CMP;
			pwm.cntr = PWMA_2CNTR;
			reg = 4;
			break;

		case Pin::PWM_B0:
			pwm.cnfg = PWMB_0CNFG;
			pwm.cs = PWMB_0CS;
			pwm.max = PWMB_0MAX;
			pwm.cmp = PWMB_0CMP;
			pwm.cntr = PWMB_0CNTR;
			reg = 2;
			break;

		case Pin::PWM_B1:
			pwm.cnfg = PWMB_1CNFG;
			pwm.cs = PWMB_1CS;
			pwm.max = PWMB_1MAX;
			pwm.cmp = PWMB_1CMP;
			pwm.cntr = PWMB_1CNTR;
			reg = 3;
			break;

		case Pin::PWM_B2:
			pwm.cnfg = PWMB_2CNFG;
			pwm.cs = PWMB_2CS;
			pwm.max = PWMB_2MAX;
			pwm.cmp = PWMB_2CMP;
			pwm.cntr = PWMB_2CNTR;
			reg = 4;
			break;

		case Pin::PWM_C0:
			pwm.cnfg = PWMC_0CNFG;
			pwm.cs = PWMC_0CS;
			pwm.max = PWMC_0MAX;
			pwm.cmp = PWMC_0CMP;
			pwm.cntr = PWMC_0CNTR;
			reg = 1;
			break;

		case Pin::PWM_C1:
			pwm.cnfg = PWMC_1CNFG;
			pwm.cs = PWMC_1CS;
			pwm.max = PWMC_1MAX;
			pwm.cmp = PWMC_1CMP;
			pwm.cntr = PWMC_1CNTR;
			reg = 3;
			break;

		default:
			message("incorrect pin to pwm");
			return;
	}

	Pwm_Configure(&pwm, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);

    Pwm_ClockSelect(&pwm, Pwm_64x); //64
    Pwm_CounterMaximum(&pwm, frequencyDivider); //12500

    if (pwm_pin == Pin::PWM_A0 || pwm_pin == Pin::PWM_A1 || pwm_pin == Pin::PWM_A2) {
		status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not read from the SYSSELECTA register!")
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << reg);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTA register!")
		if (MyRio_IsNotSuccess(status)) return;
    }else if (pwm_pin == Pin::PWM_B0 || pwm_pin == Pin::PWM_B1 || pwm_pin == Pin::PWM_B2) {
		status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not read from the SYSSELECTB register!")
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << reg);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTB register!")
		if (MyRio_IsNotSuccess(status)) return;
    }else if (pwm_pin == Pin::PWM_C0 || pwm_pin == Pin::PWM_C1) {
		status = NiFpga_ReadU8(myrio_session, SYSSELECTC, &selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not read from the SYSSELECTC register!")
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << reg);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTC, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTC register!")
		if (MyRio_IsNotSuccess(status)) return;
    }else{
    	message("incorrect pin to pwm");
    	return;
    }

    pwmWrite(startPwm);
    initSucsess = true;
};

inline void robot::Pwm::pwmWrite(int newPwmValue) {
	if (newPwmValue < 0) {
		newPwmValue = 0;
	} else if (newPwmValue > frequencyDivider) {
		newPwmValue = frequencyDivider;
	}

	Pwm_CounterCompare(&pwm, newPwmValue);
	writeValue = newPwmValue;
}

inline int robot::Pwm::getPwmWrite() const {
	return writeValue;
}

inline bool robot::Pwm::getInitStatus() const {
	return initSucsess;
}

inline bool robot::Dio::getInitStatus() const {
	return initSucsess;
}

inline bool robot::Dio::dioReadRaw() {
	readValue = Dio_ReadBit(&dio);
	return readValue;
}

inline bool robot::Dio::dioRead(const byte num_of_iter) {
	signed char numOfHigh = 0;

	for(unsigned int cntr = 0; cntr < num_of_iter; ++cntr) {
		if(dioReadRaw()) {
			++numOfHigh;
		} else {
			--numOfHigh;
		}

		wait(waitTime);
	}

	if (numOfHigh > 0) {
		return 1;
	}

	return 0;
}

inline bool robot::Dio::getDioRead() const {
	return readValue;
}

inline void robot::Dio::dioWrite(const bool newValue) {
	Dio_WriteBit(&dio, newValue);
	writeValue = newValue;
}

inline bool robot::Dio::getDioWrite() const {
	return writeValue;
}

robot::Dio::Dio(const Pin dio_pin, const bool startValue) {
	initSucsess = false;

	if (dio_pin == Pin::DIO_A0 || dio_pin == Pin::DIO_A1 || dio_pin == Pin::DIO_A2 || dio_pin == Pin::DIO_A3
	 || dio_pin == Pin::DIO_A4 || dio_pin == Pin::DIO_A5 || dio_pin == Pin::DIO_A6 || dio_pin == Pin::DIO_A7) {
		dio.dir = DIOA_70DIR;
		dio.out = DIOA_70OUT;
		dio.in = DIOA_70IN;
	} else if (dio_pin == Pin::DIO_A8 || dio_pin == Pin::DIO_A9 || dio_pin == Pin::DIO_A10 || dio_pin == Pin::DIO_A11
		 || dio_pin == Pin::DIO_A12 || dio_pin == Pin::DIO_A13 || dio_pin == Pin::DIO_A14 || dio_pin == Pin::DIO_A15) {
		dio.dir = DIOA_158DIR;
		dio.out = DIOA_158OUT;
		dio.in = DIOA_158IN;
	} else if (dio_pin == Pin::DIO_B0 || dio_pin == Pin::DIO_B1 || dio_pin == Pin::DIO_B2 || dio_pin == Pin::DIO_B3
		   || dio_pin == Pin::DIO_B4 || dio_pin == Pin::DIO_B5 || dio_pin == Pin::DIO_B6 || dio_pin == Pin::DIO_B7) {
		dio.dir = DIOB_70DIR;
		dio.out = DIOB_70OUT;
		dio.in = DIOB_70IN;
	} else if (dio_pin == Pin::DIO_B8 || dio_pin == Pin::DIO_B9 || dio_pin == Pin::DIO_B10 || dio_pin == Pin::DIO_B11
		 || dio_pin == Pin::DIO_B12 || dio_pin == Pin::DIO_B13 || dio_pin == Pin::DIO_B14 || dio_pin == Pin::DIO_B15) {
		dio.dir = DIOB_158DIR;
		dio.out = DIOB_158OUT;
		dio.in = DIOB_158IN;
	} else if (dio_pin == Pin::DIO_C0 || dio_pin == Pin::DIO_C1 || dio_pin == Pin::DIO_C2 || dio_pin == Pin::DIO_C3
		   || dio_pin == Pin::DIO_C4 || dio_pin == Pin::DIO_C5 || dio_pin == Pin::DIO_C6 || dio_pin == Pin::DIO_C7) {
		dio.dir = DIOC_70DIR;
		dio.out = DIOC_70OUT;
		dio.in = DIOC_70IN;
	} else {
		message("incorrect pin to dio");
		return;
	}

	if (dio_pin == Pin::DIO_A0 || dio_pin == Pin::DIO_B0 || dio_pin == Pin::DIO_C0
	 || dio_pin == Pin::DIO_A8 || dio_pin == Pin::DIO_B8) {
		dio.bit = 0;
	} else if (dio_pin == Pin::DIO_A1 || dio_pin == Pin::DIO_B1 || dio_pin == Pin::DIO_C1
		   || dio_pin == Pin::DIO_A9 || dio_pin == Pin::DIO_B9) {
		dio.bit = 1;
	} else if (dio_pin == Pin::DIO_A2 || dio_pin == Pin::DIO_B2 || dio_pin == Pin::DIO_C2
		  || dio_pin == Pin::DIO_A10 || dio_pin == Pin::DIO_B10) {
		dio.bit = 2;
	} else if (dio_pin == Pin::DIO_A3 || dio_pin == Pin::DIO_B3 || dio_pin == Pin::DIO_C3
		  || dio_pin == Pin::DIO_A11 || dio_pin == Pin::DIO_B11) {
		dio.bit = 3;
	} else if (dio_pin == Pin::DIO_A4 || dio_pin == Pin::DIO_B4 || dio_pin == Pin::DIO_C4
		  || dio_pin == Pin::DIO_A12 || dio_pin == Pin::DIO_B12) {
		dio.bit = 4;
	} else if (dio_pin == Pin::DIO_A5 || dio_pin == Pin::DIO_B5 || dio_pin == Pin::DIO_C5
		  || dio_pin == Pin::DIO_A13 || dio_pin == Pin::DIO_B13) {
		dio.bit = 5;
	} else if (dio_pin == Pin::DIO_A6 || dio_pin == Pin::DIO_B6 || dio_pin == Pin::DIO_C6
	      || dio_pin == Pin::DIO_A14 || dio_pin == Pin::DIO_B14) {
		dio.bit = 6;
	} else if (dio_pin == Pin::DIO_A7 || dio_pin == Pin::DIO_B7 || dio_pin == Pin::DIO_C7
		   || dio_pin == Pin::DIO_A15 || dio_pin == Pin::DIO_B15) {
		dio.bit = 7;
	} else{
		message("incorrect pin to dio");
		return;
	}

	dioWrite(startValue);
	dioRead();
	initSucsess = true;
};

inline void robot::Aio::setMinMaxVal(const double min, const double max) {
	maxVal = max;
	minVal = min;
}

inline bool robot::Aio::getInitStatus() const {
	return initSucsess;
}

inline double robot::Aio::aioReadRaw() {
	readValue = Aio_Read(&aio);
	return readValue;
}

inline double robot::Aio::aioRead(const unsigned int ITER_READ_SEN, const byte NUM_READ_SEN) {
	if(NUM_READ_SEN == 0) {
		return 0;
	}

	double ai_val = 0;
	double mas[NUM_READ_SEN];
	double asist = 0;
	unsigned int p, i, j;

	for (unsigned int counter = 0; counter < ITER_READ_SEN; ++counter) {
		for (p = 0; p < NUM_READ_SEN; ++p) {
			mas[p] = aioReadRaw();
			wait(waitTime);
		}

		for (i = 0; i < static_cast<byte>(NUM_READ_SEN - 1); ++i) {
			for (j = i; j < NUM_READ_SEN; ++j) {
				if (mas[i] > mas[j]) {
					asist = mas[i];
					mas[i] = mas[j];
					mas[j] = asist;
				}
			}
		}

		ai_val += mas[NUM_READ_SEN / 2 + 1];
	}

	ai_val /= ITER_READ_SEN;

	return ai_val;
}

inline double robot::Aio::aioReadNorm(const double max_val, const double min_val,
		const unsigned int ITER_READ_SEN, const byte NUM_READ_SEN) {
	 return  (aioRead(ITER_READ_SEN, NUM_READ_SEN) - min_val) / (max_val - min_val);
}

inline double robot::Aio::aioReadNorm(const bool useRAW,
		const unsigned int ITER_READ_SEN, const byte NUM_READ_SEN) {
	if (useRAW) {
		return  (aioReadRaw() - minVal) / (maxVal - minVal);
	}

	return  (aioRead(ITER_READ_SEN, NUM_READ_SEN) - minVal) / (maxVal - minVal);
}

inline double robot::Aio::getAioRead() const {
	return readValue;
}

inline void robot::Aio::aioWrite(const double newValue) {
	Aio_Write(&aio, newValue);
	writeValue = newValue;
}

inline double robot::Aio::getAioWrite() const {
	return writeValue;
}

robot::Aio::Aio(const Pin aio_pin, const double max_val, const double min_val, const double startValue) {
	initSucsess = false;
	canRead = false;
	canWrite = false;
	maxVal = max_val;
	minVal = min_val;

	switch(aio_pin) {
		case Pin::AI_A0:
			aio.val = AIA_0VAL;
			aio.wght = AIA_0WGHT;
			aio.ofst = AIA_0OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AI_A1:
			aio.val = AIA_1VAL;
			aio.wght = AIA_1WGHT;
			aio.ofst = AIA_1OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AI_A2:
			aio.val = AIA_2VAL;
			aio.wght = AIA_2WGHT;
			aio.ofst = AIA_2OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AI_A3:
			aio.val = AIA_3VAL;
			aio.wght = AIA_3WGHT;
			aio.ofst = AIA_3OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AO_A0:
			aio.val = AOA_0VAL;
			aio.wght = AOA_0WGHT;
			aio.ofst = AOA_0OFST;
			aio.set = AOSYSGO;
			aio.is_signed = NiFpga_False;
			canWrite = true;
			break;

		case Pin::AO_A1:
			aio.val = AOA_1VAL;
			aio.wght = AOA_1WGHT;
			aio.ofst = AOA_1OFST;
			aio.set = AOSYSGO;
			aio.is_signed = NiFpga_False;
			canWrite = true;
			break;

		case Pin::AI_B0:
			aio.val = AIB_0VAL;
			aio.wght = AIB_0WGHT;
			aio.ofst = AIB_0OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AI_B1:
			aio.val = AIB_1VAL;
			aio.wght = AIB_1WGHT;
			aio.ofst = AIB_1OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AI_B2:
			aio.val = AIB_2VAL;
			aio.wght = AIB_2WGHT;
			aio.ofst = AIB_2OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AI_B3:
			aio.val = AIB_3VAL;
			aio.wght = AIB_3WGHT;
			aio.ofst = AIB_3OFST;
			aio.is_signed = NiFpga_False;
			canRead = true;
			break;

		case Pin::AO_B0:
			aio.val = AOB_0VAL;
			aio.wght = AOB_0WGHT;
			aio.ofst = AOB_0OFST;
			aio.set = AOSYSGO;
			aio.is_signed = NiFpga_False;
			canWrite = true;
			break;

		case Pin::AO_B1:
			aio.val = AOB_1VAL;
			aio.wght = AOB_1WGHT;
			aio.ofst = AOB_1OFST;
			aio.set = AOSYSGO;
			aio.is_signed = NiFpga_False;
			canWrite = true;
			break;

		default:
			message("incorrect pin to aio");
			return;
	}

	Aio_Scaling(&aio);

	if (canRead) aioRead();
	if (canWrite) aioWrite(startValue);

	initSucsess = true;
};

inline uint32_t robot::Enc::encRead() {
	return Encoder_Counter(&enc);
}

inline bool robot::Enc::getInitStatus() const {
	return initSucsess;
}

robot::Enc::Enc(const Pin enc_pin) {
	initSucsess = false;
	NiFpga_Status status;
	uint8_t selectReg;

	if (enc_pin == Pin::ENC_A) {
		enc.cnfg = ENCACNFG;
		enc.stat = ENCASTAT;
		enc.cntr = ENCACNTR;

		status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);
		    MyRio_ReturnValueIfNotSuccess(status, status,
		        "Could not read from the SYSSELECTA register!");
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << 5);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTA register!")
		if (MyRio_IsNotSuccess(status)) return;
	} else if (enc_pin == Pin::ENC_B) {
		enc.cnfg = ENCBCNFG;
		enc.stat = ENCBSTAT;
		enc.cntr = ENCBCNTR;

		status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectReg);
		    MyRio_ReturnValueIfNotSuccess(status, status,
		        "Could not read from the SYSSELECTB register!");
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << 5);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTB register!")
		if (MyRio_IsNotSuccess(status)) return;
	} else if (enc_pin == Pin::ENC_C0) {
		enc.cnfg = ENCC_0CNFG;
		enc.stat = ENCC_0STAT;
		enc.cntr = ENCC_0CNTR;

		status = NiFpga_ReadU8(myrio_session, SYSSELECTC, &selectReg);
		    MyRio_ReturnValueIfNotSuccess(status, status,
		        "Could not read from the SYSSELECTC register!");
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << 0);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTC, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTC register!")
		if (MyRio_IsNotSuccess(status)) return;
	} else if (enc_pin == Pin::ENC_C1) {
		enc.cnfg = ENCC_1CNFG;
		enc.stat = ENCC_1STAT;
		enc.cntr = ENCC_1CNTR;

		status = NiFpga_ReadU8(myrio_session, SYSSELECTC, &selectReg);
		    MyRio_ReturnValueIfNotSuccess(status, status,
		        "Could not read from the SYSSELECTC register!");
		if (MyRio_IsNotSuccess(status)) return;

		selectReg = selectReg | (1 << 2);

		status = NiFpga_WriteU8(myrio_session, SYSSELECTC, selectReg);
		MyRio_ReturnValueIfNotSuccess(status, status,
			"Could not write to the SYSSELECTC register!")
		if (MyRio_IsNotSuccess(status)) return;
	} else {
		message("incorrect pin to enc");
		return;
	}

	Encoder_Configure(&enc, Encoder_Enable | Encoder_SignalMode,
	            Encoder_Enabled | Encoder_StepDirection);
	initSucsess = true;
};

robot::Motor::Motor(const Pin pwm_pin, const Pin dio_pin, const Pin enc_pin,
		const double enc_in_degr, const double wheel_diam, const int initMinPwmToSpin,
		const bool pwm_reversValue, const bool enc_reversValue, int startPower) {
	bool dioValue = 0;
	minPwmToSpin = initMinPwmToSpin;
	encInDegr = enc_in_degr;
	wheelDiam = wheel_diam;
	oldErrDegr = sumErr = oldErrLength = 0;
	setPID_angle(70, 10, 0);
	oldTime = time();

	if (minPwmToSpin < 0) {
		minPwmToSpin = 0;
		message("minPwmToSpin can't be less than zero. It set to zero\n");
	} else if (minPwmToSpin > 1000) {
		minPwmToSpin = 1000;
		message("minPwmToSpin can't be more than 1000. It set to 1000\n");
	}

	enc_isReversed = enc_reversValue;
	pwm_isReversed = pwm_reversValue;

	if ((startPower < 0 && !pwm_isReversed) || (startPower > 0 && pwm_isReversed)) {
		dioValue = 1;
	}

	if (startPower < 0) startPower = -startPower;

	pwm_power = new Pwm(pwm_pin, startPower);
	if (pwm_power == nullptr){
		message("no memory for new Pwm on pin " << static_cast<byte>(pwm_pin));
	} else if (!(pwm_power->getInitStatus())) {
		message("incorrect init new Pwm on pin " << static_cast<byte>(pwm_pin));
		delete pwm_power;
		pwm_power = nullptr;
	}

	dio_direction = new Dio(dio_pin, dioValue);
	if (dio_direction == nullptr){
		message("no memory for new Dio on pin " << static_cast<byte>(dio_pin));
	} else if (!(dio_direction->getInitStatus())) {
		message("incorrect init new Dio on pin " << static_cast<byte>(dio_pin));
		delete dio_direction;
		dio_direction = nullptr;
	}

	if (enc_pin == Pin::NO_PIN){
		encoder = nullptr;
	} else {
		encoder = new Enc(enc_pin);
		if (encoder == nullptr ){
			message("no memory for new Enc on pin " << static_cast<byte>(enc_pin));
		} else if (!(encoder->getInitStatus())) {
			message("incorrect init new Enc on pin " << static_cast<byte>(enc_pin));
			delete encoder;
			encoder = nullptr;
		}
	}

	if (encoder == nullptr) {
		isEncoder = false;
		old_enc = 0;
	} else {
		isEncoder = true;
		old_enc = getEncRawVal();
	}
};

robot::Motor::~Motor() {
	setPower(0);
	if (pwm_power != nullptr) { delete pwm_power; }
	if (dio_direction != nullptr) { delete dio_direction; }
	if (encoder != nullptr) { delete encoder; }
};

inline bool robot::Motor::isEnc() const {
	return isEncoder;
}

inline uint32_t robot::Motor::getEncRawVal() const {
	if (encoder == nullptr) {
		message("no Enc to read");
		return 0;
	}

	if (enc_isReversed) {
		return -(encoder->encRead());
	}

	return encoder->encRead();
}

inline uint32_t robot::Motor::getEncVal() const {
	if (encoder == nullptr) {
		message("no Enc to read");
		return 0;
	}

	wait(0.00001);
	return getEncRawVal() - old_enc;
}

inline double robot::Motor::getAngleVal() const {
	return enc_degr(getEncVal());
}

inline double robot::Motor::getLengthVal() const {
	return enc_length(getEncVal());
}

inline void robot::Motor::setPower(double newPower) const {
	bool dioValue = 0;

	if ((newPower < 0 && !pwm_isReversed) || (newPower > 0 && pwm_isReversed)) {
		dioValue = 1;
	}

	if (newPower < 0) {
		newPower = -newPower;
	}

	if (newPower < 1) {
		newPower = 0;
	} else {
		newPower = minPwmToSpin + (1000 - minPwmToSpin) * newPower * 0.001;
	}

	if (pwm_power == nullptr) {
		message("no open Pwm to write: " << newPower);
	} else {
		pwm_power->pwmWrite((int)(newPower + 0.5));
	}

	if (dio_direction == nullptr) {
		message("no open Dio to write: " << newPower);
	} else {
		dio_direction->dioWrite(dioValue);
	}
}

inline void robot::Motor::setRawPower(int newPower) const {
	bool dioValue = 0;

	if ((newPower < 0 && !pwm_isReversed) || (newPower > 0 && pwm_isReversed)) {
		dioValue = 1;
	}

	if (newPower < 0) {
		newPower = -newPower;
	}

	if (pwm_power == nullptr) {
		message("no open Pwm to write: " << newPower);
	} else {
		pwm_power->pwmWrite(newPower);
	}

	if (dio_direction == nullptr) {
		message("no open Dio to write: %d\n" << newPower);
	} else {
		dio_direction->dioWrite(dioValue);
	}
}

inline int robot::Motor::getSetRawPower() const {
	int power = 0;

	if (pwm_power != nullptr) {
		power = pwm_power->getPwmWrite();
	} else {
		message("no open Pwm to read");
	}

	if (dio_direction != nullptr) {
		if ((dio_direction->getDioWrite() && !pwm_isReversed) ||
				(!(dio_direction->getDioWrite()) && pwm_isReversed)) {
			power = -power;
		}
	} else {
		message("no open Dio to read");

		if (pwm_isReversed) {
			power = -power;
		}
	}

	return power;
}

inline uint32_t robot::Motor::degr_enc(const double degr) const {
	if (encoder == nullptr) {
		message("no encoder on this motor! degr_edc(" << degr << ")");
	}

	return degr * encInDegr + 0.5;
}

inline double robot::Motor::enc_degr(const uint32_t enc_val) const {
	return static_cast<int>(enc_val / encInDegr);
}

inline uint32_t robot::Motor::length_enc(const double length, const double whl_diam) const {
	return degr_enc(360 * length / (whl_diam * PI));
}

inline double robot::Motor::enc_length(const uint32_t enc_val) const {
	return enc_degr(enc_val) * wheelDiam * PI / 360;
}

inline uint32_t robot::Motor::length_enc(const double length) const {
	return length_enc (length, wheelDiam);
}

inline void robot::Motor::reverseMotorPower() {
	int power = getSetRawPower();
	pwm_isReversed = !pwm_isReversed;
	setRawPower(power);
}

inline void robot::Motor::reverseEncoder() {
	enc_isReversed = !enc_isReversed;
}

inline void robot::Motor::reverseMotor() {
	reverseMotorPower();
	reverseEncoder();
}

inline void robot::Motor::setPID_angle(const double P, const double I, const double D) {
	P_angle = P;
	I_angle = I;
	D_angle = D;
}

inline void robot::Motor::setPID_length(const double P, const double I, const double D) {
	P_length = P;
	I_length = I;
	D_length = D;
}

robot::WheelBaze::WheelBaze(const Pin L_pwm_pin, const Pin R_pwm_pin,
			  const Pin L_dio_pin, const Pin R_dio_pin,
			  const Pin L_enc_pin, const Pin R_enc_pin,
			  const double enc_in_degr, const double wheel_diam,
			  const double robot_diam, const int initMinPwmToSpin) {
	robotDiam = robot_diam;

	L_mtr = new Motor(L_pwm_pin, L_dio_pin, L_enc_pin, enc_in_degr, wheel_diam, initMinPwmToSpin);
	if (L_mtr == nullptr){
		message("no memory for new Motor L_motor");
	}

	R_mtr = new Motor(R_pwm_pin, R_dio_pin, R_enc_pin, enc_in_degr, wheel_diam, initMinPwmToSpin, false, true);
	if (R_mtr == nullptr){
		message("no memory for new Motor R_motor");
	}
};

robot::WheelBaze::~WheelBaze() {
	if (L_mtr != nullptr) { delete L_mtr; }
	if (R_mtr != nullptr) { delete R_mtr; }
};

inline double robot::WheelBaze::enc_length(const uint32_t enc_value) const {
	if (L_mtr != nullptr) {
		return L_mtr->enc_length(enc_value);
	}

	if (R_mtr != nullptr) {
		return R_mtr->enc_length(enc_value);
	}

	message("no motor to enc_length(" << enc_value << ")");
	return 0;
}

inline uint32_t robot::WheelBaze::length_enc(const double length) const {
	if (L_mtr != nullptr) {
		return L_mtr->length_enc(length);
	}

	if (R_mtr != nullptr) {
		return R_mtr->length_enc(length);
	}

	message("no motor to length_enc(" << length << ")");
	return 0;
}

inline double robot::WheelBaze::enc_robotDegr(const uint32_t enc_value) const {
	return enc_length(enc_value) * 360 / (robotDiam * PI);
}

inline uint32_t robot::WheelBaze::robotDegr_enc(const double rbt_degr, const double rbt_diam) const {
	return length_enc(rbt_degr * rbt_diam * PI / 360);
}

inline uint32_t robot::WheelBaze::robotDegr_enc(const double rbt_degr) const {
	return robotDegr_enc(rbt_degr, robotDiam);
}

inline void robot::WheelBaze::ride(const double L_power, const double R_power) const {
	if (L_mtr != nullptr) {
		L_mtr->setPower(L_power);
	} else {
		message("absent left motor!");
	}

	if (R_mtr != nullptr) {
		R_mtr->setPower(R_power);
	} else {
		message("absent right motor!");
	}
}

inline void robot::WheelBaze::ride(const double power) const {
	ride(power, power);
}

inline void robot::WheelBaze::stop(const double waitTime) const {
	ride(0);
	wait(waitTime, false);
}

inline void robot::WheelBaze::rideStr(uint32_t L_enc, uint32_t R_enc, const double power, const double P) const {
	ride(power + (R_enc - L_enc) * P, power + (L_enc - R_enc) * P);
}

inline void robot::WheelBaze::rideStr(const double power, const double P) const {
	rideStr(L_enc(), R_enc(), power, P);
}

inline void robot::WheelBaze::rideStr(const double power) const {
	rideStr(L_enc(), R_enc(), power, P_rideStr);
}

inline void robot::WheelBaze::setP_rideStr(const double P) {
	P_rideStr = P;
}

inline uint32_t robot::WheelBaze::encRaw(Motor *mtr) const {
	if (mtr == nullptr) {
		message("no Motor to encRaw(mtr)");
		return 0;
	} else if (!mtr->isEnc()) {
		message("no Enc on motor to encRaw(mtr)");
		return 0;
	}

	return mtr->getEncRawVal();
}

inline int robot::WheelBaze::enc(Motor *mtr) const {
	if (mtr == nullptr) {
		message("no Motor to encRaw(mtr)");
		return 0;
	}else if (!mtr->isEnc()) {
		message("no Enc on motor to encRaw(mtr)");
		return 0;
	}

	return mtr->getEncVal();
}

inline uint32_t robot::WheelBaze::L_encRaw() const {
	return encRaw(L_mtr);
}

inline uint32_t robot::WheelBaze::R_encRaw() const {
	return encRaw(R_mtr);
}

inline uint32_t robot::WheelBaze::L_enc() const {
	return enc(L_mtr);
}

inline uint32_t robot::WheelBaze::R_enc() const {
	return enc(R_mtr);
}

inline void robot::WheelBaze::L_encRes() const {
	encRes(L_mtr);
}

inline void robot::WheelBaze::R_encRes() const {
	encRes(R_mtr);
}

inline void robot::WheelBaze::encRes(Motor *mtr) const {
	if (mtr == nullptr) {
		message("no Motor to encRes(mtr)");
	} else {
		mtr->encRes();
	}

	wait(0.01);
}

inline void robot::Motor::encRes() {
	old_enc = getEncRawVal();
	wait(0.01);
}

inline float robot::Motor::holdTargetAngle(const double target_angle,
		const double P, const double I, const double D, const double dopPower, const char sign) {
	double err = target_angle - enc_degr(getEncVal());
	double d_comp = 0;
	double newTime = time();

	if (newTime - oldTime >= Timeout_PID) {
		oldTime = newTime;
	}

	if (fabs(err - oldErrDegr) > 1) {
		sumErr = 0;
	} else {
		sumErr += err * (newTime - oldTime) * I;
	}

	if ((err > 0 && oldErrDegr < 0) || (err < 0 && oldErrDegr > 0)) {
		sumErr = 0;
	}

	if ((err > 0 && sumErr < 0) || (err < 0 && sumErr > 0)) {
		sumErr = 0;
	}

	if (oldTime == newTime) {
		d_comp = 0;
		sumErr = 0;
	} else if(newTime - oldTime > MinTime_D) {
		d_comp = (oldErrDegr - err) / (newTime - oldTime) * D;
	}

	if (sign == -1 && err * P + sumErr + d_comp + dopPower > 0) {
		setPower(0.);
	} else if (sign == 1 && err * P + sumErr + d_comp + dopPower < 0) {
		setPower(0.);
	} else {
		setPower(err * P + sumErr + d_comp + dopPower);
	}

	oldTime = newTime;
	oldErrDegr = err;

	return err;
}

inline float robot::Motor::holdTargetAngle(const double  target_angle, const double dopPower, const char sign) {
	return holdTargetAngle(target_angle, P_angle, I_angle, D_angle, dopPower, sign);
}

inline void robot::Motor::setTimeout_PID(const double timeout) {
	Timeout_PID = timeout;
}

inline void robot::Motor::setMinTime_D(const double minTime) {
	MinTime_D = minTime;
}

inline void robot::Motor::setMinPwmToSpin(const int newPwm) {
	minPwmToSpin = newPwm;
}

inline float robot::Motor::holdTargetLength(const double target_length,
		const double P, const double I, const double D) {
	double err = target_length - enc_length(getEncVal());
	double newTime = time();

	if (newTime - oldTime > Timeout_PID) {
		oldTime = newTime;
	}

	if (fabs(err - oldErrLength) > 1) {
		sumErr = 0;
	} else {
		sumErr += err * (newTime - oldTime) * I;
	}

	if ((err > 0 && oldErrLength < 0) || (err < 0 && oldErrLength > 0)) {
		sumErr = 0;
	}

	if ((err > 0 && sumErr < 0) || (err < 0 && sumErr > 0)) {
		sumErr = 0;
	}

	setPower(err * P + sumErr);
	oldTime = newTime;
	oldErrLength = err;

	return err;
}

inline float robot::Motor::holdTargetLength(const double target_length) {
	return  holdTargetLength(target_length, P_length, I_length, D_length);
}

inline void robot::WheelBaze::encRes() const {
	L_encRes();
	R_encRes();
	wait(0.1);
}

inline int robot::WheelBaze::enc(const bool L_enc_inv, const bool R_enc_inv) const {
	bool L_isEnc = false;
	bool R_isEnc = false;

	int L_inv = 1, R_inv = 1;

	if (L_enc_inv) L_inv = -1;
	if (R_enc_inv) R_inv = -1;

	if (L_mtr != nullptr && L_mtr->isEnc()) {
		L_isEnc = true;
	}

	if (R_mtr != nullptr && R_mtr->isEnc()) {
		R_isEnc = true;
	}

	if (L_isEnc && R_isEnc) {
		return ((int)L_enc() * L_inv + (int)R_enc() * R_inv) / 2;
	}

	if (L_isEnc) {
		return (int)L_enc() * L_inv;
	}

	if (R_isEnc) {
		return (int)R_enc() * R_inv;
	}

	message("no encoder to read enc()");
	return 0;
}

inline robot::Motor* robot::WheelBaze::L() {
	return L_mtr;
}

inline robot::Motor* robot::WheelBaze::R() {
	return R_mtr;
}

inline void robot::WheelBaze::turn(const double L_power, const double R_power) const {
	ride(L_power, R_power);
}

inline void robot::WheelBaze::turn(const double power) const {
	turn(power, -power);
}

inline void robot::Servo::setRawPwm(const int pwm_value) const {
	if (pwm == nullptr) {
		message("no Pwm pwm to setRawPwm(" << pwm_value << ")");
		return;
	}

	pwm->pwmWrite(pwm_value);
}

inline int robot::Servo::getSetRawPwm() const {
	if (pwm == nullptr) {
		message("no Pwm pwm to getSetRawPwm()");
		return 0;
	}

	return pwm->getPwmWrite();
}

inline void robot::Servo::relax() const {
	setRawPwm(0);
}

inline void robot::Servo::setAngle(const double angle) const {
	if (angle < 0) {
		message("angle " << angle << " can't be less than zero. Pwm set to minPwm");
		setRawPwm(minPwm);
	} else if (angle > availAngle) {
		message("angle " << angle << " can't be more than availAngle. Pwm set to maxPwm");
		setRawPwm(maxPwm);
	} else {
		setRawPwm(minPwm + (maxPwm - minPwm) * angle / availAngle);
	}
}

robot::Servo::Servo(const Pin pwm_pin, const double start_pos,
		const int min_pwm, const int max_pwm, const double avail_angle) {
	minPwm = min_pwm;
	if (minPwm < 0){
		minPwm = 0;
		message("minPwm " << minPwm << " can't be less than zero. It set to zero");
	}/*else if (minPwm > 1000){
		minPwm = 1000;
		printf("minPwm %d can't be more than 1000. It set to 1000\n", minPwm);
	}*/

	maxPwm = max_pwm;
	if (maxPwm < 0){
		maxPwm = 0;
		message("maxPwm " << maxPwm << " can't be less than zero. It set to zero");
	}/*else if (maxPwm > 1000){
		maxPwm = 1000;
		printf("maxPwm %d can't be more than 1000. It set to 1000\n", maxPwm);
	}*/

	if (maxPwm < minPwm){
		message("maxPwm " << maxPwm << " can't be less than minPwm " << minPwm << " . It set to minPwm");
		maxPwm = minPwm;
	}

	availAngle = avail_angle;

	if (availAngle < 0){
		availAngle = 0;
		message("availDegr can't be less than zero. It set to zero");
	}

	pwm = new Pwm(pwm_pin, 0, 12499);
	if (pwm == nullptr) {
		message("no memory for new Pwm pwm in Servo");
		return;
	}

	if(start_pos >= 0) {
		setAngle(start_pos);
	} else {
		relax();
	}
}

robot::Servo::~Servo() {
	relax();
	if (pwm != nullptr) { delete pwm; }
}

#endif /* _ROBOT_CPP_ */
