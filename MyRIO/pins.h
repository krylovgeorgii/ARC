/* pins.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 */

#ifndef _PINS_H_
#define _PINS_H_ "1.4"

#include "type.h"

namespace robot {
	static constexpr byte P_I_S = 34;

	enum class Pin : const byte {
		NO_PIN = 0,

		PWM_A0 = 27,
		PWM_A1 = 29,
		PWM_A2 = 31,

		PWM_B0 = PWM_A0 + P_I_S,
		PWM_B1 = PWM_A1 + P_I_S,
		PWM_B2 = PWM_A2 + P_I_S,

		PWM_C0 = 14 + P_I_S + P_I_S,
		PWM_C1 = 18 + P_I_S + P_I_S,

		DIO_A0 = 11,
		DIO_A1 = 13,
		DIO_A2 = 15,
		DIO_A3 = 17,
		DIO_A4 = 19,
		DIO_A13 = 26,

		DIO_B0 = DIO_A0 + P_I_S,
		DIO_B1 = DIO_A1 + P_I_S,
		DIO_B2 = DIO_A2 + P_I_S,
		DIO_B3 = DIO_A3 + P_I_S,
		DIO_B4 = DIO_A4 + P_I_S,
		DIO_B13 = DIO_A13 + P_I_S,

		DIO_C1 = 12 + P_I_S + P_I_S,
		DIO_C5 = 16 + P_I_S + P_I_S,

		DIO_A5 = 21,
		DIO_A6 = 23,
		DIO_A7 = 25,
		DIO_A8 = 27,
		DIO_A9 = 29,
		DIO_A10 = 31,
		DIO_A11 = 18,
		DIO_A12 = 22,
		DIO_A14 = 32,
		DIO_A15 = 34,

		DIO_B5 = DIO_A5 + P_I_S,
		DIO_B6 = DIO_A6 + P_I_S,
		DIO_B7 = DIO_A7 + P_I_S,
		DIO_B8 = DIO_A8 + P_I_S,
		DIO_B9 = DIO_A9 + P_I_S,
		DIO_B10 = DIO_A10 + P_I_S,
		DIO_B11 = DIO_A12 + P_I_S,
		DIO_B12 = DIO_A12 + P_I_S,
		DIO_B14 = DIO_A14 + P_I_S,
		DIO_B15 = DIO_A15 + P_I_S,

		DIO_C0 = 11 + P_I_S + P_I_S,
		DIO_C2 = 13 + P_I_S + P_I_S,
		DIO_C3 = 14 + P_I_S + P_I_S,
		DIO_C4 = 15 + P_I_S + P_I_S,
		DIO_C6 = 17 + P_I_S + P_I_S,
		DIO_C7 = 18 + P_I_S + P_I_S,

		ENC_A = 18,
		ENC_B = ENC_A + P_I_S,
		ENC_C0 = 11 + P_I_S + P_I_S,
		ENC_C1 = 15 + P_I_S + P_I_S,

		AI_A0 = 3,
		AI_A1 = 5,
		AI_A2 = 7,
		AI_A3 = 9,

		AO_A0 = 2,
		AO_A1 = 4,

		AI_B0 = AI_A0 + P_I_S,
		AI_B1 = AI_A1 + P_I_S,
		AI_B2 = AI_A2 + P_I_S,
		AI_B3 = AI_A3 + P_I_S,

		AO_B0 = AO_A0 + P_I_S,
		AO_B1 = AO_A1 + P_I_S
	};
}
#endif /* PINS_H_ */
