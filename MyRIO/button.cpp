/*
 * button.cpp
 *
 *  Created on: 24 апр. 2017 г.
 *      Author: root
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_ "1.1"

#include "ButtonIRQ.h"
#include "type.h"
#include "log.h"
#include <thread>
#include "button.h"
#include "time.h"

namespace robot {
	namespace button {
		MyRio_IrqButton irqBtn0;
		irq::ThreadResource irqThread0;

		void setupButton() {
			int32_t status;

			const uint8_t IrqNumberConfigure = 1;
			const uint32_t CountConfigure = 1;
			const Irq_Button_Type TriggerTypeConfigure = Irq_Button_RisingEdge;

			irqBtn0.btnIrqNumber = IRQDI_BTNNO;
			irqBtn0.btnCount = IRQDI_BTNCNT;
			irqBtn0.btnIrqEnable = IRQDI_BTNENA;
			irqBtn0.btnIrqRisingEdge = IRQDI_BTNRISE;
			irqBtn0.btnIrqFallingEdge = IRQDI_BTNFALL;

			irqThread0.irqNumber = IrqNumberConfigure;

			status = Irq_RegisterButtonIrq(&irqBtn0, &(irqThread0.irqContext), IrqNumberConfigure,
											CountConfigure, TriggerTypeConfigure);

			if (status != NiMyrio_Status_Success) {
					message("CONFIGURE ERROR: " << status << ", Configuration of Button IRQ failed.");
			}
		}

		void waitButtonPush() {
			uint32_t irqAssert = 0;

			message("start wait button push");

			while(true) {
				Irq_Wait(irqThread0.irqContext, static_cast<NiFpga_Irq>(irqThread0.irqNumber),
								&irqAssert, static_cast<NiFpga_Bool *>(&(irqThread0.irqThreadRdy)));

				//message("+");
				wait(0.1);
				if (irqAssert & (1 << irqThread0.irqNumber)) {
					message("button pushed");
					return;
					//exit(distructor());
				}
			}
		}
	}
}

#endif /* BUTTON_CPP_ */
