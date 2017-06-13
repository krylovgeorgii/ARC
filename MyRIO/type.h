/* type.h
 *
 * Author: Krylov Georgii
 *
 * Contacts:
 * 			krylov.georgii@gmail.com
 * 			https://vk.com/krylov.georgii
 *
 */

#ifndef _TYPE_H_
#define _TYPE_H_ "1.4"
#include "DIIRQ.h"

using byte = unsigned char;

namespace robot {
	namespace irq {
		typedef struct {
			MyRio_IrqDi irqDI0;
			NiFpga_IrqContext irqContext;
			NiFpga_Bool irqThreadRdy;
			uint8_t irqNumber;
		} ThreadResource;
	} /* END namespace irq */
} /* END namespace robot */

#endif /* _TYPE_H_ */
