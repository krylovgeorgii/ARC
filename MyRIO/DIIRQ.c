/*
 * Configuration for Digital Input Interrupt Request (IRQ)
 *
 * Copyright (c) 2015,
 * National Instruments.
 * All rights reserved.
 */
#include <stdio.h>

/*
 * Include the myRIO header file.
 * The target type must be defined in your project, as a stand-alone #define,
 * or when calling the compiler from the command-line.
 */
#include "MyRio.h"
#include "DIIRQ.h"


/*
 * Declare the myRIO NiFpga_Session so that it can be used by any function in
 * this file. The variable is actually defined in myRIO.c.
 *
 * This removes the need to pass the myrio_session around to every function and
 * only has to be declared when it is being used.
 */
extern NiFpga_Session myrio_session;


/**
 * Reserve the interrupt from FPGA and configure DI IRQ.
 *
 * @param[in]  irqChannel   A structure containing the registers and settings
                                for a particular analog IRQ I/O to modify.
 * @param[in]  irqContext   IRQ context under which you need to reserve.
 * @param[in]  irqNumber    The IRQ number (IRQNO_MIN-IRQNO_MAX).
 * @param[in]  count        The incremental times that you use to trigger the interrupt.
 * @param[in]  type         The trigger type that you use to increment the count.
 * @return the configuration status.
 */
int32_t Irq_RegisterDiIrq(MyRio_IrqDi* irqChannel, 
                          NiFpga_IrqContext* irqContext,
                          uint8_t irqNumber, 
                          uint32_t count, 
                          Irq_Dio_Type type)
{
    int32_t status;
    uint8_t cnfgValue;
    uint16_t typeValue;

    /*
     * Reserve an IRQ context. IRQ contexts are single-threaded; only one thread
     * can wait with a particular context at any given time. To minimize jitter
     * when first waiting on IRQs, reserve as many contexts as the application requires.
     * If a context is successfully reserved, you must unreserve it later.
     * Otherwise a memory leak will occur.
     */
    status = NiFpga_ReserveIrqContext(myrio_session, irqContext);

    /*
     * Check if there was an error when you reserved an IRQ.
     *
     * If there was an error, print an error message to stdout and return the configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "A required NiFpga_IrqContext was not reserved.")

    /*
     * Limit the IRQ number within a range,
     * if the entered value is out of range, print an error message.
     */
    if (irqNumber > IRQNO_MAX || irqNumber < IRQNO_MIN)
    {
        printf("The specified IRQ Number is out of range.\n");
        return NiMyrio_Status_IrqNumberNotUsable;
    }

    /*
     * Check if the IRQ number or channel value already exists in the resource list,
     * return the configuration status and print error message.
     */
    status = Irq_CheckReserved(irqChannel->dioChannel, irqNumber);
    if (status == NiMyrio_Status_IrqNumberNotUsable)
    {
        printf("You have already registered an interrupt with the same interrupt number.\n");
        return status;
    }
    else if (status == NiMyrio_Status_IrqChannelNotUsable)
    {
        printf("You have already registered an interrupt with the same channel name.\n");
        return status;
    }

    /*
     * Write the value to the DI IRQ number register.
     */
    status = NiFpga_WriteU8(myrio_session, irqChannel->dioIrqNumber, irqNumber);

    /*
     * Check if there was an error when your wrote to the DI IRQ number register.
     *
     * If there was an error, print an error message to stdout and return the configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not write to DI IRQ number register!")

    /*
     * Write the value to the DI IRQ count register.
     */
    status = NiFpga_WriteU32(myrio_session, irqChannel->dioCount, count);

    /*
     * Check if there was an error when you reserved an IRQ.
     *
     * If there was an error, print an error message to stdout and return the configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not write to DI IRQ count register!")

    /*
     * Get the current value of the DI rising-configure register.
     */
    status = NiFpga_ReadU8(myrio_session, irqChannel->dioIrqRisingEdge, &cnfgValue);
    typeValue = (uint16_t) cnfgValue;

    /*
     * Get the current value of the DI falling-configure register.
     * Merge it with the rising-configure register and write to typeValue.
     */
    NiFpga_MergeStatus(&status,
            NiFpga_ReadU8(myrio_session, irqChannel->dioIrqFallingEdge, &cnfgValue));
    typeValue = typeValue | (cnfgValue << 8);

    /*
     * Check if there was an error reading from the DI rising/falling configure registers.
     *
     * If there was an error, the rest of the function cannot execute successfully, print 
     * an error message to stdout and return the configuration status from the earlier 
     * execution of the function.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not read from the DI rise/fall configure registers!")

    /*
     * Get the current value of the DI configure register.
     */
    status = NiFpga_ReadU8(myrio_session, irqChannel->dioIrqEnable, &cnfgValue);

    /*
     * Check if there was an error when you reserved an IRQ.
     *
     * If there was an error, print an error message to stdout and return the configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not read from the DI configure register!")

    /*
     * Configure the IRQ triggered-type for the particular digital IRQ I、O.
     */
    if (irqChannel->dioChannel == Irq_Dio_A0)
    {
        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set which IO is enabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A0_Enable);
        cnfgValue = cnfgValue | Irq_Dio_A0_Enable;

        /*
         * Clear the value of the masked bits in the DI configure register and set the I/O to enable.
         */
        typeValue = typeValue & (~Irq_Dio_A0_Edge);
        if (type == Irq_Dio_RisingEdge)
        {
            typeValue = typeValue | Irq_Dio_A0_RisingEdge;
        }
        else if (type == Irq_Dio_FallingEdge)
        {
            typeValue = typeValue | Irq_Dio_A0_FallingEdge;
        }
        else if (type == Irq_Dio_Edge)
        {
            typeValue = typeValue | Irq_Dio_A0_Edge;
        }
    }
    else if (irqChannel->dioChannel == Irq_Dio_A1)
    {
        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set which IO is enabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A1_Enable);
        cnfgValue = cnfgValue | Irq_Dio_A1_Enable;

        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set the triggered type.
         */
        typeValue = typeValue & (~Irq_Dio_A1_Edge);
        if (type == Irq_Dio_RisingEdge)
        {
            typeValue = typeValue | Irq_Dio_A1_RisingEdge;
        }
        else if (type == Irq_Dio_FallingEdge)
        {
            typeValue = typeValue | Irq_Dio_A1_FallingEdge;
        }
        else if (type == Irq_Dio_Edge)
        {
            typeValue = typeValue | Irq_Dio_A1_Edge;
        }
    }
    else if (irqChannel->dioChannel == Irq_Dio_A2)
    {
        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set which IO is enabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A2_Enable);
        cnfgValue = cnfgValue | Irq_Dio_A2_Enable;

        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set the triggered type.
         */
        typeValue = typeValue & (~Irq_Dio_A2_Edge);
        if (type == Irq_Dio_RisingEdge)
        {
            typeValue = typeValue | Irq_Dio_A2_RisingEdge;
        }
        else if (type == Irq_Dio_FallingEdge)
        {
            typeValue = typeValue | Irq_Dio_A2_FallingEdge;
        }
        else if (type == Irq_Dio_Edge)
        {
            typeValue = typeValue | Irq_Dio_A2_Edge;
        }
    }
    else if (irqChannel->dioChannel == Irq_Dio_A3)
    {
        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set which IO is enabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A3_Enable);
        cnfgValue = cnfgValue | Irq_Dio_A3_Enable;

        /*
         * Clear the value of the masked bits in the DI configure register, then
         * set the triggered type.
         */
        typeValue = typeValue & (~Irq_Dio_A3_Edge);
        if (type == Irq_Dio_RisingEdge)
        {
            typeValue = typeValue | Irq_Dio_A3_RisingEdge;
        }
        else if (type == Irq_Dio_FallingEdge)
        {
            typeValue = typeValue | Irq_Dio_A3_FallingEdge;
        }
        else if (type == Irq_Dio_Edge)
        {
            typeValue = typeValue | Irq_Dio_A3_Edge;
        }
    }

    /*
     * Write the new value of the DI enable configure register to the device.
     */
    status = NiFpga_WriteU8(myrio_session, irqChannel->dioIrqEnable, cnfgValue);

    /*
     * Check if there was an error writing to DI enable configure registers.
     *
     * If there was an error then print an error message to stdout and return configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not write to the DI enable configure registers!")

    /*
     * Write the new value of the DI rise-configure register to the device.
     */
    status = NiFpga_WriteU8(myrio_session, irqChannel->dioIrqRisingEdge, (uint8_t) typeValue);

    /*
     * Write the new value of the DI fall-configure register to the device.
     */
    NiFpga_MergeStatus(&status,
            NiFpga_WriteU8(myrio_session, irqChannel->dioIrqFallingEdge, (uint8_t)(typeValue >> 8)));

    /*
     * Check if there was an error writing to DI rise/fall configure registers.
     *
     * If there was an error then print an error message to stdout.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not write to the DI rise/fall configure registers!")

    /*
     * Add the channel value and IRQ number in the list.
     */
    Irq_AddReserved(irqChannel->dioChannel, irqNumber);

    return NiMyrio_Status_Success;
}


/**
 * Unreserve the interrupt from FPGA, and disable the particular digital IRQ IO,
 * clear according channel value and IRQ number in the resource list.
 * So the IO can be configured in the next time.
 *
 * @param[in]  irqChannel  A struct containing the registers and settings
 *                             for a particular analog IRQ IO to modify.
 * @param[in]  irqContext  IRQ context with to unreserve.
 * @return the configuration status.
 */
int32_t Irq_UnregisterDiIrq(MyRio_IrqDi* irqChannel, 
                            NiFpga_IrqContext irqContext,
                            uint8_t irqNumber)
{
    int32_t status;
    uint8_t cnfgValue;

    /*
     * Limit the IRQ number within a range,
     * if the entered value is out of range, print an error message.
     */
    if (irqNumber > IRQNO_MAX || irqNumber < IRQNO_MIN)
    {
        printf("The specified IRQ Number is out of range.\n");
        return NiMyrio_Status_IrqNumberNotUsable;
    }

    /*
     * Check if the specified IRQ resource is registered.
     */
    status = Irq_CheckReserved(irqChannel->dioChannel, irqNumber);
    if (status == NiMyrio_Status_Success)
    {
        /*
         * Did not find the resource in the list
         */
        printf("You didn't register an interrupt with this IRQ number.\n");
        return NiMyrio_Status_Success;
    }

    /*
     * Get the current value of the DI configure register.
     */
    status = NiFpga_ReadU8(myrio_session, irqChannel->dioIrqEnable, &cnfgValue);

    /*
     * Check if there was an error reading from the DI configure register.
     *
     * If there was an error then print an error message to stdout and return configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not read from the DI configure register!")

    /*
     * Disable the specified channel.
     */
    if (irqChannel->dioChannel == Irq_Dio_A0)
    {
        /*
         * Clear the value of the masked bits in the DI configure register. This is
         * done so DI0 is disabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A0_Enable);
    }
    else if (irqChannel->dioChannel == Irq_Dio_A1)
    {
        /*
         * Clear the value of the masked bits in the DI configure register. This is
         * done so DI1 is disabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A1_Enable);
    }
    else if (irqChannel->dioChannel == Irq_Dio_A2)
    {
        /*
         * Clear the value of the masked bits in the DI configure register. This is
         * done so DI2 is disabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A2_Enable);
    }
    else if (irqChannel->dioChannel == Irq_Dio_A3)
    {
        /*
         * Clear the value of the masked bits in the DI configure register. This is
         * done so DI3 is disabled.
         */
        cnfgValue = cnfgValue & (~Irq_Dio_A3_Enable);
    }

    /*
     * Write the new value of the DI configure register to the device.
     */
    status = NiFpga_WriteU8(myrio_session, irqChannel->dioIrqEnable, cnfgValue);

    /*
     * Check if there was an error writing to DI configure register.
     *
     * If there was an error then print an error message to stdout and return configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not write to the AI configure register!")

    /*
     * Remove the reserved resource in the list.
     */
    status = Irq_RemoveReserved(irqNumber);
    /*
     * Check if there was an error releasing the resource from list.
     *
     * If there was an error then print an error message to stdout.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "Could not release the irq resource!");

    /*
     * Unreserve an IRQ context obtained from Irq_ReserveIrqContext.
     * The returned NiFpga_Status value is stored for error checking.
     */
    status = NiFpga_UnreserveIrqContext(myrio_session, irqContext);

    /*
     * Check if there was an error when unreserve an IRQ.
     *
     * If there was an error then print an error message to stdout and return configuration status.
     */
    MyRio_ReturnStatusIfNotSuccess(status,
            "A required NiFpga_IrqContext was not unreserved.")

    return NiMyrio_Status_Success;
}
