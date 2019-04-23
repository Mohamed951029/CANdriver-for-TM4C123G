#include "can.h"
#include "gpio_module.h"
#include "debug.h"
#include "uart.h"
#include "application.h"
volatile uint_8 messageCausedInterrupt=0;
static void CAN_PINS_INIT(uint_8 ui8CanNo);
static void CAN_setMessageObjectAsTransmitter(uint_8 StructNoMessageObject,unsigned char canNo);
static void CAN_setMessageObjectsAsReceiverBuffer(uint_8 StructNoMessageObject,unsigned char canNo);
static void CAN_setMessageObjectAsTransmitter_REMOTE_FManualOrAutomatic(uint_8 StructNoMessageObject,unsigned char canNo);
    /*
decription :is used to get the data from the message object in put it in the location specified in the messObjConf
*/
static returnOfFnStatus CAN_getDataFromMessObj_RX(unsigned char messageObjectNo,unsigned char canNo);

volatile CAN_INTERRUPT_DATA_TYPE gflag_CAN_interrupt=NO_INTERRUPT;

void (*can_int_handler)(void) = NULL ;


static void CAN_PINS_INIT(uint_8 ui8CanNo)
{
    volatile unsigned long delay;
    if(CAN0==ui8CanNo)
    {
        RCGC0_CLOCK_MODULE|=(1<<24); //enable the can clock of the module CAN0
        RCGC2_CLOCK_PORT|=(1<<1);//enable  the clock of portB RX PB4 TX PB5
        delay = RCGC2_CLOCK_PORT;
        REG(PORTB,GPIOLOCK_offset)=LOCK_KEY;
        REG(PORTB,GPIOCR_offset)|=(1<<4)|(1<<5);
        REG(PORTB,GPIOAMSEL_offset)&=~((1<<4)|(1<<5));//disable analog mode
        REG(PORTB,GPIODEN_offset)|=(1<<4)|(1<<5);//enable digital mode
        REG(PORTB,GPIOAFSEL_offset)|=(1<<4)|(1<<5);//set the alternative fn
        REG(PORTB,GPIOPCTL_offset)=(REG(PORTB,GPIOPCTL_offset)&0xFF00FFFF)|(0x88<<16);
    }
    else if(CAN1 == ui8CanNo)
    {
        RCGC0_CLOCK_MODULE|=(1<<25); //enable the can clock of the module CAN1
        RCGC2_CLOCK_PORT|=(1<<0);//enable  the clock of portA RX PA0 TX PA1
        delay = RCGC2_CLOCK_PORT;
        REG(PORTA,GPIOLOCK_offset)=LOCK_KEY;
        REG(PORTA,GPIOCR_offset)|=(1<<0)|(1<<1);
        REG(PORTA,GPIOAMSEL_offset)&=~((1<<0)|(1<<1));//disable analog mode
        REG(PORTA,GPIODEN_offset)|=(1<<0)|(1<<1);//enable digital mode
        REG(PORTA,GPIOAFSEL_offset)|=(1<<0)|(1<<1);//set the alternative fn
        REG(PORTA,GPIOPCTL_offset)=(REG(PORTB,GPIOPCTL_offset)&0xFFFFFF00)|(0x88);
    }
}



void CANBitTimingGet(uint_8 ui8CanNo,tCANBitClkParms *psClkParms){


    /********************************************************************************************************************/
    /********************************************************************************************************************/
    /*************************              CANBIT Register Field Setting   p.1063              *************************/
    /*************************              ------------------------------------                *************************/
    /*************************              TSEG2      --> Phase2 - 1                           *************************/
    /*************************              TSEG1      --> Prop + Phase1 - 1                    *************************/
    /*************************              SJW        --> SJW - 1                              *************************/
    /*************************              BRP        --> BRP -1                               *************************/
    /********************************************************************************************************************/
    /********************************************************************************************************************/
    psClkParms->ui32PropPhase1Seg = ((REG(CANn_BASE(ui8CanNo),CANBIT_CAN_BIT_TIME) & CAN_TSEG1_SEG_MSK)>>TSEG1) + 1;
    psClkParms->ui32Phase2Seg = ((REG(CANn_BASE(ui8CanNo),CANBIT_CAN_BIT_TIME) & CAN_TSEG2_SEG_MSK)>>TSEG2) + 1;
    psClkParms->ui32SJW = ((REG(CANn_BASE(ui8CanNo),CANBIT_CAN_BIT_TIME) & CAN_SJW_SEG_MSK)>>SJW) + 1;
    psClkParms->ui32QuantumPrescaler = (((REG(CANn_BASE(ui8CanNo),CANBIT_CAN_BIT_TIME) & CAN_BRP_SEG_MSK)>>BRP)|\
    ((REG(CANn_BASE(ui8CanNo),CANBRPE_CAN_BAUD_EXTENSION)&0x0f)<<6))+1;
}
void CANBitTimingSet(uint_8 ui8CanNo){

    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL)|=(1<<INIT) | (1<<CCE); /*convert the CAN mode from normal to initialization mode
    so write access is enabled for CANBIT*/
    REG(CANn_BASE(ui8CanNo),CANBIT_CAN_BIT_TIME) = (((tCANBitClkParms_config.ui32PropPhase1Seg - 1)<<TSEG1))\
                                                                                |((tCANBitClkParms_config.ui32Phase2Seg - 1)<<TSEG2)\
                                                                                |((tCANBitClkParms_config.ui32SJW - 1)<<SJW)\
                                                                                |(((tCANBitClkParms_config.ui32QuantumPrescaler - 1)&0x3F)<<BRP);
    /*put 6 LSBs of (BRP-1) in CANBIT and 4 MSBs of (BRP-1) in CANn_BASE(ui8CanNo) */
    REG(CANn_BASE(ui8CanNo),CANBRPE_CAN_BAUD_EXTENSION) = (((tCANBitClkParms_config.ui32QuantumPrescaler - 1)>>6));

    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL)&=~ (1<<CCE) ;
    /*
        disable the change in CANBIT but still in intialization mode
    */
}
void CANDisable(uint_8 ui8CanNo){
    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL)|=(1<<INIT);//enter the initialization mode
}

void CANEnable(uint_8 ui8CanNo){
        REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL)&=~(1<<INIT); //enter the normal mode "available node in the CAN bus"
}

uint8_t CANErrCntrGet(uint_8 ui8CanNo,uint_32 *pui32RxCount,uint_32 *pui32TxCount)
{

    *pui32RxCount = (REG(CANn_BASE(ui8CanNo),CANERR_CAN_ERROR_COUNTER) & CAN_RX_ERR_MSK) >> REC;
  *pui32TxCount = (REG(CANn_BASE(ui8CanNo),CANERR_CAN_ERROR_COUNTER) & CAN_TX_ERR_MSK) ;
    if(BIT_IS_SET(REG(CANn_BASE(ui8CanNo),CANERR_CAN_ERROR_COUNTER),RP))//The Receive Error counter has reached the Error Passive level (128 or greater).
    {
        return 1 ;
    }
    else{
        return 0 ;
    }
}

void CANInit(uint_8 ui8CanNo) {
    uint_32 i=0;
    CAN_PINS_INIT(ui8CanNo);//initialize the TX ,RX pins and CLK of CAN MODULE
    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL)|=(1<<INIT);//enter the initialization mode
    if(CAN0==ui8CanNo)
    {
        EN1_INTERRUPT|=(1<<(CAN0_IRQ-32));//is still disabled as IE must be set
        PRI9_INTERRUPT=(PRI9_INTERRUPT&0x00FFFFFF)|(1<<29); //set the priority of the interrupt to 1
    }
    else
    {
        EN1_INTERRUPT|=(1<<(CAN1_IRQ-32));//is still disabled as IE must be set
        PRI10_INTERRUPT=(PRI10_INTERRUPT&0xFFFFFF00)|(2<<5);//set the priority of the interrupt to 2
    }
    while(BIT_IS_SET(REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){};
    REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF) = (1<<CONTROL)|(1<<WRNRD)|(1<<DATAA)|(1<<DATAB)|(1<<ARB)|(1<<MASK);
    REG(CANn_BASE(ui8CanNo),CANIF1DA1_DATA_A1_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1DA2_DATA_A2_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1DB1_DATA_B1_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1DB2_DATA_B2_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1ARB2_ARBITRATION_2_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1ARB1_ARBITRATION_1_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1MCTL_MESSAGE_CONTROL_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1MSK2_MASK_2_IF)=0;
    REG(CANn_BASE(ui8CanNo),CANIF1MSK1_MASK_1_IF)=0;
    for(i=0;i<=32;i++){
        REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF) = i;
        while(BIT_IS_SET(REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){};
    }

}

void CANIntClear(uint_8 ui8CanNo,uint_32 ui32IntClr){

    if(ui32IntClr == INTID_STATUS_INTERRUPT)
        {
            //if the required to clear the status interrupt
            REG(CANn_BASE(ui8CanNo),CANSTS_CAN_STATUS);//read the status reg to clear the interrupt
        }
    else
        {
            //clear the pending interrupt of the given MESSObj
            while(BIT_IS_SET(REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){};
            REG(CANn_BASE(ui8CanNo),CANIF1CMSK_COMMAND_MASK_IF) = (1<<CLRINTPND);//(1<<TXRQST_NEWDAT_CMSK);//(1<<WRNRD);
            REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF) = ui32IntClr & 0x3f ;
            while(BIT_IS_SET(REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){};
      }
}

void CANIntEnable(uint_8 ui8CanNo,uint_32 ui32IntFlags){
    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL) |= ui32IntFlags ;
}

void CANIntDisable(uint_8 ui8CanNo,uint_32 ui32IntFlags){
    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL) &=~ (ui32IntFlags   );
}

void CANIntRegister(void (*pfnHandler)(void)){
    can_int_handler = pfnHandler;
}

void CANIntUnregister(uint_8 ui8CanNo){
    can_int_handler = NULL ; //to clear the call back fn ptr
    REG(CANn_BASE(ui8CanNo),CANCTL_CAN_CONTROL) &=~ (1<<IE);
}

uint_32 CANGetIntStatus(uint_8 ui8CanNo,tCANIntStsReg eIntStsReg){
    uint_32 status;

    switch(eIntStsReg)
    {
        case CAN_INT_STS_CAUSE:
        {
                    //REG(CANn_BASE(CAN0),CANINT_CAN_INTERRUPT)
            status = REG(CANn_BASE(ui8CanNo),CANINT_CAN_INTERRUPT);
            break;
        }
        case CAN_INT_STS_OBJECT:
        {

            status = (REG(CANn_BASE(ui8CanNo),CANMSG1INT_INTERRUPT_PENDING_1) & 0xffff)\
                                            |(REG(CANn_BASE(ui8CanNo),CANMSG2INT_INTERRUPT_PENDING_2) << 16);
            break;
        }

        default:
        {
            status = 0;
            break;
        }
    }

    return status;
}

void CANMessageClear(uint_8 ui8CanNo,uint_32 ui32ObjID){
    while(BIT_IS_SET(REG(uint_32,CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){};

  REG(CANn_BASE(ui8CanNo),CANIF1CMSK_COMMAND_MASK_IF) = (1<<WRNRD) | (1<<ARB);
  REG(CANn_BASE(ui8CanNo),CANIF1ARB1_ARBITRATION_1_IF) = 0;
  REG(CANn_BASE(ui8CanNo),CANIF1ARB2_ARBITRATION_2_IF) = 0;//to clear the MSGVAL which indicates that the Messobjis in valid
  REG(CANn_BASE(ui8CanNo),CANIF1CRQ_COMMAND_REQUEST_IF) = ui32ObjID & 0x3f ;
}

returnOfFnStatus CAN_setPtrOfMessObj(uint_8 MessageObjNUM,uint_8 * ptr)
{
    returnOfFnStatus retValue = ERROR;
    uint_8 i=0;
    do{
        if(messObjConf[i].MessageObjNUM==MessageObjNUM)
        {
            messObjConf[i].ptrToStartOfData=ptr;
            retValue=SUCCESS;
        }
        i++;
    }while((i<NO_STRUCTURE_OF_MESS_OBJ)&&(retValue==ERROR));  //loop until find the required struct of messNO(SUCCESS) or until end No of struct
    return retValue;
}
void CANMessageSet(uint_8 ui8CanNo)
{
    uint_8 i=0;
    for(i=0;i<NO_STRUCTURE_OF_MESS_OBJ;i++)
    {
        if(messObjConf[i].MessType==MSG_OBJ_TYPE_TX)
        {
            CAN_setMessageObjectAsTransmitter(i,ui8CanNo);
        }
        else if((messObjConf[i].MessType==MSG_OBJ_TYPE_RX)||(messObjConf[i].MessType==MSG_OBJ_TYPE_TX_REMOTE))
        {

                            #ifdef DEBUG
                                            //LED_toggle(GREEN,500);
                            #endif
            CAN_setMessageObjectsAsReceiverBuffer(i,ui8CanNo);
        }
        else if((messObjConf[i].MessType==MSG_OBJ_TYPE_RX_REMOTE_MANUALLY)||(messObjConf[i].MessType==MSG_OBJ_TYPE_RX_REMOTE_AUTOMATICALLY))
        {
            CAN_setMessageObjectAsTransmitter_REMOTE_FManualOrAutomatic(i,ui8CanNo);
        }
    }
}





static void CAN_setMessageObjectAsTransmitter(uint_8 StructNoMessageObject,unsigned char canNo)
{

    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){}  //wait till any previous transfer bet IF and message object
    REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)|=(1<<CONTROL)|(1<<WRNRD)|(1<<ARB);
        /*  WRNRD=1 to transfer from IF REgs to MEssObj CNTROL =1 to transfer the control bits of MCTL DatAA =
        DAtAb=1 to transfer the 8 bytes ARB=1 to transfer ID XTD DIR MSVAL*/
    if(messObjConf[StructNoMessageObject].identifierType == IDENTIFIER_11_BITS)
    {
        REG(CANn_BASE(canNo),CANIF1ARB1_ARBITRATION_1_IF)=0;
        REG(CANn_BASE(canNo),CANIF1ARB2_ARBITRATION_2_IF)=(1<<MSGVAL)|(1<<DIR)|(messObjConf[StructNoMessageObject].ui32MsgID<<2);
        /*put the identifier in the ID[10:2] DIR = 1 to make the messageObj transmitter MSGVAL=1 to enable the MESSOBj XTD=0 11 bit identifier */
        REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)=0;
        /*all ids pass and the xtd &dir donot inhibit the filter*/
        REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)=(messObjConf[StructNoMessageObject].ui32MsgIDMask<<2); //take in the consindration of all bits of the filter
    }
    else
    {
      //29 bit identifier
        REG(CANn_BASE(canNo),CANIF1ARB1_ARBITRATION_1_IF)=(messObjConf[StructNoMessageObject].ui32MsgID)&0xFFFF;//ID[0:15] stored in ARB1
        REG(CANn_BASE(canNo),CANIF1ARB2_ARBITRATION_2_IF)=(1<<MSGVAL)|(1<<DIR)|((messObjConf[StructNoMessageObject].ui32MsgID)>>16)|(1<<XTD);
        /*put the identifier in the ID[10:2] DIR = 1 to make the messageObj transmitter MSGVAL=1 to enable the MESSOBj XTD=1 29 bit identifier
        ID[28:16] in ARB2[12:0]*/
        REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)=(messObjConf[StructNoMessageObject].ui32MsgIDMask)&0xFFFF;
        /*all ids pass and the xtd &dir donot inhibit the filter*/
        REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)|=(messObjConf[StructNoMessageObject].ui32MsgIDMask)>>16; //tak
    }
    REG(CANn_BASE(canNo),CANIF1MCTL_MESSAGE_CONTROL_IF)=(messObjConf[StructNoMessageObject].ui32MsgLen<<DLC)|(1<<EOB)\
        |(messObjConf[StructNoMessageObject].ui32Flags);
    /*DLC=8 no of bytes  EOB =1 single message object TXIE=1 when the message has been successful sent which enabled in ui32Flags
        and UMASK exists in ui32Flags(if you want to use the mask for the acceptance filter)
    */
    REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF)=messObjConf[StructNoMessageObject].MessageObjNUM; //transfer the IF to the required message object

}
static void CAN_setMessageObjectsAsReceiverBuffer(uint_8 StructNoMessageObject,unsigned char canNo)
{

    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}  //wait till any previous transfer bet IF and message object
                REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)=(1<<CONTROL)|(1<<WRNRD)|(1<<DATAA)|(1<<DATAB)|(1<<ARB)|(1<<MASK);
                    /*  WRNRD=1 to transfer from IF REgs to MEssObj CNTROL =1 to transfer the control bits of MCTL DatAA =
                    DAtAb=1 to transfer the 8 bytes ARB=1 to transfer ID XTD DIR MSVAL MASK=1 to transfer IDMASK DIR MXTD*/
                REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)&=~(1<<DIR);// DIR=0 receiver message object
                REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)|=(1<<MSGVAL);//enable the mesObj
                if(messObjConf[StructNoMessageObject].identifierType == IDENTIFIER_11_BITS)
                {
                    REG(CANn_BASE(canNo),CANIF2ARB1_ARBITRATION_1_IF)=0;
                    REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)=(REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)&(~(0x7ff<<2)))\
                    |(messObjConf[StructNoMessageObject].ui32MsgID<<2);
                    /*put the identifier in the ID[10:2] DIR = 1 to make the messageObj transmitter MSGVAL=1 to enable the MESSOBj XTD=0 11 bit identifier */
                    REG(CANn_BASE(canNo),CANIF2MSK1_MASK_1_IF)=0;
                    REG(CANn_BASE(canNo),CANIF2MSK2_MASK_2_IF)=0; //take in the consindration of all bits of the filter
                    /*all ids pass and the xtd &dir donot inhibit the filter*/
                    REG(CANn_BASE(canNo),CANIF2MSK2_MASK_2_IF)|=(messObjConf[StructNoMessageObject].ui32MsgIDMask<<2); //take in the consindration of all bits of the filter

                }
                else
                {
                    //29 bit identifier
                    REG(CANn_BASE(canNo),CANIF2ARB1_ARBITRATION_1_IF)=(messObjConf[StructNoMessageObject].ui32MsgID)&0xFFFF;//ID[0:15] stored in ARB1
                    REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)|=((messObjConf[StructNoMessageObject].ui32MsgID)>>16)|(1<<XTD);
                    /*put the identifier in the ID[10:2] DIR = 1 to make the messageObj transmitter MSGVAL=1 to enable the MESSOBj XTD=1 29 bit identifier
                    ID[28:16] in ARB2[12:0]*/
                    REG(CANn_BASE(canNo),CANIF2MSK1_MASK_1_IF)=(messObjConf[StructNoMessageObject].ui32MsgIDMask)&0xFFFF;
                    /*all ids pass and the xtd &dir donot inhibit the filter*/
                    REG(CANn_BASE(canNo),CANIF2MSK2_MASK_2_IF)=0;
                    REG(CANn_BASE(canNo),CANIF2MSK2_MASK_2_IF)|=(messObjConf[StructNoMessageObject].ui32MsgIDMask)>>16; //tak
                }

                REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF)=(messObjConf[StructNoMessageObject].ui32MsgLen<<DLC)|(messObjConf[StructNoMessageObject].ui32Flags);

                /*DLC=8 no of bytes  EOB =1 single message object UMASK=1 use MXTD and MSK and MDIR for the acceptance filter*/
                if((messObjConf[StructNoMessageObject].FifoFlag)== NOT_END_OF_THE_FIFO)
                {
                        //if the MessObj is not the last one then clear the EOB bit and not single FIFO
                    REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF)&=~(1<<EOB);

                }
                else //end of the FIFO
                {

                    REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF)|=(1<<EOB);
                }

                REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)=(messObjConf[StructNoMessageObject].MessageObjNUM); //transfer the IF to the required message object
                while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}
}



static void CAN_setMessageObjectAsTransmitter_REMOTE_FManualOrAutomatic(uint_8 StructNoMessageObject,unsigned char canNo)
{
    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){}  //wait till any previous transfer bet IF and message object
    REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)=(1<<WRNRD)|(1<<CONTROL)|(1<<WRNRD)|(1<<ARB);

        if(messObjConf[StructNoMessageObject].identifierType == IDENTIFIER_11_BITS)
        {
            REG(CANn_BASE(canNo),CANIF1ARB1_ARBITRATION_1_IF)=0;
            REG(CANn_BASE(canNo),CANIF1ARB2_ARBITRATION_2_IF)=(1<<MSGVAL)|(1<<DIR)|(messObjConf[StructNoMessageObject].ui32MsgID<<2);
            /*put the identifier in the ID[10:2] DIR = 1 to make the messageObj transmitter MSGVAL=1 to enable the MESSOBj XTD=0 11 bit identifier */
            REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)=0;
            /*all ids pass and the xtd &dir donot inhibit the filter*/
            REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)|=(messObjConf[StructNoMessageObject].ui32MsgIDMask<<2); //take in the consindration of all bits of the filter
        }
        else
        {
            //29 bit identifier
            REG(CANn_BASE(canNo),CANIF1ARB1_ARBITRATION_1_IF)=(messObjConf[StructNoMessageObject].ui32MsgID)&0xFFFF;//ID[0:15] stored in ARB1
            REG(CANn_BASE(canNo),CANIF1ARB2_ARBITRATION_2_IF)=(1<<MSGVAL)|(1<<DIR)|((messObjConf[StructNoMessageObject].ui32MsgID)>>16)|(1<<XTD);
            /*put the identifier in the ID[10:2] DIR = 1 to make the messageObj transmitter MSGVAL=1 to enable the MESSOBj XTD=1 29 bit identifier
            ID[28:16] in ARB2[12:0]*/
            REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)=(messObjConf[StructNoMessageObject].ui32MsgIDMask)&0xFFFF;
            /*all ids pass and the xtd &dir donot inhibit the filter*/
            REG(CANn_BASE(canNo),CANIF1MSK2_MASK_2_IF)|=(messObjConf[StructNoMessageObject].ui32MsgIDMask)>>16; //tak
        }
        REG(CANn_BASE(canNo),CANIF1MCTL_MESSAGE_CONTROL_IF)=(messObjConf[StructNoMessageObject].ui32MsgLen<<DLC)|(1<<EOB)\
        |(messObjConf[StructNoMessageObject].ui32Flags);
       /*DLC=8 no of bytes  EOB =1 single message object TXIE=1 when the message has been successful sent which enabled in ui32Flags
            and UMASK exists in ui32Flags(if you want to use the mask for the acceptance filter)
        */
        if(messObjConf[StructNoMessageObject].MessType==MSG_OBJ_TYPE_RX_REMOTE_AUTOMATICALLY)
        {
            REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)|=(1<<DATAA)|(1<<DATAB);
            REG(CANn_BASE(canNo),CANIF1DA1_DATA_A1_IF)=(messObjConf[StructNoMessageObject].ptrToStartOfData[1]<<8)\
            |messObjConf[StructNoMessageObject].ptrToStartOfData[0];
            REG(CANn_BASE(canNo),CANIF1DA2_DATA_A2_IF)=(messObjConf[StructNoMessageObject].ptrToStartOfData[3]<<8)\
            |messObjConf[StructNoMessageObject].ptrToStartOfData[2];
            REG(CANn_BASE(canNo),CANIF1DB1_DATA_B1_IF)=(messObjConf[StructNoMessageObject].ptrToStartOfData[5]<<8)\
            |messObjConf[StructNoMessageObject].ptrToStartOfData[4];
            REG(CANn_BASE(canNo),CANIF1DB2_DATA_B2_IF)=(messObjConf[StructNoMessageObject].ptrToStartOfData[7]<<8)\
            |messObjConf[StructNoMessageObject].ptrToStartOfData[6];
            REG(CANn_BASE(canNo),CANIF1MCTL_MESSAGE_CONTROL_IF)|=(1<<RMTEN);
        }
        else if(messObjConf[StructNoMessageObject].MessType==MSG_OBJ_TYPE_RX_REMOTE_MANUALLY)
        {
            REG(CANn_BASE(canNo),CANIF1MCTL_MESSAGE_CONTROL_IF)|=(1<<UMASK);
        }

     REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF)=messObjConf[StructNoMessageObject].MessageObjNUM; //transfer the IF to the required message object
}
/*void CAN_setMessageObjectAsReceiver(unsigned char messageObjectNo,unsigned char canNo,unsigned short filter_id)
{
    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}  //wait till any previous transfer bet IF and message object
    REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)|=(1<<CONTROL)|(1<<WRNRD)|(1<<DATAA)|(1<<DATAB)|(1<<ARB)|(1<<MASK);
    //      WRNRD=1 to transfer from IF REgs to MEssObj CNTROL =1 to transfer the control bits of MCTL DatAA =
    //  DAtAb=1 to transfer the 8 bytes ARB=1 to transfer ID XTD DIR MSVAL MASK=1 to transfer IDMASK DIR MXTD
    REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)&=~((1<<XTD)|(1<<DIR));//11 bit identifier DIR=0 receiver message object
    REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)=(REG(CANn_BASE(canNo),CANIF2ARB2_ARBITRATION_2_IF)&0xFFFF0000)\
            |(1<<MSGVAL)|(filter_id<<2);//enable the mesObj
    REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF)|=(8<<DLC)|(1<<EOB)|(1<<UMASK);//|(1<<RXIE);
//DLC=8 no of bytes  EOB =1 single message object UMASK=1 use MXTD and MSK and MDIR for the acceptance filter
    REG(CANn_BASE(canNo),CANIF2MSK2_MASK_2_IF)=0;
    //  /*all ids pass and the xtd &dir donot inhibit the filter
    REG(CANn_BASE(canNo),CANIF2MSK2_MASK_2_IF)|=(0x7FF<<2); //take in the consindration of all bits of the filter
    REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)|=messageObjectNo; //transfer the IF to the required message object
    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}
}*/
void CAN_transmitRemoteFrame(unsigned char messageObjectNo,unsigned char canNo)
{

        REG(CANn_BASE(canNo),CANSTS_CAN_STATUS)&=~((1<<TXOK)|(1<<RXOK));

    REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)&=~((1<<ARB));

        //|(1<<CONTROL)

    REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)=(1<<WRNRD)|(1<<CONTROL);
    /*TXRQST_NEWDAT_CMSK=1 to request the transmitting*/
    REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF)|=(1<<TXRQST_MCTL);
    REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)=(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)&(~0x3F))|messageObjectNo; //transfer the IF to the required message object
    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}




}
returnOfFnStatus CAN_putDataInMessObj(unsigned char messageObjectNo,unsigned char canNo)
{
    returnOfFnStatus returnStatus = ERROR;
    uint_8 i=0;
    while((returnStatus== ERROR)&&(i<NO_STRUCTURE_OF_MESS_OBJ))
    {
        if(messObjConf[i].MessageObjNUM==messageObjectNo)
        {
            returnStatus = SUCCESS;
            REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)|=(1<<WRNRD)|(1<<DATAA)|(1<<DATAB);
            REG(CANn_BASE(canNo),CANIF1DA1_DATA_A1_IF)=(messObjConf[i].ptrToStartOfData[1]<<8)|messObjConf[i].ptrToStartOfData[0];
            REG(CANn_BASE(canNo),CANIF1DA2_DATA_A2_IF)=(messObjConf[i].ptrToStartOfData[3]<<8)|messObjConf[i].ptrToStartOfData[2];
            REG(CANn_BASE(canNo),CANIF1DB1_DATA_B1_IF)=(messObjConf[i].ptrToStartOfData[5]<<8)|messObjConf[i].ptrToStartOfData[4];
            REG(CANn_BASE(canNo),CANIF1DB2_DATA_B2_IF)=(messObjConf[i].ptrToStartOfData[7]<<8)|messObjConf[i].ptrToStartOfData[6];
            REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer the IF to the required message object
            while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){}
        }
        i++;
    }
    return returnStatus;
}
returnOfFnStatus CAN_transmitDATA(uint_8 messageObjectNo,uint_8 canNo)
{
     returnOfFnStatus returnStatus = SUCCESS;
    REG(CANn_BASE(canNo),CANSTS_CAN_STATUS)&=~(1<<TXOK);

    REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)=(1<<WRNRD)|(1<<TXRQST_NEWDAT_CMSK);
    /*TXRQST_NEWDAT_CMSK=1 to request the transmitting*/
    REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer the IF to the required message object
    while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){}



    return returnStatus;
}
static returnOfFnStatus CAN_getDataFromMessObj_RX(unsigned char messageObjectNo,unsigned char canNo)
{
    returnOfFnStatus retValue=ERROR;
    uint_8 i=0;
    do{
        if(messageObjectNo==messObjConf[i].MessageObjNUM )
        {
            REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)=(1<<CONTROL)|(1<<DATAA)|(1<<DATAB);
            //to clear in NEWDAt in MCTL REG and INTID
            REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer  the required message object     to IF
            while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}
            messObjConf[i].ptrToStartOfData[0]=(REG(CANn_BASE(canNo),CANIF2DA1_DATA_A1_IF)&0x00FF);
            messObjConf[i].ptrToStartOfData[1]=(REG(CANn_BASE(canNo),CANIF2DA1_DATA_A1_IF)&0xFF00)>>8;
            messObjConf[i].ptrToStartOfData[2]=(REG(CANn_BASE(canNo),CANIF2DA2_DATA_A2_IF)&0x00FF);
            messObjConf[i].ptrToStartOfData[3]=(REG(CANn_BASE(canNo),CANIF2DA2_DATA_A2_IF)&0xFF00)>>8;
            messObjConf[i].ptrToStartOfData[4]=(REG(CANn_BASE(canNo),CANIF2DB1_DATA_B1_IF)&0x00FF);
            messObjConf[i].ptrToStartOfData[5]=(REG(CANn_BASE(canNo),CANIF2DB1_DATA_B1_IF)&0xFF00)>>8;
            messObjConf[i].ptrToStartOfData[6]=(REG(CANn_BASE(canNo),CANIF2DB2_DATA_B2_IF)&0x00FF);
            messObjConf[i].ptrToStartOfData[7]=(REG(CANn_BASE(canNo),CANIF2DB2_DATA_B2_IF)&0xFF00)>>8;
            messObjConf[i].flagNewData=MSG_HAS_NEW_DATA;    //to indicate that the MEss has new dATA
            retValue=SUCCESS;
            REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)=(1<<TXRQST_NEWDAT_CMSK)|(1<<CLRINTPND);
            REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer  the required message object     to IF
            while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}
        }
        i++;
    }while((i<NO_STRUCTURE_OF_MESS_OBJ)&&(retValue==ERROR));

    return retValue;
}

returnOfFnStatus CAN_receiveDATA(unsigned char messageObjectNo,unsigned char canNo)
{

        returnOfFnStatus returnStatus = SUCCESS;
        REG(CANn_BASE(canNo),CANSTS_CAN_STATUS)&=~(1<<RXOK);

        while((messageObjectNo!=0)&&(returnStatus==SUCCESS)) // if the last loop end with success and it was not a last MessObj
        {

                            #ifdef DEBUG
                                    UART_SEND_STRING("loop2:",UART0);
                                    write_number_max_5_digits(messageObjectNo,UART0);
                                    UART_SEND_STRING("\n",UART0);
                            #endif

            REG(CANn_BASE(canNo),CANIF2CMSK_COMMAND_MASK_IF)=(1<<CONTROL);//WRNRD=0 to transfer from message onj to IF
            REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer  the required message object     to IF
            while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}
            if(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF),NEWDAT_MCTL)) //if there is new data
            {
                #ifdef DEBUG

                                    UART_SEND_STRING("if1\n",UART0);
                            #endif
                 if(CAN_getDataFromMessObj_RX(messageObjectNo,canNo)==SUCCESS)
                     {


                        if(!(REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF)&(1<<EOB))) // if EOB=0 means that it's not the last Mess
                        {
                            #ifdef DEBUG
                                    UART_SEND_STRING("EOB=0 4\n",UART0);

                            #endif
                            messageObjectNo++;
                        }
                        else
                        {
                            messageObjectNo=0;//clear the MessObjNo to indicate that is the last Mess
                        #ifdef DEBUG
                                    UART_SEND_STRING("in else4\n",UART0);

                        #endif
                        }
                        returnStatus= SUCCESS;
                    }
            }
            else if(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF),MSGLST)) //if there is new data
            {
                    #ifdef DEBUG

                                    UART_SEND_STRING("if2\n",UART0);
                            #endif
                CLEAR_BIT(REG(CANn_BASE(canNo),CANIF2MCTL_MESSAGE_CONTROL_IF),MSGLST);
                returnStatus= ERROR;
                REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer the IF to the required message object
                while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF2CRQ_COMMAND_REQUEST_IF),BUSY)){}
            }
            else
            {
                    #ifdef DEBUG

                                    UART_SEND_STRING("else\n",UART0);
                            #endif
                messageObjectNo=0;//clear the MessObjNo to break the loop
            }

        }
        /*#ifdef DEBUG
                        UART_SEND_STRING("end receive\n",UART0);
            #endif*/
        return  returnStatus;
}
returnOfFnStatus CAN_receiveRemoteFrameAsDataFrame(unsigned char messageObjectNo,unsigned char canNo)
{
        uint_8 i=0;
        returnOfFnStatus returnStatus = ERROR;
        while((returnStatus ==ERROR)&&(i<NO_STRUCTURE_OF_MESS_OBJ))
        {
            if(messObjConf[i].MessageObjNUM ==messageObjectNo )
            {
                returnStatus=SUCCESS;
            }
            else
            {
                i++;
            }
        }
        if(returnStatus==SUCCESS)
        {
            REG(CANn_BASE(canNo),CANSTS_CAN_STATUS)&=~(1<<RXOK);
            REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)=(1<<CONTROL);//WRNRD=0 to transfer from message onj to IF

            REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer  the required message object     to IF
            while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){}
            if(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1MCTL_MESSAGE_CONTROL_IF),NEWDAT_MCTL)) //if there is new Remote frame
            {
                REG(CANn_BASE(canNo),CANIF1CMSK_COMMAND_MASK_IF)|=(1<<TXRQST_NEWDAT_CMSK)|(1<<CLRINTPND);//to clear in NEWDAt in MCTL REG
                messObjConf[i].flagNewData = MSG_RECIEVED_RF_AS_DATA_FRAME;
                returnStatus=SUCCESS;
            }
            else
            {
                messObjConf[i].flagNewData = MSG_NOT_RECIEVED_RF_AS_DATA_FRAME;
                returnStatus=ERROR;
            }

            REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF)=messageObjectNo; //transfer the IF to the required message object
            while(BIT_IS_SET(REG(CANn_BASE(canNo),CANIF1CRQ_COMMAND_REQUEST_IF),BUSY)){}
        }

        return  returnStatus;
}
returnOfFnStatus CAN_receiveDATAInterruptOrCheckInPolling(unsigned char * messageObjectNo_ptr,unsigned char canNo,ErrorTypesOfLEC *errorType)
{
    returnOfFnStatus return_var = SUCCESS;
    uint_8 i=0;
    volatile  uint_32 reg_read=0;
            #ifdef DEBUG_2
            UART_SEND_STRING("interrupt\n",UART0);

            #endif
     if (REG(CANn_BASE(CAN0),CANINT_CAN_INTERRUPT)==INTID_STATUS_INTERRUPT)
  {
            #ifdef DEBUG_2
            UART_SEND_STRING("status cause the interrupt:\n",UART0);

            #endif
            //reg_read=REG(CANn_BASE(CAN0),CANSTS_CAN_STATUS);//read the status register to clear the flag INTID and CANINT



            if(reg_read&(1<<RXOK))
            {
                REG(CANn_BASE(CAN0),CANSTS_CAN_STATUS)&=~(1<<RXOK);//clear the RXOK "successfull transmission"
                #ifdef DEBUG
                    UART_SEND_STRING("RXOK_1\n",UART0);
                #endif
            }
            else    if(reg_read&(1<<TXOK))
            {
                REG(CANn_BASE(CAN0),CANSTS_CAN_STATUS)&=~(1<<TXOK);//clear the RXOK "successfull transmission"
                #ifdef DEBUG
                    UART_SEND_STRING("TXOK_1\n",UART0);
                #endif
            }
            else if(reg_read&LEC_FIELD)//LEC error
            {
                *errorType = reg_read&LEC_FIELD; //put the type of the error
                return_var = ERROR; //indication for error

            }
            else if((reg_read&CAN_STATUS_EWARN_FLAG)||(reg_read&CAN_STATUS_BUS_OFF_BOFF_FLAG)) // interrupt due to error
            {
                *errorType = 0; //indication of error interrupt
                return_var = ERROR; //indication for error
            }


    }
    if(((REG(CANn_BASE(CAN0),CANINT_CAN_INTERRUPT)>0)&&(REG(CANn_BASE(CAN0),CANINT_CAN_INTERRUPT)<=32))||(gflag_CAN_interrupt==INTERRUPT_EXIST)) // message OBJ the source of the interrupt
  {
                 #ifdef DEBUG_2
            UART_SEND_STRING("Entered\n",UART0);

            #endif
        if(gflag_CAN_interrupt==INTERRUPT_EXIST){
            *messageObjectNo_ptr=messageCausedInterrupt;
        }
        else{
            *messageObjectNo_ptr=REG(CANn_BASE(CAN0),CANINT_CAN_INTERRUPT);
        }

        return_var=ERROR;//to enter the first loop
        while((return_var ==ERROR)&&(i<NO_STRUCTURE_OF_MESS_OBJ))
        {
            if(*messageObjectNo_ptr==messObjConf[i].MessageObjNUM )
            {
                return_var=SUCCESS;
            }
            else
            {
                i++;
            }
        }
        #ifdef DEBUG
            UART_SEND_STRING("i:",UART0);
            write_number_max_5_digits(i,UART0);

            UART_SEND_STRING("type:",UART0);
            write_number_max_5_digits(messObjConf[i].MessType,UART0);
            UART_SEND_STRING("\n",UART0);
        #endif
        #ifdef DEBUG_2
            UART_SEND_STRING("messeage cause the interrupt:",UART0);
            write_number_max_5_digits(i,UART0);

        #endif
        if((messObjConf[i].MessType==MSG_OBJ_TYPE_RX)||(messObjConf[i].MessType==MSG_OBJ_TYPE_TX_REMOTE))
        {

            if(CAN_receiveDATA(*messageObjectNo_ptr,canNo)== SUCCESS)
            {
                return_var = SUCCESS;
                #ifdef DEBUG
                        UART_SEND_STRING("data frame\n",UART0);
            #endif
            }

        }
        else if(messObjConf[i].MessType==MSG_OBJ_TYPE_RX_REMOTE_MANUALLY)
        {
                #ifdef DEBUG
                        UART_SEND_STRING("remote frame1\n",UART0);
            #endif
            if(CAN_receiveRemoteFrameAsDataFrame(*messageObjectNo_ptr,canNo)==SUCCESS)
            {
                    return_var = SUCCESS;
            #ifdef DEBUG
                        UART_SEND_STRING("remote frame2\n",UART0);
                UART_SEND_STRING("flag:",UART0);
            write_number_max_5_digits(messObjConf[i].flagNewData,UART0);
            #endif
            }
        }
    }

            #ifdef DEBUG_2
            UART_SEND_STRING("end\n",UART0);

            #endif
    return return_var;
}
void CAN_RECIEVE_FROM_ISR (uint_8 mssgobj_no,uint_8 canBase){
        CAN_receiveDATA(mssgobj_no,canBase);
}
void CAN0_ISR(void)
{
    uint_8 messageObjNum=0,error_type=0;
    static uint_8 i=0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CANIntClear(CAN0,INTID_STATUS_INTERRUPT);
    messageCausedInterrupt=CANGetIntStatus(CAN0,CAN_INT_STS_CAUSE);
    CANIntClear(CAN0,messageCausedInterrupt);
    #ifdef DEBUG_2
        if(i==0)
        {
            LED_on(BLUE);
            LED_off(GREEN);
            i=1;
        }
        else
        {
            LED_off(BLUE);
            LED_on(GREEN);
            i=~i;
        }
    #endif


        switch(messageCausedInterrupt){
            case 1 :
                gflag_CAN_interrupt = INTERRUPT_EXIST;
                xSemaphoreGiveFromISR(xSemaphorSpeed,&xHigherPriorityTaskWoken);
            break;
            case 2 :
                gflag_CAN_interrupt = INTERRUPT_EXIST;
                xSemaphoreGiveFromISR(xSemaphorANGLE,&xHigherPriorityTaskWoken);
            break;
            case 4 :
                gflag_CAN_interrupt = INTERRUPT_EXIST;
                xSemaphoreGiveFromISR(xSemaphorDIST,&xHigherPriorityTaskWoken);
            break;
            case 15 :
                gflag_CAN_interrupt = INTERRUPT_EXIST;
                xSemaphoreGiveFromISR(xSemaphorGPS,&xHigherPriorityTaskWoken);
            break;
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
       /* UART_SEND_STRING("\n",UART0);
        write_number_max_5_digits(messageCausedInterrupt,UART0);*/


}
