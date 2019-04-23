#ifndef CAN_H
#define CAN_H
#include "common_macros.h"
#include "data_types.h"

#include "can_conf.h"
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/*                                                                                               CAN MODULE DEFINITIONS                                                                                             */
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/* INTERRUPT FLAGS " in CANCTL Register " */
#define CAN_INT_ERROR                   0x8                                                         //Error interrupt generation
#define CAN_INT_STATUS              0x4                                                     //status interrupt generation
#define CAN_INT_MASTER              0x2                                                 //Enable CAN Interrupts

/* BUS STATUS FLAGS in CANSTS Register  */
/*
 * flag and enable bits
 */
#define CAN_INT_STATUS_ENABLE                           ((uint_32)0x04)  /*(in CANCTL)to enable the interrupt when status error LEC oR
                                                                         successful receiving or transmission when TXOK or RXOK change */
#define CAN_INT_ERROR_ENABLE                            ((uint_32)0x08)  /*(in CANCTL) to enable error interrupt which occurs when BOFF or EWRAN change*/

#define CAN_INT_MASTER_ENABLE                           ((uint_32)0x02) /*(in CANCTL)to enable all interrupt As no interrupt is generated unless this bit is set*/

#define CAN_STATUS_BUS_OFF_BOFF_FLAG                    ((uint_32)0x80)/*(in CANSTS) it's set when the controller in bus off TEC>255*/

#define CAN_STATUS_EPASS_FLAG                           ((uint_32)0x20)/*(in CANSTS) the controller in error passive and doesn't generate error interrupt*/

#define CAN_STATUS_EWARN_FLAG                           ((uint_32)0x40)/*(in CANSTS) the TEC OR REC >=96 and it generates error interrupt*/

#define CAN_STATUS_RXOK_FLAG                            ((uint_32)0x10)/*(in CANSTS) received message successfully and must be cleared by writing 0 to it*/

#define CAN_STATUS_TXOK_FLAG                            ((uint_32)0x08)/*(in CANSTS) transmitted message successfully and must be cleared by writing 0 to it*/

#define MSG_OBJ_DATA_LOST                               ((uint_32)(1<<14))/*(in CANIFnMCTL) it's set when new data is stored when new_data bit is set(valid only)
                                                                            only when MESOBJ is receiver             */
#define MSG_OBJ_EXTENDED_ID                             ((uint_32)(1<<14))/*(in CANIFnARB2) (1) 29 bit identifier (0) 11 bit identifier*/

#define MSG_OBJ_FIFO                                    ((uint_32)(1<<7))/*(in CANIFnMCTL)  (1)=single message or last message obj in FIFO
                                                                                            (0)= a message obj in fifo which isn't the last     */
#define MSG_OBJ_NEW_DATA                                ((uint_32)(1<<15))/*(in CANIFnMCTL) (1)=new data has been written by message handler or the CPU */

#define MSG_OBJ_REMOTE_FRAME                            ((uint_32)(1<<9))/*(in CANIFnMCTL) (1)=enable the remote frame(set TXRQST in CANIFnMCTL )
                                                                             or indication of receiving remote frame*/
#define MSG_OBJ_RX_INT_ENABLE                           ((uint_32)(1<<10))/*((in CANIFnMCTL))(1)= means at successful receiving set INTPND of CANIFnMCTL*/

#define MSG_OBJ_TX_INT_ENABLE                           ((uint_32)(1<<11))/*((in CANIFnMCTL))(1)= means at successful transmitting set INTPND of CANIFnMCTL*/

#define MSG_OBJ_USE_DIR_FILTER                          ((uint_32)(1<<14))/*(in CANIFnMSK2) (1)means use dir for the acceptance filter*/

#define MSG_OBJ_USE_EXT_FILTER                          ((uint_32)(1<<15))/*(in CANIFnMSK2) (1)use xtd fot the acceptance filter*/

#define INTID_STATUS_INTERRUPT                          ((uint_32)(0x8000))/*INTID =0x8000 which indicate for status interrupt*/

#define MSG_OBJ_USE_ID_FILTER                                                       ((uint_32)(1<<12))  /*(in CANIFnMCTL) UMASK=1 useUse mask (MSK, MXTD, and MDIR bits in theCANIFnMSKn registers) for acceptance filtering*/

#define LEC_FIELD                                                                               ((uint_32)(0x0007))/*the position of LEC field in cansts */
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/*                                                                                               CAN Masks                                                                                                                          */
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/* CAN Segments Masks */
#define CAN_TSEG1_SEG_MSK 0x0f00
#define CAN_TSEG2_SEG_MSK 0x7000
#define CAN_SJW_SEG_MSK   0x00c0
#define CAN_BRP_SEG_MSK   0x003f
#define CAN_BPRE_SEG_MSK   0x000f
#define CAN_BIT_VALUE(seg1,seg2,sjw) ((((seg1 - 1) << TSEG1)&CAN_TSEG1_SEG_MSK)\
                                                                         |(((seg2 - 1) << TSEG2)&CAN_TSEG2_SEG_MSK)\
                                                                         |(((sjw - 1) << SJW)&CAN_TSEG1_SEG_MSK))
/* CAN Error Counters */
#define CAN_TX_ERR_MSK 0x00ff
#define CAN_RX_ERR_MSK 0x7f00
#define CAN_RX_RP_MSK  0x8000

#define CAN_MAX_BIT_DIV   19
#define CAN_MIN_BIT_DIV   4
#define CAN_MAX_PRE_DIV   1024
#define CAN_MIN_PRE_DIV   1



/**************************************************************************************************************************/
/**************************************************************************************************************************/
/*                                                                                               CAN ADDRESSES                                                                                                      */
/**************************************************************************************************************************/
/**************************************************************************************************************************/
#define CAN0_BASE  (unsigned long)(0x40040000)
#define CAN1_BASE  (unsigned long)(0x40041000)
#define CAN0 0
#define CAN1 1

// Base Registers
#define CANn_BASE(CAN_NUM) ((unsigned long)((0x40040000)|(CAN_NUM<<12)))
// offset Registers
#define CANCTL_CAN_CONTROL                              ((unsigned long)0x000)
#define CANSTS_CAN_STATUS                               ((unsigned long)0x004)
#define CANERR_CAN_ERROR_COUNTER                    ((unsigned long)0x008)
#define CANBIT_CAN_BIT_TIME                             ((unsigned long)0x00C)
#define CANINT_CAN_INTERRUPT                            ((unsigned long)0x010)
#define CANTST_CAN_TEST                                     ((unsigned long)0x014)
#define CANBRPE_CAN_BAUD_EXTENSION              ((unsigned long)0x018)
#define CANIF1CRQ_COMMAND_REQUEST_IF            ((unsigned long)0x020)
#define CANIF2CRQ_COMMAND_REQUEST_IF            ((unsigned long)0x080)
#define CANIF1CMSK_COMMAND_MASK_IF          ((unsigned long)0x024)
#define CANIF2CMSK_COMMAND_MASK_IF            ((unsigned long)0x084)
#define CANIF1MSK1_MASK_1_IF                            ((unsigned long)0x028)
#define CANIF2MSK1_MASK_1_IF                            ((unsigned long)0x088)
#define CANIF1MSK2_MASK_2_IF                            ((unsigned long)0x02C)
#define CANIF2MSK2_MASK_2_IF                            ((unsigned long)0x08C)
#define CANIF1ARB1_ARBITRATION_1_IF             ((unsigned long)0x030)
#define CANIF2ARB1_ARBITRATION_1_IF             ((unsigned long)0x090)
#define CANIF1ARB2_ARBITRATION_2_IF             ((unsigned long)0x034)
#define CANIF2ARB2_ARBITRATION_2_IF             ((unsigned long)0x094)
#define CANIF1MCTL_MESSAGE_CONTROL_IF         ((unsigned long)0x038)
#define CANIF2MCTL_MESSAGE_CONTROL_IF           ((unsigned long)0x098)
#define CANIF1DA1_DATA_A1_IF                            ((unsigned long)0x03c)
#define CANIF2DA1_DATA_A1_IF                      ((unsigned long)0x09C)
#define CANIF1DA2_DATA_A2_IF                            ((unsigned long)0x040)
#define CANIF2DA2_DATA_A2_IF                      ((unsigned long)0x0A0)
#define CANIF1DB1_DATA_B1_IF                            ((unsigned long)0x044)
#define CANIF2DB1_DATA_B1_IF                      ((unsigned long)0x0A4)
#define CANIF1DB2_DATA_B2_IF                            ((unsigned long)0x048)
#define CANIF2DB2_DATA_B2_IF                      ((unsigned long)0x0A8)
#define CANTXRQ1_TRANSMISSION_REQ_1       ((unsigned long)0x100)
#define CANTXRQ2_TRANSMISSION_REQ_2       ((unsigned long)0x104)
#define CANNWDA1_NEW_DATA_1                     ((unsigned long)0x120)
#define CANNWDA2_NEW_DATA_2_IF                ((unsigned long)0x124)
#define CANMSG1INT_INTERRUPT_PENDING_1    ((unsigned long)0x140)
#define CANMSG2INT_INTERRUPT_PENDING_2      ((unsigned long)0x144)

//Used Registers
#define RCGC0_CLOCK_MODULE                          (*((volatile unsigned long*)0x400FE100))
#define RCGC2_CLOCK_PORT                                (*((volatile unsigned long*)0x400FE108))
#define EN1_INTERRUPT                                           (*((volatile unsigned long*)0xE000E104))
#define PRI9_INTERRUPT                                      (*((volatile unsigned long*)0xE000E424))
#define PRI10_INTERRUPT                                     (*((volatile unsigned long*)0xE000E428))

#define CAN0_IRQ  39
#define CAN1_IRQ  40

#define REG(BASE,Register) (*(volatile unsigned long*)((BASE)+(Register)))





/**************************************************************************************************************************/
/**************************************************************************************************************************/
/*                                                                                               CAN Enumeration & STRUCTURES DATA TYPES                                                                */
/**************************************************************************************************************************/
/**************************************************************************************************************************/

typedef enum{TEST=7, CCE=6, DAR=5, EIE=3, SIE=2, IE=1, INIT=0}CANCTL_bits;
typedef enum{BOFF=7, EWARN=6, EPASS=5, RXOK=4, TXOK=3, LEC=0}CANSTS_bits;
typedef enum{RP=15, REC=8, TEC=0}CANERR_bits;
typedef enum{TSEG2=12, TSEG1=8, SJW=6, BRP=0}CANBIT_bits;
typedef enum{INTID=0}CANINT_bits;
typedef enum{RX=7, TX=5, LBACK=4, SILENT=3, BASIC=2}CANTST_bits;
typedef enum{BRPE=0}CANBRPE_bits;
typedef enum{BUSY=15, MNUM=0}CANIFnCRQ_bits;
typedef enum{WRNRD=7, MASK=6, ARB=5, CONTROL=4, CLRINTPND=3, TXRQST_NEWDAT_CMSK=2, DATAA=1, DATAB=0}CANIFnCMSK_bits;
typedef enum{MXTD=15, MDIR=14, MSK=0}CANIF2MSK2_bits;
typedef enum{MSGVAL=15, XTD=14, DIR=13, ID_ARB2=0}CANIFnARB2_bits;
typedef enum{NEWDAT_MCTL=15, MSGLST=14, INTPND_MCTL=13, UMASK=12, TXIE=11, RXIE=10, RMTEN=9, TXRQST_MCTL=8, EOB=7, DLC=0}CANIFnMCTL_bits;



typedef enum{NO_ERROR,STUFF_ERROR,FORMAT_ERROR,BIT_1_ERROR,BIT_0_ERROR,CRC_ERROR,NO_EVENT}ErrorTypesOfLEC;

typedef enum    {CAN_INT_STS_CAUSE,CAN_INT_STS_OBJECT}tCANIntStsReg;
/*
    Description:
This data type is used to identify the interrupt status register. This is used when calling the
CANIntStatus() function.
Enumerators:
CAN_INT_STS_CAUSE :Read the CAN interrupt status information and to return INTID_STATUS_INTERRUPT if status interrupt occurs or 1 to 32
if there is a MessObj occcurs interrupt.
CAN_INT_STS_OBJECT Read a message object’s interrupt status to return the status of all MessObj if any one make interrupt so it's fast to read
the status of all message objs.
*/
typedef enum{CAN_STS_CONTROL,CAN_STS_TXREQUEST,CAN_STS_NEWDAT,CAN_STS_MSGVAL}tCANStsReg;
/*
Description:
This data type is used to identify which of several status registers to read when calling the
CANStatusGet() function.

CAN_STS_CONTROL Read the full CAN controller status (means read the CANSTS reg to know the status of error LEC .... ).
CAN_STS_TXREQUEST Read the full 32-bit mask of message objects with a transmit request(to know if there is any MEss obj waiting the transmision)

CAN_STS_NEWDAT Read the full 32-bit mask of message objects with new data available(to know if there is any MEss obj received new data whiched is not
read by the application .
CAN_STS_MSGVAL Read the full 32-bit mask of message objects that are enabled(to know which msg object is valid).
*/
typedef enum{MSG_OBJ_TYPE_TX ,  //MSOBJ transmit data frame and not receive data frame
                         MSG_OBJ_TYPE_TX_REMOTE ,  //MsObj is RX but it transmit RE
                         MSG_OBJ_TYPE_RX ,//MsObj is receiving data frame only
                         MSG_OBJ_TYPE_RX_REMOTE_MANUALLY , //MSObj is TX but when a matched remote frame comes it treats it as a data frame and set the newdata bit
                         MSG_OBJ_TYPE_RX_REMOTE_AUTOMATICALLY}tMsgObjType; //MSObj is TX but when a matched remote frame comes it send the data in its message RAM automatically

typedef enum{IDENTIFIER_11_BITS,IDENTIFIER_29_BITS}tIdentifierType;
typedef enum{NO_INTERRUPT,INTERRUPT_EXIST}CAN_INTERRUPT_DATA_TYPE;
typedef enum{MSG_HAS_NEW_DATA,MSG_NO_NEW_DATA,MSG_LOST_DATA,MSG_RECIEVED_RF_AS_DATA_FRAME,MSG_NOT_RECIEVED_RF_AS_DATA_FRAME}CAN_FLAG_MSG_NEWDATA_TYPE;
typedef enum{TX_MSG,RX_MSG}MSG_DIR_DATATYPE;
typedef enum{NOT_END_OF_THE_FIFO,END_OF_THE_FIFO}tFIFO_FLAG;
typedef struct
{
uint_32 ui32PropPhase1Seg;
uint_32 ui32Phase2Seg;
uint_32 ui32SJW;
uint_32 ui32QuantumPrescaler;
}tCANBitClkParms;

typedef struct
{
tIdentifierType identifierType;
uint_32 ui32MsgID;
uint_32 ui32MsgIDMask;
uint_32 ui32Flags;
uint_32 ui32MsgLen;
uint_8 MessageObjNUM;
tMsgObjType MessType;
CAN_FLAG_MSG_NEWDATA_TYPE flagNewData;
uint_8 * ptrToStartOfData;
tFIFO_FLAG FifoFlag;
}tCANMsgObject ;

extern tCANBitClkParms tCANBitClkParms_config;
extern volatile tCANMsgObject messObjConf[NO_STRUCTURE_OF_MESS_OBJ];
extern volatile CAN_INTERRUPT_DATA_TYPE gflag_CAN_interrupt;
extern volatile uint_8 messageCausedInterrupt;
/**************************************************************************************************************************/
/**************************************************************************************************************************/
/*                                                                                               CAN fUNCTIONS PROTOTYPES                                                                                           */
/**************************************************************************************************************************/
/**************************************************************************************************************************/


/*
CANBitTimingGet
input : ui8CanNo (CAN0,CAN1)
                psClkParms(ptr to structure of type tCANBitClkParms)
decription: return the values of PROG_SEG , PHAS1, PHAS2 and SJW which is stored in CANBIT reg and CANBRPE
*/
void CANBitTimingGet(uint_8 ui8CanNo,tCANBitClkParms *psClkParms);
/*
CANBitTimingSet
input : ui8CanNo (CAN0,CAN1)
decription : this fn sets the parameters for BIT TIMING from tCANBitClkParms_config in CAN_CONF.c
note: the fn convert the CAN mode from normal mode to initilization mode so the CAN will exit from the CAN BUS
*/
void CANBitTimingSet(uint_8 ui8CanNo);
/*
CANDisable
input : ui8CanNo (CAN0,CAN1)
Description:
Disables the CAN controller for message processing. When disabled, the controller no longer
automatically processes data on the CAN bus. The controller can be restarted by calling CANEnable().
*/
void CANDisable(uint_8 ui8CanNo);
/*
CANEnable
input : ui8CanNo (CAN0,CAN1)
Description:
Enables the CAN controller for message processing. Once enabled, the controller automatically
transmits any pending frames, and processes any received frames. The controller can
be stopped by calling CANDisable(). Prior to calling CANEnable(), CANInit() must have been
called to initialize
*/
void CANEnable(uint_8 ui8CanNo);
/*
CANErrCntrGet
input:ui8CanNo (CAN0,CAN1)
pui32RxCount is a pointer to storage for the receive error counter.
pui32TxCount is a pointer to storage for the transmit error counter.
Description:
This function reads the error counter register and returns the transmit and receive error counts
to the caller along with a flag indicating if the controller receive counter has reached the error
passive limit. The values of the receive and transmit error counters are returned through the
pointers provided as parameters.
Returns:
Returns true if the receive error count
*/
uint_8 CANErrCntrGet(uint_8 ui8CanNo,uint_32 *pui32RxCount,uint_32 *pui32TxCount);
/*
CANInit
input:ui8CanNo (CAN0,CAN1)
Description:
(1)intitialize the CAN clock and PINS
(2)After reset, the CAN controller is left in the disabled state. However, the memory used for
message objects contains undefined values and must be cleared prior to enabling the CAN
controller the first time. This prevents unwanted transmission or reception of data before the
message objects are configured. This function must be called before enabling the controller
the first time.
(3)set the required parameters for the interrupt (priority and the general enable interrupt),but the interrupt is still disabled
Returns:
None.
*/
void CANInit(uint_8 ui8CanNo) ;
/*
Clears a CAN interrupt source.
input:ui8CanNo (CAN0,CAN1)
ui32IntClr is a value indicating which interrupt source to clear.
Description:
This function can be used to clear a specific interrupt source. The ui32IntClr parameter must
be one of the following values:
CAN_INT_INTID_STATUS - Clears a status interrupt.
1-32 - Clears the specified message object interrupt
return none
*/
void CANIntClear(uint_8 ui8CanNo,uint_32 ui32IntClr);
/*
enable the interrupt of the CAN module
Parameters:
input:ui8CanNo (CAN0,CAN1)
ui32IntFlags is the bit mask of the interrupt sources to be enabled.
Description:
This function enables specific interrupt sources of the CAN controller. Only enabled sources
cause a processor interrupt.
The ui32IntFlags parameter is the logical OR of any of the following:
CAN_INT_ERROR - a controller error condition has occurred
CAN_INT_STATUS - a message transfer has completed, or a bus error has been detected
CAN_INT_MASTER - allow CAN controller to generate interrupts
*/
void CANIntEnable(uint_8 ui8CanNo,uint_32 ui32IntFlags);
/*
disable the interrupt of the CAN module
same parameters as CANIntClear
*/
void CANIntDisable(uint_8 ui8CanNo,uint_32 ui32IntFlags);
/*
    input:
    pfnHandler:ptr to the required fn wanted to run in CAN handler
    decription:set the call back function
*/
void CANIntRegister(void (*pfnHandler)(void));
/*
input:ui8CanNo (CAN0,CAN1)

decription:clear the call back function
and disable the CAN interrupt master(IE)
*/
void CANIntUnregister(uint_8 ui8CanNo);
/*
input:ui8CanNo (CAN0,CAN1)
eIntStsReg indicates which interrupt status register to read
Description:
This function returns the value of one of two interrupt status registers. The interrupt status
register read is determined by the eIntStsReg parameter, which can have one of the following
values:
CAN_INT_STS_CAUSE - indicates the cause of the interrupt
CAN_INT_STS_OBJECT - indicates pending interrupts of all message objects
CAN_INT_STS_CAUSE :Read the CAN interrupt status information and to return INTID_STATUS_INTERRUPT if status interrupt occurs or 1 to 32
if there is a MessObj occcurs interrupt.
CAN_INT_STS_OBJECT Read a message object’s interrupt status to return the status of all MessObj if any one make interrupt so it's fast to read
the status of all message objs.
*/
uint_32 CANGetIntStatus(uint_8 ui8CanNo,tCANIntStsReg eIntStsReg);
/*
input:ui8CanNo (CAN0,CAN1)
ui32ObjID is the message object number to disable (1-32).
Description:
This function frees the specified message object from use. Once a message object has been
“cleared, ” it no longer automatically sends or receives messages, nor does it generate interrupts.
Returns:
Non
*/
void CANMessageClear(uint_8 ui8CanNo,uint_32 ui32ObjID);
/*
input MessageObjNUM(no of required Msobj)
ptr : ptr to the required array which carries the data of this message
*/
returnOfFnStatus CAN_setPtrOfMessObj(uint_8 MessageObjNUM,uint_8 * ptr);


/*
input:ui8CanNo (CAN0,CAN1)
description:this function is used to set the fn of all MSJobj
return : none
*/
void CANMessageSet(uint_8 ui8CanNo);
/*
input:mess number
ui8CanNo (CAN0,CAN1)
decription:this fn is used to transmit remote frame when the message's type is MSG_OBJ_TYPE_TX_REMOTE
*/
void CAN_transmitRemoteFrame(unsigned char messageObjectNo,unsigned char canNo);
/*
decription : is used to fill the data of the message object
*/
returnOfFnStatus CAN_putDataInMessObj(unsigned char messageObjectNo,unsigned char canNo);

/*
decription : it tranmsits the data located in the mess obj only if the mode of the mess obj is MSG_OBJ_TYPE_TX or MSG_OBJ_TYPE_RX_REMOTE_MANUALLY
*/
returnOfFnStatus CAN_transmitDATA(uint_8 messageObjectNo,uint_8 canNo);
/*
desription:if there is new data received this fn puts the data in its location specified by structure and set the flag flagNewData to MSG_HAS_NEW_DATA
*/
returnOfFnStatus CAN_receiveDATA(unsigned char messageObjectNo,unsigned char canNo);
returnOfFnStatus CAN_receiveRemoteFrameAsDataFrame(unsigned char messageObjectNo,unsigned char canNo);
returnOfFnStatus CAN_receiveDATAInterruptOrCheckInPolling(unsigned char * messageObjectNo_ptr,unsigned char canNo,ErrorTypesOfLEC *errorType);
void CAN0_ISR(void);
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void CAN_RECIEVE_FROM_ISR (uint_8 mssgobj_no,uint_8 canBase);
#endif
