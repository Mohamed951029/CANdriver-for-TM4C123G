#include "can.h"
#include "can_conf.h"
/*typedef struct
{
uint32_t ui32SyncPropPhase1Seg;
uint32_t ui32Phase2Seg;
uint32_t ui32SJW;
uint32_t ui32QuantumPrescaler;
}
tCANBitClkParms;

typedef struct
{
tIdentifierType identifierType;
uint32_t ui32MsgID;
uint32_t ui32MsgIDMask;
uint32_t ui32Flags;       * Set MSG_OBJ_TX_INT_ENABLE flag to enable interrupt on transmission.
                                                    • Set MSG_OBJ_RX_INT_ENABLE flag to enable interrupt on receipt.
                                                    • Set MSG_OBJ_USE_ID_FILTER flag to enable filtering based on the identifier mask
uint32_t ui32MsgLen;
uint_8 MessageObjNUM;
tMsgObjType MessType;
CAN_FLAG_MSG_NEWDATA_TYPE flagNewData;
uint_8 * ptrToStartOfData;
uint_8 NoMsgObjInTheFifo;
}tCANMsgObject ;
*/

tCANBitClkParms tCANBitClkParms_config = {3,1,1,16};
//MSG_OBJ_TYPE_TX_REMOTE
volatile tCANMsgObject messObjConf[NO_STRUCTURE_OF_MESS_OBJ]={
{IDENTIFIER_11_BITS,0x11,0x7FF,MSG_OBJ_RX_INT_ENABLE|MSG_OBJ_USE_ID_FILTER,8,1,MSG_OBJ_TYPE_RX,MSG_NO_NEW_DATA,NULL,END_OF_THE_FIFO},
{IDENTIFIER_11_BITS,0x12,0x7FF,MSG_OBJ_RX_INT_ENABLE|MSG_OBJ_USE_ID_FILTER,8,2,MSG_OBJ_TYPE_RX,MSG_NO_NEW_DATA,NULL,END_OF_THE_FIFO},
{IDENTIFIER_11_BITS,0x14,0x7FF,MSG_OBJ_RX_INT_ENABLE|MSG_OBJ_USE_ID_FILTER,8,4,MSG_OBJ_TYPE_RX,MSG_NO_NEW_DATA,NULL,END_OF_THE_FIFO},
{IDENTIFIER_11_BITS,0x18,0x7FF,MSG_OBJ_RX_INT_ENABLE|MSG_OBJ_USE_ID_FILTER,8,15,MSG_OBJ_TYPE_RX,MSG_NO_NEW_DATA,NULL,END_OF_THE_FIFO}
};
