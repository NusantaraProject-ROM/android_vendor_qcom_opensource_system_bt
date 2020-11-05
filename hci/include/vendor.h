/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bt_types.h"
#include "bt_vendor_lib.h"
#include "hci_internals.h"
#include "hci_layer.h"

#define HCI_BT_SOC_CRASHED_OGF    0xfc
#define HCI_BT_SOC_CRASHED_OCF    0x00

#define HCI_CRASH_MESSAGE_SIZE    (60)

typedef enum {
  VENDOR_CHIP_POWER_CONTROL = BT_VND_OP_POWER_CTRL,
  VENDOR_OPEN_USERIAL = BT_VND_OP_USERIAL_OPEN,
  VENDOR_CLOSE_USERIAL = BT_VND_OP_USERIAL_CLOSE,
  VENDOR_GET_LPM_IDLE_TIMEOUT = BT_VND_OP_GET_LPM_IDLE_TIMEOUT,
  VENDOR_SET_LPM_WAKE_STATE = BT_VND_OP_LPM_WAKE_SET_STATE,
  VENDOR_SET_AUDIO_STATE = BT_VND_OP_SET_AUDIO_STATE
} vendor_opcode_t;

typedef enum {
  VENDOR_CONFIGURE_FIRMWARE = BT_VND_OP_FW_CFG,
  VENDOR_CONFIGURE_SCO = BT_VND_OP_SCO_CFG,
  VENDOR_SET_LPM_MODE = BT_VND_OP_LPM_SET_MODE,
  VENDOR_DO_EPILOG = BT_VND_OP_EPILOG,
  VENDOR_A2DP_OFFLOAD_START = BT_VND_OP_A2DP_OFFLOAD_START,
  VENDOR_A2DP_OFFLOAD_STOP = BT_VND_OP_A2DP_OFFLOAD_STOP,
  VENDOR_LAST_OP
} vendor_async_opcode_t;

typedef void (*vendor_cb)(bool success);

typedef struct vendor_t {
  // Opens the vendor-specific library and sets the Bluetooth
  // address of the adapter to |local_bdaddr|. |hci_interface| is
  // used to send commands on behalf of the vendor library.
  bool (*open)(const uint8_t* local_bdaddr, const hci_t* hci_interface);

  // Closes the vendor-specific library and frees all associated resources.
  // Only |vendor_open| may be called after |vendor_close|.
  void (*close)(void);

  // Sends a vendor-specific command to the library.
  int (*send_command)(vendor_opcode_t opcode, void* param);

  // Sends an asynchronous vendor-specific command to the library.
  int (*send_async_command)(vendor_async_opcode_t opcode, void* param);

  // Registers a callback for an asynchronous vendor-specific command.
  void (*set_callback)(vendor_async_opcode_t opcode, vendor_cb callback);
} vendor_t;

const vendor_t* vendor_get_interface();

typedef enum {
  BT_SOC_REASON_DEFAULT        =  0x00,

  // SoC Crash Reasons
  BT_SOC_REASON_RX_NULL        =  0x01,
  BT_SOC_REASON_TX_RX_INVALID_PKT = 0x40,
  BT_SOC_REASON_TX_RX_INVALID_LEN = 0x41,
  BT_SOC_REASON_TX_RX_INVALID_PKT_FATAL = 0xC0,
  BT_SOC_REASON_TX_RX_INVALID_LEN_FATAL = 0xC1,
  BT_SOC_REASON_UNKNOWN        =  0x81,
  BT_SOC_REASON_SW_REQUESTED   =  0x82,
  BT_SOC_REASON_STACK_OVERFLOW =  0x83,
  BT_SOC_REASON_EXCEPTION      =  0x84,
  BT_SOC_REASON_ASSERT         =  0x85,
  BT_SOC_REASON_TRAP           =  0x86,
  BT_SOC_REASON_OS_FATAL       =  0x87,
  BT_SOC_REASON_HCI_RESET      =  0x88,
  BT_SOC_REASON_PATCH_RESET    =  0x89,
  BT_SOC_REASON_ABT            =  0x8A,
  BT_SOC_REASON_RAMMASK        =  0x8B,
  BT_SOC_REASON_PREBARK        =  0x8C,
  BT_SOC_REASON_BUSERROR       =  0x8D,
  BT_SOC_REASON_IO_FATAL       =  0x8E,
  BT_SOC_REASON_SSR_CMD        =  0x8F,
  BT_SOC_REASON_POWERON        =  0x90,
  BT_SOC_REASON_WATCHDOG       =  0x91,
  BT_SOC_REASON_RAMMASK_RGN1   =  0x92,
  BT_SOC_REASON_RAMMASK_RGN0   =  0x93,
  BT_SOC_REASON_Q6_WATCHDOG    =  0x94,
  BT_SOC_REASON_ZEALIS_RAM_MASK_RGN0 =  0x95,
  BT_SOC_REASON_ZEALIS_RAM_MASK_RGN1 =  0x96,
  BT_SOC_REASON_APSS_RESET           =  0x97,
  BT_SOC_REASON_INVALID_STACK  =  0xF0,

  // Transport Driver Crash Reasons
  BT_HOST_REASON_UARTINIT_STUCK        =  0xB1,
  BT_HOST_REASON_GETVER_SEND_STUCK     =  0xB2,
  BT_HOST_REASON_GETVER_NO_RSP_RCVD    =  0xB3,
  BT_HOST_REASON_SETBAUDRATE_CMD_STUCK =  0xB4,
  BT_HOST_REASON_PATCH_DNLD_STUCK      =  0xB5,
  BT_HOST_REASON_GETBOARDID_CMD_STUCK  =  0xB6,
  BT_HOST_REASON_NVM_DNLD_STUCK        =  0xB7,
  BT_HOST_REASON_HCI_RESET_STUCK       =  0xB8,
  BT_HOST_REASON_GETBLDINFO_CMD_STUCK  =  0xB9,
  BT_HOST_REASON_ADDONFEAT_CMD_STUCK   =  0xBA,
  BT_HOST_REASON_ENHLOG_CMD_STUCK      =  0xBB,
  BT_HOST_REASON_DIAGINIT_STUCK        =  0xBC,
  BT_HOST_REASON_DIAGDEINIT_STUCK      =  0xBD,
  BT_HOST_REASON_XMEM_NVM_DNLD_STUCK   =  0xBE,
  BT_HOST_REASON_XMEM_PATCH_DNLD_STUCK =  0xBF,
  BT_HOST_REASON_SECURE_BRIDGE_CMD_STUCK = 0xC2,
  BT_HOST_REASON_FAILED_TO_SEND_CMD              =  0xC3,
  BT_HOST_REASON_HCI_RESET_CC_NOT_RCVD           =  0xC4,
  BT_HOST_REASON_HCI_PRE_SHUTDOWN_CC_NOT_RCVD    =  0xC5,
  BT_HOST_REASON_HCI_SET_BD_ADDRESS_CC_NOT_RCVD  =  0xC6,
  BT_HOST_REASON_FAILED_TO_RECEIVE_SLEEP_IND     =  0xC7,
  BT_HOST_REASON_POWER_ON_REGS_STUCK             =  0xC8,
  BT_HOST_REASON_RX_THREAD_START_STUCK           =  0xC9,
  BT_HOST_REASON_GET_LOCALADDR_STUCK             =  0xCA,
} soc_crash_reason_e;

typedef struct {
  soc_crash_reason_e reason;
  char reasonstr[HCI_CRASH_MESSAGE_SIZE];
} secondary_reason;

static secondary_reason secondary_crash_reason [] = {
{ BT_SOC_REASON_DEFAULT                  ,    "Default"},
{ BT_SOC_REASON_RX_NULL                  ,    "Rx Null"},
{ BT_SOC_REASON_UNKNOWN                  ,    "Unknown"},
{ BT_SOC_REASON_TX_RX_INVALID_PKT        ,    "Tx/Rx invalid packet"},
{ BT_SOC_REASON_TX_RX_INVALID_LEN        ,    "Tx/Rx invalid len"},
{ BT_SOC_REASON_TX_RX_INVALID_PKT_FATAL  ,    "Tx/Rx invalid packet fatal error"},
{ BT_SOC_REASON_TX_RX_INVALID_LEN_FATAL  ,    "Tx/Rx invalid lenght fatal error"},
{ BT_SOC_REASON_SW_REQUESTED             ,    "SW Requested"},
{ BT_SOC_REASON_STACK_OVERFLOW           ,    "Stack Overflow"},
{ BT_SOC_REASON_EXCEPTION                ,    "Exception"},
{ BT_SOC_REASON_ASSERT                   ,    "Assert"},
{ BT_SOC_REASON_TRAP                     ,    "Trap"},
{ BT_SOC_REASON_OS_FATAL                 ,    "OS Fatal"},
{ BT_SOC_REASON_HCI_RESET                ,    "HCI Reset"},
{ BT_SOC_REASON_PATCH_RESET              ,    "Patch Reset"},
{ BT_SOC_REASON_ABT                      ,    "SoC Abort"},
{ BT_SOC_REASON_RAMMASK                  ,    "RAM MASK"},
{ BT_SOC_REASON_PREBARK                  ,    "PREBARK"},
{ BT_SOC_REASON_BUSERROR                 ,    "Bus error"},
{ BT_SOC_REASON_IO_FATAL                 ,    "IO fatal eror"},
{ BT_SOC_REASON_SSR_CMD                  ,    "SSR CMD"},
{ BT_SOC_REASON_POWERON                  ,    "Power ON"},
{ BT_SOC_REASON_WATCHDOG                 ,    "Watchdog"},
{ BT_SOC_REASON_RAMMASK_RGN1             ,    "RAMMASK RGN1"},
{ BT_SOC_REASON_RAMMASK_RGN0             ,    "RAMMASK RGN0"},
{ BT_SOC_REASON_Q6_WATCHDOG              ,    "Q6 Watchdog"},
{ BT_SOC_REASON_ZEALIS_RAM_MASK_RGN0     ,    "ZEALIS RAM MASK RGN0"},
{ BT_SOC_REASON_ZEALIS_RAM_MASK_RGN1     ,    "ZEALIS RAM MASK RGN1"},
{ BT_SOC_REASON_APSS_RESET               ,    "APSS reset"},
{ BT_SOC_REASON_INVALID_STACK            ,    "Invalid Stack"},
{ BT_HOST_REASON_UARTINIT_STUCK           ,    "UartInitStuck"},
{ BT_HOST_REASON_GETVER_SEND_STUCK        ,    "GetVerSendStuck"},
{ BT_HOST_REASON_GETVER_NO_RSP_RCVD       ,    "GetVerNoRspRcvd"},
{ BT_HOST_REASON_SETBAUDRATE_CMD_STUCK    ,    "SetBaudRateStuck"},
{ BT_HOST_REASON_PATCH_DNLD_STUCK         ,    "PatchDnldStuck"},
{ BT_HOST_REASON_GETBOARDID_CMD_STUCK     ,    "GetBoardIdStuck"},
{ BT_HOST_REASON_NVM_DNLD_STUCK           ,    "NvmDnldStuck"},
{ BT_HOST_REASON_HCI_RESET_STUCK          ,    "HciResetStuck"},
{ BT_HOST_REASON_GETBLDINFO_CMD_STUCK     ,    "GetBldInfoCmdStuck"},
{ BT_HOST_REASON_ADDONFEAT_CMD_STUCK      ,    "AddOnFeatCmdStuck"},
{ BT_HOST_REASON_ENHLOG_CMD_STUCK         ,    "EnhLogCmdStuck"},
{ BT_HOST_REASON_DIAGINIT_STUCK           ,    "DiagInitStuck"},
{ BT_HOST_REASON_DIAGDEINIT_STUCK         ,    "DiagDeinitStuck"},
{ BT_HOST_REASON_XMEM_NVM_DNLD_STUCK      ,    "XMEM NVM Download stuck"},
{ BT_HOST_REASON_XMEM_PATCH_DNLD_STUCK    ,    "XMEM patch download stuck"},
{ BT_HOST_REASON_SECURE_BRIDGE_CMD_STUCK  ,    "Secure bridge cmd stuck"},
{ BT_HOST_REASON_FAILED_TO_SEND_CMD            , "Failed to send internal cmd"},
{ BT_HOST_REASON_HCI_RESET_CC_NOT_RCVD         , "HCI Reset Cmd CC NotRcvd"},
{ BT_HOST_REASON_HCI_PRE_SHUTDOWN_CC_NOT_RCVD  , "HCI Pre shutdown Cmd CC not Rcvd"},
{ BT_HOST_REASON_HCI_SET_BD_ADDRESS_CC_NOT_RCVD, "HCI BD address CC not Rcvd"},
{ BT_HOST_REASON_FAILED_TO_RECEIVE_SLEEP_IND   , "Failed to receive SLEEP IND from SoC"},
{ BT_HOST_REASON_POWER_ON_REGS_STUCK           , "SoC Power ON Sequence stuck"},
{ BT_HOST_REASON_RX_THREAD_START_STUCK         , "RX thread start stuck"},
{ BT_HOST_REASON_GET_LOCALADDR_STUCK           , "Get local BD address stuck"}
};

enum host_crash_reason_e  {
  BT_HOST_REASON_DEFAULT_NONE  = 0x00,                         //INVALID REASON
  BT_HOST_REASON_SOC_CRASHED = 0x01,                           //SOC WAS CRASHED
  BT_HOST_REASON_SOC_CRASHED_DIAG_SSR = 0x02,                  //SOC CRASHED DIAG INITIATED SSR
  BT_HOST_REASON_INIT_FAILED = 0x03,                           //HOST INITIALIZATION FAILED
  BT_HOST_REASON_CLOSE_RCVD_DURING_INIT = 0x04,                //CLOSE RECEIVED FROM STACK DURING SOC INIT
  BT_HOST_REASON_ERROR_READING_DATA_FROM_UART = 0x05,          //ERROR READING DATA FROM UART
  BT_HOST_REASON_WRITE_FAIL_SPCL_BUFF_CRASH_SOC = 0x06,        //FAILED TO WRITE SPECIAL BYTES TO CRASH SOC
  BT_HOST_REASON_RX_THREAD_STUCK = 0x07,                       //RX THREAD STUCK
  BT_HOST_REASON_SSR_CMD_TIMEDOUT = 0x10,                      //SSR DUE TO CMD TIMED OUT
  BT_HOST_REASON_SSR_SPURIOUS_WAKEUP = 0x11,                   //SSR DUE TO SPURIOUS WAKE UP
  BT_HOST_REASON_SSR_INVALID_BYTES_RCVD = 0x12,                //INVALID HCI CMD TYPE RECEIVED
  BT_HOST_REASON_SSR_RCVD_LARGE_PKT_FROM_SOC = 0x13,           //SSR DUE TO LARGE PKT RECVIVED FROM SOC
  BT_HOST_REASON_SSR_UNABLE_TO_WAKEUP_SOC = 0x14,              //UNABLE TO WAKE UP SOC
  BT_HOST_REASON_CMD_TIMEDOUT_SOC_WAIT_TIMEOUT = 0x20,         //COMMAND TIMEOUT AND SOC CRASH WAIT TIMEOUT
  BT_HOST_REASON_SPURIOUS_WAKEUP_SOC_WAIT_TIMEOUT = 0x21,      //SPURIOUS WAKE AND SOC CRASH WAIT TIMEOUT
  BT_HOST_REASON_INV_BYTES_SOC_WAIT_TIMEOUT = 0x22,            //INVALID BYTES AND SOC CRASH WAIT TIMEOUT
  BT_HOST_REASON_SOC_WAKEUP_FAILED_SOC_WAIT_TIMEOUT = 0x23,    //SOC WAKEUP FAILURE AND SOC CRASH WAIT TIMEOUT
  BT_HOST_REASON_SOC_CRASHED_DIAG_SSR_SOC_WAIT_TIMEOUT = 0x24, //SOC CRASHED DIAG INITIATED SSR CRASH WAIT TIMEOUT
  BT_HOST_REASON_NONE_SOC_WAIT_TIMEOUT = 0x25,                 //INVALID FAILURE AND SOC CRASH WAIT TIMEOUT
  BT_HOST_REASON_SOC_DEINIT_STUCK = 0x26,                      //SOC DEINIT STUCK
  BT_HOST_REASON_SSR_INTERNAL_CMD_TIMEDOUT = 0x27,             //SSR DUE TO CMD INTERNAL TIMED OUT
  BT_HOST_REASON_FAILED_TO_SEND_INTERNAL_CMD = 0x28,           //FAILED TO SEND INTERNAL CMD
  BT_HOST_REASON_SSR_SLEEP_IND_NOT_RCVD = 0x29,                //SOC DID NOT RCVD SLEEP IND DURING CLOSE
};

typedef struct {
  host_crash_reason_e reason;
  char reasonstr[HCI_CRASH_MESSAGE_SIZE];
} primary_reason;

static primary_reason primary_crash_reason [] = {
{ BT_HOST_REASON_DEFAULT_NONE                         , "Invalid reason"},
{ BT_HOST_REASON_SOC_CRASHED                          , "SOC crashed"},
{ BT_HOST_REASON_SOC_CRASHED_DIAG_SSR                 , "SOC crashed with diag initiated SSR"},
{ BT_HOST_REASON_INIT_FAILED                          , "Init failed"},
{ BT_HOST_REASON_CLOSE_RCVD_DURING_INIT               , "Close received from stack during SOC init"},
{ BT_HOST_REASON_ERROR_READING_DATA_FROM_UART         , "Error reading data from UART"},
{ BT_HOST_REASON_WRITE_FAIL_SPCL_BUFF_CRASH_SOC       , "Failed to write special bytes to crash SOC"},
{ BT_HOST_REASON_RX_THREAD_STUCK                      , "Rx Thread Stuck"},
{ BT_HOST_REASON_SSR_CMD_TIMEDOUT                     , "SSR due to command timed out"},
{ BT_HOST_REASON_SSR_SPURIOUS_WAKEUP                  , "SSR due to spurious wakeup"},
{ BT_HOST_REASON_SSR_INVALID_BYTES_RCVD               , "Invalid HCI cmd type received"},
{ BT_HOST_REASON_SSR_RCVD_LARGE_PKT_FROM_SOC          , "Large packet received from SOC"},
{ BT_HOST_REASON_SSR_UNABLE_TO_WAKEUP_SOC             , "Unable to wake SOC"},
{ BT_HOST_REASON_CMD_TIMEDOUT_SOC_WAIT_TIMEOUT        , "Command timedout and SOC crash wait timeout"},
{ BT_HOST_REASON_SPURIOUS_WAKEUP_SOC_WAIT_TIMEOUT     , "Spurious wake and SOC crash wait timeout"},
{ BT_HOST_REASON_INV_BYTES_SOC_WAIT_TIMEOUT           , "Invalid bytes received and SOC crash wait timeout"},
{ BT_HOST_REASON_SOC_WAKEUP_FAILED_SOC_WAIT_TIMEOUT   , "SOC Wakeup failed and SOC crash wait timeout"},
{ BT_HOST_REASON_NONE_SOC_WAIT_TIMEOUT                , "Invalid Reason and SOC crash wait timeout"},
{ BT_HOST_REASON_SOC_CRASHED_DIAG_SSR_SOC_WAIT_TIMEOUT, "SOC crashed with diag initiated SSR and SOC wait timeout"},
{ BT_HOST_REASON_NONE_SOC_WAIT_TIMEOUT                , "Invalid Reason and SOC crash wait timeout"},
{ BT_HOST_REASON_SOC_DEINIT_STUCK                     , "SOC DeInit Stuck"},
{ BT_HOST_REASON_SSR_INTERNAL_CMD_TIMEDOUT            , "SSR due to internal Command timeout"},
{ BT_HOST_REASON_FAILED_TO_SEND_INTERNAL_CMD          , "Failed to send internal command"},
{ BT_HOST_REASON_SSR_SLEEP_IND_NOT_RCVD               , "Failed to receive SLEEP IND during close"},
};

void decode_crash_reason(uint8_t* p, uint8_t evt_len);
char *get_secondary_reason_str(soc_crash_reason_e reason);
char *get_primary_reason_str(host_crash_reason_e reason);

