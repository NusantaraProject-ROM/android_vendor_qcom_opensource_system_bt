/******************************************************************************
 * Copyright (C) 2017, The Linux Foundation. All rights reserved.
 *
 *  Not a Contribution
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted (subject to the limitations in the
 *  disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

 *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 *  GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 *  HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 *  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/******************************************************************************
 *
 *  Copyright (C) 2017 Google, Inc.
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

#define LOG_TAG "btif_a2dp_audio_interface"

//#include "audio_a2dp_hw.h"
#include "btif_a2dp_audio_interface.h"
#include "bt_common.h"
#include "btif_a2dp.h"
#include "btif_a2dp_control.h"
#include "btif_a2dp_sink.h"
#include "btif_a2dp_source.h"
#include "btif_av.h"
#include "btif_av_co.h"
#include "btif_hf.h"
#include "a2dp_sbc.h"
#include <pthread.h>
#include "osi/include/osi.h"
#include <base/logging.h>
#include <utils/RefBase.h>
#include <com/qualcomm/qti/bluetooth_audio/1.0/IBluetoothAudio.h>
#include <com/qualcomm/qti/bluetooth_audio/1.0/IBluetoothAudioCallbacks.h>
#include <com/qualcomm/qti/bluetooth_audio/1.0/types.h>
#include <hwbinder/ProcessState.h>
#include <a2dp_vendor_ldac_constants.h>
#include <a2dp_vendor.h>
#include "bta/av/bta_av_int.h"
#include "btif_bat.h"

using com::qualcomm::qti::bluetooth_audio::V1_0::IBluetoothAudio;
using com::qualcomm::qti::bluetooth_audio::V1_0::IBluetoothAudioCallbacks;
using com::qualcomm::qti::bluetooth_audio::V1_0::Status;
using com::qualcomm::qti::bluetooth_audio::V1_0::CodecCfg;
using android::hardware::ProcessState;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::hardware::hidl_vec;
using ::android::sp;
using ::android::hardware::hidl_death_recipient;
using ::android::wp;
android::sp<IBluetoothAudio> btAudio;

#define CASE_RETURN_STR(const) \
  case const:                  \
    return #const;

uint8_t codec_info[30];
uint8_t len,a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
uint8_t a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
uint8_t a2dp_local_cmd_pending = A2DP_CTRL_CMD_NONE;
static char a2dp_hal_imp[PROPERTY_VALUE_MAX] = "false";

Status mapToStatus(uint8_t resp);
uint8_t btif_a2dp_audio_process_request(uint8_t cmd);
uint8_t btif_a2dp_audio_snd_ctrl_cmd(uint8_t cmd);
volatile bool server_died = false;
static pthread_t audio_hal_monitor;
typedef std::unique_lock<std::mutex> Lock;
std::mutex mtx;
std::mutex mtxBtAudio;
std::condition_variable mCV;
/*BTIF AV helper */
extern bool btif_av_is_device_disconnecting();
extern int btif_get_is_remote_started_idx();
extern bool btif_av_is_playing_on_other_idx(int current_index);
extern int btif_get_is_remote_started_idx();
extern bool reconfig_a2dp;
extern bool audio_start_awaited;
bool deinit_pending = false;
static void btif_a2dp_audio_send_start_req();
static void btif_a2dp_audio_send_suspend_req();
static void btif_a2dp_audio_send_stop_req();
static void btif_a2dp_audio_send_a2dp_ready_status();
static void btif_a2dp_audio_send_codec_config();
static void btif_a2dp_audio_send_mcast_status();
static void btif_a2dp_audio_send_num_connected_devices();
static void btif_a2dp_audio_send_connection_status();
static void btif_a2dp_audio_send_sink_latency();
extern int btif_max_av_clients;
extern bool enc_update_in_progress;
extern tBTA_AV_HNDL btif_av_get_av_hdl_from_idx(int idx);
extern void btif_av_reset_reconfig_flag();
extern tBTIF_A2DP_SOURCE_VSC btif_a2dp_src_vsc;
//extern void bta_av_vendor_offload_stop(void);

#if 0
typedef enum {
  A2DP_CTRL_GET_CODEC_CONFIG = 15,
  A2DP_CTRL_GET_MULTICAST_STATUS,
  A2DP_CTRL_GET_CONNECTION_STATUS,
  A2DP_CTRL_GET_NUM_CONNECTED_DEVICE,
}tA2DP_CTRL_EXT_CMD;
typedef enum {
  A2DP_CTRL_ACK_PENDING = 5,
}tA2DP_CTRL_ACK_EXT;
#endif
#ifdef BTA_AV_SPLIT_A2DP_DEF_FREQ_48KHZ
#define A2DP_SBC_DEFAULT_BITRATE 345

#ifndef A2DP_SBC_NON_EDR_MAX_RATE
#define A2DP_SBC_NON_EDR_MAX_RATE 237
#endif
#else
#define A2DP_SBC_DEFAULT_BITRATE 328

#ifndef A2DP_SBC_NON_EDR_MAX_RATE
#define A2DP_SBC_NON_EDR_MAX_RATE 229
#endif
#endif

void on_hidl_server_died();
//using OnServerDead = std::function<void(void)>;
struct HidlDeathRecipient : public hidl_death_recipient {
  virtual void serviceDied(
      uint64_t /*cookie*/,
      const wp<::android::hidl::base::V1_0::IBase>& /*who*/) {
    LOG_INFO(LOG_TAG,"serviceDied");
    Lock lk(mtx);
    server_died = true;
    mCV.notify_one();
  }
};
sp<HidlDeathRecipient> BTAudioHidlDeathRecipient = new HidlDeathRecipient();

bool isBAEnabled()
{
    LOG_INFO(LOG_TAG,"%s:",__func__);
    if (btif_ba_get_state() > BTIF_BA_STATE_IDLE_AUDIO_NS)
        return true;
    else
        return false;
}

class BluetoothAudioCallbacks : public IBluetoothAudioCallbacks {
 public:
    Return<void> a2dp_start_stream_req() {
        LOG_INFO(LOG_TAG,"a2dp_start_stream_req");
        btif_a2dp_audio_send_start_req();
        return Void();
    }
    Return<void> a2dp_suspend_stream_req() {
        LOG_INFO(LOG_TAG,"a2dp_suspend_stream_req");
        btif_a2dp_audio_send_suspend_req();
        return Void();
    }
    Return<void> a2dp_stop_stream_req() {
        LOG_INFO(LOG_TAG,"a2dp_stop_stream_req");
        btif_a2dp_audio_send_stop_req();
        return Void();
    }
    Return<void> a2dp_check_ready() {
        LOG_INFO(LOG_TAG,"a2dp_check_ready");
        btif_a2dp_audio_send_a2dp_ready_status();
        return Void();
    }
    Return<void> a2dp_get_codec_config() {
        LOG_INFO(LOG_TAG,"a2dp_get_codec_config");
        btif_a2dp_audio_send_codec_config();
        return Void();
    }
    Return<void> a2dp_get_multicast_status() {
        LOG_INFO(LOG_TAG,"a2dp_get_multicast_status");
        btif_a2dp_audio_send_mcast_status();
        return Void();
    }
    Return<void> a2dp_get_num_connected_devices() {
        LOG_INFO(LOG_TAG,"a2dp_get_num_connected_devices");
        btif_a2dp_audio_send_num_connected_devices();
        return Void();
    }
    Return<void> a2dp_get_connection_status() {
        LOG_INFO(LOG_TAG,"a2dp_get_connection_status");
        btif_a2dp_audio_send_connection_status();
        return Void();
    }
    Return<void> a2dp_get_sink_latency() {
        LOG_INFO(LOG_TAG,"a2dp_get_sink_latency");
        btif_a2dp_audio_send_sink_latency();
        return Void();
    }
};

Status mapToStatus(uint8_t resp)
{
    switch(resp) {
      case A2DP_CTRL_ACK_SUCCESS:
        return Status::SUCCESS;
        break;
      case A2DP_CTRL_ACK_FAILURE:
        return Status::FAILURE;
        break;
      case A2DP_CTRL_ACK_INCALL_FAILURE:
        return Status::INCALL_FAILURE;
        break;
      case A2DP_CTRL_ACK_PENDING:
        return Status::PENDING;
        break;
      case A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS:
        return Status::DISCONNECTING;
      default:
        LOG_INFO(LOG_TAG,"Invalid Status");
        return Status::UNSUPPORTED;
        break;
    }
  return Status::SUCCESS;
}

/* Thread to handle hal sever death receipt*/
static void* server_thread(UNUSED_ATTR void* arg) {
  LOG_INFO(LOG_TAG,"%s",__func__);
  {
    Lock lk(mtx);
    while (server_died == false) {
      LOG_INFO(LOG_TAG,"waitin on condition");
      mCV.wait(lk);
    }
    server_died = false;
  }
  on_hidl_server_died();
  LOG_INFO(LOG_TAG,"%s EXIT",__func__);
  return NULL;
}

uint8_t btif_a2dp_audio_interface_get_pending_cmd() {
    LOG_INFO(LOG_TAG," pending_cmd = %d", a2dp_cmd_pending);
    return a2dp_cmd_pending;
}

void btif_a2dp_audio_interface_init() {
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_interface_init");
  Lock lock(mtxBtAudio);
  btAudio = IBluetoothAudio::getService();
  CHECK(btAudio != nullptr);
  LOG_INFO(LOG_TAG, "%s: IBluetoothAudio::getService() returned %p (%s)",
           __func__, btAudio.get(), (btAudio->isRemote() ? "remote" : "local"));
  {
    LOG_INFO(LOG_TAG,"%s:Calling Init",__func__);
    android::sp<IBluetoothAudioCallbacks> callbacks = new BluetoothAudioCallbacks();
    auto ret = btAudio->initialize_callbacks(callbacks);
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"hal server is dead ");
  }
  deinit_pending = false;
  server_died = false;
  int ret = pthread_create(&audio_hal_monitor, (const pthread_attr_t*)NULL, server_thread, nullptr);
  if (ret != 0) {
    LOG_ERROR(LOG_TAG,"pthread create falied");
  } else {
    pthread_detach(audio_hal_monitor);
  }
  auto hidl_death_link = btAudio->linkToDeath(BTAudioHidlDeathRecipient, 0);
  if (!hidl_death_link.isOk()) LOG_ERROR(LOG_TAG,"hidl_death_link server is dead");
  LOG_INFO(LOG_TAG,"%s:Init returned",__func__);
}

void btif_a2dp_audio_interface_deinit() {
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_interface_deinit");
  deinit_pending = true;
  {
    Lock lock(mtxBtAudio);
    if (btAudio != nullptr) {
      tBTA_AV_STATUS status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
      if (a2dp_cmd_pending == A2DP_CTRL_CMD_START) {
        LOG_INFO(LOG_TAG,"calling method a2dp_on_started");
        auto ret = btAudio->a2dp_on_started(mapToStatus(status));
        if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_started: server died");
      } else if ((a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND)
          || (a2dp_cmd_pending == A2DP_CTRL_CMD_STOP)) {
        LOG_INFO(LOG_TAG,"calling method a2dp_on_started");
        auto ret = btAudio->a2dp_on_suspended(mapToStatus(status));
        if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_suspended: server died");
      }
      a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
    }

    if (btAudio != nullptr) {
      auto ret = btAudio->deinitialize_callbacks();
      if (!ret.isOk()) {
        LOG_ERROR(LOG_TAG,"hal server is dead");
      }
      auto hidl_death_unlink = btAudio->unlinkToDeath(BTAudioHidlDeathRecipient);
      if (!hidl_death_unlink.isOk()) LOG_ERROR(LOG_TAG,"hidl_death_unlink server died");
    }
    deinit_pending = false;
    btAudio = nullptr;
  }
  Lock lk(mtx);
  server_died = true; //Exit thread
  mCV.notify_one();
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_interface_deinit:Exit");
}

void btif_a2dp_audio_on_started(tBTA_AV_STATUS status)
{
  uint8_t ack = status;

  LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : status = %d",status);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr){
    if (property_get("persist.vendor.bt.a2dp.hal.implementation", a2dp_hal_imp, "false") &&
            !strcmp(a2dp_hal_imp, "true")) {
      if (a2dp_cmd_pending == A2DP_CTRL_CMD_NONE)
      {
        LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : no command pending, ignore ack");
        if (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE)
        {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : warning : command %s queued when no command pending, reset",
                  audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_queued));
          a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
        }
        return;
      } else if ((ack == A2DP_CTRL_ACK_SUCCESS) &&
              (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE) &&
              (a2dp_cmd_pending != a2dp_cmd_queued)) {
        // No need to send ACK to HAL as it is already timed out for current command
        // Not clearing queued command to identify later whether on completion of the same HAL to be ACKed back
        LOG_INFO(LOG_TAG, "btif_a2dp_audio_on_started: warning : queued command %s to be sent, on completion of %s",
                audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_queued),
                audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_pending));
        a2dp_cmd_pending = a2dp_cmd_queued;
        status = btif_a2dp_audio_snd_ctrl_cmd(a2dp_cmd_pending);

        if (a2dp_cmd_pending == A2DP_CTRL_CMD_START) {
          LOG_INFO(LOG_TAG,"calling method a2dp_on_started");
          auto ret = btAudio->a2dp_on_started(mapToStatus(status));
          if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
        }
        return;
      } else {
        // On current command ack failure, we do not process queued command, but flush it
        if (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE) {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : Not acking as ack is waited for queued command");
          /* no need to ack as we alreday unblocked HAL with error
               A2DP_CTRL_ACK_PREVIOUS_COMMAND_PENDING in case of queued command*/
          a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
        } else if (a2dp_local_cmd_pending != A2DP_CTRL_CMD_NONE) {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : Not acking as ack is waited for local command");
        } else {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_started : Send Ack for pending command");

          if (a2dp_cmd_pending == A2DP_CTRL_CMD_START) {
            LOG_INFO(LOG_TAG,"calling method a2dp_on_started");
            auto ret = btAudio->a2dp_on_started(mapToStatus(status));
            if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
          }
        }
        /* clear pending and queued*/
        a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
      }
    } else {
      if (a2dp_cmd_pending == A2DP_CTRL_CMD_START) {
        LOG_INFO(LOG_TAG,"calling method a2dp_on_started");
        auto ret = btAudio->a2dp_on_started(mapToStatus(status));
        if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
        a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
      }
    }
  }
}

void btif_a2dp_audio_on_suspended(tBTA_AV_STATUS status)
{
  uint8_t ack = status;

  LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : status = %d", status);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr){
    if (property_get("persist.vendor.bt.a2dp.hal.implementation", a2dp_hal_imp, "false") &&
            !strcmp(a2dp_hal_imp, "true")) {
      if (a2dp_cmd_pending == A2DP_CTRL_CMD_NONE)
      {
        LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : no command pending, ignore ack");
        if (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE)
        {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : warning : command %s queued when no command pending, reset",
                  audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_queued));
          a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
        }
        return;
      } else if ((ack == A2DP_CTRL_ACK_SUCCESS) &&
              (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE) &&
              (a2dp_cmd_pending != a2dp_cmd_queued)) {
        // No need to send ACK to HAL as it is already timed out for current command
        // Not clearing queued command to identify later whether on completion of the same HAL to be ACKed back
        LOG_INFO(LOG_TAG, "btif_a2dp_audio_on_suspended : warning : queued command %s to be sent, on completion of %s",
                audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_queued),
                audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_pending));
        a2dp_cmd_pending = a2dp_cmd_queued;
        status = btif_a2dp_audio_snd_ctrl_cmd(a2dp_cmd_pending);

        if (a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND || a2dp_cmd_pending == A2DP_CTRL_CMD_STOP) {
          LOG_INFO(LOG_TAG,"calling method a2dp_on_suspended");
          auto ret = btAudio->a2dp_on_suspended(mapToStatus(status));
          if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
        }
        return;
      } else {
        // On current command ack failure, we do not process queued command, but flush it
        if (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE) {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : Not acking as ack is waited for queued command");
          /* no need to ack as we alreday unblocked HAL with error
               A2DP_CTRL_ACK_PREVIOUS_COMMAND_PENDING in case of queued command*/
          a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
        } else if (a2dp_local_cmd_pending != A2DP_CTRL_CMD_NONE) {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : Not acking as ack is waited for local command");
        } else {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_suspended : Send Ack for pending command");

          if (a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND || a2dp_cmd_pending == A2DP_CTRL_CMD_STOP) {
            LOG_INFO(LOG_TAG,"calling method a2dp_on_suspended");
            auto ret = btAudio->a2dp_on_suspended(mapToStatus(status));
            if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
          }
        }
        /* clear pending and queued*/
        a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
      }
    } else {
      if (a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND || a2dp_cmd_pending == A2DP_CTRL_CMD_STOP) {
        LOG_INFO(LOG_TAG,"calling method a2dp_on_suspended");
        auto ret = btAudio->a2dp_on_suspended(mapToStatus(status));
        if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
        a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
      }
    }
  }
}

void btif_a2dp_audio_on_stopped(tBTA_AV_STATUS status)
{
  uint8_t ack = status;

  LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : status = %d",status);
  APPL_TRACE_IMP("%s tx_started: %d, tx_stop_initiated: %d",
         __func__, btif_a2dp_src_vsc.tx_started, btif_a2dp_src_vsc.tx_stop_initiated);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr){
    if (btif_a2dp_src_vsc.tx_started && !btif_a2dp_src_vsc.tx_stop_initiated &&
        btif_a2dp_src_vsc.multi_vsc_support) {//Is this handling required? If so in which scenario?
      bta_av_vendor_offload_stop(NULL);
    } else {
      if (property_get("persist.vendor.bt.a2dp.hal.implementation", a2dp_hal_imp, "false") &&
              !strcmp(a2dp_hal_imp, "true")) {
        if (a2dp_cmd_pending == A2DP_CTRL_CMD_NONE)
        {
          LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : no command pending, ignore ack");
          if (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE)
          {
            LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : warning : command %s queued when no command pending, reset",
                    audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_queued));
            a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
          }
          return;
        } else if ((ack == A2DP_CTRL_ACK_SUCCESS) &&
                (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE) &&
                (a2dp_cmd_pending != a2dp_cmd_queued)) {
          // No need to send ACK to HAL as it is already timed out for current command
          // Not clearing queued command to identify later whether on completion of the same HAL to be ACKed back
          LOG_INFO(LOG_TAG, "btif_a2dp_audio_on_stopped : warning : queued command %s to be sent, on completion of %s",
                  audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_queued),
                  audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)a2dp_cmd_pending));
          a2dp_cmd_pending = a2dp_cmd_queued;
          status = btif_a2dp_audio_snd_ctrl_cmd(a2dp_cmd_pending);

          if (a2dp_cmd_pending == A2DP_CTRL_CMD_STOP || a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND) {
            LOG_INFO(LOG_TAG,"calling method a2dp_on_stopped");
            auto ret = btAudio->a2dp_on_stopped(mapToStatus(status));
            if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_stopped: server died");
          } else if ((a2dp_cmd_pending == A2DP_CTRL_CMD_START) &&
                (!(btif_av_is_under_handoff() || reconfig_a2dp))) {
            LOG_INFO(LOG_TAG, "Remote disconnected when start under progress");
            auto ret = btAudio->a2dp_on_started(mapToStatus(A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS));
            if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_started: server died");
          }
          return;
        } else {
          // On current command ack failure, we do not process queued command, but flush it
          if (a2dp_cmd_queued != A2DP_CTRL_CMD_NONE) {
            LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : Not acking as ack is waited for queued command");
            /* no need to ack as we alreday unblocked HAL with error
                 A2DP_CTRL_ACK_PREVIOUS_COMMAND_PENDING in case of queued command*/
            a2dp_cmd_queued = A2DP_CTRL_CMD_NONE;
          } else if (a2dp_local_cmd_pending != A2DP_CTRL_CMD_NONE) {
            LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : Not acking as ack is waited for local command");
          } else {
            LOG_INFO(LOG_TAG,"btif_a2dp_audio_on_stopped : Send Ack for pending command");

            if (a2dp_cmd_pending == A2DP_CTRL_CMD_STOP || a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND) {
              LOG_INFO(LOG_TAG,"calling method a2dp_on_stopped");
              auto ret = btAudio->a2dp_on_stopped(mapToStatus(status));
              if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_stopped: server died");
            } else if ((a2dp_cmd_pending == A2DP_CTRL_CMD_START) &&
                    (!(btif_av_is_under_handoff() || reconfig_a2dp))) {
              LOG_INFO(LOG_TAG, "Remote disconnected when start under progress");
              auto ret = btAudio->a2dp_on_started(mapToStatus(A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS));
              if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_started: server died");
            }
          }
          /* clear pending and queued*/
          a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
        }
      } else {
        if (a2dp_cmd_pending == A2DP_CTRL_CMD_STOP || a2dp_cmd_pending == A2DP_CTRL_CMD_SUSPEND) {
          LOG_INFO(LOG_TAG,"calling method a2dp_on_stopped");
          auto ret = btAudio->a2dp_on_stopped(mapToStatus(status));
          if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_stopped: server died");
          a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
        } else if ((a2dp_cmd_pending == A2DP_CTRL_CMD_START) &&
                (!(btif_av_is_under_handoff() || reconfig_a2dp))) {
          LOG_INFO(LOG_TAG,"Remote disconnected when start under progress");
          auto ret = btAudio->a2dp_on_started(mapToStatus(A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS));
          if (!ret.isOk()) LOG_ERROR(LOG_TAG,"a2dp_on_started: server died");
          a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
        }
      }
    }
  }
}
void btif_a2dp_audio_send_start_req()
{
  uint8_t resp;
  resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_START);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
      auto ret =  btAudio->a2dp_on_started(mapToStatus(resp));
    if (resp != A2DP_CTRL_ACK_PENDING) {
      /*
       * Reset pending command. This is to avoid returning unsolicited
       * response to audio HAL when START succeeds later after timeout e.g.
       * once dual handoff is complete.
       */
      LOG_INFO(LOG_TAG,"%s, Resetting pending control command", __func__);
      a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
    }
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_suspend_req()
{
  uint8_t resp;
  resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_SUSPEND);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret =  btAudio->a2dp_on_suspended(mapToStatus(resp));
    if (resp != A2DP_CTRL_ACK_PENDING) {
      /*
       * Reset pending command. This is to avoid returning unsolicited
       * response to audio HAL when START succeeds later after timeout e.g.
       * once dual handoff is complete.
       */
      LOG_INFO(LOG_TAG,"%s, Resetting pending control command", __func__);
      a2dp_cmd_pending = A2DP_CTRL_CMD_NONE;
    }
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_stop_req()
{
  uint8_t resp;
  resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_STOP);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret =  btAudio->a2dp_on_stopped(mapToStatus(resp));
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_a2dp_ready_status()
{
  uint8_t resp;
  resp = btif_a2dp_audio_process_request(A2DP_CTRL_CMD_CHECK_READY);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret = btAudio->a2dp_on_check_ready(mapToStatus(resp));
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_codec_config()
{
  uint8_t resp;
  CodecCfg data;
  LOG_INFO(LOG_TAG,"a2dp_get_codec_config");
  resp = btif_a2dp_audio_process_request(A2DP_CTRL_GET_CODEC_CONFIG);
  LOG_INFO(LOG_TAG,"resp  = %d",resp);
  data.setToExternal(codec_info,len);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret = btAudio->a2dp_on_get_codec_config(mapToStatus(resp), data);
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_mcast_status()
{
  uint8_t mcast = 0;
  LOG_INFO(LOG_TAG,"btif_a2dp_audio_send_mcast_status:multicast");
  //Mulitcast not supported currently
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret = btAudio->a2dp_on_get_multicast_status(mcast);
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_num_connected_devices()
{
  uint8_t num_dev = 1;
  LOG_INFO(LOG_TAG,"send_num_connected_devices");
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret = btAudio->a2dp_on_get_num_connected_devices(num_dev);
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_connection_status()
{
  uint8_t resp;
  LOG_INFO(LOG_TAG,"send_connection_status");
  resp = btif_a2dp_audio_process_request(A2DP_CTRL_GET_CONNECTION_STATUS);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret = btAudio->a2dp_on_get_connection_status(mapToStatus(resp));
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}
void btif_a2dp_audio_send_sink_latency()
{
  uint16_t sink_latency;

  if (isBAEnabled()) {
      sink_latency = btif_get_ba_latency();
  }
  else {
      sink_latency = btif_av_get_sink_latency();
  }
  LOG_INFO(LOG_TAG,"send_sink_latency = %d", sink_latency);
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto ret = btAudio->a2dp_on_get_sink_latency(sink_latency);
    if (!ret.isOk()) LOG_ERROR(LOG_TAG,"server died");
  }
}

void on_hidl_server_died() {
  LOG_INFO(LOG_TAG,"on_hidl_server_died");
  Lock lock(mtxBtAudio);
  if (btAudio != nullptr) {
    auto hidl_death_unlink = btAudio->unlinkToDeath(BTAudioHidlDeathRecipient);
    if (!hidl_death_unlink.isOk()) LOG_ERROR(LOG_TAG,"hidl_death_unlink server died");
    btAudio = nullptr;
    usleep(500000); //sleep for 0.5sec for hal server to restart
    btif_dispatch_sm_event(BTIF_AV_REINIT_AUDIO_IF,NULL,0);
  }
}
uint8_t btif_a2dp_audio_process_request(uint8_t cmd)
{
  APPL_TRACE_DEBUG(LOG_TAG,"btif_a2dp_audio_process_request %s", audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
  uint8_t status = A2DP_CTRL_ACK_FAILURE;
  if (property_get("persist.vendor.bt.a2dp.hal.implementation", a2dp_hal_imp, "false") &&
          !strcmp(a2dp_hal_imp, "true")) {
    switch (cmd) {
      case A2DP_CTRL_CMD_CHECK_READY:
        a2dp_local_cmd_pending = cmd;
        if (btif_a2dp_source_media_task_is_shutting_down()) {
          APPL_TRACE_WARNING("%s: A2DP command %s while media task shutting down",
                             __func__,audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
          status = A2DP_CTRL_ACK_FAILURE;
        } else if (!bluetooth::headset::btif_hf_is_call_vr_idle()) {
          status  = A2DP_CTRL_ACK_INCALL_FAILURE;
        } else if (deinit_pending) {
          APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
          status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
        } else if (isBAEnabled()) {
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }else if (btif_av_is_under_handoff() || reconfig_a2dp || btif_a2dp_src_vsc.tx_started) {
          /*There can be instances where because of remote start received early, reconfig
          flag may get reset, for such case check for tx_started flag set as well,
          this would help returning proper status to MM*/
          APPL_TRACE_IMP("%s: A2DP command %s, reconfig: %d, tx_started:%d",
                  __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd), reconfig_a2dp,
                  btif_a2dp_src_vsc.tx_started);
          status = A2DP_CTRL_ACK_SUCCESS;
        } else if (btif_av_stream_ready() || btif_av_stream_started_ready()) {
          /* check whether AV is ready to setup A2DP datapath */
          status = A2DP_CTRL_ACK_SUCCESS;
        } else {
          APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                  __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
          status = A2DP_CTRL_ACK_FAILURE;
        }
        a2dp_local_cmd_pending = A2DP_CTRL_CMD_NONE;
        break;

      case A2DP_CTRL_CMD_OFFLOAD_START: {
        uint8_t hdl = 0;
        int idx = btif_av_get_latest_playing_device_idx();
        if (idx < btif_max_av_clients) {
          hdl = btif_av_get_av_hdl_from_idx(idx);
          APPL_TRACE_DEBUG("%s: hdl = %d",__func__, hdl);
        } else {
          APPL_TRACE_ERROR("%s: Invalid index",__func__);
          status = -1;
          break;
        }
        btif_dispatch_sm_event(BTIF_AV_OFFLOAD_START_REQ_EVT, (char *)&hdl, 1);
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }
      case A2DP_CTRL_GET_CODEC_CONFIG: {
        uint8_t p_codec_info[AVDT_CODEC_SIZE];
        uint8_t codec_type;
        tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
        uint32_t bitrate = 0;
        len = 0;
        a2dp_local_cmd_pending = cmd;
        LOG_INFO(LOG_TAG,"A2DP_CTRL_GET_CODEC_CONFIG");
        A2dpCodecConfig *CodecConfig = bta_av_get_a2dp_current_codec();
        if (CodecConfig == nullptr)
        {
            LOG_INFO(LOG_TAG,"codec config pointer is NULL");
            status = A2DP_CTRL_ACK_FAILURE;
            break;
        }
        bta_av_co_get_peer_params(&peer_param);
        LOG_INFO(LOG_TAG,"enc_update_in_progress = %d", enc_update_in_progress);
        if ((btif_av_stream_started_ready() == FALSE) ||
                (enc_update_in_progress == TRUE))
        {
          LOG_INFO(LOG_TAG,"A2DP_CTRL_GET_CODEC_CONFIG: stream not started");
          if (btif_av_is_start_ack_pending() == FALSE)
          {
            status = A2DP_CTRL_ACK_FAILURE;
            a2dp_local_cmd_pending = A2DP_CTRL_CMD_NONE;
            break;
          }
          else
          {
            LOG_INFO(LOG_TAG, "A2DP_CTRL_GET_CODEC_CONFIG: stream start ack is pending");
          }
        }
        memset(p_codec_info, 0, AVDT_CODEC_SIZE);
        memset(codec_info, 0, 30);
        if (!CodecConfig->copyOutOtaCodecConfig(p_codec_info))
        {
          LOG_INFO(LOG_TAG,"No valid codec config");
          status = A2DP_CTRL_ACK_FAILURE;
          a2dp_local_cmd_pending = A2DP_CTRL_CMD_NONE;
          break;
        }
        memcpy(&codec_info[1], &p_codec_info, p_codec_info[0] + 1);

        codec_type = A2DP_GetCodecType((const uint8_t*)p_codec_info);
        LOG_INFO(LOG_TAG,"codec_type = %x",codec_type);
        if (A2DP_MEDIA_CT_SBC == codec_type)
        {
          bitrate = A2DP_GetOffloadBitrateSbc(CodecConfig, peer_param.is_peer_edr);
          LOG_INFO(LOG_TAG,"bitrate = %d", bitrate);
          bitrate *= 1000;
        }
        else if (A2DP_MEDIA_CT_NON_A2DP == codec_type)
        {
          int samplerate = A2DP_GetTrackSampleRate(p_codec_info);
          if ((A2DP_VendorCodecGetVendorId(p_codec_info)) == A2DP_LDAC_VENDOR_ID) {
            bitrate = A2DP_GetTrackBitRate(p_codec_info);
            LOG_INFO(LOG_TAG,"bitrate = %d", bitrate);
          } else {
            /* BR = (Sampl_Rate * PCM_DEPTH * CHNL)/Compression_Ratio */
            int bits_per_sample = 16; // TODO
            bitrate = (samplerate * bits_per_sample * 2)/4;
          }
        }
        else if (A2DP_MEDIA_CT_AAC == codec_type)
        {
          bitrate = 0;//Bitrate is present in codec info
        }
        codec_info[0] = 0; //playing device handle
        len = p_codec_info[0] + 2;
        codec_info[len++] = (uint8_t)(peer_param.peer_mtu & 0x00FF);
        codec_info[len++] = (uint8_t)(((peer_param.peer_mtu & 0xFF00) >> 8) & 0x00FF);
        codec_info[len++] = (uint8_t)(bitrate & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF00) >> 8) & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF0000) >> 16) & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF000000) >> 24) & 0x00FF);
        LOG_INFO(LOG_TAG,"len  = %d", len);
        status = A2DP_CTRL_ACK_SUCCESS;
        a2dp_local_cmd_pending = A2DP_CTRL_CMD_NONE;
        break;
      }
      case A2DP_CTRL_GET_CONNECTION_STATUS:
        a2dp_local_cmd_pending = cmd;
        if (btif_av_is_connected() && !(btif_a2dp_source_media_task_is_shutting_down()))
        {
          BTIF_TRACE_DEBUG("got valid connection");
          status = A2DP_CTRL_ACK_SUCCESS;
        }
        else
          status = A2DP_CTRL_ACK_FAILURE;
        a2dp_local_cmd_pending = A2DP_CTRL_CMD_NONE;
        break;

      case A2DP_CTRL_GET_MULTICAST_STATUS:
        status = 0; //FALSE
        break;

      case A2DP_CTRL_GET_NUM_CONNECTED_DEVICE:
        status = 1;
        break;

      default:
        if (a2dp_cmd_pending != A2DP_CTRL_CMD_NONE)
        {
          status = A2DP_CTRL_ACK_PREVIOUS_COMMAND_PENDING;
          APPL_TRACE_ERROR("a2dp_process_request: warning : previous command pending, queueing command");
          if (a2dp_cmd_pending == cmd)
          {
            APPL_TRACE_ERROR("a2dp_process_request: Not queuing same command");
          }
          else
          {
            a2dp_cmd_queued = cmd;
          }
        }
        else
        {
          a2dp_cmd_pending = cmd;
          status = btif_a2dp_audio_snd_ctrl_cmd(cmd);
        }
    }
  } else {
    a2dp_cmd_pending = cmd;

    switch (cmd) {
      case A2DP_CTRL_CMD_CHECK_READY:
        if (btif_a2dp_source_media_task_is_shutting_down()) {
          APPL_TRACE_WARNING("%s: A2DP command %s while media task shutting down",
                  __func__,audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
          status = A2DP_CTRL_ACK_FAILURE;
          break;
        }
        if (!bluetooth::headset::btif_hf_is_call_vr_idle()) {
          status  = A2DP_CTRL_ACK_INCALL_FAILURE;
          break;
        }
        if (deinit_pending) {
          APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
          status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
          break;
        }
        if (isBAEnabled())
        {
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        /*There can be instances where because of remote start received early, reconfig
        flag may get reset, for such case check for tx_started flag set as well,
        this would help returning proper status to MM*/
        APPL_TRACE_IMP("%s: A2DP command %s, reconfig: %d, tx_started:%d",
                __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd), reconfig_a2dp,
                btif_a2dp_src_vsc.tx_started);
        if (btif_av_is_under_handoff() || reconfig_a2dp || btif_a2dp_src_vsc.tx_started) {
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        /* check whether AV is ready to setup A2DP datapath */
        if (btif_av_stream_ready() || btif_av_stream_started_ready()) {
          status = A2DP_CTRL_ACK_SUCCESS;
        } else {
          APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                  __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
          status = A2DP_CTRL_ACK_FAILURE;
        }
        break;

      case A2DP_CTRL_CMD_START:
        /*
         * Don't send START request to stack while we are in a call.
         * Some headsets such as "Sony MW600", don't allow AVDTP START
         * while in a call, and respond with BAD_STATE.
         */
        if (!bluetooth::headset::btif_hf_is_call_vr_idle()) {
          status = A2DP_CTRL_ACK_INCALL_FAILURE;
          break;
        }
        if (deinit_pending) {
          APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
          status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
          break;
        }
        if (isBAEnabled())
        {
          ba_send_message(BTIF_BA_AUDIO_START_REQ_EVT, 0, NULL, false);
          status = A2DP_CTRL_ACK_PENDING;
          break;
        }
        if (btif_a2dp_source_is_remote_start()) {
          int remote_start_idx = btif_get_is_remote_started_idx();
          APPL_TRACE_DEBUG("%s: remote started idx = %d",__func__, remote_start_idx);
          if ((remote_start_idx < btif_max_av_clients) &&
                  btif_av_is_playing_on_other_idx(remote_start_idx)) {
            APPL_TRACE_WARNING("%s: Already playing on other index, don't cancel remote start timer",__func__);
            status = A2DP_CTRL_ACK_PENDING;
          } else {
            APPL_TRACE_WARNING("%s: remote a2dp started, cancel remote start timer", __func__);
            btif_a2dp_source_cancel_remote_start();
            btif_dispatch_sm_event(BTIF_AV_RESET_REMOTE_STARTED_FLAG_UPDATE_AUDIO_STATE_EVT, NULL, 0);
            status = A2DP_CTRL_ACK_PENDING;
          }
        }

        if (audio_start_awaited) {
          if (reconfig_a2dp || (btif_av_is_under_handoff())) {
            APPL_TRACE_DEBUG("Audio start awaited handle start under handoff");
            audio_start_awaited = false;
            btif_dispatch_sm_event(BTIF_AV_START_STREAM_REQ_EVT, NULL, 0);
            int idx = btif_av_get_latest_device_idx_to_start();
            if (btif_av_get_peer_sep(idx) == AVDT_TSEP_SRC)
              status = A2DP_CTRL_ACK_SUCCESS;
            break;
          }
        }

        /* In dual a2dp mode check for stream started first*/
        if (btif_av_stream_started_ready()) {
          /*
           * Already started, setup audio data channel listener and ACK
           * back immediately.
           */
          APPL_TRACE_DEBUG("Av stream already started");
          if (btif_a2dp_src_vsc.tx_start_initiated == TRUE) {
            APPL_TRACE_DEBUG("VSC exchange alreday started on Handoff Start, wait");
            status = A2DP_CTRL_ACK_PENDING;
            break;
          } else if (btif_a2dp_src_vsc.tx_started == FALSE) {
            int idx = btif_av_get_latest_playing_device_idx();
            uint8_t hdl = 0;
            APPL_TRACE_DEBUG("%s: latest playing idx = %d",__func__, idx);
            if (idx < btif_max_av_clients) {
              hdl = btif_av_get_av_hdl_from_idx(idx);
              APPL_TRACE_DEBUG("%s: hdl = %d, enc_update_in_progress = %d",__func__, hdl,
                      enc_update_in_progress);
              if (hdl >= 0) {
                btif_a2dp_source_setup_codec(hdl);
                enc_update_in_progress = TRUE;
              }
            } else {
                APPL_TRACE_ERROR("%s: Invalid index",__func__);
                status = -1;//Invalid status to stop start retry
                break;
            }
            APPL_TRACE_DEBUG("Start VSC exchange on MM Start when state is remote started on hdl = %d",hdl);
            btif_dispatch_sm_event(BTIF_AV_OFFLOAD_START_REQ_EVT, (char *)&hdl, 1);
            status = A2DP_CTRL_ACK_PENDING;
            break;
          }
          btif_av_reset_reconfig_flag();
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        if (btif_av_stream_ready()) {
          /*
           * Post start event and wait for audio path to open.
           * If we are the source, the ACK will be sent after the start
           * procedure is completed, othewise send it now.
           */
          btif_dispatch_sm_event(BTIF_AV_START_STREAM_REQ_EVT, NULL, 0);
          int idx = btif_av_get_latest_device_idx_to_start();
          if (btif_av_get_peer_sep(idx) == AVDT_TSEP_SRC) {
            status = A2DP_CTRL_ACK_SUCCESS;
            break;
          }
          /*Return pending and ack when start stream cfm received from remote*/
          status = A2DP_CTRL_ACK_PENDING;
          break;
        }

        APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
        return A2DP_CTRL_ACK_FAILURE;
        break;

      case A2DP_CTRL_CMD_STOP: {
        if (isBAEnabled())
        {
          ba_send_message(BTIF_BA_AUDIO_STOP_REQ_EVT, 0, NULL, false);
          status = A2DP_CTRL_ACK_PENDING;
          break;
        }
        int idx = btif_av_get_latest_playing_device_idx();
        if (deinit_pending) {
          APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
          status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
          break;
        }
        if ((!btif_av_is_split_a2dp_enabled() && btif_av_get_peer_sep(idx) == AVDT_TSEP_SNK &&
            !btif_a2dp_source_is_streaming()) ||
            (btif_av_is_split_a2dp_enabled() && btif_av_get_peer_sep(idx) == AVDT_TSEP_SNK &&
            btif_a2dp_src_vsc.tx_started == FALSE)) {
          /* We are already stopped, just ack back */
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }

        btif_dispatch_sm_event(BTIF_AV_STOP_STREAM_REQ_EVT, NULL, 0);
        status = A2DP_CTRL_ACK_SUCCESS;

        break;
      }
      case A2DP_CTRL_CMD_SUSPEND:
        /* Local suspend */
        if (deinit_pending) {
          APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
          status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
          break;
        }
        if (isBAEnabled()) {
          ba_send_message(BTIF_BA_AUDIO_PAUSE_REQ_EVT, 0, NULL, false);
          status = A2DP_CTRL_ACK_PENDING;
          break;
        }
        if (reconfig_a2dp ||
                btif_a2dp_source_is_remote_start()) {
          LOG_INFO(LOG_TAG,"Suspend called due to reconfig");
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        if (btif_av_stream_started_ready()) {
          APPL_TRACE_DEBUG("Suspend stream request to Av");
          btif_dispatch_sm_event(BTIF_AV_SUSPEND_STREAM_REQ_EVT, NULL, 0);
          status = A2DP_CTRL_ACK_PENDING;
          break;
        }/*pls check if we need to add a condition here */
        /* If we are not in started state, just ack back ok and let
         * audioflinger close the channel. This can happen if we are
         * remotely suspended, clear REMOTE SUSPEND flag.
         */
        status = A2DP_CTRL_ACK_SUCCESS;
        break;

      case A2DP_CTRL_CMD_OFFLOAD_START: {
         uint8_t hdl = 0;
         int idx = btif_av_get_latest_playing_device_idx();
         if (idx < btif_max_av_clients) {
           hdl = btif_av_get_av_hdl_from_idx(idx);
           APPL_TRACE_DEBUG("%s: hdl = %d",__func__, hdl);
         } else {
           APPL_TRACE_ERROR("%s: Invalid index",__func__);
           status = -1;
           break;
         }
        btif_dispatch_sm_event(BTIF_AV_OFFLOAD_START_REQ_EVT, (char *)&hdl, 1);
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }
      case A2DP_CTRL_GET_CODEC_CONFIG: {
        uint8_t p_codec_info[AVDT_CODEC_SIZE];
        uint8_t codec_type;
        tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
        uint32_t bitrate = 0;
        len = 0;
        LOG_INFO(LOG_TAG,"A2DP_CTRL_GET_CODEC_CONFIG");
        memset(p_codec_info, 0, AVDT_CODEC_SIZE);
        memset(codec_info, 0, 30);
        if (isBAEnabled())
        {
            getBACodecConfig(p_codec_info);
            codec_info[0] = 0; //playing device handle
            memcpy(&codec_info[1], &p_codec_info, p_codec_info[0] + 1);
            len = codec_info[1] + 1;
            for(uint8_t k = 0; k<(p_codec_info[0] + 1); k++ )
            {
                LOG_INFO(LOG_TAG,"codec_value[%d] = %x = %d",k,codec_info[k], codec_info[k]);
            }
            status = A2DP_CTRL_ACK_SUCCESS;
            // break it from here, we filled all relevant value
            break;
        }
        A2dpCodecConfig *CodecConfig = bta_av_get_a2dp_current_codec();
        if (CodecConfig == nullptr)
        {
            LOG_INFO(LOG_TAG,"codec config pointer is NULL");
            status = A2DP_CTRL_ACK_FAILURE;
            break;
        }
        bta_av_co_get_peer_params(&peer_param);
        LOG_INFO(LOG_TAG,"enc_update_in_progress = %d", enc_update_in_progress);
        if ((btif_av_stream_started_ready() == FALSE) ||
            (enc_update_in_progress == TRUE))
        {
            LOG_INFO(LOG_TAG,"A2DP_CTRL_GET_CODEC_CONFIG: stream not started");
            status = A2DP_CTRL_ACK_FAILURE;
            break;
        }
        if (!CodecConfig->copyOutOtaCodecConfig(p_codec_info))
        {
          LOG_INFO(LOG_TAG,"No valid codec config");
          status = A2DP_CTRL_ACK_FAILURE;
          break;
        }
        memcpy(&codec_info[1], &p_codec_info, p_codec_info[0] + 1);

        codec_type = A2DP_GetCodecType((const uint8_t*)p_codec_info);
        LOG_INFO(LOG_TAG,"codec_type = %x",codec_type);
        if (A2DP_MEDIA_CT_SBC == codec_type)
        {
          bitrate = A2DP_GetOffloadBitrateSbc(CodecConfig, peer_param.is_peer_edr);
          LOG_INFO(LOG_TAG,"bitrate = %d", bitrate);
          bitrate *= 1000;
        }
        else if (A2DP_MEDIA_CT_NON_A2DP == codec_type)
        {
          int samplerate = A2DP_GetTrackSampleRate(p_codec_info);
          if ((A2DP_VendorCodecGetVendorId(p_codec_info)) == A2DP_LDAC_VENDOR_ID) {
            bitrate = A2DP_GetTrackBitRate(p_codec_info);
            LOG_INFO(LOG_TAG,"bitrate = %d", bitrate);
          } else {
            /* BR = (Sampl_Rate * PCM_DEPTH * CHNL)/Compression_Ratio */
            int bits_per_sample = 16; // TODO
            bitrate = (samplerate * bits_per_sample * 2)/4;
          }
        }
        else if (A2DP_MEDIA_CT_AAC == codec_type)
        {
          bitrate = 0;//Bitrate is present in codec info
        }
        codec_info[0] = 0; //playing device handle
        len = p_codec_info[0] + 2;
        codec_info[len++] = (uint8_t)(peer_param.peer_mtu & 0x00FF);
        codec_info[len++] = (uint8_t)(((peer_param.peer_mtu & 0xFF00) >> 8) & 0x00FF);
        codec_info[len++] = (uint8_t)(bitrate & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF00) >> 8) & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF0000) >> 16) & 0x00FF);
        codec_info[len++] = (uint8_t)(((bitrate & 0xFF000000) >> 24) & 0x00FF);
        LOG_INFO(LOG_TAG,"len  = %d", len);
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
    case A2DP_CTRL_GET_CONNECTION_STATUS:
        if (isBAEnabled())
        {
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        if (btif_av_is_connected() && !(btif_a2dp_source_media_task_is_shutting_down()))
        {
          BTIF_TRACE_DEBUG("got valid connection");
          status = A2DP_CTRL_ACK_SUCCESS;
        }
        else
          status = A2DP_CTRL_ACK_FAILURE;
      break;

      case A2DP_CTRL_GET_MULTICAST_STATUS:
/*
          uint8_t playing_devices = (uint8_t)btif_av_get_num_playing_devices();
          BOOLEAN multicast_state = btif_av_get_multicast_state();
          a2dp_cmd_acknowledge(A2DP_CTRL_ACK_SUCCESS);
          multicast_query = FALSE;
          if ((btif_max_av_clients > 1 && playing_devices == btif_max_av_clients) &&
                  multicast_state)
          {
            multicast_query = TRUE;
          }
          BTIF_TRACE_ERROR("multicast status = %d",multicast_query);
          UIPC_Send(UIPC_CH_ID_AV_CTRL, 0, &multicast_query, 1);
          UIPC_Send(UIPC_CH_ID_AV_CTRL, 0, &playing_devices, 1);
*/
        status = 0; //FALSE
        break;

      case A2DP_CTRL_GET_NUM_CONNECTED_DEVICE:
        status = 1;
        break;

      default:
        APPL_TRACE_ERROR("UNSUPPORTED CMD (%d)", cmd);
        status = A2DP_CTRL_ACK_FAILURE;
        break;
    }
    APPL_TRACE_DEBUG("btif_a2dp_audio_process_request : %s DONE",
            audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
  }
  return status;
}

uint8_t btif_a2dp_audio_snd_ctrl_cmd(uint8_t cmd)
{
  uint8_t status = A2DP_CTRL_ACK_FAILURE;

  switch (cmd) {
    case A2DP_CTRL_CMD_START:
      /*
       * Don't send START request to stack while we are in a call.
       * Some headsets such as "Sony MW600", don't allow AVDTP START
       * while in a call, and respond with BAD_STATE.
       */
      if (!bluetooth::headset::btif_hf_is_call_vr_idle()) {
        status = A2DP_CTRL_ACK_INCALL_FAILURE;
        break;
      }
      if (deinit_pending) {
        APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
        status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
        break;
      }

      if (isBAEnabled())
      {
          ba_send_message(BTIF_BA_AUDIO_START_REQ_EVT, 0, NULL, false);
          status = A2DP_CTRL_ACK_PENDING;
          break;
      }

      if (btif_a2dp_source_is_remote_start()) {
        int remote_start_idx = btif_get_is_remote_started_idx();
        APPL_TRACE_DEBUG("%s: remote started idx = %d",__func__, remote_start_idx);
        if ((remote_start_idx < btif_max_av_clients) &&
            btif_av_is_playing_on_other_idx(remote_start_idx)) {
          APPL_TRACE_WARNING("%s: Already playing on other index, don't cancel remote start timer",__func__);
          status = A2DP_CTRL_ACK_PENDING;
        } else {
          APPL_TRACE_WARNING("%s: remote a2dp started, cancel remote start timer", __func__);
          btif_a2dp_source_cancel_remote_start();
          btif_dispatch_sm_event(BTIF_AV_RESET_REMOTE_STARTED_FLAG_UPDATE_AUDIO_STATE_EVT, NULL, 0);
          status = A2DP_CTRL_ACK_PENDING;
        }
      }

      if (audio_start_awaited) {
        if (reconfig_a2dp || (btif_av_is_under_handoff())) {
          APPL_TRACE_DEBUG("Audio start awaited handle start under handoff");
          audio_start_awaited = false;
          btif_dispatch_sm_event(BTIF_AV_START_STREAM_REQ_EVT, NULL, 0);
          int idx = btif_av_get_latest_device_idx_to_start();
          if (btif_av_get_peer_sep(idx) == AVDT_TSEP_SRC)
            status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
      }

      /* In dual a2dp mode check for stream started first*/
      if (btif_av_stream_started_ready()) {
        /*
         * Already started, setup audio data channel listener and ACK
         * back immediately.
         */
        APPL_TRACE_DEBUG("Av stream already started");
        if (btif_a2dp_src_vsc.tx_start_initiated == TRUE) {
          APPL_TRACE_DEBUG("VSC exchange alreday started on Handoff Start, wait");
          status = A2DP_CTRL_ACK_PENDING;
          break;
        } else if (btif_a2dp_src_vsc.tx_started == FALSE) {
          int idx = btif_get_is_remote_started_idx();
          uint8_t hdl = 0;
          APPL_TRACE_DEBUG("%s: remote started idx = %d",__func__, idx);
          if (idx < btif_max_av_clients) {
            hdl = btif_av_get_av_hdl_from_idx(idx);
            APPL_TRACE_DEBUG("%s: hdl = %d, enc_update_in_progress = %d",__func__, hdl,
                              enc_update_in_progress);
            if (hdl >= 0) {
              btif_a2dp_source_setup_codec(hdl);
              enc_update_in_progress = TRUE;
            }
          } else {
            APPL_TRACE_ERROR("%s: Invalid index",__func__);
            status = -1;//Invalid status to stop start retry
            break;
          }
          APPL_TRACE_DEBUG("Start VSC exchange on MM Start when state is remote started on hdl = %d",hdl);
          btif_dispatch_sm_event(BTIF_AV_OFFLOAD_START_REQ_EVT, (char *)&hdl, 1);
          status = A2DP_CTRL_ACK_PENDING;
          break;
        }
        btif_av_reset_reconfig_flag();
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
      if (btif_av_stream_ready()) {
        /*
         * Post start event and wait for audio path to open.
         * If we are the source, the ACK will be sent after the start
         * procedure is completed, othewise send it now.
         */
        btif_dispatch_sm_event(BTIF_AV_START_STREAM_REQ_EVT, NULL, 0);
        int idx = btif_av_get_latest_device_idx_to_start();
        if (btif_av_get_peer_sep(idx) == AVDT_TSEP_SRC) {
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }
        /*Return pending and ack when start stream cfm received from remote*/
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }

      APPL_TRACE_WARNING("%s: A2DP command %s while AV stream is not ready",
                         __func__, audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
      return A2DP_CTRL_ACK_FAILURE;
      break;

    case A2DP_CTRL_CMD_STOP:
      {
        if (isBAEnabled())
        {
            ba_send_message(BTIF_BA_AUDIO_STOP_REQ_EVT, 0, NULL, false);
            status = A2DP_CTRL_ACK_PENDING;
            break;
        }
        int idx = btif_av_get_latest_playing_device_idx();
        if (deinit_pending) {
          APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
          status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
          break;
        }
        if ((!btif_av_is_split_a2dp_enabled() && btif_av_get_peer_sep(idx) == AVDT_TSEP_SNK &&
            !btif_a2dp_source_is_streaming()) ||
            (btif_av_is_split_a2dp_enabled() && btif_av_get_peer_sep(idx) == AVDT_TSEP_SNK &&
            btif_a2dp_src_vsc.tx_started == FALSE)) {
          /* We are already stopped, just ack back */
          status = A2DP_CTRL_ACK_SUCCESS;
          break;
        }

        btif_dispatch_sm_event(BTIF_AV_STOP_STREAM_REQ_EVT, NULL, 0);
        status = A2DP_CTRL_ACK_SUCCESS;

        break;
      }
    case A2DP_CTRL_CMD_SUSPEND:
      /* Local suspend */
      if (deinit_pending) {
        APPL_TRACE_WARNING("%s:deinit pending return disconnected",__func__);
        status = A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS;
        break;
      }

      if (isBAEnabled())
      {
          ba_send_message(BTIF_BA_AUDIO_PAUSE_REQ_EVT, 0, NULL, false);
          status = A2DP_CTRL_ACK_PENDING;
          break;
      }

      if (reconfig_a2dp ||
          btif_a2dp_source_is_remote_start()) {
        LOG_INFO(LOG_TAG,"Suspend called due to reconfig");
        status = A2DP_CTRL_ACK_SUCCESS;
        break;
      }
      if (btif_av_stream_started_ready()) {
        APPL_TRACE_DEBUG("Suspend stream request to Av");
        btif_dispatch_sm_event(BTIF_AV_SUSPEND_STREAM_REQ_EVT, NULL, 0);
        status = A2DP_CTRL_ACK_PENDING;
        break;
      }/*pls check if we need to add a condition here */
      /* If we are not in started state, just ack back ok and let
       * audioflinger close the channel. This can happen if we are
       * remotely suspended, clear REMOTE SUSPEND flag.
       */
      status = A2DP_CTRL_ACK_SUCCESS;
      break;

    default:
      APPL_TRACE_ERROR("UNSUPPORTED CMD (%d)", cmd);
      status = A2DP_CTRL_ACK_FAILURE;
      break;
  }
  APPL_TRACE_DEBUG("a2dp_snd_ctrl_cmd : %s DONE",
                   audio_a2dp_hw_dump_ctrl_event((tA2DP_CTRL_CMD)cmd));
  return status;
}

/*
const char* dump_ctrl_event(tA2DP_CTRL_CMD_EXT cmd)
{
  switch(cmd) {
    CASE_RETURN_STR(A2DP_CTRL_CMD_CHECK_READY)
    CASE_RETURN_STR(A2DP_CTRL_CMD_START)
    CASE_RETURN_STR(A2DP_CTRL_CMD_STOP)
    CASE_RETURN_STR(A2DP_CTRL_CMD_SUSPEND)
    CASE_RETURN_STR( A2DP_CTRL_GET_CODEC_CONFIG)
    CASE_RETURN_STR(A2DP_CTRL_GET_MULTICAST_STATUS)
    CASE_RETURN_STR(A2DP_CTRL_GET_CONNECTION_STATUS)
    CASE_RETURN_STR(A2DP_CTRL_GET_NUM_CONNECTED_DEVICE)
  }
}*/
