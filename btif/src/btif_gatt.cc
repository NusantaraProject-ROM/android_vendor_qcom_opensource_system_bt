/******************************************************************************
 *
 *  Copyright (C) 2009-2013 Broadcom Corporation
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

/*******************************************************************************
 *
 *  Filename:      btif_gatt.c
 *
 *  Description:   GATT Profile Bluetooth Interface
 *
 ******************************************************************************/

#define LOG_TAG "bt_btif_gatt"

#include <base/bind.h>
#include <base/run_loop.h>
#include <base/threading/thread.h>

#include <errno.h>
#include <hardware/bluetooth.h>
#include <hardware/bt_gatt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btif_common.h"
#include "btif_util.h"

#include "bta_api.h"
#include "bta_gatt_api.h"
#include "btif_gatt.h"
#include "btif_gatt_util.h"
#include "btif_storage.h"
#include "osi/include/thread.h"

#define MAX_SCANNER_WORKQUEUE_COUNT    (1024)

const btgatt_callbacks_t* bt_gatt_callbacks = NULL;

thread_t* ble_scanner_workqueue_thread;
static const char* BLE_SCANNER_WORKQUEUE_NAME = "ble_scanner_workqueue";
static base::MessageLoop* scanner_message_loop_ = NULL;
static base::RunLoop* scanner_run_loop_ = NULL;

/**
 * This function posts a task into the scanner message loop, that executes it in
 * the scanner message loop.
 **/
bt_status_t do_in_ble_scanner_thread(const base::Location& from_here,
                             const base::Closure& task) {
  if (!scanner_message_loop_ || !scanner_message_loop_->task_runner().get()) {
    BTIF_TRACE_WARNING("%s: Dropped message, message_loop not initialized yet!",
                       __func__);
    return BT_STATUS_FAIL;
  }

  if (scanner_message_loop_->task_runner()->PostTask(from_here, task))
    return BT_STATUS_SUCCESS;

  BTIF_TRACE_ERROR("%s: Post task to task runner failed!", __func__);
  return BT_STATUS_FAIL;
}

bt_status_t do_in_ble_scanner_thread(const base::Closure& task) {
  return do_in_ble_scanner_thread(FROM_HERE, task);
}

static void ble_scanner_msg_loop_start_cb() {
  LOG_INFO(LOG_TAG, "%s", __func__);;
}

static void run_ble_scanner_message_loop(UNUSED_ATTR void* context) {
  LOG_INFO(LOG_TAG, "%s entered", __func__);

  scanner_message_loop_ = new base::MessageLoop(base::MessageLoop::Type::TYPE_DEFAULT);
  scanner_run_loop_ = new base::RunLoop();

  scanner_message_loop_->task_runner()->PostTask(FROM_HERE,
                                         base::Bind(&ble_scanner_msg_loop_start_cb));

  scanner_run_loop_->Run();

  delete scanner_message_loop_;
  scanner_message_loop_ = NULL;

  delete scanner_run_loop_;
  scanner_run_loop_ = NULL;

  LOG_INFO(LOG_TAG, "%s finished", __func__);
}



/*******************************************************************************
 *
 * Function         btif_gatt_init
 *
 * Description      Initializes the GATT interface
 *
 * Returns          bt_status_t
 *
 ******************************************************************************/
static bt_status_t btif_gatt_init(const btgatt_callbacks_t* callbacks) {
  ble_scanner_workqueue_thread = thread_new_sized(BLE_SCANNER_WORKQUEUE_NAME,
                                                  MAX_SCANNER_WORKQUEUE_COUNT);
  if (ble_scanner_workqueue_thread == NULL) {
    LOG_ERROR(LOG_TAG, "%s Unable to create thread %s", __func__,
              BLE_SCANNER_WORKQUEUE_NAME);
    goto error_exit;
  }

  thread_post(ble_scanner_workqueue_thread, run_ble_scanner_message_loop, nullptr);

  bt_gatt_callbacks = callbacks;
  return BT_STATUS_SUCCESS;

  error_exit:
    thread_free(ble_scanner_workqueue_thread);

    ble_scanner_workqueue_thread = NULL;

    return BT_STATUS_FAIL;
}

/*******************************************************************************
 *
 * Function         btif_gatt_cleanup
 *
 * Description      Closes the GATT interface
 *
 * Returns          void
 *
 ******************************************************************************/
static void btif_gatt_cleanup(void) {

  if (scanner_run_loop_ && scanner_message_loop_) {
    scanner_message_loop_->task_runner()->PostTask(FROM_HERE,
                                           scanner_run_loop_->QuitClosure());
  }

  thread_free(ble_scanner_workqueue_thread);
  ble_scanner_workqueue_thread = NULL;

  if (bt_gatt_callbacks) bt_gatt_callbacks = NULL;

  BTA_GATTC_Disable();
  BTA_GATTS_Disable();
}

static btgatt_interface_t btgattInterface = {
    sizeof(btgattInterface),

    btif_gatt_init,
    btif_gatt_cleanup,

    &btgattClientInterface,
    &btgattServerInterface,
    nullptr,  // filled in btif_gatt_get_interface
    nullptr   // filled in btif_gatt_get_interface
};

/*******************************************************************************
 *
 * Function         btif_gatt_get_interface
 *
 * Description      Get the gatt callback interface
 *
 * Returns          btgatt_interface_t
 *
 ******************************************************************************/
const btgatt_interface_t* btif_gatt_get_interface() {
  // TODO(jpawlowski) right now initializing advertiser field in static
  // structure cause explosion of dependencies. It must be initialized here
  // until those dependencies are properly abstracted for tests.
  btgattInterface.scanner = get_ble_scanner_instance();
  btgattInterface.advertiser = get_ble_advertiser_instance();
  return &btgattInterface;
}
