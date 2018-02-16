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
#ifdef BT_LOGGER_LIB // gghai
#include "internal_include/bt_logger_lib.h" // gghai, add file

extern bt_logger_interface_t *logger_interface;
extern bool bt_logger_enabled;

#else
#include <stdio.h>
#include <sys/types.h>
/**
 * Commands
 */
typedef enum  {
    VENDOR_LOGGER_LOGS = 201, // Signifies Packet containing Logger Log
    GENERATE_VND_LOG_SIGNAL, // Signifies command to generate logs
    START_SNOOP_SIGNAL,
    STOP_SNOOP_SIGNAL,
    STOP_LOGGING_SIGNAL,
} CommandTypes;

/*
** Set property: "persist.bt_logger.log_mask" by ORing these features

** Example: To enable Full snoop logging and Dynamic logcat logs capture,
** property value should be (HCI_SNOOP_LOG_FULL|DYNAMIC_LOGCAT_CAPTURE) = 3
*/
typedef enum {
    HCI_SNOOP_LOG_LITE = 0,     // Always enabled, hci snoop logs sans media packets
    HCI_SNOOP_LOG_FULL = 1,     // Complete hci snoop logs with media packets
    DYNAMIC_LOGCAT_CAPTURE = 2,  // Level 6 logcat logs over logger socket
} LoggingFlags;

void init_vnd_Logger(void);
void clean_vnd_logger(void);

typedef struct {
    /** Set to sizeof(bt_vndor_interface_t) */
    size_t          size;

    /*
     * Functions need to be implemented in Logger libray (libbt-logClient.so).
     */

    /*
     *  Initialize logging by conneting client socket
     *  to Logger process
     */
    int   (*init)(void);

    /**  Sending Logs of Logger process */
    void (*send_log_msg)(const char *tag, const char *fmt_str, va_list ap);
    void (*send_log_data)(const char *tag, const char *fmt_str, ...);

    /**  Sending Logs of Logger process */
    void (*send_event)(char evt);

    /** Closes the socket connection to logger process */
    int  (*cleanup)(void);

} bt_logger_interface_t;

extern uint16_t vendor_logging_level;

#define GENERATE_VND_LOGS() if(logger_interface)logger_interface->send_event(GENERATE_VND_LOG_SIGNAL)
#define START_SNOOP_LOGGING() if(logger_interface)logger_interface->send_event(START_SNOOP_SIGNAL)
#define STOP_SNOOP_LOGGING() if(logger_interface)logger_interface->send_event(STOP_SNOOP_SIGNAL)

extern bt_logger_interface_t *logger_interface;
extern bool bt_logger_enabled;

#endif

/*
 * TODO(armansito): Work-around until we figure out a way to generate logs in a
 * platform-independent manner.
 */
#if defined(OS_GENERIC)

/* syslog didn't work well here since we would be redefining LOG_DEBUG. */
#include <stdio.h>

#define LOGWRAPPER(tag, fmt, args...) \
  fprintf(stderr, "%s: " fmt "\n", tag, ##args)

#define LOG_VERBOSE(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_DEBUG(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_INFO(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_WARN(...) LOGWRAPPER(__VA_ARGS__)
#define LOG_ERROR(...) LOGWRAPPER(__VA_ARGS__)

#define LOG_EVENT_INT(...)

#else /* !defined(OS_GENERIC) */

#include <log/log.h>

/**
 * These log statements are effectively executing only ALOG(_________, tag, fmt,
 * ## args ).
 * fprintf is only to cause compilation error when LOG_TAG is not provided,
 * which breaks build on Linux (for OS_GENERIC).
 */

#if LOG_NDEBUG
#define LOG_VERBOSE(tag, fmt, args...)                          \
  do {                                                          \
    (true) ? ((int)0) : fprintf(stderr, "%s" fmt, tag, ##args); \
  } while (0)
#else  // LOG_NDEBUG
#define LOG_VERBOSE(tag, fmt, args...)               \
  do {                                               \
    (true) ? ALOG(LOG_VERBOSE, tag, fmt, ##args)     \
           : fprintf(stderr, "%s" fmt, tag, ##args); \
  } while (0)
#endif  // !LOG_NDEBUG

#define LOG_DEBUG(tag, fmt, args...)                     \
  do {                                                   \
    (true) ? ALOG(LOG_DEBUG, tag, fmt, ##args)           \
           : fprintf(stderr, "%s" fmt, tag, ##args);     \
  } while (0);
#define LOG_INFO(tag, fmt, args...)                      \
  do {                                                   \
    (true) ? ALOG(LOG_INFO, tag, fmt, ##args)            \
           : fprintf(stderr, "%s" fmt, tag, ##args);     \
  } while (0);
#define LOG_WARN(tag, fmt, args...)                      \
  do {                                                   \
    (true) ? ALOG(LOG_WARN, tag, fmt, ##args)            \
           : fprintf(stderr, "%s" fmt, tag, ##args);     \
  } while (0);
#define LOG_ERROR(tag, fmt, args...)                     \
  do {                                                   \
    (true) ? ALOG(LOG_ERROR, tag, fmt, ##args)           \
           : fprintf(stderr, "%s" fmt, tag, ##args);     \
  } while (0);

#endif /* defined(OS_GENERIC) */
