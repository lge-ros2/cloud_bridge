/**
 *  @file   logging.hpp
 *  @date   2020-09-21
 *  @author Sungkyu Kang
 *  @brief
 *        Logging utility for Cloud Bridge
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 *
 *         SPDX-License-Identifier: MIT
 */

#ifndef _CLOUD_TRANS_LOGGING_H_
#define _CLOUD_TRANS_LOGGING_H_


#include <sstream>
#include <rcutils/logging.h>

#define LOG(x) \
  do { \
      std::stringstream ss; \
      ss << x; \
      rcutils_log(NULL, RCUTILS_LOG_SEVERITY_INFO, "cloud_bridge", "%s", ss.str().c_str()); \
  } while (0)

#define DEBUG(x) \
  do { \
      std::stringstream ss; \
      ss << x; \
      rcutils_log(NULL, RCUTILS_LOG_SEVERITY_DEBUG, "cloud_bridge", "%s", ss.str().c_str()); \
  } while (0)

#define ERROR(x) \
  do { \
      std::stringstream ss; \
      ss << x; \
      rcutils_log(NULL, RCUTILS_LOG_SEVERITY_ERROR, "cloud_bridge", "%s", ss.str().c_str()); \
  } while (0)

#endif // _CLOUD_TRANS_LOGGING_H_