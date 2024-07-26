/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file log.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-11-06
 *
 * @maintainer
 */

#ifndef COMMON_LOG_H_
#define COMMON_LOG_H_

#include <cstdarg>
#include <string>

#include "glog/logging.h"
#include "glog/raw_logging.h"

#define RESET "\033[0m"
#define BLACK "\033[0;30m"   /* Black */
#define RED "\033[0;31m"     /* Red */
#define GREEN "\033[0;32m"   /* Green */
#define YELLOW "\033[0;33m"  /* Yellow */
#define BLUE "\033[0;34m"    /* Blue */
#define MAGENTA "\033[0;35m" /* Magenta */
#define CYAN "\033[0;36m"    /* Cyan */
#define WHITE "\033[0;37m"   /* White */

#define ADEBUG DLOG(INFO) << "[DEBUG] "
#define AINFO google::LogMessage(__FILE__, __LINE__, google::INFO).stream()
#define AWARN google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()
#define AERROR google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()
#define AFATAL google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()

#define AINFO_IF(cond) LOG_IF(google::INFO, cond)
#define AWARN_IF(cond) LOG_IF(google::WARNING, cond)
#define AERROR_IF(cond) LOG_IF(google::ERROR, cond)
#define AFATAL_IF(cond) LOG_IF(google::FATAL, cond)

#define ACHECK(cond) CHECK(cond)

#define AINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define AWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define AERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return val;                        \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr) {               \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition) {                     \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition) {            \
    return;                   \
  }
#endif

#endif  // COMMON_LOG_H_
