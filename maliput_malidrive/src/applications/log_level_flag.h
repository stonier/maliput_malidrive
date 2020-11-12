// Copyright 2020 Toyota Research Institute
#pragma once

#include <gflags/gflags.h>

#ifndef MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG

/// Declares FLAGS_log_level flag with all possible log level values.
#define MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG()         \
  DEFINE_string(log_level, "unchanged",                               \
                "Sets the log output threshold; possible values are " \
                "'unchanged', "                                       \
                "'trace', "                                           \
                "'debug', "                                           \
                "'info', "                                            \
                "'warn', "                                            \
                "'err', "                                             \
                "'critical', "                                        \
                "'off'.")

#endif  // MALIPUT_MALIDRIVE_APPLICATION_DEFINE_LOG_LEVEL_FLAG
