// Copyright 2019 Toyota Research Institute
#pragma once

#include <string>

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

/// @def MALIDRIVE_DEMAND
/// Used to declare a demand. Will quit execution otherwise.
#define MALIDRIVE_DEMAND(condition) MALIPUT_DEMAND(condition)

/// @def MALIDRIVE_DEMAND
/// Aborts the program (via \::abort) with a message showing at least the
/// given message (macro argument), function name, file, and line.
#define MALIDRIVE_ABORT_MSG(msg) MALIPUT_ABORT_MESSAGE(msg)

/// @def MALIDRIVE_IS_IN_RANGE
/// Throws if `value` is within [`min_value`; `max_value`]. It forwards the call
/// to MALIPUT_THROW_UNLESS().
#define MALIDRIVE_IS_IN_RANGE(value, min_value, max_value) \
  MALIPUT_THROW_UNLESS(value >= min_value && value <= max_value)

/// @def MALIDRIVE_THROW_UNLESS
/// Used to declare a conditional throw.
#define MALIDRIVE_THROW_UNLESS(condition) MALIPUT_THROW_UNLESS(condition)

/// @def MALIDRIVE_THROW_MESSAGE
/// Throws with `msg`.
#define MALIDRIVE_THROW_MESSAGE(msg) MALIPUT_THROW_MESSAGE(msg)

/// @def MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN
/// Deletes the special member functions for copy-construction, copy-assignment,
/// move-construction, and move-assignment.
/// Invoke this macro in the public section of the class declaration, e.g.:
/// <pre>
/// class Foo {
///  public:
///   MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Foo)
///
///   // ...
/// };
/// </pre>
/// */
#define MALIDRIVE_NO_COPY_NO_MOVE_NO_ASSIGN(Classname) MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)

/// @def MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN
/// Implements default special member functions for copy-construction,
/// copy-assignment, move-construction, and move-assignment.
/// Invoke this macro in the public section of the class declaration, e.g.:
/// <pre>
/// class Foo {
///  public:
///   MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo)
///
///   // ...
/// };
/// </pre>
/// */
#define MALIDRIVE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname) MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname)

/// @def STR_SIMPLE
/// Internal stringify a token
#define STR_SIMPLE(x) #x

/// @def STR
/// Stringify a token
#define STR(x) STR_SIMPLE(x)

/// @def MALIDRIVE_VALIDATE
/// Used to validate that an argument passed into a function or method is true;
/// if not, an exception of type exctype is thrown.
#define MALIDRIVE_VALIDATE(pred, exctype, message)                           \
  do {                                                                       \
    if (!(pred)) {                                                           \
      std::string fullname = std::string(__FILE__);                          \
      size_t found = fullname.find_last_of("/");                             \
      std::string fname = fullname;                                          \
      if (found != std::string::npos) {                                      \
        fname = fullname.substr(found + 1);                                  \
      }                                                                      \
      std::string errmsg(fname);                                             \
      errmsg.append(":").append(__func__).append(":").append(STR(__LINE__)); \
      errmsg.append(": ").append(message);                                   \
      throw exctype(errmsg);                                                 \
    }                                                                        \
  } while (0)
