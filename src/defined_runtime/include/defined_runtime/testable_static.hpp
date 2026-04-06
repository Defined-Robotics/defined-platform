#ifndef DEFINED_RUNTIME_TESTABLE_STATIC_HPP
#define DEFINED_RUNTIME_TESTABLE_STATIC_HPP

/*!
 * \file testable_static.hpp
 * \brief TESTABLE_STATIC macro for exposing private helpers in test builds.
 *
 * In production builds (\c PRODUCTION_CODE == 1) the macro expands to
 * \c static, preserving internal linkage.  In test builds
 * (\c PRODUCTION_CODE == 0) it expands to nothing so that test translation
 * units can link against the symbol directly.
 *
 * Usage in a \c .cpp file:
 * \code{.cpp}
 * TESTABLE_STATIC bool validate_goal(double x, double y, double theta);
 * \endcode
 *
 * Usage in \c CMakeLists.txt:
 * \code{.cmake}
 * target_compile_definitions(bt_executor PRIVATE PRODUCTION_CODE=1)
 * target_compile_definitions(test_goto_action PRIVATE PRODUCTION_CODE=0)
 * \endcode
 */

#if PRODUCTION_CODE == 1
#define TESTABLE_STATIC static
#else
#define TESTABLE_STATIC
#endif

#endif  // DEFINED_RUNTIME_TESTABLE_STATIC_HPP
