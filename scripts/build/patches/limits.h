/* limits.h -- GCC compiler limits header (wrapper).
 *
 * This file belongs into GCC's internal include directory
 * (lib/gcc/loongarch64-linux-gnu/8.3.0/include/), where it is the first
 * <limits.h> found on the include search path.
 *
 * It defines the ISO C integer limit macros in terms of compiler builtins
 * and then chains to the system (glibc) <limits.h> via #include_next for
 * the POSIX additions.  Defining _GCC_LIMITS_H_ tells the glibc header that
 * the compiler limits have already been provided, so the glibc header will
 * not try to recurse looking for another limits.h.
 *
 * NOTE: patched in for the loongson-gnu-toolchain-8.3-i686-mingw
 * loongarch64-linux-gnu rc1.6 release, whose archive shipped without GCC's
 * internal limits.h (fixincludes installed only a README into
 * include-fixed/, leaving <limits.h> chains broken).
 */

#ifndef _GCC_LIMITS_H_
#define _GCC_LIMITS_H_ 1

/* Number of bits in a `char'.  */
#undef CHAR_BIT
#define CHAR_BIT __CHAR_BIT__

/* Maximum length of a multibyte character.  */
#ifndef MB_LEN_MAX
#define MB_LEN_MAX 1
#endif

/* Minimum and maximum values a `signed char' can hold.  */
#undef SCHAR_MIN
#define SCHAR_MIN (-SCHAR_MAX - 1)
#undef SCHAR_MAX
#define SCHAR_MAX __SCHAR_MAX__

/* Maximum value an `unsigned char' can hold.  (Minimum is 0).  */
#undef UCHAR_MAX
#define UCHAR_MAX (SCHAR_MAX * 2 + 1)

/* Minimum and maximum values a `char' can hold.  */
#ifdef __CHAR_UNSIGNED__
# undef CHAR_MIN
# define CHAR_MIN 0
# undef CHAR_MAX
# define CHAR_MAX UCHAR_MAX
#else
# undef CHAR_MIN
# define CHAR_MIN SCHAR_MIN
# undef CHAR_MAX
# define CHAR_MAX SCHAR_MAX
#endif

/* Minimum and maximum values a `signed short int' can hold.  */
#undef SHRT_MIN
#define SHRT_MIN (-SHRT_MAX - 1)
#undef SHRT_MAX
#define SHRT_MAX __SHRT_MAX__

/* Maximum value an `unsigned short int' can hold.  (Minimum is 0).  */
#undef USHRT_MAX
#define USHRT_MAX (SHRT_MAX * 2 + 1)

/* Minimum and maximum values a `signed int' can hold.  */
#undef INT_MIN
#define INT_MIN (-INT_MAX - 1)
#undef INT_MAX
#define INT_MAX __INT_MAX__

/* Maximum value an `unsigned int' can hold.  (Minimum is 0).  */
#undef UINT_MAX
#define UINT_MAX (INT_MAX * 2U + 1U)

/* Minimum and maximum values a `signed long int' can hold.  */
#undef LONG_MIN
#define LONG_MIN (-LONG_MAX - 1L)
#undef LONG_MAX
#define LONG_MAX __LONG_MAX__

/* Maximum value an `unsigned long int' can hold.  (Minimum is 0).  */
#undef ULONG_MAX
#define ULONG_MAX (LONG_MAX * 2UL + 1UL)

/* Minimum and maximum values a `signed long long int' can hold.  */
#ifdef __LONG_LONG_MAX__
# undef LLONG_MIN
# define LLONG_MIN (-LLONG_MAX - 1LL)
# undef LLONG_MAX
# define LLONG_MAX __LONG_LONG_MAX__
# undef ULLONG_MAX
# define ULLONG_MAX (LLONG_MAX * 2ULL + 1ULL)
#endif

/* Non-ISO extended limits, kept for compatibility with GNU mode code.  */
#ifndef __STRICT_ANSI__
# ifdef __LONG_LONG_MAX__
#  undef LONG_LONG_MIN
#  define LONG_LONG_MIN (-LONG_LONG_MAX - 1LL)
#  undef LONG_LONG_MAX
#  define LONG_LONG_MAX __LONG_LONG_MAX__
#  undef ULONG_LONG_MAX
#  define ULONG_LONG_MAX (LONG_LONG_MAX * 2ULL + 1ULL)
# endif
#endif

/* Chain to the system <limits.h> (glibc) for POSIX and XOPEN additions.
 * glibc sees _GCC_LIMITS_H_ already defined and skips its own recursion.  */
#include_next <limits.h>

#endif /* _GCC_LIMITS_H_ */
