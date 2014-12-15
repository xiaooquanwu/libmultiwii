#ifndef DLIB_FILE_H
#define DLIB_FILE_H

#ifndef STATIC_ASSERT
# define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]
#endif /*STATIC_ASSERT*/

/* Check Compiler */
#ifdef __clang__
# define D_COMPILER CLANG
# define D_USING_CLANG
#elif __INTEL_COMPILER
# define D_COMPILER INTEL
# define D_USING_ICC
#elif _MSC_VER
# define D_COMPILER MSVC
# define D_USING_MSVC
#elif __MINGW32__
# define D_COMPILER MINGW
# define D_USING_MINGW
#elif __MINGW64__
# define D_COMPILER MINGW
# define D_USING_MINGW
#elif __GNUC__
# define D_COMPILER GCC
# define D_USING_GCC
#else
# define D_COMPILER UNKNOWN
#endif /* check compilers */

/* Check OS */
#ifdef __ANDROID__
# define D_OS ANDROID
# define D_USING_ANDROID
#elif __FreeBSD__
# define D_OS FreeBSD
# define D_USING_BSD
# define D_USING_FreeBSD
#elif __NetBSD__
# define D_OS NetBSD
# define D_USING_BSD
# define D_USING_NetBSD
#elif __OpenBSD__
# define D_OS OpenBSD
# define D_USING_BSD
# define D_USING_OpenBSD
#elif __CYGWIN__
# define D_OS CYGWIN
# define D_USING_CYGWIN
#elif __gnu_linux__
# define D_OS LINUX
# define D_USING_LINUX
#elif __QNX__
# define D_OS QNX
# define D_USING_QNX
#elif _WIN32
# define D_OS WINDOWS
# define D_USING_MSWIN
#elif _WIN32_WCE
# define D_OS WINDOWS_CE
# define D_USING_MSWIN_CE
#else
# define D_OS UNKNOWN
#endif /* check OS */

/* check arch */
#if defined(D_USING_MSVC) && defined(_M_X64) || defined(_M_AMD64)
# define D_ARCH amd64
# define D_USING_AMD64
#elif defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(__x86_64)
# define D_ARCH amd64
# define D_USING_AMD64
#elif defined(__arm__) || defined(__thumb__) || defined(__TARGET_ARCH_ARM) \
    || defined(__TARGET_ARCH_THUMB) || defined(_ARM) || defined(_M_ARM)    \
    || defined(_M_ARMT)
# define D_ARCH ARM
# define D_USING_ARM
#elif defined(__aarch64__)
# define D_ARCH ARM64
# define D_USING_ARM64
#elif defined(i386) || defined(_M_I86) || defined(_M_IX86) || defined(_X86_) \
    || defined(__X86__) || defined(__I86__)
# define D_ARCH x86
# define D_USING_X86
#else
# define D_ARCH UNKNOWN
#endif /* check arch */

#ifndef ARRAYLEN
# define ARRAYLEN(D) sizeof(D) / sizeof(*D)
#endif /* ARRAYLEN */

// TODO: GCC check
typedef unsigned char       duint8_t;
typedef unsigned short      duint16_t;
typedef unsigned int        duint32_t;
typedef unsigned long long  duint64_t;

typedef signed char         dint8_t;
typedef signed short        dint16_t;
typedef signed int          dint32_t;
typedef signed long long    dint64_t;

#endif /* DLIB_FILE_H */
