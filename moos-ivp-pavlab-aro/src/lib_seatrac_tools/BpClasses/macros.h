/*==============================================================================
File that defines commonly used Macros
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef MACROS_H
#define MACROS_H

//Include the common type definitions
#include "types.h"

//==============================================================================
//Determine the type of compiler
//==============================================================================
#if !defined(COMPILER_CLANG) && defined(__clang__)
    #define COMPILER_CLANG
#elif !defined(COMPILER_CLANG) && (defined(__GNUC__) || defined(__GNUG__))
    #define COMPILER_GCC
#elif !defined(COMPILER_MSVC) && defined(_MSC_VER)
    #define COMPILER_MSVC
#endif

//==============================================================================
//Compiler Message Macros
//==============================================================================
#define STRINGIZE_HELPER(x)     #x
#define STRINGIZE(x)            STRINGIZE_HELPER(x)

#if defined(COMPILER_MSVC)
    /*! Directive that issues a parenthesised #pragma directive for MSVC compilers */
    #define PRAGMA(x)           __pragma(x)

    #define __MESSAGE(str)      PRAGMA(message(__FILE__ "(" STRINGIZE(__LINE__) "): " str))

    #define PRAGMA_INFO(str)    __MESSAGE(#str)
    #define PRAGMA_WARNING(str) __MESSAGE("WARNING: " #str)
    #define PRAGMA_ERROR(str)   __MESSAGE("ERROR: " #str)
    #define PRAGMA_TODO(str)    __MESSAGE("TO-DO(" STRINGIZE(__FUNCTION__) "): " #str)

#elif defined(COMPILER_GCC)
    /*! Directive that issues a parenthesised #pragma directive for GCC compilers */
    #define PRAGMA(x)			_Pragma(#x)

    #define PRAGMA_INFO(str)	PRAGMA(message #str)
    #define PRAGMA_WARNING(str)	PRAGMA(GCC warning #str)
    #define PRAGMA_ERROR(str)	PRAGMA(GCC error #str)
    #define PRAGMA_TODO(str)    PRAGMA(message("TO-DO: " #str))

#elif defined(__clang__)
    /*! Directive that issues a parenthesised #pragma directive for GCC compilers */
    #define PRAGMA(x)			_Pragma(#x)

    #define PRAGMA_INFO(str)	PRAGMA(clang message #str)
    #define PRAGMA_WARNING(str)	PRAGMA(clang  warning #str)
    #define PRAGMA_ERROR(str)	PRAGMA(clang error #str)
    #define PRAGMA_TODO(str)    PRAGMA(message("TO-DO: " #str))

#else
    #Unknown Compilter type
#endif

//==============================================================================
//GCC Macros
//==============================================================================
//Interrupt definition template
//#define FUNC_ISR				void __attribute__ ((interrupt))

//Weakly defined function template
//#define FUNC_WEAK				void __attribute__ ((weak))

//Function that should be placed and executed in RAM (and accessed with a Long-Call if it is far away from the calling statement)
//(see http://stackoverflow.com/questions/15137214/how-to-run-code-from-ram-on-arm-architecture
//and https://ez.analog.com/thread/10607)
//You may get an compiler warning that can be safely ignored: "Warning: ignoring changed section attributes for .data"
//here linker will place function in ram, but ideally linker requires seperatly defined ram function setion, but this is tedious.
//#define FUNC_RAM				void __attribute__ (( long_call, section(".data")))

//#define FUNC_RAM_OPTIMIZE_OFF	void __attribute__ (( long_call, section(".data"), optimize("O0")))

/*! Macro that disables optimisation for a specific function - usage "FUNC_OPTIMIZE_OFF foo(void) {}" */
#define FUNC_OPTIMIZE_OFF		void __attribute__((optimize("O0")))

//! Macro used to supress "parameter unused" warnings in GCC compiler
#define FUNC_PARAM_UNUSED(x)    ((void)x)

//==============================================================================
//Debug Optimisation Macros
//==============================================================================
#ifdef DEBUG
    /*! Macro that sets optimisation off for all following functions in the file until DEBUG_GLOBAL_OPTIMIZE_RESTORE is issued */
    #define DEBUG_GLOBAL_OPTIMIZE_OFF	\
        PRAGMA(GCC push_options) \
        PRAGMA(GCC optimize("O0"))

    #define DEBUG_GLOBAL_OPTIMIZE_RESTORE \
        PRAGMA(GCC pop_options)

    /*! Macro that disables optimisation for a specific function - usage "void DEBUG_FUNC_OPTIMIZE_OFF foo(void) {}" */
    #define DEBUG_FUNC_OPTIMIZE_OFF		__attribute__((optimize("O0")))

#else
    //#define DEBUG_NO_OPTIMIZE
    #define DEBUG_GLOBAL_OPTIMIZE_OFF
    #define DEBUG_GLOBAL_OPTIMIZE_RESTORE
    #define DEBUG_FUNC_OPTIMIZE_OFF
#endif

//==============================================================================
//Define Bit Manipulation & Testing Macros
//==============================================================================
#define BIT(b)                          (1u << ((uint8)b))

#define SET_BITS(r, bmsk)               (r |= (bmsk))

#define CLR_BITS(r, bmsk)               (r &= ~(bmsk))

#define INIT_BITS(r, msk, value)        if(value) { SET_BITS(r, msk); } else { CLR_BITS(r, msk); }

#define LOAD_BITS(r, msk, load)         CLR_BITS(r, msk); SET_BITS(r, (load & msk))

#define SET_BIT(r, b)                   SET_BITS(r, BIT(b))

#define CLR_BIT(r, b)                   CLR_BITS(r, BIT(b))

#define INIT_BIT(r, b, value)           if(value) { SET_BIT(r, b); } else { CLR_BIT(r, b); }

//!Macro that removes compiler warning C4800 "forcing value to bool 'true' or 'false' (performance warning)"
#define TOBOOL(x)                       ((x) != FALSE)

#define IS_BITS_MATCH(r, bmsk, value)   ((r & (bmsk)) == value)

//#define IS_BITS_SET(r, bmsk)          TOBOOL(r & (bmsk))
#define IS_BITS_SET(r, bmsk)            IS_BITS_MATCH(r, bmsk, bmsk)

//#define IS_BITS_CLR(r, bmsk)          TOBOOL(!IS_BITS_SET(r, bmsk))
#define IS_BITS_CLR(r, bmsk)            IS_BITS_MATCH(r, bmsk, 0)

#define IS_BIT_SET(r, b)                TOBOOL(r & BIT(b))

#define IS_BIT_CLR(r, b)                TOBOOL(!IS_BIT_SET(r,b))

//==============================================================================
//ANSI Escape Code "Control Sequence Identifiers"
//see http://en.wikipedia.org/wiki/ANSI_escape_code
//For use in PrintF statements as %s parameters
//==============================================================================
#define CSI_ERASE_LINE		"\x1b[2K"		/*!< Erases the entire contents of the cursors line, does not move cursor */
#define CSI_RESET_LINE		"\x1b[2K\r"		/*!< Erases the entire contents of the cursors line, resets cursor to start */

#define CSI_NORMAL			"\x1b[30;0m"	/*!< Reset text parameters */
#define CSI_TEXT_BLACK		"\x1b[30m"		/*!< Text colour Black ANSI escape sequence */
#define CSI_TEXT_MAROON		"\x1b[31m"		/*!< Text colour Dark Red ANSI escape sequence */
#define CSI_TEXT_FOREST		"\x1b[32m"		/*!< Text colour Dark Green ANSI escape sequence */
#define CSI_TEXT_OLIVE		"\x1b[33m"		/*!< Text colour Olive ANSI escape sequence */
#define CSI_TEXT_NAVY		"\x1b[34m"		/*!< Text colour Navy ANSI escape sequence */
#define CSI_TEXT_PURPLE		"\x1b[35m"		/*!< Text colour Purple ANSI escape sequence */
#define CSI_TEXT_TEAL		"\x1b[36m"		/*!< Text colour Teal ANSI escape sequence */
#define CSI_TEXT_SILVER		"\x1b[37m"		/*!< Text colour Silver ANSI escape sequence */

#define CSI_TEXT_GRAY		"\x1b[30;1m"	/*!< Text colour Gray ANSI escape sequence */
#define CSI_TEXT_RED		"\x1b[31;1m"	/*!< Text colour Red ANSI escape sequence */
#define CSI_TEXT_GREEN		"\x1b[32;1m"	/*!< Text colour Green ANSI escape sequence */
#define CSI_TEXT_YELLOW		"\x1b[33;1m"	/*!< Text colour Yellow ANSI escape sequence */
#define CSI_TEXT_BLUE		"\x1b[34;1m"	/*!< Text colour Blue ANSI escape sequence */
#define CSI_TEXT_MAGENTA	"\x1b[35;1m"	/*!< Text colour Magenta ANSI escape sequence */
#define CSI_TEXT_CYAN		"\x1b[36;1m"	/*!< Text colour Cyan ANSI escape sequence */
#define CSI_TEXT_WHITE		"\x1b[37;1m"	/*!< Text colour White ANSI escape sequence */

//==============================================================================
#endif
