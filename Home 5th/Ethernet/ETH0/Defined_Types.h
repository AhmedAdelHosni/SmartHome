#ifndef DEFINED_TYPES_H
#define DEFINED_TYPES_H

/******************************************************************************/
/*                Definition of exported function like macros                 */
/******************************************************************************/

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/******************************************************************************/
/*         Definition of exported types (typedef, enum, struct, union)        */
/******************************************************************************/

# ifndef TRUE
#  define TRUE            1u
# endif

# ifndef FALSE
#  define FALSE           0u
# endif


typedef unsigned char bool_T;

typedef signed char           sint8;        
typedef unsigned char         uint8;        
typedef signed short          sint16;       
typedef unsigned short        uint16;       
typedef signed long           sint32;       
typedef unsigned long         uint32;       

/* typedefinition of unsigned 8bit integer */
typedef uint8      u8;
/* typedefinition of unsigned 16bit integer */
typedef uint16     u16;
/* typedefinition of unsigned 32bit integer */
typedef uint32     u32;
/* typedefinition of signed 8bit integer */
typedef sint8       si8;
/* typedefinition of signed 16bit integer */
typedef sint16      si16;
/* typedefinition of signed 32bit integer */
typedef sint32      si32;

#endif  /* DEFINED_TYPES_H */