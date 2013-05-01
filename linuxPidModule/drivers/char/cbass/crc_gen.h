/**
 * \file crc_gen.h
 * Functions and types for CRC checks.
 *
 * Generated on Wed Nov  5 09:58:48 2008,
 * by pycrc v0.6.6, http://www.tty1.net/pycrc/
 * using the configuration:
 *    Width        = 8
 *    Poly         = 0x07
 *    XorIn        = 0x00
 *    ReflectIn    = False
 *    XorOut       = 0x00
 *    ReflectOut   = False
 *    Algorithm    = table-driven
 *****************************************************************************/
#ifndef __CRC_GEN_H__
#define __CRC_GEN_H__
 #include <linux/init.h>
 #include <linux/module.h>
 #include <linux/kernel.h>

//#include <stdint.h>
//#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
//typedef unsigned char uint8_t ;
/**
 * The definition of the used algorithm.
 *****************************************************************************/
#define CRC_ALGO_TABLE_DRIVEN 1

/**
 * The type of the CRC values.
 *
 * This type must be big enough to contain at least 8 bits.
 *****************************************************************************/
typedef uint8_t crc_t;

//#ifndef __size_t_defined
//# define __size_t_defined
//typedef unsigned int size_t;
//typedef unsigned char crc_t;	
//#endif
/**
 * Calculate the initial crc value.
 *
 * \return     The initial crc value.
 *****************************************************************************/
static inline crc_t crc_init(void)
{
    return 0x00;
}

/**
 * Update the crc value with new data.
 *
 * \param crc      The current crc value.
 * \param data     Pointer to a buffer of \a data_len bytes.
 * \param data_len Number of bytes in the \a data buffer.
 * \return         The updated crc value.
 *****************************************************************************/
crc_t crc_update(crc_t crc, const unsigned char *data, size_t data_len);

/**
 * Calculate the final crc value.
 *
 * \param crc  The current crc value.
 * \return     The final crc value.
 *****************************************************************************/
static inline crc_t crc_finalize(crc_t crc)
{
    return crc ^ 0x00;
}


#ifdef __cplusplus
}           /* closing brace for extern "C" */
#endif

#endif      /* __CRC_GEN_H__ */
