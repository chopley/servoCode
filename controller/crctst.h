/* FILE: crctst.h */

#ifndef CRCTST_H_INCLUDED
#define CRCTST_H_INCLUDED 1

/* define one of the following... */
//#define CRC8
//#define CRC10
//#define X25
//#define CRC16
//#define XMODEM
//#define AUTODINII
//#define MILSTD188

/* The following typedef's should be adjusted for the particular platform. */

/*CRC-8     x^8 + x^2 + x + 1 */
#define CRC8_POLY      0x0107

/*CRC-10    x^10 + x^9 + x^5 + x^4 + x + 1 */
#define CRC10_POLY     0x0633

/*CRC-16    x^16 + x^15 + x^2 + 1 */
#define CRC16_POLY     0xa001

/*X.25      x^16 + x^12 + x^5 + 1 */
#define X25_POLY       0x8408

/*XMODEM    x^16 + x^12 + x^5 + 1 (reflected/reversed) */
#define XMOD_POLY      0x1021

/*MILSTD188 x^24+x^23+x^18+x^17+x^14+x^11+x^10+x^7+x^6+x^5+x^4+x^3+x+1 */
#define MILSTD188_POLY 0x00df3261L

/*AUTODINII x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1 */
#define AUTODINII_POLY 0xedb88320L



#ifdef CRC8
#define LEN 4
#define CRC_ID "CRC-8"
#define CRC_PRN "%2.2x"
#endif

#ifdef CRC10
#define LEN 5
#define CRC_ID "CRC-10"
#define CRC_PRN "%4.4x"
#endif

#ifdef CRC16
#define LEN 5
#define CRC_ID "CRC16"
#define CRC_PRN "%4.4x"
#endif

#ifdef X25
#define LEN 5
#define CRC_ID "X.25"
#define CRC_PRN "%4.4x"
#endif

#ifdef XMODEM
#define LEN 5
#define CRC_ID "xmodem"
#define CRC_PRN "%4.4x"
#endif

#ifdef MILSTD188
#define LEN 6
#define CRC_ID "MIL STD 188-184"
#define CRC_PRN "%8.8lx"
#endif

#ifdef AUTODINII
#define LEN 7
#define CRC_ID "AUTODIN II"
#define CRC_PRN "%8.8lx"
#endif


extern void crcInit( void );
extern void displayTbl( unsigned char *tbl );
extern CRC_TYPE crcUpdate( unsigned char c );
extern CRC_TYPE crcRead( void );


#endif  /* end of if CRCTST_H_INCLUDED */


