/**********************************************************************/
/* Freeware Demo SW from the www.dsp-weimich.com                      */
/* Version 1.0                                                        */
/* median_filter.h                                                    */
/* Support the article Median Filter                                  */
/**********************************************************************/

/* Export defines */
#define TRUE     (0==0)
#define FALSE    (0==1)
#define uint8    unsigned char
#define sint8    signed char
#define uint16   unsigned int
#define sint16   signed int
#define uint32   unsigned long
#define sint32   signed long

/* Export function */
extern void InsertionSortMethod(sint16*, uint16);
extern sint16 ClassicMedianFilter(sint16*, uint16);
extern sint16 AdvanceMedianFilter(sint16*, uint16);
