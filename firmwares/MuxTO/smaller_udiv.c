#include <stdint.h>

/*
;;; Smaller 32bit division function for CM0+ The gnu libc __udivsi3
;;;   function unrolls the bitwise loop, which is quicker, but
;;;   sigificantly large (255+ bytes), and undesirable on chips with
;;;   small flash memories.  This is slower and smaller
;;;

;;; This is essentially code take from Yiu's "The Definative Guide to
;;;   the Cortex-M0 and Cortex-M0+ Processors", with slight modificatins.
;;; By Bill Westfield (WestfW), Aug 2025

;;; Note that the libgcc module being replaced defines three different
;;;   symbols, and we have to define all three if we want our code to replace
;;;   that module without causing "multiple definition" link errors.
;;;     __udivsi3 is the basic worker function
;;;     __aeabi_uidiv is an alias for that (ARM CMSIS name rather than gcc?)
;;;     __aeabi_uidivmod does 0 divisor check and has the explicit remainder.
;;;   (those are all compatible, WRT to the actual math preformed.)
*/

asm("__udivsi3: .global __wrap___udivsi3\n"
    "__aeabi_uidivmod: .global __aeabi_uidivmod\n"
    "__aeabi_uidiv: .global __aeabi_uidiv\n"
    "myudivsi3: .global myudivsi3\n"
    ".syntax unified\n"
    ".thumb\n"
    /*
    * Inputs:
    *    R0 = dividend
    *    R1 = divider
    * Outputs
    *    R0 = quotient
    *    R1 = remainder
    */
    "        cmp      r1, #0\n"
    "        beq     divzero\n"
    "        push    {r2-r4, lr}\n" // Save registers
    "        movs    r2, r0\n"      // copy dividend
    "        movs    r3, #1\n"      // counter
    "        lsls    r3, #31\n"     // N = 0x80000000
    "        movs    r0, #0\n"      // initial Quotient
    "        movs    r4, #0\n"      // initial Tmp
    "loop:\n"
    "        lsls    r2, #1\n"      // Shift dividend, MSB go into carry
    "        adcs    r4, r4\n"      // Shift Tmp, carry move into LSB
    "        cmp     r4, r1\n"
    "        bcc     lessthan\n"
    "        adds    r0, r3\n"      // Increment quotient
    "        subs    r4, r1\n"
    "lessthan:\n"
    "        lsrs    r3, #1\n"
    "        bne     loop\n"
    "        movs    R1, R4\n"      // remainder in R1, Quotient already in R0
    "        pop     {r2-r4, pc}\n"
    "divzero: movs   r0, #0\n"      // Divide by yields 0, like libc
    "         bx      lr\n"
    "        .size myudivsi3, .-myudivsi3\n"
   );
