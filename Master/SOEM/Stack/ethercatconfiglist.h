/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * DEPRECATED Configuration list of known EtherCAT slave devices.
 *
 * If a slave is found in this list it is configured according to the parameters
 * in the list. Otherwise the configuration info is read directly from the slave
 * EEPROM (SII or Slave Information Interface).
 */

#ifndef _ethercatconfiglist_
#define _ethercatconfiglist_

#ifdef __cplusplus
extern "C"
{
#endif

/*
   explanation of dev:
    1: static device with no IO mapping ie EK1100
    2: input device no mailbox ie simple IO device
    3: output device no mailbox
    4: input device with mailbox configuration
    5: output device with mailbox configuration
    6: input/output device no mailbox
    7: input.output device with mailbox configuration
*/
#define EC_CONFIGEND 0xffffffff

ec_configlist_t ec_configlist[] = {
      {/*Man=*/0x00000000,/*ID=*/0x00000000,/*Name=*/""          ,/*dtype=*/0,/*Ibits=*/ 0,/*Obits=*/ 0,/*SM2a*/     0,/*SM2f*/         0,/*SM3a*/     0,/*SM3f*/         0,/*FM0ac*/0,/*FM1ac*/0},
      {/*Man=*/0x00000929,/*ID=*/0x00000001,/*Name=*/"TSV-HL"    ,/*dtype=*/7,/*Ibits=*/ 56,/*Obits=*/ 56,/*SM2a*/0x1000,/*SM2f*/ 0x00010024,/*SM3a*/0x1100, /*SM3f*/  0x00010000,/*FM0ac*/1,/*FM1ac*/1},
      {/*Man=*/0x00000929,/*ID=*/0x00000001,/*Name=*/"TSV-HL"    ,/*dtype=*/7,/*Ibits=*/ 56,/*Obits=*/ 56,/*SM2a*/0x1000,/*SM2f*/ 0x00010024,/*SM3a*/0x1100, /*SM3f*/  0x00010000,/*FM0ac*/1,/*FM1ac*/1},
      {/*Man=*/EC_CONFIGEND,/*ID=*/0x00000000,/*Name=*/""       ,/*dtype=*/0,/*Ibits=*/ 0,/*Obits=*/ 0,/*SM2a*/     0,/*SM2f*/         0,/*SM3a*/     0,/*SM3f*/         0,/*FM0ac*/0,/*FM1ac*/0}
};

#ifdef __cplusplus
}
#endif

#endif
