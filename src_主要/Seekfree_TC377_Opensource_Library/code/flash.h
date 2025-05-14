/*
 * flash.h
 *
 *  Created on: 2024Äê9ÔÂ1ÈÕ
 *      Author: FZY_YQH
 */

#include "zf_common_headfile.h"

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

#define FLASH_SECTION_INDEX 0
#define FLASH_PAGE_INDEX    1

void eeprom_writ(void);
void eeprom_read(void);

#endif /* CODE_FLASH_H_ */
