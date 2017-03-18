/*
	Copyright (c) 2017 CurlyMo <curlymoo1@gmail.com>

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <signal.h>

#include "../../soc/soc.h"
#include "../../wiringX.h"
#include "../platform.h"
#include "cubietruck.h"

struct platform_t *cubietruck = NULL;

/*
CN8 (near Ethernet) 2x15 Pins

0v|3v
 -| -
 8| 9
10|11
12|13
14|15
0v|0v
16|17
18|19
20|21
22|23
24| -
25| -
 -| -
 -| -


CN9 (near USB Ports) 2x12 Pins

3v|3v
26|27
28|29
 0| 1
 2| 2
 4| 5
 6| 7
0v|0v
 -| -
 -| -
 -| -
 -| -
*/

static int map[] = {
	/*	PG4,	PG5,	PG6,	PG7,	*/
		196,	197,	198,	199,
	/*	PG8,	PG9,	PG10,	PG11,	*/
		200,	201,	202,	203,
	/*	PC19,	PC21,	PC20,	PC22,	*/
		 83,	 85,	 84,	 86,
	/*	PB14,	PB16,	PB15,	PB17,	*/
		 46,	 48,	 47,	 49,
	/*	PI20,	PI14,	PI21,	PI15,	*/
		276,	270,	277,	271,
	/*	PI3,	PB3,	PB2,	PB4,	*/
		259,	 35,	 34,	 36,
	/*	PB18,	PB19,	PG0,	PG3,	*/
		 50,	 51,	192,	195,
	/*	PG2,	PG1		*/
		194,	193
};

static int cubietruckValidGPIO(int pin) {
	if(pin >= 0 && pin < (sizeof(map)/sizeof(map[0]))) {
		return 0;
	} else {
		return -1;
	}
}

static int cubietruckSetup(void) {
	cubietruck->soc->setup();
	cubietruck->soc->setMap(map);
	cubietruck->soc->setIRQ(map);
	return 0;
}

void cubietruckInit(void) {
	platform_register(&cubietruck, "cubietruck");

	cubietruck->soc = soc_get("Allwinner", "A10");

	cubietruck->digitalRead = cubietruck->soc->digitalRead;
	cubietruck->digitalWrite = cubietruck->soc->digitalWrite;
	cubietruck->pinMode = cubietruck->soc->pinMode;
	cubietruck->setup = &cubietruckSetup;

	cubietruck->selectableFd = cubietruck->soc->selectableFd;
	cubietruck->gc = cubietruck->soc->gc;

	cubietruck->validGPIO = &cubietruckValidGPIO;
}
