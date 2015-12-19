/*
	Copyright (c) 2014 CurlyMo <curlymoo1@gmail.com>
				  2015 vitalogy <vitalogy_github@milaw biz>

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#include "wiringX.h"
#ifndef __FreeBSD__
	#include "i2c-dev.h"
#endif
#include "sunxidt.h"


#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
#define MAP_SIZE (4096*2)
#define MAP_MASK (MAP_SIZE - 1)
#define SUNXI_BASE (0x01C20000)
#define SUNXI_GPIO_BASE (0x01C20800)

// the most available GPIOs has the Cubieboard1 & 2, so set MAX_PINS to this
// note: must be changed if a board is added with more available GPIOs
#define MAX_PINS 68

// there are max 32 GPIOs available that can be used as an interrupt
// note: these are not all available on the various boards
#define MAX_EDGE_PINS 32

static volatile uint32_t *gpio;

static int dtBoard = -1;

static int pinModes[MAX_PINS];
static int sysFds[MAX_PINS];
static int *pinToGpio = 0;
static int *onboardLEDs = 0;
static int numGPIO = 0;
static int numLED = 0;

static int sunxiEdge[MAX_EDGE_PINS] = {
	224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234,	// PH0 - PH10 (EINT0 - EINT10)
	235 ,236, 237, 238, 239, 240, 241, 242, 243, 244, 245,	// PH11 - PH21 (EINT11 - EINT21)
	266, 267, 268, 269, 270, 271, 272, 273, 274, 275,		// PI10 - PI19 (EINT22 - EINT31)
};


// Cubietech CubieBoard/CubieBoard2   68 GPIO Pins (GPIO 0 - GPIO 67)
static int pinToGpio_CB[68] = {
	239, 238, 192, 50,		// 0, 1, 2, 3,		PH15, PH14, PG0, PB18
	51, 195, 194, 193,		// 4, 5, 6, 7,		PB19, PG3, PG2, PG1
	196, 197, 198, 199,		// 8, 9, 10, 11,	PG4, PG5, PG6, PG7
	200, 201, 202, 201,		// 12, 13, 14, 15,	PG8, PG9, PG10, PG11
	96, 98, 97, 100,		// 16, 17, 18 ,19,	PD0, PD2, PD1, PD4
	99, 102, 101, 103,		// 20, 21, 22, 23,	PD3, PD6, PD5, PD7
	105, 104, 107, 106,		// 24, 25, 26, 27,	PD9, PD8, PD11, PD10
	109, 108, 111, 110,		// 28, 29, 30, 31,	PD13, PD12, PD15, PD14
	112, 114, 113, 116,		// 32, 33, 34, 35,	PD16, PD18, PD17, PD20
	115, 118, 117, 123,		// 36, 37, 38, 39,	PD19, PD22, PD21, PD27
	119, 120, 122, 34,		// 40, 41, 42, 43,	PD23, PD24, PD26, PB2
	121, 231, 43, 42,		// 44, 45, 46, 47,	PD25, PH7, PB11, PB10
	266, 268, 267, 269,		// 48, 49, 50, 51,	PI10, PI12, PI11, PI13
	260, 261, 262, 263,		// 52, 53, 54, 55,	PI4, PI5, PI6, PI7
	264, 265, 132, 133,		// 56, 57, 58, 59,	PI8, PI9, PE4, PE5
	134, 135, 136, 137,		// 60, 61, 62, 63,	PE6, PE7, PE8, PE9
	138, 139,				// 64, 65,			PE10, PE11
	244,					// 66,				PH20 (green LED)
	245,					// 67,				PH21 (blue LED)
};


static int onboardLEDs_CB[2] = {
	244, 245,
};
// End Cubietech CubieBoard


// Cubietech CubieTruck   34 GPIO Pins (GPIO 0 - GPIO 33)
static int pinToGpio_CT[34] = {
	// same as wiringPi_CT
	196, 197, 198, 199,		// 0, 1, 2, 3,		PG4, PG5, PG6, PG7
	200, 201, 202, 203,		// 4, 5, 6, 7,		PG8, PG9, PG10, PG11
	83, 85, 84, 86,			// 8, 9, 10, 11,	PC19, PC21, PC20, PC22
	46, 48, 47, 49,			// 12, 13, 14, 15,	PB14, PB16, PB15, PB17
	276, 270, 277, 271,		// 16, 17, 18, 19,	PI20, PI14, PI21, PI15
	259, 35, 34, 36,		// 20, 21, 22, 23,	PI3, PB3, PB2, PB4
	50, 51,	192, 195,		// 24, 25, 26 ,27,	PB18, PB19, PG0, PG3
	194, 193,				// 28, 29,			PG2, PG1
	231,					// 30,				PH7 (green LED)
	235,					// 31,				PH11 (white LED)
	244,					// 32,				PH20 (orange LED)
	245,					// 33,				PH21 (blue LED)
};

static int onboardLEDs_CT[4] = {
	231, 235, 244, 245,
};
// End Cubietech CubieTruck


// LeMaker Banana Pi   24 GPIO Pins (GPIO 0 - GPIO 23)
static int pinToGpio_BPI[24] = {
	275, 226, 274, 273,		// 0, 1, 2, 3,		PI19, PH2, PI18, PI17
	244, 245, 272, 259,		// 4, 5, 6, 7,		PH20, PH21, PI16, PI3
	53, 52, 266, 270,		// 8, 9, 10, 11,	PB21, PB20, PI10, PI14
	268, 269, 267, 224,		// 12, 13, 14, 15,	PI12, PI13, PI11, PH0
	225, 229, 277, 227,		// 16, 17, 18, 19,	PH1, PH5, PI21, PH3
	276, 54, 55,			// 20, 21, 22		PI20, PB22, PB23
	248,					// 23,				PH24 (green LED)
};

static int onboardLEDs_BPI[1] = {
	248,
};
// End LeMaker Banana Pi


int sunxidtValidGPIO(int pin) {
	if(pin >= 0 && pin < numGPIO) {
		return 0;
	}
	return -1;
}

static uint32_t readl(uint32_t addr) {
	uint32_t val = 0;
	uint32_t mmap_base = (addr & ~MAP_MASK);
	uint32_t mmap_seek = ((addr - mmap_base) >> 2);
	val = *(gpio + mmap_seek);
	return val;
}

static void writel(uint32_t val, uint32_t addr) {
	uint32_t mmap_base = (addr & ~MAP_MASK);
	uint32_t mmap_seek = ((addr - mmap_base) >> 2);
	*(gpio + mmap_seek) = val;
}

static int changeOwner(char *file) {
	uid_t uid = getuid();
	uid_t gid = getgid();

	if(chown(file, uid, gid) != 0) {
		if(errno == ENOENT)	{
			wiringXLog(LOG_ERR, "sunxidt->changeOwner: File not present: %s", file);
			return -1;
		} else {
			wiringXLog(LOG_ERR, "sunxidt->changeOwner: Unable to change ownership of %s: %s", file, strerror (errno));
			return -1;
		}
	}

	return 0;
}

static int sunxiBoard(void) {
	FILE *cpuFd = NULL;
	FILE *modelFd = NULL;
	char line[120], model[40];
	char *d = 0;

	if(dtBoard != -1) {
		return dtBoard;
	}

	memset(line, '\0', 120);
	memset(model, '\0', 40);

	// check for devicetree based kernel
	if((modelFd = fopen("/sys/firmware/devicetree/base/model", "r")) == NULL) {
		wiringXLog(LOG_ERR, "sunxidt->identify: Unable to open /sys/firmware/devicetree/base/model");
		return -1;
	}
	fgets(model, 40, modelFd);
	fclose(modelFd);

	if((cpuFd = fopen("/proc/cpuinfo", "r")) == NULL) {
		wiringXLog(LOG_ERR, "sunxidt->identify: Unable to open /proc/cpuinfo");
		return -1;
	}

	while(fgets(line, 120, cpuFd) != NULL) {
		if(strncmp(line, "Hardware", 8) == 0) {
			break;
		}
	}

	fclose(cpuFd);

	if(strlen(line) == 0) {
		return -1;
	}

	if(strncmp(line, "Hardware", 8) != 0) {
		wiringXLog(LOG_ERR, "sunxidt->identify: /proc/cpuinfo has no hardware line");
		return -1;
	}

	for(d = &line[strlen(line) - 1]; (*d == '\n') || (*d == '\r') ; --d)
		*d = 0 ;

	if(strstr(line, "sun4i") != 0 || strstr(line, "sun7i") != 0) {

		/*
			dtBoard		devicetree Board
				0		Cubietech Cubieboard/Cubieboard2
				1		Cubietech Cubietruck
				2		LeMaker Banana Pi
		*/

		if(strstr(model,"Cubietech Cubieboard") != 0) {
			dtBoard = 0;
		} else if(strcmp(model,"Cubietech Cubietruck") == 0) {
			dtBoard = 1;
		} else if(strcmp(model,"LeMaker Banana Pi") == 0) {
			dtBoard = 2;
		} else {
			wiringXLog(LOG_ERR, "sunxidt->identify: Board %s is unknown to sunixdt", model);
			return -1;
		}
		wiringXLog(LOG_DEBUG, "sunxidt->identify: Detected %s", model);
		return dtBoard;
	} else {
		return -1;
	}
}

static int setup(void)	{
	int board = 0, fd = 0;

	board = sunxiBoard();
	if(board == -1) {
		return -1;
	}

	switch(board) {
		case 0:
			// Cubietech Cubieboard/Cubieboard2
			pinToGpio = pinToGpio_CB;
			onboardLEDs = onboardLEDs_CB;
			numGPIO = 68;
			numLED = 2;
			break;
		case 1:
			// Cubietech Cubietruck
			pinToGpio = pinToGpio_CT;
			onboardLEDs = onboardLEDs_CT;
			numGPIO = 34;
			numLED = 4;
			break;
		case 2:
			// LeMaker Banana Pi
			pinToGpio = pinToGpio_BPI;
			onboardLEDs = onboardLEDs_BPI;
			numGPIO = 24;
			numLED = 1;
			break;
	}

#ifdef O_CLOEXEC
	if((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
#else
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
#endif
		wiringXLog(LOG_ERR, "sunxidt->setup: Unable to open /dev/mem");
		return -1;
	}

	gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, SUNXI_BASE);
	if((int32_t)gpio == -1) {
		wiringXLog(LOG_ERR, "sunxidt->setup: mmap (GPIO) failed");
		return -1;
	}
	return 0;
}

static int sunxidtISR(int pin, int mode) {
	FILE *f = NULL;
	const char *sMode = NULL;
	char path[35], line[120], c = 0;
	int invalid = 1;
	int i = 0, fd = 0, match = 0, count = 0;

	if(sunxidtValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "sunxidt->isr: Invalid GPIO %d", pin);
		return -1;
	}

	for(i=0; i<numLED; i++) {
		if(*(onboardLEDs+i) == *(pinToGpio+pin)) {
			wiringXLog(LOG_ERR, "sunxidt->isr: The onboard LED on GPIO %d (SUNXI %d) cannot be used as interrupt", pin, *(pinToGpio+pin));
			return -1;
		}
	}

	for(i=0; i<MAX_EDGE_PINS; i++) {
		if(sunxiEdge[i] == *(pinToGpio+pin)) {
			invalid = 0;
		}
	}

	if(invalid == 1) {
		wiringXLog(LOG_ERR, "sunxidt->isr: The GPIO %d (SUNXI %d) cannot be used as interrupt", pin, *(pinToGpio+pin));
		return -1;
	}

	pinModes[pin] = SYS;

	if(mode == INT_EDGE_FALLING) {
		sMode = "falling" ;
	} else if(mode == INT_EDGE_RISING) {
		sMode = "rising" ;
	} else if(mode == INT_EDGE_BOTH) {
		sMode = "both";
	} else if(mode == INT_EDGE_NONE) {
		sMode = "none";
	} else {
		wiringXLog(LOG_ERR, "sunxidt->isr: Invalid mode. Should be INT_EDGE_BOTH, INT_EDGE_RISING, or INT_EDGE_FALLING");
		return -1;
	}

	sprintf(path, "/sys/class/gpio/gpio%d/value", *(pinToGpio+pin));
	if((fd = open(path, O_RDWR)) < 0) {
		if((f = fopen("/sys/class/gpio/export", "w")) == NULL) {
			wiringXLog(LOG_ERR, "sunxidt->isr: Unable to open GPIO export interface");
			return -1;
		}

		fprintf(f, "%d\n", *(pinToGpio+pin));
		fclose(f);
	}

	sprintf(path, "/sys/class/gpio/gpio%d/direction", *(pinToGpio+pin));
	if((f = fopen(path, "w")) == NULL) {
		wiringXLog(LOG_ERR, "sunxidt->isr: Unable to open GPIO direction interface for GPIO %d (SUNXI %d): %s", pin, *(pinToGpio+pin), strerror(errno));
		return -1;
	}

	fprintf(f, "in\n");
	fclose(f);

	sprintf(path, "/sys/class/gpio/gpio%d/edge", *(pinToGpio+pin));
	if((f = fopen(path, "w")) == NULL) {
		wiringXLog(LOG_ERR, "sunxidt->isr: Unable to open GPIO edge interface for GPIO %d (SUNXI %d): %s", pin, *(pinToGpio+pin), strerror(errno));
		return -1;
	}

	if(strcasecmp(sMode, "none") == 0) {
		fprintf(f, "none\n");
	} else if(strcasecmp(sMode, "rising") == 0) {
		fprintf(f, "rising\n");
	} else if(strcasecmp(sMode, "falling") == 0) {
		fprintf(f, "falling\n");
	} else if(strcasecmp (sMode, "both") == 0) {
		fprintf(f, "both\n");
	} else {
		wiringXLog(LOG_ERR, "sunxidt->isr: Invalid mode: %s. Should be rising, falling or both", sMode);
		return -1;
	}
	fclose(f);

	if((f = fopen(path, "r")) == NULL) {
		wiringXLog(LOG_ERR, "sunxidt->isr: Unable to open GPIO edge interface for GPIO %d (SUNXI %d): %s", pin, *(pinToGpio+pin), strerror(errno));
		return -1;
	}

	while(fgets(line, 120, f) != NULL) {
		if(strstr(line, sMode) != NULL) {
			match = 1;
			break;
		}
	}
	fclose(f);

	if(match == 0) {
		wiringXLog(LOG_ERR, "sunxidt->isr: Failed to set interrupt edge to %s on GPIO %d (SUNXI %d)", sMode, pin, *(pinToGpio+pin));
		return -1;
	}

	if (sysFds[pin] == -1) {
		sprintf(path, "/sys/class/gpio/gpio%d/value", *(pinToGpio+pin));
		if((sysFds[pin] = open(path, O_RDONLY)) < 0) {
			wiringXLog(LOG_ERR, "sunxidt->isr: Unable to open GPIO value interface for GPIO %d (SUNXI %d): %s", pin, *(pinToGpio+pin), strerror(errno));
			return -1;
		}
		changeOwner(path);
	}

	sprintf(path, "/sys/class/gpio/gpio%d/edge", *(pinToGpio+pin));
	changeOwner(path);

	ioctl(fd, FIONREAD, &count);
	for(i=0; i<count; ++i) {
		read(fd, &c, 1);
	}
	close(fd);

	return 0;
}

static int sunxidtWaitForInterrupt(int pin, int ms) {
	uint8_t c = 0;
	struct pollfd polls;
	int i = 0, x = 0;

	if(sunxidtValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "sunxidt->waitForInterrupt: Invalid GPIO %d", pin);
		return -1;
	}

	for(i=0; i<numLED; i++) {
		if(*(onboardLEDs+i) == *(pinToGpio+pin)) {
			wiringXLog(LOG_ERR, "sunxidt->waitForInterrupt: The onboard LED on GPIO %d (SUNXI %d) cannot be used as interrupt", pin, *(pinToGpio+pin));
			return -1;
		}
	}

	if(pinModes[pin] != SYS) {
		wiringXLog(LOG_ERR, "sunxidt->waitForInterrupt: Trying to read from GPIO %d (SUNXI %d), but it's not configured as interrupt", pin, *(pinToGpio+pin));
		return -1;
	}

	if(sysFds[pin] == -1) {
		wiringXLog(LOG_ERR, "sunxidt->waitForInterrupt: The GPIO %d (SUNXI %d) is not set as interrupt", pin, *(pinToGpio+pin));
		return -1;
	}

	polls.fd = sysFds[pin];
	polls.events = POLLPRI;

	(void)read(sysFds[pin], &c, 1);
	lseek(sysFds[pin], 0, SEEK_SET);

	x = poll(&polls, 1, ms);

	/* Don't react to signals */
	if(x == -1 && errno == EINTR) {
		x = 0;
	}

	return x;
}

static int sunxidtDigitalRead(int pin) {
	uint32_t regval = 0, phyaddr = 0;
	int bank = 0, i = 0;

	if(sunxidtValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "sunxidt->digitalRead: Invalid GPIO %d", pin);
		return -1;
	}

	for(i=0; i<numLED; i++) {
		if(*(onboardLEDs+i) == *(pinToGpio+pin)) {
			wiringXLog(LOG_ERR, "sunxidt->digitalRead: The onboard LED on GPIO %d (SUNXI %d) cannot be used as input", pin, *(pinToGpio+pin));
			return -1;
		}
	}

	regval = 0;
	bank = *(pinToGpio+pin) >> 5;
	i = *(pinToGpio+pin) - (bank << 5);
	phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg

	if(pinModes[pin] != INPUT && pinModes[pin] != SYS) {
		wiringXLog(LOG_ERR, "sunxidt->digitalRead: Trying to read from GPIO %d (SUNXI %d), but it's not configured as input", pin, *(pinToGpio+pin));
		return -1;
	}

	regval = readl(phyaddr);
	regval = regval >> i;
	regval &= 1;
	return regval;

	return 0;
}

static int sunxidtDigitalWrite(int pin, int value) {
	uint32_t regval = 0, phyaddr = 0;
	int bank = 0, i = 0;

	if(sunxidtValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "sunxidt->digitalWrite: Invalid GPIO %d", pin);
		return -1;
	}

	regval = 0;
	bank = *(pinToGpio+pin) >> 5;
	i = *(pinToGpio+pin) - (bank << 5);
	phyaddr = SUNXI_GPIO_BASE + (bank * 36) + 0x10; // +0x10 -> data reg

	if(pinModes[pin] != OUTPUT) {
		wiringXLog(LOG_ERR, "sunxidt->digitalWrite: Trying to write to GPIO %d (SUNXI %d), but it's not configured as output", pin, *(pinToGpio+pin));
		return -1;
	}

	regval = readl(phyaddr);

	if(value == LOW) {
		regval &= ~(1 << i);
		writel(regval, phyaddr);
		regval = readl(phyaddr);
	} else {
		regval |= (1 << i);
		writel(regval, phyaddr);
		regval = readl(phyaddr);
	}
	return 0;
}

static int sunxidtPinMode(int pin, int mode) {
	uint32_t regval = 0, phyaddr = 0;
	int bank = 0, i = 0, offset = 0;

	if(sunxidtValidGPIO(pin) != 0) {
		wiringXLog(LOG_ERR, "sunxidt->pinMode: Invalid GPIO %d", pin);
		return -1;
	}

	for(i=0; i<numLED; i++) {
		if(*(onboardLEDs+i) == *(pinToGpio+pin) && mode == INPUT) {
			wiringXLog(LOG_ERR, "sunxidt->pinMode: The onboard LED on GPIO %d (SUNXI %d) cannot be used as input", pin, *(pinToGpio+pin));
			return -1;
		}
	}

	regval = 0;
	bank = *(pinToGpio+pin) >> 5;
	i = *(pinToGpio+pin) - (bank << 5);
	offset = ((i - ((i >> 3) << 3)) << 2);
	phyaddr = SUNXI_GPIO_BASE + (bank * 36) + ((i >> 3) << 2);

	pinModes[pin] = mode;

	regval = readl(phyaddr);

	if(mode == INPUT) {
		regval &= ~(7 << offset);
		writel(regval, phyaddr);
		regval = readl(phyaddr);
	} else if(mode == OUTPUT) {
	   regval &= ~(7 << offset);
	   regval |= (1 << offset);
	   writel(regval, phyaddr);
	   regval = readl(phyaddr);
	} else {
		return -1;
	}
	return 0;
}

static int sunxidtGC(void) {
	FILE *f = NULL;
	char path[35];
	int i = 0, fd = 0;

	for(i=0; i<numGPIO; i++) {
		if(pinModes[i] == OUTPUT) {
			pinMode(i, INPUT);
		} else if(pinModes[i] == SYS) {
			sprintf(path, "/sys/class/gpio/gpio%d/value", *(pinToGpio+i));
			if((fd = open(path, O_RDWR)) > 0) {
				if((f = fopen("/sys/class/gpio/unexport", "w")) == NULL) {
					wiringXLog(LOG_ERR, "sunxidt->gc: Unable to open GPIO unexport interface: %s", strerror(errno));
				}

				fprintf(f, "%d\n", *(pinToGpio+i));
				fclose(f);
				close(fd);
			}
		}
		if(sysFds[i] > 0) {
			close(sysFds[i]);
			sysFds[i] = -1;
		}
	}

	if(gpio) {
		munmap((void *)gpio, BLOCK_SIZE);
	}
	return 0;
}

#ifndef __FreeBSD__
static int sunxidtI2CRead(int fd) {
	return i2c_smbus_read_byte(fd);
}

static int sunxidtI2CReadReg8(int fd, int reg) {
	return i2c_smbus_read_byte_data(fd, reg);
}

static int sunxidtI2CReadReg16(int fd, int reg) {
	return i2c_smbus_read_word_data(fd, reg);
}

static int sunxidtI2CWrite(int fd, int data) {
	return i2c_smbus_write_byte(fd, data);
}

static int sunxidtI2CWriteReg8(int fd, int reg, int data) {
	return i2c_smbus_write_byte_data(fd, reg, data);
}

static int sunxidtI2CWriteReg16(int fd, int reg, int data) {
	return i2c_smbus_write_word_data(fd, reg, data);
}

static int sunxidtI2CSetup(int devId) {
	wiringXLog(LOG_ERR, "sunxidt->I2CSetup: Not yet implemented");
	return -1;
}
#endif

void sunxidtInit(void) {

	memset(pinModes, -1, MAX_PINS);
	memset(sysFds, -1, MAX_PINS);

	platform_register(&sunxidt, "sunxidt");
	sunxidt->setup=&setup;
	sunxidt->pinMode=&sunxidtPinMode;
	sunxidt->digitalWrite=&sunxidtDigitalWrite;
	sunxidt->digitalRead=&sunxidtDigitalRead;
	sunxidt->identify=&sunxiBoard;
	sunxidt->isr=&sunxidtISR;
	sunxidt->waitForInterrupt=&sunxidtWaitForInterrupt;
#ifndef __FreeBSD__
	sunxidt->I2CRead=&sunxidtI2CRead;
	sunxidt->I2CReadReg8=&sunxidtI2CReadReg8;
	sunxidt->I2CReadReg16=&sunxidtI2CReadReg16;
	sunxidt->I2CWrite=&sunxidtI2CWrite;
	sunxidt->I2CWriteReg8=&sunxidtI2CWriteReg8;
	sunxidt->I2CWriteReg16=&sunxidtI2CWriteReg16;
	sunxidt->I2CSetup=&sunxidtI2CSetup;
#endif
	sunxidt->gc=&sunxidtGC;
	sunxidt->validGPIO=&sunxidtValidGPIO;
}
