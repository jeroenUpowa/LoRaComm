#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define VERSION    "1.0"
#define DISTR_DATE "2017-09-18"


#define DB_MODULE "Communication Module"
#include "debug.h"

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#ifdef GSM
#include <SoftwareSerial.h>
#ifndef __AVR_ATmega32U4__
#error Wrong board !
#endif
#elif defined LORA
#ifndef __SAMD21G18A__
#error Wrong board !
#endif
#else
#error Define either GSM or LORA on the project settings !
#endif

#include "communication.h"

#include "task_scheduler.h"
#include "storage_manager.h"

#include "sampling_task.h"
#include "reporting_task.h"

void setup()
{
	db_start();
	db_wait();

	db("comm setup - start");
	while (comm_setup() != COMM_OK) {
		db("comm setup - failed");
		db("comm setup - retry");
	}

	db("storage setup");
	stor_setup();

	db("sampling setup");
	sampling_setup();

	db("reporting setup");
	reporting_setup();

	db("testing");
	uint8_t code = do_tests();

	while (code != COMMENCE_REPORTING)
	{
		db("retest");
		delay(10000);
		code = do_tests();
	}
	db("tests finished");

	// Launch tasks
	db("scheduler setup");
	delay(100);
	sched_setup();

#ifdef GSM
	db("scheduler add task");
	sched_add_task(sampling_task, SAMPLING_BATT_LOOPTIME, SAMPLING_BATT_LOOPTIME);
	db("scheduler add task");
	sched_add_task(reporting_task, REPORTING_LOOPTIME, REPORTING_LOOPTIME);
#endif
#ifdef LORA
	db("scheduler add task");
	sched_add_task(lora_batt_sampling, SAMPLING_BATT_LOOPTIME, SAMPLING_BATT_LOOPTIME);
	db("scheduler add task");
	sched_add_task(lora_payg_sampling, SAMPLING_PAYG_LOOPTIME, SAMPLING_PAYG_LOOPTIME);
#endif

	db("scheduler mainloop");
	delay(100);
	sched_mainloop();
}

void loop()
{

}

uint8_t do_tests(void)
{
	uint8_t buffer[50];
	uint8_t code = sampling_test(buffer);
#ifdef _DEBUG
	Serial.print("sampling test result: ");
	for (int i = 0; i < 33; i++) {
		Serial.print(" ");
		Serial.print(buffer[i], HEX);
	}
	Serial.println("");
	Serial.println("Storage test");
#endif // DEBUG

	code = stor_test();
	buffer[33] = code;
#ifdef _DEBUG
	if (code == 0)
		db("stor test success");
	else
		db("stor test failed");
#endif // _DEBUG

	code = reporting_test(buffer, 34);
	return code;
}