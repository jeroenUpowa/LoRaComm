#include "communication.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define DB_MODULE "LoRa Comm"
#include "debug.h"

/*
Communication library for the Feather FONA GSM board and the Feather LORA
*/
static osjob_t blinkjob;
void blinkfunc(osjob_t* job);

// Configuration

// application router ID -> Gateway EUI (little-endian format)
static const u1_t PROGMEM APPEUI[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// Device EUI (little endian format)
static const u1_t PROGMEM DEVEUI[8] = { 0x25, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
devaddr_t DevAddr = 0x74345678;

// Device-specific AES key (big endian format) 
static const u1_t PROGMEM APPKEY[16] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45, 0x67, 0x89, 0x01 };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

//Set conection & TX timeout
ostime_t JOIN_TIME = sec2osticks(500); // wait x secondes before timeout
ostime_t TX_TIME = sec2osticks(120);

// Pin mapping
const lmic_pinmap lmic_pins = {
	.nss = 8,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = 4,
	.dio = { 3,SDA,6 },
};

// interval with which the LED blinks in seconds
// used to give information about the LoRa state
// 5 s    - default
// 500 ms - LoRa module trying to join network
// 1 s    - LoRa module successfully joined network
// 100 ms - LoRa module not joined network after retrying.
uint8_t BLINK_INTERVAL = 1;

// LMIC Event Callback

// false - default
// true - Module is successfully connected to the gateway
boolean isJoined = false;

// false - default
// true - Data successfully sent
boolean isSent = false;

// Buffers
#define SAMPLE_SIZE 19
uint8_t samp_buffer[SAMPLE_SIZE];
bool	samp_to_send = false;
#define TEST_SIZE 15
uint8_t test_buffer[TEST_SIZE];
bool	test_to_send = false;
#define PAYG_SIZE 13
uint8_t payg_buffer[PAYG_SIZE];
bool	payg_to_send = false;


void onEvent(ev_t ev) {
	db_print(os_getTime());
	db_print(": ");
	switch (ev) {
	case EV_JOINING:
		db_println("EV_JOINING");
		BLINK_INTERVAL = 500;
		break;
	case EV_JOINED:
		db_println("EV_JOINED");
		BLINK_INTERVAL = 1;
		isJoined = true;
		break;
	case EV_RFU1:
		db_println("EV_RFU1 - unhandled event");
		break;
	case EV_JOIN_FAILED:
		db_println("EV_JOIN_FAILED");
		BLINK_INTERVAL = 100;
		isJoined = false;
		break;
	case EV_TXCOMPLETE:
		db_println("EV_TXCOMPLETE (includes waiting for RX windows)");
		if (LMIC.txrxFlags & TXRX_ACK) {
			db_println("Received ack");
		}
		if (LMIC.dataLen) {
			db_print("Received ");
			db_print(LMIC.dataLen);
			db_print(" bytes of payload, dataBegin: ");
			db_println(LMIC.dataBeg);
		}
		isSent = true;
		break;
	default:
		db_println("Unknown event");
		break;
	}
}

// Jobs

void blinkfunc(osjob_t* job)
{
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

	if (BLINK_INTERVAL >= 10)
		os_setTimedCallback(job, os_getTime() + ms2osticks(BLINK_INTERVAL), blinkfunc);
	else
		os_setTimedCallback(job, os_getTime() + sec2osticks(BLINK_INTERVAL), blinkfunc);
}


enum comm_status_code comm_setup(void)
{
	db("comm setup");
	isJoined = false;
	// Configure pins
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(lmic_pins.rst, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	
	//Hard-resetting the radio
//	digitalWrite(lmic_pins.rst, LOW);
//	delay(2000);
//	digitalWrite(lmic_pins.rst, HIGH);

	// Starting OS
	os_init();

//	db("Connecting to gateway...");
	// Blink job
	os_setCallback(&blinkjob, blinkfunc);

	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
	LMIC_setAdrMode(false);
	LMIC_setDrTxpow(DR_SF12, 23);

	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF12), BAND_CENTI);
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);

	LMIC_startJoining();

	ostime_t start = os_getTime();
	while ((os_getTime() - start < JOIN_TIME) && (!isJoined))
	{
		os_runloop_once();
	}

	LMIC_setAdrMode(false);
	LMIC_setDrTxpow(DR_SF12, 23);
	
	if (!isJoined) {
		db("Connection timeout");
		BLINK_INTERVAL = 100;
		LMIC_shutdown();
		return COMM_ERR_RETRY_LATER;
	} else {
		return COMM_OK;
	}
}


enum comm_status_code comm_start_report(uint16_t totallen, uint8_t type)
{
	if (!isJoined) {
		db("start report not joined");
		return COMM_ERR_RETRY;
	}

	if (type == 1) {
		// Start report for test
		samp_to_send = true;
		test_to_send = true;
		payg_to_send = false;
	} 
	else if (type == 2) {
		// Start report for sample
		samp_to_send = true;
		test_to_send = false;
		payg_to_send = false;
	}
	else if (type == 3) {
		// Start report for payg 
		samp_to_send = false;
		test_to_send = false;
		payg_to_send = true;
	}
	else {
		samp_to_send = false;
		test_to_send = false;
		payg_to_send = false;
	}

	return COMM_OK;
}

/*
Send binary data for the report contents
Always returns COMM_OK, no overflow check is performed
*/
enum comm_status_code comm_fill_report(const uint8_t *buffer, int length)
{
	int index = 0;
	if (samp_to_send) {
		for (int i = 0; i < SAMPLE_SIZE; i++)
			samp_buffer[i] = buffer[index + i];
		index += SAMPLE_SIZE;
	}
	if (test_to_send) {
		for (int i = 0; i < TEST_SIZE; i++)
			test_buffer[i] = buffer[index + i];
		index += SAMPLE_SIZE;
	}
	if (payg_to_send) {
		for (int i = 0; i < PAYG_SIZE; i++)
			payg_buffer[i] = buffer[index + i];
		index += SAMPLE_SIZE;
	}

	return COMM_OK;
}

enum comm_status_code lora_send_packet(uint8_t *buffer, int length)
{
	isSent = false;
	// Prepare upstream data transmission at the next possible time. 
	LMIC_setTxData2(1, buffer, length, 0);
	db("Packet queued");

	ostime_t start = os_getTime();
	while ((!isSent) & (os_getTime() - start < TX_TIME))
	{
		os_runloop_once();
	}

	if (!isSent)
	{
		db("TX timeout");
		return COMM_ERR_RETRY_LATER;
	}
	return COMM_OK;
}

enum comm_status_code comm_send_report(uint8_t *buffer)
{
	for (int i = 0; i < 50; i++)
		buffer[i] = '0';

	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
	{
		db("OP_TXRXPEND, not sending");
		return COMM_ERR_RETRY_LATER;
	}
		
	// Prepare upstream data transmission at the next possible time. 
	comm_status_code code;
	int rep_index = 0;
	if (samp_to_send) {
		db("Sending sample");
		code = lora_send_packet(samp_buffer, SAMPLE_SIZE);
		if (code != COMM_OK)
			return code;
		if (LMIC.dataLen) {
			for (int i = 0; i < LMIC.dataLen; i++)
				buffer[i + rep_index] = LMIC.frame[i + LMIC.dataBeg];
			rep_index += LMIC.dataLen;
		}
	}
	if (test_to_send) {
		db("Sending test result");
		code = lora_send_packet(test_buffer, TEST_SIZE);
		if (code != COMM_OK)
			return code;
		if(LMIC.dataLen) {
			for (int i = 0; i < LMIC.dataLen; i++)
				buffer[i+rep_index] = LMIC.frame[i + LMIC.dataBeg];
			rep_index += LMIC.dataLen;
		}
	}
	if (payg_to_send) {
		db("Sending payg state");
		code = lora_send_packet(payg_buffer, PAYG_SIZE);
		if (code != COMM_OK)
			return code;
		if (LMIC.dataLen) {
			for (int i = 0; i < LMIC.dataLen; i++)
				buffer[i + rep_index] = LMIC.frame[i+LMIC.dataBeg];
			rep_index += LMIC.dataLen;
		}
	}
	return COMM_OK;
}

enum comm_status_code comm_abort(void)
{
	db("Abort");
	os_radio(RADIO_RST); // put radio to sleep
	delay(2000);
	digitalWrite(LED_BUILTIN, LOW);
	return COMM_OK;
}
