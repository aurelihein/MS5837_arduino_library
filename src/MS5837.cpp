#include "MS5837.h"
#include <Wire.h>

const uint8_t MS5837_ADDR = 0x76;
const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;
const uint8_t MS5837::MS5837_UNRECOGNISED = 255;

const uint8_t MS5837_02BA01 = 0x00; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_02BA21 = 0x15; // Sensor version: From MS5837_02BA datasheet Version PROM Word 0
const uint8_t MS5837_30BA26 = 0x1A; // Sensor version: From MS5837_30BA datasheet Version PROM Word 0

MS5837::MS5837() {
}

bool MS5837::begin(TwoWire &wirePort) {
	return (init(wirePort));
}

void MS5837::debug(Stream& _debug_port, bool very_verbose) {
	debug_port = &_debug_port;
}

#if DEBUG_VERY_VERBOSE
void MS5837::debugEepromCoeff(void){
	debug_port->print("eepromCoeff[0]:0x");debug_port->println(eepromCoeff[0], HEX);
	for ( uint8_t i = 1 ; i < 7 ; i++ ) {
		debug_port->print("eepromCoeff[");
		debug_port->print(i);
		debug_port->print("]:");
		debug_port->println(eepromCoeff[i]);
	}
}
#endif /* DEBUG_VERY_VERBOSE */

#if DEBUG_TEST_CALCUL
void MS5837::EepromCoeffTest(void){
	eepromCoeff[1] = 46372;
	eepromCoeff[2] = 43981;
	eepromCoeff[3] = 29059;
	eepromCoeff[4] = 27842;
	eepromCoeff[5] = 31553;
	eepromCoeff[6] = 28165;
}
#endif /* DEBUG_TEST_CALCUL */

bool MS5837::init(TwoWire &wirePort) {
	_i2cPort = &wirePort; //Grab which port the user wants us to use

	// Reset the MS5837, per datasheet
	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_RESET);
	_i2cPort->endTransmission();

	// Wait for reset to complete
	delay(10);

	// Read calibration values and CRC
	for ( uint8_t i = 0 ; i < 7 ; i++ ) {
		_i2cPort->beginTransmission(MS5837_ADDR);
		_i2cPort->write(MS5837_PROM_READ+i*2);
		_i2cPort->endTransmission();

		_i2cPort->requestFrom(MS5837_ADDR,(size_t)2);
		uint8_t value_1 = _i2cPort->read();
		uint8_t value_2 = _i2cPort->read();
		eepromCoeff[i] = (value_1 << 8) | value_2;
	}
#if DEBUG_TEST_CALCUL
	EepromCoeffTest();
#endif /* DEBUG_TEST_CALCUL */
#if DEBUG_VERY_VERBOSE
	debugEepromCoeff();
#endif /* DEBUG_VERY_VERBOSE */

	//eepromCoeff[0] :
	// 15-12 is CRC
	// 11-5 is MS5837 type :
	//  -> 0x1A (dec:26) : MS5837-30BA
	//  -> 0x00 (dec:0) : MS5837-02BA01
	//  -> 0x15 (dec:21) : MS5837-02BA21
	// 4-0 factory settings
#if 1
	// Verify that data is correct with CRC
	uint8_t crcRead = eepromCoeff[0] >> 12;
	uint8_t crcCalculated = crc4(eepromCoeff);
	if ( crcCalculated != crcRead ) {
		debug_port->println("MS5837[ERR]:Bad CRC");
		return false; // CRC fail
	}
#endif 
#if HANDLE_ALL_MS5837
	uint8_t version = (eepromCoeff[0] >> 5) & 0x7F; // Extract the sensor version from PROM Word 0

	debug_port->print("MS5837[INF]:version:");
	debug_port->println(version);

	// Set _model according to the sensor version
	if (version == MS5837_02BA01)
	{
		_model = MS5837_02BA;
		debug_port->println("MS5837_02BA01");
	}
	else if (version == MS5837_02BA21)
	{
		_model = MS5837_02BA;
		debug_port->println("MS5837_02BA21");
	}
	else if (version == MS5837_30BA26)
	{
		_model = MS5837_30BA;
		debug_port->println("MS5837_30BA");
	}
	else
	{
		_model = MS5837_UNRECOGNISED;
		debug_port->println("MS5837[ERR]:Unrecognized");
		return false;
	}
	debug_port->print("MS5837[INF]:model:");
	debug_port->println(_model);
	// The sensor has passed the CRC check, so we should return true even if
	// the sensor version is unrecognised.
	// (The MS5637 has the same address as the MS5837 and will also pass the CRC check)
	// (but will hopefully be unrecognised.)
#endif /* HANDLE_ALL_MS5837 */
	return true;
}

#if HANDLE_ALL_MS5837
void MS5837::setModel(uint8_t model) {
#if DEBUG_VERY_VERBOSE
	debug_port->print("MS5837.setModel:");debug_port->println(_model);
#endif /* DEBUG_VERY_VERBOSE */
	_model = model;
}

uint8_t MS5837::getModel() {
#if DEBUG_VERY_VERBOSE
	debug_port->print("MS5837.getModel:");debug_port->print("eepromCoeff[0]:0x");debug_port->println(eepromCoeff[0], HEX);
#endif /* DEBUG_VERY_VERBOSE */
	return (_model);
}
#endif /* HANDLE_ALL_MS5837 */

int32_t MS5837::read() {
	//Check that _i2cPort is not NULL (i.e. has the user forgoten to call .init or .begin?)
	if (_i2cPort == NULL)
	{
		return 0;
	}

	// Request D1 conversion
	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_CONVERT_D1_8192);
	_i2cPort->endTransmission();

	delay(20); // Max conversion time per datasheet

	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_ADC_READ);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(MS5837_ADDR,(size_t)3);
	D1_pres = 0;
	D1_pres = _i2cPort->read();
	D1_pres = (D1_pres << 8) | _i2cPort->read();
	D1_pres = (D1_pres << 8) | _i2cPort->read();
	// Request D2 conversion
	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_CONVERT_D2_8192);
	_i2cPort->endTransmission();

	delay(20); // Max conversion time per datasheet

	_i2cPort->beginTransmission(MS5837_ADDR);
	_i2cPort->write(MS5837_ADC_READ);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(MS5837_ADDR,(size_t)3);
	D2_temp = 0;
	D2_temp = _i2cPort->read();
	D2_temp = (D2_temp << 8) | _i2cPort->read();
	D2_temp = (D2_temp << 8) | _i2cPort->read();

#if DEBUG_VERY_VERBOSE
	debug_port->print("MS5837[INF]:D1_pres:");debug_port->println(D1_pres);
	debug_port->print("MS5837[INF]:D2_temp:");debug_port->println(D2_temp);
#endif /* DEBUG_VERY_VERBOSE */
	calculate();
	return P;
}

void MS5837::calculate(void) {
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	float SENSi = 0;
	float OFFi = 0;
	float Ti = 0;

#if DEBUG_TEST_CALCUL
	D1_pres = 6465444;
	D2_temp = 8077636;
#endif /* DEBUG_TEST_CALCUL */

#if DEBUG_VERY_VERBOSE
	if( _model == MS5837_02BA ){
		debug_port->println("Calc for MS5837_02BA");
	}else{
		debug_port->println("Calc for MS5837_30BA");
	}
#endif /* DEBUG_VERY_VERBOSE */

	dT = ((int32_t)D2_temp) - (((int32_t)eepromCoeff[5]) << 8);
	TEMP = 2000 + ( ((int64_t)dT) * ((int64_t)eepromCoeff[6]) >> 23);
#if DEBUG_TEST_CALCUL
	debug_port->print("dT:");debug_port->println(dT);
	debug_port->print("TEMP:");debug_port->println(TEMP);
#endif /* DEBUG_TEST_CALCUL */
#if HANDLE_ALL_MS5837
	if ( _model == MS5837_02BA ) {
#endif /* HANDLE_ALL_MS5837 */
#if ALLOW_MS5837_02BA
		OFF = (((int64_t)eepromCoeff[2]) << 17) + (( ((int64_t)eepromCoeff[4]) * ((int64_t)dT) ) >> 6);
		SENS = (((int64_t)eepromCoeff[1]) << 16) + (( ((int64_t)eepromCoeff[3]) * ((int64_t)dT) ) >> 7);
#endif /* ALLOW_MS5837_02BA */
#if HANDLE_ALL_MS5837
	} else {
#endif /* HANDLE_ALL_MS5837 */
#if ALLOW_MS5837_30BA
		OFF = (((int64_t)eepromCoeff[2]) << 16) + (( ((int64_t)eepromCoeff[4]) * ((int64_t)dT) ) >> 7);
		SENS = (((int64_t)eepromCoeff[1]) << 15) + (( ((int64_t)eepromCoeff[3]) * ((int64_t)dT) ) >> 8);
#endif /* ALLOW_MS5837_30BA */
#if HANDLE_ALL_MS5837
	}
#endif /* HANDLE_ALL_MS5837 */
#if DEBUG_TEST_CALCUL
	debug_port->print("OFF:");debug_port->print(OFF);debug_port->print("[");debug_port->println(5764707214);
	debug_port->print("SENS:");debug_port->print(SENS);debug_port->print("[");debug_port->println(3039050829);
#endif /* DEBUG_TEST_CALCUL */

	//Second order compensation
#if HANDLE_ALL_MS5837
	if ( _model == MS5837_02BA ) {
#endif /* HANDLE_ALL_MS5837 */
#if ALLOW_MS5837_02BA
		if(TEMP<2000){         //Low temp
			Ti = ( 11 * ((int64_t)dT) * ((int64_t)dT)) >> 35;
			OFFi = ( 31 * (((int64_t)TEMP) - 2000) * (((int64_t)TEMP) - 2000) ) >> 3;
			SENSi = ( 63 * (((int64_t)TEMP) - 2000) * (((int64_t)TEMP) - 2000) ) >> 5;
			P = (int32_t)(( (int64_t)( ( ((int64_t)D1_pres) * ((int64_t)( ((int64_t)SENS) - ((int64_t)SENSi) )) ) >> 21 )) - ( ((int64_t)OFF) - ((int64_t)OFFi) ) >> 15) ;
		}else{
			P = (int32_t)(( (( ((int64_t)D1_pres) * SENS) >> 21) - OFF) >> 15);
		}
		P /= 10;
#endif /* ALLOW_MS5837_02BA */
#if HANDLE_ALL_MS5837
	} else {
#endif /* HANDLE_ALL_MS5837 */
#if ALLOW_MS5837_30BA
		if(TEMP<2000){         //Low temp
			Ti = ( 3 * ( ((int64_t)dT) * ((int64_t)dT) ) ) >> 33;
			OFFi = ( 3 * ( ((int64_t)TEMP) - 2000) * ( ((int64_t)TEMP) - 2000) ) >> 1;
			SENSi = ( 5 * ( ((int64_t)TEMP) - 2000 ) * ( ((int64_t)TEMP) - 2000 ) ) >> 3;
			if(TEMP<-1500){    //Very low temp
				OFFi += 7 * ( ((int64_t)TEMP) + 1500 ) * ( ((int64_t)TEMP) + 1500 );
				SENSi += 4 * ( ((int64_t)TEMP) + 1500 ) * ( ((int64_t)TEMP) + 1500 );
			}
		} else {    //High temp
			Ti = ( 2 * ( ((int64_t)dT) * ((int64_t)dT) ) ) >> 37;
			OFFi = ( ( ((int64_t)TEMP) - 2000 ) * ( ((int64_t)TEMP) - 2000 ) ) >> 4;
			SENSi = 0;
		}
		P = (int32_t)(( (int64_t)( ( ((int64_t)D1_pres) * ((int64_t)( ((int64_t)SENS) - ((int64_t)SENSi) )) ) >> 21 )) - ( ((int64_t)OFF) - ((int64_t)OFFi) ) >> 13) ;
#endif /* ALLOW_MS5837_30BA */
#if HANDLE_ALL_MS5837
	}
#endif /* HANDLE_ALL_MS5837 */
#if DEBUG_TEST_CALCUL
	debug_port->print("P:");debug_port->println(P);
	debug_port->print("Ti:");debug_port->println(Ti);
	debug_port->print("OFFi:");debug_port->println(OFFi);
	debug_port->print("SENSi:");debug_port->println(SENSi);
#endif /* DEBUG_TEST_CALCUL */
	TEMP = (int32_t)( ((int64_t)TEMP) - ((int64_t)Ti) );
}

float MS5837::pressure(float conversion) {
	return P*conversion/10.0f;
}

float MS5837::temperature() {
	return TEMP/100.0f;
}

// The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
// We subtract the atmospheric pressure to calculate the depth with only the water pressure
// The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
// If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
// In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
// that value should subtracted for subsequent depth calculations.
float MS5837::depth(uint16_t fluidDensity, uint16_t atmospheric_pressure_10mbar) {
	return (float)(( ((float)P) - ((float)atmospheric_pressure_10mbar) )/((float)( ((float)fluidDensity) * 9.80665)));
}

float MS5837::altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
